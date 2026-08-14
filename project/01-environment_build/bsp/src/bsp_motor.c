/**
 * @file    bsp_motor.c
 * @brief   板级电机 BSP 层实现（TB6612 驱动）v1.2
 * @note    - 使用 TIM1_CH1/CH2 作为 PWM 输出
 *          - 两个电机共用 TIM1，共享句柄带引用计数
 *          - 完整支持 DISABLED/IDLE/RUNNING/BRAKING 四态
 * @author  xserein
 * @version v1.2
 */

#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_motor.h"
#include "dal_motor.h"
#include "bsp_timer.h"
#include <string.h>

/* ========================================================================== */
/*                             硬件配置                                        */
/* ========================================================================== */

#define MOTOR_PWM_FREQ_HZ        (20000U)
#define MOTOR_DEFAULT_TIMEOUT_MS (100U)
#define MOTOR_COUNT              (2U)

/* ========================================================================== */
/*                    私有上下文与硬件描述符                                    */
/* ========================================================================== */

typedef struct {
    const char   *name;
    uint8_t       timer_channel;
    GPIO_TypeDef *in1_port;
    uint32_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint32_t      in2_pin;
} motor_hw_desc_t;

typedef struct {
    bsp_timer_handle_t timer_handle;
    uint8_t            timer_channel;
    GPIO_TypeDef      *in1_port;
    uint32_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint32_t           in2_pin;
    dal_motor_dir_t    direction;
    uint8_t            duty;
    bool               enabled;
    bool               braking;         /**< [v1.2] 刹车状态标志 */
    bool               is_initialized;
} bsp_motor_priv_t;

/* ========================================================================== */
/*                        静态数据池                                           */
/* ========================================================================== */

static const motor_hw_desc_t s_motor_table[MOTOR_COUNT] = {
    {
        .name          = "motor_a",
        .timer_channel = 1,
        .in1_port      = BSP_TB6612_MOTORA_IN1_PORT,
        .in1_pin       = BSP_TB6612_MOTORA_IN1_PIN,
        .in2_port      = BSP_TB6612_MOTORA_IN2_PORT,
        .in2_pin       = BSP_TB6612_MOTORA_IN2_PIN,
    },
    {
        .name          = "motor_b",
        .timer_channel = 2,
        .in1_port      = BSP_TB6612_MOTORB_IN1_PORT,
        .in1_pin       = BSP_TB6612_MOTORB_IN1_PIN,
        .in2_port      = BSP_TB6612_MOTORB_IN2_PORT,
        .in2_pin       = BSP_TB6612_MOTORB_IN2_PIN,
    },
};

static bsp_motor_priv_t s_priv_pool[MOTOR_COUNT];
static dal_motor_dev_t  s_dev_pool[MOTOR_COUNT];

static bsp_timer_handle_t s_shared_timer_handle = NULL;
static uint8_t            s_timer_ref_count     = 0;
static bool               s_stby_global         = false;

/* ========================================================================== */
/*                       内部辅助函数                                          */
/* ========================================================================== */

/**
 * @brief [v1.2] 使能 GPIO 端口时钟，覆盖 STM32F1 全部可用端口
 */
static void _enable_gpio_clk(GPIO_TypeDef *port)
{
    if (port == GPIOA)      __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
#if defined(GPIOE)
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#if defined(GPIOF)
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#if defined(GPIOG)
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
}

static void _config_motor_gpio(const motor_hw_desc_t *hw)
{
    _enable_gpio_clk(hw->in1_port);
    _enable_gpio_clk(hw->in2_port);

    GPIO_InitTypeDef gpio = {
        .Mode  = GPIO_MODE_OUTPUT_PP,
        .Pull  = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };

    gpio.Pin = hw->in1_pin;
    HAL_GPIO_Init(hw->in1_port, &gpio);

    gpio.Pin = hw->in2_pin;
    HAL_GPIO_Init(hw->in2_port, &gpio);
}

static void _config_stby_gpio(void)
{
    _enable_gpio_clk(BSP_TB6612_STBY_PORT);

    GPIO_InitTypeDef gpio = {
        .Pin   = BSP_TB6612_STBY_PIN,
        .Mode  = GPIO_MODE_OUTPUT_PP,
        .Pull  = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(BSP_TB6612_STBY_PORT, &gpio);
}

static void _set_stby(bool enable)
{
    HAL_GPIO_WritePin(BSP_TB6612_STBY_PORT, BSP_TB6612_STBY_PIN,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_stby_global = enable;
}

static void _set_motor_direction(bsp_motor_priv_t *priv, dal_motor_dir_t dir)
{
    if (dir == DAL_MOTOR_DIR_CW) {
        HAL_GPIO_WritePin(priv->in1_port, priv->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(priv->in2_port, priv->in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(priv->in1_port, priv->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(priv->in2_port, priv->in2_pin, GPIO_PIN_SET);
    }
    priv->direction = dir;
}

static void _set_motor_brake(bsp_motor_priv_t *priv)
{
    HAL_GPIO_WritePin(priv->in1_port, priv->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(priv->in2_port, priv->in2_pin, GPIO_PIN_SET);
}

static void _set_motor_coast(bsp_motor_priv_t *priv)
{
    HAL_GPIO_WritePin(priv->in1_port, priv->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(priv->in2_port, priv->in2_pin, GPIO_PIN_RESET);
}

static dal_err_t _acquire_shared_timer(bsp_motor_priv_t *priv)
{
    if (s_shared_timer_handle == NULL) {
        bsp_timer_config_t cfg = {
            .freq_hz      = MOTOR_PWM_FREQ_HZ,
            .mode         = BSP_TIMER_MODE_PWM,
            .pwm_align    = BSP_TIMER_PWM_EDGE_ALIGNED,
            .pwm_polarity = BSP_TIMER_PWM_ACTIVE_HIGH,
            .enable_irq   = false,
        };
        bsp_err_t err = bsp_timer_open(1, &cfg, &s_shared_timer_handle);
        if (err != BSP_OK) return DAL_ERR_DEPENDENCY;

        err = bsp_timer_start(s_shared_timer_handle, 0, 0);
        if (err != BSP_OK) {
            bsp_timer_close(s_shared_timer_handle);
            s_shared_timer_handle = NULL;
            return DAL_ERR_DEPENDENCY;
        }
    }

    priv->timer_handle = s_shared_timer_handle;
    s_timer_ref_count++;
    return DAL_OK;
}

static void _release_shared_timer(void)
{
    if (s_timer_ref_count > 0) s_timer_ref_count--;
    if (s_timer_ref_count == 0 && s_shared_timer_handle != NULL) {
        bsp_timer_stop(s_shared_timer_handle);
        bsp_timer_close(s_shared_timer_handle);
        s_shared_timer_handle = NULL;
    }
}

/* ========================================================================== */
/*                         DAL ops 实现                                        */
/* ========================================================================== */

static dal_err_t bsp_motor_ops_init(dal_motor_dev_t *dev)
{
    bsp_motor_priv_t *priv = (bsp_motor_priv_t *)dev->drv_priv;
    if (!priv) return DAL_ERR_DEPENDENCY;

    dal_err_t ret = _acquire_shared_timer(priv);
    if (ret != DAL_OK) return ret;

    if (priv->timer_channel != 1) {
        bsp_err_t err = bsp_timer_pwm_start_channel(priv->timer_handle, priv->timer_channel);
        if (err != BSP_OK) {
            _release_shared_timer();
            return DAL_ERR_DEPENDENCY;
        }
    }

    priv->duty           = 0;
    priv->direction      = DAL_MOTOR_DIR_CW;
    priv->enabled        = false;
    priv->braking        = false;
    priv->is_initialized = true;

    bsp_timer_pwm_set_duty(priv->timer_handle, priv->timer_channel, 0);
    return DAL_OK;
}

static dal_err_t bsp_motor_ops_deinit(dal_motor_dev_t *dev)
{
    bsp_motor_priv_t *priv = (bsp_motor_priv_t *)dev->drv_priv;
    if (!priv || !priv->is_initialized) return DAL_ERR_NOT_READY;

    bsp_timer_pwm_set_duty(priv->timer_handle, priv->timer_channel, 0);
    _set_motor_coast(priv);

    priv->is_initialized = false;
    _release_shared_timer();
    priv->timer_handle = NULL;
    return DAL_OK;
}

static dal_err_t bsp_motor_ops_selftest(dal_motor_dev_t *dev,
                                        dal_motor_selftest_result_t *result)
{
    (void)dev;
    *result = DAL_MOTOR_SELFTEST_NOT_IMPL;
    return DAL_OK;
}

static dal_err_t bsp_motor_ops_set_duty(dal_motor_dev_t *dev, uint8_t duty_pct)
{
    bsp_motor_priv_t *priv = (bsp_motor_priv_t *)dev->drv_priv;
    if (!priv || !priv->is_initialized) return DAL_ERR_NOT_READY;
    if (!priv->enabled) return DAL_ERR_NOT_READY;
    if (duty_pct > 100) return DAL_ERR_PARAM_INVALID;

    /* [v1.2] 设置新占空比时自动退出刹车状态 */
    if (priv->braking && duty_pct > 0) {
        _set_motor_direction(priv, priv->direction);
        priv->braking = false;
    }

    bsp_err_t err = bsp_timer_pwm_set_duty(priv->timer_handle,
                                            priv->timer_channel, duty_pct);
    if (err != BSP_OK) {
        return (err == BSP_ERR_UNSUPPORT) ? DAL_ERR_NOT_SUPPORTED : DAL_ERR_DEPENDENCY;
    }
    priv->duty = duty_pct;
    return DAL_OK;
}

static dal_err_t bsp_motor_ops_set_direction(dal_motor_dev_t *dev, dal_motor_dir_t dir)
{
    bsp_motor_priv_t *priv = (bsp_motor_priv_t *)dev->drv_priv;
    if (!priv || !priv->is_initialized) return DAL_ERR_NOT_READY;
    if (!priv->enabled) return DAL_ERR_NOT_READY;

    /* [v1.2] 刹车状态下仅缓存方向，不立即切换 GPIO（避免短路风险） */
    if (priv->braking) {
        priv->direction = dir;
        return DAL_OK;
    }

    _set_motor_direction(priv, dir);
    return DAL_OK;
}

static dal_err_t bsp_motor_ops_enable(dal_motor_dev_t *dev)
{
    bsp_motor_priv_t *priv = (bsp_motor_priv_t *)dev->drv_priv;
    if (!priv || !priv->is_initialized) return DAL_ERR_NOT_READY;

    if (!s_stby_global) _set_stby(true);

    /* [v1.2] enable 清除刹车标志，恢复正常方向控制 */
    priv->braking = false;
    _set_motor_direction(priv, priv->direction);
    bsp_timer_pwm_set_duty(priv->timer_handle, priv->timer_channel, priv->duty);

    priv->enabled = true;
    return DAL_OK;
}

static dal_err_t bsp_motor_ops_disable(dal_motor_dev_t *dev)
{
    bsp_motor_priv_t *priv = (bsp_motor_priv_t *)dev->drv_priv;
    if (!priv || !priv->is_initialized) return DAL_ERR_NOT_READY;

    bsp_timer_pwm_set_duty(priv->timer_handle, priv->timer_channel, 0);
    _set_motor_coast(priv);

    priv->enabled = false;
    priv->braking = false;  /* [v1.2] disable 同时清除刹车标志 */
    return DAL_OK;
}

/**
 * @brief [v1.2] 刹车：同步更新 duty + braking 标志
 */
static dal_err_t bsp_motor_ops_brake(dal_motor_dev_t *dev)
{
    bsp_motor_priv_t *priv = (bsp_motor_priv_t *)dev->drv_priv;
    if (!priv || !priv->is_initialized) return DAL_ERR_NOT_READY;
    if (!priv->enabled) return DAL_ERR_NOT_READY;

    bsp_timer_pwm_set_duty(priv->timer_handle, priv->timer_channel, 0);
    _set_motor_brake(priv);

    priv->duty    = 0;      /* 同步软件占空比 */
    priv->braking = true;   /* 标记刹车状态 */
    return DAL_OK;
}

/**
 * @brief [v1.2] 完整四态状态机
 */
static dal_err_t bsp_motor_ops_get_state(dal_motor_dev_t *dev, dal_motor_state_t *state)
{
    bsp_motor_priv_t *priv = (bsp_motor_priv_t *)dev->drv_priv;
    if (!priv || !priv->is_initialized) return DAL_ERR_NOT_READY;

    if (!priv->enabled) {
        *state = DAL_MOTOR_STATE_DISABLED;
    } else if (priv->braking) {
        *state = DAL_MOTOR_STATE_BRAKING;
    } else if (priv->duty == 0) {
        *state = DAL_MOTOR_STATE_IDLE;
    } else {
        *state = DAL_MOTOR_STATE_RUNNING;
    }
    return DAL_OK;
}

static dal_err_t bsp_motor_ops_get_fault(dal_motor_dev_t *dev, uint32_t *fault)
{
    (void)dev;
    *fault = 0U;
    return DAL_OK;
}

static dal_err_t bsp_motor_ops_set_fault_irq_enable(dal_motor_dev_t *dev, bool enable)
{
    (void)dev;
    (void)enable;
    return DAL_ERR_NOT_SUPPORTED;
}

/* ========================================================================== */
/*                       dal_motor_ops_t 实例                                  */
/* ========================================================================== */

static const dal_motor_ops_t g_bsp_motor_ops = {
    .init                 = bsp_motor_ops_init,
    .deinit               = bsp_motor_ops_deinit,
    .selftest             = bsp_motor_ops_selftest,
    .set_duty             = bsp_motor_ops_set_duty,
    .set_direction        = bsp_motor_ops_set_direction,
    .enable               = bsp_motor_ops_enable,
    .disable              = bsp_motor_ops_disable,
    .brake                = bsp_motor_ops_brake,
    .get_state            = bsp_motor_ops_get_state,
    .get_fault            = bsp_motor_ops_get_fault,
    .set_fault_irq_enable = bsp_motor_ops_set_fault_irq_enable,
};

/* ========================================================================== */
/*                     BSP 公共接口                                            */
/* ========================================================================== */

bsp_err_t bsp_motor_init(void)
{
    /* [v1.2] STBY 配置与初始化放在最前面，确保后续 GPIO 操作安全 */
    _config_stby_gpio();
    _set_stby(false);

    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        const motor_hw_desc_t *hw = &s_motor_table[i];
        _config_motor_gpio(hw);

        bsp_motor_priv_t *priv = &s_priv_pool[i];
        memset(priv, 0, sizeof(bsp_motor_priv_t));
        priv->timer_channel = hw->timer_channel;
        priv->in1_port      = hw->in1_port;
        priv->in1_pin       = hw->in1_pin;
        priv->in2_port      = hw->in2_port;
        priv->in2_pin       = hw->in2_pin;
        priv->direction     = DAL_MOTOR_DIR_CW;

        dal_motor_dev_t *dev = &s_dev_pool[i];
        dev->name     = hw->name;
        dev->ops      = &g_bsp_motor_ops;
        dev->drv_priv = priv;

        dal_err_t ret = dal_motor_register(dev);
        if (ret != DAL_OK) {
            for (uint32_t j = 0; j < i; j++) {
                (void)dal_motor_unregister(&s_dev_pool[j]);
            }
            return BSP_ERR_FAIL;
        }
    }

    return BSP_OK;
}