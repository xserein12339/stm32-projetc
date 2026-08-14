/**
 * @file    bsp_led.c
 * @brief   板级 LED BSP 层实现（基于 STM32F1 HAL + FreeRTOS）
 * @note    - 硬件映射完全引用 board_v1_config.h，禁止硬编码 GPIO
 *          - 对外仅暴露 bsp_led_init()，内部静态管理所有实例
 *          - LED 逻辑状态与硬件电平极性映射在 set/get_state 中处理
 * @author xserein
 * @version v1.1
 */
#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_led.h"
#include "dal_led.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ========================================================================== */
/*                          内部配置                                            */
/* ========================================================================== */

typedef struct {
    const char   *name;
    GPIO_TypeDef *port;
    uint32_t      pin;
    uint8_t       active_level;  ///< 点亮电平：GPIO_PIN_SET 或 GPIO_PIN_RESET
} led_hw_desc_t;

static const led_hw_desc_t s_led_table[] = {
    { "led1", BSP_LED1_PORT, BSP_LED1_PIN, GPIO_PIN_RESET },
    { "led2", BSP_LED2_PORT, BSP_LED2_PIN, GPIO_PIN_SET   },
    { "led3", BSP_LED3_PORT, BSP_LED3_PIN, GPIO_PIN_RESET },
};

#define LED_COUNT  (sizeof(s_led_table) / sizeof(s_led_table[0]))

/* ========================================================================== */
/*                               私有上下文                                      */
/* ========================================================================== */

typedef struct {
    GPIO_TypeDef     *port;
    uint32_t          pin;
    uint8_t           active_level;
    dal_led_state_t   current_state;
} bsp_led_priv_t;

static bsp_led_priv_t s_priv_pool[LED_COUNT];
static dal_led_dev_t  s_dev_pool[LED_COUNT];

/* ========================================================================== */
/*                            内联硬件操作                                      */
/* ========================================================================== */

/**
 * @brief 使能指定 GPIO 端口的时钟（支持所有端口，与 bsp_key 保持一致）
 */
static inline void _gpio_clock_enable(GPIO_TypeDef *port)
{
#if defined(GPIOA)
    if (port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); return; }
#endif
#if defined(GPIOB)
    if (port == GPIOB)      { __HAL_RCC_GPIOB_CLK_ENABLE(); return; }
#endif
#if defined(GPIOC)
    if (port == GPIOC)      { __HAL_RCC_GPIOC_CLK_ENABLE(); return; }
#endif
#if defined(GPIOD)
    if (port == GPIOD)      { __HAL_RCC_GPIOD_CLK_ENABLE(); return; }
#endif
#if defined(GPIOE)
    if (port == GPIOE)      { __HAL_RCC_GPIOE_CLK_ENABLE(); return; }
#endif
#if defined(GPIOF)
    if (port == GPIOF)      { __HAL_RCC_GPIOF_CLK_ENABLE(); return; }
#endif
#if defined(GPIOG)
    if (port == GPIOG)      { __HAL_RCC_GPIOG_CLK_ENABLE(); return; }
#endif
#if defined(GPIOH)
    if (port == GPIOH)      { __HAL_RCC_GPIOH_CLK_ENABLE(); return; }
#endif
#if defined(GPIOI)
    if (port == GPIOI)      { __HAL_RCC_GPIOI_CLK_ENABLE(); return; }
#endif
}

/**
 * @brief 将逻辑状态转换为物理电平
 */
static inline GPIO_PinState _state_to_pin_level(uint8_t active_level, dal_led_state_t state)
{
    if (state == DAL_LED_ON) {
        return (GPIO_PinState)active_level;
    }
    return (active_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

/* ========================================================================== */
/*                         DAL ops 实现                                         */
/* ========================================================================== */

static dal_err_t bsp_led_ops_init(dal_led_dev_t *dev)
{
    bsp_led_priv_t *priv = (bsp_led_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_DEPENDENCY;
    }

    _gpio_clock_enable(priv->port);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = priv->pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(priv->port, &gpio);

    /* 默认熄灭 */
    HAL_GPIO_WritePin(priv->port, priv->pin,
                      _state_to_pin_level(priv->active_level, DAL_LED_OFF));
    priv->current_state = DAL_LED_OFF;

    return DAL_OK;
}

static dal_err_t bsp_led_ops_deinit(dal_led_dev_t *dev)
{
    bsp_led_priv_t *priv = (bsp_led_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    /* 先确保 LED 熄灭，再切换为模拟输入 */
    HAL_GPIO_WritePin(priv->port, priv->pin,
                      _state_to_pin_level(priv->active_level, DAL_LED_OFF));

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = priv->pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(priv->port, &gpio);

    priv->current_state = DAL_LED_OFF;
    return DAL_OK;
}

static dal_err_t bsp_led_ops_set_state(dal_led_dev_t *dev, dal_led_state_t state)
{
    bsp_led_priv_t *priv = (bsp_led_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    HAL_GPIO_WritePin(priv->port, priv->pin,
                      _state_to_pin_level(priv->active_level, state));
    priv->current_state = state;
    return DAL_OK;
}

static dal_err_t bsp_led_ops_get_state(dal_led_dev_t *dev, dal_led_state_t *state)
{
    bsp_led_priv_t *priv = (bsp_led_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    *state = priv->current_state;
    return DAL_OK;
}

/* ========================================================================== */
/*                         DAL 操作集实例                                       */
/* ========================================================================== */

static const dal_led_ops_t g_bsp_led_ops = {
    .init      = bsp_led_ops_init,
    .deinit    = bsp_led_ops_deinit,
    .selftest  = NULL,
    .set_state = bsp_led_ops_set_state,
    .get_state = bsp_led_ops_get_state,
};

/* ========================================================================== */
/*                     BSP 公共接口（唯一对外入口）                              */
/* ========================================================================== */

bsp_err_t bsp_led_init(void)
{
    for (uint32_t i = 0; i < LED_COUNT; i++) {
        const led_hw_desc_t *hw = &s_led_table[i];

        bsp_led_priv_t *priv = &s_priv_pool[i];
        memset(priv, 0, sizeof(bsp_led_priv_t));
        priv->port          = hw->port;
        priv->pin           = hw->pin;
        priv->active_level  = hw->active_level;
        priv->current_state = DAL_LED_OFF;

        dal_led_dev_t *dev = &s_dev_pool[i];
        dev->name     = hw->name;
        dev->ops      = &g_bsp_led_ops;
        dev->drv_priv = priv;

        dal_err_t ret = dal_led_register(dev);
        if (ret != DAL_OK) {
            for (uint32_t j = 0; j < i; j++) {
                (void)dal_led_unregister(&s_dev_pool[j]);
            }
            return BSP_ERR_FAIL;
        }
    }

    return BSP_OK;
}