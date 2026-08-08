/**
 * @file    bsp_key.c
 * @brief   板级按键 BSP 层实现（基于 STM32F1 HAL + FreeRTOS）
 * @note    - 硬件映射完全引用 board_v1_config.h 
 *          - 对外仅暴露 bsp_key_init()，内部静态管理所有实例
 *          - 消抖采用时间戳比较法，依赖 xTaskGetTickCount()
 * @author xserein
 * @version v1.0
 */

#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_key.h"
#include "dal_key.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ========================================================================== */
/*                          内部配置                                            */
/* ========================================================================== */

/** 消抖窗口，自动适配 configTICK_RATE_HZ */
#define KEY_DEBOUNCE_TICKS  pdMS_TO_TICKS(20U)

/**
 * @brief 按键硬件描述符 
 */
typedef struct {
    const char   *name;
    GPIO_TypeDef *port;
    uint32_t      pin;
    uint8_t       active_level;  
} key_hw_desc_t;

static const key_hw_desc_t s_key_table[] = {
    { "key1", BSP_KEY1_PORT, BSP_KEY1_PIN, GPIO_PIN_RESET },
    { "key2", BSP_KEY2_PORT, BSP_KEY2_PIN, GPIO_PIN_RESET },
    { "key3", BSP_KEY3_PORT, BSP_KEY3_PIN, GPIO_PIN_RESET },
};

#define KEY_COUNT  (sizeof(s_key_table) / sizeof(s_key_table[0]))

/* ========================================================================== */
/*                               私有上下文                                      */
/* ========================================================================== */

typedef struct {
    GPIO_TypeDef     *port;
    uint32_t          pin;
    uint8_t           active_level;
    uint8_t           stable_state;      ///< 1=按下, 0=释放
    uint8_t           last_raw;          ///< 上一次采样的原始电平
    TickType_t        last_change_tick;  ///< 最后一次边沿变化的时间戳
} bsp_key_priv_t;

static bsp_key_priv_t s_priv_pool[KEY_COUNT];
static dal_key_dev_t  s_dev_pool[KEY_COUNT];

/* ========================================================================== */
/*                            内联硬件读取                                       */
/* ========================================================================== */

static inline uint8_t read_raw(const bsp_key_priv_t *priv)
{
    return (HAL_GPIO_ReadPin(priv->port, priv->pin) == priv->active_level) ? 1U : 0U;
}

/* ========================================================================== */
/*                         DAL ops 实现                                         */
/* ========================================================================== */

static dal_err_t bsp_key_ops_init(dal_key_dev_t *dev)
{
    bsp_key_priv_t *priv = (bsp_key_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_DEPENDENCY;
    }

    if (priv->port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (priv->port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (priv->port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (priv->port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else { return DAL_ERR_DEPENDENCY; }

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = priv->pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = (priv->active_level == GPIO_PIN_RESET) ? GPIO_PULLUP : GPIO_PULLDOWN;
    HAL_GPIO_Init(priv->port, &gpio);

    /* 同步初始状态 */
    uint8_t raw = read_raw(priv);
    priv->stable_state     = raw;
    priv->last_raw         = raw;
    priv->last_change_tick = xTaskGetTickCount();

    return DAL_OK;
}

static dal_err_t bsp_key_ops_deinit(dal_key_dev_t *dev)
{
    bsp_key_priv_t *priv = (bsp_key_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = priv->pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(priv->port, &gpio);

    return DAL_OK;
}

/**
 * @brief 带消抖的状态读取
 * @note  单任务轮询，无临界区
 */
static dal_err_t bsp_key_ops_get_state(dal_key_dev_t *dev, int *state)
{
    bsp_key_priv_t *priv = (bsp_key_priv_t *)dev->drv_priv;
    uint8_t raw = read_raw(priv);
    TickType_t now = xTaskGetTickCount();

    if (raw != priv->last_raw) {
        /* 边沿变化：重置消抖计时器，但不立即更新稳定状态 */
        priv->last_raw = raw;
        priv->last_change_tick = now;
    } else if ((now - priv->last_change_tick) >= KEY_DEBOUNCE_TICKS) {
        /* 电平稳定超过消抖窗口，确认新状态 */
        priv->stable_state = raw;
    }

    *state = priv->stable_state;
    return DAL_OK;
}

 

static const dal_key_ops_t g_bsp_key_ops = {
    .init      = bsp_key_ops_init,
    .deinit    = bsp_key_ops_deinit,
    .get_state = bsp_key_ops_get_state,
    .selftest  = NULL,
};

/* ========================================================================== */
/*                     BSP 公共接口                                              */
/* ========================================================================== */

bsp_err_t bsp_key_init(void)
{
    for (uint32_t i = 0; i < KEY_COUNT; i++) {
        const key_hw_desc_t *hw = &s_key_table[i];

        /* 初始化私有上下文 */
        bsp_key_priv_t *priv = &s_priv_pool[i];
        memset(priv, 0, sizeof(bsp_key_priv_t));
        priv->port         = hw->port;
        priv->pin          = hw->pin;
        priv->active_level = hw->active_level;

        /* 填充 DAL 设备描述符 */
        dal_key_dev_t *dev = &s_dev_pool[i];
        dev->name     = hw->name;
        dev->ops      = &g_bsp_key_ops;
        dev->drv_priv = priv;

        /* 注册到 DAL 框架 */
        dal_err_t ret = dal_key_register(dev);
        if (ret != DAL_OK) {
            /* 回滚：注销已成功的实例 */
            for (uint32_t j = 0; j < i; j++) {
                (void)dal_key_unregister(&s_dev_pool[j]);
            }
            return BSP_ERR_FAIL;
        }
    }

    return BSP_OK;
}