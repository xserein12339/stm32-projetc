/**
 * @file    bsp_led.c
 * @brief   板级 LED BSP 层实现（基于 STM32F1 HAL + FreeRTOS）
 * @note    - 硬件映射完全引用 board_v1_config.h，禁止硬编码 GPIO
 *          - 对外仅暴露 bsp_led_init()，内部静态管理所有实例
 *          - LED 逻辑状态与硬件电平极性映射在 set/get_state 中处理
 * @author xserein
 * @version v1.0
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

/**
 * @brief LED 硬件描述符（编译期常量）
 */
typedef struct {
    const char   *name;          ///< LED 名称（如 "led1", "led2"）
    GPIO_TypeDef *port;          ///< GPIO 端口
    uint32_t      pin;           ///< GPIO 引脚
    uint8_t       active_level;  ///< 点亮电平：GPIO_PIN_SET 或 GPIO_PIN_RESET
} led_hw_desc_t;

/**
 * @brief 板载 LED 硬件映射表
 * @note  所有引脚定义来自 board_v1_config.h
 *        根据实际板卡 LED 连接方式配置 active_level（高/低电平有效）
 */
static const led_hw_desc_t s_led_table[] = {
    { "led1", BSP_LED1_PORT, BSP_LED1_PIN, GPIO_PIN_SET   },  
    { "led2", BSP_LED2_PORT, BSP_LED2_PIN, GPIO_PIN_SET   },
    { "led3", BSP_LED3_PORT, BSP_LED3_PIN, GPIO_PIN_RESET },   
};

#define LED_COUNT  (sizeof(s_led_table) / sizeof(s_led_table[0]))

/* ========================================================================== */
/*                               私有上下文                                      */
/* ========================================================================== */

typedef struct {
    GPIO_TypeDef     *port;          ///< GPIO 端口
    uint32_t          pin;           ///< GPIO 引脚
    uint8_t           active_level;  ///< 点亮电平
    dal_led_state_t   current_state; ///< 当前逻辑状态（用于状态追踪，可选）
} bsp_led_priv_t;

static bsp_led_priv_t s_priv_pool[LED_COUNT];
static dal_led_dev_t  s_dev_pool[LED_COUNT];

/* ========================================================================== */
/*                         DAL ops 实现                                         */
/* ========================================================================== */

/**
 * @brief 初始化 LED GPIO（由上层 dal_led_init 调用）
 */
static dal_err_t bsp_led_ops_init(dal_led_dev_t *dev)
{
    bsp_led_priv_t *priv = (bsp_led_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_DEPENDENCY;
    }

    /* 使能 GPIO 时钟 */
    if (priv->port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (priv->port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (priv->port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (priv->port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else { return DAL_ERR_DEPENDENCY; }

    /* 配置为推挽输出，上拉/下拉取决于硬件（通常无上下拉） */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = priv->pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(priv->port, &gpio);

    /* 默认熄灭（根据 active_level 设置初始电平） */
    if (priv->active_level == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(priv->port, priv->pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(priv->port, priv->pin, GPIO_PIN_SET);
    }
    priv->current_state = DAL_LED_OFF;

    return DAL_OK;
}

/**
 * @brief 反初始化（切换为模拟输入以降低功耗）
 */
static dal_err_t bsp_led_ops_deinit(dal_led_dev_t *dev)
{
    bsp_led_priv_t *priv = (bsp_led_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = priv->pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(priv->port, &gpio);

    priv->current_state = DAL_LED_OFF;
    return DAL_OK;
}
 

/**
 * @brief 设置 LED 逻辑状态（映射硬件有效电平）
 */
static dal_err_t bsp_led_ops_set_state(dal_led_dev_t *dev, dal_led_state_t state)
{
    bsp_led_priv_t *priv = (bsp_led_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    /* 将逻辑状态转换为硬件电平 */
    GPIO_PinState pin_level;
    if (state == DAL_LED_ON) {
        pin_level = priv->active_level;      ///< 点亮电平
    } else { ///< OFF
        pin_level = (priv->active_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    HAL_GPIO_WritePin(priv->port, priv->pin, pin_level);
    priv->current_state = state;
    return DAL_OK;
}
static dal_err_t bsp_led_ops_get_state(dal_led_dev_t *dev, dal_led_state_t *state)
{
    bsp_led_priv_t *priv = (bsp_led_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    /* LED 为纯输出设备，逻辑状态由软件维护，无需回读硬件 */
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

        /* 初始化私有上下文 */
        bsp_led_priv_t *priv = &s_priv_pool[i];
        memset(priv, 0, sizeof(bsp_led_priv_t));
        priv->port         = hw->port;
        priv->pin          = hw->pin;
        priv->active_level = hw->active_level;
        priv->current_state = DAL_LED_OFF;

        /* 填充 DAL 设备描述符 */
        dal_led_dev_t *dev = &s_dev_pool[i];
        dev->name     = hw->name;
        dev->ops      = &g_bsp_led_ops;
        dev->drv_priv = priv;

        /* 注册到 DAL 框架 */
        dal_err_t ret = dal_led_register(dev);
        if (ret != DAL_OK) {
            /* 回滚：注销已成功的实例 */
            for (uint32_t j = 0; j < i; j++) {
                (void)dal_led_unregister(&s_dev_pool[j]);
            }
            return BSP_ERR_FAIL;
        }
    }

    return BSP_OK;
}