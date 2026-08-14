/**
 * @file    bsp_key.c
 * @brief   板级按键 BSP 层实现（基于 STM32F1 HAL + FreeRTOS）
 * @note    - 硬件映射完全引用 board_v1_config.h 
 *          - 对外仅暴露 bsp_key_init()，内部静态管理所有实例
 *          - 消抖采用中断 + 时间戳法，支持异步回调与同步轮询双模式
 *          - 中断优先级默认与 FreeRTOS configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 一致
 * @author xserein
 * @version v2.1
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

#ifndef KEY_DEBOUNCE_MS
#define KEY_DEBOUNCE_MS  20U
#endif

#define KEY_DEBOUNCE_TICKS  pdMS_TO_TICKS(KEY_DEBOUNCE_MS)

/* ========================================================================== */
/*                     硬件描述符与私有上下文                                   */
/* ========================================================================== */

typedef struct {
    const char   *name;
    GPIO_TypeDef *port;
    uint32_t      pin;
    uint32_t      exti_line;
    IRQn_Type     irqn;
    uint8_t       active_level;     ///< 有效电平：0=低电平有效，1=高电平有效
} key_hw_desc_t;

typedef struct {
    GPIO_TypeDef     *port;
    uint32_t          pin;
    uint32_t          exti_line;
    IRQn_Type         irqn;
    uint8_t           active_level;

    /* 消抖状态（ISR 与轮询共享，需注意并发安全） */
    volatile uint8_t  stable_level;      ///< 稳定后的电平 (0/1)
    volatile uint8_t  last_raw;          ///< 上一次采样的原始电平
    volatile TickType_t last_change_tick;///< 最后一次边沿变化的时间戳

    /* 事件通知：ISR 置位，由 notify 消费 */
    volatile uint8_t  pending_event;     ///< 0=无, DAL_KEY_EVT_DOWN, DAL_KEY_EVT_UP
    volatile bool     irq_enabled;       ///< 中断是否已使能
} bsp_key_priv_t;

/* ========================================================================== */
/*                        静态数据池                                            */
/* ========================================================================== */

static const key_hw_desc_t s_key_table[] = {
    {
        .name          = "key1",
        .port          = BSP_KEY1_PORT,
        .pin           = BSP_KEY1_PIN,
        .exti_line     = BSP_KEY1_EXTI_LINE,
        .irqn          = BSP_KEY1_IRQN,
        .active_level  = GPIO_PIN_RESET,
    },
    {
        .name          = "key2",
        .port          = BSP_KEY2_PORT,
        .pin           = BSP_KEY2_PIN,
        .exti_line     = BSP_KEY2_EXTI_LINE,
        .irqn          = BSP_KEY2_IRQN,
        .active_level  = GPIO_PIN_RESET,
    },
    {
        .name          = "key3",
        .port          = BSP_KEY3_PORT,
        .pin           = BSP_KEY3_PIN,
        .exti_line     = BSP_KEY3_EXTI_LINE,
        .irqn          = BSP_KEY3_IRQN,
        .active_level  = GPIO_PIN_RESET,
    },
};

#define KEY_COUNT  (sizeof(s_key_table) / sizeof(s_key_table[0]))

static bsp_key_priv_t s_priv_pool[KEY_COUNT];
static dal_key_dev_t  s_dev_pool[KEY_COUNT];

/* ========================================================================== */
/*                            内联硬件操作                                      */
/* ========================================================================== */

static inline uint8_t _read_raw(const bsp_key_priv_t *priv)
{
    return (HAL_GPIO_ReadPin(priv->port, priv->pin) == priv->active_level) ? 1U : 0U;
}

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
static dal_err_t bsp_key_ops_set_irq_enable(dal_key_dev_t *dev, bool enable);
/* ========================================================================== */
/*                         DAL ops 实现                                         */
/* ========================================================================== */

static dal_err_t bsp_key_ops_init(dal_key_dev_t *dev)
{
    bsp_key_priv_t *priv = (bsp_key_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_DEPENDENCY;
    }

    _gpio_clock_enable(priv->port);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = priv->pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = (priv->active_level == 0U) ? GPIO_PULLUP : GPIO_PULLDOWN;
    HAL_GPIO_Init(priv->port, &gpio);

    /* 同步初始状态 */
    uint8_t raw = _read_raw(priv);
    priv->stable_level     = raw;
    priv->last_raw         = raw;
    priv->last_change_tick = xTaskGetTickCount();
    priv->pending_event    = 0;
    priv->irq_enabled      = false;

    return DAL_OK;
}

static dal_err_t bsp_key_ops_deinit(dal_key_dev_t *dev)
{
    bsp_key_priv_t *priv = (bsp_key_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    (void)bsp_key_ops_set_irq_enable(dev, false);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = priv->pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(priv->port, &gpio);

    return DAL_OK;
}

/**
 * @brief 获取当前稳定电平（同步轮询模式）
 * @note  【并发安全说明】
 *        当 IRQ 已使能时，此函数与 ISR 并发访问消抖状态变量。
 *        由于这些变量均为 volatile 单字节/单字读写，在 Cortex-M 上
 *        原子性由硬件保证，不会出现撕裂读。
 *        最坏情况：轮询读到 ISR 刚更新的中间状态，下一次轮询即可收敛。
 *        对于按键消抖场景，此精度完全可接受。
 */
static dal_err_t bsp_key_ops_get_level(dal_key_dev_t *dev, dal_key_level_t *level)
{
    bsp_key_priv_t *priv = (bsp_key_priv_t *)dev->drv_priv;

    /*
     * 若中断已使能，消抖由 ISR 驱动，轮询仅读取稳定值。
     * 避免轮询与 ISR 同时更新消抖状态导致时序紊乱。
     */
    if (priv->irq_enabled) {
        *level = (priv->stable_level == 1U) ? DAL_KEY_LEVEL_PRESSED : DAL_KEY_LEVEL_RELEASED;
        return DAL_OK;
    }

    /* 纯轮询模式：自行执行消抖 */
    uint8_t raw = _read_raw(priv);
    TickType_t now = xTaskGetTickCount();

    if (raw != priv->last_raw) {
        priv->last_raw = raw;
        priv->last_change_tick = now;
    } else if ((now - priv->last_change_tick) >= KEY_DEBOUNCE_TICKS) {
        priv->stable_level = raw;
    }

    *level = (priv->stable_level == 1U) ? DAL_KEY_LEVEL_PRESSED : DAL_KEY_LEVEL_RELEASED;
    return DAL_OK;
}

static void _config_exti(bsp_key_priv_t *priv)
{
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = priv->pin;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = (priv->active_level == 0U) ? GPIO_PULLUP : GPIO_PULLDOWN;
    HAL_GPIO_Init(priv->port, &gpio);

    HAL_NVIC_SetPriority(priv->irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(priv->irqn);
}

static dal_err_t bsp_key_ops_set_irq_enable(dal_key_dev_t *dev, bool enable)
{
    bsp_key_priv_t *priv = (bsp_key_priv_t *)dev->drv_priv;
    if (!priv || !priv->port) {
        return DAL_ERR_NOT_READY;
    }

    if (enable && !priv->irq_enabled) {
        /* 使能前同步当前电平，避免历史残留触发假事件 */
        uint8_t raw = _read_raw(priv);
        priv->stable_level     = raw;
        priv->last_raw         = raw;
        priv->last_change_tick = xTaskGetTickCount();
        priv->pending_event    = 0;

        _config_exti(priv);
        priv->irq_enabled = true;
    } else if (!enable && priv->irq_enabled) {
        HAL_NVIC_DisableIRQ(priv->irqn);

        GPIO_InitTypeDef gpio = {0};
        gpio.Pin  = priv->pin;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = (priv->active_level == 0U) ? GPIO_PULLUP : GPIO_PULLDOWN;
        HAL_GPIO_Init(priv->port, &gpio);

        priv->irq_enabled = false;
    }

    return DAL_OK;
}

/**
 * @brief 消抖更新（中断专用）
 * @note  仅更新状态和置位 pending_event，【不】在此处调用 dal_key_notify_event。
 *        事件通知延迟到 IRQ handler 末尾统一处理，避免消抖窗口内重复触发。
 */
static inline void _update_debounce_from_isr(bsp_key_priv_t *priv)
{
    uint8_t raw = _read_raw(priv);
    TickType_t now = xTaskGetTickCountFromISR();

    if (raw != priv->last_raw) {
        priv->last_raw = raw;
        priv->last_change_tick = now;
    } else if ((now - priv->last_change_tick) >= KEY_DEBOUNCE_TICKS) {
        if (raw != priv->stable_level) {
            priv->stable_level = raw;
            /* 仅置位，不消费；IRQ handler 统一通知 */
            priv->pending_event = (raw == 1U) ? DAL_KEY_EVT_DOWN : DAL_KEY_EVT_UP;
        }
    }
}

/* ========================================================================== */
/*                     EXTI 中断服务例程                                        */
/* ========================================================================== */

/**
 * @brief 根据 EXTI 线号查找对应的设备索引
 * @note  运行时查表替代编译期硬绑定，确保 IRQn 与设备索引正确对应。
 *        KEY_COUNT 通常 ≤ 8，线性扫描开销可忽略。
 */
static inline int32_t _find_key_index_by_pin(uint32_t pin)
{
    for (uint32_t i = 0; i < KEY_COUNT; i++) {
        if (s_key_table[i].pin == pin) {
            return (int32_t)i;
        }
    }
    return -1;
}

/**
 * @brief 中断服务例程核心逻辑
 * @param pin  触发中断的 GPIO_PIN_x（用于运行时匹配设备）
 */
static void _bsp_key_irq_handler(uint32_t pin)
{
    int32_t idx = _find_key_index_by_pin(pin);
    if (idx < 0) {
        /* 非本模块管理的引脚，仅清除标志 */
        __HAL_GPIO_EXTI_CLEAR_IT(pin);
        return;
    }

    dal_key_dev_t *dev = &s_dev_pool[idx];
    bsp_key_priv_t *priv = (bsp_key_priv_t *)dev->drv_priv;

    __HAL_GPIO_EXTI_CLEAR_IT(pin);

    /* 更新消抖状态（仅置位 pending_event） */
    _update_debounce_from_isr(priv);

    /*
     * 统一在 IRQ handler 末尾通知事件。
     * 读取并清除 pending_event 使用局部变量快照，
     * 避免通知期间新中断覆盖未处理的事件。
     */
    uint8_t evt = priv->pending_event;
    if (evt != 0) {
        priv->pending_event = 0;
        dal_key_notify_event(dev, (dal_key_event_t)evt);
    }
}

/* ========================================================================== */
/*                      EXTI 中断向量                                           */
/* ========================================================================== */

/**
 * @note  使用运行时 pin 匹配替代编译期索引绑定。
 *        即使 board_v1_config.h 中 KEY 映射到非连续 EXTI 线，也能正确路由。
 */
#if defined(BSP_KEY1_IRQN)
void EXTI0_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_0); }
#endif

#if defined(BSP_KEY2_IRQN)
void EXTI1_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_1); }
#endif

#if defined(BSP_KEY3_IRQN)
void EXTI2_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_2); }
#endif

#if defined(BSP_KEY4_IRQN)
void EXTI3_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_3); }
#endif

#if defined(BSP_KEY5_IRQN)
void EXTI4_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_4); }
#endif

/* EXTI9_5 / EXTI15_10 共享中断需额外判断 PIN，此处省略模板 */

/* ========================================================================== */
/*                     BSP 公共接口                                              */
/* ========================================================================== */

static const dal_key_ops_t g_bsp_key_ops = {
    .init             = bsp_key_ops_init,
    .deinit           = bsp_key_ops_deinit,
    .selftest         = NULL,
    .get_level        = bsp_key_ops_get_level,
    .set_irq_enable   = bsp_key_ops_set_irq_enable,
};

bsp_err_t bsp_key_init(void)
{
    for (uint32_t i = 0; i < KEY_COUNT; i++) {
        const key_hw_desc_t *hw = &s_key_table[i];

        bsp_key_priv_t *priv = &s_priv_pool[i];
        memset(priv, 0, sizeof(bsp_key_priv_t));
        priv->port         = hw->port;
        priv->pin          = hw->pin;
        priv->exti_line    = hw->exti_line;
        priv->irqn         = hw->irqn;
        priv->active_level = hw->active_level;

        dal_key_dev_t *dev = &s_dev_pool[i];
        dev->name     = hw->name;
        dev->ops      = &g_bsp_key_ops;
        dev->drv_priv = priv;

        dal_err_t ret = dal_key_register(dev);
        if (ret != DAL_OK) {
            for (uint32_t j = 0; j < i; j++) {
                (void)dal_key_unregister(&s_dev_pool[j]);
            }
            return BSP_ERR_FAIL;
        }
    }

    return BSP_OK;
}