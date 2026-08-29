/**
 * @file    bsp_key.c
 * @brief   板级按键 BSP 层实现（基于 STM32F1 HAL + FreeRTOS）
 * @note    - 硬件映射完全引用 board_v1_config.h
 *          - 对外仅暴露 bsp_key_init()，内部静态管理所有实例
 *          - 消抖 v2.0：EXTI 仅记录边沿时间戳，稳定确认由 1ms tick 扫描
 *            （bsp_key_tick_scan，经 FreeRTOS tick hook 调用）完成
 *          - 中断优先级默认与 FreeRTOS configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 一致
 * @author xserein
 * @version v2.2
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

    /* 消抖状态（EXTI ISR 与 tick 扫描共享，需注意并发安全） */
    volatile uint8_t  stable_level;      ///< 稳定后的电平 (0/1)
    volatile uint8_t  last_raw;          ///< 上一次采样的原始电平
    volatile TickType_t last_change_tick;///< 最后一次边沿变化的时间戳

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
 * @brief 边沿记录（EXTI ISR 专用）
 * @note  v2.2：只更新电平与时间戳，不做稳定确认、不发事件。
 *        WHY：EXTI 为边沿触发，按键稳定后不再产生中断，"下一次中断
 *        时确认"的逻辑永远执行不到（干净按键无事件，见 v2.1 缺陷）。
 *        稳定确认移至 bsp_key_tick_scan（1ms 周期采样）。
 */
static inline void _record_edge_from_isr(bsp_key_priv_t *priv)
{
    uint8_t raw = _read_raw(priv);
    TickType_t now = xTaskGetTickCountFromISR();

    if (raw != priv->last_raw) {
        priv->last_raw = raw;
        priv->last_change_tick = now;
    }
    /* raw == last_raw 的重触发（EMI 毛刺）：忽略，等待 tick 扫描仲裁 */
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

    bsp_key_priv_t *priv = (bsp_key_priv_t *)s_dev_pool[idx].drv_priv;

    __HAL_GPIO_EXTI_CLEAR_IT(pin);

    /* v2.2：仅记录边沿，事件确认在 tick 扫描（见 _record_edge_from_isr） */
    _record_edge_from_isr(priv);
}

/* ========================================================================== */
/*                      EXTI 中断向量                                           */
/* ========================================================================== */

/**
 * @note  向量守卫按 IRQn **实际值**判断（而非 KEY 槽位序号）：
 *        引脚可映射到任意 EXTI 线，按槽位匹配会把 KEYn 与 EXTIn 错绑。
 *        EXTI9_5 / EXTI15_10 为共享向量，handler 内部按 PR 寄存器分发。
 */
/* ---- 专用向量（EXTI0~4，一线一向量） ---- */
#if (defined(BSP_KEY1_IRQN) && BSP_KEY1_IRQN == EXTI0_IRQn) \
    || (defined(BSP_KEY2_IRQN) && BSP_KEY2_IRQN == EXTI0_IRQn)
void EXTI0_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_0); }
#endif

#if (defined(BSP_KEY1_IRQN) && BSP_KEY1_IRQN == EXTI1_IRQn) \
    || (defined(BSP_KEY2_IRQN) && BSP_KEY2_IRQN == EXTI1_IRQn)
void EXTI1_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_1); }
#endif

#if (defined(BSP_KEY1_IRQN) && BSP_KEY1_IRQN == EXTI2_IRQn) \
    || (defined(BSP_KEY2_IRQN) && BSP_KEY2_IRQN == EXTI2_IRQn)
void EXTI2_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_2); }
#endif

#if (defined(BSP_KEY1_IRQN) && BSP_KEY1_IRQN == EXTI3_IRQn) \
    || (defined(BSP_KEY2_IRQN) && BSP_KEY2_IRQN == EXTI3_IRQn)
void EXTI3_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_3); }
#endif

#if (defined(BSP_KEY1_IRQN) && BSP_KEY1_IRQN == EXTI4_IRQn) \
    || (defined(BSP_KEY2_IRQN) && BSP_KEY2_IRQN == EXTI4_IRQn)
void EXTI4_IRQHandler(void) { _bsp_key_irq_handler(GPIO_PIN_4); }
#endif

/* ---- 共享向量（EXTI9_5 / EXTI15_10：多线共享，按 PR 分发） ---- */
#if (defined(BSP_KEY1_IRQN) && (BSP_KEY1_IRQN == EXTI9_5_IRQn  || BSP_KEY1_IRQN == EXTI15_10_IRQn)) \
    || (defined(BSP_KEY2_IRQN) && (BSP_KEY2_IRQN == EXTI9_5_IRQn  || BSP_KEY2_IRQN == EXTI15_10_IRQn))
/**
 * @brief EXTI 线 5~9 共享中断：逐线查 PR，命中才进消抖处理
 */
void EXTI9_5_IRQHandler(void)
{
    for (uint32_t pin = GPIO_PIN_5; pin <= GPIO_PIN_9; pin <<= 1) {
        if (__HAL_GPIO_EXTI_GET_IT(pin) != RESET) {
            _bsp_key_irq_handler(pin);
        }
    }
}

/**
 * @brief EXTI 线 10~15 共享中断：同上
 */
void EXTI15_10_IRQHandler(void)
{
    for (uint32_t pin = GPIO_PIN_10; pin <= GPIO_PIN_15; pin <<= 1) {
        if (__HAL_GPIO_EXTI_GET_IT(pin) != RESET) {
            _bsp_key_irq_handler(pin);
        }
    }
}
#endif

/* ========================================================================== */
/*                     BSP 公共接口                                              */
/* ========================================================================== */

/**
 * @brief   按键消抖扫描（1ms 周期，SysTick hook 上下文调用）
 * @note    v2.2 核心修复：电平稳定超过 KEY_DEBOUNCE_MS 且与上次确认值
 *          不同时，产生 DOWN/UP 事件（经 dal_key_notify_event -> 用户
 *          回调，回调须为 ISR 安全：本项目 key_cb 仅 FromISR 队列投递）。
 *          EXTI 边沿只重置时间戳，本扫描完成最终确认，干净按键也能出事件。
 *
 * @warning 仅可在 ISR 上下文（tick hook）调用；单次开销 2 键 × 数条指令。
 */
void bsp_key_tick_scan(void)
{
    TickType_t now = xTaskGetTickCountFromISR();

    for (uint32_t i = 0; i < KEY_COUNT; i++) {
        bsp_key_priv_t *priv = (bsp_key_priv_t *)s_dev_pool[i].drv_priv;
        if (!priv->irq_enabled || priv->port == NULL) {
            continue;
        }

        uint8_t raw = _read_raw(priv);

        if (raw != priv->last_raw) {
            /* EXTI 丢失的边沿（如中断被长临界区延迟）：补记时间戳 */
            priv->last_raw = raw;
            priv->last_change_tick = now;
        } else if ((now - priv->last_change_tick) >= KEY_DEBOUNCE_TICKS
                   && raw != priv->stable_level) {
            priv->stable_level = raw;
            dal_key_notify_event(&s_dev_pool[i],
                                 (raw == 1U) ? DAL_KEY_EVT_DOWN
                                             : DAL_KEY_EVT_UP);
        }
    }
}

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

        /* 设备级 init（GPIO 输入 + 初始状态同步，置 initialized=true）。
         * WHY：不调用则 dal_key_set_irq_enable 因 !initialized 静默返回
         * NOT_READY，EXTI 永远配不上（按键无中断的根因，v2.3 修复） */
        ret = dal_key_init(dev);
        if (ret != DAL_OK) {
            return BSP_ERR_FAIL;
        }
    }

    return BSP_OK;
}