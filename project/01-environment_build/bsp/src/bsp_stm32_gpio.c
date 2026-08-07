/**
 * @file bsp_stm32_gpio.c
 * @brief GPIO HAL 接口的 STM32F1 BSP 静态实现
 * 
 * @details STM32F1 平台关键适配点:
 *          - EXTI 复用通过 AFIO_EXTICR 配置 (非 SYSCFG)
 *          - 输入上下拉由 ODR + CNF=10 组合决定，需显式预置 ODR
 *          - GPIO 模式/速度通过 CRL/CRH 寄存器配置
 *          - BSRR 寄存器实现原子置位/复位，避免读-改-写竞争
 *          - EXTI5-9 / EXTI10-15 为共享 NVIC 通道，注销时需引用计数检查
 * 
 * @author xserein
 * @version v1.0
 */

#include "hal_gpio.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ========================================================================== */
/*                           内部宏与常量定义                                    */
/* ========================================================================== */

#define BSP_GPIO_PIN_MAX        16U             /**< 单端口最大引脚数 (0~15) */
#define BSP_EXTI_LINE_COUNT     16U             /**< EXTI 线总数 (0~15) */

/** 从引脚标识符中提取端口索引（高8位） */
#define PIN_GET_PORT_IDX(pin)   ((uint8_t)((pin) >> 8))
/** 从引脚标识符中提取引脚编号（低8位） */
#define PIN_GET_PIN_NUM(pin)    ((uint8_t)((pin) & 0xFFU))
/** 由端口索引和引脚编号重新构造引脚标识符 */
#define PIN_MAKE(port_idx, pin_num) \
    ((hal_gpio_pin_t)(((uint16_t)(port_idx) << 8) | ((uint16_t)(pin_num) & 0xFFU)))

/** 
 * @brief 临界区保护宏：保存 PRIMASK 状态并关闭全局中断，退出时恢复
 * @note  用于保护 `exti_table` 及 EXTI 寄存器操作，防止主程序与中断上下文冲突
 */
#define CRITICAL_ENTER()        uint32_t _primask = __get_PRIMASK(); __disable_irq()
#define CRITICAL_EXIT()         __set_PRIMASK(_primask)


/* ========================================================================== */
/*                            内部数据结构                                      */
/* ========================================================================== */

/**
 * @brief EXTI 线回调表条目
 */
typedef struct {
    hal_gpio_irq_callback_t callback;   /**< 用户注册的回调函数 */
    void                   *user_data;  /**< 用户透传参数 */
    volatile bool           registered; /**< 该 EXTI 线是否已注册（ISR 读取，主程序写入） */
} exti_cb_entry_t;

/**
 * @brief BSP GPIO 全局状态
 */
static struct {
    bool             initialized;                        /**< 模块初始化标志 */
    exti_cb_entry_t  exti_table[BSP_EXTI_LINE_COUNT];   /**< EXTI 回调表 */
} g_gpio = {0};


/* ========================================================================== */
/*                         内部辅助函数(私有)                                    */
/* ========================================================================== */

/**
 * @brief 将端口索引转换为 STM32 GPIO 基地址指针
 * @param port_idx  端口索引（0 = GPIOA, 1 = GPIOB, ...）
 * @return GPIO_TypeDef*  对应的 GPIO 寄存器基地址，若索引无效则返回 NULL
 * @note  依赖预编译宏判断硬件是否支持该端口，不支持的端口会直接返回 NULL
 */
static inline GPIO_TypeDef *port_idx_to_base(uint8_t port_idx)
{
    switch (port_idx) {
#if defined(GPIOA)
        case 0: return GPIOA;
#endif
#if defined(GPIOB)
        case 1: return GPIOB;
#endif
#if defined(GPIOC)
        case 2: return GPIOC;
#endif
#if defined(GPIOD)
        case 3: return GPIOD;
#endif
#if defined(GPIOE)
        case 4: return GPIOE;
#endif
#if defined(GPIOF)
        case 5: return GPIOF;
#endif
#if defined(GPIOG)
        case 6: return GPIOG;
#endif
        default: return NULL;
    }
}

/**
 * @brief 使能指定端口的外设时钟
 * @param port_idx  端口索引（0 = GPIOA, 1 = GPIOB, ...）
 * @note  使用 __HAL_RCC_GPIOx_CLK_ENABLE() 宏，该操作幂等，重复调用无害
 */
static inline void enable_port_clock(uint8_t port_idx)
{
    switch (port_idx) {
#if defined(__HAL_RCC_GPIOA_CLK_ENABLE)
        case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
#endif
#if defined(__HAL_RCC_GPIOB_CLK_ENABLE)
        case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
#endif
#if defined(__HAL_RCC_GPIOC_CLK_ENABLE)
        case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
#endif
#if defined(__HAL_RCC_GPIOD_CLK_ENABLE)
        case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
#endif
#if defined(__HAL_RCC_GPIOE_CLK_ENABLE)
        case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
#endif
#if defined(__HAL_RCC_GPIOF_CLK_ENABLE)
        case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
#endif
#if defined(__HAL_RCC_GPIOG_CLK_ENABLE)
        case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
#endif
        default: break;
    }
}

/**
 * @brief 将 HAL GPIO 速度枚举映射到 STM32 速度宏
 * @param speed   HAL 速度枚举
 * @return uint32_t  STM32 速度宏（GPIO_SPEED_FREQ_LOW/MEDIUM/HIGH）
 * @note  F1 仅有 2MHz、10MHz、50MHz 三档，分别对应 LOW/MEDIUM/HIGH
 */
static inline uint32_t speed_to_stm32(hal_gpio_speed_t speed)
{
    switch (speed) {
        case HAL_GPIO_SPEED_LOW:    return GPIO_SPEED_FREQ_LOW;      /* 2MHz */
        case HAL_GPIO_SPEED_MEDIUM: return GPIO_SPEED_FREQ_MEDIUM;   /* 10MHz */
        case HAL_GPIO_SPEED_HIGH:   return GPIO_SPEED_FREQ_HIGH;     /* 50MHz */
        default:                    return GPIO_SPEED_FREQ_LOW;
    }
}

/**
 * @brief 将 HAL GPIO 上下拉枚举映射到 STM32 上下拉宏
 * @param pull   HAL 上下拉枚举
 * @return uint32_t  STM32 上下拉宏（GPIO_PULLUP / GPIO_PULLDOWN / GPIO_NOPULL）
 */
static inline uint32_t pull_to_stm32(hal_gpio_pull_t pull)
{
    switch (pull) {
        case HAL_GPIO_PULL_UP:      return GPIO_PULLUP;
        case HAL_GPIO_PULL_DOWN:    return GPIO_PULLDOWN;
        case HAL_GPIO_PULL_NONE:
        default:                    return GPIO_NOPULL;
    }
}

/**
 * @brief 检查引脚标识符是否有效
 * @param pin   待检查的引脚标识符
 * @return true  引脚有效（端口存在且引脚编号 < 16）
 * @return false 引脚无效（为 HAL_GPIO_PIN_NONE 或端口/编号超出范围）
 */
static inline bool is_valid_pin(hal_gpio_pin_t pin)
{
    if (pin == HAL_GPIO_PIN_NONE) {
        return false;
    }
    uint8_t port_idx = PIN_GET_PORT_IDX(pin);
    uint8_t pin_num  = PIN_GET_PIN_NUM(pin);
    return (port_idx_to_base(port_idx) != NULL) && (pin_num < BSP_GPIO_PIN_MAX);
}

/**
 * @brief 将端口索引转换为 AFIO_EXTICR 寄存器所需的编码值
 * @param port_idx  端口索引（0 = PA, 1 = PB, ...）
 * @return uint8_t  AFIO_EXTICR 编码值（与端口索引一致）
 * @note  STM32F1 中 AFIO_EXTICR 编码规则为：0x0=PA, 0x1=PB, ..., 0x6=PG，
 *        与端口索引完全一致，故直接返回原值。
 */
static inline uint8_t port_idx_to_afio_code(uint8_t port_idx)
{
    return port_idx;
}

/**
 * @brief 配置或禁用 EXTI 线对应的 NVIC 中断
 * @param pin_num   EXTI 线编号 (0~15)
 * @param enable    true=使能中断，false=禁用中断
 * @note  - EXTI9_5 和 EXTI15_10 为共享中断通道，重复使能无副作用。
 *        - 禁用共享通道时需检查同组其他引脚是否仍在使用，本函数自身不处理引用计数，
 *          仅根据 enable 参数直接操作 NVIC。
 *        - 中断优先级固定为 15（最低），应用层可通过 HAL_NVIC_SetPriority 重新配置。
 */
static void exti_nvic_config(uint8_t pin_num, bool enable)
{
    IRQn_Type irqn;
    
    if (pin_num <= 4) {
        irqn = (IRQn_Type)(EXTI0_IRQn + pin_num);
    } else if (pin_num <= 9) {
        irqn = EXTI9_5_IRQn;
    } else if (pin_num <= 15) {
        irqn = EXTI15_10_IRQn;
    } else {
        return;
    }
    
    if (enable) {
        HAL_NVIC_SetPriority(irqn, 15, 0);  /* 默认最低优先级，APP 可后续调整 */
        HAL_NVIC_EnableIRQ(irqn);
    } else {
        HAL_NVIC_DisableIRQ(irqn);
    }
}


/* ========================================================================== */
/*                     HAL 公共 API 实现 (Public API)                           */
/* ========================================================================== */

hal_err_t hal_gpio_init(void)
{
    if (g_gpio.initialized) {
        return HAL_ERR_ALREADY_INIT;
    }
    
    memset(&g_gpio.exti_table, 0, sizeof(g_gpio.exti_table));
    __HAL_RCC_AFIO_CLK_ENABLE();
    
    g_gpio.initialized = true;
    return HAL_SUCCESS;
}

hal_err_t hal_gpio_deinit(void)
{
    if (!g_gpio.initialized) {
        return HAL_ERR_NOT_INITIALIZED;
    }
    
    CRITICAL_ENTER();
    
    /* 关闭所有 EXTI 中断线路和事件线路 */
    EXTI->IMR = 0;
    EXTI->EMR = 0;
    
    /* 禁用所有 NVIC EXTI 通道 */
    for (uint8_t i = 0; i < BSP_EXTI_LINE_COUNT; i++) {
        if (g_gpio.exti_table[i].registered) {
            exti_nvic_config(i, false);
        }
    }
    
    memset(&g_gpio.exti_table, 0, sizeof(g_gpio.exti_table));
    
    __HAL_RCC_AFIO_CLK_DISABLE();
    
    g_gpio.initialized = false;
    CRITICAL_EXIT();
    
    return HAL_SUCCESS;
}

hal_err_t hal_gpio_configure(hal_gpio_pin_t pin, const hal_gpio_config_t *config)
{
    if (!g_gpio.initialized) {
        return HAL_ERR_NOT_INITIALIZED;
    }
    if (config == NULL) {
        return HAL_ERR_INVALID_PARAM;
    }
    if (!is_valid_pin(pin)) {
        return HAL_GPIO_ERR_INVALID_PIN;
    }
    if ((config->direction == HAL_GPIO_DIR_INPUT) && config->open_drain) {
        return HAL_GPIO_ERR_INVALID_MODE;
    }
    
    uint8_t port_idx = PIN_GET_PORT_IDX(pin);
    uint8_t pin_num  = PIN_GET_PIN_NUM(pin);
    GPIO_TypeDef *gpio = port_idx_to_base(port_idx);
    
    enable_port_clock(port_idx);
    
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin   = (uint16_t)(1U << pin_num);
    gpio_init.Speed = speed_to_stm32(config->speed);
    gpio_init.Pull  = pull_to_stm32(config->pull);
    
    if (config->direction == HAL_GPIO_DIR_OUTPUT) {
        gpio_init.Mode = config->open_drain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
        /* 先设置初始电平再初始化，避免配置过程中的毛刺 */
        if (config->init_state) {
            gpio->BSRR = (uint32_t)(1U << pin_num);
        } else {
            gpio->BSRR = (uint32_t)(1U << (pin_num + 16U));
        }
    } else {
        gpio_init.Mode = GPIO_MODE_INPUT;
        /* F1 的输入上下拉通过 ODR 寄存器配合实现。
           兼容所有 HAL 版本：显式预置 ODR，确保 Pull 生效 */
        if (config->pull == HAL_GPIO_PULL_UP) {
            gpio->BSRR = (uint32_t)(1U << pin_num);
        } else if (config->pull == HAL_GPIO_PULL_DOWN) {
            gpio->BRR  = (uint32_t)(1U << pin_num);
        }
    }
    
    HAL_GPIO_Init(gpio, &gpio_init);
    return HAL_SUCCESS;
}

void hal_gpio_write(hal_gpio_pin_t pin, bool level)
{
    if (!is_valid_pin(pin)) {
        return;
    }
    
    uint8_t port_idx = PIN_GET_PORT_IDX(pin);
    uint8_t pin_num  = PIN_GET_PIN_NUM(pin);
    GPIO_TypeDef *gpio = port_idx_to_base(port_idx);
    
    if (level) {
        gpio->BSRR = (uint32_t)(1U << pin_num);
    } else {
        gpio->BSRR = (uint32_t)(1U << (pin_num + 16U));
    }
}

bool hal_gpio_read(hal_gpio_pin_t pin)
{
    if (!is_valid_pin(pin)) {
        return false;
    }
    
    uint8_t port_idx = PIN_GET_PORT_IDX(pin);
    uint8_t pin_num  = PIN_GET_PIN_NUM(pin);
    GPIO_TypeDef *gpio = port_idx_to_base(port_idx);
    
    /* F1: CRL/CRH 每引脚 4 位，低 2 位为 MODE。
       MODE=00 输入，其他为输出 */
    uint32_t cr_reg = (pin_num < 8) ? gpio->CRL : gpio->CRH;
    uint32_t cr_pos = (pin_num % 8) * 4;
    uint32_t mode   = (cr_reg >> cr_pos) & 0x03U;
    
    if (mode == 0U) {
        /* 输入模式：读 IDR（外部实际电平） */
        return (gpio->IDR & (1U << pin_num)) != 0U;
    } else {
        /* 输出模式：读 ODR（软件写入值） */
        return (gpio->ODR & (1U << pin_num)) != 0U;
    }
}

void hal_gpio_toggle(hal_gpio_pin_t pin)
{
    if (!is_valid_pin(pin)) {
        return;
    }
    
    uint8_t port_idx = PIN_GET_PORT_IDX(pin);
    uint8_t pin_num  = PIN_GET_PIN_NUM(pin);
    GPIO_TypeDef *gpio = port_idx_to_base(port_idx);
    
    /* 通过 BSRR 实现伪原子翻转，避免 ODR 读-改-写竞争。
       单核场景下，只要中断不操作同一引脚即安全 */
    if (gpio->ODR & (1U << pin_num)) {
        gpio->BSRR = (uint32_t)(1U << (pin_num + 16U)); /* Reset */
    } else {
        gpio->BSRR = (uint32_t)(1U << pin_num);         /* Set */
    }
}

hal_err_t hal_gpio_set_irq(hal_gpio_pin_t pin, hal_gpio_irq_edge_t edge,
                           hal_gpio_irq_callback_t callback, void *user_data)
{
    if (!g_gpio.initialized) {
        return HAL_ERR_NOT_INITIALIZED;
    }
    if (!is_valid_pin(pin)) {
        return HAL_ERR_INVALID_PARAM;
    }
    
    uint8_t pin_num  = PIN_GET_PIN_NUM(pin);
    uint8_t port_idx = PIN_GET_PORT_IDX(pin);
    uint8_t exticr_idx = pin_num / 4U;
    uint8_t exticr_pos = (pin_num % 4U) * 4U;
    
    /* --- 注销分支 --- */
    if (callback == NULL) {
        if (!g_gpio.exti_table[pin_num].registered) {
            return HAL_SUCCESS;
        }
        
        CRITICAL_ENTER();
        EXTI->IMR &= ~(1U << pin_num);
        AFIO->EXTICR[exticr_idx] &= ~(0x0FU << exticr_pos);
        g_gpio.exti_table[pin_num].callback   = NULL;
        g_gpio.exti_table[pin_num].user_data  = NULL;
        g_gpio.exti_table[pin_num].registered = false;
        
        /* 禁用 NVIC：独立通道直接关；共享通道需检查同组是否还有活跃引脚 */
        if (pin_num <= 4) {
            exti_nvic_config(pin_num, false);
        } else if (pin_num <= 9) {
            bool other_active = false;
            for (uint8_t i = 5; i <= 9; i++) {
                if (g_gpio.exti_table[i].registered) { other_active = true; break; }
            }
            if (!other_active) exti_nvic_config(pin_num, false);
        } else {
            bool other_active = false;
            for (uint8_t i = 10; i <= 15; i++) {
                if (g_gpio.exti_table[i].registered) { other_active = true; break; }
            }
            if (!other_active) exti_nvic_config(pin_num, false);
        }
        
        CRITICAL_EXIT();
        return HAL_SUCCESS;
    }
    
    /* --- 注册分支：检查 EXTI 线是否已被其他端口占用 --- */
    uint8_t current_port = (uint8_t)((AFIO->EXTICR[exticr_idx] >> exticr_pos) & 0x0FU);
    uint8_t target_port  = port_idx_to_afio_code(port_idx);
    
    if (g_gpio.exti_table[pin_num].registered && (current_port != target_port)) {
        return HAL_GPIO_ERR_IRQ_CONFLICT;
    }
    
    CRITICAL_ENTER();
    
    /* 配置 AFIO EXTI 复用 */
    AFIO->EXTICR[exticr_idx] &= ~(0x0FU << exticr_pos);
    AFIO->EXTICR[exticr_idx] |= ((uint32_t)target_port << exticr_pos);
    
    /* 配置触发边沿 */
    switch (edge) {
        case HAL_GPIO_IRQ_RISING:
            EXTI->RTSR |=  (1U << pin_num);
            EXTI->FTSR &= ~(1U << pin_num);
            break;
        case HAL_GPIO_IRQ_FALLING:
            EXTI->RTSR &= ~(1U << pin_num);
            EXTI->FTSR |=  (1U << pin_num);
            break;
        case HAL_GPIO_IRQ_BOTH:
            EXTI->RTSR |= (1U << pin_num);
            EXTI->FTSR |= (1U << pin_num);
            break;
        default:
            CRITICAL_EXIT();
            return HAL_ERR_INVALID_PARAM;
    }
    
    /* 清除可能残留的挂起标志，避免注册后立即触发旧中断 */
    EXTI->PR = (1U << pin_num);
    
    /* 注册回调并使能中断 */
    g_gpio.exti_table[pin_num].callback   = callback;
    g_gpio.exti_table[pin_num].user_data  = user_data;
    g_gpio.exti_table[pin_num].registered = true;
    
    EXTI->IMR |= (1U << pin_num);
    exti_nvic_config(pin_num, true);
    
    CRITICAL_EXIT();
    return HAL_SUCCESS;
}

hal_err_t hal_gpio_enable_irq(hal_gpio_pin_t pin, bool enable)
{
    if (!g_gpio.initialized) {
        return HAL_ERR_NOT_INITIALIZED;
    }
    if (!is_valid_pin(pin)) {
        return HAL_ERR_INVALID_PARAM;
    }
    
    uint8_t pin_num = PIN_GET_PIN_NUM(pin);
    
    if (!g_gpio.exti_table[pin_num].registered) {
        return HAL_GPIO_ERR_IRQ_NOT_REGISTERED;
    }
    
    CRITICAL_ENTER();
    if (enable) {
        EXTI->IMR |= (1U << pin_num);
        exti_nvic_config(pin_num, true);
    } else {
        EXTI->IMR &= ~(1U << pin_num);
        /* 注意：不关闭 NVIC，因为 EXTI9_5 / EXTI15_10 为共享通道 */
    }
    CRITICAL_EXIT();
    
    return HAL_SUCCESS;
}


/* ========================================================================== */
/*                            EXTI 中断服务入口                                 */
/* ========================================================================== */

/**
 * @brief EXTI 中断处理核心逻辑
 * @param pin_num EXTI 线编号 (0~15)
 * @note  - 该函数在中断上下文执行，必须保持精简，避免阻塞。
 *        - 清除挂起标志后，从 AFIO_EXTICR 反查当前绑定的端口，重建完整引脚标识，
 *          并调用用户注册的回调函数。
 *        - 若回调函数需要在任务上下文执行，用户应自行使用信号量/事件标志通知任务。
 */
static void exti_isr_handler(uint8_t pin_num)
{
    if (pin_num >= BSP_EXTI_LINE_COUNT) {
        return;
    }

    uint32_t pr_mask = (1U << pin_num);
    if ((EXTI->PR & pr_mask) == 0U) {
        return;
    }

    /* 清除挂起标志 (写1清除) */
    EXTI->PR = pr_mask;

    /* 执行用户回调 */
    exti_cb_entry_t *entry = &g_gpio.exti_table[pin_num];
    if (entry->registered && entry->callback != NULL) {
        /* 从 AFIO_EXTICR 反查端口号以重建完整 pin 标识 */
        uint8_t exticr_idx = pin_num / 4U;
        uint8_t exticr_pos = (pin_num % 4U) * 4U;
        uint8_t port_idx = (uint8_t)((AFIO->EXTICR[exticr_idx] >> exticr_pos) & 0x0FU);

        hal_gpio_pin_t pin = PIN_MAKE(port_idx, pin_num);
        entry->callback(pin, entry->user_data);
    }
}


/* --- STM32F1 EXTI 中断向量入口 --- */

void EXTI0_IRQHandler(void)
{
    exti_isr_handler(0);
}

void EXTI1_IRQHandler(void)
{
    exti_isr_handler(1);
}

void EXTI2_IRQHandler(void)
{
    exti_isr_handler(2);
}

void EXTI3_IRQHandler(void)
{
    exti_isr_handler(3);
}

void EXTI4_IRQHandler(void)
{
    exti_isr_handler(4);
}

void EXTI9_5_IRQHandler(void)
{
    for (uint8_t i = 5U; i <= 9U; i++) {
        exti_isr_handler(i);
    }
}

void EXTI15_10_IRQHandler(void)
{
    for (uint8_t i = 10U; i <= 15U; i++) {
        exti_isr_handler(i);
    }
}