/**
 * @file hal_gpio.h
 * @brief GPIO 硬件抽象层接口 — 纯接口头文件，由 BSP 静态实现
 * 
 * @author xserein
 * @version v1.0
 */

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "hal_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ========================================================================== */
/*                               数据结构                                      */
/* ========================================================================== */

/**
 * @brief GPIO 引脚标识符
 * @note  编码方式: 高8位=端口号, 低8位=引脚号
 *        例如: PA5 = (0 << 8) | 5, PB12 = (1 << 8) | 12
 *        具体有效范围由 BSP 决定，HAL 层不做假设
 */
typedef uint16_t hal_gpio_pin_t;

#define HAL_GPIO_PIN_NONE       ((hal_gpio_pin_t)0xFFFF) ///< 无效引脚标记

/**
 * @brief GPIO 方向枚举
 */
typedef enum {
    HAL_GPIO_DIR_INPUT,         ///< 输入模式
    HAL_GPIO_DIR_OUTPUT,        ///< 输出模式
} hal_gpio_dir_t;

/**
 * @brief GPIO 上下拉配置枚举
 */
typedef enum {
    HAL_GPIO_PULL_NONE,         ///< 浮空/无上下拉
    HAL_GPIO_PULL_UP,           ///< 内部上拉
    HAL_GPIO_PULL_DOWN,         ///< 内部下拉
} hal_gpio_pull_t;

/**
 * @brief GPIO 输出速度/驱动能力枚举
 * @note  部分 MCU 不支持速度分级，BSP 可忽略此字段
 */
typedef enum {
    HAL_GPIO_SPEED_LOW,         ///< 低速/低驱动
    HAL_GPIO_SPEED_MEDIUM,      ///< 中速/中驱动
    HAL_GPIO_SPEED_HIGH,        ///< 高速/高驱动
} hal_gpio_speed_t;

/**
 * @brief GPIO 中断触发边沿枚举
 */
typedef enum {
    HAL_GPIO_IRQ_RISING,        ///< 上升沿触发
    HAL_GPIO_IRQ_FALLING,       ///< 下降沿触发
    HAL_GPIO_IRQ_BOTH,          ///< 双边沿触发
} hal_gpio_irq_edge_t;

/**
 * @brief GPIO 中断回调函数原型
 * @param pin       触发中断的引脚
 * @param user_data 用户自定义参数
 * @note  执行上下文: 中断上下文(ISR)。回调应尽可能简短，
 *        避免阻塞操作。如需复杂处理，建议通过信号量/事件标志
 *        通知任务上下文执行。
 */
typedef void (*hal_gpio_irq_callback_t)(hal_gpio_pin_t pin, void *user_data);

/**
 * @brief GPIO 配置参数结构体
 * @note  用于一次性完成引脚的完整配置
 * 
 */
typedef struct {
    hal_gpio_dir_t     direction;    ///< 引脚方向
    hal_gpio_pull_t    pull;         ///< 上下拉配置
    hal_gpio_speed_t   speed;        ///< 输出速度(仅输出模式有效，部分MCU可忽略)
    bool               open_drain;   ///< true=开漏输出, false=推挽输出 (仅在 direction=OUTPUT 时有效，输入模式必须为 false)
    bool               init_state;   ///< 输出模式的初始电平(true=高)
} hal_gpio_config_t;


/* ========================================================================== */
/*                          HAL 公共 API (Public API)                          */
/* ========================================================================== */

/**
 * @brief 初始化 GPIO 子系统全局资源
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_IO 底层时钟使能失败
 * @note   必须在系统启动早期调用一次，内部完成时钟使能等全局准备。
 *         重复调用行为由 BSP 定义，建议返回 HAL_ERR_ALREADY_INIT 或幂等处理。
 */
hal_err_t hal_gpio_init(void);

/**
 * @brief 反初始化 GPIO 子系统，释放全局资源
 * @return HAL_SUCCESS=成功
 * @note   用于低功耗场景进入深度睡眠前关闭 GPIO 时钟。
 *         调用后所有引脚配置失效，需重新 init + configure。
 */
hal_err_t hal_gpio_deinit(void);

/**
 * @brief 配置引脚模式
 * @param pin    目标引脚
 * @param config 配置参数
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_INVALID_PARAM      config 为 NULL 或参数组合非法
 * @retval HAL_GPIO_ERR_INVALID_PIN   引脚编号超出硬件范围
 * @retval HAL_GPIO_ERR_INVALID_MODE  配置组合不合法(如输入模式指定 open_drain)
 */
hal_err_t hal_gpio_configure(hal_gpio_pin_t pin, const hal_gpio_config_t *config);

/**
 * @brief 写输出电平
 * @param pin   目标引脚
 * @param level true=高电平, false=低电平
 * @note   对未配置为输出的引脚调用，行为未定义(BSP 可选择静默忽略或断言)。
 *         此操作对引脚输出寄存器是原子的，可在中断上下文安全调用。
 */
void hal_gpio_write(hal_gpio_pin_t pin, bool level);

/**
 * @brief 读引脚电平
 * @return 当前引脚电平状态
 * @note   输入模式: 读取外部输入电平；输出模式: 尝试读取输出寄存器，
 *        若硬件不支持读取，则返回最近一次 write/toggle 写入的状态（由 BSP 保证一致性）。
 *        无效引脚或未初始化: 返回 false。
 */
bool hal_gpio_read(hal_gpio_pin_t pin);

/**
 * @brief 翻转输出电平
 * @param pin 目标引脚
 * @note   对未配置为输出的引脚调用，行为未定义。
 *         此操作对引脚输出寄存器是原子的，可在中断上下文安全调用。
 */
void hal_gpio_toggle(hal_gpio_pin_t pin);

/**
 * @brief 注册或注销外部中断回调
 * @param pin       目标引脚
 * @param edge      触发边沿
 * @param callback  回调函数指针，传 NULL 表示注销
 * @param user_data 回调时透传的用户参数
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_INVALID_PARAM       引脚无效或边沿参数非法
 * @retval HAL_ERR_NOT_SUPPORTED       该引脚不支持外部中断(无对应 EXTI 线)
 * @retval HAL_GPIO_ERR_IRQ_CONFLICT   EXTI 通道已被其他引脚占用
 * @note   中断优先级由 BSP 默认配置，如需调整请通过 NVIC 接口或 BSP 专用 API。
 *         注销后该引脚中断将被自动禁用，如需重新注册，请重新调用 set_irq 注册回调，
 *         随后调用 enable_irq 显式使能。
 */
hal_err_t hal_gpio_set_irq(hal_gpio_pin_t pin, hal_gpio_irq_edge_t edge,
                           hal_gpio_irq_callback_t callback, void *user_data);

/**
 * @brief 使能或禁用已注册中断的引脚
 * @param pin    目标引脚
 * @param enable true=使能, false=禁用
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_INVALID_PARAM       引脚无效
 * @retval HAL_ERR_NOT_INITIALIZED     全局 GPIO 子系统未初始化 (hal_gpio_init 未调用)
 * @retval HAL_GPIO_ERR_IRQ_NOT_REGISTERED 该引脚未注册中断回调
 */
hal_err_t hal_gpio_enable_irq(hal_gpio_pin_t pin, bool enable);


/* ========================================================================== */
/*                            便捷配置宏                                       */
/* ========================================================================== */

/** @brief 快速构建推挽输出配置 */
#define HAL_GPIO_CONFIG_OUTPUT(state) \
    ((hal_gpio_config_t){             \
        .direction  = HAL_GPIO_DIR_OUTPUT, \
        .pull       = HAL_GPIO_PULL_NONE,  \
        .speed      = HAL_GPIO_SPEED_LOW,  \
        .open_drain = false,               \
        .init_state = (state)              \
    })

/** @brief 快速构建开漏输出配置(如 I2C SDA/SCL 软件模拟) */
#define HAL_GPIO_CONFIG_OUTPUT_OD(state) \
    ((hal_gpio_config_t){                \
        .direction  = HAL_GPIO_DIR_OUTPUT, \
        .pull       = HAL_GPIO_PULL_NONE,  \
        .speed      = HAL_GPIO_SPEED_LOW,  \
        .open_drain = true,                \
        .init_state = (state)              \
    })

/** @brief 快速构建浮空输入配置 */
#define HAL_GPIO_CONFIG_INPUT_FLOATING \
    ((hal_gpio_config_t){              \
        .direction  = HAL_GPIO_DIR_INPUT, \
        .pull       = HAL_GPIO_PULL_NONE, \
        .speed      = HAL_GPIO_SPEED_LOW,  \
        .open_drain = false,               \
        .init_state = false                \
    })

/** @brief 快速构建上拉输入配置 */
#define HAL_GPIO_CONFIG_INPUT_PULLUP \
    ((hal_gpio_config_t){            \
        .direction  = HAL_GPIO_DIR_INPUT, \
        .pull       = HAL_GPIO_PULL_UP,   \
        .speed      = HAL_GPIO_SPEED_LOW,  \
        .open_drain = false,               \
        .init_state = false                \
    })

/** @brief 快速构建下拉输入配置 */
#define HAL_GPIO_CONFIG_INPUT_PULLDOWN \
    ((hal_gpio_config_t){              \
        .direction  = HAL_GPIO_DIR_INPUT, \
        .pull       = HAL_GPIO_PULL_DOWN, \
        .speed      = HAL_GPIO_SPEED_LOW,  \
        .open_drain = false,               \
        .init_state = false                \
    })


#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */