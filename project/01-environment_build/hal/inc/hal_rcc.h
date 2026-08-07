/**
 * @file hal_rcc.h
 * @brief RCC 硬件抽象层接口 — 系统时钟与外设时钟管理
 * 
 * @author xserein
 * @version v1.0
 */

#ifndef HAL_RCC_H
#define HAL_RCC_H

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
 * @brief 系统总线枚举
 * @note  覆盖 Cortex-M 系列常见总线拓扑。BSP 对不支持的枚举返回 0。
 */
typedef enum {
    HAL_RCC_BUS_AHB,        ///< AHB 总线（ARM 内核、DMA、Flash）
    HAL_RCC_BUS_AHB2,       ///< AHB2 总线（F4/F7/H7 的 GPIO、USB OTG、DCMI 等）
    HAL_RCC_BUS_AHB3,       ///< AHB3 总线（F7/H7 的 FMC、QSPI、SDMMC 等）
    HAL_RCC_BUS_APB1,       ///< APB1 低速外设总线
    HAL_RCC_BUS_APB2,       ///< APB2 高速外设总线
    HAL_RCC_BUS_APB3,       ///< APB3 / AXI 总线（H7 系列部分外设）
} hal_rcc_bus_t;

/**
 * @brief 系统时钟源枚举
 */
typedef enum {
    HAL_RCC_SRC_HSI,        ///< 内部高速 RC 振荡器（通常 8/16MHz）
    HAL_RCC_SRC_HSE,        ///< 外部高速晶振（通常 4~50MHz）
    HAL_RCC_SRC_PLL,        ///< PLL 倍频输出
    HAL_RCC_SRC_LSI,        ///< 内部低速 RC（~40kHz，供 IWDG/RTC）
    HAL_RCC_SRC_LSE,        ///< 外部低速晶振（32.768kHz，供 RTC）
} hal_rcc_source_t;

/**
 * @brief 系统时钟配置结构体
 * @note  应用层只需关心最终频率，具体 PLL 系数、分频比由 BSP 层确定。
 *        字段为 0 的含义取决于调用上下文：
 *        - hal_rcc_init():            0 = 使用 BSP 编译期默认值
 *        - hal_rcc_enter_low_power(): 0 = 无效值，必须填写完整配置
 *        sysclk_src 和 sysclk_hz 在所有上下文中均为必填项。
 */
typedef struct {
    hal_rcc_source_t sysclk_src;    ///< 系统时钟来源
    uint32_t         sysclk_hz;     ///< 期望的系统时钟频率（Hz），0=使用默认
    uint32_t         ahb_hz;        ///< AHB 总线频率（Hz），0=使用默认
    uint32_t         apb1_hz;       ///< APB1 总线频率（Hz），0=使用默认
    uint32_t         apb2_hz;       ///< APB2 总线频率（Hz），0=使用默认
} hal_rcc_config_t;

/**
 * @brief 外设标识符（用于时钟使能/禁用/复位/频率查询/状态查询）
 * @note  具体编码由 BSP 完全自定义，HAL 层不假设任何编码规则。
 *        强烈建议 BSP 使用内部查表法实现映射，而非位域编码。
 */
typedef uint16_t hal_rcc_periph_t;

#define HAL_RCC_PERIPH_NONE   ((hal_rcc_periph_t)0xFFFF) ///< 无效/空外设标识符


/* ========================================================================== */
/*                          HAL 公共 API                                      */
/* ========================================================================== */

/**
 * @brief 初始化系统时钟
 * @param config  时钟配置参数。若传入 NULL，BSP 使用编译期默认配置。
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_INVALID_PARAM      配置参数超出硬件能力
 * @retval HAL_ERR_TIMEOUT            外部晶振起振超时
 * @retval HAL_ERR_ALREADY_INIT       重复初始化
 * @note  必须在使用任何外设前调用一次
 */
hal_err_t hal_rcc_init(const hal_rcc_config_t *config);

/**
 * @brief 反初始化系统时钟，恢复到复位默认状态（HSI）
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_NOT_INITIALIZED    模块未初始化
 * @note  - 用于 Bootloader 跳转前或深度低功耗进入前恢复时钟默认态
 *        - 调用后系统时钟恢复到 HSI，但外设时钟使能寄存器不会被清零，
 *          各外设驱动需自行管理时钟开关
 */
hal_err_t hal_rcc_deinit(void);

/**
 * @brief 使能或禁用外设时钟
 * @param periph  外设标识符
 * @param enable  true=使能，false=禁用
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_INVALID_PARAM      外设标识符无效（含 HAL_RCC_PERIPH_NONE）
 * @note  对同一外设重复使能无害（幂等）
 */
hal_err_t hal_rcc_periph_enable(hal_rcc_periph_t periph, bool enable);

/**
 * @brief 查询外设时钟是否已使能
 * @param periph  外设标识符
 * @return true=已使能，false=未使能或标识符无效
 */
bool hal_rcc_periph_is_enabled(hal_rcc_periph_t periph);

/**
 * @brief 获取指定总线的当前时钟频率
 * @param bus  总线枚举
 * @return uint32_t  频率值（Hz）。若总线无效或未初始化，返回 0。
 */
uint32_t hal_rcc_get_freq(hal_rcc_bus_t bus);

/**
 * @brief 获取指定外设的当前工作时钟频率
 * @param periph  外设标识符
 * @return uint32_t  频率值（Hz）。若外设无效或未初始化，返回 0。
 * @note  实现提示：建议 BSP 在 hal_rcc_init 时预计算所有外设频率并缓存，
 *        避免运行时反复读取 RCC_CFGR 和计算分频。
 */
uint32_t hal_rcc_get_periph_freq(hal_rcc_periph_t periph);

/**
 * @brief 获取当前系统时钟源
 * @return hal_rcc_source_t  当前生效的时钟源。
 *         若模块未初始化，固定返回 HAL_RCC_SRC_HSI。
 */
hal_rcc_source_t hal_rcc_get_source(void);

/**
 * @brief 软件复位指定外设
 * @param periph  外设标识符
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_INVALID_PARAM      外设标识符无效
 * @note  复位后外设寄存器恢复默认值，需重新初始化；时钟使能状态保持不变
 */
hal_err_t hal_rcc_reset(hal_rcc_periph_t periph);

/**
 * @brief 进入低功耗时钟模式
 * @param config  目标时钟配置。若传入 NULL，BSP 使用内部默认低功耗配置。
 *                若传入非 NULL，必须提供完整有效配置（所有字段非 0）。
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_INVALID_PARAM                目标频率/时钟源超出范围
 * @retval HAL_ERR_TIMEOUT                      时钟切换超时
 * @retval HAL_RCC_ERR_LOW_POWER_UNSUPPORTED    当前硬件不支持所请求的低功耗模式
 * @note  与 hal_rcc_exit_low_power 成对使用
 */
hal_err_t hal_rcc_enter_low_power(const hal_rcc_config_t *config);

/**
 * @brief 退出低功耗时钟模式，恢复到 hal_rcc_init 时的全速配置
 * @return HAL_SUCCESS=成功
 * @retval HAL_ERR_NOT_INITIALIZED    模块未初始化
 * @retval HAL_ERR_TIMEOUT            外部晶振/PLL 锁定超时
 */
hal_err_t hal_rcc_exit_low_power(void);


/* ========================================================================== */
/*                            便捷配置宏                                       */
/* ========================================================================== */

/**
 * @brief 快速构建使用 HSE 作为 PLL 输入、PLL 输出系统时钟的配置模板
 * @note  具体频率值由应用层填充，BSP 负责计算合法的分频系数。
 *        例如：HAL_RCC_CONFIG_PLL_HSE(72000000U, 36000000U, 72000000U)
 */
#define HAL_RCC_CONFIG_PLL_HSE(sysclk, apb1, apb2) \
    ((hal_rcc_config_t){                           \
        .sysclk_src = HAL_RCC_SRC_PLL,             \
        .sysclk_hz  = (sysclk),                    \
        .ahb_hz     = (sysclk),                    \
        .apb1_hz    = (apb1),                      \
        .apb2_hz    = (apb2)                       \
    })

/**
 * @brief 快速构建使用 HSI 直驱的配置模板（无 PLL）
 * @note  适用于低功耗或无需高精度的场景
 */
#define HAL_RCC_CONFIG_HSI_ONLY(sysclk) \
    ((hal_rcc_config_t){                \
        .sysclk_src = HAL_RCC_SRC_HSI,  \
        .sysclk_hz  = (sysclk),         \
        .ahb_hz     = (sysclk),         \
        .apb1_hz    = (sysclk),         \
        .apb2_hz    = (sysclk)          \
    })


#ifdef __cplusplus
}
#endif

#endif /* HAL_RCC_H */