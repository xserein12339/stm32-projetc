/**
 * @file    bsp_timer.h
 * @brief   BSP Timer 精简驱动 — 仅支持 PWM 与编码器模式
 *
 * @details 支持多定时器外设独立管理，提供 PWM 输出和正交编码器计数。
 *          不包含周期定时、单次定时、输入捕获等无关功能。
 *          本驱动面向 STM32F1 系列 TIM1~TIM4，不依赖 FreeRTOS 等 RTOS。
 *
 * @author  xserein
 * @version v3.0
 */

#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "bsp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                               常量定义                                       */
/* ========================================================================== */

/**
 * @brief 频率默认值标记
 * @note  当 bsp_timer_config_t::freq_hz 设置为该值时，表示使用硬件默认时钟源，
 *        具体默认频率由芯片适配层决定（通常为 1kHz 或 APB 时钟分频后的值）。
 *        若用户明确指定频率，则传入具体数值（如 1000、5000 等）。
 */
#define BSP_TIMER_FREQ_DEFAULT  0U

/* ========================================================================== */
/*                               类型前向声明                                   */
/* ========================================================================== */

/** @brief Timer 设备句柄（不透明指针），由 bsp_timer_open 返回，后续所有操作均通过此句柄进行 */
typedef struct bsp_timer_dev *bsp_timer_handle_t;

/* ========================================================================== */
/*                             枚举定义                                         */
/* ========================================================================== */

/**
 * @brief 定时器工作模式
 * @note  本驱动仅支持以下两种模式，用于特定硬件场景：
 *        - PWM 模式：适用于电机调速、LED 调光、音频生成等占空比控制场景
 *        - 编码器模式：适用于正交编码器（增量式）的位置/速度测量
 */
typedef enum {
    BSP_TIMER_MODE_PWM,          ///< PWM 输出模式：生成可调占空比的方波信号
    BSP_TIMER_MODE_ENCODER,      ///< 正交编码器模式：计数 A/B 相信号，支持 X1/X2/X4 倍频
} bsp_timer_mode_t;

/**
 * @brief PWM 对齐方式
 * @note  影响 PWM 波形的中心位置，对于电机控制建议使用边沿对齐以降低计算复杂度。
 *        中心对齐模式适用于需要更高精度的应用（如音频合成），但最大频率会减半。
 */
typedef enum {
    BSP_TIMER_PWM_EDGE_ALIGNED = 0,   ///< 边沿对齐：计数器从 0 向上计数到 ARR，到达 ARR 时重置
    BSP_TIMER_PWM_CENTER_ALIGNED,     ///< 中心对齐：计数器先向上计数到 ARR，再向下计数到 0，一个完整周期为 2×ARR
} bsp_timer_pwm_align_t;

/**
 * @brief PWM 输出极性
 * @note  决定有效电平（占空比对应的高电平或低电平）。
 *        对于电机驱动，通常使用 ACTIVE_HIGH 使占空比与转速正相关。
 */
typedef enum {
    BSP_TIMER_PWM_ACTIVE_HIGH = 0,    ///< 高电平有效：占空比越高，高电平持续时间越长
    BSP_TIMER_PWM_ACTIVE_LOW,         ///< 低电平有效：占空比越高，低电平持续时间越长（相位反转）
} bsp_timer_pwm_polarity_t;

/**
 * @brief 编码器计数模式（倍频）
 * @note  倍频系数影响分辨率与最大计数频率的权衡：
 *        - X1：仅 A 相上升沿计数，分辨率最低，功耗最低
 *        - X2：A 相和 B 相的上升沿计数，分辨率翻倍
 *        - X4：A 相和 B 相的上升沿和下降沿计数，分辨率最高，计数频率也最高
 * @warning STM32F1 硬件仅支持 X1 和 X4 两种模式，X2 模式会被降级映射为 X4，
 *          如需真正的 X2 模式请在软件层对 X4 计数值除以 2。
 */
typedef enum {
    BSP_TIMER_ENC_MODE_X1 = 0,        ///< 1 倍频：仅 A 相单边沿计数
    BSP_TIMER_ENC_MODE_X2,            ///< 2 倍频：A/B 双相单边沿计数（STM32F1 降级为 X4）
    BSP_TIMER_ENC_MODE_X4,            ///< 4 倍频：A/B 双相双边沿计数（最高分辨率）
} bsp_timer_enc_mode_t;

/**
 * @brief Timer 异步事件枚举
 * @note  事件由中断服务程序触发，通过回调函数通知上层应用。
 *        事件按位定义，支持按位与判断，方便同时处理多个事件。
 */
typedef enum {
    BSP_TIMER_EVT_ERROR    = 0x04U,   ///< 硬件错误（时钟源失效、PSC/ARR 配置异常等）
    BSP_TIMER_EVT_ABORT    = 0x08U,   ///< 主动中止：由 bsp_timer_stop() 主动触发，不表示故障
} bsp_timer_event_t;

/**
 * @brief Timer 错误详情
 * @note  当事件类型为 EVT_ERROR 时有效，上层可根据具体标志位排查故障原因。
 */
typedef struct {
    bool clock_fail;  ///< 时钟源失效标志：置位表示定时器输入时钟异常，需检查 RCC 配置
} bsp_timer_err_detail_t;

/**
 * @brief Timer 异步事件信息
 * @note  该结构体作为回调函数的参数，传递事件类型和附加信息。
 *        对于 EVT_ERROR，err_detail 字段包含详细的故障子码；
 *        对于 EVT_ABORT，count 和 err_detail 均无意义（固定为 0）。
 */
typedef struct {
    bsp_timer_event_t      event;      ///< 事件类型（ERROR 或 ABORT）
    uint32_t               count;      ///< 事件附加计数值（当前版本未使用，固定为 0）
    bsp_timer_err_detail_t err_detail; ///< 错误详情（仅 EVT_ERROR 有效）
} bsp_timer_evt_info_t;

/* ========================================================================== */
/*                           配置结构体                                        */
/* ========================================================================== */

/**
 * @brief Timer 初始化配置结构体
 * @note  该结构体为纯输入参数，由调用者在 bsp_timer_open 之前填充，
 *        驱动内部仅读取使用，不会修改任何字段。
 *        不同模式下，某些字段会被忽略，详见各字段的 mode 条件说明。
 */
typedef struct {
    /* ----- 通用参数（所有模式均有效） ----- */
    uint32_t               freq_hz;       ///< 计数频率(Hz)：PWM 模式为 PWM 频率，编码器模式为采样时钟
                                         ///< 设置为 BSP_TIMER_FREQ_DEFAULT(0) 表示使用硬件默认值
    bsp_timer_mode_t       mode;          ///< 工作模式：PWM 或 ENCODER，决定后续字段的生效范围

    /* ----- PWM 专用参数（mode == PWM 时有效） ----- */
    bsp_timer_pwm_align_t  pwm_align;     ///< PWM 对齐方式（边沿对齐/中心对齐）
    bsp_timer_pwm_polarity_t pwm_polarity; ///< PWM 输出极性（高电平有效/低电平有效）

    /* ----- 编码器专用参数（mode == ENCODER 时有效） ----- */
    bsp_timer_enc_mode_t   enc_mode;      ///< 编码器倍频模式（X1/X2/X4）
    bool                   enc_invert;    ///< 是否反转计数方向：false 为正向，true 为反向（A/B 相同时取反）

    /* ----- 通用参数（所有模式均有效） ----- */
    bool                   enable_irq;    ///< 是否使能中断回调：true 表示允许触发 EVT_ERROR/ABORT 回调
                                         ///< 编码器模式下建议始终为 true（用于 32 位溢出累加）
                                         ///< PWM 模式下此字段被忽略（PWM 无需中断）
} bsp_timer_config_t;

/* ========================================================================== */
/*                              回调定义                                        */
/* ========================================================================== */

/**
 * @brief Timer 异步事件回调函数原型
 * @param[in] handle    触发事件的设备句柄（由 bsp_timer_open 返回）
 * @param[in] info      事件详细信息（类型 + 错误码）
 * @param[in] user_data 用户自定义上下文指针（由 bsp_timer_set_callback 传入）
 *
 * @warning 【中断上下文安全约束】
 *          - 此回调在定时器中断服务程序中执行，严禁调用任何阻塞函数
 *          - 严禁调用 bsp_timer_delay_us/ms 等可能阻塞的函数
 *          - 严禁调用 vTaskDelay、信号量获取等 FreeRTOS 阻塞 API
 *          - 如需传递事件到任务层，请使用 xQueueSendFromISR 等 ISR 安全接口
 *          - 回调执行时间应尽可能短（建议 < 10μs）
 */
typedef void (*bsp_timer_callback_t)(bsp_timer_handle_t handle,
                                     const bsp_timer_evt_info_t *info,
                                     void *user_data);

/* ========================================================================== */
/*                            生命周期接口                                      */
/* ========================================================================== */

/**
 * @brief 打开并初始化指定定时器外设
 * @param[in]  id      定时器外设编号（1~4，对应 TIM1~TIM4）
 * @param[in]  cfg     配置参数指针（纯输入，驱动不会修改）
 * @param[out] handle  输出设备句柄（后续操作通过此句柄进行）
 * @retval BSP_OK           初始化成功
 * @retval BSP_ERR_PARAM    参数无效（cfg 为 NULL、id 超出范围、freq_hz 不合法等）
 * @retval BSP_ERR_BUSY     指定 id 的定时器已被占用（需先 close）
 * @retval BSP_ERR_IO       硬件初始化失败（HAL 配置错误）
 * @retval BSP_ERR_UNSUPPORT 请求的频率/模式/对齐方式/编码器倍频不被硬件支持
 *
 * @note  TIM1 为高级定时器，但本驱动仅使用其通用定时器功能，与 TIM2~TIM4 无差异。
 *        打开后定时器尚未启动，需调用 bsp_timer_start() 开始工作。
 */
bsp_err_t bsp_timer_open(uint8_t id, const bsp_timer_config_t *cfg,
                         bsp_timer_handle_t *handle);

/**
 * @brief 关闭定时器外设并释放所有资源
 * @param[in] handle 设备句柄
 * @retval BSP_OK           释放成功
 * @retval BSP_ERR_PARAM    句柄无效（未 open 或已 close）
 * @note   自动停止运行中的定时器（调用 bsp_timer_stop），
 *         禁用外设时钟，释放所有硬件资源，
 *         注销回调函数，防止悬垂指针。
 */
bsp_err_t bsp_timer_close(bsp_timer_handle_t handle);

/* ========================================================================== */
/*                            阻塞延时接口                                      */
/* ========================================================================== */

/**
 * @brief 阻塞微秒级延时（忙等待，高精度）
 * @param[in] us 延时微秒数（0~约 60 秒，超出范围会回绕），0 时立即返回
 *
 * @warning 【使用约束】
 *          - 基于 Cortex-M3 DWT 周期计数器忙等待，不释放 CPU
 *          - 【禁止】在中断/回调上下文中调用
 *          - 多任务环境下仅建议用于 < 1ms 的短延时（过长会阻塞其他任务）
 *          - 延时精度约为 1μs，受系统时钟精度影响
 * @note   不依赖任何定时器实例，可在任意位置调用（中断除外）；
 *         长延时应使用 bsp_timer_delay_ms 或 RTOS 的 vTaskDelay。
 */
void bsp_timer_delay_us(uint32_t us);

/**
 * @brief 阻塞毫秒级延时
 * @param[in] ms 延时毫秒数，0 时立即返回
 *
 * @note   行为由编译期宏 BSP_USE_RTOS 控制：
 *         - 定义了 BSP_USE_RTOS 宏：使用 RTOS tick 延时（低功耗友好，任务级安全）
 *           若 RTOS 未启动时调用，行为等同于忙等待
 *         - 未定义 BSP_USE_RTOS 宏：回退到 DWT 硬件忙等待（同 delay_us 约束）
 *         检测策略为纯编译期宏判断，无运行时开销。
 */
void bsp_timer_delay_ms(uint32_t ms);

/* ========================================================================== */
/*                          控制接口                                           */
/* ========================================================================== */

/**
 * @brief 启动定时器
 * @param[in] handle    设备句柄
 * @param[in] period_us PWM 模式：周期(us)，用于动态更改 PWM 频率；
 *                      编码器模式：此参数被忽略，建议传入 0
 * @param[in] duty_pct  PWM 模式：占空比 (0~100)，0 表示完全低电平，100 表示完全高电平；
 *                      编码器模式：此参数被忽略，建议传入 0
 * @retval BSP_OK           启动/更新成功
 * @retval BSP_ERR_PARAM    句柄无效、duty_pct 超范围、period_us 无法实现
 * @retval BSP_ERR_IO       硬件启动失败（HAL 启动函数返回错误）
 * @retval BSP_ERR_UNSUPPORT 当前模式不支持该操作
 *
 * @note   PWM 模式：
 *         - 若传入 period_us > 0，会重新计算 PSC/ARR 并重新启动定时器（存在短暂停止）
 *         - 若仅需调整占空比，建议使用 bsp_timer_pwm_set_duty() 以获得更平滑的效果
 *         - 启动后 CH1 输出使能，CH2~CH4 需通过 bsp_timer_pwm_start_channel() 单独使能
 *
 * @note   编码器模式：
 *         - 启动后硬件计数器从 0 开始计数
 *         - 自动使能更新中断用于 32 位溢出累加
 *         - 若之前已调用过 stop，再次 start 会从 0 重新计数（非恢复计数）
 */
bsp_err_t bsp_timer_start(bsp_timer_handle_t handle,
                          uint32_t period_us, uint8_t duty_pct);

/**
 * @brief 停止定时器
 * @param[in] handle 设备句柄
 * @retval BSP_OK           停止成功或本未在运行
 * @retval BSP_ERR_PARAM    句柄无效
 * @note   停止后【必定】触发 BSP_TIMER_EVT_ABORT 事件（若已注册回调且 enable_irq==true）
 *         编码器模式下停止后计数值保持，可通过 bsp_timer_encoder_get_count 读取；
 *         如需重新计数请调用 bsp_timer_encoder_reset 后再 start。
 */
bsp_err_t bsp_timer_stop(bsp_timer_handle_t handle);

/**
 * @brief 注册异步回调
 * @param[in] handle    设备句柄
 * @param[in] cb        回调函数指针（传入 NULL 表示注销）
 * @param[in] user_data 用户自定义上下文（透传给回调函数）
 * @retval BSP_OK           注册/注销成功
 * @retval BSP_ERR_PARAM    句柄无效
 * @note   回调触发条件：enable_irq == true（open 时设定）且 cb != NULL
 *         回调触发时机：bsp_timer_stop() 产生 EVT_ABORT 或硬件异常产生 EVT_ERROR
 *         编码器模式下回调用于通知错误事件，正常溢出中断不触发用户回调。
 */
bsp_err_t bsp_timer_set_callback(bsp_timer_handle_t handle,
                                 bsp_timer_callback_t cb,
                                 void *user_data);

/**
 * @brief 查询定时器运行状态
 * @param[in]  handle  设备句柄
 * @param[out] running 输出运行标志：true 表示正在运行，false 表示已停止
 * @retval BSP_OK           查询成功
 * @retval BSP_ERR_PARAM    句柄无效或 running 为 NULL
 */
bsp_err_t bsp_timer_is_running(bsp_timer_handle_t handle, bool *running);

/**
 * @brief 获取当前硬件计数器的瞬时值
 * @param[in]  handle 设备句柄
 * @param[out] count  输出当前计数值（0~ARR，ARR 为 open 时计算的值）
 * @retval BSP_OK           读取成功
 * @retval BSP_ERR_PARAM    句柄无效或 count 为 NULL
 * @retval BSP_ERR_UNSUPPORT 当前模式为 ENCODER（应使用 bsp_timer_encoder_get_count）
 * @note   返回值可用于计算时间差：time_diff = (count2 - count1) / freq_hz
 *         计数器位宽 16 位，溢出后自然回绕，上层需自行处理溢出逻辑。
 */
bsp_err_t bsp_timer_get_count(bsp_timer_handle_t handle, uint32_t *count);

/* ========================================================================== */
/*                           PWM 专用接口                                       */
/* ========================================================================== */

/**
 * @brief 获取当前 PWM 输出实际频率
 * @param[in]  handle  设备句柄
 * @param[out] freq_hz 输出实际 PWM 频率 (Hz)
 * @retval BSP_OK           成功
 * @retval BSP_ERR_PARAM    句柄无效或 freq_hz 为 NULL
 * @retval BSP_ERR_UNSUPPORT 当前模式非 PWM
 * @note   返回值由 open 时的 freq_hz 与硬件分频系数共同决定，
 *         可能不等于请求值（硬件取最接近的可实现频率）。
 *         使用此接口可获知实际运行频率，用于校准上层参数。
 */
bsp_err_t bsp_timer_pwm_get_freq(bsp_timer_handle_t handle, uint32_t *freq_hz);

/**
 * @brief 更新指定 PWM 通道的占空比
 * @param[in] handle    设备句柄
 * @param[in] channel   通道编号 (1~4)，对应 TIMx_CH1~CH4
 * @param[in] duty_pct  占空比 (0~100)：0 完全低电平，100 完全高电平
 * @retval BSP_OK           更新成功
 * @retval BSP_ERR_PARAM    句柄无效、通道超范围（非 1~4）、占空比超范围
 * @retval BSP_ERR_UNSUPPORT 当前模式非 PWM
 * @note   直接写 CCR 寄存器，原子操作，无需 stop/restart，约 10ns 完成。
 *         前提：open 时已预初始化所有通道的 OC 配置（由 _config_pwm_mode 完成）。
 *         通道 1 的初始占空比由 bsp_timer_start 设置，通道 2~4 需通过本接口设置。
 *         若定时器未启动，写入占空比不会生效，需先调用 bsp_timer_start。
 */
bsp_err_t bsp_timer_pwm_set_duty(bsp_timer_handle_t handle,
                                 uint8_t channel, uint8_t duty_pct);

/**
 * @brief 启动指定 PWM 通道输出
 * @param[in] handle  设备句柄
 * @param[in] channel 通道编号 (1~4)
 * @retval BSP_OK           启动成功
 * @retval BSP_ERR_PARAM    句柄无效或通道超范围
 * @retval BSP_ERR_UNSUPPORT 当前模式非 PWM
 * @retval BSP_ERR_IO       硬件启动失败（HAL 启动函数返回错误）
 * @note   启动后该通道立即输出对应占空比的 PWM 信号。
 *         通道 1 由 bsp_timer_start 自动启动，通道 2~4 需显式调用本接口。
 *         若定时器未启动，本接口会返回错误。
 */
bsp_err_t bsp_timer_pwm_start_channel(bsp_timer_handle_t handle, uint8_t channel);

/**
 * @brief 启动全部 PWM 通道输出
 * @param[in] handle 设备句柄
 * @retval BSP_OK           全部启动成功
 * @retval BSP_ERR_PARAM    句柄无效
 * @retval BSP_ERR_UNSUPPORT 当前模式非 PWM
 * @retval BSP_ERR_IO       任意通道启动失败（返回具体错误）
 * @note   等效于依次调用 bsp_timer_pwm_start_channel(1~4)。
 *         若部分通道启动失败，已启动的通道不会自动回退（需上层自行处理）。
 */
bsp_err_t bsp_timer_pwm_start_all(bsp_timer_handle_t handle);

/* ========================================================================== */
/*                         编码器专用接口                                       */
/* ========================================================================== */

/**
 * @brief 获取编码器累计计数值（有符号 32 位）
 * @param[in]  handle 设备句柄
 * @param[out] count  输出累计脉冲数（正=正向，负=反向）
 * @retval BSP_OK           读取成功
 * @retval BSP_ERR_PARAM    句柄无效或 count 为 NULL
 * @retval BSP_ERR_UNSUPPORT 当前模式非 ENCODER
 * @note   驱动内部通过中断累加硬件计数器，自动处理 16 位回绕，
 *         上层无需关心硬件位宽，直接获得连续的 32 位有符号值。
 *         正常使用下 int32_t 范围（±2,147,483,647）足够，
 *         若超过此范围会发生溢出，且不会触发饱和处理（建议定期重置）。
 *         调用本接口时，会短暂禁用全局中断（约 8~10 个 CPU 周期），
 *         确保读取过程中不会发生溢出导致计数丢失。
 */
bsp_err_t bsp_timer_encoder_get_count(bsp_timer_handle_t handle, int32_t *count);

/**
 * @brief 重置编码器计数器为零
 * @param[in] handle 设备句柄
 * @retval BSP_OK           重置成功
 * @retval BSP_ERR_PARAM    句柄无效
 * @retval BSP_ERR_UNSUPPORT 当前模式非 ENCODER
 * @note   可在运行时调用，不影响编码器继续计数；
 *         用于建立机械零点或消除累积误差。
 *         重置后，硬件计数器和软件累加器同时归零。
 */
bsp_err_t bsp_timer_encoder_reset(bsp_timer_handle_t handle);

/**
 * @brief 获取编码器硬件计数器位宽
 * @param[in]  handle 设备句柄
 * @param[out] bits   输出位宽（16 或 32）
 * @retval BSP_OK           成功
 * @retval BSP_ERR_PARAM    句柄无效或 bits 为 NULL
 * @retval BSP_ERR_UNSUPPORT 当前模式非 ENCODER
 * @note   供上层评估采样周期是否合理（避免两次读取间硬件溢出超过半量程）；
 *         对于 STM32F1，所有定时器均为 16 位，返回值固定为 16。
 *         尽管驱动已做 32 位累加，但在极端高速场景下仍需此信息做防御性设计。
 */
bsp_err_t bsp_timer_encoder_get_bit_width(bsp_timer_handle_t handle, uint8_t *bits);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TIMER_H__ */