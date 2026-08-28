/**
 * @file    mw_pid.h
 * @brief   定点 PID 控制器中间件 
 *
 * @author  xserein
 * @version v1.0
 */
#ifndef __MW_PID_H__
#define __MW_PID_H__

#include <stdint.h>
#include <stdbool.h>
#include "q15_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                        PID 控制器结构体定义                                    */
/* ========================================================================== */
/**
 * @brief PID 控制器运行时状态结构体
 *
 * @note 实例化要求：
 *       - 推荐使用全局或静态存储期
 *       - 初始化前需通过 mw_pid_init() 配置参数
 *       - 内部状态字段由框架管理，外部禁止直接修改
 */
typedef struct {
    /* === PID 增益参数 (Q15) === */
    q15_t Kp;               ///< 比例增益
    q15_t Ki;               ///< 积分增益
    q15_t Kd;               ///< 微分增益

    /* === 输出限幅 (Q15) === */
    q15_t out_min;          ///< 输出下限
    q15_t out_max;          ///< 输出上限

    /* === 积分限幅 (Q15) === */
    q15_t integral_min;     ///< 积分累加器下限 (纯误差累积的 Q15 等效值)
    q15_t integral_max;     ///< 积分累加器上限 (纯误差累积的 Q15 等效值)

    /* === 内部状态 (框架管理，外部禁止直接修改) === */
    q30_t integral;         ///< 积分累加器 (Q30)，存储纯误差累积 error
    q15_t prev_feedback;    ///< 上一次反馈值 (Q15)，用于 Derivative on Measurement
    q15_t prev_d_term;      ///< 上一拍微分项 (Q15)，用于微分低通滤波

    /* === 微分滤波配置 === */
    q15_t d_filter_coef;    ///< 一阶低通滤波系数 α (0~1, Q15), 0=无滤波
} mw_pid_t;

/* ========================================================================== */
/*                         PID 初始化参数结构体                                   */
/* ========================================================================== */

/**
 * @brief PID 初始化参数 (便于批量配置)
 */
typedef struct {
    q15_t Kp;               ///< 比例增益 (Q15)
    q15_t Ki;               ///< 积分增益 (Q15)
    q15_t Kd;               ///< 微分增益 (Q15)
    q15_t out_min;          ///< 输出下限 (Q15)
    q15_t out_max;          ///< 输出上限 (Q15)
    q15_t integral_min;     ///< 积分下限 (Q15)
    q15_t integral_max;     ///< 积分上限 (Q15)
    q15_t d_filter_coef;    ///< 微分滤波系数 α, 0=无滤波 (Q15)
} mw_pid_params_t;

/* ========================================================================== */
/*                           核心 API 函数                                       */
/* ========================================================================== */

/**
 * @brief  初始化 PID 控制器
 * @param  pid     PID 对象指针
 * @param  params  参数结构体指针 (若为 NULL，则使用默认值: Kp=1.0, 其余=0)
 */
void mw_pid_init(mw_pid_t *pid, const mw_pid_params_t *params);

/**
 * @brief  重置 PID 内部状态 (保留参数不变)
 * @param  pid  PID 对象指针
 * @note   清零积分累加器、历史反馈值、历史微分项。
 */
void mw_pid_reset(mw_pid_t *pid);

/**
 * @brief  执行一次 PID 计算
 *
 * @param  pid         PID 对象指针
 * @param  setpoint    设定值 (Q15)
 * @param  feedback    反馈值 (Q15)
 * @return q15_t       控制输出 (已限幅至 [out_min, out_max])
 *
 * @note 【算法特性】
 *       - Derivative on Measurement: 微分仅作用于反馈值，消除 setpoint 突变冲击
 *       - 条件积分抗饱和 (Conditional Integration): 输出饱和时阻止积分继续恶化
 *       - 积分器 Q30 全精度累加，输出时右移 15 位转换回 Q15
 *       - 积分累加使用饱和加法，防止溢出
 */
q15_t mw_pid_update(mw_pid_t *pid, q15_t setpoint, q15_t feedback);

/* ========================================================================== */
/*                     在线参数修改 (内联，零开销)                                */
/* ========================================================================== */

/**
 * @brief  在线修改比例增益
 * @param  pid  PID 对象指针
 * @param  kp   新比例增益 (Q15)
 */
static inline void mw_pid_set_kp(mw_pid_t *pid, q15_t kp)
{
    pid->Kp = kp;
}

/**
 * @brief  在线修改积分增益
 * @param  pid  PID 对象指针
 * @param  ki   新积分增益 (Q15)
 */
static inline void mw_pid_set_ki(mw_pid_t *pid, q15_t ki)
{
    pid->Ki = ki;
}

/**
 * @brief  在线修改微分增益
 * @param  pid  PID 对象指针
 * @param  kd   新微分增益 (Q15)
 */
static inline void mw_pid_set_kd(mw_pid_t *pid, q15_t kd)
{
    pid->Kd = kd;
}

/**
 * @brief  在线修改输出限幅
 * @param  pid  PID 对象指针
 * @param  min  输出下限 (Q15)
 * @param  max  输出上限 (Q15)
 */
static inline void mw_pid_set_out_limit(mw_pid_t *pid, q15_t min, q15_t max)
{
    pid->out_min = min;
    pid->out_max = max;
}

/**
 * @brief  在线修改积分限幅
 * @param  pid  PID 对象指针
 * @param  min  积分下限 (Q15)
 * @param  max  积分上限 (Q15)
 */
static inline void mw_pid_set_integral_limit(mw_pid_t *pid, q15_t min, q15_t max)
{
    pid->integral_min = min;
    pid->integral_max = max;
}

#ifdef __cplusplus
}
#endif

#endif /* __MW_PID_H__ */