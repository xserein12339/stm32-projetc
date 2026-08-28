/**
 * @file    mw_pid.c
 * @brief   定点 PID 控制器实现 
 *
 * @author  xserein
 * @version v1.0
 *
 */
#include "mw_pid.h"
#include "q15_math.h"

/* ========================================================================== */
/*                           生命周期管理                                        */
/* ========================================================================== */
/**
 * @brief  初始化 PID 控制器
 * @param  pid     PID 对象指针
 * @param  params  参数结构体指针 (若为 NULL，则使用默认值)
 */
void mw_pid_init(mw_pid_t *pid, const mw_pid_params_t *params)
{
    if (params) {
        pid->Kp             = params->Kp;
        pid->Ki             = params->Ki;
        pid->Kd             = params->Kd;
        pid->out_min        = params->out_min;
        pid->out_max        = params->out_max;
        pid->integral_min   = params->integral_min;
        pid->integral_max   = params->integral_max;
        pid->d_filter_coef  = params->d_filter_coef;
    } else {
        /* 默认参数：比例 1.0，积分/微分 0，限幅 ±1.0 */
        pid->Kp             = Q15_ONE;
        pid->Ki             = Q15_ZERO;
        pid->Kd             = Q15_ZERO;
        pid->out_min        = Q15_MIN;
        pid->out_max        = Q15_MAX;
        pid->integral_min   = Q15_MIN;
        pid->integral_max   = Q15_MAX;
        pid->d_filter_coef  = Q15_ZERO;
    }

    /* 安全防护：滤波系数钳位至 [0, 1] */
    if (pid->d_filter_coef > Q15_ONE)  pid->d_filter_coef = Q15_ONE;
    if (pid->d_filter_coef < Q15_ZERO) pid->d_filter_coef = Q15_ZERO;

    mw_pid_reset(pid);
}

/**
 * @brief  重置 PID 内部状态 (保留参数不变)
 * @param  pid  PID 对象指针
 */
void mw_pid_reset(mw_pid_t *pid)
{
    pid->integral      = 0;
    pid->prev_feedback = 0;
    pid->prev_d_term   = 0;
}

/* ========================================================================== */
/*                           核心控制计算                                        */
/* ========================================================================== */
/**
 * @brief  执行一次 PID 计算
 *
 * @param  pid         PID 对象指针
 * @param  setpoint    设定值 (Q15)
 * @param  feedback    反馈值 (Q15)
 * @return q15_t       控制输出 (已限幅)
 *
 * @par 计算流程:
 *   1. 误差计算 (饱和减法)
 *   2. 比例项 P = Kp * error
 *   3. 微分项 D = Kd * (prev_feedback - feedback)，可选一阶低通滤波
 *   4. 积分项 I = Ki * (∑error)，条件积分抗饱和
 *   5. 合成输出 out = P + I + D，输出限幅
 */
q15_t mw_pid_update(mw_pid_t *pid, q15_t setpoint, q15_t feedback)
{
    /* 误差计算 */
    q15_t error = q15_sub(setpoint, feedback);

    /* 比例项: P = Kp * error */ 
    q15_t p_term = q15_mul(pid->Kp, error);

    /* 微分项 D = Kd * (prev_feedback - feedback) */
    q15_t d_feedback = q15_sub(pid->prev_feedback, feedback);
    q15_t d_term_raw = q15_mul(pid->Kd, d_feedback);
    q15_t d_term;
    if (pid->d_filter_coef != Q15_ZERO) {
        /* 一阶低通滤波: d_term = (1-α)*prev_d + α*d_raw */
        q15_t alpha           = pid->d_filter_coef;
        q15_t one_minus_alpha = q15_sub(Q15_ONE, alpha);
        q15_t filtered_prev   = q15_mul(one_minus_alpha, pid->prev_d_term);
        q15_t filtered_raw    = q15_mul(alpha, d_term_raw);
        d_term = q15_add(filtered_prev, filtered_raw);
    } else {
        d_term = d_term_raw;
    }
    /* 积分项 (Q30 全精度纯误差累积 + 条件积分抗饱和) */
    /* 计算当前积分项输出 (基于上一拍累积值) */
    q30_t int_max_q30 = (q30_t)pid->integral_max << Q15_SHIFT;
    q30_t int_min_q30 = (q30_t)pid->integral_min << Q15_SHIFT;

    q15_t i_term = q15_mul(pid->Ki, q15_sat(pid->integral >> Q15_SHIFT));

    /* 计算未限幅的总输出 */
    q15_t out_unsat = q15_add(p_term, i_term);
    out_unsat = q15_add(out_unsat, d_term);

    /* 输出限幅 */
    q15_t out = out_unsat;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    /* 条件积分：判断是否应阻止积分累加 */
    bool output_saturated = (out_unsat > pid->out_max) || (out_unsat < pid->out_min);

    if (!output_saturated) {
        /* 输出未饱和，正常累加误差（使用饱和加法防溢出） */
        q30_t delta = (q30_t)error << Q15_SHIFT;
        pid->integral = q30_sat_add(pid->integral, delta);
    } else {
        /* 输出已饱和，仅当误差方向有助于退出饱和时才累加 */
        q15_t sat_diff = q15_sub(out_unsat, out);
        bool error_agrees_with_saturation =
            ((sat_diff > 0) && (error > 0)) ||
            ((sat_diff < 0) && (error < 0));

        if (!error_agrees_with_saturation) {
            /* 误差方向有助于退出饱和，允许累加 */
            q30_t delta = (q30_t)error << Q15_SHIFT;
            pid->integral = q30_sat_add(pid->integral, delta);
        }
        /* 否则：误差加剧饱和，跳过累加，积分器冻结 */
    }

    /* 积分累加器限幅 */
    if (pid->integral > int_max_q30) pid->integral = int_max_q30;
    if (pid->integral < int_min_q30) pid->integral = int_min_q30;

    /* 保存状态供下一拍使用 */
    pid->prev_feedback = feedback;
    pid->prev_d_term   = d_term;

    return out;
}