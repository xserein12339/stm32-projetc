/**
 * @file    mw_filter.c
 * @brief   常用定点数字滤波器实现 
 *
 * @author  xserein
 * @version v1.0
 *
 */
#include "mw_filter.h"
#include <string.h>

/* ========================================================================== */
/*                            一阶低通滤波器                                    */
/* ========================================================================== */

void mw_filter_lpf1_init(mw_filter_lpf1_t *f, q15_t alpha)
{
    if (alpha > Q15_ONE)  alpha = Q15_ONE;
    if (alpha < Q15_ZERO) alpha = Q15_ZERO;
    f->alpha    = alpha;
    f->prev_out = Q15_ZERO;
}

q15_t mw_filter_lpf1_update(mw_filter_lpf1_t *f, q15_t in)
{
    q15_t alpha           = f->alpha;
    q15_t one_minus_alpha = q15_sub(Q15_ONE, alpha);

    /* y[n] = α * x[n] + (1-α) * y[n-1] */
    q15_t out = q15_mul(alpha, in);
    out = q15_add(out, q15_mul(one_minus_alpha, f->prev_out));

    f->prev_out = out;
    return out;
}

/* ========================================================================== */
/*                            滑动平均滤波器                                    */
/* ========================================================================== */
void mw_filter_mavg_init(mw_filter_mavg_t *f, q15_t *buf, uint16_t size)
{
    if (buf == NULL || size == 0) {
        f->buffer = NULL;
        f->size   = 0;
        return;
    }
    f->buffer = buf;
    f->size   = size;
    f->index  = 0;
    f->sum    = 0;
    f->is_full = false;
    memset(buf, 0, sizeof(q15_t) * size);
}

void mw_filter_mavg_reset(mw_filter_mavg_t *f)
{
    if (f->buffer == NULL || f->size == 0) return;
    memset(f->buffer, 0, sizeof(q15_t) * f->size);
    f->index   = 0;
    f->sum     = 0;
    f->is_full = false;
}

q15_t mw_filter_mavg_update(mw_filter_mavg_t *f, q15_t in)
{
    if (f->buffer == NULL || f->size == 0) {
        return Q15_ZERO;
    }
    uint16_t idx      = f->index;
    q15_t    old_val  = f->buffer[idx];
    f->sum += ((q30_t)in - (q30_t)old_val);
    f->buffer[idx] = in;
    idx++;
    if (idx >= f->size) {
        idx = 0;
        f->is_full = true;
    }
    f->index = idx;
    uint16_t count = f->is_full ? f->size : idx;
    if (count == 0) return Q15_ZERO;

    q30_t avg = f->sum / (q30_t)count;
    return q15_sat(avg);
}

/* ========================================================================== */
/*                            一维卡尔曼滤波器                                  */
/* ========================================================================== */

/**
 * @brief 内部宏: Q15 转 Q30 (左移 15 位)
 * @note  用于将 Q15 参数提升到 Q30 域进行全精度运算
 */
#define Q15_TO_Q30(val)  ((q30_t)(val) << Q15_SHIFT)

/**
 * @brief 内部宏: Q30 转 Q15 (右移 15 位 + 饱和)
 */
#define Q30_TO_Q15(val)  q15_sat((val) >> Q15_SHIFT)

void mw_filter_kalman1d_init(mw_filter_kalman1d_t *f, q15_t q, q15_t r, q15_t init_x)
{
    /* 钳位参数到合法范围 [0, 1] */
    f->q = (q > Q15_ONE) ? Q15_ONE : (q < Q15_ZERO ? Q15_ZERO : q);
    f->r = (r > Q15_ONE) ? Q15_ONE : (r < Q15_ZERO ? Q15_ZERO : r);
    mw_filter_kalman1d_reset(f, init_x);
}

void mw_filter_kalman1d_reset(mw_filter_kalman1d_t *f, q15_t init_x)
{
    f->x = init_x;
    f->p = Q15_TO_Q30(Q15_ONE);
}

q15_t mw_filter_kalman1d_update(mw_filter_kalman1d_t *f, q15_t z)
{

    /* 预测 (先验) */
    /* p_est = p + q  (Q30 + Q15→Q30) */
    q30_t p_est = q30_sat_add(f->p, Q15_TO_Q30(f->q));

    /* 卡尔曼增益 K = p_est / (p_est + r)  */
    /* 分母 = p_est(Q30) + r(Q15→Q30) */
    q30_t denom_q30 = q30_sat_add(p_est, Q15_TO_Q30(f->r));

    /* K = p_est / denom，两者同为 Q30，商为纯比值 → Q15 */
    /* 为避免 Q30/Q30 直接除丢失精度，将被除数左移 15 位后再除 */
    q15_t k_gain;
    if (denom_q30 == 0) {
        k_gain = Q15_ONE;  ///< 分母为零时完全信任测量值
    } else {
        q30_t num_shifted = p_est << Q15_SHIFT;
        q30_t k_raw = num_shifted / denom_q30;
        k_gain = q15_sat(k_raw);
    }

    /* 状态更新 x = x + K * (z - x)  */
    q15_t innovation  = q15_sub(z, f->x);
    q15_t correction  = q15_mul(k_gain, innovation);
    f->x = q15_add(f->x, correction);

    /* 协方差更新 P = (1 - K) * p_est  */
    /* (1-K) 为 Q15，p_est 为 Q30，乘积为 Q30 (需右移 15 位归一化) */
    q15_t one_minus_k = q15_sub(Q15_ONE, k_gain);
    /* q15_mul_trunc: (Q15 * Q30) 不能直接用，手动计算 */
    f->p = ((q30_t)one_minus_k * p_est) >> Q15_SHIFT;

    /* 防止 P 因截断累积衰减至 0 (设置最小地板值) */
    if (f->p < 0) f->p = 0;

    return f->x;
}