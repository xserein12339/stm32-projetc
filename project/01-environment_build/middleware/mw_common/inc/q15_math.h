/**
 * @file    q15_math.h
 * @brief   Q15 定点数学基础库
 * @note    采用标准C实现，保证跨平台可移植性。
 *          在Cortex-M3/M4上使用时，核心运算可被编译器优化为
 *          SMULBB / SSAT 等 DSP 指令。
 *
 * @author  xserein
 * @version v1.2
 */
#ifndef __Q15_MATH_H__
#define __Q15_MATH_H__

#include <stdint.h>
#include <limits.h>   /* INT32_MAX / INT32_MIN for q30_sat_add */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              类型与常量定义                                   */
/* ========================================================================== */

typedef int16_t q15_t;  ///< Q15 定点数 (16-bit, 1 符号位 + 15 小数位)
typedef int32_t q30_t;  ///< Q30 定点数 (32-bit, 用于中间运算防溢出)

#define Q15_SHIFT       15
#define Q15_SCALE       (1 << Q15_SHIFT)  /**< 32768 */

/**
 * @brief Q15 数值范围约定: [-1.0, +1.0)
 *
 * |  宏定义    |  十六进制  |  物理值                     |
 * |-----------|-----------|----------------------------|
 * |  Q15_MAX  |  0x7FFF   |  +32767/32768 ≈ +0.999969  |
 * |  Q15_MIN  |  0x8000   |  -32768/32768 = -1.0       |
 */
#define Q15_MAX         ((q15_t)0x7FFF)
#define Q15_MIN         ((q15_t)0x8000)
#define Q15_ONE         Q15_MAX           ///< 近似 +1.0
#define Q15_HALF        ((q15_t)0x4000)   ///< 精确 +0.5
#define Q15_ZERO        ((q15_t)0x0000)   ///< 精确  0.0

/* ========================================================================== */
/*                         内部工具函数                                          */
/* ========================================================================== */

/**
 * @brief  32位值饱和至16位有符号范围 (纯C实现)
 * @note   使用字面量而非 Q15_MAX/Q15_MIN，避免隐式符号扩展干扰
 *         编译器对 ARM SSAT 指令的模式匹配。
 */
static inline int32_t _q15_sat16(int32_t val)
{
    if (val > 32767)  return 32767;
    if (val < -32768) return -32768;
    return val;
}

/**
 * @brief  Q30 饱和加法 (防有符号溢出 UB)
 * @param  a  加数 (Q30)
 * @param  b  被加数 (Q30)
 * @return 饱和后的和 (Q30)
 */
static inline q30_t q30_sat_add(q30_t a, q30_t b)
{
    q30_t res = a + b;
    if ((a > 0) && (b > 0) && (res < a)) return INT32_MAX;
    if ((a < 0) && (b < 0) && (res > a)) return INT32_MIN;
    return res;
}

/* ========================================================================== */
/*                           核心运算接口                                        */
/* ========================================================================== */

/**
 * @brief  Q15 乘法 (带四舍五入 + 饱和保护)
 * @note   PID 参数计算推荐使用此版本，消除截断偏差。
 */
static inline q15_t q15_mul(q15_t a, q15_t b)
{
    q30_t product = (q30_t)a * (q30_t)b;
    product += (1 << (Q15_SHIFT - 1));   /* 四舍五入偏置 0x4000 */
    return (q15_t)_q15_sat16(product >> Q15_SHIFT);
}

/**
 * @brief  Q15 乘法 (截断版，无四舍五入)
 * @note   适用于对精度不敏感的高频内环计算。
 */
static inline q15_t q15_mul_trunc(q15_t a, q15_t b)
{
    return (q15_t)_q15_sat16(((q30_t)a * (q30_t)b) >> Q15_SHIFT);
}

/**
 * @brief  Q15 乘累加 (MAC): acc + (a * b)
 * @note   acc 保持 Q30 全精度。调用者需自行管理量纲。
 */
static inline q30_t q15_mac(q30_t acc, q15_t a, q15_t b)
{
    return acc + (q30_t)a * (q30_t)b;
}

/** @brief Q15 饱和加法 */
static inline q15_t q15_add(q15_t a, q15_t b)
{
    return (q15_t)_q15_sat16((q30_t)a + (q30_t)b);
}

/** @brief Q15 饱和减法 */
static inline q15_t q15_sub(q15_t a, q15_t b)
{
    return (q15_t)_q15_sat16((q30_t)a - (q30_t)b);
}

/**
 * @brief  Q15 绝对值 (饱和处理)
 * @note   abs(Q15_MIN) 安全返回 Q15_MAX，避免补码溢出。
 */
static inline q15_t q15_abs(q15_t val)
{
    if (val == Q15_MIN) return Q15_MAX;
    return (val < 0) ? (q15_t)(-val) : val;
}

/** @brief Q30 饱和截断至 Q15 */
static inline q15_t q15_sat(q30_t val)
{
    return (q15_t)_q15_sat16(val);
}

/**
 * @brief  Q15 定点除法: a / b
 * @note   物理意义: Q15(a) / Q15(b) = Q15(result)
 *         - 除数为 0 时返回饱和极值 (a>=0 → Q15_MAX, a<0 → Q15_MIN)
 *         - 结果为截断取整 (向零方向)，非四舍五入
 *         - 中间量使用 Q30 防止溢出，最终结果经 _q15_sat16 饱和保护
 *         - Q15_MIN (-1.0) 作为除数是合法的，不会被当作异常处理
 *
 * @param  a  被除数 (Q15)
 * @param  b  除数 (Q15)
 * @return Q15 商 (饱和保护)
 */
static inline q15_t q15_div(q15_t a, q15_t b)
{
    if (b == 0) {
        return (a >= 0) ? Q15_MAX : Q15_MIN;
    }
    /* a 左移 15 位提升为 Q30 精度，除以 Q15 的 b 后结果自动为 Q15 */
    q30_t dividend = (q30_t)a << Q15_SHIFT;
    q30_t result = dividend / (q30_t)b;
    return (q15_t)_q15_sat16(result);
}

/* ========================================================================== */
/*                      转换接口 (仅用于配置/调试)                                */
/* ========================================================================== */

/**
 * @brief  float → Q15 (带饱和保护)
 * @note   仅在参数初始化或上位机通信时使用，运行时禁止调用。
 */
static inline q15_t float_to_q15(float val)
{
    if (val >= 1.0f)  return Q15_MAX;
    if (val <= -1.0f) return Q15_MIN;
    return (q15_t)(val * (float)Q15_SCALE);
}

/**
 * @brief  Q15 → float
 * @note   仅在调试观测 / 日志输出时使用，运行时禁止调用。
 */
static inline float q15_to_float(q15_t val)
{
    return (float)val / (float)Q15_SCALE;
}

#ifdef __cplusplus
}
#endif

#endif /* __Q15_MATH_H__ */