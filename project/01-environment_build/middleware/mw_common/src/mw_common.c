/**
 * @file    mw_common.c
 * @brief   中间件通用数学工具库实现
 *
 * 参考文档：本项目《开发手册》5.6 定点运算约定
 * @author  xserein
 * @version v1.0
 */
#include "mw_common.h"
#include "q15_math.h"
#include <stdbool.h>

/* ========================================================================== */
/*                          整数域三角函数                                       */
/* ========================================================================== */

/**
 * @brief   atan(r) 查表（毫度），r = k/16，k = 0..16
 * @note    表值 = atan(k/16) * 1000，四舍五入到整数毫度。
 *          线性插值在相邻表项间引入的最大误差 < 0.2°，
 *          对互补滤波的加速度计角度基准而言精度足够。
 */
static const int32_t s_atan_table_mdg[MW_MATH_ATAN2_TABLE_SIZE] = {
    0,     3576,  7125,  10620, 14036, 17354, 20556, 23629,
    26565, 29358, 32005, 34509, 36870, 39090, 41186, 43152,
    45000,
};

int32_t mw_math_atan2_mdg(int32_t y, int32_t x)
{
    /* --- 边界特判 --- */
    if (y == 0) {
        return (x >= 0) ? 0 : 180000;
    }
    if (x == 0) {
        return (y > 0) ? 90000 : -90000;
    }

    /* --- 象限归约：取绝对值，映射到第一象限 --- */
    int32_t ay = (y < 0) ? -y : y;
    int32_t ax = (x < 0) ? -x : x;

    /*
     * WHY: atan 在 |y|>|x| 时比值发散，故交换轴用 atan(x/y) = 90° - atan(y/x)，
     *      保证查表输入比值恒在 [0, 1] 区间。
     *      q15_div 直接以原始整数为入参：((a << 15) / b) 恰为比值 a/b 的 Q15
     *      表示，与输入是否为 Q15 格式无关（比值对公因子缩放不敏感）。
     */
    bool swap_axes = (ay > ax);
    q15_t r_q15 = swap_axes ? q15_div((q15_t)ax, (q15_t)ay)
                            : q15_div((q15_t)ay, (q15_t)ax);

    /* --- 查表 + 线性插值（16 段，每段 2048/32768） --- */
    uint16_t idx  = ((uint16_t)r_q15) >> 11;      /* 0..15 */
    uint16_t frac = ((uint16_t)r_q15) & 0x7FFU;   /* 段内小数 (0..2047) */
    int32_t base  = s_atan_table_mdg[idx];
    int32_t delta = s_atan_table_mdg[idx + 1] - base;

    int32_t angle = base + (delta * (int32_t)frac) / 2048;

    if (swap_axes) {
        angle = 90000 - angle;
    }

    /* --- 按原始象限还原符号 --- */
    if (x > 0) {
        /* 第一 / 第四象限 */
        return (y > 0) ? angle : -angle;
    }
    /* 第二 / 第三象限 */
    return (y > 0) ? (180000 - angle) : -(180000 - angle);
}
