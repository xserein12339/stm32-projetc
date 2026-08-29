/**
 * @file    test_mw_math.c
 * @brief   mw_math_atan2_mdg 单元测试（与浮点 atan2 对照，全象限扫描）
 *
 * 验收指标：最大绝对误差 <= 25 毫度（0.025°，为 SRS 0.1° 指标的 1/4）
 *
 * @author  xserein
 * @version v1.1
 */
#include "test_framework.h"
#include "mw_common.h"
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

static void test_cardinal_angles(void)
{
    /* 四个基点：单位毫度（y 分量在前，atan2(y, x)） */
    TEST_ASSERT_INT_WITHIN(3, 0,      mw_math_atan2_mdg(0, 1000));
    TEST_ASSERT_INT_WITHIN(3, 90000,  mw_math_atan2_mdg(1000, 0));
    TEST_ASSERT_INT_WITHIN(3, 180000, mw_math_atan2_mdg(0, -1000));
    TEST_ASSERT_INT_WITHIN(3, -90000, mw_math_atan2_mdg(-1000, 0));
}

static void test_symmetry(void)
{
    /* 奇对称：atan2(-y, x) = -atan2(y, x) */
    int32_t a = mw_math_atan2_mdg(1234, 5678);
    int32_t b = mw_math_atan2_mdg(-1234, 5678);
    TEST_ASSERT_INT_WITHIN(5, 0, a + b);
}

static void test_octant_sweep(void)
{
    /* 全象限扫描（含轴交换路径）：与浮点 atan2 对照，单位毫度。
     * 基准取【同样的整数输入】，剥离 (int32_t) 截断引入的输入失配
     * （r=500 时 y 截断 0.5 可造成 ~80 mdg 假误差），纯测实现误差。 */
    int32_t max_err = 0;

    for (int deg = -179; deg <= 180; deg += 3) {
        double r = 500.0;
        double y = r * sin((double)deg * PI / 180.0);
        double x = r * cos((double)deg * PI / 180.0);

        /* 不同幅值独立对照：各自与自身整数输入的精确 atan2 比，
         * 避免幅值截断差异被误判为实现误差（swap_axes 边界尤甚） */
        double r2 = 2000.0;
        double y2 = r2 * sin((double)deg * PI / 180.0);
        double x2 = r2 * cos((double)deg * PI / 180.0);

        int32_t iy = (int32_t)y, ix = (int32_t)x;
        int32_t got  = mw_math_atan2_mdg(iy, ix);
        int32_t want = (int32_t)(atan2((double)iy, (double)ix)
                                 * 180000.0 / PI);

        int32_t iy2 = (int32_t)y2, ix2 = (int32_t)x2;
        int32_t got2  = mw_math_atan2_mdg(iy2, ix2);
        int32_t want2 = (int32_t)(atan2((double)iy2, (double)ix2)
                                  * 180000.0 / PI);

        int32_t err  = abs(got - want);
        int32_t err2 = abs(got2 - want2);
        if (err > max_err)  { max_err = err; }
        if (err2 > max_err) { max_err = err2; }
    }

    /* 验收：<= 25 毫度 */
    TEST_ASSERT_INT_WITHIN(0, 1, (max_err <= 25) ? 1 : 0);
    printf("  [info] atan2 max err = %d mdg\n", max_err);
}

static void test_degenerate_inputs(void)
{
    /* 零向量：约定返回 0 */
    TEST_ASSERT_EQUAL_INT(0, mw_math_atan2_mdg(0, 0));
    /* 极小量值（右移 11 位可能归零的路径） */
    TEST_ASSERT_INT_WITHIN(3, 90000, mw_math_atan2_mdg(2, 0));
}

int test_mw_math(void)
{
    TF_BEGIN();
    test_cardinal_angles();
    test_symmetry();
    test_octant_sweep();
    test_degenerate_inputs();
    TF_REPORT("mw_math");
}
