/**
 * @file    test_q15_math.c
 * @brief   q15_math 内联运算单元测试（饱和/乘法/除法/回绕安全）
 *
 * @author  xserein
 * @version v1.0
 */
#include "test_framework.h"
#include "q15_math.h"

static void test_saturation(void)
{
    /* 正向饱和 */
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_sat(40000));
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_sat(Q15_SCALE * 2));
    /* 负向饱和 */
    TEST_ASSERT_EQUAL_INT(Q15_MIN, q15_sat(-40000));
    /* 边界保持 */
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_sat(Q15_MAX));
    TEST_ASSERT_EQUAL_INT(Q15_MIN, q15_sat(Q15_MIN));
    TEST_ASSERT_EQUAL_INT(0, q15_sat(0));
}

static void test_add_sub_saturation(void)
{
    /* 加法饱和：0.5 + 0.6 = 1.1 -> 饱和至 +1.0 */
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_add(Q15_HALF, 20480));
    /* 减法饱和：-0.5 - 0.6 -> 饱和至 -1.0 */
    TEST_ASSERT_EQUAL_INT(Q15_MIN, q15_sub(-Q15_HALF, 20480));
    /* 常规加法 */
    TEST_ASSERT_EQUAL_INT(Q15_HALF, q15_add(Q15_HALF, 0));
}

static void test_abs_min(void)
{
    /* abs(-1.0) 必须安全返回 +1.0（补码溢出防护） */
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_abs(Q15_MIN));
    TEST_ASSERT_EQUAL_INT(Q15_HALF, q15_abs(-Q15_HALF));
}

static void test_mul(void)
{
    /* 0.5 * 0.5 = 0.25 */
    TEST_ASSERT_EQUAL_INT(8192, q15_mul(Q15_HALF, Q15_HALF));
    /* 0.5 * -1.0 = -0.5 */
    TEST_ASSERT_EQUAL_INT(-Q15_HALF, q15_mul(Q15_HALF, Q15_MIN));
    /* -1.0 * -1.0 = 1.0 -> 乘积 Q30 为 0x40000000，右移后恰好 32768，饱和 */
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_mul(Q15_MIN, Q15_MIN));
    /* 乘 0 */
    TEST_ASSERT_EQUAL_INT(0, q15_mul(Q15_MAX, 0));
}

static void test_div(void)
{
    /* 0.5 / 0.5 = 1.0（截断） */
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_div(Q15_HALF, Q15_HALF));
    /* 1.0 / 0.5 = 2.0 -> 饱和至 +1.0 */
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_div(Q15_MAX, Q15_HALF));
    /* 除以 -1.0：符号翻转 */
    TEST_ASSERT_EQUAL_INT(-Q15_HALF, q15_div(Q15_HALF, Q15_MIN));
    /* 除以 0：被除数 >= 0 饱和至正极值 */
    TEST_ASSERT_EQUAL_INT(Q15_MAX, q15_div(Q15_HALF, 0));
    TEST_ASSERT_EQUAL_INT(Q15_MIN, q15_div(-Q15_HALF, 0));
}

int test_q15_math(void)
{
    TF_BEGIN();
    test_saturation();
    test_add_sub_saturation();
    test_abs_min();
    test_mul();
    test_div();
    TF_REPORT("q15_math");
}
