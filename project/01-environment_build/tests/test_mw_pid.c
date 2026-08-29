/**
 * @file    test_mw_pid.c
 * @brief   mw_pid 单元测试（P/I/D 各通道、抗饱和、限幅、reset）
 *
 * @author  xserein
 * @version v1.1
 */
#include "test_framework.h"
#include "mw_pid.h"

/* 便捷参数构造：P / I / D 增益 */
static mw_pid_params_t pid_params(q15_t kp, q15_t ki, q15_t kd,
                                  q15_t imin, q15_t imax)
{
    mw_pid_params_t p = {0};
    p.Kp = kp;
    p.Ki = ki;
    p.Kd = kd;
    p.out_min = Q15_MIN;
    p.out_max = Q15_MAX;
    p.integral_min = imin;
    p.integral_max = imax;
    p.d_filter_coef = 0;
    return p;
}

static void test_p_channel(void)
{
    mw_pid_t pid;
    mw_pid_params_t p = pid_params(Q15_HALF, 0, 0, Q15_MIN, Q15_MAX);
    mw_pid_init(&pid, &p);

    /* err = 0.5 - 0 = 0.5, P = 0.5 * 0.5 = 0.25（截断差 1 LSB 内） */
    q15_t out = mw_pid_update(&pid, Q15_HALF, 0);
    TEST_ASSERT_INT_WITHIN(1, 8192, out);
}

static void test_i_channel_accumulation(void)
{
    mw_pid_t pid;
    /* Ki = 1.0：i_term 直接反映误差累积 */
    mw_pid_params_t p = pid_params(0, Q15_ONE, 0, Q15_MIN, Q15_MAX);
    mw_pid_init(&pid, &p);

    /* 固定误差 0.25；第 n 拍 i_term = (n-1)*0.25（先算后累加） */
    q15_t out1 = mw_pid_update(&pid, 8192, 0);
    TEST_ASSERT_EQUAL_INT(0, out1);          /* 第 1 拍：积分器尚为 0 */

    q15_t out2 = mw_pid_update(&pid, 8192, 0);
    TEST_ASSERT_INT_WITHIN(1, 8192, out2);   /* 第 2 拍：累积 1 个误差 */

    q15_t out5 = 0;
    for (int i = 0; i < 3; i++) {
        out5 = mw_pid_update(&pid, 8192, 0);
    }
    /* 第 5 拍：累积 4 个误差 = 1.0（Q15_ONE=32767 截断差 1 LSB 内） */
    TEST_ASSERT_INT_WITHIN(1, Q15_ONE, out5);

    /* 继续累加：输出饱和在 +1.0 附近 */
    (void)mw_pid_update(&pid, 8192, 0);
    TEST_ASSERT_INT_WITHIN(1, Q15_ONE, mw_pid_update(&pid, 8192, 0));
}

static void test_d_on_measurement_impl(void)
{
    mw_pid_t pid;
    mw_pid_params_t p = pid_params(0, 0, Q15_ONE, Q15_MIN, Q15_MAX);
    mw_pid_init(&pid, &p);

    /* 反馈跳变：D = Kd * (prev - fb)，prev=0, fb=-0.25 -> +0.25 */
    q15_t out = mw_pid_update(&pid, 0, -8192);
    TEST_ASSERT_INT_WITHIN(1, 8192, out);

    /* setpoint 突变、反馈不变：D = 0（DoM 核心特性，微分零冲击） */
    mw_pid_t pid2;
    mw_pid_init(&pid2, &p);
    (void)mw_pid_update(&pid2, 0, 8192);          /* 建立历史 prev_feedback=0.25 */
    q15_t out2 = mw_pid_update(&pid2, Q15_HALF, 8192);
    TEST_ASSERT_EQUAL_INT(0, out2);               /* P/I/D 均 0 */
}

static void test_output_limit(void)
{
    mw_pid_t pid;
    mw_pid_params_t p = pid_params(Q15_ONE, 0, 0, Q15_MIN, Q15_MAX);
    p.out_min = -4096;    /* ±0.125 输出限幅 */
    p.out_max = 4096;
    mw_pid_init(&pid, &p);

    q15_t out = mw_pid_update(&pid, Q15_HALF, -Q15_HALF);  /* err = 1.0 */
    TEST_ASSERT_EQUAL_INT(4096, out);
}

static void test_antiwindup(void)
{
    mw_pid_t pid;
    /* P=1.0 强饱和 + I=1.0：观察积分器是否被冻结 */
    mw_pid_params_t p = pid_params(Q15_ONE, Q15_ONE, 0, Q15_MIN, Q15_MAX);
    p.out_min = -4096;
    p.out_max = 4096;
    mw_pid_init(&pid, &p);

    /* 持续正误差 1.0：输出饱和在 +4096，正误差加剧饱和 -> 积分冻结 */
    for (int i = 0; i < 100; i++) {
        (void)mw_pid_update(&pid, Q15_HALF, -Q15_HALF);
    }

    /* 反向误差 -1.0（setpoint=-0.5, feedback=+0.5）：
     * 积分被冻结在 0 附近 -> 输出仅由 P 决定，立刻翻到负饱和 */
    q15_t out = mw_pid_update(&pid, -Q15_HALF, Q15_HALF);
    TEST_ASSERT_EQUAL_INT(-4096, out);
}

static void test_integral_clamp(void)
{
    mw_pid_t pid;
    /* 积分限幅 ±0.25（纯误差累积 Q15 等效值） */
    mw_pid_params_t p = pid_params(0, Q15_ONE, 0, -8192, 8192);
    mw_pid_init(&pid, &p);

    for (int i = 0; i < 50; i++) {
        (void)mw_pid_update(&pid, Q15_HALF, 0);
    }
    /* i_term 应被钳位在 0.25（8192，截断差 1 LSB 内） */
    q15_t out = mw_pid_update(&pid, Q15_HALF, 0);
    TEST_ASSERT_INT_WITHIN(1, 8192, out);
}

static void test_reset(void)
{
    mw_pid_t pid;
    mw_pid_params_t p = pid_params(0, Q15_ONE, 0, Q15_MIN, Q15_MAX);
    mw_pid_init(&pid, &p);

    (void)mw_pid_update(&pid, Q15_HALF, 0);
    (void)mw_pid_update(&pid, Q15_HALF, 0);   /* i_term = 0.5 */

    mw_pid_reset(&pid);                        /* 参数保留，状态清零 */

    q15_t out = mw_pid_update(&pid, Q15_HALF, 0);
    TEST_ASSERT_EQUAL_INT(0, out);             /* 积分器已清零 */

    /* 参数未丢：Ki 仍生效（第 3 拍 i = 2 个累积误差 = 1.0） */
    (void)mw_pid_update(&pid, Q15_HALF, 0);
    TEST_ASSERT_INT_WITHIN(1, Q15_ONE, mw_pid_update(&pid, Q15_HALF, 0));
}

int test_mw_pid(void)
{
    TF_BEGIN();
    test_p_channel();
    test_i_channel_accumulation();
    test_d_on_measurement_impl();
    test_output_limit();
    test_antiwindup();
    test_integral_clamp();
    test_reset();
    TF_REPORT("mw_pid");
}
