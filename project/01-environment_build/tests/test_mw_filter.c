/**
 * @file    test_mw_filter.c
 * @brief   mw_filter 单元测试（LPF1 收敛/MAVG 均值/Kalman1D 噪声抑制）
 *
 * @author  xserein
 * @version v1.0
 */
#include "test_framework.h"
#include "mw_filter.h"

static void test_lpf1_convergence(void)
{
    mw_filter_lpf1_t f;
    mw_filter_lpf1_init(&f, 6554);   /* α ≈ 0.2 */

    /* 阶跃输入 0.5：输出以 (1-α)^n 趋近，100 拍后误差 < 1% */
    q15_t out = 0;
    for (int i = 0; i < 100; i++) {
        out = mw_filter_lpf1_update(&f, Q15_HALF);
    }
    TEST_ASSERT_INT_WITHIN(328, Q15_HALF, out);

    /* 常数输入下输出稳定不变 */
    q15_t out2 = mw_filter_lpf1_update(&f, Q15_HALF);
    TEST_ASSERT_EQUAL_INT(out, out2);
}

static void test_lpf1_alpha_zero(void)
{
    mw_filter_lpf1_t f;
    mw_filter_lpf1_init(&f, 0);      /* α=0：完全保持初值 */

    q15_t out = mw_filter_lpf1_update(&f, Q15_HALF);
    TEST_ASSERT_EQUAL_INT(0, out);
}

static void test_lpf1_alpha_one(void)
{
    mw_filter_lpf1_t f;
    mw_filter_lpf1_init(&f, Q15_ONE);/* α=1：直通 */

    q15_t out = mw_filter_lpf1_update(&f, Q15_HALF);
    TEST_ASSERT_EQUAL_INT(Q15_HALF, out);
}

static void test_mavg_mean(void)
{
    mw_filter_mavg_t f;
    q15_t buf[4];
    mw_filter_mavg_init(&f, buf, 4);

    /* 窗口未满前：返回已写入样本的均值 */
    q15_t out1 = mw_filter_mavg_update(&f, 4096);
    TEST_ASSERT_EQUAL_INT(4096, out1);          /* 1 个样本 */

    (void)mw_filter_mavg_update(&f, 8192);
    (void)mw_filter_mavg_update(&f, 12288);
    q15_t out4 = mw_filter_mavg_update(&f, 16384);  /* 窗口满：均值 0.25*4 样本 */
    TEST_ASSERT_EQUAL_INT(10240, out4);

    /* 滑动：丢最老样本 4096，进 16384，均值 = (8192+12288+16384+16384)/4 */
    q15_t out5 = mw_filter_mavg_update(&f, 16384);
    TEST_ASSERT_EQUAL_INT((8192 + 12288 + 16384 + 16384) / 4, out5);
}

static void test_mavg_reset(void)
{
    mw_filter_mavg_t f;
    q15_t buf[4];
    mw_filter_mavg_init(&f, buf, 4);

    (void)mw_filter_mavg_update(&f, Q15_HALF);
    mw_filter_mavg_reset(&f);

    /* 重置后重新从单样本开始 */
    q15_t out = mw_filter_mavg_update(&f, 2048);
    TEST_ASSERT_EQUAL_INT(2048, out);
}

static void test_kalman1d_noise(void)
{
    mw_filter_kalman1d_t f;
    /* q=0.001, r=0.1：强平滑（角度滤波典型配置） */
    mw_filter_kalman1d_init(&f, 33, 3277, 0);

    /* 确定性伪随机（LCG，可复现）：均值 0.5、幅度 ±0.05 噪声 */
    uint32_t seed = 12345U;
    int32_t sum_raw = 0, sum_est = 0;
    const int32_t mean = 16384;
    const int32_t amp  = 1638;
    const int N = 500;

    for (int i = 0; i < N; i++) {
        seed = seed * 1664525U + 1013904223U;
        int32_t noise = (int32_t)((seed >> 28) & 0xFU) - 8;  /* -8..7 */
        int32_t z = mean + (noise * amp) / 8;

        q15_t est = mw_filter_kalman1d_update(&f, (q15_t)z);
        if (i >= N / 2) {   /* 抛弃收敛期 */
            sum_raw += z;
            sum_est += est;
        }
    }

    /* 估计均值应明显更接近真值 0.5（滤波后方差更小） */
    TEST_ASSERT_INT_WITHIN(300, mean, (int32_t)(sum_est / (N / 2)));
    TEST_ASSERT_INT_WITHIN(600, mean, (int32_t)(sum_raw / (N / 2)));
}

static void test_kalman1d_convergence(void)
{
    mw_filter_kalman1d_t f;
    mw_filter_kalman1d_init(&f, 328, 3277, 0);

    /* 常值测量 0.5：200 拍内收敛到真值附近 */
    q15_t out = 0;
    for (int i = 0; i < 200; i++) {
        out = mw_filter_kalman1d_update(&f, Q15_HALF);
    }
    TEST_ASSERT_INT_WITHIN(600, Q15_HALF, out);
}

int test_mw_filter(void)
{
    TF_BEGIN();
    test_lpf1_convergence();
    test_lpf1_alpha_zero();
    test_lpf1_alpha_one();
    test_mavg_mean();
    test_mavg_reset();
    test_kalman1d_noise();
    test_kalman1d_convergence();
    TF_REPORT("mw_filter");
}
