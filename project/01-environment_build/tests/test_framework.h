/**
 * @file    test_framework.h
 * @brief   宿主机单元测试极简框架（Unity 风格宏子集）
 *
 * 设计目标：零外部依赖（无网络拉取 Unity）、输出风格与 Unity 兼容
 * （PASS/FAIL 行 + 汇总）。仅用于宿主机（PC）侧测试，不参与固件构建。
 *
 * @author  xserein
 * @version v1.0
 */

#ifndef __TEST_FRAMEWORK_H__
#define __TEST_FRAMEWORK_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/** 测试统计（runner 汇总用） */
extern unsigned long g_tf_pass;
extern unsigned long g_tf_fail;

/** 每个用例文件在 RUN_TEST 前调用 */
#define TF_BEGIN()  do { g_tf_pass = 0; g_tf_fail = 0; } while (0)

#define TF_REPORT(name)  do { \
    printf("\n==== %s: %lu passed, %lu failed ====\n", \
           (name), g_tf_pass, g_tf_fail); \
    return (g_tf_fail == 0) ? 0 : 1; \
} while (0)

/** 断言：整数相等 */
#define TEST_ASSERT_EQUAL_INT(expected, actual)  TF_CHECK_INT((expected), (actual), __LINE__)
#define TF_CHECK_INT(exp, act, line)  do { \
    long long e_ = (long long)(exp), a_ = (long long)(act); \
    if (e_ == a_) { g_tf_pass++; } \
    else { g_tf_fail++; \
        printf("FAIL %s:%d: expected %lld, was %lld\n", __func__, line, e_, a_); } \
} while (0)

/** 断言：真/假 */
#define TEST_ASSERT_TRUE(cond)   TF_CHECK_BOOL((cond), 1, __LINE__)
#define TEST_ASSERT_FALSE(cond)  TF_CHECK_BOOL((cond), 0, __LINE__)
#define TF_CHECK_BOOL(cond, want, line)  do { \
    if (((cond) ? 1 : 0) == (want)) { g_tf_pass++; } \
    else { g_tf_fail++; \
        printf("FAIL %s:%d: expected %s\n", __func__, line, (want) ? "true" : "false"); } \
} while (0)

/** 断言：浮点近似相等（容差内） */
#define TEST_ASSERT_FLOAT_WITHIN(tol, expected, actual)  TF_CHECK_FLT((tol), (expected), (actual), __LINE__)
#define TF_CHECK_FLT(tol, exp, act, line)  do { \
    double e_ = (double)(exp), a_ = (double)(act), d_; \
    d_ = (a_ > e_) ? (a_ - e_) : (e_ - a_); \
    if (d_ <= (double)(tol)) { g_tf_pass++; } \
    else { g_tf_fail++; \
        printf("FAIL %s:%d: expected %f +- %f, was %f\n", \
               __func__, line, e_, (double)(tol), a_); } \
} while (0)

/** 断言：整数在范围内 */
#define TEST_ASSERT_INT_WITHIN(tol, expected, actual)  TF_CHECK_INTW((tol), (expected), (actual), __LINE__)
#define TF_CHECK_INTW(tol, exp, act, line)  do { \
    long long e_ = (long long)(exp), a_ = (long long)(act), t_ = (long long)(tol); \
    long long d_ = (a_ > e_) ? (a_ - e_) : (e_ - a_); \
    if (d_ <= t_) { g_tf_pass++; } \
    else { g_tf_fail++; \
        printf("FAIL %s:%d: expected %lld +-%lld, was %lld\n", \
               __func__, line, e_, t_, a_); } \
} while (0)

#endif /* __TEST_FRAMEWORK_H__ */
