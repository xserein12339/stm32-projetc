/**
 * @file    test_main.c
 * @brief   宿主机单元测试入口（middleware 全模块）
 *
 * 运行：cmake -S tests -B tests/build && cmake --build tests/build
 *       && ./tests/build/mw_tests
 *
 * @author  xserein
 * @version v1.0
 */
#include "test_framework.h"

/* 各模块 runner（返回 0=全部通过） */
int test_q15_math(void);
int test_mw_math(void);
int test_mw_pid(void);
int test_mw_filter(void);
int test_mw_log(void);

/* TF_BEGIN 复位用（框架统计变量定义在 runner 内共享，这里只做调度） */
unsigned long g_tf_pass;
unsigned long g_tf_fail;

int main(void)
{
    int failed_suites = 0;

    if (test_q15_math() != 0) { failed_suites++; }
    if (test_mw_math()   != 0) { failed_suites++; }
    if (test_mw_pid()    != 0) { failed_suites++; }
    if (test_mw_filter() != 0) { failed_suites++; }
    if (test_mw_log()    != 0) { failed_suites++; }

    printf("\n==== MW TESTS: %s (%d suite(s) failed) ====\n",
           (failed_suites == 0) ? "ALL PASS" : "FAILED", failed_suites);

    return (failed_suites == 0) ? 0 : 1;
}
