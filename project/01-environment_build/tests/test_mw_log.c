/**
 * @file    test_mw_log.c
 * @brief   mw_log 单元测试（sink 注入、级别过滤、截断保护）
 *
 * @author  xserein
 * @version v1.0
 */
#include "test_framework.h"
#include "mw_log.h"
#include <string.h>

/** 测试 sink：捕获最近一条日志 */
static char    s_last_buf[128];
static uint32_t s_last_len;
static int     s_sink_calls;

static void test_sink(const char *buf, uint32_t len)
{
    memcpy(s_last_buf, buf, (len < sizeof(s_last_buf)) ? len : sizeof(s_last_buf) - 1U);
    s_last_buf[(len < sizeof(s_last_buf) - 1U) ? len : sizeof(s_last_buf) - 1U] = '\0';
    s_last_len = len;
    s_sink_calls++;
}

static void test_sink_injection(void)
{
    (void)mw_log_init(NULL);
    LOG_I("t", "should be dropped");
    TEST_ASSERT_EQUAL_INT(NULL != mw_log_get_sink(), 0);

    (void)mw_log_init(test_sink);
    TEST_ASSERT_EQUAL_INT(NULL != mw_log_get_sink(), 1);

    s_sink_calls = 0;
    LOG_I("tag", "hello %d", 42);
    TEST_ASSERT_EQUAL_INT(1, s_sink_calls);
    /* 格式：[I][tag] hello 42\n */
    TEST_ASSERT_TRUE(strstr(s_last_buf, "[I][tag] hello 42") != NULL);
    TEST_ASSERT_TRUE(s_last_len == strlen(s_last_buf));
    TEST_ASSERT_TRUE(s_last_buf[s_last_len - 1] == '\n');
}

static void test_level_chars(void)
{
    (void)mw_log_init(test_sink);

    LOG_E("t", "x");
    TEST_ASSERT_TRUE(s_last_buf[1] == 'E');
    LOG_W("t", "x");
    TEST_ASSERT_TRUE(s_last_buf[1] == 'W');
    LOG_I("t", "x");
    TEST_ASSERT_TRUE(s_last_buf[1] == 'I');
}

static void test_truncation(void)
{
    (void)mw_log_init(test_sink);

    /* 超长正文：截断到 MW_LOG_BUF_SIZE（含换行），不越界 */
    LOG_I("t", "%s",
          "012345678901234567890123456789012345678901234567890123456789"
          "012345678901234567890123456789012345678901234567890123456789");
    TEST_ASSERT_TRUE(s_last_len <= MW_LOG_BUF_SIZE);
    TEST_ASSERT_TRUE(s_last_buf[s_last_len - 1] == '\n');
}

static void test_null_tag(void)
{
    (void)mw_log_init(test_sink);

    LOG_I(NULL, "no tag");
    TEST_ASSERT_TRUE(strncmp(s_last_buf, "[I] ", 4) == 0);
}

int test_mw_log(void)
{
    TF_BEGIN();
    test_sink_injection();
    test_level_chars();
    test_truncation();
    test_null_tag();
    TF_REPORT("mw_log");
}
