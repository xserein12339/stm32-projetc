/**
 * @file    mw_log.c
 * @brief   中间件日志模块实现 v1.0
 *
 * 格式："[E][tag] message\n"。级别/标签在本地 buf 内拼装后
 * 经 sink 输出；线程安全由 sink 侧保证（见 mw_log.h 契约）。
 *
 * @author  xserein
 * @version v1.0
 */
#include "mw_log.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/** 绑定的输出通道（NULL = 关闭输出） */
static mw_log_sink_fn_t s_sink;

/** 级别字符表（索引即级别值） */
static const char s_level_char[4] = { 'E', 'W', 'I', 'D' };

int32_t mw_log_init(mw_log_sink_fn_t sink)
{
    s_sink = sink;
    return 0;
}

mw_log_sink_fn_t mw_log_get_sink(void)
{
    return s_sink;
}

void mw_log_write(uint32_t level, const char *tag,
                  const char *fmt, ...)
{
    if (s_sink == NULL || fmt == NULL || level > MW_LOG_LEVEL_DEBUG) {
        return;
    }

    char buf[MW_LOG_BUF_SIZE];
    int32_t pos = 0;

    /* 前缀 [X][tag] */
    if (tag != NULL) {
        pos = snprintf(buf, sizeof(buf), "[%c][%s] ",
                       s_level_char[level], tag);
    } else {
        pos = snprintf(buf, sizeof(buf), "[%c] ", s_level_char[level]);
    }
    if (pos < 0) {
        return;
    }

    /* 正文（截断保护） */
    va_list ap;
    va_start(ap, fmt);
    int32_t body = vsnprintf(&buf[pos], (size_t)(sizeof(buf) - (size_t)pos),
                             fmt, ap);
    va_end(ap);
    if (body < 0) {
        return;
    }
    pos += (body > (int32_t)sizeof(buf) - pos - 2)
           ? (int32_t)sizeof(buf) - pos - 2 : body;

    buf[pos++] = '\n';
    s_sink(buf, (uint32_t)pos);
}
