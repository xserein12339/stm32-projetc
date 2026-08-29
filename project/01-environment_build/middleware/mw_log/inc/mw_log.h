/**
 * @file    mw_log.h
 * @brief   中间件日志模块 v1.0（四级可裁剪 + 输出通道注入）
 *
 * 编译时裁剪：定义 MW_LOG_LEVEL（默认 INFO），低于该级别的日志
 * 宏展开为空（零代码体积、零 RAM、零周期开销）。
 *
 * 输出通道解耦：middleware 不依赖 HAL/RTOS，实际输出（ITM/SWO、
 * RTT、UART）由 app 在启动时经 mw_log_init() 注入 sink 函数。
 *
 * 用法：
 *   app 启动：mw_log_init(bsp_dbg_sink);   // 或其他通道包装
 *   模块内：  LOG_I("mot", "duty=%d", duty);
 *
 * 参考文档：本项目《开发手册》7 章（日志规范）
 * @author  xserein
 * @version v1.0
 */

#ifndef __MW_LOG_H__
#define __MW_LOG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  日志级别
 * ================================================================ */

#define MW_LOG_LEVEL_ERROR  0U
#define MW_LOG_LEVEL_WARN   1U
#define MW_LOG_LEVEL_INFO   2U
#define MW_LOG_LEVEL_DEBUG  3U

/** 编译时裁剪级别（构建系统可 -DMW_LOG_LEVEL=n 覆盖） */
#ifndef MW_LOG_LEVEL
#define MW_LOG_LEVEL        MW_LOG_LEVEL_INFO
#endif

/** 单条日志格式化缓冲区上限（含前缀与换行） */
#define MW_LOG_BUF_SIZE     (96U)

/* ================================================================
 *  类型定义
 * ================================================================ */

/**
 * @brief 日志输出通道函数原型（由 app 注入）
 * @param[in] buf 格式化完成的字符串（不含换行符）
 * @param[in] len 字符串长度
 * @note    在调用者任务上下文执行，须非阻塞；
 *          实现自行保证多任务串行化（如临界区）。
 */
typedef void (*mw_log_sink_fn_t)(const char *buf, uint32_t len);

/* ================================================================
 *  公开 API
 * ================================================================ */

/**
 * @brief   初始化日志模块并绑定输出通道
 * @param[in] sink 输出函数（NULL = 关闭输出）
 * @return  0 成功；-1 参数无效（保留语义，当前恒 0）
 *
 * @note    须在调度器启动前调用；sink 应为轻量非阻塞实现。
 */
int32_t mw_log_init(mw_log_sink_fn_t sink);

/**
 * @brief   获取当前绑定的输出通道（诊断用）
 * @return  sink 函数指针（未初始化为 NULL）
 */
mw_log_sink_fn_t mw_log_get_sink(void);

/**
 * @brief   日志写入核心函数（由宏调用，一般不直接使用）
 * @warning 栈上使用 MW_LOG_BUF_SIZE 字节缓冲 + vsnprintf 栈帧，
 *          小栈任务（<256 word）内慎用 DEBUG 级长格式。
 */
void mw_log_write(uint32_t level, const char *tag,
                  const char *fmt, ...);

/* ================================================================
 *  日志宏（编译时裁剪）
 * ================================================================ */

#if MW_LOG_LEVEL >= MW_LOG_LEVEL_ERROR
#define LOG_E(tag, fmt, ...) \
    mw_log_write(MW_LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_E(tag, fmt, ...)  ((void)0)
#endif

#if MW_LOG_LEVEL >= MW_LOG_LEVEL_WARN
#define LOG_W(tag, fmt, ...) \
    mw_log_write(MW_LOG_LEVEL_WARN, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_W(tag, fmt, ...)  ((void)0)
#endif

#if MW_LOG_LEVEL >= MW_LOG_LEVEL_INFO
#define LOG_I(tag, fmt, ...) \
    mw_log_write(MW_LOG_LEVEL_INFO, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_I(tag, fmt, ...)  ((void)0)
#endif

#if MW_LOG_LEVEL >= MW_LOG_LEVEL_DEBUG
#define LOG_D(tag, fmt, ...) \
    mw_log_write(MW_LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)
#else
#define LOG_D(tag, fmt, ...)  ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MW_LOG_H__ */
