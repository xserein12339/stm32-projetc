/**
 * @file    bsp_dbg.h
 * @brief   调试输出通道 BSP 层接口（USART2 / PA2）v2.0
 *
 * 经 USART2 TX（PA2，115200 8N1）输出日志 / printf / HardFault 取证，
 * 上位机串口调试助手直接查看。寄存器级轮询 TX，无锁无缓冲，
 * HardFault 上下文安全。
 *
 * 参考文档：RM0008 Rev 21 §27 (USART)
 * @author  xserein
 * @version v2.0
 */

#ifndef __BSP_DBG_H__
#define __BSP_DBG_H__

#include <stdint.h>
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   初始化日志通道（PA2 AF_PP + USART2 115200 8N1 轮询 TX）
 *
 * @retval  BSP_OK       成功
 * @retval  BSP_ERR_PARAM 波特率计算异常（PCLK1 为 0）
 *
 * @note    必须在系统时钟配置之后调用（BRR 依赖 PCLK1）。
 *          USART2 为日志专用：勿经 bsp_uart 框架打开（中断/DMA 冲突）。
 */
bsp_err_t bsp_dbg_init(void);

/**
 * @brief   日志通道写入（字节流，TXE 轮询）
 *
 * @param[in] data 数据缓冲区
 * @param[in] len  字节数
 * @return  实际写入字节数（通道未就绪时返回 0）
 *
 * @note    HardFault / ISR / 任务上下文均可调用：纯寄存器轮询，无锁。
 *          返回前等待 TC，确保字节真正上线（返回后立刻崩溃不丢字节）。
 */
int32_t bsp_dbg_write(const char *data, uint32_t len);

/**
 * @brief   打印复位原因（RCC->CSR raw hex，零 libc 依赖）
 * @note    在系统稳定后的窗口调用（非 bsp_dbg_init 内：
 *          早期打印现场曾 12 字节处停死，见 board_v1.c 二分诊断注释）
 */
void bsp_dbg_report_reset_cause(void);

/**
 * @brief   日志 sink 包装（签名匹配 mw_log_sink_fn_t，供 mw_log 注入）
 * @param[in] buf 格式化完成的日志字符串
 * @param[in] len 字符串长度
 * @note    即 bsp_dbg_write 的适配器；任务上下文调用。
 */
void bsp_dbg_sink(const char *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DBG_H__ */
