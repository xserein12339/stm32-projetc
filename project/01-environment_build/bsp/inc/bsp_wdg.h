/**
 * @file    bsp_wdg.h
 * @brief   独立看门狗（IWDG）BSP 层接口
 *
 * IWDG 时钟源为 LSI（约 40kHz，F1 无出厂校准，-30%~+60% 偏差），
 * 递减到 0 触发系统复位。本模块提供 init / refresh 两个原语，
 * 喂狗职责由上层（svc_monitor 经注入的回调）承担。
 *
 * 参考文档：RM0008 Rev 21 §18 独立看门狗 (IWDG)
 * @author  xserein
 * @version v1.0
 */

#ifndef __BSP_WDG_H__
#define __BSP_WDG_H__

#include <stdint.h>
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 默认超时 500ms（监控任务周期 100ms，容忍连续 4 次漏喂才复位） */
#define BSP_WDG_DEFAULT_TIMEOUT_MS  (500U)

/**
 * @brief   启动独立看门狗（写入 KR=0xCCCC 后无法停止，直至复位）
 *
 * LSI ≈ 40kHz，预分频 64 -> 计数时钟 ≈ 625Hz（1.6ms/tick），
 * 超时 500ms 对应重装值 312。实际超时受 LSI 偏差影响（350~640ms）。
 *
 * @param[in] timeout_ms 超时时间（ms），实际值按 1.6ms 粒度向上取整
 *
 * @retval  BSP_OK          启动成功
 * @retval  BSP_ERR_PARAM   timeout_ms 超出可配置范围（> ~10.5s）
 * @retval  BSP_ERR_FAIL    HAL 初始化失败
 *
 * @note  一旦启动不可关闭（硬件特性，KR 只能写 0xAAAA/0xCCCC）；
 *        调用后必须保证周期性 bsp_wdg_refresh()，否则系统复位。
 * @warning 阻塞式等待 LSI 就绪（LSIRDLY），须在任务/主线程上下文调用。
 */
bsp_err_t bsp_wdg_init(uint32_t timeout_ms);

/**
 * @brief   喂狗（重装计数器，写 KR=0xAAAA）
 * @retval  BSP_OK 成功（未初始化时静默返回，便于测试桩）
 * @note    ISR 安全（单寄存器写入）；调用前须已 bsp_wdg_init()。
 */
bsp_err_t bsp_wdg_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WDG_H__ */
