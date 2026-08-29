/**
 * @file    app_hmi.h
 * @brief   人机界面模块 v1.0（OLED 文本渲染 + 刷新）
 *
 * 职责：把 app_ctrl 格式化好的文本行渲染到 SSD1306 128x64
 * （DAL 点阵 draw 接口，1bpp 行 Major 数据），并触发异步刷新。
 * 本模块不格式化业务数据，只负责"文本 -> 像素"。
 *
 * 资源策略（RAM 硬约束）：不创建独立任务，由 app_ctrl 任务
 * 周期调用（2Hz）；行缓冲为模块静态单份（单任务调用契约）。
 *
 * 参考文档：本项目《需求分析》FR-HMI-003
 * @author  xserein
 * @version v1.0
 */

#ifndef __APP_HMI_H__
#define __APP_HMI_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 单行可渲染字符数（6px/字符 × 21 = 126px） */
#define APP_HMI_LINE_CHARS      (21U)
/** 屏幕行数（8px/行 × 8 = 64px） */
#define APP_HMI_LINE_COUNT      (8U)

/* ================================================================
 *  公开 API
 * ================================================================ */

/**
 * @brief   初始化 HMI（获取 OLED 设备，不触发硬件初始化）
 * @return  0 成功；-1 设备不可用
 * @note    须在 bsp_init 之后调用；硬件首次初始化延迟到首次
 *          app_hmi_show()（任务上下文，避免调度器前阻塞 I2C）。
 */
int32_t app_hmi_init(void);

/**
 * @brief   渲染并刷新一屏文本
 *
 * @param[in] lines      文本行数组（每行 <= APP_HMI_LINE_CHARS 字符，超长截断）
 * @param[in] line_count 行数（<= APP_HMI_LINE_COUNT）
 *
 * @note    单任务调用契约（内部静态行缓冲非重入）；
 *          首次调用自动初始化 OLED 硬件；DMA 刷新进行中的
 *          draw 会返回 BUSY（本函数忽略，下一周期重画）。
 */
void app_hmi_show(const char *const *lines, uint8_t line_count);

#ifdef __cplusplus
}
#endif

#endif /* __APP_HMI_H__ */
