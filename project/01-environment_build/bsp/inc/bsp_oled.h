/**
 * @file    bsp_oled.h
 * @brief   OLED BSP 公共接口（SSD1306 128x64 I2C）v2.1
 * @author  xserein
 * @version v2.1
 */
#ifndef __BSP_OLED_H__
#define __BSP_OLED_H__

#include "bsp_err.h"
#include "dal_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 注册 OLED 到 DAL 框架（幂等） */
bsp_err_t bsp_oled_init(void);

/**
 * @brief 局部刷新（非标准 DAL 扩展，脏矩形优化）
 * @param x 起始列（必须为 8 的倍数）
 * @param y 起始行
 * @param w 宽度（必须为 8 的倍数，且 > 0）
 * @param h 高度（且 > 0）
 * @note  供上层直接调用以实现脏矩形优化。
 *        标准 DAL 接口仍通过 flush() 全屏刷新。
 */
dal_err_t bsp_oled_flush_region(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_OLED_H__ */
