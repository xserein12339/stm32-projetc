/**
 * @file    bsp_oled.h
 * @brief   板级 OLED BSP 层接口（SSD1306，128x64，I2C）
 * @note    - 硬件映射引用 board_v1_config.h
 *          - 依赖 bsp_i2c 进行通信
 *          - 实现 dal_display_ops_t 并注册到 dal_display 框架
 * @author  xserein
 * @version v1.0
 */

#ifndef __BSP_OLED_H__
#define __BSP_OLED_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief OLED 初始化并注册到 DAL 框架
 * @retval BSP_OK          成功
 * @retval BSP_ERR_IO      I2C 通信失败或设备无响应
 * @retval BSP_ERR_FAIL    注册失败
 */
bsp_err_t bsp_oled_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_OLED_H__ */