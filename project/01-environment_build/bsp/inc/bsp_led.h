/**
 * @file bsp_led.h
 * @brief 板级 LED BSP 层接口
 * @note    本文件声明板级 LED 初始化函数，所有 LED 实例在内部静态管理。
 *          支持多个 LED，配置在 bsp_led.c 中的数组内定义。
 * @author xserein
 * @version v1.0
 */

#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化所有板载 LED 设备并注册到 DAL 框架
 * @note  应在 FreeRTOS 调度器启动前调用（如 bsp_init 阶段）
 * @retval BSP_OK         全部注册成功
 * @retval BSP_ERR_FAIL   注册失败（首个失败即停止）
 */
bsp_err_t bsp_led_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LED_H__ */