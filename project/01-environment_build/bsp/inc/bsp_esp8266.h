/**
 * @file    bsp_esp8266.h
 * @brief   板级 ESP8266 BSP 层接口（AT 指令）
 * @note    - 依赖 bsp_uart 进行 AT 指令与数据收发
 *          - 实现 dal_wifi_ops_t 并注册到 dal_wifi 框架
 *          - 支持 STA/AP 模式、扫描、连接、数据透传
 * @author  xserein
 * @version v1.0
 */

#ifndef __BSP_ESP8266_H__
#define __BSP_ESP8266_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 ESP8266 并注册到 DAL 框架
 * @note  自动初始化 UART、发送 AT 检测、设置模式等
 * @retval BSP_OK           成功
 * @retval BSP_ERR_IO       AT 通信失败或无响应
 * @retval BSP_ERR_FAIL     注册失败
 */
bsp_err_t bsp_esp8266_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ESP8266_H__ */