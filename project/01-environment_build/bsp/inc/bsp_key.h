/**
 * @file bsp_key.h
 * @brief 板级按键 BSP 层接口
 * @note    本文件声明板级按键初始化函数，所有按键实例在内部静态管理。
 *          支持多个按键，配置在 bsp_key.c 中的数组内定义。
 * @author xserein
 * @version v1.0
 */

#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

#include <stdint.h>
#include <stdbool.h>
#include "board_v1_config.h"
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 按键初始化与注册
 */
bsp_err_t bsp_key_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_KEY_H__ */