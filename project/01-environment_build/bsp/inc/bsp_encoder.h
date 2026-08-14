/**
 * @file    bsp_encoder.h
 * @brief   板级编码器 BSP 层接口
 * @note    本文件声明板级编码器初始化函数，所有编码器实例在内部静态管理。
 *          支持多个编码器，配置在 bsp_encoder.c 中的数组内定义。
 * @author  xserein
 * @version v1.0
 */

#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 编码器初始化并注册到 DAL 框架
 * @retval BSP_OK          成功
 * @retval BSP_ERR_FAIL    注册失败（回滚后返回）
 */
bsp_err_t bsp_encoder_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ENCODER_H__ */