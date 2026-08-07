/**
 * @file bsp_err.h
 * @brief BSP层统一错误码定义
 * 
 * @author xserein
 * @version v1.0
 */

#ifndef BSP_ERR_H
#define BSP_ERR_H

#include <stdint.h>

/**
 * @brief BSP 全局状态码
 */
typedef enum {
    BSP_OK          = 0,    ///< 成功 
    BSP_ERR_PARAM   = -1,   ///< 参数非法 
    BSP_ERR_BUSY    = -2,   ///< 资源忙/未就绪 
    BSP_ERR_TIMEOUT = -3,   ///< 超时 
    BSP_ERR_IO      = -4,   ///< 底层通信失败(NACK/CRC/总线错误) 
    BSP_ERR_NOMEM   = -5,   ///< 内存/缓冲区不足 
    BSP_ERR_NOT_INIT= -6,   ///< 模块未初始化 
} bsp_err_t;

#endif