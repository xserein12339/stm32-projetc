/**
 * @file bsp_err.h
 * @brief BSP层统一错误码定义
 * 
 * @author xserein
 * @version v1.0
 * 
 * @note 本文件为BSP层统一错误码定义，所有BSP模块应使用此错误码。
 */

#ifndef BSP_ERR_H
#define BSP_ERR_H

#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP 全局状态码
 * 
 * @note 0 表示成功，负数表示各种错误。
 *       所有 BSP 层函数应返回此枚举类型。
 */
typedef enum {
    BSP_OK              =  0,   ///< 成功 
    BSP_ERR_FAIL        = -1,   ///< 错误
    BSP_ERR_PARAM       = -2,   ///< 参数非法 
    BSP_ERR_BUSY        = -3,   ///< 资源忙/未就绪 
    BSP_ERR_TIMEOUT     = -4,   ///< 超时 
    BSP_ERR_IO          = -5,   ///< 底层通信失败(NACK/CRC/总线错误) 
    BSP_ERR_NOMEM       = -6,   ///< 内存/缓冲区不足 
    BSP_ERR_NOT_INIT    = -7,   ///< 模块未初始化 
    BSP_ERR_NOT_FOUND   = -8,   ///< 设备/资源未找到
    BSP_ERR_UNSUPPORT   = -9,   ///< 不支持的操作
    BSP_ERR_VERIFY      = -10,  ///< 校验失败(CRC/Checksum/ECC)
    BSP_ERR_CLOCK       = -11,  ///< 时钟错误
} bsp_err_t;

/**
 * @brief 检查错误码是否表示成功
 * @param e 错误码
 * @return true 成功，false 失败
 */
static inline bool bsp_is_ok(bsp_err_t e)
{
    return (e == BSP_OK);
}

/**
 * @brief 检查错误码是否表示失败
 * @param e 错误码
 * @return true 失败，false 成功
 */
static inline bool bsp_is_err(bsp_err_t e)
{
    return (e != BSP_OK);
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_ERR_H */