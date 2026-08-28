/**
 * @file    svc_err.h
 * @brief   服务层（svc）公共错误码定义
 *
 * 服务层统一错误码约定（与 bsp_err / dal_err 风格一致）：
 *   - 0 表示成功，负数表示失败
 *   - 各服务模块的错误语义优先复用本枚举，模块私有语义再自行扩展
 *
 * @author  xserein
 * @version v1.0
 */
#ifndef __SVC_ERR_H__
#define __SVC_ERR_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 服务层错误码
 */
typedef enum {
    SVC_OK            = 0,   ///< 成功
    SVC_ERR_FAIL      = -1,  ///< 通用失败
    SVC_ERR_PARAM     = -2,  ///< 参数非法
    SVC_ERR_NOT_INIT  = -3,  ///< 服务未初始化
    SVC_ERR_BUSY      = -4,  ///< 服务忙（如校准进行中 / 任务已启动）
    SVC_ERR_DEV       = -5,  ///< 下层设备不可用（DAL 设备获取/操作失败）
    SVC_ERR_TIMEOUT   = -6,  ///< 超时
    SVC_ERR_STATE     = -7,  ///< 当前状态不允许该操作
} svc_err_t;

/**
 * @brief  判断服务层返回值是否成功
 */
static inline bool svc_is_ok(svc_err_t err)
{
    return (err == SVC_OK);
}

/**
 * @brief  判断服务层返回值是否失败
 */
static inline bool svc_is_err(svc_err_t err)
{
    return (err != SVC_OK);
}

#ifdef __cplusplus
}
#endif

#endif /* __SVC_ERR_H__ */
