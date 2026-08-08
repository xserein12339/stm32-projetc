/**
 * @file    dal_err.h
 * @brief   设备抽象层错误码
 * 
 * @author xserein
 * @version v1.0
 */
#ifndef __DAL_ERR_H__
#define __DAL_ERR_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DAL层设备通用错误码
 */
typedef enum {
    DAL_OK                =  0,     ///< 成功
    DAL_ERR_FAIL          = -1,     ///< 通用失败(兜底)
    DAL_ERR_NOT_READY     = -2,     ///< 设备未就绪/未初始化
    DAL_ERR_BUSY          = -3,     ///< 设备忙
    DAL_ERR_TIMEOUT       = -4,     ///< 操作超时
    DAL_ERR_PARAM_INVALID = -5,     ///< 参数非法
    DAL_ERR_DATA_INVALID  = -6,     ///< 数据无效(NaN/超量程)
    DAL_ERR_ALGO_DIVERGE  = -7,     ///< 算法发散/数值异常
    DAL_ERR_DEPENDENCY    = -8,     ///< 依赖的底层设备不可用
    DAL_ERR_NOT_FOUND     = -9,     ///< 未找到指定项
    DAL_ERR_DUPLICATE     = -10,    ///< 重复注册/已存在
    DAL_ERR_FULL          = -11,    ///< 资源已满（无法再添加）
    DAL_ERR_NO_MEM        = -12,    ///< 内存不足（若驱动使用动态分配）
    DAL_ERR_NOT_SUPPORTED = -13,    ///< 不支持的操作
    DAL_ERR_ALREADY_INIT  = -14,    ///< 已经初始化
} dal_err_t;

#ifdef __cplusplus
}
#endif

#endif /* __DAL_ERR_H__ */