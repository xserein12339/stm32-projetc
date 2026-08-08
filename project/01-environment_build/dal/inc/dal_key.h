/**
 * @file    dal_key.h
 * @brief   按键设备抽象层
 * 
 * @author xserein
 * @version v1.0
 */
#ifndef __DAL_KEY_H__
#define __DAL_KEY_H__

#include <stdint.h>
#include <stdbool.h>
#include "dal_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                               类型前向声明                                    */
/* ========================================================================== */

typedef struct dal_key_dev dal_key_dev_t; ///< 按键设备实例结构体前向声明

/* ========================================================================== */
/*                             纯硬件事件定义                                    */
/* ========================================================================== */

/**
 * @brief 原始按键物理事件枚举（不含任何策略语义）
 */
typedef enum {
    DAL_KEY_RAW_DOWN = 0x01, ///< 硬件确认按下（电平变化，极性由DRV层定义）
    DAL_KEY_RAW_UP   = 0x02, ///< 硬件确认释放
} dal_key_raw_event_t;

/* ========================================================================== */
/*                           自检结果码定义                                      */
/* ========================================================================== */

/**
 * @brief 按键设备专用自检结果码
 * @note  独立于通用 dal_err_t，避免与正常业务返回值混淆
 */
typedef enum {
    DAL_KEY_SELFTEST_PASS     = 0x00, ///< 自检通过，硬件电气与驱动均正常
    DAL_KEY_SELFTEST_ERR_PIN  = 0x01, ///< 引脚电气异常（开短路/上拉失效）
    DAL_KEY_SELFTEST_ERR_DRV  = 0x02, ///< 驱动上下文未就绪或内部状态损坏
    DAL_KEY_SELFTEST_NOT_IMPL = 0xFF, ///< 当前设备不支持自检功能
} dal_key_selftest_result_t;

/* ========================================================================== */
/*                              回调函数定义                                     */
/* ========================================================================== */

/* ========================================================================== */
/*                        设备操作集（DRV层核心契约）                             */
/* ========================================================================== */

/**
 * @brief 按键硬件抽象操作接口（多实例版）
 * @note  由BSP/DRV层实现并绑定到设备实例。
 *        DAL框架通过此ops操作硬件，绝不直接访问drv_priv。
 */
typedef struct {
    /* --- 生命周期管理 --- */
    /**
     * @brief 初始化按键硬件与驱动上下文
     * @param dev 设备实例指针
     * @retval DAL_OK              初始化成功
     * @retval DAL_ERR_BUSY        设备已初始化，禁止重复调用
     * @retval DAL_ERR_DEPENDENCY  底层GPIO/EXTI等硬件资源不可用
     * @retval DAL_ERR_FAIL        其他底层初始化失败
     *
     * @note 此函数应完成GPIO初始化、中断配置等硬件操作，
     *       但不应使能中断（使能由start负责）。
     */
    dal_err_t (*init)(dal_key_dev_t *dev);

    /**
     * @brief 反初始化，释放硬件资源并复位内部状态
     * @param dev 设备实例指针
     * @retval DAL_OK            释放成功
     * @retval DAL_ERR_NOT_READY 设备未初始化
     * @retval DAL_ERR_BUSY      设备仍在运行中，需先调用stop
     * @retval DAL_ERR_FAIL      底层资源释放失败
     */
    dal_err_t (*deinit)(dal_key_dev_t *dev);

    /* --- 诊断与测试 --- */

    /**
     * @brief 非破坏性硬件自检（不影响正常扫描状态）
     * @param dev    设备实例指针
     * @param result [出参] 自检详细结果码
     * @retval DAL_OK 自检流程执行完成（具体健康状态查看result参数）
     *
     * @note 自检应尽可能不干扰当前运行状态。
     *       若设备正在扫描中，可返回DAL_ERR_BUSY，
     *       或实现为不影响功能的轻量级检测。
     */
    dal_err_t (*selftest)(dal_key_dev_t *dev, dal_key_selftest_result_t *result);

    /* --- 状态查询 --- */
    /**
     * @brief 获取当前按键状态 
     * @param dev   设备实例指针
     * @param state [出参] 1=按下(DOWN), 0=释放(UP)
     * @retval DAL_OK            读取成功
     * @retval DAL_ERR_NOT_READY 设备未初始化
     */
    dal_err_t (*get_state)(dal_key_dev_t *dev, int *state);

} dal_key_ops_t;

/* ========================================================================== */
/*                       设备实例结构体（多实例核心描述符）                        */
/* ========================================================================== */

/**
 * @brief 按键设备实例描述符
 *
 * @note 实例化要求：
 *       - 必须是全局或静态存储期（不允许栈上分配）
 *       - 注册前需填充：name, ops, drv_priv
 *       - 其余字段由DAL框架管理，DRV/SVC层禁止直接修改
 */
struct dal_key_dev {
    /* === 由BSP层在注册前填充 === */
    const char           *name;     ///< 全局唯一设备标识名（如 "pwr_key", "vol_up"）
    const dal_key_ops_t  *ops;      ///< 该实例绑定的硬件操作集指针
    void                 *drv_priv; ///< DRV层私有数据（如指向包含GPIO pin的结构体）
    
     /* === 由DAL框架管理，DRV/SVC层禁止直接修改 === */
    bool                  initialized;   ///< 硬件是否已完成 
};

/* ========================================================================== */
/*                     DAL框架公共API（任务上下文调用）                           */
/* ========================================================================== */
/**
 * @brief 注册按键设备实例到DAL框架
 * @param dev 预先分配并填充好的设备实例指针（必须为全局或静态生命周期）
 * @retval DAL_OK                注册成功
 * @retval DAL_ERR_PARAM_INVALID dev为空，或name/ops为空，或magic不匹配
 * @retval DAL_ERR_BUSY          同名设备已注册
 * @retval DAL_ERR_FAIL          框架内部资源分配失败
 */
dal_err_t dal_key_register(dal_key_dev_t *dev);

/**
 * @brief 从DAL框架注销设备实例
 * @param dev 设备实例指针
 * @retval DAL_OK                注销成功
 * @retval DAL_ERR_PARAM_INVALID dev为空
 * @retval DAL_ERR_NOT_READY     设备未注册
 * @retval DAL_ERR_BUSY          设备正在运行中（需先调用 dal_key_stop）
 */
dal_err_t dal_key_unregister(dal_key_dev_t *dev);

/**
 * @brief 按名称查找已注册的按键设备实例
 * @param name 唯一设备标识名
 * @return 设备实例指针；若不存在或name为空则返回NULL
 */
dal_key_dev_t* dal_key_get_dev(const char *name);

/**
 * @brief 初始化设备硬件
 * @param dev 设备实例指针
 * @retval DAL_OK 初始化成功
 *
 * @note 内部调用 ops->init()，并设置初始化标志。
 */
dal_err_t dal_key_init(dal_key_dev_t *dev);

/**
 * @brief 反初始化设备硬件
 * @param dev 设备实例指针
 * @retval DAL_OK 释放成功
 *
 * @note 若设备仍在运行，会先自动调用 stop。
 */
dal_err_t dal_key_deinit(dal_key_dev_t *dev);

/**
 * @brief 执行硬件自检
 * @param dev    设备实例指针
 * @param result [出参] 自检详细结果码
 * @retval DAL_OK 自检流程执行完成（具体结果查看result参数）
 */
dal_err_t dal_key_selftest(dal_key_dev_t *dev, dal_key_selftest_result_t *result);

/**
 * @brief 获取当前电平状态
 * @param dev   设备实例指针
 * @param state [出参] 1=按下, 0=释放
 * @retval DAL_OK 读取成功
 */
dal_err_t dal_key_get_state(dal_key_dev_t *dev, int *state);

#ifdef __cplusplus
}
#endif

#endif /* __DAL_KEY_H__ */
