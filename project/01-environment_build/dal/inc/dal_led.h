/**
 * @file    dal_led.h
 * @brief   led设备抽象层
 * 
 * @author xserein
 * @version v1.0
 */
#ifndef __DAL_LED_H__
#define __DAL_LED_H__

#include <stdint.h>
#include <stdbool.h>
#include "dal_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                               类型前向声明                                  */
/* ========================================================================== */

typedef struct dal_led_dev dal_led_dev_t; ///< LED设备实例结构体前向声明

/* ========================================================================== */
/*                              LED状态定义                                    */
/* ========================================================================== */

/**
 * @brief LED 逻辑状态枚举
 */
typedef enum {
    DAL_LED_OFF = 0,   ///< 熄灭
    DAL_LED_ON  = 1,   ///< 点亮
} dal_led_state_t;

/* ========================================================================== */
/*                           自检结果码定义                                     */
/* ========================================================================== */

/**
 * @brief LED设备专用自检结果码
 */
typedef enum {
    DAL_LED_SELFTEST_PASS     = 0x00, ///< 自检通过
    DAL_LED_SELFTEST_ERR_PIN  = 0x01, ///< 引脚电气异常（开短路/上拉失效）
    DAL_LED_SELFTEST_ERR_DRV  = 0x02, ///< 驱动上下文未就绪或内部状态损坏
    DAL_LED_SELFTEST_NOT_IMPL = 0xFF, ///< 当前设备不支持自检功能
} dal_led_selftest_result_t;

/* ========================================================================== */
/*                        设备操作集（DRV层核心契约）                            */
/* ========================================================================== */

/**
 * @brief LED硬件抽象操作接口（多实例版）
 * @note  由BSP/DRV层实现并绑定到设备实例。
 *        DAL框架通过此ops操作硬件，绝不直接访问drv_priv。
 */
typedef struct {
    /* --- 生命周期管理 --- */
    /**
     * @brief 初始化LED硬件与驱动上下文
     * @param dev 设备实例指针
     * @retval DAL_OK              初始化成功
     * @retval DAL_ERR_BUSY        设备已初始化，禁止重复调用
     * @retval DAL_ERR_DEPENDENCY  底层GPIO/PWM等硬件资源不可用
     * @retval DAL_ERR_FAIL        其他底层初始化失败
     */
    dal_err_t (*init)(dal_led_dev_t *dev);

    /**
     * @brief 反初始化，释放硬件资源并复位内部状态
     * @param dev 设备实例指针
     * @retval DAL_OK            释放成功
     * @retval DAL_ERR_NOT_READY 设备未初始化
     * @retval DAL_ERR_BUSY      设备正在被占用（如正在进行PWM输出，需先停止）
     * @retval DAL_ERR_FAIL      底层资源释放失败
     */
    dal_err_t (*deinit)(dal_led_dev_t *dev);

    /* --- 诊断与测试 --- */
    /**
     * @brief 非破坏性硬件自检（不影响正常LED状态）
     * @param dev    设备实例指针
     * @param result [出参] 自检详细结果码
     * @retval DAL_OK 自检流程执行完成（具体健康状态查看result参数）
     */
    dal_err_t (*selftest)(dal_led_dev_t *dev, dal_led_selftest_result_t *result);

    /* --- 控制与状态查询 --- */
    /**
     * @brief 设置LED逻辑状态（开/关）
     * @param dev   设备实例指针
     * @param state 目标状态（DAL_LED_ON 或 DAL_LED_OFF）
     * @retval DAL_OK            设置成功
     * @retval DAL_ERR_NOT_READY 设备未初始化
     * @retval DAL_ERR_PARAM_INVALID state参数无效
     *
     * @note 【语义契约】
     *       - DAL_LED_ON/OFF 为逻辑状态，与硬件有效电平无关
     *       - BSP实现负责根据硬件极性(active_level)将逻辑状态映射为实际GPIO电平
     *       - 上层调用者无需关心LED是高电平点亮还是低电平点亮
     */
    dal_err_t (*set_state)(dal_led_dev_t *dev, dal_led_state_t state);

    /**
     * @brief 获取当前LED逻辑状态
     * @param dev   设备实例指针
     * @param state [出参] 当前状态
     * @retval DAL_OK            读取成功
     * @retval DAL_ERR_NOT_READY 设备未初始化
     */
    dal_err_t (*get_state)(dal_led_dev_t *dev, dal_led_state_t *state);

} dal_led_ops_t;

/* ========================================================================== */
/*                       设备实例结构体（多实例核心描述符）                       */
/* ========================================================================== */

/**
 * @brief LED设备实例描述符
 *
 * @note 实例化要求：
 *       - 必须是全局或静态存储期（不允许栈上分配）
 *       - 注册前需填充：name, ops, drv_priv
 *       - 其余字段由DAL框架管理，DRV/SVC层禁止直接修改
 */
struct dal_led_dev {
    /* === 由BSP层在注册前填充 === */
    const char           *name;     ///< 全局唯一设备标识名
    const dal_led_ops_t  *ops;      ///< 该实例绑定的硬件操作集指针
    void                 *drv_priv; ///< DRV层私有数据

    /* === 由DAL框架管理，DRV/SVC层禁止直接修改 === */
    bool                  initialized;   ///< 硬件是否已完成 init 
};

/* ========================================================================== */
/*                     DAL框架公共API（任务上下文调用）                           */
/* ========================================================================== */

/**
 * @brief 注册LED设备实例到DAL框架
 * @param dev 预先分配并填充好的设备实例指针（必须为全局或静态生命周期）
 * @retval DAL_OK                注册成功
 * @retval DAL_ERR_PARAM_INVALID dev为空，或name/ops为空
 * @retval DAL_ERR_BUSY          同名设备已注册
 * @retval DAL_ERR_FAIL          框架内部资源分配失败（如设备数超限）
 */
dal_err_t dal_led_register(dal_led_dev_t *dev);

/**
 * @brief 从DAL框架注销设备实例
 * @param dev 设备实例指针
 * @retval DAL_OK                注销成功
 * @retval DAL_ERR_PARAM_INVALID dev为空
 * @retval DAL_ERR_NOT_READY     设备未注册
 * @retval DAL_ERR_BUSY          设备正在运行中（需先调用 dal_led_deinit）
 */
dal_err_t dal_led_unregister(dal_led_dev_t *dev);

/**
 * @brief 按名称查找已注册的LED设备实例
 * @param name 唯一设备标识名
 * @return 设备实例指针；若不存在或name为空则返回NULL
 */
dal_led_dev_t* dal_led_get_dev(const char *name);

/**
 * @brief 初始化设备硬件
 * @param dev 设备实例指针
 * @retval DAL_OK 初始化成功
 *
 * @note 内部调用 ops->init()，并设置初始化标志。
 *       若设备已初始化，返回 DAL_ERR_BUSY。
 */
dal_err_t dal_led_init(dal_led_dev_t *dev);

/**
 * @brief 反初始化设备硬件
 * @param dev 设备实例指针
 * @retval DAL_OK 释放成功
 *
 * @note 内部调用 ops->deinit()，并清除初始化标志。
 *       若设备未初始化，返回 DAL_ERR_NOT_READY。
 */
dal_err_t dal_led_deinit(dal_led_dev_t *dev);

/**
 * @brief 执行硬件自检
 * @param dev    设备实例指针
 * @param result [出参] 自检详细结果码
 * @retval DAL_OK 自检流程执行完成（具体结果查看result参数）
 *
 * @note 若 ops->selftest 为 NULL，自动返回 DAL_LED_SELFTEST_NOT_IMPL。
 */
dal_err_t dal_led_selftest(dal_led_dev_t *dev, dal_led_selftest_result_t *result);

/**
 * @brief 设置LED状态（开/关）
 * @param dev   设备实例指针
 * @param state 目标状态（DAL_LED_ON/OFF）
 * @retval DAL_OK 设置成功
 */
dal_err_t dal_led_set_state(dal_led_dev_t *dev, dal_led_state_t state);

/**
 * @brief 获取LED当前状态
 * @param dev   设备实例指针
 * @param state [出参] 当前状态
 * @retval DAL_OK 读取成功
 */
dal_err_t dal_led_get_state(dal_led_dev_t *dev, dal_led_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* __DAL_LED_H__ */