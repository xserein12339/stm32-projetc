/**
 * @file    dal_key.h
 * @brief   按键设备抽象层 v2.0
 * 
 * @details 支持多按键设备独立管理，提供同步轮询状态查询，
 *          以及异步中断回调通知两种工作模式。
 *          回调在中断上下文中执行，调用者需严格遵守 ISR 安全契约。
 * 
 * @author xserein
 * @version v2.0
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
/*                            线程安全契约                                       */
/* ========================================================================== */

/**
 * @defgroup KEY_THREAD_SAFETY 线程安全说明
 * @{
 * - 本接口【非线程安全】，所有公共 API 默认不提供内部互斥保护。
 * - 同一设备的操作（如 get_state / set_callback / unregister）需由调用者
 *   自行保证串行化（如通过互斥锁或临界区）。
 * - 回调函数在中断上下文中执行，严禁调用任何阻塞 API。
 * - 跨设备操作无需互斥，设备间完全独立。
 * @}
 */

/* ========================================================================== */
/*                               类型前向声明                                    */
/* ========================================================================== */

typedef struct dal_key_dev dal_key_dev_t; ///< 按键设备实例结构体前向声明

/* ========================================================================== */
/*                             枚举与事件定义                                    */
/* ========================================================================== */

/**
 * @brief 按键电平状态
 */
typedef enum {
    DAL_KEY_LEVEL_RELEASED = 0, ///< 释放
    DAL_KEY_LEVEL_PRESSED  = 1, ///< 按下
} dal_key_level_t;

/**
 * @brief 按键异步事件类型
 */
typedef enum {
    DAL_KEY_EVT_DOWN = 0x01U, ///< 按下事件（去抖确认后）
    DAL_KEY_EVT_UP   = 0x02U, ///< 释放事件（去抖确认后）
} dal_key_event_t;

/**
 * @brief 按键设备专用自检结果码
 */
typedef enum {
    DAL_KEY_SELFTEST_PASS      = 0x00, ///< 自检通过
    DAL_KEY_SELFTEST_ERR_PIN   = 0x01, ///< 引脚电气异常
    DAL_KEY_SELFTEST_ERR_DRV   = 0x02, ///< 驱动上下文异常
    DAL_KEY_SELFTEST_NOT_IMPL  = 0xFF, ///< 不支持自检
} dal_key_selftest_result_t;

/**
 * @brief 按键异步事件回调函数原型
 * @param[in] dev       触发事件的设备实例指针
 * @param[in] event     事件类型（DOWN / UP）
 * @param[in] user_data 用户自定义上下文（由 set_callback 时传入）
 *
 * @warning 此回调在中断上下文中执行！
 *          - 严禁调用任何阻塞 API（如 delay、mutex_lock 等）
 *          - 严禁执行浮点运算或耗时操作
 *          - 严禁使用信号量/消息队列的【阻塞】发送接口
 *          - 如需将事件传递给任务，请使用非阻塞的 ISR 专用接口
 *            （如 FreeRTOS 的 xQueueSendFromISR / xSemaphoreGiveFromISR）
 *          - 违反以上约束将导致系统崩溃或实时性劣化
 */
typedef void (*dal_key_callback_t)(dal_key_dev_t *dev,
                                   dal_key_event_t event,
                                   void *user_data);

/* ========================================================================== */
/*                        设备操作集（DRV层核心契约）                             */
/* ========================================================================== */

/**
 * @brief 按键硬件抽象操作接口
 */
typedef struct {
    /* --- 生命周期管理 --- */
    dal_err_t (*init)(dal_key_dev_t *dev);
    dal_err_t (*deinit)(dal_key_dev_t *dev);

    /* --- 诊断与测试 --- */
    dal_err_t (*selftest)(dal_key_dev_t *dev, dal_key_selftest_result_t *result);

    /* --- 状态查询（同步轮询） --- */
    /**
     * @brief 获取当前按键电平状态（已去抖）
     * @param dev   设备实例指针
     * @param level [出参] 按下/释放
     * @retval DAL_OK            读取成功
     * @retval DAL_ERR_NOT_READY 设备未初始化
     */
    dal_err_t (*get_level)(dal_key_dev_t *dev, dal_key_level_t *level);

    /* --- 中断控制（可选） --- */
    /**
     * @brief 使能/禁用按键硬件中断
     * @param dev    设备实例指针
     * @param enable true=使能，false=禁用
     * @retval DAL_OK            操作成功
     * @retval DAL_ERR_NOTSUP    硬件不支持中断模式
     * @retval DAL_ERR_NOT_READY 设备未初始化
     *
     * @note 若设备不支持中断，返回 NOTSUP；BSP 层可退化为轮询模式。
     *       中断使能后，按键按下/释放将触发回调通知。
     */
    dal_err_t (*set_irq_enable)(dal_key_dev_t *dev, bool enable);

} dal_key_ops_t;

/* ========================================================================== */
/*                       设备实例结构体                                          */
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
    const char          *name;      ///< 全局唯一设备标识名
    const dal_key_ops_t *ops;       ///< 硬件操作集指针
    void                *drv_priv;  ///< DRV层私有数据

    /* === 由DAL框架管理，DRV/SVC层禁止直接修改 === */
    bool                  initialized;   ///< 硬件是否已初始化
    dal_key_callback_t    cb;           ///< 注册的回调函数
    void                 *cb_user_data; ///< 回调用户上下文
};

/* ========================================================================== */
/*                     DAL框架公共API                                           */
/* ========================================================================== */

/**
 * @brief 注册按键设备实例到DAL框架
 * @param dev 预先分配并填充好的设备实例指针（全局/静态生命周期）
 * @retval DAL_OK                注册成功
 * @retval DAL_ERR_PARAM_INVALID dev为空，或name/ops为空
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
 *
 * @note 注销前会自动执行安全清理序列（调用 deinit + 强制从注册表移除）。
 *       即使 deinit 返回错误，设备仍会从注册表中移除（强制注销），
 *       避免紧急关机场景下资源残留。
 */
dal_err_t dal_key_unregister(dal_key_dev_t *dev);

/**
 * @brief 按名称查找已注册的按键设备实例
 * @param name 设备标识名
 * @return 设备实例指针；不存在或name为空则返回NULL
 */
dal_key_dev_t* dal_key_get_dev(const char *name);

/**
 * @brief 获取已注册的按键设备总数
 */
uint32_t dal_key_get_count(void);

/**
 * @brief 按索引获取已注册的按键设备实例
 * @param index 设备索引 (0 ~ dal_key_get_count()-1)
 * @return 设备实例指针；索引越界返回NULL
 */
dal_key_dev_t* dal_key_get_dev_by_index(uint32_t index);

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
 * @note 幂等设计：若未初始化则直接返回 DAL_OK。
 *       同时会禁用硬件中断（若已使能）。
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
 * @brief 获取当前电平状态（同步轮询，已去抖）
 * @param dev   设备实例指针
 * @param level [出参] 按下/释放
 * @retval DAL_OK 读取成功
 */
dal_err_t dal_key_get_level(dal_key_dev_t *dev, dal_key_level_t *level);

/**
 * @brief 注册异步事件回调
 * @param[in] dev       设备实例指针
 * @param[in] cb        回调函数，NULL=注销
 * @param[in] user_data 用户自定义上下文，原样透传给回调
 * @retval DAL_OK           成功
 * @retval DAL_ERR_INVAL    句柄无效
 *
 * @note 回调在中断上下文中触发，调用者需遵守 ISR 安全契约。
 *       若硬件不支持中断模式，注册回调后永远不会触发。
 */
dal_err_t dal_key_set_callback(dal_key_dev_t *dev,
                               dal_key_callback_t cb,
                               void *user_data);

/**
 * @brief 使能/禁用按键硬件中断
 * @param dev    设备实例指针
 * @param enable true=使能，false=禁用
 * @retval DAL_OK           操作成功
 * @retval DAL_ERR_NOTSUP   硬件不支持中断
 * @retval DAL_ERR_NOT_READY 设备未初始化
 *
 * @note 中断使能后，按键按下/释放将触发回调通知（需已注册回调）。
 *       禁用中断后，按键事件仅能通过轮询 get_level 获取。
 */
dal_err_t dal_key_set_irq_enable(dal_key_dev_t *dev, bool enable);

/**
 * @brief 通知按键事件（供BSP层在中断ISR中调用）
 * @param dev   设备实例指针
 * @param event 事件类型（DOWN / UP）
 *
 * @note 此函数【专供 BSP 层 ISR 调用】，在中断上下文中执行。
 *       若已注册回调，则直接调用回调函数。
 *       若未注册回调，事件被静默丢弃。
 *       此函数不持有任何锁，完全在调用者上下文中执行。
 */
void dal_key_notify_event(dal_key_dev_t *dev, dal_key_event_t event);

#ifdef __cplusplus
}
#endif

#endif /* __DAL_KEY_H__ */