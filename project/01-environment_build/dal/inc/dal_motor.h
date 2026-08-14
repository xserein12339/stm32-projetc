/**
 * @file    dal_motor.h
 * @brief   电机设备抽象层 v2.0
 * 
 * @details 支持多电机设备独立管理，提供同步占空比/方向/使能控制，
 *          以及异步故障回调通知机制。
 *          故障回调在中断上下文中执行，调用者需严格遵守 ISR 安全契约。
 * 
 * @author xserein
 * @version v2.0
 */
#ifndef __DAL_MOTOR_H__
#define __DAL_MOTOR_H__

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
 * @defgroup MOTOR_THREAD_SAFETY 线程安全说明
 * @{
 * - 本接口【非线程安全】，所有公共 API 默认不提供内部互斥保护。
 * - 同一设备的操作（如 set_duty / enable / brake / unregister）需由调用者
 *   自行保证串行化（如通过互斥锁或临界区）。
 * - 故障回调函数在中断上下文中执行，严禁调用任何阻塞 API 或 DAL API。
 * - 跨设备操作无需互斥，设备间完全独立。
 * @}
 */

/* ========================================================================== */
/*                               类型前向声明                                    */
/* ========================================================================== */

typedef struct dal_motor_dev dal_motor_dev_t; ///< 电机设备实例结构体前向声明

/* ========================================================================== */
/*                             枚举与状态定义                                    */
/* ========================================================================== */

/**
 * @brief 电机旋转方向
 */
typedef enum {
    DAL_MOTOR_DIR_CW  = 0, ///< 顺时针 (Clockwise)
    DAL_MOTOR_DIR_CCW = 1, ///< 逆时针 (Counter-Clockwise)
} dal_motor_dir_t;

/**
 * @brief 电机运行状态
 */
typedef enum {
    DAL_MOTOR_STATE_IDLE     = 0, ///< 空闲（已使能，占空比为0）
    DAL_MOTOR_STATE_RUNNING  = 1, ///< 运行中（占空比 > 0）
    DAL_MOTOR_STATE_BRAKING  = 2, ///< 刹车中（线圈短接）
    DAL_MOTOR_STATE_DISABLED = 3, ///< 禁用（高阻态/休眠）
    DAL_MOTOR_STATE_FAULT    = 4, ///< 故障锁定
} dal_motor_state_t;

/**
 * @brief 电机异步故障事件类型
 * @note  支持位组合，多个故障可同时触发。
 *        FAULT_CLEAR 为独立事件，不与故障标志位组合使用。
 */
typedef enum {
    DAL_MOTOR_EVT_OVER_CUR    = 0x01U, ///< 过流保护触发
    DAL_MOTOR_EVT_OVER_TMP    = 0x02U, ///< 过温保护触发
    DAL_MOTOR_EVT_UNDER_VOL   = 0x04U, ///< 欠压保护触发
    DAL_MOTOR_EVT_FAULT_CLEAR = 0x80U, ///< 所有故障已清除（硬件恢复正常）
} dal_motor_fault_event_t;

/**
 * @brief 电机专用自检结果码
 */
typedef enum {
    DAL_MOTOR_SELFTEST_PASS       = 0x00, ///< 自检通过
    DAL_MOTOR_SELFTEST_ERR_DRV    = 0x01, ///< 驱动上下文异常
    DAL_MOTOR_SELFTEST_ERR_BRIDGE = 0x02, ///< 桥臂故障
    DAL_MOTOR_SELFTEST_NOT_IMPL   = 0xFF, ///< 不支持自检
} dal_motor_selftest_result_t;

/**
 * @brief 电机异步故障回调函数原型
 * @param[in] dev       触发故障的设备实例指针
 * @param[in] event     故障事件类型（支持位组合，或 FAULT_CLEAR）
 * @param[in] user_data 用户自定义上下文
 *
 * @warning 【ISR 安全契约 — 违反将导致系统崩溃】
 *          1. 此回调在中断上下文中执行，严禁调用任何阻塞 API
 *             （delay、mutex_lock、malloc、printf 等）
 *          2. 【严禁】在回调中调用任何 DAL 公共 API
 *             （包括 get_fault、set_duty、enable、disable、brake、
 *              set_fault_irq_enable、set_fault_callback 等）
 *          3. 【严禁】在回调中修改设备状态或全局共享变量（除非原子操作）
 *          4. 回调可能被重入（中断嵌套/多源并发），实现必须是可重入的
 *          5. 推荐做法：仅通过非阻塞 ISR 专用接口传递事件到任务
 *             （如 xQueueSendFromISR / xSemaphoreGiveFromISR），
 *             由任务上下文完成故障诊断与恢复操作
 *          6. 如需查询故障详情，请在任务中收到事件后调用 get_fault
 */
typedef void (*dal_motor_fault_callback_t)(dal_motor_dev_t *dev,
                                           uint32_t event,
                                           void *user_data);

/* ========================================================================== */
/*                        设备操作集（DRV层核心契约）                             */
/* ========================================================================== */

/**
 * @brief 电机硬件抽象操作接口
 */
typedef struct {
    /* --- 生命周期管理 --- */
    dal_err_t (*init)(dal_motor_dev_t *dev);
    dal_err_t (*deinit)(dal_motor_dev_t *dev);

    /* --- 诊断与测试 --- */
    dal_err_t (*selftest)(dal_motor_dev_t *dev, dal_motor_selftest_result_t *result);

    /* --- 核心控制 --- */
    /**
     * @brief 设置输出占空比
     * @param dev      设备实例指针
     * @param duty_pct 占空比 (0~100)
     * @retval DAL_OK                设置成功
     * @retval DAL_ERR_NOT_READY     设备未初始化
     * @retval DAL_ERR_DISABLED      设备处于DISABLED状态
     * @retval DAL_ERR_PARAM_INVALID 占空比超出范围
     *
     * @note PWM频率在init时固定，运行时不可修改。
     */
    dal_err_t (*set_duty)(dal_motor_dev_t *dev, uint8_t duty_pct);

    /**
     * @brief 设置电机旋转方向
     * @note 建议在duty=0时切换方向，避免大电流冲击。
     */
    dal_err_t (*set_direction)(dal_motor_dev_t *dev, dal_motor_dir_t dir);

    /**
     * @brief 使能电机输出
     * @note 使能后初始占空比为0（IDLE状态）。
     *       若从BRAKING状态调用，将退出刹车进入IDLE。
     */
    dal_err_t (*enable)(dal_motor_dev_t *dev);

    /**
     * @brief 禁用电机输出（高阻态/休眠）
     */
    dal_err_t (*disable)(dal_motor_dev_t *dev);

    /**
     * @brief 主动刹车（短接线圈，快速制动）
     * @retval DAL_ERR_NOTSUP 硬件不支持（退化为disable）
     * @retval DAL_ERR_DISABLED 设备处于DISABLED状态
     *
     * @note 刹车状态下调用set_duty将自动退出刹车。
     */
    dal_err_t (*brake)(dal_motor_dev_t *dev);

    /* --- 状态查询 --- */
    dal_err_t (*get_state)(dal_motor_dev_t *dev, dal_motor_state_t *state);
    dal_err_t (*get_fault)(dal_motor_dev_t *dev, uint32_t *fault);

    /* --- 中断控制（可选） --- */
    /**
     * @brief 使能/禁用硬件故障中断
     * @param dev    设备实例指针
     * @param enable true=使能，false=禁用
     * @retval DAL_OK         操作成功
     * @retval DAL_ERR_NOTSUP 硬件不支持故障中断
     *
     * @note 中断使能后，硬件故障将触发回调通知。
     *       若不支持中断，故障仅能通过轮询get_fault获取。
     */
    dal_err_t (*set_fault_irq_enable)(dal_motor_dev_t *dev, bool enable);

} dal_motor_ops_t;

/* ========================================================================== */
/*                       设备实例结构体                                          */
/* ========================================================================== */

/**
 * @brief 电机设备实例描述符
 *
 * @note 实例化要求：
 *       - 必须是全局或静态存储期（不允许栈上分配）
 *       - name 必须为静态字符串常量，框架仅保存指针不复制
 *       - 注册前需填充：name, ops, drv_priv
 *       - 其余字段由DAL框架管理，DRV/SVC层禁止直接修改
 */
struct dal_motor_dev {
    /* === 由BSP层在注册前填充 === */
    const char              *name;      ///< 全局唯一设备标识名
    const dal_motor_ops_t   *ops;       ///< 硬件操作集指针
    void                    *drv_priv;  ///< DRV层私有数据

    /* === 由DAL框架管理，DRV/SVC层禁止直接修改 === */
    bool                     initialized;    ///< 硬件是否已初始化
    dal_motor_fault_callback_t fault_cb;     ///< 故障回调函数
    void                    *fault_cb_data;  ///< 故障回调用户上下文
};

/* ========================================================================== */
/*                     DAL框架公共API                                           */
/* ========================================================================== */

/**
 * @brief 注册电机设备实例到DAL框架
 * @param dev 预先分配并填充好的设备实例指针（全局/静态生命周期）
 * @retval DAL_OK                注册成功
 * @retval DAL_ERR_PARAM_INVALID dev为空，或name/ops为空
 * @retval DAL_ERR_DUPLICATE     同名设备已注册
 * @retval DAL_ERR_FAIL          框架内部资源分配失败
 */
dal_err_t dal_motor_register(dal_motor_dev_t *dev);

/**
 * @brief 从DAL框架注销设备实例
 * @param dev 设备实例指针
 * @retval DAL_OK                注销成功（注册表已移除）
 * @retval DAL_ERR_PARAM_INVALID dev为空
 * @retval DAL_ERR_NOT_READY     设备未注册
 *
 * @note 【强制注销语义】
 *       注销前会自动执行安全停止序列（get_state检查 -> set_duty(0) ->
 *       brake -> disable）+ deinit。
 *       即使 deinit 返回错误，设备仍会从注册表中移除，函数返回 DAL_OK。
 *       调用者【无法】通过返回值感知底层 deinit 是否失败。
 *       此设计确保紧急关机/异常恢复场景下不会因驱动层错误导致资源残留。
 *       如需诊断 deinit 失败原因，请在调用 unregister 前单独调用 deinit。
 */
dal_err_t dal_motor_unregister(dal_motor_dev_t *dev);

/**
 * @brief 按名称查找已注册的电机设备实例
 * @param name 设备标识名
 * @return 设备实例指针；不存在或name为空则返回NULL
 */
dal_motor_dev_t* dal_motor_get_dev(const char *name);

/**
 * @brief 获取已注册的电机设备总数
 */
uint32_t dal_motor_get_count(void);

/**
 * @brief 按逻辑索引获取已注册的电机设备实例
 * @param index 逻辑索引 (0 ~ dal_motor_get_count()-1)，表示第 N 个有效设备
 * @return 设备实例指针；索引越界返回 NULL
 *
 * @note 索引基于有效设备计数，不受注册表中空闲槽位影响。
 *       可安全用于 for(i=0; i<get_count(); i++) 遍历模式。
 */
dal_motor_dev_t* dal_motor_get_dev_by_index(uint32_t index);

/**
 * @brief 初始化设备硬件
 * @param dev 设备实例指针
 * @retval DAL_OK 初始化成功
 */
dal_err_t dal_motor_init(dal_motor_dev_t *dev);

/**
 * @brief 反初始化设备硬件
 * @note 幂等设计：若未初始化则直接返回 DAL_OK。
 *       同时会禁用故障中断（若已使能）。
 */
dal_err_t dal_motor_deinit(dal_motor_dev_t *dev);

/**
 * @brief 执行硬件自检
 * @param dev    设备实例指针
 * @param result [出参] 自检详细结果码
 * @retval DAL_OK 自检流程执行完成（具体结果查看result参数）
 */
dal_err_t dal_motor_selftest(dal_motor_dev_t *dev, dal_motor_selftest_result_t *result);

/**
 * @brief 设置输出占空比
 * @param dev      设备实例指针
 * @param duty_pct 占空比 (0~100)
 * @retval DAL_OK 设置成功
 */
dal_err_t dal_motor_set_duty(dal_motor_dev_t *dev, uint8_t duty_pct);

/**
 * @brief 立即停止电机（占空比归零，保持使能）
 * @note 等效于 set_duty(0)，语义更明确。
 */
dal_err_t dal_motor_stop(dal_motor_dev_t *dev);

/**
 * @brief 设置电机旋转方向
 */
dal_err_t dal_motor_set_direction(dal_motor_dev_t *dev, dal_motor_dir_t dir);

/**
 * @brief 使能电机输出
 */
dal_err_t dal_motor_enable(dal_motor_dev_t *dev);

/**
 * @brief 禁用电机输出（高阻态/休眠）
 */
dal_err_t dal_motor_disable(dal_motor_dev_t *dev);

/**
 * @brief 主动刹车（短接线圈，快速制动）
 */
dal_err_t dal_motor_brake(dal_motor_dev_t *dev);

/**
 * @brief 获取当前运行状态
 */
dal_err_t dal_motor_get_state(dal_motor_dev_t *dev, dal_motor_state_t *state);

/**
 * @brief 获取当前故障标志位
 */
dal_err_t dal_motor_get_fault(dal_motor_dev_t *dev, uint32_t *fault);

/**
 * @brief 注册异步故障回调
 * @param[in] dev       设备实例指针
 * @param[in] cb        回调函数，NULL=注销
 * @param[in] user_data 用户自定义上下文，原样透传给回调
 * @retval DAL_OK           成功
 * @retval DAL_ERR_INVAL    句柄无效
 *
 * @note 回调在中断上下文中触发，调用者需遵守 ISR 安全契约。
 *       若硬件不支持故障中断，注册回调后永远不会触发。
 */
dal_err_t dal_motor_set_fault_callback(dal_motor_dev_t *dev,
                                       dal_motor_fault_callback_t cb,
                                       void *user_data);

/**
 * @brief 使能/禁用硬件故障中断
 */
dal_err_t dal_motor_set_fault_irq_enable(dal_motor_dev_t *dev, bool enable);

/**
 * @brief 通知电机故障事件（供BSP层在中断ISR中调用）
 * @param dev   设备实例指针
 * @param event 故障事件类型（含 FAULT_CLEAR）
 *
 * @note 此函数【专供 BSP 层 ISR 调用】，在中断上下文中执行。
 *       - 若已注册回调则直接调用，否则静默丢弃
 *       - 此函数不持有任何锁，完全在调用者上下文中执行
 *       - 同一设备的中断嵌套可能导致回调重入，BSP 层应在
 *         硬件允许的情况下屏蔽同级中断，或由回调自行保证可重入
 *       - 故障清除事件由 BSP 层在检测到硬件恢复后主动调用本函数通知
 */
void dal_motor_notify_fault(dal_motor_dev_t *dev, uint32_t event);

#ifdef __cplusplus
}
#endif

#endif /* __DAL_MOTOR_H__ */