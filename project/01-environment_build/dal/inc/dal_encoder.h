/**
 * @file    dal_encoder.h
 * @brief   编码器设备抽象层 v1.1
 * 
 * @details 支持增量式与绝对式编码器的统一抽象，提供位置/速度/方向读取，
 *          以及零位校准、异步事件回调机制。
 *          位置单位统一为脉冲数(Counts)，速度单位为 0.1 RPM，
 *          角度单位为 0.01 度，全程整数运算，无 FPU 依赖。
 * 
 * @author xserein
 * @version v1.1
 */
#ifndef __DAL_ENCODER_H__
#define __DAL_ENCODER_H__

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
 * @defgroup ENCODER_THREAD_SAFETY 线程安全说明
 * @{
 * - 本接口【非线程安全】，所有公共 API 默认不提供内部互斥保护。
 * - 同一设备的操作（如 get_position / reset / unregister）需由调用者
 *   自行保证串行化（如通过互斥锁或临界区）。
 * - 事件回调函数在中断上下文中执行，严禁调用任何阻塞 API 或 DAL API。
 * - 跨设备操作无需互斥，设备间完全独立。
 * @}
 */

/* ========================================================================== */
/*                               类型前向声明                                    */
/* ========================================================================== */

typedef struct dal_encoder_dev dal_encoder_dev_t; ///< 编码器设备实例结构体前向声明

/* ========================================================================== */
/*                             枚举与状态定义                                    */
/* ========================================================================== */

/**
 * @brief 编码器类型
 */
typedef enum {
    DAL_ENCODER_TYPE_INCREMENTAL = 0, ///< 增量式（正交脉冲 A/B/Z）
    DAL_ENCODER_TYPE_ABSOLUTE    = 1, ///< 绝对式（SSI/SPI/I2C/UART 等协议）
} dal_encoder_type_t;

/**
 * @brief 编码器旋转方向
 */
typedef enum {
    DAL_ENCODER_DIR_UNKNOWN    = 0, ///< 未知 / 静止
    DAL_ENCODER_DIR_CW         = 1, ///< 顺时针 (Clockwise / 正向)
    DAL_ENCODER_DIR_CCW        = 2, ///< 逆时针 (Counter-Clockwise / 反向)
} dal_encoder_dir_t;

/**
 * @brief 编码器运行状态
 */
typedef enum {
    DAL_ENCODER_STATE_DISABLED = 0, ///< 禁用（硬件未使能/休眠）
    DAL_ENCODER_STATE_IDLE     = 1, ///< 空闲（已使能，轴静止）
    DAL_ENCODER_STATE_RUNNING  = 2, ///< 运行中（检测到运动）
    DAL_ENCODER_STATE_FAULT    = 3, ///< 故障锁定（断线/通信异常）
} dal_encoder_state_t;

/**
 * @brief 编码器异步事件类型
 * @note  支持位组合，多个事件可同时触发。
 */
typedef enum {
    DAL_ENCODER_EVT_INDEX      = 0x01U, ///< 零位信号到达（增量式 Z 相脉冲）
    DAL_ENCODER_EVT_OVERFLOW   = 0x02U, ///< 计数器溢出（超出 int32_t 范围）
    DAL_ENCODER_EVT_UNDERFLOW  = 0x04U, ///< 计数器下溢
    DAL_ENCODER_EVT_FAULT      = 0x08U, ///< 硬件故障（断线/通信超时/校验错误）
    DAL_ENCODER_EVT_FAULT_CLEAR= 0x80U, ///< 所有故障已清除（硬件恢复正常）
} dal_encoder_event_t;

/**
 * @brief 编码器专用自检结果码
 */
typedef enum {
    DAL_ENCODER_SELFTEST_PASS       = 0x00, ///< 自检通过
    DAL_ENCODER_SELFTEST_ERR_DRV    = 0x01, ///< 驱动上下文异常
    DAL_ENCODER_SELFTEST_ERR_COMM   = 0x02, ///< 通信链路故障（绝对式）
    DAL_ENCODER_SELFTEST_ERR_SIGNAL = 0x03, ///< 信号异常（增量式无脉冲/缺相）
    DAL_ENCODER_SELFTEST_NOT_IMPL   = 0xFF, ///< 不支持自检
} dal_encoder_selftest_result_t;

/**
 * @brief 编码器能力标志位（位掩码）
 * @note  由 get_info() 返回，上层在初始化阶段查询一次即可缓存静态能力。
 *        运行时动态变化的标志（如 ZEROED）每次调用 get_info 都会刷新。
 */
typedef enum {
    /** 支持硬件测速（get_velocity 返回有效值，否则需上层软件差分） */
    DAL_ENCODER_CAP_HW_VELOCITY   = 0x0001U,

    /** 支持单圈绝对角度读取（绝对式编码器始终置位） */
    DAL_ENCODER_CAP_ABS_ANGLE     = 0x0002U,

    /**
     * 增量式编码器已完成归零（find_zero 或 reset 成功执行过）
     * - 仅对增量式有意义；绝对式始终置位
     * - 未置位时 get_angle() 将返回 DAL_ERR_NOTSUP
     * - 此标志为【运行时动态】状态，非固定能力
     */
    DAL_ENCODER_CAP_ZEROED        = 0x0004U,

    /** 支持硬件事件中断（INDEX/OVERFLOW/FAULT 等） */
    DAL_ENCODER_CAP_EVENT_IRQ     = 0x0008U,

    /** 支持多圈计数（绝对式多圈型号或增量式软件计圈） */
    DAL_ENCODER_CAP_MULTI_TURN    = 0x0010U,
} dal_encoder_capability_t;

/**
 * @brief 编码器异步事件回调函数原型
 * @param[in] dev       触发事件的设备实例指针
 * @param[in] event     事件类型（支持位组合）
 * @param[in] user_data 用户自定义上下文
 *
 * @warning 【ISR 安全契约 — 违反将导致系统崩溃】
 *          1. 此回调在中断上下文中执行，严禁调用任何阻塞 API
 *             （delay、mutex_lock、malloc、printf 等）
 *          2. 【严禁】在回调中调用任何 DAL 公共 API
 *             （包括 get_position、get_velocity、reset、find_zero 等）
 *          3. 【严禁】在回调中修改设备状态或全局共享变量（除非原子操作）
 *          4. 回调可能被重入（中断嵌套/多源并发），实现必须是可重入的
 *          5. 推荐做法：仅通过非阻塞 ISR 专用接口传递事件到任务
 *             （如 xQueueSendFromISR / xSemaphoreGiveFromISR），
 *             由任务上下文完成事件处理
 */
typedef void (*dal_encoder_event_callback_t)(dal_encoder_dev_t *dev,
                                             uint32_t event,
                                             void *user_data);

/* ========================================================================== */
/*                        设备操作集（DRV层核心契约）                             */
/* ========================================================================== */

/**
 * @brief 编码器硬件抽象操作接口
 * @note  DRV 层实现者需根据编码器类型（增量/绝对）选择性实现接口。
 *        不适用的接口应置为 NULL，DAL 层将返回 DAL_ERR_NOT_SUPPORTED。
 */
typedef struct {
    /* --- 生命周期管理 --- */
    dal_err_t (*init)(dal_encoder_dev_t *dev);
    dal_err_t (*deinit)(dal_encoder_dev_t *dev);

    /* --- 诊断与测试 --- */
    dal_err_t (*selftest)(dal_encoder_dev_t *dev, dal_encoder_selftest_result_t *result);

    /* --- 核心读取 --- */

    /**
     * @brief 获取当前位置（多圈累积脉冲数）
     * @param dev      设备实例指针
     * @param position [出参] 当前位置，单位：脉冲数 (Counts)
     *                 - 增量式：软件计数的累积值，reset() 后归零
     *                 - 绝对式：单圈位置 + 多圈圈数 × 单圈分辨率
     *
     * @retval DAL_OK            读取成功
     * @retval DAL_ERR_NOT_READY 设备未初始化
     * @retval DAL_ERR_FAULT     硬件故障/通信异常
     *
     * @warning 【溢出边界与补偿机制】
     *          - 返回值类型为 int32_t，有效范围 ±2,147,483,647 Counts
     *          - 以 16384 CPR 编码器 6000 RPM 为例，约 17.7 小时触达上限
     *          - 溢出时框架会触发 DAL_ENCODER_EVT_OVERFLOW / EVT_UNDERFLOW 事件
     *          - 【上层必须】在回调中记录溢出次数 N，实际位置计算公式：
     *              actual_position = (int64_t)N * INT32_MAX + position
     *          - 若应用运行时间可能超过溢出周期，【必须】注册事件回调
     *            进行补偿，否则位置值将回绕导致控制发散
     *          - 对于短行程/短时运行场景，可忽略此事件
     */
    dal_err_t (*get_position)(dal_encoder_dev_t *dev, int32_t *position);

    /**
     * @brief 获取单圈绝对角度
     * @param dev      设备实例指针
     * @param angle    [出参] 单圈角度，单位：0.01 度 (范围 0 ~ 35999，即 0.00° ~ 359.99°)
     * @retval DAL_OK            读取成功
     * @retval DAL_ERR_NOTSUP    不支持（见下方说明）
     *
     * @note  【增量式编码器前置条件】
     *        增量式编码器调用本接口前，必须满足以下任一条件：
     *          1. 已成功调用 find_zero()（硬件归零）
     *          2. 已成功调用 reset()（软件归零）
     *        未归零时返回 DAL_ERR_NOTSUP。
     *        可通过 get_info() 查询 DAL_ENCODER_CAP_ZEROED 标志预判。
     *
     * @note  绝对式编码器始终可用，无前置条件。
     */
    dal_err_t (*get_angle)(dal_encoder_dev_t *dev, uint32_t *angle);

    /**
     * @brief 获取当前转速
     * @param dev      设备实例指针
     * @param velocity [出参] 转速，单位：0.1 RPM
     *                 - 正值表示 CW，负值表示 CCW
     *                 - 例如：15000 表示 1500.0 RPM
     * @retval DAL_OK            读取成功
     * @retval DAL_ERR_NOTSUP    不支持硬件测速（需上层软件差分计算）
     *
     * @note  调用前建议通过 get_info() 查询 DAL_ENCODER_CAP_HW_VELOCITY
     *        标志，避免运行时反复尝试失败。
     */
    dal_err_t (*get_velocity)(dal_encoder_dev_t *dev, int32_t *velocity);

    /**
     * @brief 获取当前旋转方向
     * @param dev       设备实例指针
     * @param direction [出参] 旋转方向
     */
    dal_err_t (*get_direction)(dal_encoder_dev_t *dev, dal_encoder_dir_t *direction);

    /* --- 控制与校准 --- */

    /**
     * @brief 软件清零（将当前位置计数器重置为 0）
     * @note  增量式：重置软件计数器。
     *        绝对式：设置当前位置偏移量，使后续读取值从 0 开始。
     */
    dal_err_t (*reset)(dal_encoder_dev_t *dev);

    /**
     * @brief 硬件寻零（寻找机械零位/Z相）
     * @param dev        设备实例指针
     * @param timeout_ms 超时时间 (ms)，0 表示使用默认超时
     * @retval DAL_OK          寻零成功，当前位置已自动归零
     * @retval DAL_ERR_TIMEOUT 超时未找到零位
     * @retval DAL_ERR_NOTSUP  不支持（绝对式编码器上电即知位置，无需寻零）
     *
     * @note  【与 reset() 的关系】
     *        - find_zero() 成功后，内部会自动执行等效于 reset() 的操作，
     *          将当前位置计数器设为 0，后续 get_position() 从 0 开始累积
     *        - find_zero() = 物理定位 + 软件归零（原子操作）
     *        - reset()     = 仅软件归零（不移动轴，不依赖 Z 相）
     *        - 若仅需清除累积误差而不重新定位，应使用 reset()
     *        - 若需要建立机械坐标系原点，应使用 find_zero()
     *
     * @note  此函数可能阻塞较长时间（等待轴转动到 Z 相），
     *        仅在任务上下文中调用。对于需要电机配合转动的场景，
     *        建议由 SVC 层编排"电机低速转动 + 编码器寻零"序列。
     */
    dal_err_t (*find_zero)(dal_encoder_dev_t *dev, uint32_t timeout_ms);

    /* --- 状态查询 --- */
    dal_err_t (*get_state)(dal_encoder_dev_t *dev, dal_encoder_state_t *state);
    dal_err_t (*get_fault)(dal_encoder_dev_t *dev, uint32_t *fault);

    /**
     * @brief 获取编码器硬件参数与能力标志
     * @param dev          设备实例指针
     * @param resolution   [出参] 单圈分辨率 (Counts/Rev)，不需要时传 NULL
     * @param type         [出参] 编码器类型，不需要时传 NULL
     * @param capability   [出参] 能力标志位掩码（dal_encoder_capability_t 组合），
     *                     不需要时传 NULL
     *
     * @note  capability 中的 DAL_ENCODER_CAP_ZEROED 为运行时动态状态，
     *        每次调用均反映当前最新归零状态。其余标志为静态能力，
     *        初始化后不变，上层可缓存。
     */
    dal_err_t (*get_info)(dal_encoder_dev_t *dev, uint32_t *resolution,
                          dal_encoder_type_t *type, uint32_t *capability);

    /* --- 中断控制（可选） --- */
    /**
     * @brief 使能/禁用硬件事件中断
     * @param dev    设备实例指针
     * @param enable true=使能，false=禁用
     * @retval DAL_OK         操作成功
     * @retval DAL_ERR_NOTSUP 硬件不支持事件中断
     */
    dal_err_t (*set_event_irq_enable)(dal_encoder_dev_t *dev, bool enable);

} dal_encoder_ops_t;

/* ========================================================================== */
/*                       设备实例结构体                                          */
/* ========================================================================== */

/**
 * @brief 编码器设备实例描述符
 *
 * @note 实例化要求：
 *       - 必须是全局或静态存储期（不允许栈上分配）
 *       - name 必须为静态字符串常量，框架仅保存指针不复制
 *       - 注册前需填充：name, ops, drv_priv
 *       - 其余字段由DAL框架管理，DRV/SVC层禁止直接修改
 */
struct dal_encoder_dev {
    /* === 由BSP层在注册前填充 === */
    const char                *name;      ///< 全局唯一设备标识名
    const dal_encoder_ops_t   *ops;       ///< 硬件操作集指针
    void                      *drv_priv;  ///< DRV层私有数据

    /* === 由DAL框架管理，DRV/SVC层禁止直接修改 === */
    bool                       initialized;    ///< 硬件是否已初始化
    dal_encoder_event_callback_t event_cb;     ///< 事件回调函数
    void                      *event_cb_data;  ///< 事件回调用户上下文
};

/* ========================================================================== */
/*                     DAL框架公共API                                           */
/* ========================================================================== */

/**
 * @brief 注册编码器设备实例到DAL框架
 * @param dev 预先分配并填充好的设备实例指针（全局/静态生命周期）
 * @retval DAL_OK                注册成功
 * @retval DAL_ERR_PARAM_INVALID dev为空，或name/ops为空
 * @retval DAL_ERR_DUPLICATE     同名设备已注册
 * @retval DAL_ERR_FAIL          框架内部资源分配失败
 */
dal_err_t dal_encoder_register(dal_encoder_dev_t *dev);

/**
 * @brief 从DAL框架注销设备实例
 * @param dev 设备实例指针
 * @retval DAL_OK                注销成功（注册表已移除）
 * @retval DAL_ERR_PARAM_INVALID dev为空
 * @retval DAL_ERR_NOT_READY     设备未注册
 *
 * @note 【强制注销语义】
 *       注销前会自动执行 deinit（若已初始化）。
 *       即使 deinit 返回错误，设备仍会从注册表中移除，函数返回 DAL_OK。
 *       调用者【无法】通过返回值感知底层 deinit 是否失败。
 *       如需诊断 deinit 失败原因，请在调用 unregister 前单独调用 deinit。
 */
dal_err_t dal_encoder_unregister(dal_encoder_dev_t *dev);

/**
 * @brief 按名称查找已注册的编码器设备实例
 * @param name 设备标识名
 * @return 设备实例指针；不存在或name为空则返回NULL
 */
dal_encoder_dev_t* dal_encoder_get_dev(const char *name);

/**
 * @brief 获取已注册的编码器设备总数
 */
uint32_t dal_encoder_get_count(void);

/**
 * @brief 按逻辑索引获取已注册的编码器设备实例
 * @param index 逻辑索引 (0 ~ dal_encoder_get_count()-1)，表示第 N 个有效设备
 * @return 设备实例指针；索引越界返回 NULL
 *
 * @note 索引基于有效设备计数，不受注册表中空闲槽位影响。
 *       可安全用于 for(i=0; i<get_count(); i++) 遍历模式。
 */
dal_encoder_dev_t* dal_encoder_get_dev_by_index(uint32_t index);

/**
 * @brief 初始化设备硬件
 * @param dev 设备实例指针
 * @retval DAL_OK 初始化成功
 */
dal_err_t dal_encoder_init(dal_encoder_dev_t *dev);

/**
 * @brief 反初始化设备硬件
 * @note 幂等设计：若未初始化则直接返回 DAL_OK。
 *       同时会禁用事件中断（若已使能）。
 */
dal_err_t dal_encoder_deinit(dal_encoder_dev_t *dev);

/**
 * @brief 执行硬件自检
 * @param dev    设备实例指针
 * @param result [出参] 自检详细结果码
 * @retval DAL_OK 自检流程执行完成（具体结果查看result参数）
 */
dal_err_t dal_encoder_selftest(dal_encoder_dev_t *dev,
                               dal_encoder_selftest_result_t *result);

/**
 * @brief 获取当前位置（多圈累积脉冲数）
 * @param dev      设备实例指针
 * @param position [出参] 当前位置 (Counts)
 * @retval DAL_OK 读取成功
 * @see dal_encoder_ops_t::get_position 了解溢出补偿机制
 */
dal_err_t dal_encoder_get_position(dal_encoder_dev_t *dev, int32_t *position);

/**
 * @brief 获取单圈绝对角度
 * @param dev   设备实例指针
 * @param angle [出参] 单圈角度，单位 0.01 度 (0 ~ 35999)
 * @retval DAL_OK 读取成功
 */
dal_err_t dal_encoder_get_angle(dal_encoder_dev_t *dev, uint32_t *angle);

/**
 * @brief 获取当前转速
 * @param dev      设备实例指针
 * @param velocity [出参] 转速，单位 0.1 RPM
 * @retval DAL_OK 读取成功
 */
dal_err_t dal_encoder_get_velocity(dal_encoder_dev_t *dev, int32_t *velocity);

/**
 * @brief 获取当前旋转方向
 * @param dev       设备实例指针
 * @param direction [出参] 旋转方向
 * @retval DAL_OK 读取成功
 */
dal_err_t dal_encoder_get_direction(dal_encoder_dev_t *dev,
                                    dal_encoder_dir_t *direction);

/**
 * @brief 软件清零（将当前位置计数器重置为 0）
 */
dal_err_t dal_encoder_reset(dal_encoder_dev_t *dev);

/**
 * @brief 硬件寻零（寻找机械零位/Z相）
 * @param dev        设备实例指针
 * @param timeout_ms 超时时间 (ms)，0 表示使用默认超时
 * @retval DAL_OK          寻零成功
 * @retval DAL_ERR_TIMEOUT 超时
 * @retval DAL_ERR_NOTSUP  不支持
 */
dal_err_t dal_encoder_find_zero(dal_encoder_dev_t *dev, uint32_t timeout_ms);

/**
 * @brief 获取当前运行状态
 */
dal_err_t dal_encoder_get_state(dal_encoder_dev_t *dev,
                                dal_encoder_state_t *state);

/**
 * @brief 获取当前故障标志位
 */
dal_err_t dal_encoder_get_fault(dal_encoder_dev_t *dev, uint32_t *fault);

/**
 * @brief 获取编码器硬件参数与能力标志
 * @param dev          设备实例指针
 * @param resolution   [出参] 单圈分辨率，不需要时传 NULL
 * @param type         [出参] 编码器类型，不需要时传 NULL
 * @param capability   [出参] 能力标志位掩码，不需要时传 NULL
 * @retval DAL_OK 读取成功
 *
 * @par 使用示例（SVC 层初始化时）：
 * @code
 * uint32_t cap = 0;
 * dal_encoder_get_info(dev, NULL, NULL, &cap);
 * 
 * bool has_hw_vel = (cap & DAL_ENCODER_CAP_HW_VELOCITY) != 0;
 * bool is_zeroed  = (cap & DAL_ENCODER_CAP_ZEROED) != 0;
 * 
 * if (!has_hw_vel) {
 *     // 初始化软件差分测速模块
 * }
 * if (!is_zeroed && type == DAL_ENCODER_TYPE_INCREMENTAL) {
 *     // 提示用户需先执行 find_zero / reset
 * }
 * @endcode
 */
dal_err_t dal_encoder_get_info(dal_encoder_dev_t *dev, uint32_t *resolution,
                               dal_encoder_type_t *type, uint32_t *capability);

/**
 * @brief 注册异步事件回调
 * @param[in] dev       设备实例指针
 * @param[in] cb        回调函数，NULL=注销
 * @param[in] user_data 用户自定义上下文，原样透传给回调
 * @retval DAL_OK           成功
 * @retval DAL_ERR_INVAL    句柄无效
 *
 * @note 回调在中断上下文中触发，调用者需遵守 ISR 安全契约。
 *       若硬件不支持事件中断，注册回调后永远不会触发。
 */
dal_err_t dal_encoder_set_event_callback(dal_encoder_dev_t *dev,
                                         dal_encoder_event_callback_t cb,
                                         void *user_data);

/**
 * @brief 使能/禁用硬件事件中断
 */
dal_err_t dal_encoder_set_event_irq_enable(dal_encoder_dev_t *dev, bool enable);

/**
 * @brief 通知编码器事件（供BSP层在中断ISR中调用）
 * @param dev   设备实例指针
 * @param event 事件类型
 *
 * @note 此函数【专供 BSP 层 ISR 调用】，在中断上下文中执行。
 *       - 若已注册回调则直接调用，否则静默丢弃
 *       - 此函数不持有任何锁，完全在调用者上下文中执行
 *       - 同一设备的中断嵌套可能导致回调重入，BSP 层应在
 *         硬件允许的情况下屏蔽同级中断，或由回调自行保证可重入
 */
void dal_encoder_notify_event(dal_encoder_dev_t *dev, uint32_t event);

#ifdef __cplusplus
}
#endif

#endif /* __DAL_ENCODER_H__ */