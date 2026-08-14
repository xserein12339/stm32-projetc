/**
 * @file    dal_imu.h
 * @brief   IMU（惯性测量单元）设备抽象层 v1.0
 * 
 * @details 支持加速度计、陀螺仪、磁力计及其组合芯片（6轴/9轴）的统一抽象。
 *          提供原始数据读取、校准参数管理、FIFO 批量读取、异步数据就绪通知。
 *          物理量单位统一为 SI 标准（m/s², rad/s, μT），由驱动层负责硬件原始值转换。
 *          全程整数运算，无 FPU 依赖；SI 值以定点数形式传递。
 * 
 * @author xserein
 * @version v1.0
 */
#ifndef __DAL_IMU_H__
#define __DAL_IMU_H__

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
 * @defgroup IMU_THREAD_SAFETY 线程安全说明
 * @{
 * - 本接口【非线程安全】，所有公共 API 默认不提供内部互斥保护。
 * - 同一设备的操作（如 read / set_odr / fifo_read）需由调用者
 *   自行保证串行化（如通过互斥锁或临界区）。
 * - 事件回调函数在中断/DMA 完成上下文中执行，严禁调用任何阻塞 API。
 * - 跨设备操作无需互斥，设备间完全独立。
 * @}
 */

/* ========================================================================== */
/*                               类型前向声明                                    */
/* ========================================================================== */

typedef struct dal_imu_dev dal_imu_dev_t; ///< IMU 设备实例结构体前向声明

/* ========================================================================== */
/*                             枚举与状态定义                                    */
/* ========================================================================== */

/**
 * @brief IMU 传感器子模块类型（位掩码，支持组合查询）
 */
typedef enum {
    DAL_IMU_MODULE_ACCEL = 0x01U, ///< 加速度计
    DAL_IMU_MODULE_GYRO  = 0x02U, ///< 陀螺仪
    DAL_IMU_MODULE_MAG   = 0x04U, ///< 磁力计
} dal_imu_module_t;

/**
 * @brief 加速度计量程 (g)
 */
typedef enum {
    DAL_IMU_ACCEL_RANGE_2G  = 0, ///< ±2g
    DAL_IMU_ACCEL_RANGE_4G  = 1, ///< ±4g
    DAL_IMU_ACCEL_RANGE_8G  = 2, ///< ±8g
    DAL_IMU_ACCEL_RANGE_16G = 3, ///< ±16g
} dal_imu_accel_range_t;

/**
 * @brief 陀螺仪量程 (dps, degrees per second)
 */
typedef enum {
    DAL_IMU_GYRO_RANGE_250DPS  = 0, ///< ±250°/s
    DAL_IMU_GYRO_RANGE_500DPS  = 1, ///< ±500°/s
    DAL_IMU_GYRO_RANGE_1000DPS = 2, ///< ±1000°/s
    DAL_IMU_GYRO_RANGE_2000DPS = 3, ///< ±2000°/s
} dal_imu_gyro_range_t;

/**
 * @brief 输出数据率 (ODR, Hz)
 * @note  具体支持的 ODR 由 DRV 层决定，不支持的值返回 DAL_ERR_PARAM_INVALID。
 */
typedef enum {
    DAL_IMU_ODR_OFF    = 0,  ///< 关闭（低功耗停机）
    DAL_IMU_ODR_1      = 1,  ///< 1 Hz
    DAL_IMU_ODR_10     = 2,  ///< 10 Hz
    DAL_IMU_ODR_25     = 3,  ///< 25 Hz
    DAL_IMU_ODR_50     = 4,  ///< 50 Hz
    DAL_IMU_ODR_100    = 5,  ///< 100 Hz
    DAL_IMU_ODR_200    = 6,  ///< 200 Hz
    DAL_IMU_ODR_400    = 7,  ///< 400 Hz
    DAL_IMU_ODR_800    = 8,  ///< 800 Hz
    DAL_IMU_ODR_1600   = 9,  ///< 1600 Hz
} dal_imu_odr_t;

/**
 * @brief IMU 设备运行状态
 */
typedef enum {
    DAL_IMU_STATE_OFF      = 0, ///< 关闭（电源关断）
    DAL_IMU_STATE_IDLE     = 1, ///< 空闲（已使能，无数据就绪/FIFO 未满）
    DAL_IMU_STATE_READY    = 2, ///< 数据就绪（DRDY 触发或 FIFO 有数据）
    DAL_IMU_STATE_FAULT    = 3, ///< 故障锁定（通信异常/自检失败）
} dal_imu_state_t;

/**
 * @brief IMU 异步事件类型（位掩码，支持组合触发）
 * @note  多个事件可能同时触发（如 FIFO 满溢出同时引发 FAULT），
 *        回调中应使用 if (event & DAL_IMU_EVT_XXX) 方式逐位判断，
 *        不可用 switch-case 或等值比较。
 */
typedef enum {
    DAL_IMU_EVT_DRDY       = 0x01U, ///< 数据就绪中断（单点采样完成）
    DAL_IMU_EVT_FIFO_TH    = 0x02U, ///< FIFO 水位阈值触发
    DAL_IMU_EVT_FIFO_FULL  = 0x04U, ///< FIFO 满溢出
    DAL_IMU_EVT_FAULT      = 0x08U, ///< 硬件故障（通信超时/校验错误）
    DAL_IMU_EVT_FAULT_CLR  = 0x80U, ///< 所有故障已清除
} dal_imu_event_t;

/**
 * @brief IMU 专用自检结果码
 */
typedef enum {
    DAL_IMU_SELFTEST_PASS       = 0x00, ///< 自检通过
    DAL_IMU_SELFTEST_ERR_DRV    = 0x01, ///< 驱动上下文异常
    DAL_IMU_SELFTEST_ERR_COMM   = 0x02, ///< 通信链路故障（SPI/I2C 无应答）
    DAL_IMU_SELFTEST_ERR_ACCEL  = 0x03, ///< 加速度计自检失败
    DAL_IMU_SELFTEST_ERR_GYRO   = 0x04, ///< 陀螺仪自检失败
    DAL_IMU_SELFTEST_ERR_MAG    = 0x05, ///< 磁力计自检失败
    DAL_IMU_SELFTEST_NOT_IMPL   = 0xFF, ///< 不支持自检
} dal_imu_selftest_result_t;

/**
 * @brief IMU 设备能力标志位（位掩码）
 */
typedef enum {
    /** 内置硬件 FIFO */
    DAL_IMU_CAP_FIFO            = 0x0001U,

    /** 支持 DRDY 中断引脚 */
    DAL_IMU_CAP_DRDY_IRQ        = 0x0002U,

    /** 支持磁力计（9轴） */
    DAL_IMU_CAP_MAG             = 0x0004U,

    /** 支持温度传感器 */
    DAL_IMU_CAP_TEMP            = 0x0008U,

    /** 支持硬件低通/高通滤波器配置 */
    DAL_IMU_CAP_FILTER          = 0x0010U,

    /** 支持 DMA 批量读取 */
    DAL_IMU_CAP_DMA             = 0x0020U,

    /** 支持唤醒/运动检测中断 */
    DAL_IMU_CAP_WAKEUP          = 0x0040U,
} dal_imu_capability_t;

/* ========================================================================== */
/*                           数据结构定义                                        */
/* ========================================================================== */

/**
 * @brief 三轴定点数向量
 * @note  数值含义由上下文决定（accel: mg, gyro: mdps, mag: nT）。
 *        采用 int32_t 保证全量程精度且无 FPU 依赖。
 */
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} dal_imu_vec3_t;

/**
 * @brief IMU 原始数据包（时间戳 + 多轴数据）
 *
 * @par 【FIFO 批量读取时间戳约定】
 *      FIFO 批量读取时，仅 buf[0] 的 timestamp_us 为 BSP 层在 ISR/读取时刻
 *      记录的真实硬件时间戳。后续样本的时间戳由调用者按以下公式推算：
 *      @code
 *      buf[i].timestamp_us = buf[0].timestamp_us + i * odr_interval_us
 *      @endcode
 *      其中 odr_interval_us = 1000000 / current_odr_hz。
 *      DRV 层在 fifo_read 实现中仅需填充 buf[0].timestamp_us，
 *      其余样本的 timestamp_us 字段可置 0，由 SVC 层推算填充。
 */
typedef struct {
    uint64_t       timestamp_us; ///< 微秒级时间戳（见上方 FIFO 时间戳约定）
    dal_imu_vec3_t accel_mg;     ///< 加速度 (milli-g)，未校准时为原始转换值
    dal_imu_vec3_t gyro_mdps;    ///< 角速度 (milli-dps)，未校准时为原始转换值
    dal_imu_vec3_t mag_nt;       ///< 磁场强度 (nano-Tesla)，仅 9 轴有效
    int16_t        temp_mc;      ///< 温度 (milli-℃)，仅支持温度传感时有效
    uint32_t       valid_mask;   ///< 有效数据位掩码（dal_imu_module_t 组合）
} dal_imu_data_t;

/**
 * @brief IMU 校准参数
 * @note  校准由 SVC 层计算并注入，DRV 层在数据转换时应用。
 *
 * @par 【DAL 层校准职责边界】
 *      DAL 层仅负责传递和应用零偏/硬铁偏移等线性校正参数。
 *      软铁校正矩阵、椭球拟合、9轴融合姿态解算等高级校准算法
 *      【不属于 DAL 层职责】，应由 SVC 层在收到校准后的数据后自行实现。
 */
typedef struct {
    dal_imu_vec3_t accel_offset_mg;  ///< 加速度零偏 (mg)
    dal_imu_vec3_t gyro_offset_mdps; ///< 陀螺仪零偏 (mdps)
    dal_imu_vec3_t mag_hard_nt;      ///< 磁力计硬铁偏移 (nT)
} dal_imu_calibration_t;

/**
 * @brief IMU 异步事件回调函数原型
 * @param[in] dev       触发事件的设备实例指针
 * @param[in] event     事件类型（支持位组合，参见 dal_imu_event_t 注释）
 * @param[in] user_data 用户自定义上下文
 *
 * @warning 【ISR/DMA 安全契约 — 违反将导致系统崩溃】
 *          1. 此回调在中断或 DMA 完成回调上下文中执行
 *          2. 严禁调用任何阻塞 API（delay、mutex_lock、malloc 等）
 *          3. 【严禁】在回调中调用任何 DAL IMU API
 *             （包括 read、fifo_read、set_odr 等）
 *          4. 推荐做法：仅通过非阻塞 ISR 专用接口传递事件到任务
 */
typedef void (*dal_imu_event_callback_t)(dal_imu_dev_t *dev,
                                         uint32_t event,
                                         void *user_data);

/* ========================================================================== */
/*                        设备操作集（DRV层核心契约）                             */
/* ========================================================================== */

/**
 * @brief IMU 硬件抽象操作接口
 * @note  DRV 层实现者需根据芯片能力选择性实现接口。
 *        不适用的接口应置为 NULL，DAL 层将返回 DAL_ERR_NOT_SUPPORTED。
 */
typedef struct {
    /* --- 生命周期管理 --- */
    dal_err_t (*init)(dal_imu_dev_t *dev);
    dal_err_t (*deinit)(dal_imu_dev_t *dev);

    /* --- 诊断与测试 --- */
    dal_err_t (*selftest)(dal_imu_dev_t *dev,
                          dal_imu_selftest_result_t *result);

    /* --- 数据读取 --- */

    /**
     * @brief 同步读取最新单点数据
     * @param dev  设备实例指针
     * @param data [出参] 数据缓冲区
     * @retval DAL_OK         读取成功
     * @retval DAL_ERR_BUSY   上一次异步 FIFO 读取尚未完成
     *
     * @note  【并发互斥契约】
     *        若异步 FIFO 读取正在进行（get_state == READY 且 FIFO 模式激活），
     *        本接口【必须】立即返回 DAL_ERR_BUSY，【禁止】阻塞等待。
     *        调用者应通过 get_state() 或等待 EVT_FIFO_TH/EVT_FIFO_FULL
     *        事件后再重试。
     *
     * @note  返回的 data->valid_mask 指示哪些轴数据有效。
     *        若已注入校准参数，返回值应为校准后的物理量。
     */
    dal_err_t (*read)(dal_imu_dev_t *dev, dal_imu_data_t *data);

    /**
     * @brief 批量读取 FIFO 中的数据
     * @param dev       设备实例指针
     * @param buf       [出参] 数据数组缓冲区
     * @param max_count 缓冲区最大容量
     * @param actual    [出参] 实际读取条数
     *
     * @retval DAL_OK           读取成功（actual <= max_count）
     * @retval DAL_ERR_NOTSUP   不支持 FIFO
     * @retval DAL_ERR_BUSY     同步 read 正在执行中
     * @retval DAL_ERR_OVERFLOW FIFO 溢出，数据可能丢失
     *
     * @note  【并发互斥契约（与 read 对称）】
     *        若同步 read 正在执行（阻塞在 SPI/I2C 传输中），
     *        本接口【必须】立即返回 DAL_ERR_BUSY，【禁止】阻塞等待。
     *        read 与 fifo_read 构成双向互斥对，任一操作进行中
     *        另一操作均须返回 BUSY。
     *
     * @note  【时间戳填充约定】
     *        DRV 层仅需填充 buf[0].timestamp_us 为真实硬件时间戳，
     *        后续样本时间戳由 SVC 层按 ODR 间隔推算。
     *        详见 dal_imu_data_t 的 FIFO 时间戳约定。
     */
    dal_err_t (*fifo_read)(dal_imu_dev_t *dev, dal_imu_data_t *buf,
                           uint32_t max_count, uint32_t *actual);

    /* --- 配置与控制 --- */

    /**
     * @brief 设置输出数据率
     * @param dev    设备实例指针
     * @param module 目标子模块（dal_imu_module_t 位组合）
     * @param odr    目标 ODR
     * @retval DAL_ERR_PARAM_INVALID 不支持该 ODR
     */
    dal_err_t (*set_odr)(dal_imu_dev_t *dev, uint32_t module,
                         dal_imu_odr_t odr);

    /**
     * @brief 设置加速度计量程
     */
    dal_err_t (*set_accel_range)(dal_imu_dev_t *dev,
                                 dal_imu_accel_range_t range);

    /**
     * @brief 设置陀螺仪量程
     */
    dal_err_t (*set_gyro_range)(dal_imu_dev_t *dev,
                                dal_imu_gyro_range_t range);

    /**
     * @brief 设置 FIFO 水位阈值
     * @param threshold 触发阈值（样本数），0 = 禁用阈值中断
     * @retval DAL_ERR_NOTSUP 不支持 FIFO
     */
    dal_err_t (*set_fifo_threshold)(dal_imu_dev_t *dev, uint16_t threshold);

    /**
     * @brief 注入校准参数
     * @note  DRV 层应在后续 read/fifo_read 的数据转换中应用此参数。
     *        传 NULL 表示清除校准，恢复原始值输出。
     */
    dal_err_t (*set_calibration)(dal_imu_dev_t *dev,
                                 const dal_imu_calibration_t *cal);

    /**
     * @brief 开启/关闭传感器（低功耗控制）
     * @param dev    设备实例指针
     * @param on     true=开启，false=进入低功耗/待机
     */
    dal_err_t (*set_power)(dal_imu_dev_t *dev, bool on);

    /* --- 状态查询 --- */
    dal_err_t (*get_state)(dal_imu_dev_t *dev, dal_imu_state_t *state);
    dal_err_t (*get_fault)(dal_imu_dev_t *dev, uint32_t *fault);

    /**
     * @brief 获取 IMU 硬件参数与能力标志
     * @param dev        设备实例指针
     * @param modules    [出参] 支持的子模块位掩码，不需要时传 NULL
     * @param capability [出参] 能力标志位掩码，不需要时传 NULL
     * @param fifo_depth [出参] FIFO 最大深度（样本数），不需要时传 NULL
     */
    dal_err_t (*get_info)(dal_imu_dev_t *dev, uint32_t *modules,
                          uint32_t *capability, uint16_t *fifo_depth);

    /* --- 中断控制（可选） --- */
    dal_err_t (*set_event_irq_enable)(dal_imu_dev_t *dev, bool enable);

} dal_imu_ops_t;

/* ========================================================================== */
/*                       设备实例结构体                                          */
/* ========================================================================== */

/**
 * @brief IMU 设备实例描述符
 *
 * @note 实例化要求：
 *       - 必须是全局或静态存储期（不允许栈上分配）
 *       - name 必须为静态字符串常量，框架仅保存指针不复制
 *       - 注册前需填充：name, ops, drv_priv
 *       - 其余字段由 DAL 框架管理，DRV/SVC 层禁止直接修改
 */
struct dal_imu_dev {
    /* === 由 BSP 层在注册前填充 === */
    const char             *name;      ///< 全局唯一设备标识名
    const dal_imu_ops_t    *ops;       ///< 硬件操作集指针
    void                   *drv_priv;  ///< DRV 层私有数据

    /* === 由 DAL 框架管理，DRV/SVC 层禁止直接修改 === */
    bool                      initialized;     ///< 硬件是否已初始化
    dal_imu_event_callback_t  event_cb;        ///< 事件回调函数
    void                     *event_cb_data;   ///< 事件回调用户上下文
};

/* ========================================================================== */
/*                     DAL 框架公共 API                                         */
/* ========================================================================== */

/** @brief 注册 IMU 设备实例到 DAL 框架 */
dal_err_t dal_imu_register(dal_imu_dev_t *dev);

/**
 * @brief 从 DAL 框架注销 IMU 设备实例
 * @note 【强制注销语义】同 dal_display_unregister
 */
dal_err_t dal_imu_unregister(dal_imu_dev_t *dev);

/** @brief 按名称查找已注册的 IMU 设备实例 */
dal_imu_dev_t* dal_imu_get_dev(const char *name);

/** @brief 获取已注册的 IMU 设备总数 */
uint32_t dal_imu_get_count(void);

/** @brief 按逻辑索引获取已注册的 IMU 设备实例 */
dal_imu_dev_t* dal_imu_get_dev_by_index(uint32_t index);

/** @brief 初始化 IMU 设备硬件 */
dal_err_t dal_imu_init(dal_imu_dev_t *dev);

/** @brief 反初始化 IMU 设备硬件（幂等） */
dal_err_t dal_imu_deinit(dal_imu_dev_t *dev);

/** @brief 执行硬件自检 */
dal_err_t dal_imu_selftest(dal_imu_dev_t *dev,
                           dal_imu_selftest_result_t *result);

/** @brief 同步读取最新单点数据 */
dal_err_t dal_imu_read(dal_imu_dev_t *dev, dal_imu_data_t *data);

/** @brief 批量读取 FIFO 数据 */
dal_err_t dal_imu_fifo_read(dal_imu_dev_t *dev, dal_imu_data_t *buf,
                            uint32_t max_count, uint32_t *actual);

/** @brief 设置输出数据率 */
dal_err_t dal_imu_set_odr(dal_imu_dev_t *dev, uint32_t module,
                          dal_imu_odr_t odr);

/** @brief 设置加速度计量程 */
dal_err_t dal_imu_set_accel_range(dal_imu_dev_t *dev,
                                  dal_imu_accel_range_t range);

/** @brief 设置陀螺仪量程 */
dal_err_t dal_imu_set_gyro_range(dal_imu_dev_t *dev,
                                 dal_imu_gyro_range_t range);

/** @brief 设置 FIFO 水位阈值 */
dal_err_t dal_imu_set_fifo_threshold(dal_imu_dev_t *dev, uint16_t threshold);

/** @brief 注入校准参数 */
dal_err_t dal_imu_set_calibration(dal_imu_dev_t *dev,
                                  const dal_imu_calibration_t *cal);

/** @brief 开启/关闭传感器 */
dal_err_t dal_imu_set_power(dal_imu_dev_t *dev, bool on);

/** @brief 获取当前运行状态 */
dal_err_t dal_imu_get_state(dal_imu_dev_t *dev, dal_imu_state_t *state);

/** @brief 获取当前故障标志位 */
dal_err_t dal_imu_get_fault(dal_imu_dev_t *dev, uint32_t *fault);

/** @brief 获取 IMU 硬件参数与能力标志 */
dal_err_t dal_imu_get_info(dal_imu_dev_t *dev, uint32_t *modules,
                           uint32_t *capability, uint16_t *fifo_depth);

/** @brief 注册异步事件回调 */
dal_err_t dal_imu_set_event_callback(dal_imu_dev_t *dev,
                                     dal_imu_event_callback_t cb,
                                     void *user_data);

/** @brief 使能/禁用硬件事件中断 */
dal_err_t dal_imu_set_event_irq_enable(dal_imu_dev_t *dev, bool enable);

/**
 * @brief 通知 IMU 事件（供 BSP 层在中断/DMA 完成回调中调用）
 * @note 此函数【专供 BSP 层 ISR 调用】，在中断上下文中执行。
 */
void dal_imu_notify_event(dal_imu_dev_t *dev, uint32_t event);

#ifdef __cplusplus
}
#endif

#endif /* __DAL_IMU_H__ */