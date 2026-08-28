/**
 * @file    svc_att_algo.h
 * @brief   姿态解算服务 v1.0
 *
 * 从 IMU 设备周期采样原始数据，经互补滤波融合输出车体倾角与角速度，
 * 并提供陀螺仪零偏校准与数据有效性监测能力。
 *
 * 职责边界（与 app / mot_ctrl 的约定）：
 *   - 本服务只负责"算得准"：采样、滤波、校准、失效检测
 *   - 平衡控制（PID / 电机输出）由 svc_mot_ctrl 消费本服务快照完成
 *   - 业务流程（何时校准、何时启停平衡）由 app 层模式状态机编排
 *
 * 依赖：dal_imu（设备）、mw_common / mw_filter（算法）、FreeRTOS（任务/互斥）
 *
 * 参考文档：本项目《需求分析》FR-ATT、《开发手册》5.2 / 5.6
 * @author  xserein
 * @version v1.0
 */
#ifndef __SVC_ATT_ALGO_H__
#define __SVC_ATT_ALGO_H__

#include <stdint.h>
#include <stdbool.h>
#include "q15_math.h"
#include "svc_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              配置默认值                                      */
/* ========================================================================== */

#define SVC_ATT_ALGO_DEFAULT_IMU_NAME        "mpu6050" /**< 默认 IMU 设备名 */
#define SVC_ATT_ALGO_DEFAULT_PERIOD_MS       (5U)      /**< 采样周期 200Hz */
#define SVC_ATT_ALGO_DEFAULT_ALPHA_Q15       (655)     /**< 加速度计权重 ≈0.02 */
#define SVC_ATT_ALGO_DEFAULT_CALIB_SAMPLES   (200U)    /**< 校准采样数（5ms*200=1s） */
#define SVC_ATT_ALGO_DEFAULT_MOTION_TH_MDPS  (100)     /**< 校准期运动判定阈值 */

/** 采样任务优先级（控制任务=4 最高，采样次之，见《开发手册》5.4） */
#define SVC_ATT_ALGO_TASK_PRIORITY           (3U)
/** 采样任务栈大小（word）。dal_imu_read 内含 HAL I2C 调用链，余量已留 */
#define SVC_ATT_ALGO_TASK_STACK_WORDS        (256U)
/** 连续读取失败次数达到该值判定数据失效（FR-ATT-003） */
#define SVC_ATT_ALGO_FAIL_COUNT_MAX          (3U)

/* ========================================================================== */
/*                              类型定义                                        */
/* ========================================================================== */

/**
 * @brief 服务运行状态
 * @note  状态迁移：UNINIT -> IDLE -> RUNNING <-> CALIBRATING
 *                        RUNNING -> FAULT -> RUNNING（数据恢复）
 */
typedef enum {
    SVC_ATT_ALGO_STATE_UNINIT      = 0, ///< 未初始化
    SVC_ATT_ALGO_STATE_IDLE        = 1, ///< 已初始化，任务未启动
    SVC_ATT_ALGO_STATE_RUNNING     = 2, ///< 正常采样解算
    SVC_ATT_ALGO_STATE_CALIBRATING = 3, ///< 零偏校准中（采样暂停）
    SVC_ATT_ALGO_STATE_FAULT       = 4, ///< IMU 数据失效（FR-ATT-003）
} svc_att_algo_state_t;

/**
 * @brief 服务异步事件类型（位掩码，回调中逐位判断）
 */
typedef enum {
    SVC_ATT_ALGO_EVT_CALIB_DONE    = 0x01U, ///< 零偏校准完成
    SVC_ATT_ALGO_EVT_CALIB_FAILED  = 0x02U, ///< 零偏校准失败（运动干扰/读取失败）
    SVC_ATT_ALGO_EVT_DATA_LOST     = 0x04U, ///< 数据失效（连续读取失败）
    SVC_ATT_ALGO_EVT_DATA_RECOVER  = 0x08U, ///< 数据失效后恢复
} svc_att_algo_event_t;

/**
 * @brief 服务配置
 */
typedef struct {
    const char *imu_name;             /**< IMU 设备注册名（NULL 用默认值） */
    uint32_t    period_ms;            /**< 采样周期 ms（0 用默认值） */
    q15_t       alpha_q15;            /**< 互补滤波加速度计权重（0 用默认值） */
    uint16_t    calib_samples;        /**< 校准采样数（0 用默认值） */
    int32_t     calib_motion_th_mdps; /**< 校准期运动判定阈值 mdps（0 用默认值） */
} svc_att_algo_config_t;

/**
 * @brief 姿态快照（拉模式查询，互斥保护的一致性视图）
 */
typedef struct {
    int32_t  angle_mdeg;   /**< 融合后倾角，毫度（安装轴向约定见 .c 实现注释） */
    int32_t  rate_mdps;    /**< 角速度（零偏校正后），毫度/秒 */
    uint32_t timestamp_ms; /**< 本拍数据的系统 tick 时间戳 */
    bool     valid;        /**< 数据有效性（校准中/失效时为 false） */
} svc_att_algo_attitude_t;

/**
 * @brief 服务异步事件回调原型
 * @param[in] event     事件类型（位组合）
 * @param[in] user_data 注册时传入的用户上下文
 *
 * @note  回调在【服务采样任务上下文】执行（非 ISR），允许调用本服务 API，
 *        但回调耗时会直接挤占采样周期，应尽快返回。
 */
typedef void (*svc_att_algo_event_cb_t)(uint32_t event, void *user_data);

/* ========================================================================== */
/*                              公开 API                                        */
/* ========================================================================== */

/**
 * @brief   初始化姿态解算服务（不创建任务）
 *
 * 获取 IMU 设备并配置 ODR 至 200Hz（匹配采样周期）。
 *
 * @param[in] cfg  服务配置，NULL 表示全部使用默认值
 * @return  SVC_OK 成功；SVC_ERR_DEV 设备不可用；SVC_ERR_BUSY 重复初始化
 *
 * @note  须在调度器启动前、bsp_init() 之后调用。
 */
svc_err_t svc_att_algo_init(const svc_att_algo_config_t *cfg);

/**
 * @brief   启动采样任务（静态创建）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT 未初始化；SVC_ERR_BUSY 任务已启动
 */
svc_err_t svc_att_algo_start(void);

/**
 * @brief   停止采样任务（快照数据冻结，valid 保持原值）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT 未初始化；SVC_ERR_STATE 任务未启动
 */
svc_err_t svc_att_algo_stop(void);

/**
 * @brief   获取姿态快照（线程安全）
 *
 * @param[out] out  输出快照（内部互斥保护拷贝）
 * @return  SVC_OK 成功；SVC_ERR_PARAM 参数为空；SVC_ERR_NOT_INIT 未初始化
 *
 * @note  仅任务上下文可调用（内部使用互斥锁，禁止 ISR 调用）。
 */
svc_err_t svc_att_algo_get_attitude(svc_att_algo_attitude_t *out);

/**
 * @brief   获取服务运行状态
 * @param[out] state  输出状态
 * @return  SVC_OK 成功；SVC_ERR_PARAM 参数为空；SVC_ERR_NOT_INIT 未初始化
 */
svc_err_t svc_att_algo_get_state(svc_att_algo_state_t *state);

/**
 * @brief   请求陀螺仪零偏校准（异步）
 *
 * 请求置位后由采样任务执行：静置采样 calib_samples 拍取均值，
 * 经 dal_imu_set_calibration 注入驱动，随后融合角重置为当前加速度计角。
 * 结果通过事件回调通知（CALIB_DONE / CALIB_FAILED）。
 *
 * @return  SVC_OK 请求受理；SVC_ERR_BUSY 已在校准中；SVC_ERR_STATE 任务未运行
 *
 * @note  校准期间采样暂停、快照 valid=false，调用方（app）应确保
 *        平衡控制已停止且小车静置。
 */
svc_err_t svc_att_algo_calibrate_gyro(void);

/**
 * @brief   注册异步事件回调
 *
 * @param[in] cb         回调函数，NULL 表示注销
 * @param[in] user_data  用户上下文（回调原样透传）
 * @return  SVC_OK 成功
 *
 * @note  回调在服务任务上下文执行，见回调原型注释。
 */
svc_err_t svc_att_algo_register_event_cb(svc_att_algo_event_cb_t cb,
                                         void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_ATT_ALGO_H__ */
