/**
 * @file    svc_mot_ctrl.h
 * @brief   运动控制服务 v1.0
 *
 * 消费 svc_att_algo 姿态快照与编码器速度，经串级 PID（直立环 + 速度环 +
 * 转向前馈）解算双电机 PWM 输出，并提供倒地保护、电机故障响应与
 * 运动指令 / 遥测 / 在线调参接口。
 *
 * 职责边界：
 *   - 本服务负责"控得稳"：串级 PID、保护、指令执行
 *   - 姿态来源固定为 svc_att_algo（唯一的服务间依赖边）
 *   - 业务流程（何时启停平衡）由 app 层模式状态机编排
 *
 * 控制结构（默认参数为工程初值，上板必须整定）：
 *   - 直立环：5ms（200Hz），PD 控制，输出 = 双电机同向 PWM
 *   - 速度环：50ms（speed_loop_div 分频），PI 控制，输出修正直立环设定角
 *   - 转向：  指令直接叠加左右轮 PWM 差（前馈，v1.0 不闭环）
 *
 * 参考文档：本项目《需求分析》FR-CTRL/FR-MOT/FR-ENC、《开发手册》5.7
 * @author  xserein
 * @version v1.0
 */
#ifndef __SVC_MOT_CTRL_H__
#define __SVC_MOT_CTRL_H__

#include <stdint.h>
#include <stdbool.h>
#include "q15_math.h"
#include "mw_pid.h"
#include "svc_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              配置默认值                                      */
/* ========================================================================== */

#define SVC_MOT_CTRL_DEFAULT_MOTOR_A_NAME     "motor_a"
#define SVC_MOT_CTRL_DEFAULT_MOTOR_B_NAME     "motor_b"
#define SVC_MOT_CTRL_DEFAULT_ENC_L_NAME       "encoder_left"
#define SVC_MOT_CTRL_DEFAULT_ENC_R_NAME       "encoder_right"

#define SVC_MOT_CTRL_DEFAULT_PERIOD_MS        (5U)     /**< 控制周期 200Hz */
#define SVC_MOT_CTRL_DEFAULT_SPEED_DIV        (10U)    /**< 速度环分频 -> 50ms */
#define SVC_MOT_CTRL_DEFAULT_TILT_TH_MDEG     (45000)  /**< 倒地保护阈值 45° */

/**
 * @brief 直立环 PID 输入的角度满量程（毫度）
 * @note  WHY: 平衡工作点附近角度很小（±6° 内），以 ±6° 为满量程映射 Q15
 *        可让小角度误差占据足够的 Q15 动态范围，使 Kp/Kd 在 Q15 域内
 *        （≤1.0）获得合理的物理增益；超过满量程的角度经 PID 输出自然
 *        饱和到 100% 占空比，符合倒地前全力回正的策略。
 */
#define SVC_MOT_CTRL_ANGLE_FS_MDEG            (6000)

/** 速度环反馈满量程（0.1 RPM），典型减速电机轮速 < 100RPM */
#define SVC_MOT_CTRL_VEL_FS_R01               (500)

/** 控制任务优先级（最高，见《开发手册》5.4 约定） */
#define SVC_MOT_CTRL_TASK_PRIORITY            (4U)
/** 控制任务栈（word）：PID 计算 + DAL 调用链，无大数组 */
#define SVC_MOT_CTRL_TASK_STACK_WORDS         (320U)

/* ========================================================================== */
/*                              类型定义                                        */
/* ========================================================================== */

/**
 * @brief 服务运行状态
 */
typedef enum {
    SVC_MOT_CTRL_STATE_UNINIT     = 0, ///< 未初始化
    SVC_MOT_CTRL_STATE_IDLE       = 1, ///< 已初始化，任务未启动 / 平衡未开启
    SVC_MOT_CTRL_STATE_BALANCING  = 2, ///< 平衡控制运行中
    SVC_MOT_CTRL_STATE_PROTECTED  = 3, ///< 倒地保护（倾角超限，锁定）
    SVC_MOT_CTRL_STATE_FAULT      = 4, ///< 故障（姿态失效 / 电机故障，锁定）
} svc_mot_ctrl_state_t;

/**
 * @brief 服务异步事件类型（位掩码）
 */
typedef enum {
    SVC_MOT_CTRL_EVT_TILT_SHUTDOWN = 0x01U, ///< 倒地保护触发
    SVC_MOT_CTRL_EVT_MOTOR_FAULT   = 0x02U, ///< 电机故障
    SVC_MOT_CTRL_EVT_ATT_LOST      = 0x04U, ///< 姿态数据失效
    SVC_MOT_CTRL_EVT_RECOVER       = 0x08U, ///< 故障清除（balance_start 成功）
} svc_mot_ctrl_event_t;

/**
 * @brief PID 环选择
 */
typedef enum {
    SVC_MOT_CTRL_PID_UPRIGHT = 0, ///< 直立环（PD）
    SVC_MOT_CTRL_PID_SPEED   = 1, ///< 速度环（PI）
} svc_mot_ctrl_pid_loop_t;

/**
 * @brief 服务配置
 * @note  invert 标志用于适配电机接线方向 / 安装朝向，上板标定时确认。
 */
typedef struct {
    const char *motor_a_name;       /**< 左电机设备名（NULL 用默认值） */
    const char *motor_b_name;       /**< 右电机设备名 */
    const char *enc_left_name;      /**< 左编码器设备名 */
    const char *enc_right_name;     /**< 右编码器设备名 */
    bool motor_a_invert;            /**< 左电机方向取反 */
    bool motor_b_invert;            /**< 右电机方向取反 */
    bool speed_loop_invert;         /**< 速度环输出符号取反（整机标定用） */
    uint32_t period_ms;             /**< 控制周期 ms（0 用默认值） */
    uint32_t speed_loop_div;        /**< 速度环分频（0 用默认值） */
    int32_t tilt_th_mdeg;           /**< 倒地保护阈值毫度（0 用默认值） */
    int32_t balance_angle_mdeg;     /**< 机械平衡角偏移毫度（默认 0） */
    mw_pid_params_t upright_pid;    /**< 直立环参数（全 0 用默认值） */
    mw_pid_params_t speed_pid;      /**< 速度环参数（全 0 用默认值） */
} svc_mot_ctrl_config_t;

/**
 * @brief 运动遥测快照（拉模式查询，互斥保护的一致性视图）
 */
typedef struct {
    int32_t  angle_mdeg;            /**< 当前倾角（毫度，来自姿态服务） */
    int32_t  rate_mdps;             /**< 当前角速度（毫度/秒） */
    int32_t  vel_left_r01;          /**< 左轮速度（0.1 RPM） */
    int32_t  vel_right_r01;         /**< 右轮速度（0.1 RPM） */
    int32_t  vel_target_r01;        /**< 目标速度（0.1 RPM） */
    uint8_t  duty_left_pct;         /**< 左电机占空比（0~100） */
    uint8_t  duty_right_pct;        /**< 右电机占空比（0~100） */
    svc_mot_ctrl_state_t state;     /**< 服务状态 */
    uint32_t timestamp_ms;          /**< 快照系统 tick 时间戳 */
} svc_mot_ctrl_telemetry_t;

/**
 * @brief 服务异步事件回调原型
 * @param[in] event     事件类型（位组合）
 * @param[in] user_data 注册时传入的用户上下文
 *
 * @note  回调在【控制任务上下文】执行，允许调用本服务 API，
 *        但回调耗时会直接挤占控制周期（5ms），应尽快返回。
 */
typedef void (*svc_mot_ctrl_event_cb_t)(uint32_t event, void *user_data);

/* ========================================================================== */
/*                              公开 API                                        */
/* ========================================================================== */

/**
 * @brief   初始化运动控制服务（不创建任务，电机保持 DISABLED）
 *
 * 获取电机 / 编码器设备，注册电机故障回调，定格 PID 参数。
 * 前置条件：svc_att_algo 已初始化（本服务依赖其快照）。
 *
 * @param[in] cfg  服务配置，NULL 表示全部使用默认值
 * @return  SVC_OK 成功；SVC_ERR_DEV 设备不可用；SVC_ERR_BUSY 重复初始化
 *
 * @note  须在调度器启动前、svc_att_algo_init() 之后调用。
 */
svc_err_t svc_mot_ctrl_init(const svc_mot_ctrl_config_t *cfg);

/**
 * @brief   启动控制任务（静态创建）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT / SVC_ERR_BUSY
 */
svc_err_t svc_mot_ctrl_start(void);

/**
 * @brief   停止控制任务（电机保持 DISABLED）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT / SVC_ERR_STATE
 */
svc_err_t svc_mot_ctrl_stop(void);

/**
 * @brief   开启平衡控制（清除保护/故障锁定，电机使能）
 * @return  SVC_OK 成功；SVC_ERR_STATE 任务未运行；SVC_ERR_DEV 电机不可用
 *
 * @note  调用前提：姿态数据有效且车体接近直立（|倾角| < 保护阈值），
 *        否则下一控制周期立即触发倒地保护。
 */
svc_err_t svc_mot_ctrl_balance_start(void);

/**
 * @brief   停止平衡控制（电机停止并禁用，速度指令清零）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT
 */
svc_err_t svc_mot_ctrl_balance_stop(void);

/**
 * @brief   紧急停机（任意状态可用：立即禁用双电机并锁定）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT
 *
 * @note  任务上下文调用；FR-CTRL-004 / FR-MOT-002 的 10ms 指标由
 *        控制任务内的周期保护路径保证，本 API 供指令通道异步触发。
 */
svc_err_t svc_mot_ctrl_emergency_stop(void);

/**
 * @brief   设定目标行进速度（速度环设定值）
 * @param[in] vel_r01  目标速度（0.1 RPM，正值前进）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT
 */
svc_err_t svc_mot_ctrl_set_velocity(int32_t vel_r01);

/**
 * @brief   设定转向强度（左右轮 PWM 差前馈）
 * @param[in] turn  转向强度（-100 ~ +100，正值右转，0 直行）
 * @return  SVC_OK 成功；SVC_ERR_PARAM 超范围；SVC_ERR_NOT_INIT
 */
svc_err_t svc_mot_ctrl_set_turn(int32_t turn);

/**
 * @brief   在线修改 PID 参数（供遥调指令）
 * @param[in] loop    目标环
 * @param[in] params  新参数（字段含义同 mw_pid_params_t）
 * @return  SVC_OK 成功；SVC_ERR_PARAM / SVC_ERR_NOT_INIT
 */
svc_err_t svc_mot_ctrl_set_pid_params(svc_mot_ctrl_pid_loop_t loop,
                                      const mw_pid_params_t *params);

/**
 * @brief   获取遥测快照（线程安全）
 * @param[out] out  输出快照
 * @return  SVC_OK 成功；SVC_ERR_PARAM / SVC_ERR_NOT_INIT
 *
 * @note  仅任务上下文可调用（内部使用互斥锁，禁止 ISR 调用）。
 */
svc_err_t svc_mot_ctrl_get_telemetry(svc_mot_ctrl_telemetry_t *out);

/**
 * @brief   获取服务运行状态
 * @param[out] state  输出状态
 * @return  SVC_OK 成功；SVC_ERR_PARAM / SVC_ERR_NOT_INIT
 */
svc_err_t svc_mot_ctrl_get_state(svc_mot_ctrl_state_t *state);

/**
 * @brief   注册异步事件回调
 * @param[in] cb         回调函数（NULL 注销）
 * @param[in] user_data  用户上下文
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT
 */
svc_err_t svc_mot_ctrl_register_event_cb(svc_mot_ctrl_event_cb_t cb,
                                         void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_MOT_CTRL_H__ */
