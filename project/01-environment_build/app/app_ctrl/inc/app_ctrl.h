/**
 * @file    app_ctrl.h
 * @brief   应用控制模块 v1.0（模式状态机 + 人机交互 + 服务装配）
 *
 * 职责（对应《需求分析》FR-CTRL-001 / FR-HMI / FR-FAULT-001~004）：
 *   1. 模式状态机：IDLE -> CALIBRATING -> BALANCE -> FAULT
 *      - KEY1（PA4）按下：启动流程（先陀螺校准，成功后开平衡）
 *      - KEY2（PA5）按下：停止（回 IDLE）
 *      - KEY3（PA3）按下：急停（进 FAULT，仅复位/再按 KEY1 恢复）
 *   2. LED 状态指示：LED1=平衡运行，LED2=校准中，LED3=故障
 *   3. svc_monitor 装配：watch 槽位（姿态/运动遥测时间戳）+ 喂狗注入
 *   4. svc_comm 装配：下行指令分发（速度/转向/PID）+ 10Hz 遥测上报
 *
 * 本模块是唯一允许同时接触 svc 与 bsp 的"装配层"（依赖倒置顶层）。
 *
 * 参考文档：本项目《开发手册》5.7 服务层划分
 * @author  xserein
 * @version v1.0
 */

#ifndef __APP_CTRL_H__
#define __APP_CTRL_H__

#include <stdint.h>
#include <stdbool.h>
#include "svc_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  配置默认值
 * ================================================================ */

#define APP_CTRL_TASK_PRIORITY          (2U)    /**< app 任务优先级 */
#define APP_CTRL_TASK_STACK_WORDS       (192U)  /**< 768B 栈（事件驱动，调用链浅） */
#define APP_CTRL_EVENT_QUEUE_DEPTH      (8U)    /**< 按键事件队列深度 */

/* ================================================================
 *  数据类型
 * ================================================================ */

/**
 * @brief 整机运行模式
 */
typedef enum {
    APP_CTRL_MODE_IDLE        = 0,  ///< 待机（电机 DISABLED）
    APP_CTRL_MODE_CALIBRATING = 1,  ///< 陀螺零偏校准中
    APP_CTRL_MODE_BALANCE     = 2,  ///< 自平衡运行中
    APP_CTRL_MODE_FAULT       = 3,  ///< 故障锁定（需人工干预恢复）
} app_ctrl_mode_t;

/**
 * @brief 通信指令编号（帧协议 cmd 字段，上下行约定）
 */
typedef enum {
    APP_COMM_CMD_TELEM       = 0x00,  ///< 遥测上报（上行） */
    APP_COMM_CMD_SET_VEL     = 0x10,  ///< 设定速度（下行，payload: 4B 有符号 0.1RPM 小端） */
    APP_COMM_CMD_SET_TURN    = 0x11,  ///< 设定转向（下行，payload: 2B 有符号 -100~100 小端） */
    APP_COMM_CMD_SET_PID     = 0x12,  ///< 设定 PID 参数（下行，调试用） */
    APP_COMM_CMD_STOP        = 0x13,  ///< 停止平衡（下行，无 payload） */
    APP_COMM_CMD_HEARTBEAT   = 0x14,  ///< 心跳（下行，无 payload，仅刷新链路活跃） */
} app_comm_cmd_t;

/* ================================================================
 *  公开 API
 * ================================================================ */

/**
 * @brief   初始化应用控制模块（不创建任务）
 *
 * 完成按键/LED 设备获取、按键回调注册、svc_comm 指令与遥测装配、
 * svc_monitor watch 槽位与喂狗注入。须在所有 svc_init 之后、
 * 调度器启动之前调用。
 *
 * @return  SVC_OK；SVC_ERR_DEV 设备不可用
 */
svc_err_t app_ctrl_init(void);

/**
 * @brief   启动应用控制任务（静态创建）
 * @return  SVC_OK；SVC_ERR_NOT_INIT / SVC_ERR_FAIL
 */
svc_err_t app_ctrl_start(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CTRL_H__ */
