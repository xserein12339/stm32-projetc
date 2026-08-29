/**
 * @file    app_ctrl.c
 * @brief   应用控制模块实现 v1.0
 *
 * 事件驱动：按键 ISR 回调 -> FromISR 队列 -> app 任务状态机；
 * 服务事件（校准完成/倒地保护/故障）经各服务事件回调投递同一队列。
 *
 * @author  xserein
 * @version v1.0
 */
#include "app_ctrl.h"
#include "fw_version.h"
#include "app_hmi.h"
#include "svc_att_algo.h"
#include "svc_mot_ctrl.h"
#include "svc_monitor.h"
#include "svc_comm.h"
#include "dal_key.h"
#include "dal_led.h"
#include "bsp_wdg.h"
#include "bsp_dbg.h"
#include "mw_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>

/** OLED 刷新周期（ms）：2Hz，兼顾 I2C 帧耗时（~25ms/整屏）与 CPU 占用 */
#define APP_CTRL_DISPLAY_PERIOD_MS   (500U)

/** 显示行缓冲宽度（含结尾 NUL；屏宽 21 字符） */
#define APP_CTRL_LINE_BUF_CHARS      (24U)

/* ========================================================================== */
/*                          内部类型与静态状态                                   */
/* ========================================================================== */

/**
 * @brief 应用事件（队列元素）
 */
typedef enum {
    APP_EVT_KEY_START = 1,    /**< KEY1（PA4）：启动（校准+平衡） */
    APP_EVT_KEY_STOP  = 2,    /**< KEY2（PA5）/ comm 0x13：停止 */
    APP_EVT_ATT_CB    = 4,    /**< 姿态服务事件（event 位域存 arg） */
    APP_EVT_MOT_CB    = 5,    /**< 运动服务事件（event 位域存 arg） */
    APP_EVT_MON_FAULT = 6,    /**< 监视超时（fault 掩码存 arg） */
    APP_EVT_PID_TUNE  = 7,    /**< PID 调节模式切换（arg: 1=进入 0=退出） */
} app_evt_id_t;

typedef struct {
    uint8_t id;
    uint8_t arg;              /**< 事件附加参数（位域/掩码，截断到 8 位） */
} app_evt_t;

/**
 * @brief 模块运行时状态
 */
typedef struct {
    volatile app_ctrl_mode_t mode;

    volatile bool pending_tune;  /**< 校准期间收到进入调节模式请求，平衡启动后生效 */

    dal_key_dev_t *key_start;   /**< KEY1（PA4） */
    dal_key_dev_t *key_stop;    /**< KEY2（PA5） */
    dal_led_dev_t *led_run;     /**< LED1（PC13）：校准/平衡/调节运行指示 */
    dal_led_dev_t *led_fault;   /**< LED2（PB13）：故障指示 */

    QueueHandle_t   evt_queue;
    StaticQueue_t   evt_queue_mem;
    uint8_t         evt_queue_storage[APP_CTRL_EVENT_QUEUE_DEPTH
                                      * sizeof(app_evt_t)];

    TaskHandle_t    task_handle;
    StaticTask_t    task_tcb;
    StackType_t     task_stack[APP_CTRL_TASK_STACK_WORDS];
    bool            task_running;
} app_ctrl_ctx_t;

static app_ctrl_ctx_t s_ctx;

/** 固件版本（comm 版本查询应答 payload） */
static const uint8_t k_fw_version[3] = {
    FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH
};

/* ========================================================================== */
/*                          事件投递（多上下文安全）                              */
/* ========================================================================== */

/**
 * @brief   ISR 上下文事件投递
 */
static void evt_post_from_isr(uint8_t id, uint8_t arg)
{
    app_evt_t evt = { .id = id, .arg = arg };
    BaseType_t woken = pdFALSE;

    (void)xQueueSendFromISR(s_ctx.evt_queue, &evt, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * @brief   任务上下文事件投递（满队列丢弃：事件可合并，旧事件更及时）
 */
static void evt_post(uint8_t id, uint8_t arg)
{
    app_evt_t evt = { .id = id, .arg = arg };

    (void)xQueueSend(s_ctx.evt_queue, &evt, 0U);
}

/* ========================================================================== */
/*                          服务事件回调                                         */
/* ========================================================================== */

/**
 * @brief 姿态服务事件（采样任务上下文，尽快返回）
 */
static void att_event_cb(uint32_t event, void *user)
{
    (void)user;
    evt_post(APP_EVT_ATT_CB, (uint8_t)event);
}

/**
 * @brief 运动服务事件（控制任务上下文，尽快返回）
 */
static void mot_event_cb(uint32_t event, void *user)
{
    (void)user;
    evt_post(APP_EVT_MOT_CB, (uint8_t)event);
}

/* ========================================================================== */
/*                          按键回调（ISR 上下文）                                */
/* ========================================================================== */

/**
 * @brief   按键去抖事件回调（ISR 上下文，仅 FromISR 队列投递）
 * @warning 遵守 dal_key ISR 安全契约
 */
static void key_cb(dal_key_dev_t *dev, dal_key_event_t event, void *user)
{
    /* 只处理按下沿；user_data 携带本键对应的应用事件 id */
    if (event != DAL_KEY_EVT_DOWN) {
        return;
    }

    evt_post_from_isr((uint8_t)(uintptr_t)user, 0U);
    (void)dev;
}

/* ========================================================================== */
/*                          svc_monitor 装配                                     */
/* ========================================================================== */

/**
 * @brief   喂狗函数（注入 svc_monitor，监控任务上下文执行）
 */
static void wdg_feed(void)
{
    (void)bsp_wdg_refresh();
}

/**
 * @brief   姿态链路活跃性（读取姿态快照时间戳）
 * @note    get_attitude 带互斥（portMAX_DELAY）——监控任务优先级 1，
 *          持有互斥的采样任务优先级 3，优先级反转无风险（占用微秒级）。
 */
static uint32_t att_activity(void *user)
{
    (void)user;
    svc_att_algo_attitude_t att;
    if (svc_att_algo_get_attitude(&att) == SVC_OK) {
        return att.timestamp_ms;
    }
    return 0U;
}

/**
 * @brief   运动链路活跃性（读取运动遥测时间戳）
 */
static uint32_t mot_activity(void *user)
{
    (void)user;
    svc_mot_ctrl_telemetry_t tlm;
    if (svc_mot_ctrl_get_telemetry(&tlm) == SVC_OK) {
        return tlm.timestamp_ms;
    }
    return 0U;
}

/* ========================================================================== */
/*                          svc_comm 装配                                        */
/* ========================================================================== */

/**
 * @brief   读 4 字节小端有符号整数
 */
static int32_t le32_to_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8)
                     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/**
 * @brief   读 2 字节小端有符号整数
 */
static int32_t le16_to_i32(const uint8_t *p)
{
    return (int32_t)(int16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8));
}

/**
 * @brief   写 2 字节小端有符号整数
 */
static void i32_to_le16(uint8_t *p, int32_t v)
{
    int16_t s = (int16_t)v;
    p[0] = (uint8_t)((uint16_t)s & 0xFFU);
    p[1] = (uint8_t)(((uint16_t)s >> 8) & 0xFFU);
}

/**
 * @brief   指令处理（svc_comm 任务上下文）
 * @note    CMD_SET_PID 载荷：[loop:1][Kp:2][Ki:2][Kd:2] 全小端 Q15 原始值。
 */
static void comm_cmd_handler(uint8_t cmd, const uint8_t *payload,
                             uint8_t len, void *user)
{
    (void)user;

    switch (cmd) {
    case APP_COMM_CMD_SET_VEL:
        if (len >= 4U) {
            (void)svc_mot_ctrl_set_velocity(le32_to_i32(payload));
        }
        break;

    case APP_COMM_CMD_SET_TURN:
        if (len >= 2U) {
            (void)svc_mot_ctrl_set_turn(le16_to_i32(payload));
        }
        break;

    case APP_COMM_CMD_SET_PID:
        if (len >= 7U) {
            mw_pid_params_t params = {0};
            params.Kp = (q15_t)(int16_t)le16_to_i32(&payload[1]);
            params.Ki = (q15_t)(int16_t)le16_to_i32(&payload[3]);
            params.Kd = (q15_t)(int16_t)le16_to_i32(&payload[5]);
            (void)svc_mot_ctrl_set_pid_params(
                (payload[0] == 0U) ? SVC_MOT_CTRL_PID_UPRIGHT
                                   : SVC_MOT_CTRL_PID_SPEED,
                &params);
        }
        break;

    case APP_COMM_CMD_STOP:
        evt_post(APP_EVT_KEY_STOP, 0U);
        break;

    case APP_COMM_CMD_GET_VERSION:
        (void)svc_comm_send_frame(APP_COMM_CMD_GET_VERSION,
                                  (const uint8_t *)&k_fw_version[0], 3U);
        break;

    case APP_COMM_CMD_PID_TUNE:
        /* 载荷 [1B]：1=进入 0=退出；模式切换须经 app 任务，投递事件 */
        if (len >= 1U) {
            evt_post(APP_EVT_PID_TUNE, payload[0]);
        }
        break;

    case APP_COMM_CMD_GET_PID:
        /* 回 12B：直立环 Kp/Ki/Kd + 速度环 Kp/Ki/Kd，小端 Q15 原始值 */
    {
        mw_pid_params_t up = {0};
        mw_pid_params_t sp = {0};
        uint8_t resp[12];
        (void)svc_mot_ctrl_get_pid_params(SVC_MOT_CTRL_PID_UPRIGHT, &up);
        (void)svc_mot_ctrl_get_pid_params(SVC_MOT_CTRL_PID_SPEED, &sp);
        i32_to_le16(&resp[0], up.Kp);
        i32_to_le16(&resp[2], up.Ki);
        i32_to_le16(&resp[4], up.Kd);
        i32_to_le16(&resp[6], sp.Kp);
        i32_to_le16(&resp[8], sp.Ki);
        i32_to_le16(&resp[10], sp.Kd);
        (void)svc_comm_send_frame(APP_COMM_CMD_GET_PID, resp, 12U);
    }
        break;

    case APP_COMM_CMD_HEARTBEAT:
        /* 无动作：链路活跃性由上位机侧超时判断 */
        break;

    default:
        break;
    }
}

/**
 * @brief   遥测构造（svc_comm 任务上下文）
 *
 * 载荷 18 字节：[mode:1][angle:4][rate:4][vel_l:2][vel_r:2][duty:1][duty:1][state:1][seq:2]
 * 数值均为小端；seq 为帧序号（uint16 回绕）。
 */
static uint8_t comm_telem_builder(uint8_t *buf, void *user)
{
    (void)user;

    svc_att_algo_attitude_t att;
    svc_mot_ctrl_telemetry_t tlm;
    static uint16_t seq = 0U;

    bool att_ok = (svc_att_algo_get_attitude(&att) == SVC_OK);
    bool mot_ok = (svc_mot_ctrl_get_telemetry(&tlm) == SVC_OK);

    buf[0] = (uint8_t)s_ctx.mode;

    int32_t angle = att_ok ? att.angle_mdeg : 0;
    int32_t rate  = att_ok ? att.rate_mdps : 0;
    buf[1] = (uint8_t)((uint32_t)angle & 0xFFU);
    buf[2] = (uint8_t)(((uint32_t)angle >> 8) & 0xFFU);
    buf[3] = (uint8_t)(((uint32_t)angle >> 16) & 0xFFU);
    buf[4] = (uint8_t)(((uint32_t)angle >> 24) & 0xFFU);
    buf[5] = (uint8_t)((uint32_t)rate & 0xFFU);
    buf[6] = (uint8_t)(((uint32_t)rate >> 8) & 0xFFU);
    buf[7] = (uint8_t)(((uint32_t)rate >> 16) & 0xFFU);
    buf[8] = (uint8_t)(((uint32_t)rate >> 24) & 0xFFU);

    int32_t vl = mot_ok ? tlm.vel_left_r01 : 0;
    int32_t vr = mot_ok ? tlm.vel_right_r01 : 0;
    buf[9]  = (uint8_t)((uint32_t)vl & 0xFFU);
    buf[10] = (uint8_t)(((uint32_t)vl >> 8) & 0xFFU);
    buf[11] = (uint8_t)((uint32_t)vr & 0xFFU);
    buf[12] = (uint8_t)(((uint32_t)vr >> 8) & 0xFFU);

    buf[13] = mot_ok ? tlm.duty_left_pct : 0U;
    buf[14] = mot_ok ? tlm.duty_right_pct : 0U;
    buf[15] = mot_ok ? (uint8_t)tlm.state : 0U;

    buf[16] = (uint8_t)(seq & 0xFFU);
    buf[17] = (uint8_t)((seq >> 8) & 0xFFU);
    seq++;

    return 18U;
}

/* ========================================================================== */
/*                          模式状态机                                           */
/* ========================================================================== */

/**
 * @brief   LED 状态刷新（单一出口，避免多处直写）
 * @note    板 v2：仅 2 LED -- LED1 运行指示（校准/平衡/调节点亮），
 *          LED2 故障指示；校准不再单独占灯（阶段 <1s，无区分必要）。
 */
static void led_update(void)
{
    bool running = (s_ctx.mode == APP_CTRL_MODE_CALIBRATING
                    || s_ctx.mode == APP_CTRL_MODE_BALANCE
                    || s_ctx.mode == APP_CTRL_MODE_TUNING);
    (void)dal_led_set_state(s_ctx.led_run,
                            running ? DAL_LED_ON : DAL_LED_OFF);
    (void)dal_led_set_state(s_ctx.led_fault,
                (s_ctx.mode == APP_CTRL_MODE_FAULT) ? DAL_LED_ON
                                                    : DAL_LED_OFF);
}

/**
 * @brief   模式名（OLED 显示用）
 */
static const char *mode_name(app_ctrl_mode_t m)
{
    switch (m) {
    case APP_CTRL_MODE_CALIBRATING: return "CALIB";
    case APP_CTRL_MODE_BALANCE:     return "BALANCE";
    case APP_CTRL_MODE_FAULT:       return "FAULT";
    case APP_CTRL_MODE_TUNING:      return "TUNING";
    case APP_CTRL_MODE_IDLE:
    default:                        return "IDLE";
    }
}

/**
 * @brief   OLED 状态页（8 行 21 字符）
 */
static void display_status_page(char (*lines)[APP_CTRL_LINE_BUF_CHARS],
                                const char **ptrs)
{
    svc_att_algo_attitude_t att;
    svc_mot_ctrl_telemetry_t tlm;
    bool att_ok = (svc_att_algo_get_attitude(&att) == SVC_OK);
    bool mot_ok = (svc_mot_ctrl_get_telemetry(&tlm) == SVC_OK);

    int32_t angle = att_ok ? att.angle_mdeg : 0;
    int32_t rate  = att_ok ? att.rate_mdps : 0;
    int32_t vl = mot_ok ? tlm.vel_left_r01 : 0;
    int32_t vr = mot_ok ? tlm.vel_right_r01 : 0;

    (void)snprintf(lines[0], APP_CTRL_LINE_BUF_CHARS,
                   "STM32 BAL v%s", FW_VERSION_STRING);
    (void)snprintf(lines[1], APP_CTRL_LINE_BUF_CHARS,
                   "MODE : %s", mode_name(s_ctx.mode));
    (void)snprintf(lines[2], APP_CTRL_LINE_BUF_CHARS,
                   "ANG  : %6d mdg", (int)angle);
    (void)snprintf(lines[3], APP_CTRL_LINE_BUF_CHARS,
                   "RATE : %7d mdps", (int)rate);
    (void)snprintf(lines[4], APP_CTRL_LINE_BUF_CHARS,
                   "VL/VR: %5d %5d", (int)vl, (int)vr);
    (void)snprintf(lines[5], APP_CTRL_LINE_BUF_CHARS,
                   "DUTY : %3d%% %3d%%",
                   mot_ok ? (int)tlm.duty_left_pct : 0,
                   mot_ok ? (int)tlm.duty_right_pct : 0);
    (void)snprintf(lines[6], APP_CTRL_LINE_BUF_CHARS,
                   "STATE: %d",
                   mot_ok ? (int)tlm.state : 0);
    (void)snprintf(lines[7], APP_CTRL_LINE_BUF_CHARS,
                   "K1:start K2:stop");

    for (uint8_t i = 0U; i < APP_HMI_LINE_COUNT; i++) {
        ptrs[i] = lines[i];
    }
}

/**
 * @brief   OLED PID 调节页：显示两环实时参数（Q15 原始值）
 */
static void display_tuning_page(char (*lines)[APP_CTRL_LINE_BUF_CHARS],
                                const char **ptrs)
{
    mw_pid_params_t up = {0};
    mw_pid_params_t sp = {0};
    (void)svc_mot_ctrl_get_pid_params(SVC_MOT_CTRL_PID_UPRIGHT, &up);
    (void)svc_mot_ctrl_get_pid_params(SVC_MOT_CTRL_PID_SPEED, &sp);

    (void)snprintf(lines[0], APP_CTRL_LINE_BUF_CHARS, "== PID TUNING ==");
    (void)snprintf(lines[1], APP_CTRL_LINE_BUF_CHARS,
                   "UP:Kp %6d", (int)up.Kp);
    (void)snprintf(lines[2], APP_CTRL_LINE_BUF_CHARS,
                   "UP:Ki %6d Kd %5d", (int)up.Ki, (int)up.Kd);
    (void)snprintf(lines[3], APP_CTRL_LINE_BUF_CHARS,
                   "SP:Kp %6d", (int)sp.Kp);
    (void)snprintf(lines[4], APP_CTRL_LINE_BUF_CHARS,
                   "SP:Ki %6d Kd %5d", (int)sp.Ki, (int)sp.Kd);

    svc_att_algo_attitude_t att;
    if (svc_att_algo_get_attitude(&att) == SVC_OK) {
        (void)snprintf(lines[5], APP_CTRL_LINE_BUF_CHARS,
                       "ANG  : %6d mdg", (int)att.angle_mdeg);
    } else {
        (void)snprintf(lines[5], APP_CTRL_LINE_BUF_CHARS, "ANG  : n/a");
    }
    (void)snprintf(lines[6], APP_CTRL_LINE_BUF_CHARS,
                   "set:0x12 get:0x17");
    (void)snprintf(lines[7], APP_CTRL_LINE_BUF_CHARS,
                   "exit:0x16(0)");

    for (uint8_t i = 0U; i < APP_HMI_LINE_COUNT; i++) {
        ptrs[i] = lines[i];
    }
}

/**
 * @brief   OLED 整屏刷新（app 任务上下文，2Hz）
 * @note    行缓冲为函数级静态（192B bss）：避免占用任务栈
 *          （192w 栈 + snprintf ~200B 会溢出）。
 */
static void display_refresh(void)
{
    static char lines[APP_HMI_LINE_COUNT][APP_CTRL_LINE_BUF_CHARS];
    static const char *ptrs[APP_HMI_LINE_COUNT];

    if (s_ctx.mode == APP_CTRL_MODE_TUNING) {
        display_tuning_page(lines, ptrs);
    } else {
        display_status_page(lines, ptrs);
    }

    app_hmi_show(ptrs, APP_HMI_LINE_COUNT);
}

/**
 * @brief   进入故障模式（电机已由 svc_mot_ctrl 内部停机）
 */
static void enter_fault(void)
{
    LOG_E("app", "enter FAULT mode");
    s_ctx.mode = APP_CTRL_MODE_FAULT;
    led_update();
}

/**
 * @brief   启动流程：先校准，校准完成事件（ATT_CB）中再开平衡
 * @return  true 已受理
 */
static bool balance_start_request(void)
{
    if (s_ctx.mode == APP_CTRL_MODE_BALANCE
        || s_ctx.mode == APP_CTRL_MODE_CALIBRATING
        || s_ctx.mode == APP_CTRL_MODE_TUNING) {
        return true;    /* 幂等 */
    }

    if (svc_att_algo_calibrate_gyro() == SVC_OK) {
        s_ctx.mode = APP_CTRL_MODE_CALIBRATING;
    } else {
        s_ctx.mode = APP_CTRL_MODE_FAULT;
    }
    led_update();
    return true;
}

/**
 * @brief   处理姿态服务事件（校准结果驱动状态迁移）
 */
static void handle_att_event(uint8_t event)
{
    if (s_ctx.mode != APP_CTRL_MODE_CALIBRATING) {
        /* 运行中数据失效/恢复：仅 FAULT 联动（svc_mot_ctrl 已停机） */
        if ((event & (uint8_t)SVC_ATT_ALGO_EVT_DATA_LOST) != 0U
            && (s_ctx.mode == APP_CTRL_MODE_BALANCE
                || s_ctx.mode == APP_CTRL_MODE_TUNING)) {
            enter_fault();
        }
        return;
    }

    if ((event & (uint8_t)SVC_ATT_ALGO_EVT_CALIB_DONE) != 0U) {
        if (svc_mot_ctrl_balance_start() == SVC_OK) {
            LOG_I("app", "calib done, balance started");
            /* 校准期间收到调参请求 -> 直接进入 TUNING */
            s_ctx.mode = s_ctx.pending_tune ? APP_CTRL_MODE_TUNING
                                            : APP_CTRL_MODE_BALANCE;
            if (s_ctx.mode == APP_CTRL_MODE_TUNING) {
                LOG_I("app", "pid tuning mode on");
            }
        } else {
            LOG_E("app", "balance_start failed");
            s_ctx.mode = APP_CTRL_MODE_FAULT;
        }
    } else if ((event & (uint8_t)SVC_ATT_ALGO_EVT_CALIB_FAILED) != 0U) {
        /* 校准失败（运动干扰）：回到 IDLE，可重试 */
        LOG_W("app", "gyro calib failed (motion?), back to IDLE");
        s_ctx.mode = APP_CTRL_MODE_IDLE;
        s_ctx.pending_tune = false;
    } else {
        return;
    }
    led_update();
}

/**
 * @brief   处理运动服务事件（保护/故障联动）
 */
static void handle_mot_event(uint8_t event)
{
    if (s_ctx.mode != APP_CTRL_MODE_BALANCE
        && s_ctx.mode != APP_CTRL_MODE_TUNING) {
        return;
    }

    if ((event & (uint8_t)(SVC_MOT_CTRL_EVT_TILT_SHUTDOWN
                           | SVC_MOT_CTRL_EVT_MOTOR_FAULT
                           | SVC_MOT_CTRL_EVT_ATT_LOST)) != 0U) {
        enter_fault();
    }
}

/**
 * @brief   处理 PID 调节模式切换（指令 0x16 / arg: 1=进入 0=退出）
 */
static void handle_tune_event(uint8_t arg)
{
    if (arg != 0U) {
        /* 进入：仅校准/平衡/调节中受理；IDLE 先走启动流程（pending 生效） */
        if (s_ctx.mode == APP_CTRL_MODE_BALANCE) {
            s_ctx.mode = APP_CTRL_MODE_TUNING;
            LOG_I("app", "pid tuning mode on");
        } else if (s_ctx.mode == APP_CTRL_MODE_CALIBRATING
                   || s_ctx.mode == APP_CTRL_MODE_TUNING) {
            s_ctx.pending_tune = true;    /* 平衡启动后生效 / 已在调参 */
        } else if (s_ctx.mode == APP_CTRL_MODE_IDLE) {
            s_ctx.pending_tune = true;
            (void)balance_start_request();
        } else {
            LOG_W("app", "tune enter rejected in FAULT");
        }
    } else {
        /* 退出：回到普通平衡页（不停止电机） */
        s_ctx.pending_tune = false;
        if (s_ctx.mode == APP_CTRL_MODE_TUNING) {
            s_ctx.mode = APP_CTRL_MODE_BALANCE;
            LOG_I("app", "pid tuning mode off");
        }
    }
    led_update();
}

/**
 * @brief   app 控制任务主函数
 * @note    事件驱动 + 2Hz OLED 刷新：队列 500ms 超时兼作刷新节拍；
 *          事件到来时立即唤醒（零额外延迟），刷屏用 tick 判限流。
 */
static void app_ctrl_task(void *arg)
{
    (void)arg;
    app_evt_t evt;
    TickType_t last_disp_tick = 0U;

    /* 上电先画一屏（IDLE 页），之后周期刷新 */
    display_refresh();
    last_disp_tick = xTaskGetTickCount();

    for (;;) {
        evt.id = 0U;    /* 超时返回不写 evt，清零防止重复处理上一事件 */
        (void)xQueueReceive(s_ctx.evt_queue, &evt,
                            pdMS_TO_TICKS(APP_CTRL_DISPLAY_PERIOD_MS));

        if (evt.id != 0U) {
            switch (evt.id) {
            case APP_EVT_KEY_START:
                if (s_ctx.mode == APP_CTRL_MODE_FAULT) {
                    /* 故障恢复：手动确认后重新走启动流程 */
                    (void)svc_monitor_clear_fault();
                    s_ctx.pending_tune = false;
                    s_ctx.mode = APP_CTRL_MODE_IDLE;
                    led_update();
                }
                (void)balance_start_request();
                break;

            case APP_EVT_KEY_STOP:
                (void)svc_mot_ctrl_balance_stop();
                s_ctx.pending_tune = false;
                s_ctx.mode = APP_CTRL_MODE_IDLE;
                led_update();
                break;

            case APP_EVT_ATT_CB:
                handle_att_event(evt.arg);
                break;

            case APP_EVT_MOT_CB:
                handle_mot_event(evt.arg);
                break;

            case APP_EVT_MON_FAULT:
                /* 监视超时：看门狗已兜底，此处仅指示 */
                enter_fault();
                break;

            case APP_EVT_PID_TUNE:
                handle_tune_event(evt.arg);
                break;

            default:
                break;
            }
        }

        /* OLED 刷新：周期到即整屏重绘（页内容由当前 mode 决定） */
        if ((xTaskGetTickCount() - last_disp_tick)
                >= pdMS_TO_TICKS(APP_CTRL_DISPLAY_PERIOD_MS)) {
            display_refresh();
            last_disp_tick = xTaskGetTickCount();
        }
    }
}

/* ========================================================================== */
/*                              公开 API                                        */
/* ========================================================================== */

svc_err_t app_ctrl_init(void)
{
    /* --- 日志通道装配（最早：后续各阶段日志可见） --- */
    (void)mw_log_init(bsp_dbg_sink);
    LOG_I("app", "fw v" FW_VERSION_STRING " boot");

    /* --- 按键 / LED 设备（板 v2：2 键 2 LED，PA2/PA3 让位 USART2 日志） --- */
    s_ctx.key_start = dal_key_get_dev("key1");
    s_ctx.key_stop  = dal_key_get_dev("key2");
    s_ctx.led_run    = dal_led_get_dev("led1");
    s_ctx.led_fault  = dal_led_get_dev("led2");
    if (s_ctx.key_start == NULL || s_ctx.key_stop == NULL
        || s_ctx.led_run == NULL || s_ctx.led_fault == NULL) {
        return SVC_ERR_DEV;
    }

    /* --- 事件队列（静态创建） --- */
    s_ctx.evt_queue = xQueueCreateStatic(APP_CTRL_EVENT_QUEUE_DEPTH,
                                         sizeof(app_evt_t),
                                         s_ctx.evt_queue_storage,
                                         &s_ctx.evt_queue_mem);

    /* --- 按键回调：user_data 携带事件 id，ISR 侧只投队列 --- */
    (void)dal_key_set_callback(s_ctx.key_start, key_cb,
                               (void *)(uintptr_t)APP_EVT_KEY_START);
    (void)dal_key_set_callback(s_ctx.key_stop, key_cb,
                               (void *)(uintptr_t)APP_EVT_KEY_STOP);
    /* 失败必须可见：EXTI 配不上 = 按键整链路死（v2.3 修复前曾静默） */
    if (dal_key_set_irq_enable(s_ctx.key_start, true) != DAL_OK
        || dal_key_set_irq_enable(s_ctx.key_stop, true) != DAL_OK) {
        LOG_E("app", "key irq enable failed");
        return SVC_ERR_DEV;
    }

    /* --- 服务事件装配 --- */
    (void)svc_att_algo_register_event_cb(att_event_cb, NULL);
    (void)svc_mot_ctrl_register_event_cb(mot_event_cb, NULL);

    /* --- svc_monitor 装配：watch + 喂狗 --- */
    (void)svc_monitor_init(0U, wdg_feed);
    (void)svc_monitor_add_watch("att", att_activity, NULL, 0U);
    (void)svc_monitor_add_watch("mot", mot_activity, NULL, 0U);

    /* --- svc_comm 装配：指令分发 + 遥测 --- */
    (void)svc_comm_register_cmd(APP_COMM_CMD_SET_VEL, comm_cmd_handler, NULL);
    (void)svc_comm_register_cmd(APP_COMM_CMD_SET_TURN, comm_cmd_handler, NULL);
    (void)svc_comm_register_cmd(APP_COMM_CMD_SET_PID, comm_cmd_handler, NULL);
    (void)svc_comm_register_cmd(APP_COMM_CMD_STOP, comm_cmd_handler, NULL);
    (void)svc_comm_register_cmd(APP_COMM_CMD_HEARTBEAT, comm_cmd_handler, NULL);
    (void)svc_comm_register_cmd(APP_COMM_CMD_GET_VERSION, comm_cmd_handler, NULL);
    (void)svc_comm_register_cmd(APP_COMM_CMD_PID_TUNE, comm_cmd_handler, NULL);
    (void)svc_comm_register_cmd(APP_COMM_CMD_GET_PID, comm_cmd_handler, NULL);
    (void)svc_comm_set_telemetry(comm_telem_builder, NULL);

    /* --- OLED（I2C1）初始化：失败不阻断启动（屏可缺省） --- */
    if (app_hmi_init() != 0) {
        LOG_W("app", "oled init failed, display disabled");
    }

    /* --- IWDG 启动（之后不可停止） --- */
    (void)bsp_wdg_init(BSP_WDG_DEFAULT_TIMEOUT_MS);

    s_ctx.mode = APP_CTRL_MODE_IDLE;
    led_update();
    return SVC_OK;
}

svc_err_t app_ctrl_start(void)
{
    if (s_ctx.evt_queue == NULL) {
        return SVC_ERR_NOT_INIT;
    }
    if (s_ctx.task_running) {
        return SVC_ERR_BUSY;
    }

    s_ctx.task_handle = xTaskCreateStatic(app_ctrl_task, "app_ctrl",
                                          APP_CTRL_TASK_STACK_WORDS, NULL,
                                          APP_CTRL_TASK_PRIORITY,
                                          s_ctx.task_stack, &s_ctx.task_tcb);
    if (s_ctx.task_handle == NULL) {
        return SVC_ERR_FAIL;
    }

    s_ctx.task_running = true;
    return SVC_OK;
}
