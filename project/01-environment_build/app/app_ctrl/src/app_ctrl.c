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
#include "svc_att_algo.h"
#include "svc_mot_ctrl.h"
#include "svc_monitor.h"
#include "svc_comm.h"
#include "dal_key.h"
#include "dal_led.h"
#include "bsp_wdg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

/* ========================================================================== */
/*                          内部类型与静态状态                                   */
/* ========================================================================== */

/**
 * @brief 应用事件（队列元素）
 */
typedef enum {
    APP_EVT_KEY_START = 1,    /**< KEY1：启动（校准+平衡） */
    APP_EVT_KEY_STOP  = 2,    /**< KEY2：停止 */
    APP_EVT_KEY_ESTOP = 3,    /**< KEY3：急停 */
    APP_EVT_ATT_CB    = 4,    /**< 姿态服务事件（event 位域存 arg） */
    APP_EVT_MOT_CB    = 5,    /**< 运动服务事件（event 位域存 arg） */
    APP_EVT_MON_FAULT = 6,    /**< 监视超时（fault 掩码存 arg） */
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

    dal_key_dev_t *key_start;   /**< KEY1 */
    dal_key_dev_t *key_stop;    /**< KEY2 */
    dal_key_dev_t *key_estop;   /**< KEY3 */
    dal_led_dev_t *led_balance; /**< LED1 */
    dal_led_dev_t *led_calib;   /**< LED2 */
    dal_led_dev_t *led_fault;   /**< LED3 */

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
 */
static void led_update(void)
{
    (void)dal_led_set_state(s_ctx.led_balance,
                (s_ctx.mode == APP_CTRL_MODE_BALANCE) ? DAL_LED_ON
                                                      : DAL_LED_OFF);
    (void)dal_led_set_state(s_ctx.led_calib,
                (s_ctx.mode == APP_CTRL_MODE_CALIBRATING) ? DAL_LED_ON
                                                          : DAL_LED_OFF);
    (void)dal_led_set_state(s_ctx.led_fault,
                (s_ctx.mode == APP_CTRL_MODE_FAULT) ? DAL_LED_ON
                                                    : DAL_LED_OFF);
}

/**
 * @brief   进入故障模式（电机已由 svc_mot_ctrl 内部停机）
 */
static void enter_fault(void)
{
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
        || s_ctx.mode == APP_CTRL_MODE_CALIBRATING) {
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
            && s_ctx.mode == APP_CTRL_MODE_BALANCE) {
            enter_fault();
        }
        return;
    }

    if ((event & (uint8_t)SVC_ATT_ALGO_EVT_CALIB_DONE) != 0U) {
        if (svc_mot_ctrl_balance_start() == SVC_OK) {
            s_ctx.mode = APP_CTRL_MODE_BALANCE;
        } else {
            s_ctx.mode = APP_CTRL_MODE_FAULT;
        }
    } else if ((event & (uint8_t)SVC_ATT_ALGO_EVT_CALIB_FAILED) != 0U) {
        /* 校准失败（运动干扰）：回到 IDLE，可重试 */
        s_ctx.mode = APP_CTRL_MODE_IDLE;
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
    if (s_ctx.mode != APP_CTRL_MODE_BALANCE) {
        return;
    }

    if ((event & (uint8_t)(SVC_MOT_CTRL_EVT_TILT_SHUTDOWN
                           | SVC_MOT_CTRL_EVT_MOTOR_FAULT
                           | SVC_MOT_CTRL_EVT_ATT_LOST)) != 0U) {
        enter_fault();
    }
}

/**
 * @brief   app 控制任务主函数
 */
static void app_ctrl_task(void *arg)
{
    (void)arg;
    app_evt_t evt;

    for (;;) {
        (void)xQueueReceive(s_ctx.evt_queue, &evt, portMAX_DELAY);

        switch (evt.id) {
        case APP_EVT_KEY_START:
            if (s_ctx.mode == APP_CTRL_MODE_FAULT) {
                /* 故障恢复：手动确认后重新走启动流程 */
                (void)svc_monitor_clear_fault();
                s_ctx.mode = APP_CTRL_MODE_IDLE;
                led_update();
            }
            (void)balance_start_request();
            break;

        case APP_EVT_KEY_STOP:
        case APP_COMM_CMD_STOP:
            (void)svc_mot_ctrl_balance_stop();
            s_ctx.mode = APP_CTRL_MODE_IDLE;
            led_update();
            break;

        case APP_EVT_KEY_ESTOP:
            (void)svc_mot_ctrl_emergency_stop();
            enter_fault();
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

        default:
            break;
        }
    }
}

/* ========================================================================== */
/*                              公开 API                                        */
/* ========================================================================== */

svc_err_t app_ctrl_init(void)
{
    /* --- 按键 / LED 设备 --- */
    s_ctx.key_start = dal_key_get_dev("key1");
    s_ctx.key_stop  = dal_key_get_dev("key2");
    s_ctx.key_estop = dal_key_get_dev("key3");
    s_ctx.led_balance = dal_led_get_dev("led1");
    s_ctx.led_calib   = dal_led_get_dev("led2");
    s_ctx.led_fault   = dal_led_get_dev("led3");
    if (s_ctx.key_start == NULL || s_ctx.key_stop == NULL
        || s_ctx.key_estop == NULL || s_ctx.led_balance == NULL
        || s_ctx.led_calib == NULL || s_ctx.led_fault == NULL) {
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
    (void)dal_key_set_callback(s_ctx.key_estop, key_cb,
                               (void *)(uintptr_t)APP_EVT_KEY_ESTOP);
    (void)dal_key_set_irq_enable(s_ctx.key_start, true);
    (void)dal_key_set_irq_enable(s_ctx.key_stop, true);
    (void)dal_key_set_irq_enable(s_ctx.key_estop, true);

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
    (void)svc_comm_set_telemetry(comm_telem_builder, NULL);

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
