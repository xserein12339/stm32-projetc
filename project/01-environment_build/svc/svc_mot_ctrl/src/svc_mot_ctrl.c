/**
 * @file    svc_mot_ctrl.c
 * @brief   运动控制服务实现 v1.0
 *
 * 控制流水线（每 period_ms 一拍）：
 *   姿态快照拉取 -> 有效性/倒地检查 -> 直立环 PID -> 转向差叠加
 *   -> 双电机 PWM 下发；每 speed_loop_div 拍执行一次速度环修正设定角。
 *
 * 参考文档：本项目《需求分析》FR-CTRL/FR-MOT/FR-ENC、《开发手册》5.7
 * @author  xserein
 * @version v1.0
 */
#include "svc_mot_ctrl.h"
#include "svc_att_algo.h"
#include "dal_motor.h"
#include "dal_encoder.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* ========================================================================== */
/*                          内部类型与静态状态                                   */
/* ========================================================================== */

/**
 * @brief 服务内部运行时状态
 */
typedef struct {
    /* --- 配置（init 时定格） --- */
    svc_mot_ctrl_config_t cfg;

    /* --- 设备 --- */
    dal_motor_dev_t   *motor_a;
    dal_motor_dev_t   *motor_b;
    dal_encoder_dev_t *enc_left;
    dal_encoder_dev_t *enc_right;

    /* --- 任务（静态创建） --- */
    TaskHandle_t       task_handle;
    StaticTask_t       task_tcb;
    StackType_t        task_stack[SVC_MOT_CTRL_TASK_STACK_WORDS];
    bool               task_running;

    /* --- 快照互斥 --- */
    SemaphoreHandle_t  tlm_mutex;
    StaticSemaphore_t  tlm_mutex_mem;

    /* --- 遥测快照与状态 --- */
    svc_mot_ctrl_telemetry_t tlm;
    volatile svc_mot_ctrl_state_t state;
    volatile bool      balancing;      /**< 平衡使能（balance_start/stop） */

    /* --- 事件回调 --- */
    svc_mot_ctrl_event_cb_t event_cb;
    void                    *event_cb_user;

    /* --- 控制器 --- */
    mw_pid_t upright_pid;
    mw_pid_t speed_pid;
    q15_t    speed_angle_adj;   /**< 速度环对直立环设定角的修正（Q15） */

    /* --- 指令（跨任务写入，volatile） --- */
    volatile int32_t cmd_vel_r01;
    volatile int32_t cmd_turn;

    /* --- 电机故障（ISR 回调置位，任务轮询清除） --- */
    volatile uint32_t motor_fault_flags;

    /* --- 分频计数 --- */
    uint32_t speed_div_counter;
} mot_ctrl_ctx_t;

static mot_ctrl_ctx_t s_ctx;

/* ========================================================================== */
/*                          默认 PID 参数                                       */
/* ========================================================================== */

/*
 * 默认参数为工程初值，上板必须整定（见《需求分析》FR-CTRL-002 验证方法）：
 *   直立环（输入满量程 ±6°）：
 *     Kp=0.60 -> 约 10% 占空比 / 度
 *     Kd=0.35 -> 约 8% 占空比 / (100°/s)（微分作用于测量，即角速度阻尼）
 *   速度环（输入满量程 ±50RPM，输出修正设定角 ±0.3 Q15 ≈ ±1.8°）：
 *     Kp=0.30, Ki=0.05，积分限幅防漂移
 */
static const mw_pid_params_t s_upright_pid_def = {
    .Kp = Q15_HALF,               /* 0.5 */
    .Ki = 0,
    .Kd = 11469,                  /* 0.35 */
    .out_min = Q15_MIN,
    .out_max = Q15_MAX,
    .integral_min = Q15_MIN,
    .integral_max = Q15_MAX,
    .d_filter_coef = 9830,        /* 0.3 微分低通 */
};

static const mw_pid_params_t s_speed_pid_def = {
    .Kp = 9830,                   /* 0.3 */
    .Ki = 1638,                   /* 0.05 */
    .Kd = 0,
    .out_min = -9830,             /* ±0.3 Q15 ≈ ±1.8° 设定角修正 */
    .out_max = 9830,
    .integral_min = -6554,
    .integral_max = 6554,
    .d_filter_coef = 0,
};

/* ========================================================================== */
/*                          内部函数                                             */
/* ========================================================================== */

/**
 * @brief   毫度整数 -> Q15（按满量程线性映射，超界饱和）
 */
static q15_t angle_to_q15(int32_t mdeg)
{
    q30_t scaled = ((q30_t)mdeg << Q15_SHIFT) / SVC_MOT_CTRL_ANGLE_FS_MDEG;
    return q15_sat(scaled);
}

/**
 * @brief   0.1RPM 整数 -> Q15（按满量程线性映射）
 */
static q15_t vel_to_q15(int32_t vel_r01)
{
    q30_t scaled = ((q30_t)vel_r01 << Q15_SHIFT) / SVC_MOT_CTRL_VEL_FS_R01;
    return q15_sat(scaled);
}

/**
 * @brief   触发事件回调（控制任务上下文）
 */
static void event_notify(uint32_t event)
{
    if (s_ctx.event_cb != NULL) {
        s_ctx.event_cb(event, s_ctx.event_cb_user);
    }
}

/**
 * @brief   遥测快照发布（互斥保护）
 */
static void telemetry_publish(void)
{
    if (xSemaphoreTake(s_ctx.tlm_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    s_ctx.tlm.state        = s_ctx.state;
    s_ctx.tlm.timestamp_ms = (uint32_t)xTaskGetTickCount();
    xSemaphoreGive(s_ctx.tlm_mutex);
}

/**
 * @brief   停机并进入指定状态（保护/故障共用路径）
 */
static void shutdown_motors(svc_mot_ctrl_state_t new_state)
{
    (void)dal_motor_stop(s_ctx.motor_a);
    (void)dal_motor_stop(s_ctx.motor_b);
    (void)dal_motor_disable(s_ctx.motor_a);
    (void)dal_motor_disable(s_ctx.motor_b);

    mw_pid_reset(&s_ctx.upright_pid);
    mw_pid_reset(&s_ctx.speed_pid);
    s_ctx.speed_angle_adj = 0;
    s_ctx.balancing       = false;
    s_ctx.state           = new_state;
    telemetry_publish();
}

/**
 * @brief   下发双电机 PWM（占空比 + 方向，含转向差与方向取反）
 *
 * @param[in] base_pct  直立环输出的基础占空比（0~100）
 * @param[in] forward   true=前行方向（与倾角恢复方向一致）
 */
static void motors_apply(int32_t base_pct, bool forward)
{
    int32_t turn = s_ctx.cmd_turn; /* ±100 */

    /* 转向差：右转 = 左轮加速 / 右轮减速 */
    int32_t duty_l = base_pct + (turn * base_pct) / 100;
    int32_t duty_r = base_pct - (turn * base_pct) / 100;

    if (duty_l < 0)   duty_l = 0;
    if (duty_l > 100) duty_l = 100;
    if (duty_r < 0)   duty_r = 0;
    if (duty_r > 100) duty_r = 100;

    /* 方向映射：电机接线方向差异由 invert 标志吸收 */
    bool dir_l = s_ctx.cfg.motor_a_invert ? !forward : forward;
    bool dir_r = s_ctx.cfg.motor_b_invert ? !forward : forward;

    (void)dal_motor_set_direction(s_ctx.motor_a,
                                  dir_l ? DAL_MOTOR_DIR_CW : DAL_MOTOR_DIR_CCW);
    (void)dal_motor_set_direction(s_ctx.motor_b,
                                  dir_r ? DAL_MOTOR_DIR_CW : DAL_MOTOR_DIR_CCW);
    (void)dal_motor_set_duty(s_ctx.motor_a, (uint8_t)duty_l);
    (void)dal_motor_set_duty(s_ctx.motor_b, (uint8_t)duty_r);

    s_ctx.tlm.duty_left_pct  = (uint8_t)duty_l;
    s_ctx.tlm.duty_right_pct = (uint8_t)duty_r;
}

/**
 * @brief   电机故障回调（ISR 上下文：仅置标志）
 * @warning 遵守 dal_motor ISR 安全契约，禁止任何 DAL/阻塞调用
 */
static void motor_fault_cb(dal_motor_dev_t *dev, uint32_t event,
                           void *user_data)
{
    (void)dev;
    (void)user_data;
    s_ctx.motor_fault_flags |= event;
}

/**
 * @brief   速度环执行（每 speed_loop_div 拍一次）
 */
static void speed_loop_execute(int32_t vel_left, int32_t vel_right)
{
    int32_t vel_avg = (vel_left + vel_right) / 2;

    s_ctx.tlm.vel_left_r01  = vel_left;
    s_ctx.tlm.vel_right_r01 = vel_right;
    s_ctx.tlm.vel_target_r01 = s_ctx.cmd_vel_r01;

    q15_t out = mw_pid_update(&s_ctx.speed_pid,
                              vel_to_q15(s_ctx.cmd_vel_r01),
                              vel_to_q15(vel_avg));
    s_ctx.speed_angle_adj = s_ctx.cfg.speed_loop_invert ? -out : out;
}

/**
 * @brief   控制任务主函数
 */
static void mot_ctrl_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(s_ctx.cfg.period_ms);
    svc_att_algo_attitude_t att;
    dal_motor_state_t m_state;

    for (;;) {
        vTaskDelayUntil(&wake, period);

        /* --- 电机故障轮询（ISR 置位 + 状态机检查） --- */
        if (s_ctx.motor_fault_flags != 0) {
            shutdown_motors(SVC_MOT_CTRL_STATE_FAULT);
            event_notify(SVC_MOT_CTRL_EVT_MOTOR_FAULT);
            s_ctx.motor_fault_flags = 0;
            continue;
        }

        /* --- 姿态快照 --- */
        if (svc_att_algo_get_attitude(&att) != SVC_OK) {
            continue; /* 姿态服务未就绪（理论不可达），跳过本拍 */
        }
        s_ctx.tlm.angle_mdeg = att.angle_mdeg;
        s_ctx.tlm.rate_mdps  = att.rate_mdps;

        /* --- 电机状态轮询（DAL_MOTOR_STATE_FAULT 检查） --- */
        if (dal_motor_get_state(s_ctx.motor_a, &m_state) == DAL_OK
            && m_state == DAL_MOTOR_STATE_FAULT) {
            shutdown_motors(SVC_MOT_CTRL_STATE_FAULT);
            event_notify(SVC_MOT_CTRL_EVT_MOTOR_FAULT);
            continue;
        }

        /* --- 未开启平衡：维持停机，仅刷新遥测 --- */
        if (!s_ctx.balancing) {
            telemetry_publish();
            continue;
        }

        /* --- 姿态有效性检查 --- */
        if (!att.valid) {
            shutdown_motors(SVC_MOT_CTRL_STATE_FAULT);
            event_notify(SVC_MOT_CTRL_EVT_ATT_LOST);
            continue;
        }

        /* --- 倒地保护（FR-CTRL-004） --- */
        int32_t abs_angle = (att.angle_mdeg < 0) ? -att.angle_mdeg
                                                 : att.angle_mdeg;
        if (abs_angle > s_ctx.cfg.tilt_th_mdeg) {
            shutdown_motors(SVC_MOT_CTRL_STATE_PROTECTED);
            event_notify(SVC_MOT_CTRL_EVT_TILT_SHUTDOWN);
            continue;
        }

        /* --- 速度环（分频执行，编码器测速约定单任务调用） --- */
        if (++s_ctx.speed_div_counter >= s_ctx.cfg.speed_loop_div) {
            s_ctx.speed_div_counter = 0;
            int32_t vl = 0, vr = 0;
            (void)dal_encoder_get_velocity(s_ctx.enc_left, &vl);
            (void)dal_encoder_get_velocity(s_ctx.enc_right, &vr);
            speed_loop_execute(vl, vr);
        }

        /* --- 直立环：设定角 = 机械平衡角 + 速度环修正 --- */
        q15_t setpoint = angle_to_q15(s_ctx.cfg.balance_angle_mdeg)
                         + s_ctx.speed_angle_adj;
        q15_t out = mw_pid_update(&s_ctx.upright_pid, setpoint,
                                  angle_to_q15(att.angle_mdeg));

        /* 输出符号 -> 方向 + 占空比（符号约定：前倾为正时输出后驱） */
        int32_t duty_pct = ((int32_t)(out < 0 ? -out : out) * 100)
                           >> Q15_SHIFT;
        motors_apply(duty_pct, out < 0);

        telemetry_publish();
    }
}

/* ========================================================================== */
/*                              公开 API                                        */
/* ========================================================================== */

svc_err_t svc_mot_ctrl_init(const svc_mot_ctrl_config_t *cfg)
{
    if (s_ctx.state != SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_BUSY;
    }

    /* --- 配置定格（NULL / 0 字段回退默认值） --- */
    const svc_mot_ctrl_config_t *c = (cfg != NULL) ? cfg : NULL;
    s_ctx.cfg.motor_a_name = (c && c->motor_a_name) ? c->motor_a_name
                                    : SVC_MOT_CTRL_DEFAULT_MOTOR_A_NAME;
    s_ctx.cfg.motor_b_name = (c && c->motor_b_name) ? c->motor_b_name
                                    : SVC_MOT_CTRL_DEFAULT_MOTOR_B_NAME;
    s_ctx.cfg.enc_left_name = (c && c->enc_left_name) ? c->enc_left_name
                                     : SVC_MOT_CTRL_DEFAULT_ENC_L_NAME;
    s_ctx.cfg.enc_right_name = (c && c->enc_right_name) ? c->enc_right_name
                                      : SVC_MOT_CTRL_DEFAULT_ENC_R_NAME;
    s_ctx.cfg.motor_a_invert = (c != NULL) && c->motor_a_invert;
    s_ctx.cfg.motor_b_invert = (c != NULL) && c->motor_b_invert;
    s_ctx.cfg.speed_loop_invert = (c != NULL) && c->speed_loop_invert;
    s_ctx.cfg.period_ms = (c && c->period_ms) ? c->period_ms
                            : SVC_MOT_CTRL_DEFAULT_PERIOD_MS;
    s_ctx.cfg.speed_loop_div = (c && c->speed_loop_div) ? c->speed_loop_div
                                  : SVC_MOT_CTRL_DEFAULT_SPEED_DIV;
    s_ctx.cfg.tilt_th_mdeg = (c && c->tilt_th_mdeg) ? c->tilt_th_mdeg
                                : SVC_MOT_CTRL_DEFAULT_TILT_TH_MDEG;
    s_ctx.cfg.balance_angle_mdeg = (c != NULL) ? c->balance_angle_mdeg : 0;

    /* PID 参数：配置全 0 视为未指定，用默认值 */
    const mw_pid_params_t *up = &s_upright_pid_def;
    const mw_pid_params_t *sp = &s_speed_pid_def;
    if (c != NULL) {
        if (c->upright_pid.Kp != 0 || c->upright_pid.Kd != 0) {
            up = &c->upright_pid;
        }
        if (c->speed_pid.Kp != 0 || c->speed_pid.Ki != 0) {
            sp = &c->speed_pid;
        }
    }
    mw_pid_init(&s_ctx.upright_pid, up);
    mw_pid_init(&s_ctx.speed_pid, sp);

    /* --- 获取设备 --- */
    s_ctx.motor_a = dal_motor_get_dev(s_ctx.cfg.motor_a_name);
    s_ctx.motor_b = dal_motor_get_dev(s_ctx.cfg.motor_b_name);
    s_ctx.enc_left = dal_encoder_get_dev(s_ctx.cfg.enc_left_name);
    s_ctx.enc_right = dal_encoder_get_dev(s_ctx.cfg.enc_right_name);
    if (s_ctx.motor_a == NULL || s_ctx.motor_b == NULL
        || s_ctx.enc_left == NULL || s_ctx.enc_right == NULL) {
        return SVC_ERR_DEV;
    }

    /* --- 电机故障回调（ISR 置标志，任务轮询处理） --- */
    (void)dal_motor_set_fault_callback(s_ctx.motor_a, motor_fault_cb, NULL);
    (void)dal_motor_set_fault_callback(s_ctx.motor_b, motor_fault_cb, NULL);
    (void)dal_motor_set_fault_irq_enable(s_ctx.motor_a, true);
    (void)dal_motor_set_fault_irq_enable(s_ctx.motor_b, true);

    /* --- 运行时状态复位（上电默认 DISABLED，FR-MOT-002） --- */
    memset(&s_ctx.tlm, 0, sizeof(s_ctx.tlm));
    s_ctx.cmd_vel_r01 = 0;
    s_ctx.cmd_turn = 0;
    s_ctx.speed_angle_adj = 0;
    s_ctx.speed_div_counter = 0;
    s_ctx.motor_fault_flags = 0;
    s_ctx.balancing = false;
    s_ctx.task_running = false;
    s_ctx.state = SVC_MOT_CTRL_STATE_IDLE;

    (void)dal_motor_disable(s_ctx.motor_a);
    (void)dal_motor_disable(s_ctx.motor_b);

    return SVC_OK;
}

svc_err_t svc_mot_ctrl_start(void)
{
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }
    if (s_ctx.task_running) {
        return SVC_ERR_BUSY;
    }

    if (s_ctx.tlm_mutex == NULL) {
        s_ctx.tlm_mutex = xSemaphoreCreateMutexStatic(&s_ctx.tlm_mutex_mem);
    }

    s_ctx.task_handle = xTaskCreateStatic(mot_ctrl_task, "mot_ctrl",
                                          SVC_MOT_CTRL_TASK_STACK_WORDS, NULL,
                                          SVC_MOT_CTRL_TASK_PRIORITY,
                                          s_ctx.task_stack, &s_ctx.task_tcb);
    if (s_ctx.task_handle == NULL) {
        return SVC_ERR_FAIL;
    }

    s_ctx.task_running = true;
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_stop(void)
{
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }
    if (!s_ctx.task_running) {
        return SVC_ERR_STATE;
    }

    vTaskDelete(s_ctx.task_handle);
    s_ctx.task_handle = NULL;
    s_ctx.task_running = false;
    s_ctx.balancing = false;
    s_ctx.state = SVC_MOT_CTRL_STATE_IDLE;
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_balance_start(void)
{
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }
    if (!s_ctx.task_running) {
        return SVC_ERR_STATE;
    }

    if (dal_motor_enable(s_ctx.motor_a) != DAL_OK
        || dal_motor_enable(s_ctx.motor_b) != DAL_OK) {
        return SVC_ERR_DEV;
    }

    mw_pid_reset(&s_ctx.upright_pid);
    mw_pid_reset(&s_ctx.speed_pid);
    s_ctx.speed_angle_adj = 0;
    s_ctx.speed_div_counter = 0;
    s_ctx.balancing = true;
    s_ctx.state = SVC_MOT_CTRL_STATE_BALANCING;
    telemetry_publish();
    event_notify(SVC_MOT_CTRL_EVT_RECOVER);
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_balance_stop(void)
{
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    shutdown_motors(SVC_MOT_CTRL_STATE_IDLE);
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_emergency_stop(void)
{
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    shutdown_motors(SVC_MOT_CTRL_STATE_FAULT);
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_set_velocity(int32_t vel_r01)
{
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }
    s_ctx.cmd_vel_r01 = vel_r01;
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_set_turn(int32_t turn)
{
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }
    if (turn < -100 || turn > 100) {
        return SVC_ERR_PARAM;
    }
    s_ctx.cmd_turn = turn;
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_set_pid_params(svc_mot_ctrl_pid_loop_t loop,
                                      const mw_pid_params_t *params)
{
    if (params == NULL) {
        return SVC_ERR_PARAM;
    }
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    mw_pid_t *pid = (loop == SVC_MOT_CTRL_PID_UPRIGHT) ? &s_ctx.upright_pid
                                                       : &s_ctx.speed_pid;
    mw_pid_set_kp(pid, params->Kp);
    mw_pid_set_ki(pid, params->Ki);
    mw_pid_set_kd(pid, params->Kd);
    mw_pid_set_out_limit(pid, params->out_min, params->out_max);
    mw_pid_set_integral_limit(pid, params->integral_min, params->integral_max);
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_get_telemetry(svc_mot_ctrl_telemetry_t *out)
{
    if (out == NULL) {
        return SVC_ERR_PARAM;
    }
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    if (xSemaphoreTake(s_ctx.tlm_mutex, portMAX_DELAY) != pdTRUE) {
        return SVC_ERR_FAIL;
    }
    *out = s_ctx.tlm;
    xSemaphoreGive(s_ctx.tlm_mutex);
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_get_state(svc_mot_ctrl_state_t *state)
{
    if (state == NULL) {
        return SVC_ERR_PARAM;
    }
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    *state = s_ctx.state;
    return SVC_OK;
}

svc_err_t svc_mot_ctrl_register_event_cb(svc_mot_ctrl_event_cb_t cb,
                                         void *user_data)
{
    if (s_ctx.state == SVC_MOT_CTRL_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    s_ctx.event_cb = cb;
    s_ctx.event_cb_user = user_data;
    return SVC_OK;
}
