/**
 * @file    svc_att_algo.c
 * @brief   姿态解算服务实现 v1.0
 *
 * 实现要点：
 *   - 采样：200Hz 轮询 dal_imu_read（DRDY/FIFO 驱动暂未支持，轮询为当前方案）
 *   - 融合：互补滤波（加速度计角度低通 + 陀螺仪积分高通）
 *   - 校准：静置采样均值 -> dal_imu_set_calibration 注入驱动层
 *   - 有效性：连续 3 次读取失败置失效标志（FR-ATT-003）
 *
 * 参考文档：本项目《需求分析》FR-ATT、《开发手册》5.2 / 5.6
 * @author  xserein
 * @version v1.0
 */
#include "svc_att_algo.h"
#include "mw_common.h"
#include "dal_imu.h"
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
    svc_att_algo_config_t cfg;

    /* --- 设备 --- */
    dal_imu_dev_t *imu_dev;

    /* --- 任务（静态创建） --- */
    TaskHandle_t         task_handle;
    StaticTask_t         task_tcb;
    StackType_t          task_stack[SVC_ATT_ALGO_TASK_STACK_WORDS];
    bool                 task_running;

    /* --- 快照互斥 --- */
    SemaphoreHandle_t    snap_mutex;
    StaticSemaphore_t    snap_mutex_mem;

    /* --- 快照与状态 --- */
    svc_att_algo_attitude_t snapshot;
    volatile svc_att_algo_state_t state;

    /* --- 事件回调 --- */
    svc_att_algo_event_cb_t event_cb;
    void                   *event_cb_user;

    /* --- 采样任务内部状态 --- */
    int32_t  angle_mdeg;        /**< 融合角（毫度） */
    int32_t  gyro_rem_udeg;     /**< 陀螺仪积分亚毫度余数（防截断漂移） */
    uint32_t fail_count;        /**< 连续读取失败计数 */
    volatile bool calib_request;/**< 校准请求标志（跨任务置位） */
} att_algo_ctx_t;

static att_algo_ctx_t s_ctx;

/* ========================================================================== */
/*                          内部函数                                             */
/* ========================================================================== */

/**
 * @brief   发布快照（互斥保护）
 */
static void snapshot_publish(int32_t angle_mdeg, int32_t rate_mdps,
                             bool valid)
{
    if (xSemaphoreTake(s_ctx.snap_mutex, portMAX_DELAY) != pdTRUE) {
        return; /* 静态互斥锁创建成功后不会失败，防御性返回 */
    }
    s_ctx.snapshot.angle_mdeg   = angle_mdeg;
    s_ctx.snapshot.rate_mdps    = rate_mdps;
    s_ctx.snapshot.timestamp_ms = (uint32_t)xTaskGetTickCount();
    s_ctx.snapshot.valid        = valid;
    xSemaphoreGive(s_ctx.snap_mutex);
}

/**
 * @brief   触发事件回调（服务任务上下文）
 */
static void event_notify(uint32_t event)
{
    if (s_ctx.event_cb != NULL) {
        s_ctx.event_cb(event, s_ctx.event_cb_user);
    }
}

/**
 * @brief   从原始数据计算加速度计角度基准
 *
 * 轴向约定（与 MPU6050 在车体上的安装方向绑定，装反则改此处两行）：
 *   angle_accel = atan2(accel_y, accel_z)，车体前倾为正。
 *
 * @param[in] data  IMU 原始数据
 * @return     加速度计角度（毫度）
 */
static int32_t accel_angle_compute(const dal_imu_data_t *data)
{
    return mw_math_atan2_mdg(data->accel_mg.y, data->accel_mg.z);
}

/**
 * @brief   执行一次互补滤波迭代
 *
 * @param[in] angle_accel_mdeg  加速度计角度基准（毫度）
 * @param[in] gyro_mdps         角速度（零偏校正后，毫度/秒）
 *
 * @note    陀螺仪积分项保留亚毫度余数（gyro_rem_udeg），
 *          WHY: 直接毫度截断会在小角速度时丢失积分（如 10mdps * 5ms
 *          = 0.05mdeg/拍，全被截断），导致长时间静置角度漂移。
 *          加速度计修正项一次性作用于毫度域，无累积截断问题。
 */
static void complementary_filter_update(int32_t angle_accel_mdeg,
                                        int32_t gyro_mdps)
{
    /* --- 陀螺仪积分（带余数累积） --- */
    int32_t inc_udeg = gyro_mdps * (int32_t)s_ctx.cfg.period_ms
                       + s_ctx.gyro_rem_udeg;
    s_ctx.angle_mdeg     += inc_udeg / 1000;
    s_ctx.gyro_rem_udeg   = inc_udeg % 1000;

    /* --- 加速度计修正：angle += (accel - angle) * alpha --- */
    int32_t diff = angle_accel_mdeg - s_ctx.angle_mdeg;
    s_ctx.angle_mdeg += (diff * (int32_t)s_ctx.cfg.alpha_q15) >> Q15_SHIFT;
}

/**
 * @brief   执行陀螺仪零偏校准流程（在采样任务内同步执行）
 *
 * @return  true 校准成功；false 失败（运动干扰或读取失败）
 *
 * @note    校准期间置失效快照（valid=false），防止控制环使用冻结角度。
 */
static bool gyro_calibration_execute(void)
{
    dal_imu_data_t data;
    int32_t  sum_x = 0;
    int32_t  avg   = 0;
    uint16_t done  = 0;

    s_ctx.state = SVC_ATT_ALGO_STATE_CALIBRATING;
    s_ctx.snapshot.valid = false; /* 快照由校准流程独占更新，无需持锁 */

    while (done < s_ctx.cfg.calib_samples) {
        vTaskDelay(pdMS_TO_TICKS(s_ctx.cfg.period_ms));

        if (dal_imu_read(s_ctx.imu_dev, &data) != DAL_OK) {
            return false;
        }

        int32_t gx = data.gyro_mdps.x;
        sum_x += gx;
        done++;

        /* 运动检测：偏离运行均值超过阈值即判定小车在校准期间被移动 */
        avg = sum_x / (int32_t)done;
        int32_t deviation = (gx > avg) ? (gx - avg) : (avg - gx);
        if (deviation > s_ctx.cfg.calib_motion_th_mdps) {
            return false;
        }
    }

    /* 校准值注入驱动层（read 转换时自动扣除零偏） */
    dal_imu_calibration_t cal = {
        .gyro_offset_mdps = { .x = avg, .y = 0, .z = 0 },
    };
    if (dal_imu_set_calibration(s_ctx.imu_dev, &cal) != DAL_OK) {
        return false;
    }

    /* 融合角重置为当前加速度计角，避免旧积分值污染 */
    if (dal_imu_read(s_ctx.imu_dev, &data) == DAL_OK) {
        s_ctx.angle_mdeg     = accel_angle_compute(&data);
        s_ctx.gyro_rem_udeg  = 0;
    }

    return true;
}

/**
 * @brief   采样任务主函数
 */
static void att_algo_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(s_ctx.cfg.period_ms);
    dal_imu_data_t data;

    for (;;) {
        vTaskDelayUntil(&wake, period);

        /* --- 校准请求优先处理（期间暂停正常采样） --- */
        if (s_ctx.calib_request) {
            s_ctx.calib_request = false;
            if (gyro_calibration_execute()) {
                event_notify(SVC_ATT_ALGO_EVT_CALIB_DONE);
            } else {
                /* 校准失败（多为运动干扰）不等于数据失效，
                 * 采样恢复 RUNNING；若因读取失败，后续失效检测会另行置 FAULT */
                event_notify(SVC_ATT_ALGO_EVT_CALIB_FAILED);
            }
            s_ctx.state = SVC_ATT_ALGO_STATE_RUNNING;
            wake = xTaskGetTickCount(); /* 校准耗时后重定基准，防追赶风暴 */
            continue;
        }

        /* --- 正常采样与解算 --- */
        if (dal_imu_read(s_ctx.imu_dev, &data) == DAL_OK) {
            if (s_ctx.fail_count >= SVC_ATT_ALGO_FAIL_COUNT_MAX) {
                /* 失效恢复：融合角重置为加速度计角，丢弃失效期积分 */
                s_ctx.angle_mdeg    = accel_angle_compute(&data);
                s_ctx.gyro_rem_udeg = 0;
                s_ctx.state = SVC_ATT_ALGO_STATE_RUNNING;
                event_notify(SVC_ATT_ALGO_EVT_DATA_RECOVER);
            }
            s_ctx.fail_count = 0;

            complementary_filter_update(accel_angle_compute(&data),
                                        data.gyro_mdps.x);
            snapshot_publish(s_ctx.angle_mdeg, data.gyro_mdps.x, true);
        } else {
            s_ctx.fail_count++;
            if (s_ctx.fail_count >= SVC_ATT_ALGO_FAIL_COUNT_MAX
                && s_ctx.state != SVC_ATT_ALGO_STATE_FAULT) {
                s_ctx.state = SVC_ATT_ALGO_STATE_FAULT;
                snapshot_publish(s_ctx.angle_mdeg, 0, false);
                event_notify(SVC_ATT_ALGO_EVT_DATA_LOST);
            }
        }
    }
}

/* ========================================================================== */
/*                              公开 API                                        */
/* ========================================================================== */

svc_err_t svc_att_algo_init(const svc_att_algo_config_t *cfg)
{
    if (s_ctx.state != SVC_ATT_ALGO_STATE_UNINIT) {
        return SVC_ERR_BUSY;
    }

    /* --- 配置定格（NULL / 0 字段回退默认值） --- */
    s_ctx.cfg.imu_name = (cfg != NULL && cfg->imu_name != NULL)
                             ? cfg->imu_name : SVC_ATT_ALGO_DEFAULT_IMU_NAME;
    s_ctx.cfg.period_ms = (cfg != NULL && cfg->period_ms != 0)
                              ? cfg->period_ms : SVC_ATT_ALGO_DEFAULT_PERIOD_MS;
    s_ctx.cfg.alpha_q15 = (cfg != NULL && cfg->alpha_q15 != 0)
                              ? cfg->alpha_q15 : SVC_ATT_ALGO_DEFAULT_ALPHA_Q15;
    s_ctx.cfg.calib_samples = (cfg != NULL && cfg->calib_samples != 0)
                                  ? cfg->calib_samples
                                  : SVC_ATT_ALGO_DEFAULT_CALIB_SAMPLES;
    s_ctx.cfg.calib_motion_th_mdps =
        (cfg != NULL && cfg->calib_motion_th_mdps != 0)
            ? cfg->calib_motion_th_mdps
            : SVC_ATT_ALGO_DEFAULT_MOTION_TH_MDPS;

    /* --- 获取 IMU 设备并配置 ODR --- */
    s_ctx.imu_dev = dal_imu_get_dev(s_ctx.cfg.imu_name);
    if (s_ctx.imu_dev == NULL) {
        return SVC_ERR_DEV;
    }

    /* ODR 提升至 200Hz 匹配采样周期（驱动默认 100Hz） */
    (void)dal_imu_set_odr(s_ctx.imu_dev,
                          DAL_IMU_MODULE_ACCEL | DAL_IMU_MODULE_GYRO,
                          DAL_IMU_ODR_200);

    /* --- 运行时状态复位 --- */
    memset(&s_ctx.snapshot, 0, sizeof(s_ctx.snapshot));
    s_ctx.angle_mdeg    = 0;
    s_ctx.gyro_rem_udeg = 0;
    s_ctx.fail_count    = 0;
    s_ctx.calib_request = false;
    s_ctx.task_running  = false;
    s_ctx.state         = SVC_ATT_ALGO_STATE_IDLE;

    return SVC_OK;
}

svc_err_t svc_att_algo_start(void)
{
    if (s_ctx.state == SVC_ATT_ALGO_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }
    if (s_ctx.task_running) {
        return SVC_ERR_BUSY;
    }

    /* 互斥锁静态创建（仅首次） */
    if (s_ctx.snap_mutex == NULL) {
        s_ctx.snap_mutex = xSemaphoreCreateMutexStatic(&s_ctx.snap_mutex_mem);
    }

    s_ctx.task_handle = xTaskCreateStatic(att_algo_task, "att_algo",
                                          SVC_ATT_ALGO_TASK_STACK_WORDS, NULL,
                                          SVC_ATT_ALGO_TASK_PRIORITY,
                                          s_ctx.task_stack, &s_ctx.task_tcb);
    if (s_ctx.task_handle == NULL) {
        return SVC_ERR_FAIL;
    }

    s_ctx.task_running = true;
    s_ctx.state        = SVC_ATT_ALGO_STATE_RUNNING;
    return SVC_OK;
}

svc_err_t svc_att_algo_stop(void)
{
    if (s_ctx.state == SVC_ATT_ALGO_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }
    if (!s_ctx.task_running) {
        return SVC_ERR_STATE;
    }

    vTaskDelete(s_ctx.task_handle);
    s_ctx.task_handle = NULL;
    s_ctx.task_running = false;
    s_ctx.state        = SVC_ATT_ALGO_STATE_IDLE;
    return SVC_OK;
}

svc_err_t svc_att_algo_get_attitude(svc_att_algo_attitude_t *out)
{
    if (out == NULL) {
        return SVC_ERR_PARAM;
    }
    if (s_ctx.state == SVC_ATT_ALGO_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    if (xSemaphoreTake(s_ctx.snap_mutex, portMAX_DELAY) != pdTRUE) {
        return SVC_ERR_FAIL;
    }
    *out = s_ctx.snapshot;
    xSemaphoreGive(s_ctx.snap_mutex);
    return SVC_OK;
}

svc_err_t svc_att_algo_get_state(svc_att_algo_state_t *state)
{
    if (state == NULL) {
        return SVC_ERR_PARAM;
    }
    if (s_ctx.state == SVC_ATT_ALGO_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    *state = s_ctx.state;
    return SVC_OK;
}

svc_err_t svc_att_algo_calibrate_gyro(void)
{
    if (s_ctx.state == SVC_ATT_ALGO_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }
    if (s_ctx.state == SVC_ATT_ALGO_STATE_CALIBRATING) {
        return SVC_ERR_BUSY;
    }
    if (!s_ctx.task_running) {
        return SVC_ERR_STATE;
    }

    s_ctx.calib_request = true;
    return SVC_OK;
}

svc_err_t svc_att_algo_register_event_cb(svc_att_algo_event_cb_t cb,
                                         void *user_data)
{
    if (s_ctx.state == SVC_ATT_ALGO_STATE_UNINIT) {
        return SVC_ERR_NOT_INIT;
    }

    s_ctx.event_cb       = cb;
    s_ctx.event_cb_user  = user_data;
    return SVC_OK;
}
