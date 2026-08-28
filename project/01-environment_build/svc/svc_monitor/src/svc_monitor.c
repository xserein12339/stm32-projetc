/**
 * @file    svc_monitor.c
 * @brief   系统监控服务实现 v1.0
 *
 * 监控周期内依次执行：喂狗 -> 扫描 watch 槽位（对比当前 tick 与
 * 最近活动 tick，超时置位故障掩码）。
 *
 * @author  xserein
 * @version v1.0
 */
#include "svc_monitor.h"
#include "FreeRTOS.h"
#include "task.h"

/* ========================================================================== */
/*                          内部类型与静态状态                                   */
/* ========================================================================== */

/**
 * @brief watch 槽位运行时描述
 */
typedef struct {
    const char                 *name;
    svc_monitor_activity_fn_t   activity;
    void                       *user;
    uint32_t                    timeout_ms;
    bool                        used;
} watch_slot_t;

/**
 * @brief 服务内部运行时状态
 */
typedef struct {
    uint32_t            period_ms;
    svc_monitor_feed_fn_t feed_fn;

    watch_slot_t        watches[SVC_MONITOR_MAX_WATCHES];

    volatile uint32_t   fault_mask;

    TaskHandle_t        task_handle;
    StaticTask_t        task_tcb;
    StackType_t         task_stack[SVC_MONITOR_TASK_STACK_WORDS];
    bool                task_running;
} monitor_ctx_t;

static monitor_ctx_t s_ctx;

/* ========================================================================== */
/*                          内部函数                                             */
/* ========================================================================== */

/**
 * @brief   监控任务主函数
 */
static void monitor_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(s_ctx.period_ms);

    for (;;) {
        vTaskDelayUntil(&wake, period);

        /* --- 喂狗（每周期一次，超时 = 5 × 周期时才复位） --- */
        if (s_ctx.feed_fn != NULL) {
            s_ctx.feed_fn();
        }

        /* --- watch 槽位扫描 --- */
        uint32_t now = (uint32_t)xTaskGetTickCount();
        for (uint32_t i = 0; i < SVC_MONITOR_MAX_WATCHES; i++) {
            const watch_slot_t *w = &s_ctx.watches[i];
            if (!w->used || w->activity == NULL) {
                continue;
            }

            uint32_t last = w->activity(w->user);
            uint32_t elapsed = now - last;  /* 无符号回绕安全 */

            if (elapsed > w->timeout_ms) {
                s_ctx.fault_mask |= (1UL << i);
            }
        }
    }
}

/* ========================================================================== */
/*                              公开 API                                        */
/* ========================================================================== */

svc_err_t svc_monitor_init(uint32_t period_ms, svc_monitor_feed_fn_t feed_fn)
{
    if (s_ctx.task_running) {
        return SVC_ERR_BUSY;
    }

    s_ctx.period_ms = (period_ms != 0U) ? period_ms
                                        : SVC_MONITOR_DEFAULT_PERIOD_MS;
    s_ctx.feed_fn   = feed_fn;
    s_ctx.fault_mask = 0;

    for (uint32_t i = 0; i < SVC_MONITOR_MAX_WATCHES; i++) {
        s_ctx.watches[i].used = false;
    }

    return SVC_OK;
}

int32_t svc_monitor_add_watch(const char *name,
                              svc_monitor_activity_fn_t activity,
                              void *user, uint32_t timeout_ms)
{
    if (activity == NULL) {
        return SVC_ERR_PARAM;
    }
    if (s_ctx.task_running) {
        return SVC_ERR_BUSY;    /* 运行中禁止改槽位，避免并发访问 */
    }

    for (uint32_t i = 0; i < SVC_MONITOR_MAX_WATCHES; i++) {
        watch_slot_t *w = &s_ctx.watches[i];
        if (w->used) {
            continue;
        }

        w->name       = name;
        w->activity   = activity;
        w->user       = user;
        w->timeout_ms = (timeout_ms != 0U) ? timeout_ms
                                           : 3U * s_ctx.period_ms;
        w->used       = true;
        return (int32_t)i;
    }

    return SVC_ERR_BUSY;    /* 槽位已满 */
}

svc_err_t svc_monitor_start(void)
{
    if (s_ctx.period_ms == 0U) {
        return SVC_ERR_NOT_INIT;
    }
    if (s_ctx.task_running) {
        return SVC_ERR_BUSY;
    }

    s_ctx.task_handle = xTaskCreateStatic(monitor_task, "monitor",
                                          SVC_MONITOR_TASK_STACK_WORDS, NULL,
                                          SVC_MONITOR_TASK_PRIORITY,
                                          s_ctx.task_stack, &s_ctx.task_tcb);
    if (s_ctx.task_handle == NULL) {
        return SVC_ERR_FAIL;
    }

    s_ctx.task_running = true;
    return SVC_OK;
}

svc_err_t svc_monitor_stop(void)
{
    if (!s_ctx.task_running) {
        return SVC_ERR_STATE;
    }

    vTaskDelete(s_ctx.task_handle);
    s_ctx.task_handle = NULL;
    s_ctx.task_running = false;
    return SVC_OK;
}

svc_err_t svc_monitor_get_fault_mask(uint32_t *mask)
{
    if (mask == NULL) {
        return SVC_ERR_PARAM;
    }

    *mask = s_ctx.fault_mask;
    return SVC_OK;
}

svc_err_t svc_monitor_clear_fault(void)
{
    s_ctx.fault_mask = 0;
    return SVC_OK;
}
