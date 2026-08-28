/**
 * @file    svc_monitor.h
 * @brief   系统监控服务 v1.0（看门狗喂狗 + 关键任务活跃性监视）
 *
 * 职责（对应《需求分析》FR-FAULT 系列）：
 *   1. 周期性喂独立看门狗（IWDG 喂狗函数由 app 注入，本服务不接触 BSP）
 *   2. 监视关键任务活跃性：app 将各服务快照时间戳包装成 watch 槽位，
 *      超时则置位故障掩码（拉模式查询，由 app 决定联动动作）
 *
 * 本服务为叶子服务：不依赖其他 svc，仅依赖 FreeRTOS。
 *
 * 参考文档：本项目《开发手册》5.7 服务层划分
 * @author  xserein
 * @version v1.0
 */

#ifndef __SVC_MONITOR_H__
#define __SVC_MONITOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "svc_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  配置默认值
 * ================================================================ */

#define SVC_MONITOR_DEFAULT_PERIOD_MS      (100U)  /**< 监控周期 10Hz */
#define SVC_MONITOR_TASK_PRIORITY          (1U)    /**< 最低业务优先级 */
#define SVC_MONITOR_TASK_STACK_WORDS       (160U)  /**< 640B 栈 */

#define SVC_MONITOR_MAX_WATCHES            (4U)    /**< watch 槽位上限 */

/* ================================================================
 *  数据类型
 * ================================================================ */

/**
 * @brief 活跃性查询函数原型（app 侧包装各服务快照时间戳）
 *
 * @param[in] user 注册时传入的用户上下文
 * @return  最近一次活动的系统 tick（ms）
 *
 * @note  在监控任务上下文调用；实现应只做快照读取，
 *        禁止阻塞（互斥量带超时或无锁设计）。
 */
typedef uint32_t (*svc_monitor_activity_fn_t)(void *user);

/**
 * @brief 喂狗函数原型（app 注入 bsp_wdg_refresh 的包装）
 * @note  在监控任务上下文调用，须非阻塞。
 */
typedef void (*svc_monitor_feed_fn_t)(void);

/** 故障掩码位定义 */
#define SVC_MONITOR_FAULT_WATCH0     (0x01U)  /**< watch 槽 0 超时 */
#define SVC_MONITOR_FAULT_WATCH1     (0x02U)  /**< watch 槽 1 超时 */
#define SVC_MONITOR_FAULT_WATCH2     (0x04U)  /**< watch 槽 2 超时 */
#define SVC_MONITOR_FAULT_WATCH3     (0x08U)  /**< watch 槽 3 超时 */

/* ================================================================
 *  公开 API
 * ================================================================ */

/**
 * @brief   初始化监控服务（不创建任务）
 *
 * @param[in] period_ms 监控周期 ms（0 用默认值 100ms）
 * @param[in] feed_fn   喂狗函数（NULL 表示不喂狗，仅监视）
 * @return  SVC_OK 成功；SVC_ERR_BUSY 重复初始化
 *
 * @note  须在调度器启动前调用；若 feed_fn 非空，应先完成
 *        bsp_wdg_init()（IWDG 启动后不可停止）。
 */
svc_err_t svc_monitor_init(uint32_t period_ms, svc_monitor_feed_fn_t feed_fn);

/**
 * @brief   添加活跃性监视槽位
 *
 * @param[in] name       槽位名（诊断用，可 NULL）
 * @param[in] activity   活跃性查询函数（NULL 拒绝）
 * @param[in] user       传给 activity 的用户上下文（可 NULL）
 * @param[in] timeout_ms 超时阈值 ms（0 用默认值 = 3 × 监控周期）
 * @return  >=0 槽位索引；负数错误码（SVC_ERR_PARAM / SVC_ERR_BUSY 槽满）
 */
int32_t svc_monitor_add_watch(const char *name,
                              svc_monitor_activity_fn_t activity,
                              void *user, uint32_t timeout_ms);

/**
 * @brief   启动监控任务（静态创建）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT / SVC_ERR_FAIL
 */
svc_err_t svc_monitor_start(void);

/**
 * @brief   停止监控任务
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT / SVC_ERR_STATE
 */
svc_err_t svc_monitor_stop(void);

/**
 * @brief   查询故障掩码（拉模式）
 * @param[out] mask 故障位组合（见 SVC_MONITOR_FAULT_x）
 * @return  SVC_OK 成功；SVC_ERR_PARAM / SVC_ERR_NOT_INIT
 */
svc_err_t svc_monitor_get_fault_mask(uint32_t *mask);

/**
 * @brief   清除故障掩码（故障恢复后由 app 调用）
 * @return  SVC_OK 成功；SVC_ERR_NOT_INIT
 * @note    仅清除记录，不触发任何联动；恢复动作由 app 状态机决定。
 */
svc_err_t svc_monitor_clear_fault(void);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_MONITOR_H__ */
