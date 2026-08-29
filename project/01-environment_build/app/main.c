/**
 * @file    main.c
 * @brief   自平衡车主程序入口：系统装配与调度器启动
 *
 * 装配顺序（依赖自底向上，全部在调度器启动前完成）：
 *   bsp_init（含 esp8266 设备注册）
 *   -> svc_att_algo_init（姿态：控制链数据源）
 *   -> svc_mot_ctrl_init（运动：消费姿态快照）
 *   -> svc_comm_init（通信：RAW_SERIAL 透传模式，不连 WiFi）
 *   -> app_ctrl_init（模式状态机 + monitor/comm/看门狗装配）
 *   -> 各任务 start
 *   -> vTaskStartScheduler
 *
 * 每步装配/启动打一条里程碑日志：IWDG 复位循环时最后一行日志
 * 即为死点（Error_Handler 也已带串口输出）。
 *
 * @author  xserein
 * @version v1.2
 */

#include "board_v1.h"
#include "bsp_dbg.h"
#include "mw_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "svc_att_algo.h"
#include "svc_mot_ctrl.h"
#include "svc_monitor.h"
#include "svc_comm.h"
#include "app_ctrl.h"

/**
 * @brief   系统装配入口
 * @return  不会返回（调度器启动）或进入 Error_Handler
 */
int main(void)
{
    /* --- 日志通道装配（mw_log 提前初始化，后续每步里程碑可见） --- */
    (void)mw_log_init(bsp_dbg_sink);

    /* --- 板级初始化（含 mpu6050/oled/motor/encoder/key/led/esp8266 注册） --- */
    if (bsp_init() != BSP_OK) {
        Error_Handler();
    }
    /* 原始直写（不经 mw_log）：区分 bsp_init 死 / mw_log 输出链死 */
    (void)bsp_dbg_write("P:bsp-ok\n", 9U);
    /* 复位原因（raw hex：POR=0x08000000 PIN=0x04000000 SFT=0x10000000
     * IWDG=0x20000000 WWDG=0x40000000 LPWR=0x80000000）--稳定窗口打印 */
    bsp_dbg_report_reset_cause();
    LOG_I("main", "bsp init done");

    /* --- 服务层初始化（顺序即依赖顺序） --- */
    if (svc_att_algo_init(NULL) != SVC_OK) {    /* 姿态：默认配置 */
        Error_Handler();
    }
    LOG_I("main", "att init done");

    if (svc_mot_ctrl_init(NULL) != SVC_OK) {    /* 运动：默认 PID，上板整定 */
        Error_Handler();
    }
    LOG_I("main", "mot init done");
    const svc_comm_config_t comm_cfg = {
        .dev_name = "esp8266",
        .ssid = "xiamu",
        .password = "zhx815869815869",
    };
    if (svc_comm_init(&comm_cfg) != SVC_OK) {        /* 通信：透传模式（ssid=NULL） */
        Error_Handler();
    }
    LOG_I("main", "comm init done");

    if (app_ctrl_init() != SVC_OK) {            /* 应用：状态机 + 装配（IWDG 于此启动） */
        Error_Handler();
    }
    LOG_I("main", "app init done");

    /* --- 任务启动（优先级：mot_ctrl=4 > att/comm=3/2 > app=2 > monitor=1） --- */
    if (svc_att_algo_start() != SVC_OK) {
        Error_Handler();
    }
    if (svc_mot_ctrl_start() != SVC_OK) {
        Error_Handler();
    }
    if (svc_comm_start() != SVC_OK) {
        Error_Handler();
    }
    if (app_ctrl_start() != SVC_OK) {
        Error_Handler();
    }
    /* 喂狗任务必须最后启动：bsp_wdg 已在 app_ctrl_init 中运行，
     * monitor 不启动则 IWDG 500ms 后复位整机（boot 日志循环即此症状） */
    if (svc_monitor_start() != SVC_OK) {
        Error_Handler();
    }
    LOG_I("main", "tasks started, scheduler...");

    /* --- 调度器启动（不返回） --- */
    vTaskStartScheduler();

    /* 调度器启动失败（静态内存不足，理论不可达） */
    Error_Handler();
    return 0;
}
