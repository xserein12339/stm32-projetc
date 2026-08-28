/**
 * @file    svc_comm.h
 * @brief   通信服务 v1.0（帧协议编解码 + 指令分发 + 周期遥测上报）
 *
 * 帧格式（上行/下行统一）：
 *   [0xA5][cmd][len][payload × len][crc8]     len ≤ 56
 *   crc8 覆盖 cmd + len + payload，多项式 0x07，初值 0x00
 *
 * 职责边界（业务无关，对应《开发手册》5.7）：
 *   - 接收：RAW_SERIAL 数据回调 -> SPSC 无锁环形缓冲 -> 任务内解析分发
 *   - 发送：帧编码 + dal_wifi_transmit
 *   - 指令语义（cmd 编号含义、payload 格式）由 app 注册的 handler 定义
 *
 * 参考文档：本项目《需求分析》FR-COMM-001/002
 * @author  xserein
 * @version v1.0
 */

#ifndef __SVC_COMM_H__
#define __SVC_COMM_H__

#include <stdint.h>
#include <stdbool.h>
#include "svc_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  配置默认值
 * ================================================================ */

#define SVC_COMM_DEFAULT_DEV_NAME         "esp8266"
#define SVC_COMM_DEFAULT_TELEM_PERIOD_MS  (100U)   /**< 遥测上报周期 10Hz */
#define SVC_COMM_DEFAULT_TELEM_CMD        (0x00U)  /**< 遥测帧 cmd 编号 */

#define SVC_COMM_MAX_PAYLOAD              (56U)    /**< payload 上限 */
#define SVC_COMM_RING_SIZE                (256U)   /**< RX 环形缓冲（2 的幂） */

#define SVC_COMM_TASK_PRIORITY            (2U)     /**< 低于采样/控制 */
#define SVC_COMM_TASK_STACK_WORDS         (256U)   /**< 1KB 栈 */

/* ================================================================
 *  数据类型
 * ================================================================ */

/**
 * @brief 指令处理回调（svc_comm 任务上下文执行）
 *
 * @param[in] cmd      帧指令编号
 * @param[in] payload  载荷指针（任务栈上，回调内须同步消费，不可保存指针）
 * @param[in] len      载荷长度（0~SVC_COMM_MAX_PAYLOAD）
 * @param[in] user     注册时传入的用户上下文
 *
 * @note  允许调用 svc_mot_ctrl 等上层 API；禁止长时间阻塞
 *        （阻塞会延迟后续帧解析与遥测上报）。
 */
typedef void (*svc_comm_cmd_fn_t)(uint8_t cmd, const uint8_t *payload,
                                  uint8_t len, void *user);

/**
 * @brief 遥测构造回调（svc_comm 任务上下文执行）
 *
 * @param[out] buf  载荷缓冲区（SVC_COMM_MAX_PAYLOAD 字节）
 * @param[in]  user 注册时传入的用户上下文
 * @return  载荷实际长度（0 表示本周期不上报）
 */
typedef uint8_t (*svc_comm_telem_fn_t)(uint8_t *buf, void *user);

/**
 * @brief 服务配置
 */
typedef struct {
    const char *dev_name;            /**< WiFi 设备名（NULL 用默认值） */
    const char *ssid;                /**< STA SSID（NULL = 不连 WiFi，透传串口模式） */
    const char *password;            /**< STA 密码（NULL 视为开放网络） */
    uint32_t    telemetry_period_ms; /**< 遥测周期 ms（0 关闭周期上报） */
    uint8_t     telemetry_cmd;       /**< 遥测帧 cmd 编号 */
} svc_comm_config_t;

/**
 * @brief 通信统计（诊断用，拉模式查询）
 */
typedef struct {
    uint32_t rx_bytes;      /**< 累计接收字节数 */
    uint32_t rx_frames;     /**< 累计完整帧数（CRC 通过） */
    uint32_t rx_crc_err;    /**< 累计 CRC 错误帧数 */
    uint32_t rx_overflow;   /**< 累计环形缓冲溢出丢弃字节数 */
    uint32_t tx_frames;     /**< 累计发送帧数 */
    uint32_t tx_fail;       /**< 累计发送失败次数 */
} svc_comm_stats_t;

/* ================================================================
 *  公开 API
 * ================================================================ */

/**
 * @brief   初始化通信服务（不创建任务，不连接 WiFi）
 *
 * @param[in] cfg 服务配置，NULL 表示全部使用默认值
 * @return  SVC_OK 成功；SVC_ERR_DEV 设备不可用；SVC_ERR_BUSY 重复初始化
 *
 * @note    须在调度器启动前、bsp_esp8266_init()（board_v1.c）之后调用。
 */
svc_err_t svc_comm_init(const svc_comm_config_t *cfg);

/**
 * @brief   注册指令处理函数（覆盖式：同 cmd 重复注册替换旧值）
 *
 * @param[in] cmd   指令编号（0x00~0xFF）
 * @param[in] fn    处理函数（NULL = 注销）
 * @param[in] user  用户上下文（可 NULL）
 * @return  SVC_OK；SVC_ERR_PARAM；SVC_ERR_BUSY 表项已满
 *
 * @note    须在 svc_comm_start() 之前完成注册（运行中注册不加锁）。
 */
svc_err_t svc_comm_register_cmd(uint8_t cmd, svc_comm_cmd_fn_t fn,
                                void *user);

/**
 * @brief   注册遥测构造函数
 * @param[in] fn   遥测构造函数（NULL = 停止周期上报）
 * @param[in] user 用户上下文（可 NULL）
 * @return  SVC_OK；SVC_ERR_NOT_INIT
 * @note    须在 svc_comm_start() 之前注册。
 */
svc_err_t svc_comm_set_telemetry(svc_comm_telem_fn_t fn, void *user);

/**
 * @brief   启动通信任务（静态创建；ssid 非空时发起 STA 连接）
 * @return  SVC_OK；SVC_ERR_NOT_INIT / SVC_ERR_FAIL
 */
svc_err_t svc_comm_start(void);

/**
 * @brief   停止通信任务
 * @return  SVC_OK；SVC_ERR_NOT_INIT / SVC_ERR_STATE
 */
svc_err_t svc_comm_stop(void);

/**
 * @brief   发送一帧（帧编码 + transmit）
 *
 * @param[in] cmd      指令编号
 * @param[in] payload  载荷（可 NULL，len 为 0）
 * @param[in] len      载荷长度（≤ SVC_COMM_MAX_PAYLOAD）
 * @return  SVC_OK 成功；SVC_ERR_PARAM / SVC_ERR_FAIL 发送失败
 *
 * @note    可从任意任务上下文调用（transmit 由驱动加锁）；
 *          @warning 禁止在 ISR 上下文调用。
 */
svc_err_t svc_comm_send_frame(uint8_t cmd, const uint8_t *payload,
                              uint8_t len);

/**
 * @brief   查询通信统计
 * @param[out] stats 统计快照
 * @return  SVC_OK；SVC_ERR_PARAM / SVC_ERR_NOT_INIT
 */
svc_err_t svc_comm_get_stats(svc_comm_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_COMM_H__ */
