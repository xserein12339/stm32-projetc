/**
 * @file    svc_comm.c
 * @brief   通信服务实现 v1.0
 *
 * RX 数据流：dal_wifi rx 回调（ISR/驱动任务上下文）
 *   -> SPSC 无锁环形缓冲（仅写 head）
 *   -> svc_comm 任务 drain + 帧解析状态机
 *   -> CRC 校验 -> cmd 分发表查找 -> handler（本任务上下文）
 *
 * TX 数据流：遥测 tick（周期）或 send_frame（按需）
 *   -> 帧编码 -> dal_wifi_transmit
 *
 * @author  xserein
 * @version v1.0
 */
#include "svc_comm.h"
#include "dal_wifi.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ========================================================================== */
/*                          内部类型与静态状态                                   */
/* ========================================================================== */

/**
 * @brief SPSC 无锁环形缓冲
 * @note  生产者（rx 回调，ISR 或驱动任务）只写 head；
 *        消费者（svc_comm 任务）只写 tail。uint16_t 访问在
 *        Cortex-M3 上天然原子（单 STR/LDR）。满时丢弃新数据。
 */
typedef struct {
    uint8_t  buf[SVC_COMM_RING_SIZE];
    volatile uint16_t head;   /**< 生产者指针（仅写） */
    volatile uint16_t tail;   /**< 消费者指针（仅写） */
} spsc_ring_t;

/** 指令分发表项 */
typedef struct {
    uint8_t            cmd;
    svc_comm_cmd_fn_t  fn;
    void              *user;
    bool               used;
} cmd_slot_t;

/**
 * @brief 帧解析状态机
 */
typedef enum {
    PARSER_SYNC = 0,    /**< 等待帧头 0xA5 */
    PARSER_CMD,         /**< 读 cmd */
    PARSER_LEN,         /**< 读 len */
    PARSER_PAYLOAD,     /**< 读 payload */
    PARSER_CRC,         /**< 读 crc8 */
} parser_state_t;

/**
 * @brief 服务内部运行时状态
 */
typedef struct {
    /* --- 配置 --- */
    char     ssid[33];
    char     password[65];
    bool     use_wifi;
    uint32_t telem_period_ms;
    uint8_t  telem_cmd;

    /* --- 设备 --- */
    dal_wifi_dev_t *dev;

    /* --- RX 路径 --- */
    spsc_ring_t   ring;
    parser_state_t parser_state;
    uint8_t        frame_cmd;
    uint8_t        frame_len;
    uint8_t        frame_pos;
    uint8_t        frame_payload[SVC_COMM_MAX_PAYLOAD];
    uint8_t        frame_crc;

    /* --- 指令分发 --- */
    cmd_slot_t cmd_table[8];

    /* --- 遥测 --- */
    svc_comm_telem_fn_t telem_fn;
    void               *telem_user;

    /* --- 统计 --- */
    volatile svc_comm_stats_t stats;

    /* --- 任务 --- */
    TaskHandle_t   task_handle;
    StaticTask_t   task_tcb;
    StackType_t    task_stack[SVC_COMM_TASK_STACK_WORDS];
    bool           task_running;
} comm_ctx_t;

static comm_ctx_t s_ctx;

/* ========================================================================== */
/*                          SPSC 环形缓冲                                       */
/* ========================================================================== */

/**
 * @brief   环形缓冲写入（生产者侧，ISR 安全）
 * @return  true 写入成功；false 缓冲满，该字节被丢弃
 */
static bool ring_push(spsc_ring_t *r, uint8_t byte)
{
    uint16_t head = r->head;
    uint16_t next = (uint16_t)((head + 1U) & (SVC_COMM_RING_SIZE - 1U));

    if (next == r->tail) {          /* 满：写方只读 tail（单变量，安全） */
        return false;
    }

    r->buf[head] = byte;
    r->head = next;                 /* 单 16 位写，原子发布 */
    return true;
}

/**
 * @brief   环形缓冲读取（消费者侧，仅 svc_comm 任务调用）
 * @return  true 读出；false 空
 */
static bool ring_pop(spsc_ring_t *r, uint8_t *byte)
{
    uint16_t tail = r->tail;

    if (tail == r->head) {          /* 空 */
        return false;
    }

    *byte = r->buf[tail];
    r->tail = (uint16_t)((tail + 1U) & (SVC_COMM_RING_SIZE - 1U));
    return true;
}

/* ========================================================================== */
/*                          CRC8（多项式 0x07）                                  */
/* ========================================================================== */

/**
 * @brief   CRC-8 计算（poly 0x07，init 0x00，无反射，无异或输出）
 * @param[in] data 数据指针
 * @param[in] len  长度
 * @return  CRC 值
 */
static uint8_t crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00U;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            crc = (uint8_t)((crc & 0x80U) ? ((crc << 1) ^ 0x07U)
                                          : (uint8_t)(crc << 1));
        }
    }
    return crc;
}

/* ========================================================================== */
/*                          帧解析与分发                                         */
/* ========================================================================== */

/**
 * @brief   处理一帧完整数据（CRC 已校验通过）
 */
static void frame_dispatch(void)
{
    s_ctx.stats.rx_frames++;

    for (uint32_t i = 0; i < 8U; i++) {
        cmd_slot_t *slot = &s_ctx.cmd_table[i];
        if (slot->used && slot->cmd == s_ctx.frame_cmd) {
            slot->fn(s_ctx.frame_cmd, s_ctx.frame_payload,
                     s_ctx.frame_len, slot->user);
            return;
        }
    }
    /* 未注册的 cmd：静默丢弃（统计已计入 rx_frames） */
}

/**
 * @brief   喂给解析状态机一个字节
 */
static void parser_feed(uint8_t byte)
{
    s_ctx.stats.rx_bytes++;

    switch (s_ctx.parser_state) {
    case PARSER_SYNC:
        if (byte == 0xA5U) {
            s_ctx.parser_state = PARSER_CMD;
        }
        break;

    case PARSER_CMD:
        s_ctx.frame_cmd = byte;
        s_ctx.parser_state = PARSER_LEN;
        break;

    case PARSER_LEN:
        if (byte > SVC_COMM_MAX_PAYLOAD) {
            s_ctx.parser_state = PARSER_SYNC;   /* 超长帧，回同步 */
            break;
        }
        s_ctx.frame_len  = byte;
        s_ctx.frame_pos  = 0U;
        s_ctx.parser_state = (byte == 0U) ? PARSER_CRC : PARSER_PAYLOAD;
        break;

    case PARSER_PAYLOAD:
        s_ctx.frame_payload[s_ctx.frame_pos++] = byte;
        if (s_ctx.frame_pos >= s_ctx.frame_len) {
            s_ctx.parser_state = PARSER_CRC;
        }
        break;

    case PARSER_CRC:
        s_ctx.frame_crc = byte;
        s_ctx.parser_state = PARSER_SYNC;

        /* CRC 覆盖 cmd + len + payload */
        uint8_t crc_input[2 + SVC_COMM_MAX_PAYLOAD];
        crc_input[0] = s_ctx.frame_cmd;
        crc_input[1] = s_ctx.frame_len;
        (void)memcpy(&crc_input[2], s_ctx.frame_payload, s_ctx.frame_len);

        if (crc8(crc_input, (uint32_t)(2U + s_ctx.frame_len))
            == s_ctx.frame_crc) {
            frame_dispatch();
        } else {
            s_ctx.stats.rx_crc_err++;
        }
        break;

    default:
        s_ctx.parser_state = PARSER_SYNC;
        break;
    }
}

/* ========================================================================== */
/*                          RX / 遥测 / 任务                                     */
/* ========================================================================== */

/**
 * @brief   WiFi RAW_SERIAL 数据回调（ISR 或 esp 驱动任务上下文）
 * @warning 遵守 dal_wifi ISR 安全契约：只做 ring_push（无阻塞无分配）
 */
static void wifi_rx_cb(dal_wifi_dev_t *dev, const uint8_t *data,
                       uint32_t len, void *user_data)
{
    (void)dev;
    (void)user_data;

    for (uint32_t i = 0; i < len; i++) {
        if (!ring_push(&s_ctx.ring, data[i])) {
            s_ctx.stats.rx_overflow++;
        }
    }
}

/**
 * @brief   通信任务主函数
 */
static void comm_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t telem_period =
        pdMS_TO_TICKS(s_ctx.telem_period_ms ? s_ctx.telem_period_ms : 1U);
    uint8_t byte;
    uint8_t telem_buf[SVC_COMM_MAX_PAYLOAD];

    for (;;) {
        vTaskDelayUntil(&wake, telem_period);

        /* --- 1. drain RX 环形缓冲并解析 --- */
        while (ring_pop(&s_ctx.ring, &byte)) {
            parser_feed(byte);
        }

        /* --- 2. 周期遥测上报 --- */
        if (s_ctx.telem_fn != NULL) {
            uint8_t len = s_ctx.telem_fn(telem_buf, s_ctx.telem_user);
            if (len > 0U) {
                (void)svc_comm_send_frame(s_ctx.telem_cmd, telem_buf, len);
            }
        }
    }
}

/* ========================================================================== */
/*                              公开 API                                        */
/* ========================================================================== */

svc_err_t svc_comm_init(const svc_comm_config_t *cfg)
{
    if (s_ctx.task_running) {
        return SVC_ERR_BUSY;
    }

    /* --- 配置定格 --- */
    const char *dev_name = SVC_COMM_DEFAULT_DEV_NAME;
    if (cfg != NULL && cfg->dev_name != NULL) {
        dev_name = cfg->dev_name;
    }
    s_ctx.use_wifi = (cfg != NULL) && (cfg->ssid != NULL);
    if (s_ctx.use_wifi) {
        (void)strncpy(s_ctx.ssid, cfg->ssid, sizeof(s_ctx.ssid) - 1U);
        s_ctx.ssid[sizeof(s_ctx.ssid) - 1U] = '\0';
        if (cfg->password != NULL) {
            (void)strncpy(s_ctx.password, cfg->password,
                          sizeof(s_ctx.password) - 1U);
            s_ctx.password[sizeof(s_ctx.password) - 1U] = '\0';
        } else {
            s_ctx.password[0] = '\0';
        }
    }
    s_ctx.telem_period_ms = (cfg != NULL) ? cfg->telemetry_period_ms
                                          : SVC_COMM_DEFAULT_TELEM_PERIOD_MS;
    s_ctx.telem_cmd = (cfg != NULL) ? cfg->telemetry_cmd
                                    : SVC_COMM_DEFAULT_TELEM_CMD;

    /* --- 获取设备 --- */
    s_ctx.dev = dal_wifi_get_dev(dev_name);
    if (s_ctx.dev == NULL) {
        return SVC_ERR_DEV;
    }

    /* --- 注册 RX 回调（写入 ring，ISR 安全） --- */
    if (dal_wifi_register_rx_callback(s_ctx.dev, wifi_rx_cb, NULL) != DAL_OK) {
        return SVC_ERR_DEV;
    }

    /* --- 状态复位 --- */
    s_ctx.ring.head = 0;
    s_ctx.ring.tail = 0;
    s_ctx.parser_state = PARSER_SYNC;
    for (uint32_t i = 0; i < 8U; i++) {
        s_ctx.cmd_table[i].used = false;
    }
    s_ctx.telem_fn  = NULL;
    s_ctx.telem_user = NULL;
    memset((void *)&s_ctx.stats, 0, sizeof(s_ctx.stats));

    return SVC_OK;
}

svc_err_t svc_comm_register_cmd(uint8_t cmd, svc_comm_cmd_fn_t fn,
                                void *user)
{
    cmd_slot_t *free_slot = NULL;

    for (uint32_t i = 0; i < 8U; i++) {
        cmd_slot_t *slot = &s_ctx.cmd_table[i];
        if (slot->used && slot->cmd == cmd) {
            if (fn == NULL) {
                slot->used = false;     /* 注销 */
                return SVC_OK;
            }
            slot->fn   = fn;            /* 覆盖替换 */
            slot->user = user;
            return SVC_OK;
        }
        if (!slot->used && free_slot == NULL) {
            free_slot = slot;
        }
    }

    if (fn == NULL) {
        return SVC_OK;                  /* 注销不存在的项，幂等 */
    }
    if (free_slot == NULL) {
        return SVC_ERR_BUSY;            /* 表满 */
    }

    free_slot->cmd  = cmd;
    free_slot->fn   = fn;
    free_slot->user = user;
    free_slot->used = true;
    return SVC_OK;
}

svc_err_t svc_comm_set_telemetry(svc_comm_telem_fn_t fn, void *user)
{
    s_ctx.telem_fn  = fn;
    s_ctx.telem_user = user;
    return SVC_OK;
}

svc_err_t svc_comm_start(void)
{
    if (s_ctx.dev == NULL) {
        return SVC_ERR_NOT_INIT;
    }
    if (s_ctx.task_running) {
        return SVC_ERR_BUSY;
    }

    /* WiFi STA 连接（配置了 ssid 才发起；失败不阻塞任务启动，
     * 断线重连由 bsp_esp8266 驱动任务负责） */
    if (s_ctx.use_wifi) {
        dal_wifi_sta_config_t sta = {0};
        (void)strncpy(sta.ssid, s_ctx.ssid, sizeof(sta.ssid) - 1U);
        (void)strncpy(sta.password, s_ctx.password, sizeof(sta.password) - 1U);
        sta.auth_mode = DAL_WIFI_AUTH_AUTO;
        (void)dal_wifi_sta_connect(s_ctx.dev, &sta);
    }

    s_ctx.task_handle = xTaskCreateStatic(comm_task, "comm",
                                          SVC_COMM_TASK_STACK_WORDS, NULL,
                                          SVC_COMM_TASK_PRIORITY,
                                          s_ctx.task_stack, &s_ctx.task_tcb);
    if (s_ctx.task_handle == NULL) {
        return SVC_ERR_FAIL;
    }

    s_ctx.task_running = true;
    return SVC_OK;
}

svc_err_t svc_comm_stop(void)
{
    if (!s_ctx.task_running) {
        return SVC_ERR_STATE;
    }

    vTaskDelete(s_ctx.task_handle);
    s_ctx.task_handle = NULL;
    s_ctx.task_running = false;
    return SVC_OK;
}

svc_err_t svc_comm_send_frame(uint8_t cmd, const uint8_t *payload,
                              uint8_t len)
{
    if (len > SVC_COMM_MAX_PAYLOAD) {
        return SVC_ERR_PARAM;
    }
    if (len > 0U && payload == NULL) {
        return SVC_ERR_PARAM;
    }
    if (s_ctx.dev == NULL) {
        return SVC_ERR_NOT_INIT;
    }

    /* 帧编码：头 + cmd + len + payload + crc8(cmd,len,payload) */
    uint8_t frame[3U + SVC_COMM_MAX_PAYLOAD + 1U];
    frame[0] = 0xA5U;
    frame[1] = cmd;
    frame[2] = len;
    if (len > 0U) {
        (void)memcpy(&frame[3], payload, len);
    }
    frame[3U + len] = crc8(&frame[1], (uint32_t)(2U + len));

    if (dal_wifi_transmit(s_ctx.dev, frame, (uint32_t)(4U + len), 100U)
        != DAL_OK) {
        s_ctx.stats.tx_fail++;
        return SVC_ERR_FAIL;
    }

    s_ctx.stats.tx_frames++;
    return SVC_OK;
}

svc_err_t svc_comm_get_stats(svc_comm_stats_t *stats)
{
    if (stats == NULL) {
        return SVC_ERR_PARAM;
    }

    /* volatile 逐字段拷贝（统计量允许撕裂读，仅诊断用途） */
    stats->rx_bytes    = s_ctx.stats.rx_bytes;
    stats->rx_frames   = s_ctx.stats.rx_frames;
    stats->rx_crc_err  = s_ctx.stats.rx_crc_err;
    stats->rx_overflow = s_ctx.stats.rx_overflow;
    stats->tx_frames   = s_ctx.stats.tx_frames;
    stats->tx_fail     = s_ctx.stats.tx_fail;
    return SVC_OK;
}
