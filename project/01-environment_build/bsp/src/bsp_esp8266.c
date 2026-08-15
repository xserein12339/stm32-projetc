/**
 * @file    bsp_esp8266.c
 * @brief   板级 ESP8266 BSP 层实现（AT 指令）v1.4
 * @note    - [v1.4] RAM 深度优化：RX buf 2048→512，scan results 20→8
 *          - [v1.4] _handle_ipd 移除 512B 栈缓冲，统一堆分配，消除栈溢出
 *          - [v1.4] sta_connect/ap_start 缩减栈上 cmd 缓冲区
 *          - [v1.4] bsp_esp8266_init 仅注册，不执行硬件初始化
 *          - [v1.3] 修复扫描结果丢失：_at_send_cmd_wait 持锁期间同步解析 +CWLAP
 *          - [v1.3] _parse_cwlap 补全 bssid/channel 字段解析
 *          - [v1.3] 修正 auth_mode 枚举映射
 *          - [v1.3] get_scan_result 支持 scan_sem 阻塞等待
 *          - [v1.3] sta_connect/ap_start 增加 SSID/密码引号转义保护
 *          - [v1.2] +IPD 兼容多连接模式
 *          - [v1.1] AT 命令引擎与异步事件解析器分离
 * @author  xserein
 * @version v1.4
 */

#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_esp8266.h"
#include "dal_wifi.h"
#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ========================================================================== */
/*                             硬件配置                                        */
/* ========================================================================== */

#define ESP8266_UART_ID             (3U)
#define ESP8266_BAUDRATE            (115200U)
#define ESP8266_CMD_TIMEOUT_MS      (3000U)
#define ESP8266_SEND_TIMEOUT_MS     (1000U)
#define ESP8266_SCAN_TIMEOUT_MS     (10000U)

/* [v1.4] RAM 优化：115200 波特率下 512B 足够应对突发（~44ms 缓冲） */
#define ESP8266_RX_BUF_SIZE         (512U)

/* [v1.4] RAM 优化：扫描结果从 20 缩减至 8（节省 ~768B 静态 RAM） */
#define ESP8266_MAX_SCAN_RESULTS    (8U)

#define ESP8266_AT_LINE_MAX         (256U)

/* [v1.4] IPD 最大单包接收限制，防止超大包导致堆分配失败 */
#define ESP8266_IPD_MAX_PAYLOAD     (1460U)

/* ========================================================================== */
/*                         AT 指令字符串定义                                    */
/* ========================================================================== */

#define AT_CMD_ATE0             "ATE0\r\n"
#define AT_CMD_CWMODE           "AT+CWMODE=%d\r\n"
#define AT_CMD_CWJAP            "AT+CWJAP=\"%s\",\"%s\"\r\n"
#define AT_CMD_CWQAP            "AT+CWQAP\r\n"
#define AT_CMD_CWLAP            "AT+CWLAP\r\n"
#define AT_CMD_CWSAP            "AT+CWSAP=\"%s\",\"%s\",%d,%d\r\n"
#define AT_CMD_CWSAP_DEL        "AT+CWSAP_DEL\r\n"
#define AT_CMD_CIPMUX           "AT+CIPMUX=%d\r\n"
#define AT_CMD_CIPSEND          "AT+CIPSEND=%lu\r\n"
#define AT_CMD_CIPCLOSE         "AT+CIPCLOSE\r\n"
#define AT_CMD_RST              "AT+RST\r\n"
#define AT_CMD_RF               "AT+RF=%d\r\n"

/* ========================================================================== */
/*                         内部状态定义                                        */
/* ========================================================================== */

typedef enum {
    ESP_STATE_IDLE,
    ESP_STATE_CONNECTING,
    ESP_STATE_CONNECTED,
    ESP_STATE_GOT_IP,
    ESP_STATE_DISCONNECTING,
    ESP_STATE_AP_ACTIVE,
    ESP_STATE_SCANNING,
    ESP_STATE_FAULT,
} esp_state_t;

typedef struct {
    SemaphoreHandle_t   sem;
    char                expect[32];
    volatile bool       matched;
    volatile bool       error;
    volatile bool       scan_parsing;
    char                line_buf[ESP8266_AT_LINE_MAX];
    uint16_t            line_len;
} at_cmd_ctx_t;

typedef struct {
    bsp_uart_handle_t   uart_handle;
    esp_state_t         state;
    TaskHandle_t        task_handle;

    at_cmd_ctx_t        cmd_ctx;
    SemaphoreHandle_t   evt_sem;

    uint8_t             rx_buf[ESP8266_RX_BUF_SIZE];
    volatile uint16_t   rx_head;
    volatile uint16_t   rx_tail;
    volatile bool       rx_overflow;

    SemaphoreHandle_t   at_mutex;

    dal_wifi_rx_callback_t rx_cb;
    void                *rx_cb_arg;

    char                sta_ssid[33];
    char                sta_password[65];
    dal_wifi_ip_t       sta_ip;
    bool                sta_ip_valid;

    SemaphoreHandle_t   conn_sem;
    volatile bool       conn_success;

    uint16_t            scan_count;
    dal_wifi_scan_record_t scan_records[ESP8266_MAX_SCAN_RESULTS];
    SemaphoreHandle_t   scan_sem;
    volatile bool       scan_done;

    bool                initialized;
} esp8266_priv_t;

static esp8266_priv_t s_esp;

/* ========================================================================== */
/*                    Ringbuf 操作（SPSC 模型）                                 */
/* ========================================================================== */

static inline uint16_t _rx_available(void)
{
    return (s_esp.rx_head - s_esp.rx_tail) % ESP8266_RX_BUF_SIZE;
}

static inline void _rx_putc(uint8_t c)
{
    uint16_t next = (s_esp.rx_head + 1) % ESP8266_RX_BUF_SIZE;
    if (next == s_esp.rx_tail) {
        s_esp.rx_overflow = true;
        return;
    }
    s_esp.rx_buf[s_esp.rx_head] = c;
    s_esp.rx_head = next;
}

static inline int _rx_getc(void)
{
    if (s_esp.rx_tail == s_esp.rx_head) return -1;
    uint8_t c = s_esp.rx_buf[s_esp.rx_tail];
    s_esp.rx_tail = (s_esp.rx_tail + 1) % ESP8266_RX_BUF_SIZE;
    return c;
}

static uint16_t _rx_read(uint8_t *buf, uint16_t len)
{
    uint16_t avail = _rx_available();
    uint16_t to_read = (len < avail) ? len : avail;
    for (uint16_t i = 0; i < to_read; i++) {
        buf[i] = (uint8_t)_rx_getc();
    }
    return to_read;
}

/* ========================================================================== */
/*                    UART 回调                                                */
/* ========================================================================== */

static void _uart_rx_callback(bsp_uart_handle_t handle,
                              const bsp_uart_evt_info_t *info,
                              void *user_data)
{
    (void)handle;
    (void)user_data;

    if (info->event == BSP_UART_EVT_RX_COMPLETE ||
        info->event == BSP_UART_EVT_RX_TIMEOUT) {
        const uint8_t *data = info->buf;
        for (uint32_t i = 0; i < info->bytes_done; i++) {
            _rx_putc(data[i]);
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_esp.evt_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ========================================================================== */
/*          +CWLAP 解析                                                        */
/* ========================================================================== */

static void _parse_mac(const char *str, uint8_t mac[6])
{
    unsigned int m[6];
    if (sscanf(str, "%x:%x:%x:%x:%x:%x",
               &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
        for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
    }
}

static void _parse_cwlap(const char *line)
{
    if (s_esp.scan_count >= ESP8266_MAX_SCAN_RESULTS) return;

    const char *p = strstr(line, "+CWLAP:(");
    if (!p) return;
    p += 8;

    dal_wifi_scan_record_t *rec = &s_esp.scan_records[s_esp.scan_count];
    memset(rec, 0, sizeof(*rec));

    int ecn = 0;
    if (sscanf(p, "(%d,", &ecn) == 1) {
        switch (ecn) {
            case 0: rec->auth_mode = DAL_WIFI_AUTH_OPEN; break;
            case 3: rec->auth_mode = DAL_WIFI_AUTH_WPA2_PSK; break;
            case 4: rec->auth_mode = DAL_WIFI_AUTH_WPA2_WPA3; break;
            default: rec->auth_mode = DAL_WIFI_AUTH_AUTO; break;
        }
    }

    const char *ssid_start = strchr(p, '"');
    if (ssid_start) {
        ssid_start++;
        const char *ssid_end = strchr(ssid_start, '"');
        if (ssid_end) {
            uint16_t ssid_len = (uint16_t)(ssid_end - ssid_start);
            if (ssid_len >= sizeof(rec->ssid)) ssid_len = sizeof(rec->ssid) - 1;
            memcpy(rec->ssid, ssid_start, ssid_len);
            rec->ssid[ssid_len] = '\0';
        }
    }

    const char *after_ssid = strstr(line, "\",");
    if (after_ssid) {
        after_ssid += 2;
        int rssi = 0;
        if (sscanf(after_ssid, "%d", &rssi) == 1) {
            rec->rssi = (int8_t)rssi;
        }

        const char *mac_start = strchr(after_ssid, '"');
        if (mac_start) {
            mac_start++;
            _parse_mac(mac_start, rec->bssid.addr);
        }

        const char *mac_end = strchr(after_ssid, ',');
        if (mac_end) {
            const char *ch_str = strchr(mac_end + 1, ',');
            if (ch_str) {
                ch_str++;
                int ch = 0;
                if (sscanf(ch_str, "%d", &ch) == 1) {
                    rec->channel = (uint8_t)ch;
                }
            }
        }
    }

    s_esp.scan_count++;
}

/* ========================================================================== */
/*               AT 命令同步引擎                                               */
/* ========================================================================== */

static bool _at_send_cmd_wait(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    if (xSemaphoreTake(s_esp.at_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return false;
    }

    s_esp.cmd_ctx.matched = false;
    s_esp.cmd_ctx.error   = false;
    s_esp.cmd_ctx.line_len = 0;

    if (expect) {
        strncpy(s_esp.cmd_ctx.expect, expect, sizeof(s_esp.cmd_ctx.expect) - 1);
        s_esp.cmd_ctx.expect[sizeof(s_esp.cmd_ctx.expect) - 1] = '\0';
    } else {
        s_esp.cmd_ctx.expect[0] = '\0';
    }

    if (bsp_uart_send(s_esp.uart_handle, (const uint8_t *)cmd,
                      (uint16_t)strlen(cmd), 100) != BSP_OK) {
        xSemaphoreGive(s_esp.at_mutex);
        return false;
    }

    if (expect == NULL) {
        xSemaphoreGive(s_esp.at_mutex);
        return true;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        int c = _rx_getc();
        if (c >= 0) {
            if (c == '\n') {
                uint16_t len = s_esp.cmd_ctx.line_len;
                if (len > 0 && s_esp.cmd_ctx.line_buf[len - 1] == '\r') len--;
                s_esp.cmd_ctx.line_buf[len] = '\0';

                if (s_esp.cmd_ctx.scan_parsing &&
                    strstr(s_esp.cmd_ctx.line_buf, "+CWLAP:") != NULL) {
                    _parse_cwlap(s_esp.cmd_ctx.line_buf);
                }

                if (s_esp.cmd_ctx.expect[0] != '\0' &&
                    strstr(s_esp.cmd_ctx.line_buf, s_esp.cmd_ctx.expect) != NULL) {
                    s_esp.cmd_ctx.matched = true;
                }
                if (strstr(s_esp.cmd_ctx.line_buf, "ERROR") != NULL) {
                    s_esp.cmd_ctx.error = true;
                }
                s_esp.cmd_ctx.line_len = 0;

                if (s_esp.cmd_ctx.matched || s_esp.cmd_ctx.error) break;
            } else if (s_esp.cmd_ctx.line_len < ESP8266_AT_LINE_MAX - 1) {
                s_esp.cmd_ctx.line_buf[s_esp.cmd_ctx.line_len++] = (char)c;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    s_esp.cmd_ctx.scan_parsing = false;

    bool result = s_esp.cmd_ctx.matched;
    xSemaphoreGive(s_esp.at_mutex);
    return result;
}

/* ========================================================================== */
/*          [v1.4] +IPD 解析（移除 512B 栈缓冲，统一堆分配）                    */
/* ========================================================================== */

static void _handle_ipd(void)
{
    char hdr[48];
    uint16_t hdr_len = 0;

    /* 解析 +IPD 头部直到 ':' */
    while (hdr_len < sizeof(hdr) - 1) {
        int c = _rx_getc();
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            c = _rx_getc();
            if (c < 0) return;
        }
        if (c == ':') break;
        hdr[hdr_len++] = (char)c;
    }
    hdr[hdr_len] = '\0';

    /* 解析 payload 长度 */
    uint16_t payload_len = 0;
    char *last_comma = strrchr(hdr, ',');
    if (last_comma != NULL) {
        if (sscanf(last_comma + 1, "%hu", &payload_len) != 1) return;
    } else {
        if (sscanf(hdr, "%hu", &payload_len) != 1) return;
    }

    if (payload_len == 0) return;

    /* [v1.4] 安全限制：防止畸形 AT 响应导致巨量堆分配 */
    if (payload_len > ESP8266_IPD_MAX_PAYLOAD) {
        payload_len = ESP8266_IPD_MAX_PAYLOAD;
    }

    /* 等待数据到达 ring buffer */
    TickType_t start = xTaskGetTickCount();
    while (_rx_available() < payload_len &&
           (xTaskGetTickCount() - start) < pdMS_TO_TICKS(1000)) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (_rx_available() < payload_len) {
        /* 数据不足，丢弃已收到的部分以清空 ring buffer */
        uint16_t avail = _rx_available();
        uint8_t discard[64];
        while (avail > 0) {
            uint16_t chunk = (avail < sizeof(discard)) ? avail : sizeof(discard);
            _rx_read(discard, chunk);
            avail -= chunk;
        }
        return;
    }

    /* [v1.4] 统一堆分配，彻底消除 _esp_task 栈上的 512B 缓冲 */
    uint8_t *tmp = (uint8_t *)pvPortMalloc(payload_len);
    if (!tmp) {
        /* 堆分配失败，丢弃数据 */
        uint8_t discard[64];
        uint16_t remaining = payload_len;
        while (remaining > 0) {
            uint16_t chunk = (remaining < sizeof(discard)) ? remaining : sizeof(discard);
            _rx_read(discard, chunk);
            remaining -= chunk;
        }
        return;
    }

    _rx_read(tmp, payload_len);

    if (s_esp.rx_cb) {
        s_esp.rx_cb((dal_wifi_dev_t *)&s_esp, tmp, payload_len, s_esp.rx_cb_arg);
    }

    vPortFree(tmp);
}

/* ========================================================================== */
/*               异步事件解析器                                                 */
/* ========================================================================== */

static void _process_event_line(const char *line)
{
    if (strstr(line, "WIFI CONNECTED")) {
        s_esp.state = ESP_STATE_CONNECTED;
        dal_wifi_notify_event((dal_wifi_dev_t *)&s_esp, DAL_WIFI_EVT_STA_CONNECTED, NULL);
    } else if (strstr(line, "WIFI GOT IP")) {
        s_esp.state = ESP_STATE_GOT_IP;
        const char *ip = strstr(line, "IP:");
        if (ip) {
            ip += 3;
            unsigned int a, b, c, d;
            if (sscanf(ip, "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
                s_esp.sta_ip.addr = (a << 24) | (b << 16) | (c << 8) | d;
                s_esp.sta_ip_valid = true;
            }
        }
        dal_wifi_notify_event((dal_wifi_dev_t *)&s_esp, DAL_WIFI_EVT_STA_GOT_IP, NULL);
        s_esp.conn_success = true;
        if (s_esp.conn_sem) xSemaphoreGive(s_esp.conn_sem);
    } else if (strstr(line, "WIFI DISCONNECT")) {
        s_esp.state = ESP_STATE_IDLE;
        s_esp.sta_ip_valid = false;
        dal_wifi_event_data_t evt = {.sta_disconn.reason = DAL_WIFI_DISCONN_REASON_UNSPECIFIED};
        dal_wifi_notify_event((dal_wifi_dev_t *)&s_esp, DAL_WIFI_EVT_STA_DISCONNECTED, &evt);
        s_esp.conn_success = false;
        if (s_esp.conn_sem) xSemaphoreGive(s_esp.conn_sem);
    } else if (strstr(line, "AP MODE START")) {
        s_esp.state = ESP_STATE_AP_ACTIVE;
        dal_wifi_notify_event((dal_wifi_dev_t *)&s_esp, DAL_WIFI_EVT_AP_STARTED, NULL);
    } else if (strstr(line, "AP MODE STOP")) {
        s_esp.state = ESP_STATE_IDLE;
        dal_wifi_notify_event((dal_wifi_dev_t *)&s_esp, DAL_WIFI_EVT_AP_STOPPED, NULL);
    }
}

/* ========================================================================== */
/*                    ESP8266 后台处理任务                                      */
/* ========================================================================== */

static void _esp_task(void *arg)
{
    (void)arg;
    char line[ESP8266_AT_LINE_MAX];
    uint16_t line_len = 0;

    while (1) {
        if (xSemaphoreTake(s_esp.evt_sem, portMAX_DELAY) != pdTRUE) continue;
        if (xSemaphoreTake(s_esp.at_mutex, 0) != pdTRUE) continue;

        while (1) {
            int c = _rx_getc();
            if (c < 0) break;

            if (line_len == 4 && strncmp(line, "+IPD", 4) == 0 && c == ',') {
                line_len = 0;
                _handle_ipd();
                continue;
            }

            if (c == '\n') {
                if (line_len > 0 && line[line_len - 1] == '\r') line_len--;
                line[line_len] = '\0';
                if (line_len > 0) {
                    _process_event_line(line);
                }
                line_len = 0;
            } else if (line_len < ESP8266_AT_LINE_MAX - 1) {
                line[line_len++] = (char)c;
            }
        }

        xSemaphoreGive(s_esp.at_mutex);
    }
}

/* ========================================================================== */
/*               SSID/密码引号转义辅助                                          */
/* ========================================================================== */

static void _escape_quotes(char *dst, const char *src, size_t dst_size)
{
    size_t di = 0;
    for (size_t si = 0; src[si] != '\0' && di < dst_size - 2; si++) {
        if (src[si] == '"') {
            dst[di++] = '\\';
        }
        dst[di++] = src[si];
    }
    dst[di] = '\0';
}

/* ========================================================================== */
/*                         DAL Wi-Fi ops 实现                                  */
/* ========================================================================== */

static dal_err_t bsp_esp_ops_init(dal_wifi_dev_t *dev)
{
    (void)dev;

    bsp_uart_config_t uart_cfg = {
        .baudrate      = ESP8266_BAUDRATE,
        .data_bits     = BSP_UART_DATA_BITS_8,
        .parity        = BSP_UART_PARITY_NONE,
        .stop_bits     = BSP_UART_STOP_BITS_1,
        .rx_timeout_ms = 10,
    };
    if (bsp_uart_open(ESP8266_UART_ID, &uart_cfg, &s_esp.uart_handle) != BSP_OK) {
        return DAL_ERR_DEPENDENCY;
    }

    bsp_uart_set_callback(s_esp.uart_handle, _uart_rx_callback, NULL);

    s_esp.evt_sem  = xSemaphoreCreateBinary();
    s_esp.at_mutex = xSemaphoreCreateMutex();
    s_esp.conn_sem = xSemaphoreCreateBinary();
    s_esp.scan_sem = xSemaphoreCreateBinary();
    if (!s_esp.evt_sem || !s_esp.at_mutex || !s_esp.conn_sem || !s_esp.scan_sem) {
        bsp_uart_close(s_esp.uart_handle);
        return DAL_ERR_NO_MEM;
    }

    /* [v1.4] 任务栈从 1024 降至 512 word = 2048B（堆分配后栈压力大幅降低） */
    if (xTaskCreate(_esp_task, "esp8266", 512, NULL, 3, &s_esp.task_handle) != pdPASS) {
        vSemaphoreDelete(s_esp.evt_sem);
        vSemaphoreDelete(s_esp.at_mutex);
        vSemaphoreDelete(s_esp.conn_sem);
        vSemaphoreDelete(s_esp.scan_sem);
        bsp_uart_close(s_esp.uart_handle);
        return DAL_ERR_NO_MEM;
    }

    static uint8_t s_dma_rx_buf[ESP8266_RX_BUF_SIZE];
    bsp_uart_recv_dma(s_esp.uart_handle, s_dma_rx_buf, ESP8266_RX_BUF_SIZE);

    if (!_at_send_cmd_wait(AT_CMD_ATE0, "OK", 1000)) return DAL_ERR_FAIL;

    char cmd[32];
    snprintf(cmd, sizeof(cmd), AT_CMD_CWMODE, 3);
    if (!_at_send_cmd_wait(cmd, "OK", 1000)) return DAL_ERR_FAIL;

    snprintf(cmd, sizeof(cmd), AT_CMD_CIPMUX, 0);
    if (!_at_send_cmd_wait(cmd, "OK", 1000)) return DAL_ERR_FAIL;

    s_esp.initialized = true;
    s_esp.state = ESP_STATE_IDLE;
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_deinit(dal_wifi_dev_t *dev)
{
    (void)dev;
    if (s_esp.task_handle) { vTaskDelete(s_esp.task_handle); s_esp.task_handle = NULL; }
    if (s_esp.evt_sem)     { vSemaphoreDelete(s_esp.evt_sem); s_esp.evt_sem = NULL; }
    if (s_esp.at_mutex)    { vSemaphoreDelete(s_esp.at_mutex); s_esp.at_mutex = NULL; }
    if (s_esp.conn_sem)    { vSemaphoreDelete(s_esp.conn_sem); s_esp.conn_sem = NULL; }
    if (s_esp.scan_sem)    { vSemaphoreDelete(s_esp.scan_sem); s_esp.scan_sem = NULL; }
    if (s_esp.uart_handle) { bsp_uart_close(s_esp.uart_handle); s_esp.uart_handle = NULL; }
    s_esp.initialized = false;
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_set_mode(dal_wifi_dev_t *dev, dal_wifi_mode_t mode)
{
    (void)dev;
    uint8_t m;
    switch (mode) {
        case DAL_WIFI_MODE_STA:    m = 1; break;
        case DAL_WIFI_MODE_AP:     m = 2; break;
        case DAL_WIFI_MODE_STA_AP: m = 3; break;
        case DAL_WIFI_MODE_NULL:   m = 0; break;
        default: return DAL_ERR_PARAM_INVALID;
    }
    char cmd[32];
    snprintf(cmd, sizeof(cmd), AT_CMD_CWMODE, m);
    return _at_send_cmd_wait(cmd, "OK", 1000) ? DAL_OK : DAL_ERR_FAIL;
}

/* ========================================================================== */
/*               扫描功能                                                      */
/* ========================================================================== */

static dal_err_t bsp_esp_ops_scan_start(dal_wifi_dev_t *dev)
{
    (void)dev;
    if (!s_esp.initialized) return DAL_ERR_NOT_READY;

    s_esp.scan_count = 0;
    s_esp.scan_done  = false;
    memset(s_esp.scan_records, 0, sizeof(s_esp.scan_records));

    s_esp.state = ESP_STATE_SCANNING;
    s_esp.cmd_ctx.scan_parsing = true;

    if (!_at_send_cmd_wait(AT_CMD_CWLAP, "OK", ESP8266_SCAN_TIMEOUT_MS)) {
        s_esp.state = ESP_STATE_IDLE;
        s_esp.cmd_ctx.scan_parsing = false;
        return DAL_ERR_FAIL;
    }

    s_esp.state = ESP_STATE_IDLE;
    s_esp.scan_done = true;

    xSemaphoreGive(s_esp.scan_sem);
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_get_scan_result(dal_wifi_dev_t *dev,
                                             dal_wifi_scan_record_t *records,
                                             uint16_t max_count, uint16_t *actual)
{
    (void)dev;
    if (records == NULL || actual == NULL) return DAL_ERR_PARAM_INVALID;

    if (!s_esp.scan_done) {
        if (xSemaphoreTake(s_esp.scan_sem,
                           pdMS_TO_TICKS(ESP8266_SCAN_TIMEOUT_MS)) != pdTRUE) {
            *actual = 0;
            return DAL_ERR_TIMEOUT;
        }
    }

    uint16_t cnt = (s_esp.scan_count < max_count) ? s_esp.scan_count : max_count;
    memcpy(records, s_esp.scan_records, cnt * sizeof(dal_wifi_scan_record_t));
    *actual = cnt;
    return DAL_OK;
}

/* ========================================================================== */
/*         [v1.4] STA / AP（缩减栈缓冲区）                                    */
/* ========================================================================== */

static dal_err_t bsp_esp_ops_sta_connect(dal_wifi_dev_t *dev,
                                         const dal_wifi_sta_config_t *config)
{
    (void)dev;

    /* [v1.4] 缩减栈缓冲：SSID 48B + PASS 80B + cmd 160B = 288B（原 448B） */
    char safe_ssid[48], safe_pass[80];
    _escape_quotes(safe_ssid, config->ssid, sizeof(safe_ssid));
    _escape_quotes(safe_pass, config->password, sizeof(safe_pass));

    char cmd[160];
    snprintf(cmd, sizeof(cmd), AT_CMD_CWJAP, safe_ssid, safe_pass);

    if (!_at_send_cmd_wait(cmd, "OK", 5000)) return DAL_ERR_FAIL;

    s_esp.conn_success = false;
    if (xSemaphoreTake(s_esp.conn_sem, pdMS_TO_TICKS(15000)) != pdTRUE) {
        return DAL_ERR_TIMEOUT;
    }
    return s_esp.conn_success ? DAL_OK : DAL_ERR_FAIL;
}

static dal_err_t bsp_esp_ops_sta_disconnect(dal_wifi_dev_t *dev)
{
    (void)dev;
    if (!_at_send_cmd_wait(AT_CMD_CWQAP, "OK", 1000)) return DAL_ERR_FAIL;
    s_esp.state = ESP_STATE_IDLE;
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_sta_get_info(dal_wifi_dev_t *dev, dal_wifi_sta_info_t *info)
{
    (void)dev;
    if (s_esp.state == ESP_STATE_GOT_IP && s_esp.sta_ip_valid) {
        strcpy(info->ssid, s_esp.sta_ssid);
        info->ip = s_esp.sta_ip;
        return DAL_OK;
    }
    return DAL_ERR_NOT_READY;
}

static dal_err_t bsp_esp_ops_ap_start(dal_wifi_dev_t *dev,
                                      const dal_wifi_ap_config_t *config)
{
    (void)dev;

    /* [v1.4] 缩减栈缓冲，与 sta_connect 一致 */
    char safe_ssid[48], safe_pass[80];
    _escape_quotes(safe_ssid, config->ssid, sizeof(safe_ssid));
    _escape_quotes(safe_pass, config->password, sizeof(safe_pass));

    char cmd[160];
    snprintf(cmd, sizeof(cmd), AT_CMD_CWSAP,
             safe_ssid, safe_pass, config->channel, 0);
    if (!_at_send_cmd_wait(cmd, "OK", 2000)) return DAL_ERR_FAIL;
    s_esp.state = ESP_STATE_AP_ACTIVE;
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_ap_stop(dal_wifi_dev_t *dev)
{
    (void)dev;
    if (!_at_send_cmd_wait(AT_CMD_CWSAP_DEL, "OK", 1000)) return DAL_ERR_FAIL;
    s_esp.state = ESP_STATE_IDLE;
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_transmit(dal_wifi_dev_t *dev,
                                      const uint8_t *data, uint32_t len,
                                      uint32_t timeout_ms)
{
    (void)dev;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), AT_CMD_CIPSEND, len);

    if (!_at_send_cmd_wait(cmd, ">", timeout_ms)) return DAL_ERR_TIMEOUT;

    if (bsp_uart_send(s_esp.uart_handle, data, (uint16_t)len, 100) != BSP_OK) {
        return DAL_ERR_FAIL;
    }

    if (!_at_send_cmd_wait("", "SEND OK", ESP8266_SEND_TIMEOUT_MS)) {
        return DAL_ERR_FAIL;
    }
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_set_rf_power(dal_wifi_dev_t *dev, bool on)
{
    (void)dev;
    char cmd[16];
    snprintf(cmd, sizeof(cmd), AT_CMD_RF, on ? 1 : 0);
    return _at_send_cmd_wait(cmd, "OK", 1000) ? DAL_OK : DAL_ERR_FAIL;
}

static void bsp_esp_ops_register_rx_callback(dal_wifi_dev_t *dev,
                                             dal_wifi_rx_callback_t cb,
                                             void *user_data)
{
    (void)dev;
    s_esp.rx_cb = cb;
    s_esp.rx_cb_arg = user_data;
}

static dal_err_t bsp_esp_ops_get_state(dal_wifi_dev_t *dev, dal_wifi_state_t *state)
{
    (void)dev;
    switch (s_esp.state) {
        case ESP_STATE_IDLE:          *state = DAL_WIFI_STATE_IDLE; break;
        case ESP_STATE_CONNECTING:    *state = DAL_WIFI_STATE_CONNECTING; break;
        case ESP_STATE_CONNECTED:
        case ESP_STATE_GOT_IP:        *state = DAL_WIFI_STATE_CONNECTED; break;
        case ESP_STATE_DISCONNECTING: *state = DAL_WIFI_STATE_DISCONNECTING; break;
        case ESP_STATE_AP_ACTIVE:     *state = DAL_WIFI_STATE_AP_ACTIVE; break;
        case ESP_STATE_SCANNING:      *state = DAL_WIFI_STATE_IDLE; break;
        default:                      *state = DAL_WIFI_STATE_FAULT; break;
    }
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_get_info(dal_wifi_dev_t *dev, uint32_t *capability,
                                      uint8_t *max_sta)
{
    (void)dev;
    if (capability) {
        *capability = DAL_WIFI_CAP_AP_MODE | DAL_WIFI_CAP_RAW_DATA_TX;
    }
    if (max_sta) *max_sta = 4;
    return DAL_OK;
}

static dal_err_t bsp_esp_ops_get_mode(dal_wifi_dev_t *dev, dal_wifi_mode_t *mode)
{ (void)dev; (void)mode; return DAL_ERR_NOT_SUPPORTED; }
static dal_err_t bsp_esp_ops_get_mac(dal_wifi_dev_t *dev, dal_wifi_mac_t *mac)
{ (void)dev; (void)mac; return DAL_ERR_NOT_SUPPORTED; }
static dal_err_t bsp_esp_ops_get_netif_handle(dal_wifi_dev_t *dev, void **h)
{ (void)dev; *h = NULL; return DAL_ERR_NOT_SUPPORTED; }

/* ========================================================================== */
/*                       dal_wifi_ops_t 实例                                   */
/* ========================================================================== */

static const dal_wifi_ops_t g_bsp_esp_ops = {
    .init                 = bsp_esp_ops_init,
    .deinit               = bsp_esp_ops_deinit,
    .set_mode             = bsp_esp_ops_set_mode,
    .get_mode             = bsp_esp_ops_get_mode,
    .set_rf_power         = bsp_esp_ops_set_rf_power,
    .scan_start           = bsp_esp_ops_scan_start,
    .get_scan_result      = bsp_esp_ops_get_scan_result,
    .sta_connect          = bsp_esp_ops_sta_connect,
    .sta_disconnect       = bsp_esp_ops_sta_disconnect,
    .sta_get_info         = bsp_esp_ops_sta_get_info,
    .ap_start             = bsp_esp_ops_ap_start,
    .ap_stop              = bsp_esp_ops_ap_stop,
    .get_state            = bsp_esp_ops_get_state,
    .get_mac              = bsp_esp_ops_get_mac,
    .get_info             = bsp_esp_ops_get_info,
    .get_netif_handle     = bsp_esp_ops_get_netif_handle,
    .transmit             = bsp_esp_ops_transmit,
    .register_rx_callback = bsp_esp_ops_register_rx_callback,
};

/* ========================================================================== */
/*                     BSP 公共接口（仅注册，不初始化硬件）                      */
/* ========================================================================== */

bsp_err_t bsp_esp8266_init(void)
{
    memset(&s_esp, 0, sizeof(s_esp));

    static dal_wifi_dev_t wifi_dev;
    wifi_dev.name     = "esp8266";
    wifi_dev.ops      = &g_bsp_esp_ops;
    wifi_dev.drv_priv = &s_esp;

    static const dal_wifi_config_t cfg = {
        .data_path = DAL_WIFI_DATA_PATH_RAW_SERIAL,
    };
    wifi_dev.config = &cfg;

    dal_err_t ret = dal_wifi_register(&wifi_dev);
    if (ret != DAL_OK) {
        (void)dal_wifi_unregister(&wifi_dev);
        return BSP_ERR_IO;
    }
    return BSP_OK;
}