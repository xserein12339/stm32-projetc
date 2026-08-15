#include "board_v1.h"
#include "dal_key.h"
#include "dal_led.h"
#include "dal_display.h"
#include "dal_motor.h"
#include "dal_encoder.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*                       内置轻量级 MONO 绘图库 (无外部依赖)                      */
/* ========================================================================== */
/**
 * @brief 极简 5x7 ASCII 字模表 (仅包含测试所需字符: 0-9 A-Z a-z : . - / = + %)
 * @note  MSB 在上，LSB 在下，每字节一列，共5列
 */
static const uint8_t s_font5x7[][5] = {
    /* '0'-'9' (ASCII 48-57) */
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    /* ':' '.' '-' '/' '=' '+' '%' */
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x60,0x60,0x00,0x00},
    {0x08,0x08,0x08,0x08,0x08}, {0x20,0x10,0x08,0x04,0x02},
    {0x14,0x14,0x14,0x14,0x14}, {0x10,0x38,0x54,0x10,0x10},
    {0x23,0x13,0x08,0x64,0x62},
    /* 'A'-'Z' (ASCII 65-90) */
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x20,0x1F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
};

static void _oled_clear(uint8_t *buf, uint32_t size) { memset(buf, 0x00, size); }

/**
 * @brief 帧缓冲置像素
 * @note  缓冲格式必须符合 dal_display_draw 契约：MSB-first 水平布局
 *        - 每字节 = 8 个水平相邻像素，bit7 = 最左
 *        - 字节序：从左到右、从上到下
 *        bsp_oled 的 draw() 会将其转置为 SSD1306 垂直页格式后再传输
 */
static void _oled_set_pixel(uint8_t *buf, uint16_t x, uint16_t y, bool on)
{
    if (x >= 128 || y >= 64) return;
    uint16_t idx = (uint16_t)y * 16U + (x / 8U);   /* 每行 16 字节 */
    uint8_t  bit = (uint8_t)(1U << (7U - (x % 8U))); /* bit7 = 最左像素 */
    if (on) buf[idx] |=  bit;
    else    buf[idx] &= (uint8_t)~bit;
}

static void _oled_draw_char(uint8_t *buf, uint8_t x, uint8_t y, char c)
{
    const uint8_t *glyph = NULL;
    if (c >= '0' && c <= '9')      glyph = s_font5x7[c - '0'];
    else if (c >= 'A' && c <= 'Z') glyph = s_font5x7[10 + 7 + (c - 'A')];
    else if (c == ':') glyph = s_font5x7[10];
    else if (c == '.') glyph = s_font5x7[11];
    else if (c == '-') glyph = s_font5x7[12];
    else if (c == '/') glyph = s_font5x7[13];
    else if (c == '=') glyph = s_font5x7[14];
    else if (c == '+') glyph = s_font5x7[15];
    else if (c == '%') glyph = s_font5x7[16];
    else return; /* 不支持的字符跳过 */

    for (uint8_t col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (uint8_t row = 0; row < 7; row++) {
            _oled_set_pixel(buf, x + col, y + row, (line >> row) & 0x01);
        }
    }
}

static void _oled_draw_str(uint8_t *buf, uint8_t x, uint8_t y, const char *str)
{
    while (*str) { _oled_draw_char(buf, x, y, *str); x += 6; str++; }
}

static void _oled_draw_hline(uint8_t *buf, uint8_t x0, uint8_t x1, uint8_t y)
{
    for (uint8_t x = x0; x <= x1; x++) _oled_set_pixel(buf, x, y, true);
}

/**
 * @brief 立即渲染上屏（绕过变化检测）
 * @note  供启动/测试阶段使用：不等 50ms 监控周期，画完即刷
 */
static void _oled_present(dal_display_dev_t *oled, const uint8_t *buf)
{
    dal_display_rect_t full = { .x = 0, .y = 0, .w = 128, .h = 64 };
    if (dal_display_draw(oled, &full, buf, 128 * 64 / 8) == DAL_OK) {
        dal_display_flush(oled);
    }
}

/* ========================================================================== */
/*                              全局资源                                        */
/* ========================================================================== */
#define APP_MAX_ENCODERS    (4U)
#define APP_MAX_MOTORS      (4U)

static uint8_t s_oled_buf[128 * 64 / 8];        /**< 当前帧渲染缓冲 */
static uint8_t s_oled_prev[128 * 64 / 8];       /**< 上次已刷出的帧，用于变化检测 */

/** @brief 初始化时缓存的设备指针，避免每帧遍历 DAL 注册表 */
static dal_encoder_dev_t *s_enc_devs[APP_MAX_ENCODERS];
static dal_motor_dev_t   *s_mot_devs[APP_MAX_MOTORS];
static uint32_t           s_enc_cnt = 0;
static uint32_t           s_mot_cnt = 0;

static QueueHandle_t s_evt_queue = NULL;

/**
 * @brief 诊断错误码（LED1 闪烁次数）
 * @note  取值即闪码次数，勿随意改动已定档的数值
 */
typedef enum {
    DIAG_ERR_OLED_NOT_FOUND = 1,    /**< OLED 设备未注册 */
    DIAG_ERR_OLED_INIT      = 2,    /**< OLED 初始化失败（I2C 不通） */
    DIAG_ERR_OLED_INFO      = 3,    /**< OLED 设备信息异常 */
    DIAG_ERR_QUEUE_CREATE   = 5,    /**< 事件队列创建失败（堆不足） */
} diag_err_t;

typedef struct {
    enum { EVT_SRC_ENCODER, EVT_SRC_MOTOR } src;
    void     *dev_ptr;
    uint32_t  event_bits;
} isr_event_msg_t;

/* ========================================================================== */
/*                   ISR 回调 (严格遵守 ISR 安全契约)                            */
/* ========================================================================== */
static void _encoder_isr_cb(dal_encoder_dev_t *dev, uint32_t event, void *user_data)
{
    (void)user_data;
    BaseType_t woken = pdFALSE;
    isr_event_msg_t msg = { .src = EVT_SRC_ENCODER, .dev_ptr = dev, .event_bits = event };
    xQueueSendFromISR(s_evt_queue, &msg, &woken);
    portYIELD_FROM_ISR(woken);
}

static void _motor_fault_isr_cb(dal_motor_dev_t *dev, uint32_t event, void *user_data)
{
    (void)user_data;
    BaseType_t woken = pdFALSE;
    isr_event_msg_t msg = { .src = EVT_SRC_MOTOR, .dev_ptr = dev, .event_bits = event };
    xQueueSendFromISR(s_evt_queue, &msg, &woken);
    portYIELD_FROM_ISR(woken);
}
/* ========================================================================== */
/*                          OLED 诊断指示 (LED闪码)                              */
/* ========================================================================== */
/**
 * @brief OLED/系统错误指示：LED1 快闪 N 次报错阶段，无限循环
 * @param err_code 错误阶段码（闪几次）
 */
static void _oled_error_halt(uint8_t err_code)
{
    dal_led_dev_t *led1 = dal_led_get_dev("led1");
    if (led1) {
        dal_led_init(led1);
    }

    while (1) {
        for (uint8_t i = 0; i < err_code; i++) {
            if (led1) dal_led_set_state(led1, DAL_LED_ON);
            vTaskDelay(pdMS_TO_TICKS(120));
            if (led1) dal_led_set_state(led1, DAL_LED_OFF);
            vTaskDelay(pdMS_TO_TICKS(120));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/* ========================================================================== */
/*                        原有按键 LED 任务                                     */
/* ========================================================================== */
void app_start_task(void *arg)
{
    (void)arg; 
    dal_key_dev_t *key2 = dal_key_get_dev("key2");
    if (key2) dal_key_init(key2);
    dal_led_dev_t *led2 = dal_led_get_dev("led2");
    if (led2) dal_led_init(led2);

    dal_key_level_t key_state = DAL_KEY_LEVEL_RELEASED;
    while (1) {
        if (key2 && dal_key_get_level(key2, &key_state) == DAL_OK) {
            dal_led_state_t ls = (key_state == DAL_KEY_LEVEL_PRESSED) ? DAL_LED_ON : DAL_LED_OFF;
            if (led2) dal_led_set_state(led2, ls);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ========================================================================== */
/*                     电机+编码器 OLED 综合测试任务                              */
/* ========================================================================== */
void motor_encoder_test_task(void *arg)
{
    (void)arg;

    /* 1. 创建事件队列 */
    s_evt_queue = xQueueCreate(16, sizeof(isr_event_msg_t));
    if (!s_evt_queue) { _oled_error_halt(DIAG_ERR_QUEUE_CREATE); return; }

    /* 2. 初始化所有电机并注册故障回调（同时缓存指针供主循环直用） */
    uint32_t motor_cnt = dal_motor_get_count();
    for (uint32_t i = 0; i < motor_cnt && s_mot_cnt < APP_MAX_MOTORS; i++) {
        dal_motor_dev_t *m = dal_motor_get_dev_by_index(i);
        if (m && dal_motor_init(m) == DAL_OK) {
            dal_motor_set_fault_callback(m, _motor_fault_isr_cb, NULL);
            dal_motor_set_fault_irq_enable(m, true);
            s_mot_devs[s_mot_cnt++] = m;
        }
    }

    /* 3. 初始化所有编码器并注册事件回调（同时缓存指针） */
    uint32_t enc_cnt = dal_encoder_get_count();
    for (uint32_t i = 0; i < enc_cnt && s_enc_cnt < APP_MAX_ENCODERS; i++) {
        dal_encoder_dev_t *e = dal_encoder_get_dev_by_index(i);
        if (e && dal_encoder_init(e) == DAL_OK) {
            dal_encoder_set_event_callback(e, _encoder_isr_cb, NULL);
            dal_encoder_set_event_irq_enable(e, true);
            s_enc_devs[s_enc_cnt++] = e;
        }
    }

    /* 4. 获取并初始化 OLED */
    dal_display_dev_t *oled = dal_display_get_dev("oled");
    dal_display_rect_t full_rect = { .x = 0, .y = 0, .w = 128, .h = 64 };
    if (!oled) { _oled_error_halt(DIAG_ERR_OLED_NOT_FOUND); return; }
    if (dal_display_init(oled) != DAL_OK) { _oled_error_halt(DIAG_ERR_OLED_INIT); return; }

    /* 验证为点阵屏且为 MONO 格式 (符合 draw/set_segment 互斥契约) */
    dal_display_type_t dtype;
    dal_display_pixel_fmt_t fmt;
    dal_display_get_info(oled, NULL, NULL, &fmt, &dtype, NULL);
    if (dtype != DAL_DISPLAY_TYPE_DOT_MATRIX || fmt != DAL_DISPLAY_FMT_MONO) {
        _oled_error_halt(DIAG_ERR_OLED_INFO); return;
    }

    /* 5. 启动所有电机连续运转（CW，30% 占空比），主循环实时监控转速 */
    for (uint32_t i = 0; i < s_mot_cnt; i++) {
        dal_motor_dev_t *m = s_mot_devs[i];
        if (!m) continue;
        dal_motor_enable(m);
        dal_motor_set_direction(m, DAL_MOTOR_DIR_CW);
        dal_motor_set_duty(m, 30);
    }

    /* 启动提示屏（监控主循环接管前先有反馈） */
    _oled_clear(s_oled_buf, sizeof(s_oled_buf));
    _oled_draw_str(s_oled_buf, 0, 0, "== MTR RUN ======");
    _oled_draw_hline(s_oled_buf, 0, 127, 9);
    _oled_draw_str(s_oled_buf, 0, 20, "CW DUTY 30 PCT");
    _oled_present(oled, s_oled_buf);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* KEY1：运行/停止切换。停止时清零编码器计数，供减速比实测校准 */
    dal_key_dev_t *cal_key = dal_key_get_dev("key1");
    if (cal_key) dal_key_init(cal_key);
    dal_key_level_t cal_prev = DAL_KEY_LEVEL_RELEASED;
    bool motors_running = true;

    /* 6. 主循环：OLED 实时监控 + 事件处理 */
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        /* 6a. 非阻塞消费 ISR 事件 (任务上下文，安全调用 DAL API) */
        isr_event_msg_t evt;
        while (xQueueReceive(s_evt_queue, &evt, 0) == pdTRUE) {
            if (evt.src == EVT_SRC_ENCODER && (evt.event_bits & DAL_ENCODER_EVT_OVERFLOW)) {
                /* TODO: 溢出补偿 */
            }
            if (evt.src == EVT_SRC_MOTOR && (evt.event_bits & DAL_MOTOR_EVT_FAULT_CLEAR)) {
                /* TODO: 故障恢复指示 */
            }
        }

        /* 6a'. KEY1 按下沿：电机运行/停止切换
         * 停止时清零编码器计数 -> 手转输出轴整圈读 P 即可实测减速比 */
        if (cal_key) {
            dal_key_level_t kl = DAL_KEY_LEVEL_RELEASED;
            if (dal_key_get_level(cal_key, &kl) == DAL_OK) {
                if (kl == DAL_KEY_LEVEL_PRESSED && cal_prev == DAL_KEY_LEVEL_RELEASED) {
                    motors_running = !motors_running;
                    for (uint32_t i = 0; i < s_mot_cnt; i++) {
                        dal_motor_dev_t *m = s_mot_devs[i];
                        if (!m) continue;
                        if (motors_running) {
                            dal_motor_enable(m);
                            dal_motor_set_direction(m, DAL_MOTOR_DIR_CW);
                            dal_motor_set_duty(m, 30);
                        } else {
                            dal_motor_stop(m);
                            dal_motor_disable(m);
                        }
                    }
                    if (!motors_running) {
                        for (uint32_t i = 0; i < s_enc_cnt; i++) {
                            dal_encoder_reset(s_enc_devs[i]);
                        }
                    }
                }
                cal_prev = kl;
            }
        }

        /* 6b. 检查 flush 并发状态 (遵守 flush 并发互斥契约) */
        dal_display_state_t disp_state;
        dal_display_get_state(oled, &disp_state);
        if (disp_state == DAL_DISPLAY_STATE_BUSY) {
            /* 异步刷新未完成，跳过本帧绘制，避免返回 DAL_ERR_BUSY */
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
            continue;
        }

        /* 6c. 渲染帧缓冲（按运行状态切换页面，每帧全清避免残影） */
        _oled_clear(s_oled_buf, sizeof(s_oled_buf));

        if (motors_running) {
            /* ---- 页面 1：运行监控页 ---- */
            _oled_draw_str(s_oled_buf, 0, 0, "== MTR/ENC MON ==");
            _oled_draw_hline(s_oled_buf, 0, 127, 9);
            uint8_t line = 1;
            for (uint32_t i = 0; i < s_enc_cnt && line < 4; i++) {
                int32_t pos = 0, vel = 0;
                dal_encoder_get_position(s_enc_devs[i], &pos);
                dal_encoder_get_velocity(s_enc_devs[i], &vel);
                /* vel 契约单位 0.1 RPM，拆为整数+小数显示；负数小数位取绝对值 */
                int32_t v_int = vel / 10;
                int32_t v_dec = vel % 10;
                if (v_dec < 0) v_dec = -v_dec;
                char buf[22];
                snprintf(buf, sizeof(buf), "E%lu P:%ld V:%ld.%ld",
                         (unsigned long)i, (long)pos, (long)v_int, (long)v_dec);
                _oled_draw_str(s_oled_buf, 0, line * 10 + 2, buf);
                line++;
            }

            for (uint32_t i = 0; i < s_mot_cnt && line < 6; i++) {
                dal_motor_state_t st;
                uint32_t fault = 0;
                dal_motor_get_state(s_mot_devs[i], &st);
                dal_motor_get_fault(s_mot_devs[i], &fault);
                char buf[22];
                snprintf(buf, sizeof(buf), "M%lu S:%d F:%02lX", (unsigned long)i, (int)st, (unsigned long)fault);
                _oled_draw_str(s_oled_buf, 0, line * 10 + 2, buf);
                line++;
            }
        } else {
            /* ---- 页面 2：减速比校准页（独立整页，不与监控页混排） ---- */
            _oled_draw_str(s_oled_buf, 0, 0, "== GEAR CAL =====");
            _oled_draw_hline(s_oled_buf, 0, 127, 9);

            /* 只读计数，不读速度（停止态测速无意义） */
            uint8_t line = 1;
            for (uint32_t i = 0; i < s_enc_cnt && line < 4; i++) {
                int32_t pos = 0;
                dal_encoder_get_position(s_enc_devs[i], &pos);
                char buf[22];
                snprintf(buf, sizeof(buf), "E%lu P:%ld", (unsigned long)i, (long)pos);
                _oled_draw_str(s_oled_buf, 0, line * 10 + 2, buf);
                line++;
            }

            /* 操作指引 */
            _oled_draw_str(s_oled_buf, 0, 40, "TURN SHAFT 1 TURN");
            _oled_draw_str(s_oled_buf, 0, 50, "RATIO = P / 44");
            _oled_draw_str(s_oled_buf, 0, 60, "KEY1=RESUME RUN");
        }

        /* 6d. 变化检测：内容无变化时跳过 draw+flush，
         *     将 1KB I2C 传输（100kHz 下约 10ms）降为 1KB memcmp（约 0.1ms） */
        if (memcmp(s_oled_buf, s_oled_prev, sizeof(s_oled_buf)) != 0) {
            memcpy(s_oled_prev, s_oled_buf, sizeof(s_oled_buf));
            dal_display_draw(oled, &full_rect, s_oled_buf, sizeof(s_oled_buf));
            dal_display_flush(oled);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

/* ========================================================================== */
/*                                main                                         */
/* ========================================================================== */
int main(void)
{
    if (bsp_init() != BSP_OK) Error_Handler();

    xTaskCreate(app_start_task, "AppStart", 256, NULL, 2, NULL);
    xTaskCreate(motor_encoder_test_task, "MotEncTest", 1024, NULL, 3, NULL);

    vTaskStartScheduler();
    Error_Handler();
    return 0;
}