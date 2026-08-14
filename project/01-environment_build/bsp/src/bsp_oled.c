/**
 * @file    bsp_oled.c
 * @brief   板级 OLED BSP 层实现（SSD1306，128x64，I2C）v1.2
 * @note    - 依赖 bsp_i2c 进行 I2C 通信
 *          - 实现 dal_display_ops_t 并注册到 dal_display 框架
 *          - 内存中维护 1024 字节帧缓冲（8页 × 128列）
 *          - 单色 MONO 格式（1bpp），LSB-first 页内位序
 *          - flush 使用水平寻址模式 + 分块传输（见 _oled_flush_full 注释）
 * @author  xserein
 * @version v1.2
 */

#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_oled.h"
#include "bsp_i2c.h"
#include "dal_display.h"
#include <string.h>

/* ========================================================================== */
/*                             硬件配置                                          */
/* ========================================================================== */

#define OLED_I2C_ADDR           (0x3C)
#define OLED_WIDTH              (128U)
#define OLED_HEIGHT             (64U)
#define OLED_PAGES              (OLED_HEIGHT / 8U)
#define OLED_BUFFER_SIZE        (OLED_WIDTH * OLED_PAGES)
#define OLED_I2C_TIMEOUT_MS     (100U)

/** @brief 命令流缓冲区上限（控制字节 + 最多15条命令） */
#define OLED_CMD_BUF_MAX        (16U)
#define OLED_CMD_PAYLOAD_MAX    (OLED_CMD_BUF_MAX - 1U)

/* ========================================================================== */
/*                         SSD1306 命令定义                                     */
/* ========================================================================== */

#define SSD1306_CMD_SETCONTRAST         0x81
#define SSD1306_CMD_DISPLAYALLON_RESUME 0xA4
#define SSD1306_CMD_SETNORMAL           0xA6
#define SSD1306_CMD_DISPLAYOFF          0xAE
#define SSD1306_CMD_DISPLAYON           0xAF
#define SSD1306_CMD_SETMEMORYMODE       0x20
#define SSD1306_CMD_SETCOLUMN           0x21
#define SSD1306_CMD_SETPAGE             0x22
#define SSD1306_CMD_SETSTARTLINE        0x40
#define SSD1306_CMD_SETSEGMENTREMAP     0xA1
#define SSD1306_CMD_SETCOMSCANDIR_REV   0xC8
#define SSD1306_CMD_SETCOMHWPINS        0xDA
#define SSD1306_CMD_SETMULTIPLEX        0xA8
#define SSD1306_CMD_SETDISPLAYOFFSET    0xD3
#define SSD1306_CMD_SETCHARGEPUMP       0x8D
#define SSD1306_CMD_SETCLOCKDIV         0xD5

#define SSD1306_MEM_MODE_HORIZONTAL     0x00
#define SSD1306_CHARGEPUMP_ENABLE       0x14

#define SSD1306_CTRL_CMD_SINGLE         0x00
#define SSD1306_CTRL_CMD_STREAM         0x00
#define SSD1306_CTRL_DATA_STREAM        0x40

/* ========================================================================== */
/*                        私有数据结构                                          */
/* ========================================================================== */

typedef struct {
    uint8_t frame_buffer[OLED_BUFFER_SIZE];
    bool    is_on;
    bool    is_initialized;
} bsp_oled_priv_t;

static bsp_oled_priv_t   s_oled_priv;
static dal_display_dev_t s_oled_dev;

/* ========================================================================== */
/*                   I2C 通信辅助函数                                            */
/* ========================================================================== */

static dal_err_t _oled_write_cmd(uint8_t cmd, uint8_t param)
{
    uint8_t buf[3];
    uint8_t len;

    if (param == 0) {
        buf[0] = SSD1306_CTRL_CMD_SINGLE;
        buf[1] = cmd;
        len = 2;
    } else {
        buf[0] = SSD1306_CTRL_CMD_STREAM;
        buf[1] = cmd;
        buf[2] = param;
        len = 3;
    }

    bsp_err_t ret = bsp_i2c_write_raw(OLED_I2C_ADDR, buf, len, OLED_I2C_TIMEOUT_MS);
    return (ret == BSP_OK) ? DAL_OK : DAL_ERR_FAIL;
}

/**
 * @brief [v1.2] 批量命令流，超限返回错误而非静默截断
 */
static dal_err_t _oled_write_cmds(const uint8_t *cmds, uint8_t len)
{
    if (len > OLED_CMD_PAYLOAD_MAX) {
        return DAL_ERR_PARAM_INVALID;
    }

    uint8_t buf[OLED_CMD_BUF_MAX];
    buf[0] = SSD1306_CTRL_CMD_STREAM;
    memcpy(&buf[1], cmds, len);

    bsp_err_t ret = bsp_i2c_write_raw(OLED_I2C_ADDR, buf, len + 1, OLED_I2C_TIMEOUT_MS);
    return (ret == BSP_OK) ? DAL_OK : DAL_ERR_FAIL;
}

/**
 * @brief [v1.2] 水平寻址模式全屏刷新（分块传输）
 *
 * @par 设计契约：逻辑原子性 vs 物理原子性
 *      本函数通过 bsp_i2c_lock/unlock 保证 **逻辑原子性**：
 *      在锁持有期间，不会有其他 I2C 设备事务插入到控制命令与数据之间。
 *
 *      但 I2C 总线上仍存在 STOP/START 分隔（每 128B 一块），
 *      这并非真正的 **物理原子性**（单次连续 START...STOP）。
 *
 *      **安全性依据**：SSD1306 在水平寻址模式（Memory Mode = 0x00）下，
 *      每次写入后内部列地址自动递增，跨页时自动换行。
 *      因此多次独立的 I2C Write 事务等效于一次连续传输，
 *      只要中间没有其他设备向同一 SSD1306 发送命令改变地址指针。
 *      bsp_i2c_lock 已保证此条件成立。
 *
 * @par 为何不使用单次 1025B 传输？
 *      HAL_I2C_Master_Transmit 需要 1025B 连续内存缓冲区，
 *      在 FreeRTOS 任务栈（通常 256~512B）上分配会导致栈溢出。
 *      动态分配 1025B 则引入堆碎片风险。
 *      当前分块方案是 RAM 受限环境下的最优折中。
 */
static dal_err_t _oled_flush_full(void)
{
    dal_err_t ret;

    /* 设置水平寻址模式 */
    ret = _oled_write_cmd(SSD1306_CMD_SETMEMORYMODE, SSD1306_MEM_MODE_HORIZONTAL);
    if (ret != DAL_OK) return ret;

    /* 设置列范围 0~127 */
    const uint8_t col_cmds[] = { SSD1306_CMD_SETCOLUMN, 0x00, 0x7F };
    ret = _oled_write_cmds(col_cmds, sizeof(col_cmds));
    if (ret != DAL_OK) return ret;

    /* 设置页范围 0~7 */
    const uint8_t page_cmds[] = { SSD1306_CMD_SETPAGE, 0x00, 0x07 };
    ret = _oled_write_cmds(page_cmds, sizeof(page_cmds));
    if (ret != DAL_OK) return ret;

    /* 获取 I2C 总线锁，保证逻辑原子性 */
    bsp_err_t lock_ret = bsp_i2c_lock(OLED_I2C_TIMEOUT_MS);
    if (lock_ret != BSP_OK) return DAL_ERR_BUSY;

    /* 发送数据控制字节 */
    uint8_t ctrl = SSD1306_CTRL_DATA_STREAM;
    bsp_err_t i2c_ret = bsp_i2c_write_raw(OLED_I2C_ADDR, &ctrl, 1, OLED_I2C_TIMEOUT_MS);

    /* 分块发送帧缓冲（每块 128B，共 8 块） */
    for (uint8_t page = 0; page < OLED_PAGES && i2c_ret == BSP_OK; page++) {
        i2c_ret = bsp_i2c_write_raw(OLED_I2C_ADDR,
                                     &s_oled_priv.frame_buffer[page * OLED_WIDTH],
                                     OLED_WIDTH, OLED_I2C_TIMEOUT_MS);
    }

    bsp_i2c_unlock();
    return (i2c_ret == BSP_OK) ? DAL_OK : DAL_ERR_FAIL;
}

/* ========================================================================== */
/*                         DAL ops 实现                                         */
/* ========================================================================== */

static dal_err_t bsp_oled_ops_init(dal_display_dev_t *dev)
{
    (void)dev;
    bsp_oled_priv_t *priv = &s_oled_priv;
    dal_err_t ret;

    if (bsp_i2c_probe(OLED_I2C_ADDR, 50) != BSP_OK) {
        return DAL_ERR_DEPENDENCY;
    }

    ret = _oled_write_cmd(SSD1306_CMD_DISPLAYOFF, 0);          if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETCLOCKDIV, 0x80);      if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETMULTIPLEX, 0x3F);     if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETDISPLAYOFFSET, 0x00); if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETSTARTLINE, 0);        if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETSEGMENTREMAP, 0);     if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETCOMSCANDIR_REV, 0);   if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETCOMHWPINS, 0x12);     if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETCONTRAST, 0x7F);      if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_DISPLAYALLON_RESUME, 0); if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETNORMAL, 0);           if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETMEMORYMODE, SSD1306_MEM_MODE_HORIZONTAL);
                                                                if (ret != DAL_OK) return ret;
    ret = _oled_write_cmd(SSD1306_CMD_SETCHARGEPUMP, SSD1306_CHARGEPUMP_ENABLE);
                                                                if (ret != DAL_OK) return ret;

    memset(priv->frame_buffer, 0x00, OLED_BUFFER_SIZE);
    ret = _oled_flush_full();
    if (ret != DAL_OK) return ret;

    ret = _oled_write_cmd(SSD1306_CMD_DISPLAYON, 0);
    if (ret != DAL_OK) return ret;

    priv->is_on = true;
    priv->is_initialized = true;
    return DAL_OK;
}

static dal_err_t bsp_oled_ops_deinit(dal_display_dev_t *dev)
{
    (void)dev;
    bsp_oled_priv_t *priv = &s_oled_priv;

    if (!priv->is_initialized) return DAL_OK;

    _oled_write_cmd(SSD1306_CMD_DISPLAYOFF, 0);
    memset(priv->frame_buffer, 0x00, OLED_BUFFER_SIZE);
    priv->is_on = false;
    priv->is_initialized = false;
    return DAL_OK;
}

static dal_err_t bsp_oled_ops_selftest(dal_display_dev_t *dev,
                                       dal_display_selftest_result_t *result)
{
    (void)dev;
    *result = DAL_DISPLAY_SELFTEST_NOT_IMPL;
    return DAL_OK;
}

static dal_err_t bsp_oled_ops_draw(dal_display_dev_t *dev,
                                   const dal_display_rect_t *rect,
                                   const uint8_t *data,
                                   uint32_t len)
{
    (void)dev;
    bsp_oled_priv_t *priv = &s_oled_priv;

    if (rect == NULL || data == NULL) return DAL_ERR_PARAM_INVALID;
    if (rect->w == 0 || rect->h == 0) return DAL_ERR_PARAM_INVALID;
    if (rect->x + rect->w > OLED_WIDTH || rect->y + rect->h > OLED_HEIGHT)
        return DAL_ERR_PARAM_INVALID;
    if ((rect->x & 0x07) != 0 || (rect->w & 0x07) != 0)
        return DAL_ERR_PARAM_INVALID;

    uint16_t bytes_per_row = rect->w / 8;
    uint32_t expected_len = (uint32_t)bytes_per_row * rect->h;
    if (len < expected_len) return DAL_ERR_PARAM_INVALID;

    for (uint16_t row = 0; row < rect->h; row++) {
        uint16_t y_global = rect->y + row;
        uint8_t  page     = y_global / 8;
        uint8_t  bit_pos  = y_global % 8;
        uint16_t row_off  = row * bytes_per_row;

        for (uint16_t col_byte = 0; col_byte < bytes_per_row; col_byte++) {
            uint16_t x_global = rect->x + col_byte * 8;
            uint8_t  byte_val = data[row_off + col_byte];

            for (uint8_t b = 0; b < 8; b++) {
                uint16_t fb_idx = page * OLED_WIDTH + x_global + b;
                if (byte_val & (1U << b)) {
                    priv->frame_buffer[fb_idx] |= (1U << bit_pos);
                } else {
                    priv->frame_buffer[fb_idx] &= ~(1U << bit_pos);
                }
            }
        }
    }

    return DAL_OK;
}

static dal_err_t bsp_oled_ops_flush(dal_display_dev_t *dev)
{
    (void)dev;
    if (!s_oled_priv.is_initialized) return DAL_ERR_NOT_READY;
    return _oled_flush_full();
}

static dal_err_t bsp_oled_ops_flush_async(dal_display_dev_t *dev)
{
    (void)dev;
    return DAL_ERR_NOT_SUPPORTED;
}

static dal_err_t bsp_oled_ops_set_segment(dal_display_dev_t *dev,
                                          uint32_t index, uint32_t value)
{
    (void)dev; (void)index; (void)value;
    return DAL_ERR_NOT_SUPPORTED;
}

static dal_err_t bsp_oled_ops_set_backlight(dal_display_dev_t *dev, uint8_t brightness)
{
    (void)dev; (void)brightness;
    return DAL_ERR_NOT_SUPPORTED;
}

static dal_err_t bsp_oled_ops_set_rotation(dal_display_dev_t *dev,
                                           dal_display_rotation_t rotation)
{
    (void)dev; (void)rotation;
    return DAL_ERR_NOT_SUPPORTED;
}

static dal_err_t bsp_oled_ops_set_power(dal_display_dev_t *dev, bool on)
{
    (void)dev;
    bsp_oled_priv_t *priv = &s_oled_priv;
    if (!priv->is_initialized) return DAL_ERR_NOT_READY;

    dal_err_t ret = _oled_write_cmd(on ? SSD1306_CMD_DISPLAYON : SSD1306_CMD_DISPLAYOFF, 0);
    if (ret == DAL_OK) priv->is_on = on;
    return ret;
}

static dal_err_t bsp_oled_ops_get_state(dal_display_dev_t *dev,
                                        dal_display_state_t *state)
{
    (void)dev;
    bsp_oled_priv_t *priv = &s_oled_priv;

    if (!priv->is_initialized) {
        *state = DAL_DISPLAY_STATE_OFF;
    } else {
        *state = priv->is_on ? DAL_DISPLAY_STATE_IDLE : DAL_DISPLAY_STATE_OFF;
    }
    return DAL_OK;
}

static dal_err_t bsp_oled_ops_get_fault(dal_display_dev_t *dev, uint32_t *fault)
{
    (void)dev;
    *fault = 0U;
    return DAL_OK;
}

/**
 * @brief [v1.2] capability 保持 INTERNAL_GRAM，不添加 PARTIAL_REFRESH
 * @note  draw() 修改帧缓冲属于软件层面的局部更新，
 *        DAL_DISPLAY_CAP_PARTIAL_REFRESH 语义为"硬件加速局部刷新"，
 *        SSD1306 不支持指定区域的硬件级局部写入，故不声明此能力。
 */
static dal_err_t bsp_oled_ops_get_info(dal_display_dev_t *dev,
                                       uint16_t *width, uint16_t *height,
                                       dal_display_pixel_fmt_t *pixel_fmt,
                                       dal_display_type_t *type,
                                       uint32_t *capability)
{
    (void)dev;
    if (!s_oled_priv.is_initialized) return DAL_ERR_NOT_READY;

    if (width)      *width      = OLED_WIDTH;
    if (height)     *height     = OLED_HEIGHT;
    if (pixel_fmt)  *pixel_fmt  = DAL_DISPLAY_FMT_MONO;
    if (type)       *type       = DAL_DISPLAY_TYPE_DOT_MATRIX;
    if (capability) *capability = DAL_DISPLAY_CAP_INTERNAL_GRAM;
    return DAL_OK;
}

static dal_err_t bsp_oled_ops_set_event_irq_enable(dal_display_dev_t *dev, bool enable)
{
    (void)dev; (void)enable;
    return DAL_ERR_NOT_SUPPORTED;
}

/* ========================================================================== */
/*                       dal_display_ops_t 实例                                 */
/* ========================================================================== */

static const dal_display_ops_t g_bsp_oled_ops = {
    .init                 = bsp_oled_ops_init,
    .deinit               = bsp_oled_ops_deinit,
    .selftest             = bsp_oled_ops_selftest,
    .draw                 = bsp_oled_ops_draw,
    .flush                = bsp_oled_ops_flush,
    .flush_async          = bsp_oled_ops_flush_async,
    .set_segment          = bsp_oled_ops_set_segment,
    .set_backlight        = bsp_oled_ops_set_backlight,
    .set_rotation         = bsp_oled_ops_set_rotation,
    .set_power            = bsp_oled_ops_set_power,
    .get_state            = bsp_oled_ops_get_state,
    .get_fault            = bsp_oled_ops_get_fault,
    .get_info             = bsp_oled_ops_get_info,
    .set_event_irq_enable = bsp_oled_ops_set_event_irq_enable,
};

/* ========================================================================== */
/*                     BSP 公共接口                                            */
/* ========================================================================== */

bsp_err_t bsp_oled_init(void)
{
    memset(&s_oled_priv, 0, sizeof(s_oled_priv));

    s_oled_dev.name     = "oled";
    s_oled_dev.ops      = &g_bsp_oled_ops;
    s_oled_dev.drv_priv = &s_oled_priv;

    dal_err_t ret = dal_display_register(&s_oled_dev);
    if (ret != DAL_OK) return BSP_ERR_FAIL;

    ret = dal_display_init(&s_oled_dev);
    if (ret != DAL_OK) {
        (void)dal_display_unregister(&s_oled_dev);
        return BSP_ERR_IO;
    }

    return BSP_OK;
}