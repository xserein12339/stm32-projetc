/**
 * @file    dal_display.h
 * @brief   显示设备抽象层 v1.0
 * 
 * @details 支持点阵屏（OLED/LCD/TFT）、段码屏、字符屏的统一抽象。
 *          提供帧缓冲管理、局部刷新、背光控制、异步刷新完成通知。
 *          颜色格式统一为 RGB565 或 MONO，由驱动层负责硬件格式转换。
 *          全程整数运算，无 FPU 依赖。
 * 
 * @author xserein
 * @version v1.0
 */
#ifndef __DAL_DISPLAY_H__
#define __DAL_DISPLAY_H__

#include <stdint.h>
#include <stdbool.h>
#include "dal_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            线程安全契约                                       */
/* ========================================================================== */

/**
 * @defgroup DISPLAY_THREAD_SAFETY 线程安全说明
 * @{
 * - 本接口【非线程安全】，所有公共 API 默认不提供内部互斥保护。
 * - 同一设备的操作（如 draw / flush / set_backlight）需由调用者
 *   自行保证串行化（如通过互斥锁或临界区）。
 * - 事件回调函数在中断/DMA 完成上下文中执行，严禁调用任何阻塞 API。
 * - 跨设备操作无需互斥，设备间完全独立。
 * @}
 */

/* ========================================================================== */
/*                               类型前向声明                                    */
/* ========================================================================== */

typedef struct dal_display_dev dal_display_dev_t; ///< 显示设备实例结构体前向声明

/* ========================================================================== */
/*                             枚举与状态定义                                    */
/* ========================================================================== */

/**
 * @brief 显示设备类型
 */
typedef enum {
    DAL_DISPLAY_TYPE_DOT_MATRIX  = 0, ///< 点阵屏（OLED/LCD/TFT，支持像素级绘图）
    DAL_DISPLAY_TYPE_SEGMENT     = 1, ///< 段码屏（数码管/自定义段码，仅支持预定义图案）
    DAL_DISPLAY_TYPE_CHAR        = 2, ///< 字符屏（HD44780/COG，支持文本行列写入）
} dal_display_type_t;

/**
 * @brief 像素颜色格式
 * @note  帧缓冲区内存布局由驱动层解释，DAL 层仅传递格式标识。
 */
typedef enum {
    DAL_DISPLAY_FMT_MONO     = 0, ///< 单色 1bpp（每字节8像素，MSB优先）
    DAL_DISPLAY_FMT_GRAY4    = 1, ///< 4级灰度 2bpp
    DAL_DISPLAY_FMT_RGB565   = 2, ///< 彩色 16bpp（R5-G6-B5，小端序）
    DAL_DISPLAY_FMT_RGB888   = 3, ///< 彩色 24bpp（R8-G8-B8）
} dal_display_pixel_fmt_t;

/**
 * @brief 屏幕旋转方向
 */
typedef enum {
    DAL_DISPLAY_ROT_0   = 0, ///< 不旋转
    DAL_DISPLAY_ROT_90  = 1, ///< 顺时针旋转 90°
    DAL_DISPLAY_ROT_180 = 2, ///< 顺时针旋转 180°
    DAL_DISPLAY_ROT_270 = 3, ///< 顺时针旋转 270°
} dal_display_rotation_t;

/**
 * @brief 显示设备运行状态
 */
typedef enum {
    DAL_DISPLAY_STATE_OFF      = 0, ///< 关闭（电源/背光均关）
    DAL_DISPLAY_STATE_IDLE     = 1, ///< 空闲（已使能，无刷新进行中）
    DAL_DISPLAY_STATE_BUSY     = 2, ///< 忙（异步刷新/DMA 传输中）
    DAL_DISPLAY_STATE_FAULT    = 3, ///< 故障锁定（通信异常/初始化失败）
} dal_display_state_t;

/**
 * @brief 显示异步事件类型
 * @note  支持位组合，多个事件可同时触发。
 */
typedef enum {
    DAL_DISPLAY_EVT_FLUSH_DONE   = 0x01U, ///< 帧缓冲刷新完成（DMA/传输结束）
    DAL_DISPLAY_EVT_VSYNC        = 0x02U, ///< 垂直同步信号（仅支持 TE 信号的 LCD）
    DAL_DISPLAY_EVT_FAULT        = 0x08U, ///< 硬件故障（通信超时/校验错误）
    DAL_DISPLAY_EVT_FAULT_CLEAR  = 0x80U, ///< 所有故障已清除
} dal_display_event_t;

/**
 * @brief 显示设备专用自检结果码
 */
typedef enum {
    DAL_DISPLAY_SELFTEST_PASS       = 0x00, ///< 自检通过
    DAL_DISPLAY_SELFTEST_ERR_DRV    = 0x01, ///< 驱动上下文异常
    DAL_DISPLAY_SELFTEST_ERR_COMM   = 0x02, ///< 通信链路故障（SPI/I2C 无应答）
    DAL_DISPLAY_SELFTEST_ERR_MEM    = 0x03, ///< 显存读写校验失败
    DAL_DISPLAY_SELFTEST_NOT_IMPL   = 0xFF, ///< 不支持自检
} dal_display_selftest_result_t;

/**
 * @brief 显示设备能力标志位（位掩码）
 */
typedef enum {
    /** 支持硬件加速局部刷新（否则需全刷或软件裁剪） */
    DAL_DISPLAY_CAP_PARTIAL_REFRESH = 0x0001U,

    /** 支持背光亮度调节（set_backlight 有效） */
    DAL_DISPLAY_CAP_BACKLIGHT       = 0x0002U,

    /** 支持屏幕旋转（set_rotation 有效） */
    DAL_DISPLAY_CAP_ROTATION        = 0x0004U,

    /** 支持 VSYNC/TE 信号中断 */
    DAL_DISPLAY_CAP_VSYNC           = 0x0008U,

    /** 支持 DMA 异步刷新（flush_async 有效） */
    DAL_DISPLAY_CAP_ASYNC_FLUSH     = 0x0010U,

    /**
     * 控制器内置显存（GRAM）
     * - 该标志仅表示支持随机像素写入（draw 可直接更新屏幕内容）
     * - 【不代表】可以省略 flush 调用：部分内置 GRAM 控制器
     *   （如 SSD1306）仍需发送特定更新命令才能使写入生效
     * - flush 是否为空操作由具体 DRV 实现决定，上层应始终
     *   在绘制完成后调用 flush，由 DRV 层判断是否需要实际执行
     */
    DAL_DISPLAY_CAP_INTERNAL_GRAM   = 0x0020U,
} dal_display_capability_t;

/**
 * @brief 矩形区域描述符
 * @note  w == 0 或 h == 0 表示无效区域，任何使用无效区域的接口
 *        都应返回 DAL_ERR_PARAM_INVALID，不进行任何硬件操作。
 */
typedef struct {
    uint16_t x;      ///< 左上角 X 坐标 (px)
    uint16_t y;      ///< 左上角 Y 坐标 (px)
    uint16_t w;      ///< 宽度 (px)，0 表示无效区域
    uint16_t h;      ///< 高度 (px)，0 表示无效区域
} dal_display_rect_t;

/**
 * @brief 显示异步事件回调函数原型
 * @param[in] dev       触发事件的设备实例指针
 * @param[in] event     事件类型（支持位组合）
 * @param[in] user_data 用户自定义上下文
 *
 * @warning 【ISR/DMA 安全契约 — 违反将导致系统崩溃】
 *          1. 此回调在中断或 DMA 完成回调上下文中执行
 *          2. 严禁调用任何阻塞 API（delay、mutex_lock、malloc 等）
 *          3. 【严禁】在回调中调用任何 DAL 显示 API
 *             （包括 draw、flush、set_backlight 等）
 *          4. 推荐做法：仅通过非阻塞 ISR 专用接口传递事件到任务
 */
typedef void (*dal_display_event_callback_t)(dal_display_dev_t *dev,
                                             uint32_t event,
                                             void *user_data);

/* ========================================================================== */
/*                        设备操作集（DRV层核心契约）                             */
/* ========================================================================== */

/**
 * @brief 显示硬件抽象操作接口
 * @note  DRV 层实现者需根据显示类型选择性实现接口。
 *        不适用的接口应置为 NULL，DAL 层将返回 DAL_ERR_NOT_SUPPORTED。
 *
 * @par 【draw 与 set_segment 互斥契约】
 *      同一设备【不允许】同时支持 draw 和 set_segment：
 *      - 点阵屏（DOT_MATRIX）：必须实现 draw，set_segment 置 NULL
 *      - 段码屏（SEGMENT）：必须实现 set_segment，draw 置 NULL
 *      - 字符屏（CHAR）：set_segment 用于字符写入，draw 置 NULL
 *      违反此约定的混合实现属于未定义行为。
 */
typedef struct {
    /* --- 生命周期管理 --- */
    dal_err_t (*init)(dal_display_dev_t *dev);
    dal_err_t (*deinit)(dal_display_dev_t *dev);

    /* --- 诊断与测试 --- */
    dal_err_t (*selftest)(dal_display_dev_t *dev,
                          dal_display_selftest_result_t *result);

    /* --- 核心绘图 --- */

    /**
     * @brief 向指定区域写入像素数据
     * @param dev    设备实例指针
     * @param rect   目标区域（坐标相对于当前旋转后的逻辑坐标系）
     * @param data   像素数据缓冲区（格式由 get_info 返回的 pixel_fmt 决定）
     * @param len    数据长度（字节数）
     *
     * @retval DAL_OK               写入成功（同步完成）
     * @retval DAL_ERR_PARAM_INVALID 区域无效（见下方说明）或数据长度不匹配
     * @retval DAL_ERR_NOTSUP       段码屏/字符屏不支持任意像素写入
     *
     * @note  【无效区域判定】
     *        - 若 rect 为 NULL，返回 DAL_ERR_PARAM_INVALID
     *        - 若 rect->w == 0 或 rect->h == 0，视为无效区域，
     *          直接返回 DAL_ERR_PARAM_INVALID，不进行任何硬件操作
     *        - 此判定由 DRV 层负责实现，DAL 框架层不进行拦截
     *
     * @note  【越界检查责任】
     *        DAL 框架不做坐标裁剪，越界检查完全由 DRV 层负责。
     *        DRV 层必须校验 rect 是否超出当前逻辑分辨率，
     *        越界时返回 DAL_ERR_PARAM_INVALID。
     *
     * @note  此接口仅写入显存/GRAM，不保证屏幕立即可见。
     *        绘制完成后应调用 flush / flush_async 使内容生效。
     */
    dal_err_t (*draw)(dal_display_dev_t *dev, const dal_display_rect_t *rect,
                      const uint8_t *data, uint32_t len);

    /**
     * @brief 同步刷新整个屏幕
     * @retval DAL_OK         刷新完成
     * @retval DAL_ERR_BUSY   当前有异步刷新正在进行（见下方并发契约）
     *
     * @note  【并发互斥契约】
     *        若当前有 flush_async 发起的异步刷新正在进行（即
     *        get_state 返回 DAL_DISPLAY_STATE_BUSY），本接口
     *        【必须】立即返回 DAL_ERR_BUSY，【禁止】阻塞等待。
     *        调用者应通过 get_state() 轮询或等待 EVT_FLUSH_DONE
     *        事件后再重试。
     *
     * @note  对于带内部 GRAM 且无需额外更新命令的控制器，
     *        此接口可为空操作（直接返回 DAL_OK）。
     */
    dal_err_t (*flush)(dal_display_dev_t *dev);

    /**
     * @brief 异步刷新整个屏幕
     * @retval DAL_OK         刷新已启动，完成后触发 DAL_DISPLAY_EVT_FLUSH_DONE
     * @retval DAL_ERR_BUSY   上一次异步刷新尚未完成
     * @retval DAL_ERR_NOTSUP 不支持异步刷新
     *
     * @note  【并发互斥契约】
     *        若上一次 flush_async 尚未完成（get_state == BUSY），
     *        再次调用应返回 DAL_ERR_BUSY。
     *        调用者应在收到 EVT_FLUSH_DONE 事件或确认
     *        get_state != BUSY 后才能发起新的异步刷新。
     *
     * @note  调用前建议通过 get_info() 查询 DAL_DISPLAY_CAP_ASYNC_FLUSH。
     */
    dal_err_t (*flush_async)(dal_display_dev_t *dev);

    /* --- 段码屏/字符屏专用 --- */

    /**
     * @brief 设置段码图案或字符内容
     * @param dev     设备实例指针
     * @param index   段码索引或字符位置（行×列+列）
     * @param value   段码编码或 ASCII 字符
     *
     * @retval DAL_OK         设置成功
     * @retval DAL_ERR_NOTSUP 点阵屏不支持此接口
     *
     * @note  参见 dal_display_ops_t 顶部的 draw/set_segment 互斥契约。
     */
    dal_err_t (*set_segment)(dal_display_dev_t *dev, uint32_t index,
                             uint32_t value);

    /* --- 控制与配置 --- */

    /**
     * @brief 设置背光亮度
     * @param dev        设备实例指针
     * @param brightness 亮度等级 (0~255)，0=灭，255=最亮
     * @retval DAL_ERR_NOTSUP 不支持背光调节
     */
    dal_err_t (*set_backlight)(dal_display_dev_t *dev, uint8_t brightness);

    /**
     * @brief 设置屏幕旋转方向
     * @note  旋转后逻辑分辨率自动交换（如 128×64 → 64×128）。
     *        已写入的显存内容不会自动旋转，需重新绘制。
     */
    dal_err_t (*set_rotation)(dal_display_dev_t *dev,
                              dal_display_rotation_t rotation);

    /**
     * @brief 开启/关闭显示（低功耗控制）
     * @param dev    设备实例指针
     * @param on     true=开启显示，false=关闭显示（保留显存内容）
     */
    dal_err_t (*set_power)(dal_display_dev_t *dev, bool on);

    /* --- 状态查询 --- */
    dal_err_t (*get_state)(dal_display_dev_t *dev,
                           dal_display_state_t *state);
    dal_err_t (*get_fault)(dal_display_dev_t *dev, uint32_t *fault);

    /**
     * @brief 获取显示设备硬件参数与能力标志
     * @param dev        设备实例指针
     * @param width      [出参] 逻辑宽度 (px)，不需要时传 NULL
     * @param height     [出参] 逻辑高度 (px)，不需要时传 NULL
     * @param pixel_fmt  [出参] 像素格式，不需要时传 NULL
     * @param type       [出参] 显示类型，不需要时传 NULL
     * @param capability [出参] 能力标志位掩码，不需要时传 NULL
     *
     * @note  width/height 反映当前旋转后的逻辑分辨率。
     */
    dal_err_t (*get_info)(dal_display_dev_t *dev,
                          uint16_t *width, uint16_t *height,
                          dal_display_pixel_fmt_t *pixel_fmt,
                          dal_display_type_t *type,
                          uint32_t *capability);

    /* --- 中断控制（可选） --- */
    dal_err_t (*set_event_irq_enable)(dal_display_dev_t *dev, bool enable);

} dal_display_ops_t;

/* ========================================================================== */
/*                       设备实例结构体                                          */
/* ========================================================================== */

/**
 * @brief 显示设备实例描述符
 *
 * @note 实例化要求：
 *       - 必须是全局或静态存储期（不允许栈上分配）
 *       - name 必须为静态字符串常量，框架仅保存指针不复制
 *       - 注册前需填充：name, ops, drv_priv
 *       - 其余字段由DAL框架管理，DRV/SVC层禁止直接修改
 */
struct dal_display_dev {
    /* === 由BSP层在注册前填充 === */
    const char                *name;      ///< 全局唯一设备标识名
    const dal_display_ops_t   *ops;       ///< 硬件操作集指针
    void                      *drv_priv;  ///< DRV层私有数据

    /* === 由DAL框架管理，DRV/SVC层禁止直接修改 === */
    bool                         initialized;     ///< 硬件是否已初始化
    dal_display_event_callback_t event_cb;        ///< 事件回调函数
    void                        *event_cb_data;   ///< 事件回调用户上下文
};

/* ========================================================================== */
/*                     DAL框架公共API                                           */
/* ========================================================================== */

/** @brief 注册显示设备实例到DAL框架 */
dal_err_t dal_display_register(dal_display_dev_t *dev);

/**
 * @brief 从DAL框架注销显示设备实例
 * @note 【强制注销语义】同 dal_encoder_unregister
 */
dal_err_t dal_display_unregister(dal_display_dev_t *dev);

/** @brief 按名称查找已注册的显示设备实例 */
dal_display_dev_t* dal_display_get_dev(const char *name);

/** @brief 获取已注册的显示设备总数 */
uint32_t dal_display_get_count(void);

/** @brief 按逻辑索引获取已注册的显示设备实例 */
dal_display_dev_t* dal_display_get_dev_by_index(uint32_t index);

/** @brief 初始化显示设备硬件 */
dal_err_t dal_display_init(dal_display_dev_t *dev);

/** @brief 反初始化显示设备硬件（幂等） */
dal_err_t dal_display_deinit(dal_display_dev_t *dev);

/** @brief 执行硬件自检 */
dal_err_t dal_display_selftest(dal_display_dev_t *dev,
                               dal_display_selftest_result_t *result);

/**
 * @brief 向指定区域写入像素数据
 * @param dev    设备实例指针
 * @param rect   目标区域（坐标相对于当前旋转后的逻辑坐标系）
 * @param data   像素数据缓冲区（格式由 get_info 返回的 pixel_fmt 决定）
 * @param len    数据长度（字节数）
 *
 * @retval DAL_OK               写入成功
 * @retval DAL_ERR_PARAM_INVALID 参数无效（rect 为 NULL、w/h 为 0、数据长度不匹配等）
 * @retval DAL_ERR_NOT_READY    设备未初始化或未注册
 * @retval DAL_ERR_NOTSUP       当前设备不支持像素级写入
 *
 * @note  若 rect->w == 0 或 rect->h == 0，视为无效区域，
 *        直接返回 DAL_ERR_PARAM_INVALID。
 *        区域越界检查由 DRV 层负责。
 */
dal_err_t dal_display_draw(dal_display_dev_t *dev,
                           const dal_display_rect_t *rect,
                           const uint8_t *data, uint32_t len);

/** @brief 同步刷新整个屏幕 */
dal_err_t dal_display_flush(dal_display_dev_t *dev);

/** @brief 异步刷新整个屏幕 */
dal_err_t dal_display_flush_async(dal_display_dev_t *dev);

/** @brief 设置段码图案或字符内容 */
dal_err_t dal_display_set_segment(dal_display_dev_t *dev,
                                  uint32_t index, uint32_t value);

/** @brief 设置背光亮度 (0~255) */
dal_err_t dal_display_set_backlight(dal_display_dev_t *dev,
                                    uint8_t brightness);

/** @brief 设置屏幕旋转方向 */
dal_err_t dal_display_set_rotation(dal_display_dev_t *dev,
                                   dal_display_rotation_t rotation);

/** @brief 开启/关闭显示 */
dal_err_t dal_display_set_power(dal_display_dev_t *dev, bool on);

/** @brief 获取当前运行状态 */
dal_err_t dal_display_get_state(dal_display_dev_t *dev,
                                dal_display_state_t *state);

/** @brief 获取当前故障标志位 */
dal_err_t dal_display_get_fault(dal_display_dev_t *dev, uint32_t *fault);

/**
 * @brief 获取显示设备硬件参数与能力标志
 * @see dal_display_ops_t::get_info
 */
dal_err_t dal_display_get_info(dal_display_dev_t *dev,
                               uint16_t *width, uint16_t *height,
                               dal_display_pixel_fmt_t *pixel_fmt,
                               dal_display_type_t *type,
                               uint32_t *capability);

/** @brief 注册异步事件回调 */
dal_err_t dal_display_set_event_callback(dal_display_dev_t *dev,
                                         dal_display_event_callback_t cb,
                                         void *user_data);

/** @brief 使能/禁用硬件事件中断 */
dal_err_t dal_display_set_event_irq_enable(dal_display_dev_t *dev,
                                           bool enable);

/**
 * @brief 通知显示事件（供BSP层在中断/DMA完成回调中调用）
 * @note 此函数【专供 BSP 层 ISR 调用】，在中断上下文中执行。
 */
void dal_display_notify_event(dal_display_dev_t *dev, uint32_t event);

#ifdef __cplusplus
}
#endif

#endif /* __DAL_DISPLAY_H__ */