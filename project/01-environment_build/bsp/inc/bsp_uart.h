/**
 * @file    bsp_uart.h
 * @brief   BSP UART 多实例驱动接口（含DMA与非阻塞模式）
 *
 * @details 支持多UART外设独立管理，提供阻塞/DMA/非阻塞三种传输模式。
 *          支持全双工独立控制、用户上下文回调及精细化超时配置。
 *
 * @author  xserein
 * @version v2.2
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "bsp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                               类型前向声明                                    */
/* ========================================================================== */

typedef struct bsp_uart_dev *bsp_uart_handle_t; ///< UART 设备句柄（不透明指针）

/* ========================================================================== */
/*                             枚举与常量定义                                    */
/* ========================================================================== */

/**
 * @brief UART 校验方式
 */
typedef enum {
    BSP_UART_PARITY_NONE = 0, ///< 无校验
    BSP_UART_PARITY_ODD,      ///< 奇校验
    BSP_UART_PARITY_EVEN,     ///< 偶校验
} bsp_uart_parity_t;

/**
 * @brief UART 停止位
 */
typedef enum {
    BSP_UART_STOP_BITS_1 = 0, ///< 1位停止位
    BSP_UART_STOP_BITS_2,     ///< 2位停止位
} bsp_uart_stop_bits_t;

/**
 * @brief UART 数据位
 * @note  具体支持范围取决于底层硬件。若传入硬件不支持的枚举值，
 *        bsp_uart_open 将返回 BSP_ERR_NOTSUP。
 */
typedef enum {
    BSP_UART_DATA_BITS_5 = 5, ///< 5位数据位
    BSP_UART_DATA_BITS_6 = 6, ///< 6位数据位
    BSP_UART_DATA_BITS_7 = 7, ///< 7位数据位
    BSP_UART_DATA_BITS_8 = 8, ///< 8位数据位
    BSP_UART_DATA_BITS_9 = 9, ///< 9位数据位
} bsp_uart_data_bits_t;

/**
 * @brief UART 异步事件枚举
 */
typedef enum {
    BSP_UART_EVT_TX_COMPLETE = 0x01U, ///< 发送完成
    BSP_UART_EVT_RX_COMPLETE = 0x02U, ///< 接收完成（达到请求长度）
    BSP_UART_EVT_RX_TIMEOUT  = 0x04U, ///< 接收超时（部分数据已接收）
    BSP_UART_EVT_ERROR       = 0x08U, ///< 硬件错误（溢出/帧/校验/噪声）
    BSP_UART_EVT_ABORT       = 0x10U, ///< 主动中止（由 bsp_uart_abort 触发）
} bsp_uart_event_t;

/**
 * @brief 传输方向标志（用于独立控制TX/RX，支持位运算组合）
 */
typedef enum {
    BSP_UART_DIR_TX = 0x01U, ///< 发送方向
    BSP_UART_DIR_RX = 0x02U, ///< 接收方向
} bsp_uart_dir_t;

/* ========================================================================== */
/*                           配置与事件结构体                                     */
/* ========================================================================== */

/**
 * @brief UART 初始化配置结构体（纯输入参数）
 */
typedef struct {
    uint32_t             baudrate;      ///< 波特率
    bsp_uart_data_bits_t data_bits;     ///< 数据位
    bsp_uart_parity_t    parity;        ///< 校验方式
    bsp_uart_stop_bits_t stop_bits;     ///< 停止位
    uint32_t             rx_timeout_ms; ///< 非阻塞接收超时(ms)，0=禁用超时
} bsp_uart_config_t;

/**
 * @brief UART 错误详情
 */
typedef struct {
    bool overrun; ///< 溢出错误
    bool framing; ///< 帧错误
    bool parity;  ///< 校验错误
    bool noise;   ///< 噪声错误
} bsp_uart_err_detail_t;

/**
 * @brief UART 异步事件信息结构体
 */
typedef struct {
    bsp_uart_event_t      event;      ///< 事件类型
    uint32_t              bytes_done; ///< 已完成字节数
    const uint8_t        *buf;        ///< 关联的数据缓冲区指针
    bsp_uart_err_detail_t err_detail; ///< 仅 EVT_ERROR 时有效；EVT_ABORT 时所有字段为 false
} bsp_uart_evt_info_t;

/* ========================================================================== */
/*                              回调函数定义                                     */
/* ========================================================================== */

/**
 * @brief UART 异步事件回调函数原型
 * @param[in] handle    触发事件的设备句柄
 * @param[in] info      事件详细信息结构体
 * @param[in] user_data 用户自定义上下文指针
 *
 * @warning 此回调在中断 / DMA完成上下文中执行！
 *          - 严禁调用任何阻塞API（如 bsp_uart_send/recv、delay、mutex_lock 等）
 *          - 严禁执行浮点运算或耗时操作
 *          - 严禁使用信号量/消息队列的【阻塞】发送接口
 *          - 如需将事件传递给任务，请使用非阻塞的 ISR 专用接口
 *            （如 FreeRTOS 的 xQueueSendFromISR / xSemaphoreGiveFromISR，
 *             RT-Thread 的 rt_sem_release 等非阻塞变体）
 *          - 违反以上约束将导致系统崩溃或实时性劣化
 */
typedef void (*bsp_uart_callback_t)(bsp_uart_handle_t handle,
                                    const bsp_uart_evt_info_t *info,
                                    void *user_data);

/* ========================================================================== */
/*                            生命周期接口                                       */
/* ========================================================================== */

/**
 * @brief 打开并初始化指定UART外设
 * @param[in]  id     UART外设编号（如1, 2, 3）
 * @param[in]  cfg    配置参数（纯输入，驱动不会修改此结构体）
 * @param[out] handle 输出设备句柄
 * @retval BSP_OK           成功
 * @retval BSP_ERR_INVAL    参数无效或id超出范围
 * @retval BSP_ERR_NOTSUP   硬件不支持请求的配置（如特定的数据位/校验位组合）
 * @retval BSP_ERR_IO       硬件初始化失败
 * @retval BSP_ERR_NOMEM    内部资源分配失败
 * @retval BSP_ERR_BUSY     同一id重复调用
 */
bsp_err_t bsp_uart_open(uint8_t id, const bsp_uart_config_t *cfg,
                        bsp_uart_handle_t *handle);

/**
 * @brief 关闭UART外设并释放所有资源
 * @param[in] handle 设备句柄
 * @retval BSP_OK           成功
 * @retval BSP_ERR_INVAL    句柄无效
 * @note   自动中止该实例所有进行中的传输并注销回调
 */
bsp_err_t bsp_uart_close(bsp_uart_handle_t handle);

/* ========================================================================== */
/*                             阻塞传输接口                                      */
/* ========================================================================== */

/**
 * @brief 阻塞发送
 * @param[in] handle     设备句柄
 * @param[in] data       数据缓冲区
 * @param[in] len        数据长度
 * @param[in] timeout_ms 超时时间(ms)，0=永久等待
 * @retval BSP_OK           发送成功
 * @retval BSP_ERR_BUSY     TX通道正被DMA/非阻塞占用（立即返回，不等待）
 * @retval BSP_ERR_TIMEOUT  超时
 * @retval BSP_ERR_IO       硬件错误
 * @retval BSP_ERR_INVAL    参数无效
 * @note   TX/RX通道独立锁定，阻塞发送期间RX通道不受影响
 */
bsp_err_t bsp_uart_send(bsp_uart_handle_t handle,
                        const uint8_t *data, uint16_t len,
                        uint32_t timeout_ms);

/**
 * @brief 阻塞接收
 * @param[in]  handle     设备句柄
 * @param[out] buf        接收缓冲区
 * @param[in]  len        期望长度
 * @param[out] recv_len   【强制】实际接收字节数，不可为NULL
 * @param[in]  timeout_ms 超时时间(ms)，0=永久等待
 * @retval BSP_OK           接收成功 (recv_len == len)
 * @retval BSP_ERR_TIMEOUT  超时 (recv_len < len，包含已接收的部分数据)
 * @retval BSP_ERR_BUSY     RX通道正被DMA/非阻塞占用（立即返回，不等待）
 * @retval BSP_ERR_IO       硬件错误
 * @retval BSP_ERR_INVAL    参数无效（含 recv_len 为 NULL）
 * @note   TX/RX通道独立锁定，阻塞接收期间TX通道不受影响
 */
bsp_err_t bsp_uart_recv(bsp_uart_handle_t handle,
                        uint8_t *buf, uint16_t len,
                        uint16_t *recv_len, uint32_t timeout_ms);

/* ========================================================================== */
/*                              DMA 传输接口                                     */
/* ========================================================================== */

/**
 * @brief DMA发送（非阻塞）
 * @param[in] handle 设备句柄
 * @param[in] data   数据缓冲区（生命周期须持续到 TX_COMPLETE 回调触发）
 * @param[in] len    数据长度
 * @retval BSP_OK           DMA传输已启动
 * @retval BSP_ERR_BUSY     TX通道已被占用
 * @retval BSP_ERR_INVAL    参数无效或长度超出DMA单次上限
 * @retval BSP_ERR_NOT_INIT 未初始化
 */
bsp_err_t bsp_uart_send_dma(bsp_uart_handle_t handle,
                            const uint8_t *data, uint16_t len);

/**
 * @brief DMA接收（非阻塞）
 * @param[in] handle 设备句柄
 * @param[out] buf   接收缓冲区（生命周期须持续到回调触发）
 * @param[in] len    期望接收长度
 * @retval BSP_OK           DMA接收已启动
 * @retval BSP_ERR_BUSY     RX通道已被占用
 * @retval BSP_ERR_INVAL    参数无效或长度超出DMA单次上限
 * @retval BSP_ERR_NOT_INIT 未初始化
 * @note   超时由 open 时 cfg->rx_timeout_ms 决定
 */
bsp_err_t bsp_uart_recv_dma(bsp_uart_handle_t handle,
                            uint8_t *buf, uint16_t len);

/* ========================================================================== */
/*                          非阻塞控制接口                                       */
/* ========================================================================== */

/**
 * @brief 注册异步回调（每实例独立，支持用户上下文）
 * @param[in] handle    设备句柄
 * @param[in] cb        回调函数，NULL=注销
 * @param[in] user_data 用户自定义上下文，原样透传给回调
 * @retval BSP_OK           成功
 * @retval BSP_ERR_INVAL    句柄无效
 */
bsp_err_t bsp_uart_set_callback(bsp_uart_handle_t handle,
                                bsp_uart_callback_t cb,
                                void *user_data);

/**
 * @brief 查询指定方向的忙状态（TX/RX独立）
 * @param[in]  handle 设备句柄
 * @param[in]  dir    查询方向（BSP_UART_DIR_TX / BSP_UART_DIR_RX）
 * @param[out] busy   输出忙标志
 * @retval BSP_OK           成功
 * @retval BSP_ERR_INVAL    句柄无效或busy为NULL
 */
bsp_err_t bsp_uart_is_busy(bsp_uart_handle_t handle,
                           bsp_uart_dir_t dir, bool *busy);

/**
 * @brief 按方向独立中止传输
 * @param[in] handle 设备句柄
 * @param[in] dir    中止方向（可用 | 组合同时中止TX和RX）
 * @retval BSP_OK           中止成功或本无传输在进行
 * @retval BSP_ERR_INVAL    句柄无效或dir为0
 * @note   中止后【必定】触发 BSP_UART_EVT_ABORT 事件；
 *         info->bytes_done 为中止前已传输字节数；
 *         info->err_detail 所有字段为 false（区别于硬件错误）；
 *         不会触发 TX_COMPLETE / RX_COMPLETE
 */
bsp_err_t bsp_uart_abort(bsp_uart_handle_t handle, bsp_uart_dir_t dir);

/**
 * @brief 获取当前实例的DMA单次最大传输长度
 * @param[in]  handle  设备句柄
 * @param[out] max_len 输出最大长度
 * @retval BSP_OK           成功
 * @retval BSP_ERR_INVAL    句柄无效或max_len为NULL
 * @note   由底层硬件决定，供上层预检避免 BSP_ERR_INVAL
 */
bsp_err_t bsp_uart_get_dma_max_len(bsp_uart_handle_t handle,
                                   uint16_t *max_len);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H__ */