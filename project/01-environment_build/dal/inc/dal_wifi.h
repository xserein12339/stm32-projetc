/**
 * @file    dal_wifi.h
 * @brief   Wi-Fi 设备抽象层 v1.1
 * 
 * @details 提供 Wi-Fi 射频控制、Station/AP 模式管理、扫描与连接状态机抽象。
 *          【核心设计原则：控制面与数据面分离】
 *          - 控制面（本层）：负责配置、扫描、连接/断开、事件通知。
 *          - 数据面（协议栈）：支持两种数据路径模式，由数据路径类型决定：
 *            ① HOST_STACK 模式：IP 层收发由 LwIP/NetX 等协议栈通过 DRV 层暴露的
 *               get_netif_handle() 直接处理，本层不提供通用数据收发 API。
 *            ② RAW_SERIAL 模式：适用于 ESP8266 AT 等自带协议栈的串口 Wi-Fi 模组，
 *               通过 transmit() 和 register_rx_callback() 实现裸数据收发。
 * 
 * @author xserein
 * @version v1.1
 */
#ifndef __DAL_WIFI_H__
#define __DAL_WIFI_H__

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
 * @defgroup WIFI_THREAD_SAFETY 线程安全说明
 * @{
 * - 本接口【非线程安全】，所有公共 API 默认不提供内部互斥保护。
 * - 同一设备的操作（如 connect / disconnect / scan）需由调用者
 *   自行保证串行化（如通过互斥锁或 RTOS 任务串行执行）。
 * - 事件回调函数在 Wi-Fi 驱动的任务上下文或中断上下文中执行，
 *   【严禁】在回调中调用任何阻塞 API 或本模块的控制面 API。
 * - 数据接收回调（RAW_SERIAL 模式）在 UART ISR 或驱动轮询任务中执行，
 *   同样禁止阻塞操作。
 * @}
 */

/* ========================================================================== */
/*                               类型前向声明                                    */
/* ========================================================================== */

typedef struct dal_wifi_dev dal_wifi_dev_t; ///< Wi-Fi 设备实例结构体前向声明

/* ========================================================================== */
/*                             枚举与状态定义                                    */
/* ========================================================================== */

/**
 * @brief Wi-Fi 工作模式
 */
typedef enum {
    DAL_WIFI_MODE_NULL    = 0, ///< 未初始化 / 射频关闭
    DAL_WIFI_MODE_STA     = 1, ///< Station 模式（客户端）
    DAL_WIFI_MODE_AP      = 2, ///< Access Point 模式（热点）
    DAL_WIFI_MODE_STA_AP  = 3, ///< Station + AP 共存模式
} dal_wifi_mode_t;

/**
 * @brief 数据路径类型（决定数据面的实现方式）
 * @note  该类型在设备注册前通过 dal_wifi_config_t 指定，
 *        运行时不可切换。
 */
typedef enum {
    /**
     * @brief 主机协议栈模式（默认）
     * - 适用于 SDIO/SPI 接口的 Wi-Fi 芯片（ESP32-C3、RTL8720、BCM 等）
     * - 数据收发由主机 LwIP/NetX 协议栈通过 netif 接口处理
     * - 本层仅提供控制面 API，数据面由协议栈接管
     * - get_netif_handle() 返回有效的协议栈句柄
     */
    DAL_WIFI_DATA_PATH_HOST_STACK = 0,

    /**
     * @brief 串口裸数据透传模式
     * - 适用于 ESP8266 AT、EC200 等自带 TCP/IP 协议栈的串口模组
     * - 数据收发通过 transmit() 发送（AT+CIPSEND），
     *   通过 register_rx_callback() 接收（+IPD 回调）
     * - 本层提供完整的控制面 + 数据面接口
     * - get_netif_handle() 返回 NULL（不支持主机协议栈）
     */
    DAL_WIFI_DATA_PATH_RAW_SERIAL = 1,
} dal_wifi_data_path_t;

/**
 * @brief Wi-Fi 加密/认证方式
 */
typedef enum {
    DAL_WIFI_AUTH_OPEN         = 0, ///< 开放网络（无密码）
    DAL_WIFI_AUTH_WPA2_PSK     = 1, ///< WPA2-PSK (AES) - 最常用
    DAL_WIFI_AUTH_WPA3_SAE     = 2, ///< WPA3-SAE
    DAL_WIFI_AUTH_WPA2_WPA3    = 3, ///< WPA2/WPA3 混合模式
    DAL_WIFI_AUTH_ENTERPRISE   = 4, ///< WPA2-Enterprise (802.1X)
    DAL_WIFI_AUTH_AUTO         = 0xFF, ///< 自动协商（由 DRV 层自动适配 AP 加密方式）
} dal_wifi_auth_mode_t;

/**
 * @brief Wi-Fi 频段
 */
typedef enum {
    DAL_WIFI_BAND_2G = 0, ///< 2.4 GHz
    DAL_WIFI_BAND_5G = 1, ///< 5 GHz
} dal_wifi_band_t;

/**
 * @brief Wi-Fi 设备运行状态（连接状态机）
 */
typedef enum {
    DAL_WIFI_STATE_IDLE        = 0, ///< 空闲（射频已开启，未连接/未启动AP）
    DAL_WIFI_STATE_CONNECTING  = 1, ///< STA 正在连接中
    DAL_WIFI_STATE_CONNECTED   = 2, ///< STA 已连接（关联成功，可能尚未获取IP）
    DAL_WIFI_STATE_DISCONNECTING = 3, ///< STA 正在断开中
    DAL_WIFI_STATE_AP_ACTIVE   = 4, ///< AP 模式已启动并广播
    DAL_WIFI_STATE_FAULT       = 5, ///< 故障锁定（硬件异常/固件崩溃）
} dal_wifi_state_t;

/**
 * @brief Wi-Fi 异步事件类型（位掩码，支持组合触发）
 * @note  多个事件可能同时触发，回调中应使用 if (event & DAL_WIFI_EVT_XXX) 
 *        方式逐位判断，不可用 switch-case。
 */
typedef enum {
    /** STA 成功连接到 AP（关联成功，此时可能尚未获取 IP） */
    DAL_WIFI_EVT_STA_CONNECTED     = 0x0001U,

    /** STA 从 AP 断开（包含主动断开与被动掉线） */
    DAL_WIFI_EVT_STA_DISCONNECTED  = 0x0002U,

    /** STA 扫描完成（结果需通过 get_scan_result 获取） */
    DAL_WIFI_EVT_SCAN_DONE         = 0x0004U,

    /** STA 成功获取 IP 地址（DHCP 完成或静态 IP 生效） */
    DAL_WIFI_EVT_STA_GOT_IP        = 0x0008U,

    /** AP 模式成功启动 */
    DAL_WIFI_EVT_AP_STARTED        = 0x0010U,

    /** AP 模式已停止 */
    DAL_WIFI_EVT_AP_STOPPED        = 0x0020U,

    /** 有新的 Station 连接到本 AP */
    DAL_WIFI_EVT_AP_STA_CONNECTED  = 0x0040U,

    /** 有 Station 从本 AP 断开 */
    DAL_WIFI_EVT_AP_STA_DISCONNECTED = 0x0080U,

    /** 硬件/固件故障 */
    DAL_WIFI_EVT_FAULT             = 0x8000U,
} dal_wifi_event_t;

/**
 * @brief Wi-Fi 断开原因码（用于 EVT_STA_DISCONNECTED 事件）
 */
typedef enum {
    DAL_WIFI_DISCONN_REASON_UNSPECIFIED    = 0,  ///< 未指定
    DAL_WIFI_DISCONN_REASON_USER_REQUEST   = 1,  ///< 用户主动调用 disconnect
    DAL_WIFI_DISCONN_REASON_AUTH_EXPIRE    = 2,  ///< 认证超时/失败（含加密方式不匹配）
    DAL_WIFI_DISCONN_REASON_ASSOC_EXPIRE   = 3,  ///< 关联超时
    DAL_WIFI_DISCONN_REASON_HANDSHAKE_TIMEOUT = 4, ///< 四次握手超时（密码错误常见原因）
    DAL_WIFI_DISCONN_REASON_BEACON_TIMEOUT = 5,  ///< 信标超时（离开覆盖范围）
    DAL_WIFI_DISCONN_REASON_NO_AP_FOUND    = 6,  ///< 找不到目标 AP
    DAL_WIFI_DISCONN_REASON_ASSOC_LEAVE    = 7,  ///< AP 主动踢出
} dal_wifi_disconnect_reason_t;

/**
 * @brief Wi-Fi 设备能力标志位（位掩码）
 */
typedef enum {
    /** 支持 5GHz 频段（双频） */
    DAL_WIFI_CAP_5G           = 0x0001U,

    /** 支持 WPA3 加密 */
    DAL_WIFI_CAP_WPA3         = 0x0002U,

    /** 支持 AP 模式 */
    DAL_WIFI_CAP_AP_MODE      = 0x0004U,

    /** 支持 STA+AP 共存 */
    DAL_WIFI_CAP_STA_AP_COEX  = 0x0008U,

    /** 支持连接状态下后台扫描 */
    DAL_WIFI_CAP_SCAN_WHILE_CONN = 0x0010U,

    /**
     * 支持裸数据收发（RAW_SERIAL 模式）
     * - 此标志由 DRV 层根据数据路径类型自动设置
     * - 若支持，则 transmit() 和 register_rx_callback() 可用
     * - 若不支持（HOST_STACK 模式），调用 transmit() 返回 NOTSUP
     */
    DAL_WIFI_CAP_RAW_DATA_TX  = 0x0020U,
} dal_wifi_capability_t;

/* ========================================================================== */
/*                           数据结构定义                                        */
/* ========================================================================== */

/**
 * @brief MAC 地址结构体
 */
typedef struct {
    uint8_t addr[6]; ///< 6 字节 MAC 地址（网络字节序）
} dal_wifi_mac_t;

/**
 * @brief IP 地址结构体（仅支持 IPv4，IPv6 由协议栈独立管理）
 */
typedef struct {
    uint32_t addr; ///< IPv4 地址（网络字节序，如 0x0100A8C0 表示 192.168.0.1）
} dal_wifi_ip_t;

/**
 * @brief Wi-Fi 设备初始化配置
 * @note  在 dal_wifi_register() 之前填充，用于指定数据路径模式。
 *        若未提供此结构体，默认为 HOST_STACK 模式（向后兼容）。
 */
typedef struct {
    /** 数据路径类型，决定数据面的实现方式 */
    dal_wifi_data_path_t data_path;

    /**
     * RAW_SERIAL 模式专用：UART 接收回调的上下文
     * @note  当 data_path == RAW_SERIAL 时，应用层可通过此字段
     *        在注册设备时预先绑定接收回调的用户上下文。
     *        也可在注册后通过 register_rx_callback() 另行设置。
     */
    void *rx_callback_arg;
} dal_wifi_config_t;

/**
 * @brief STA 模式连接配置
 */
typedef struct {
    char             ssid[33];     ///< SSID（最长 32 字节 + '\0'）
    char             password[65]; ///< 密码（最长 64 字节 + '\0'）
    dal_wifi_mac_t   bssid;        ///< 目标 BSSID（全 0 表示不指定，连接任意同名 SSID）
    bool             bssid_set;    ///< 是否启用 BSSID 过滤
    uint8_t          channel;      ///< 指定信道（0 = 自动扫描所有信道）
    /**
     * @brief 预期的加密方式
     * @note  若设为 DAL_WIFI_AUTH_AUTO，DRV 层自动适配 AP 的加密方式。
     *        否则，若与 AP 实际加密方式不匹配，连接失败并触发
     *        EVT_STA_DISCONNECTED，reason 为 AUTH_EXPIRE。
     */
    dal_wifi_auth_mode_t auth_mode;
} dal_wifi_sta_config_t;

/**
 * @brief AP 模式配置
 */
typedef struct {
    char             ssid[33];     ///< 广播的 SSID
    char             password[65]; ///< 密码（auth_mode 为 OPEN 时忽略）
    uint8_t          channel;      ///< 工作信道（1~13 for 2.4G）
    dal_wifi_auth_mode_t auth_mode;///< 加密方式
    uint8_t          max_stations; ///< 允许连接的最大 Station 数量（0 = 使用硬件默认值）
    bool             hidden;       ///< 是否隐藏 SSID
} dal_wifi_ap_config_t;

/**
 * @brief 扫描结果条目
 */
typedef struct {
    char             ssid[33];     ///< SSID
    dal_wifi_mac_t   bssid;        ///< BSSID
    int8_t           rssi;         ///< 信号强度 (dBm，通常为负值)
    uint8_t          channel;      ///< 信道
    dal_wifi_auth_mode_t auth_mode;///< 加密方式
    dal_wifi_band_t  band;         ///< 频段
} dal_wifi_scan_record_t;

/**
 * @brief STA 模式当前连接信息（运行时状态）
 */
typedef struct {
    char             ssid[33];     ///< 当前连接的 SSID
    dal_wifi_mac_t   bssid;        ///< 当前连接的 BSSID
    int8_t           rssi;         ///< 当前信号强度 (dBm)
    uint8_t          channel;      ///< 当前信道
    dal_wifi_ip_t    ip;           ///< 当前 IP 地址（未获取 IP 时为 0）
    dal_wifi_ip_t    netmask;      ///< 子网掩码
    dal_wifi_ip_t    gateway;      ///< 网关
} dal_wifi_sta_info_t;

/**
 * @brief Wi-Fi 异步事件回调附带的数据载荷
 * @note  根据触发的 event 类型，联合体中只有对应的成员有效。
 */
typedef union {
    /** 当 event == DAL_WIFI_EVT_STA_DISCONNECTED 时有效 */
    struct {
        dal_wifi_disconnect_reason_t reason; ///< 断开原因
    } sta_disconn;

    /** 当 event == DAL_WIFI_EVT_STA_GOT_IP 时有效 */
    struct {
        dal_wifi_ip_t ip;       ///< 获取到的 IP
        dal_wifi_ip_t netmask;  ///< 子网掩码
        dal_wifi_ip_t gateway;  ///< 网关
    } sta_got_ip;

    /** 当 event == DAL_WIFI_EVT_AP_STA_CONNECTED / DISCONNECTED 时有效 */
    struct {
        dal_wifi_mac_t mac;     ///< 连入/断出的 Station MAC 地址
    } ap_sta;

    /** 当 event == DAL_WIFI_EVT_SCAN_DONE 时有效 */
    struct {
        uint16_t count;         ///< 本次扫描到的 AP 总数（可用作分页读取的总数参考）
    } scan_done;
} dal_wifi_event_data_t;

/**
 * @brief Wi-Fi 异步事件回调函数原型
 * @param[in] dev       触发事件的设备实例指针
 * @param[in] event     事件类型（支持位组合）
 * @param[in] data      事件附带数据（可能为 NULL，视 event 类型而定）
 * @param[in] user_data 用户自定义上下文
 *
 * @warning 【上下文安全契约 — 违反将导致系统死锁或崩溃】
 *          1. 此回调在 Wi-Fi 驱动的内部任务或中断上下文中执行。
 *          2. 【严禁】调用任何阻塞 API（delay、mutex_lock、malloc 等）。
 *          3. 【严禁】在回调中调用本模块的控制面 API（如 connect、disconnect）。
 *          4. 推荐做法：通过 RTOS 消息队列或事件标志组将事件传递给主任务。
 */
typedef void (*dal_wifi_event_callback_t)(dal_wifi_dev_t *dev,
                                          uint32_t event,
                                          const dal_wifi_event_data_t *data,
                                          void *user_data);

/**
 * @brief Wi-Fi 数据接收回调函数原型（RAW_SERIAL 模式）
 * @param[in] dev       设备实例指针
 * @param[in] data      接收到的数据缓冲区
 * @param[in] len       数据长度
 * @param[in] user_data 用户自定义上下文（由 register_rx_callback 传入）
 *
 * @warning 【ISR 安全契约 — 违反将导致系统崩溃】
 *          1. 此回调在 UART ISR 或驱动轮询任务中执行。
 *          2. 【严禁】调用任何阻塞 API（delay、mutex_lock、malloc 等）。
 *          3. 【严禁】在回调中调用本模块的控制面 API。
 *          4. 推荐做法：将数据复制到应用层缓冲区或通过消息队列发送给任务。
 *
 * @note  此回调仅在 DAL_WIFI_DATA_PATH_RAW_SERIAL 模式下有效。
 *        HOST_STACK 模式下数据由协议栈接管，此回调不会被触发。
 */
typedef void (*dal_wifi_rx_callback_t)(dal_wifi_dev_t *dev,
                                       const uint8_t *data,
                                       uint32_t len,
                                       void *user_data);

/* ========================================================================== */
/*                        设备操作集（DRV层核心契约）                             */
/* ========================================================================== */

/**
 * @brief Wi-Fi 硬件抽象操作接口
 * @note  DRV 层实现者需根据芯片能力选择性实现接口。
 *        不适用的接口应置为 NULL，DAL 层将返回 DAL_ERR_NOT_SUPPORTED。
 */
typedef struct {
    /* --- 生命周期管理 --- */
    dal_err_t (*init)(dal_wifi_dev_t *dev);
    dal_err_t (*deinit)(dal_wifi_dev_t *dev);

    /* --- 模式与射频控制 --- */
    dal_err_t (*set_mode)(dal_wifi_dev_t *dev, dal_wifi_mode_t mode);
    dal_err_t (*get_mode)(dal_wifi_dev_t *dev, dal_wifi_mode_t *mode);
    dal_err_t (*set_rf_power)(dal_wifi_dev_t *dev, bool on);

    /* --- STA 模式操作 --- */
    dal_err_t (*scan_start)(dal_wifi_dev_t *dev);
    dal_err_t (*get_scan_result)(dal_wifi_dev_t *dev,
                                 dal_wifi_scan_record_t *records,
                                 uint16_t max_count, uint16_t *actual);
    dal_err_t (*sta_connect)(dal_wifi_dev_t *dev, const dal_wifi_sta_config_t *config);
    dal_err_t (*sta_disconnect)(dal_wifi_dev_t *dev);
    dal_err_t (*sta_get_info)(dal_wifi_dev_t *dev, dal_wifi_sta_info_t *info);

    /* --- AP 模式操作 --- */
    dal_err_t (*ap_start)(dal_wifi_dev_t *dev, const dal_wifi_ap_config_t *config);
    dal_err_t (*ap_stop)(dal_wifi_dev_t *dev);

    /* --- 状态与诊断 --- */
    dal_err_t (*get_state)(dal_wifi_dev_t *dev, dal_wifi_state_t *state);
    dal_err_t (*get_mac)(dal_wifi_dev_t *dev, dal_wifi_mac_t *mac);
    dal_err_t (*get_info)(dal_wifi_dev_t *dev, uint32_t *capability,
                          uint8_t *max_sta);

    /* ================================================================ */
    /* 数据面接口（根据数据路径类型选择性实现）                            */
    /* ================================================================ */

    /**
     * @brief 获取底层网络接口句柄（供网络协议栈使用）
     * @note  【HOST_STACK 模式】必须实现，返回 LwIP netif 等句柄。
     *        【RAW_SERIAL 模式】置 NULL，调用返回 NOTSUP。
     */
    dal_err_t (*get_netif_handle)(dal_wifi_dev_t *dev, void **netif_handle);

    /**
     * @brief 发送裸数据（RAW_SERIAL 模式专用）
     * @param dev        设备实例指针
     * @param data       数据缓冲区
     * @param len        数据长度（字节）
     * @param timeout_ms 超时时间 (ms)，0 = 使用默认超时
     *
     * @retval DAL_OK            发送成功
     * @retval DAL_ERR_TIMEOUT   发送超时（串口堵塞）
     * @retval DAL_ERR_NOTSUP    当前模式不支持（HOST_STACK 模式）
     * @retval DAL_ERR_DISCONNECTED  STA 未连接 / 链路断开
     *
     * @note  【仅 RAW_SERIAL 模式有效】
     *        适用于 ESP8266 AT（AT+CIPSEND）等串口透传模组。
     *        调用前需确保 STA 已连接到 AP（可通过 get_state 确认）。
     *        对于 TCP 连接，数据将发送到当前已建立的 Socket；
     *        对于 UDP，需在配置阶段指定目标地址和端口（由 DRV 层管理）。
     */
    dal_err_t (*transmit)(dal_wifi_dev_t *dev, const uint8_t *data,
                          uint32_t len, uint32_t timeout_ms);

    /**
     * @brief 注册数据接收回调（RAW_SERIAL 模式专用）
     * @param dev        设备实例指针
     * @param cb         接收回调函数（NULL = 注销）
     * @param user_data  用户自定义上下文（透传给回调）
     *
     * @note  【仅 RAW_SERIAL 模式有效】
     *        用于接收 ESP8266 AT 的 +IPD 异步数据回调。
     *        回调在 UART ISR 或驱动轮询任务中执行，必须遵守 ISR 安全契约。
     *        若 cb 为 NULL，停止接收数据（底层可关闭接收通道以节省 CPU）。
     *
     * @note  若设备已注册且 data_path == RAW_SERIAL，但未调用此接口，
     *        接收到的数据将被丢弃（驱动不触发回调）。
     */
    void (*register_rx_callback)(dal_wifi_dev_t *dev,
                                 dal_wifi_rx_callback_t cb,
                                 void *user_data);

} dal_wifi_ops_t;

/* ========================================================================== */
/*                       设备实例结构体                                          */
/* ========================================================================== */

/**
 * @brief Wi-Fi 设备实例描述符
 *
 * @note 实例化要求：
 *       - 必须是全局或静态存储期（不允许栈上分配）
 *       - name 必须为静态字符串常量，框架仅保存指针不复制
 *       - 注册前需填充：name, ops, drv_priv
 *       - 若需要自定义数据路径，可在注册前填充 config
 *       - 其余字段由 DAL 框架管理，DRV/SVC 层禁止直接修改
 */
struct dal_wifi_dev {
    /* === 由 BSP 层在注册前填充 === */
    const char             *name;      ///< 全局唯一设备标识名（如 "wlan0"）
    const dal_wifi_ops_t   *ops;       ///< 硬件操作集指针
    void                   *drv_priv;  ///< DRV 层私有数据

    /**
     * @brief 设备初始化配置（可选）
     * @note  若为 NULL，默认使用 HOST_STACK 模式（向后兼容）。
     *        若需使用 RAW_SERIAL 模式，必须在注册前填充此结构体。
     */
    const dal_wifi_config_t *config;

    /* === 由 DAL 框架管理，DRV/SVC 层禁止直接修改 === */
    bool                      initialized;     ///< 硬件是否已初始化
    dal_wifi_event_callback_t event_cb;        ///< 事件回调函数
    void                     *event_cb_data;   ///< 事件回调用户上下文

    /**
     * @brief 数据路径类型（由框架根据 config 自动设置）
     * @note  只读，DRV 层可通过此字段判断应使用哪种数据面接口。
     */
    dal_wifi_data_path_t      data_path;
};

/* ========================================================================== */
/*                     DAL 框架公共 API                                         */
/* ========================================================================== */

/** @brief 注册 Wi-Fi 设备实例到 DAL 框架 */
dal_err_t dal_wifi_register(dal_wifi_dev_t *dev);

/**
 * @brief 从 DAL 框架注销 Wi-Fi 设备实例
 * @note 【强制注销语义】同其他 DAL 模块，注销时会自动断开连接/停止 AP。
 */
dal_err_t dal_wifi_unregister(dal_wifi_dev_t *dev);

/** @brief 按名称查找已注册的 Wi-Fi 设备实例 */
dal_wifi_dev_t* dal_wifi_get_dev(const char *name);

/** @brief 获取已注册的 Wi-Fi 设备总数 */
uint32_t dal_wifi_get_count(void);

/** @brief 按逻辑索引获取已注册的 Wi-Fi 设备实例 */
dal_wifi_dev_t* dal_wifi_get_dev_by_index(uint32_t index);

/** @brief 初始化 Wi-Fi 设备硬件（加载固件、初始化射频） */
dal_err_t dal_wifi_init(dal_wifi_dev_t *dev);

/** @brief 反初始化 Wi-Fi 设备硬件（幂等） */
dal_err_t dal_wifi_deinit(dal_wifi_dev_t *dev);

/** @brief 设置工作模式 */
dal_err_t dal_wifi_set_mode(dal_wifi_dev_t *dev, dal_wifi_mode_t mode);

/** @brief 获取当前工作模式 */
dal_err_t dal_wifi_get_mode(dal_wifi_dev_t *dev, dal_wifi_mode_t *mode);

/** @brief 开启/关闭射频 */
dal_err_t dal_wifi_set_rf_power(dal_wifi_dev_t *dev, bool on);

/** @brief 启动 Wi-Fi 扫描（异步，完成后触发 EVT_SCAN_DONE） */
dal_err_t dal_wifi_scan_start(dal_wifi_dev_t *dev);

/**
 * @brief 获取扫描结果
 * @param records    [出参] 结果数组缓冲区
 * @param max_count  缓冲区最大容量
 * @param actual     [出参] 实际获取的条目数
 *
 * @note  若 actual 达到 max_count，且扫描结果总数（EVT_SCAN_DONE 中的 count）
 *        大于 max_count，可多次调用本接口获取剩余结果。
 *        具体分页行为由 DRV 层实现决定，建议 DRV 层支持通过内部索引连续读取。
 */
dal_err_t dal_wifi_get_scan_result(dal_wifi_dev_t *dev,
                                   dal_wifi_scan_record_t *records,
                                   uint16_t max_count, uint16_t *actual);

/** @brief 连接到 AP（异步，成功触发 EVT_STA_CONNECTED，失败触发 DISCONNECTED） */
dal_err_t dal_wifi_sta_connect(dal_wifi_dev_t *dev,
                               const dal_wifi_sta_config_t *config);

/** @brief 断开与 AP 的连接（异步，完成后触发 EVT_STA_DISCONNECTED） */
dal_err_t dal_wifi_sta_disconnect(dal_wifi_dev_t *dev);

/** @brief 获取 STA 模式当前连接信息 */
dal_err_t dal_wifi_sta_get_info(dal_wifi_dev_t *dev, dal_wifi_sta_info_t *info);

/** @brief 启动 AP（异步，成功后触发 EVT_AP_STARTED） */
dal_err_t dal_wifi_ap_start(dal_wifi_dev_t *dev,
                            const dal_wifi_ap_config_t *config);

/** @brief 停止 AP（异步，完成后触发 EVT_AP_STOPPED） */
dal_err_t dal_wifi_ap_stop(dal_wifi_dev_t *dev);

/** @brief 获取当前运行状态 */
dal_err_t dal_wifi_get_state(dal_wifi_dev_t *dev, dal_wifi_state_t *state);

/** @brief 获取设备 MAC 地址 */
dal_err_t dal_wifi_get_mac(dal_wifi_dev_t *dev, dal_wifi_mac_t *mac);

/** @brief 获取 Wi-Fi 硬件参数与能力标志 */
dal_err_t dal_wifi_get_info(dal_wifi_dev_t *dev, uint32_t *capability,
                            uint8_t *max_sta);

/** @brief 注册异步事件回调 */
dal_err_t dal_wifi_set_event_callback(dal_wifi_dev_t *dev,
                                      dal_wifi_event_callback_t cb,
                                      void *user_data);

/* ========================================================================== */
/*                       数据面接口（RAW_SERIAL 模式）                          */
/* ========================================================================== */

/**
 * @brief 发送裸数据（RAW_SERIAL 模式专用）
 * @param dev        设备实例指针
 * @param data       数据缓冲区
 * @param len        数据长度（字节）
 * @param timeout_ms 超时时间 (ms)，0 = 使用默认超时
 *
 * @retval DAL_OK            发送成功
 * @retval DAL_ERR_TIMEOUT   发送超时
 * @retval DAL_ERR_NOTSUP    当前模式不支持（HOST_STACK 模式）
 * @retval DAL_ERR_DISCONNECTED  STA 未连接
 * @retval DAL_ERR_PARAM_INVALID 参数无效
 *
 * @note  此接口仅在 DAL_WIFI_DATA_PATH_RAW_SERIAL 模式下可用。
 *        调用前建议通过 get_info() 确认 DAL_WIFI_CAP_RAW_DATA_TX 标志。
 */
dal_err_t dal_wifi_transmit(dal_wifi_dev_t *dev, const uint8_t *data,
                            uint32_t len, uint32_t timeout_ms);

/**
 * @brief 注册数据接收回调（RAW_SERIAL 模式专用）
 * @param dev        设备实例指针
 * @param cb         接收回调函数（NULL = 注销）
 * @param user_data  用户自定义上下文（透传给回调）
 *
 * @note  此接口仅在 DAL_WIFI_DATA_PATH_RAW_SERIAL 模式下有效。
 *        若设备为 HOST_STACK 模式，此函数不生效（接收由协议栈处理）。
 *
 * @note  回调在 ISR 或驱动任务上下文中执行，须遵守 ISR 安全契约。
 */
dal_err_t dal_wifi_register_rx_callback(dal_wifi_dev_t *dev,
                                        dal_wifi_rx_callback_t cb,
                                        void *user_data);

/**
 * @brief 获取底层网络接口句柄（供协议栈使用）
 * @note  【HOST_STACK 模式】返回有效句柄。
 *        【RAW_SERIAL 模式】返回 NULL。
 */
dal_err_t dal_wifi_get_netif_handle(dal_wifi_dev_t *dev, void **netif_handle);

/* ========================================================================== */
/*                   底层通知函数（专供 BSP/DRV 层调用）                        */
/* ========================================================================== */

/**
 * @brief 通知 Wi-Fi 事件（供 BSP/DRV 层在内部任务或中断中调用）
 * @note 此函数【专供底层驱动调用】，应用层严禁调用。
 */
void dal_wifi_notify_event(dal_wifi_dev_t *dev, uint32_t event,
                           const dal_wifi_event_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __DAL_WIFI_H__ */