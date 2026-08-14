/**
 * @file    dal_wifi.c
 * @brief   Wi-Fi 设备 DAL 层实现 v1.1
 * @note    依赖 dal_registry 提供的通用注册表管理。
 *          本文件【不提供】内部互斥，调用者需自行保证线程安全。
 *          支持 HOST_STACK 与 RAW_SERIAL 两种数据路径模式。
 * 
 * @author xserein
 * @version v1.1
 */
#include "dal_wifi.h"
#include "dal_registry.h"
#include <string.h>

/* ========================================================================== */
/*                               配置宏                                         */
/* ========================================================================== */

#ifndef DAL_WIFI_REGISTRY_SIZE
#define DAL_WIFI_REGISTRY_SIZE 2   /**< 默认最大 Wi-Fi 设备数 */
#endif

/* ========================================================================== */
/*                              内部数据                                        */
/* ========================================================================== */

static dal_reg_entry_t s_wifi_reg_buf[DAL_WIFI_REGISTRY_SIZE];
static dal_registry_t  s_wifi_reg;
static bool            s_wifi_reg_inited = false;

/* ========================================================================== */
/*                            内部辅助函数                                       */
/* ========================================================================== */

static dal_err_t _wifi_reg_ensure_init(void)
{
    if (!s_wifi_reg_inited) {
        dal_err_t err = dal_registry_init(&s_wifi_reg, s_wifi_reg_buf,
                                          DAL_WIFI_REGISTRY_SIZE);
        if (err != DAL_OK) {
            return err;
        }
        s_wifi_reg_inited = true;
    }
    return DAL_OK;
}

/**
 * @brief 校验设备实例是否确实存在于注册表中
 * @note  ISR 安全：dal_registry_find_ops 为纯只读静态数组遍历。
 */
static bool _wifi_dev_is_registered(const dal_wifi_dev_t *dev)
{
    if (!s_wifi_reg_inited || dev == NULL || dev->name == NULL) {
        return false;
    }

    void *ctx = NULL;
    const void *ops = dal_registry_find_ops(&s_wifi_reg, dev->name, &ctx);
    return (ops != NULL && ctx == (void *)dev);
}

/* ========================================================================== */
/*                             注册 / 注销 / 查找                                */
/* ========================================================================== */

dal_err_t dal_wifi_register(dal_wifi_dev_t *dev)
{
    if (dev == NULL || dev->name == NULL || dev->name[0] == '\0' || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    dal_err_t err = _wifi_reg_ensure_init();
    if (err != DAL_OK) {
        return err;
    }

    /* 查重：同名设备已存在 */
    void *ctx = NULL;
    if (dal_registry_find_ops(&s_wifi_reg, dev->name, &ctx) != NULL) {
        return DAL_ERR_DUPLICATE;
    }

    /*
     * 【数据路径类型解析与固化】
     * 根据 config 字段确定数据路径，写入只读字段 data_path。
     * config 为 NULL 时默认为 HOST_STACK（向后兼容 v1.0）。
     */
    if (dev->config != NULL) {
        dev->data_path = dev->config->data_path;
    } else {
        dev->data_path = DAL_WIFI_DATA_PATH_HOST_STACK;
    }

    /* 重置 DAL 内部状态（防止 re-register 后残留旧回调） */
    dev->initialized   = false;
    dev->event_cb      = NULL;
    dev->event_cb_data = NULL;

    return dal_registry_register(&s_wifi_reg, dev->name,
                                 (const void *)dev->ops, (void *)dev);
}

dal_err_t dal_wifi_unregister(dal_wifi_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!s_wifi_reg_inited || !_wifi_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    /*
     * 【强制注销语义】
     * 即使 deinit 返回错误，设备仍会从注册表中移除。
     */
    if (dev->initialized) {
        /* RAW_SERIAL 模式下先注销接收回调，防止注销过程中触发回调 */
        if (dev->data_path == DAL_WIFI_DATA_PATH_RAW_SERIAL &&
            dev->ops->register_rx_callback != NULL) {
            dev->ops->register_rx_callback(dev, NULL, NULL);
        }

        /* 尝试 deinit，忽略错误码 */
        if (dev->ops->deinit != NULL) {
            (void)dev->ops->deinit(dev);
        }
        dev->initialized = false;
    }

    /* 注销所有回调，防止悬垂指针 */
    dev->event_cb      = NULL;
    dev->event_cb_data = NULL;

    return dal_registry_unregister(&s_wifi_reg, dev->name);
}

dal_wifi_dev_t* dal_wifi_get_dev(const char *name)
{
    if (name == NULL || name[0] == '\0' || !s_wifi_reg_inited) {
        return NULL;
    }

    void *ctx = NULL;
    dal_registry_find_ops(&s_wifi_reg, name, &ctx);
    return (dal_wifi_dev_t *)ctx;
}

uint32_t dal_wifi_get_count(void)
{
    return s_wifi_reg_inited ? (uint32_t)dal_registry_count(&s_wifi_reg) : 0;
}

dal_wifi_dev_t* dal_wifi_get_dev_by_index(uint32_t index)
{
    if (!s_wifi_reg_inited) {
        return NULL;
    }

    uint32_t valid_idx = 0;
    uint16_t cap = s_wifi_reg.capacity;

    for (uint16_t i = 0; i < cap; i++) {
        const char *name = NULL;
        void *ctx = NULL;

        dal_err_t err = dal_registry_get_entry(&s_wifi_reg, i, &name, NULL, &ctx);
        if (err != DAL_OK || name == NULL) {
            continue;
        }

        if (valid_idx == index) {
            return (dal_wifi_dev_t *)ctx;
        }
        valid_idx++;
    }

    return NULL;
}

/* ========================================================================== */
/*                           生命周期管理                                        */
/* ========================================================================== */

dal_err_t dal_wifi_init(dal_wifi_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->initialized) {
        return DAL_ERR_ALREADY_INIT;
    }
    if (dev->ops->init == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    dal_err_t err = dev->ops->init(dev);
    if (err == DAL_OK) {
        dev->initialized = true;
    }
    return err;
}

dal_err_t dal_wifi_deinit(dal_wifi_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    /* 幂等：未初始化直接返回成功 */
    if (!dev->initialized) {
        return DAL_OK;
    }

    /* RAW_SERIAL 模式下先注销接收回调 */
    if (dev->data_path == DAL_WIFI_DATA_PATH_RAW_SERIAL &&
        dev->ops->register_rx_callback != NULL) {
        dev->ops->register_rx_callback(dev, NULL, NULL);
    }

    if (dev->ops->deinit == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    dal_err_t err = dev->ops->deinit(dev);
    if (err == DAL_OK) {
        dev->initialized = false;
    }
    return err;
}

/* ========================================================================== */
/*                         模式与射频控制                                        */
/* ========================================================================== */

dal_err_t dal_wifi_set_mode(dal_wifi_dev_t *dev, dal_wifi_mode_t mode)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_mode == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_mode(dev, mode);
}

dal_err_t dal_wifi_get_mode(dal_wifi_dev_t *dev, dal_wifi_mode_t *mode)
{
    if (dev == NULL || mode == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_mode == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_mode(dev, mode);
}

dal_err_t dal_wifi_set_rf_power(dal_wifi_dev_t *dev, bool on)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_rf_power == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_rf_power(dev, on);
}

/* ========================================================================== */
/*                          STA 模式操作                                         */
/* ========================================================================== */

dal_err_t dal_wifi_scan_start(dal_wifi_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->scan_start == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->scan_start(dev);
}

dal_err_t dal_wifi_get_scan_result(dal_wifi_dev_t *dev,
                                   dal_wifi_scan_record_t *records,
                                   uint16_t max_count, uint16_t *actual)
{
    if (dev == NULL || records == NULL || actual == NULL || max_count == 0) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_scan_result == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_scan_result(dev, records, max_count, actual);
}

dal_err_t dal_wifi_sta_connect(dal_wifi_dev_t *dev,
                               const dal_wifi_sta_config_t *config)
{
    if (dev == NULL || config == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->sta_connect == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->sta_connect(dev, config);
}

dal_err_t dal_wifi_sta_disconnect(dal_wifi_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->sta_disconnect == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->sta_disconnect(dev);
}

dal_err_t dal_wifi_sta_get_info(dal_wifi_dev_t *dev, dal_wifi_sta_info_t *info)
{
    if (dev == NULL || info == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->sta_get_info == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->sta_get_info(dev, info);
}

/* ========================================================================== */
/*                          AP 模式操作                                          */
/* ========================================================================== */

dal_err_t dal_wifi_ap_start(dal_wifi_dev_t *dev,
                            const dal_wifi_ap_config_t *config)
{
    if (dev == NULL || config == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->ap_start == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->ap_start(dev, config);
}

dal_err_t dal_wifi_ap_stop(dal_wifi_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->ap_stop == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->ap_stop(dev);
}

/* ========================================================================== */
/*                          状态与诊断                                           */
/* ========================================================================== */

dal_err_t dal_wifi_get_state(dal_wifi_dev_t *dev, dal_wifi_state_t *state)
{
    if (dev == NULL || state == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_state(dev, state);
}

dal_err_t dal_wifi_get_mac(dal_wifi_dev_t *dev, dal_wifi_mac_t *mac)
{
    if (dev == NULL || mac == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_mac == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_mac(dev, mac);
}

dal_err_t dal_wifi_get_info(dal_wifi_dev_t *dev, uint32_t *capability,
                            uint8_t *max_sta)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_info == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_info(dev, capability, max_sta);
}

/* ========================================================================== */
/*                          异步回调管理                                         */
/* ========================================================================== */

dal_err_t dal_wifi_set_event_callback(dal_wifi_dev_t *dev,
                                      dal_wifi_event_callback_t cb,
                                      void *user_data)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    dev->event_cb      = cb;
    dev->event_cb_data = user_data;
    return DAL_OK;
}

/* ========================================================================== */
/*              数据面接口（根据 data_path 分发）                                  */
/* ========================================================================== */

dal_err_t dal_wifi_transmit(dal_wifi_dev_t *dev, const uint8_t *data,
                            uint32_t len, uint32_t timeout_ms)
{
    if (dev == NULL || data == NULL || len == 0) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }

    /* 【数据路径守卫】仅 RAW_SERIAL 模式允许调用 */
    if (dev->data_path != DAL_WIFI_DATA_PATH_RAW_SERIAL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    if (dev->ops->transmit == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    return dev->ops->transmit(dev, data, len, timeout_ms);
}

dal_err_t dal_wifi_register_rx_callback(dal_wifi_dev_t *dev,
                                        dal_wifi_rx_callback_t cb,
                                        void *user_data)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    /* 【数据路径守卫】仅 RAW_SERIAL 模式允许调用 */
    if (dev->data_path != DAL_WIFI_DATA_PATH_RAW_SERIAL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    if (dev->ops->register_rx_callback == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    /* register_rx_callback 返回 void，无错误码可传递 */
    dev->ops->register_rx_callback(dev, cb, user_data);
    return DAL_OK;
}

dal_err_t dal_wifi_get_netif_handle(dal_wifi_dev_t *dev, void **netif_handle)
{
    if (dev == NULL || netif_handle == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_wifi_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }

    /* 【数据路径守卫】仅 HOST_STACK 模式允许调用 */
    if (dev->data_path != DAL_WIFI_DATA_PATH_HOST_STACK) {
        *netif_handle = NULL;
        return DAL_ERR_NOT_SUPPORTED;
    }

    if (dev->ops->get_netif_handle == NULL) {
        *netif_handle = NULL;
        return DAL_ERR_NOT_SUPPORTED;
    }

    return dev->ops->get_netif_handle(dev, netif_handle);
}

/* ========================================================================== */
/*                  底层通知函数（专供 BSP/DRV 层调用）                            */
/* ========================================================================== */

void dal_wifi_notify_event(dal_wifi_dev_t *dev, uint32_t event,
                           const dal_wifi_event_data_t *data)
{
    if (dev == NULL || dev->event_cb == NULL) {
        return;
    }

    /*
     * ISR 安全说明：_wifi_dev_is_registered 内部调用 dal_registry_find_ops，
     * 该函数仅遍历静态数组、不持锁，可在中断/驱动任务上下文安全调用。
     */
    if (!_wifi_dev_is_registered(dev)) {
        return;
    }

    dev->event_cb(dev, event, data, dev->event_cb_data);
}