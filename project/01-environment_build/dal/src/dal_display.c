/**
 * @file    dal_display.c
 * @brief   显示设备 DAL 层实现 v1.0
 * @note    依赖 dal_registry 提供的通用注册表管理。
 *          本文件【不提供】内部互斥，调用者需自行保证线程安全。
 * 
 * @author xserein
 * @version v1.0
 */
#include "dal_display.h"
#include "dal_registry.h"
#include <string.h>

/* ========================================================================== */
/*                               配置宏                                         */
/* ========================================================================== */

#ifndef DAL_DISPLAY_REGISTRY_SIZE
#define DAL_DISPLAY_REGISTRY_SIZE 4   /**< 默认最大显示设备数 */
#endif

/* ========================================================================== */
/*                              内部数据                                        */
/* ========================================================================== */

static dal_reg_entry_t s_disp_reg_buf[DAL_DISPLAY_REGISTRY_SIZE];
static dal_registry_t  s_disp_reg;
static bool            s_disp_reg_inited = false;

/* ========================================================================== */
/*                            内部辅助函数                                       */
/* ========================================================================== */

static dal_err_t _disp_reg_ensure_init(void)
{
    if (!s_disp_reg_inited) {
        dal_err_t err = dal_registry_init(&s_disp_reg, s_disp_reg_buf,
                                          DAL_DISPLAY_REGISTRY_SIZE);
        if (err != DAL_OK) {
            return err;
        }
        s_disp_reg_inited = true;
    }
    return DAL_OK;
}

/**
 * @brief 校验设备实例是否确实存在于注册表中
 * @note  ISR 安全：dal_registry_find_ops 为纯只读静态数组遍历。
 */
static bool _disp_dev_is_registered(const dal_display_dev_t *dev)
{
    if (!s_disp_reg_inited || dev == NULL || dev->name == NULL) {
        return false;
    }

    void *ctx = NULL;
    const void *ops = dal_registry_find_ops(&s_disp_reg, dev->name, &ctx);
    return (ops != NULL && ctx == (void *)dev);
}

/* ========================================================================== */
/*                             注册 / 注销 / 查找                                */
/* ========================================================================== */

dal_err_t dal_display_register(dal_display_dev_t *dev)
{
    if (dev == NULL || dev->name == NULL || dev->name[0] == '\0' || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    dal_err_t err = _disp_reg_ensure_init();
    if (err != DAL_OK) {
        return err;
    }

    /* 查重：同名设备已存在 */
    void *ctx = NULL;
    if (dal_registry_find_ops(&s_disp_reg, dev->name, &ctx) != NULL) {
        return DAL_ERR_DUPLICATE;
    }

    /* 重置 DAL 内部状态（防止 re-register 后残留旧回调） */
    dev->initialized   = false;
    dev->event_cb      = NULL;
    dev->event_cb_data = NULL;

    return dal_registry_register(&s_disp_reg, dev->name,
                                 (const void *)dev->ops, (void *)dev);
}

dal_err_t dal_display_unregister(dal_display_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!s_disp_reg_inited || !_disp_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    /*
     * 【强制注销语义】
     * 即使 deinit 返回错误，设备仍会从注册表中移除。
     * 调用者无法通过返回值感知底层 deinit 是否失败。
     */
    if (dev->initialized) {
        /* 先禁用事件中断 */
        if (dev->ops->set_event_irq_enable != NULL) {
            (void)dev->ops->set_event_irq_enable(dev, false);
        }

        /* 尝试 deinit，忽略错误码 */
        if (dev->ops->deinit != NULL) {
            (void)dev->ops->deinit(dev);
        }
        dev->initialized = false;
    }

    /* 注销回调，防止悬垂指针 */
    dev->event_cb      = NULL;
    dev->event_cb_data = NULL;

    return dal_registry_unregister(&s_disp_reg, dev->name);
}

dal_display_dev_t* dal_display_get_dev(const char *name)
{
    if (name == NULL || name[0] == '\0' || !s_disp_reg_inited) {
        return NULL;
    }

    void *ctx = NULL;
    dal_registry_find_ops(&s_disp_reg, name, &ctx);
    return (dal_display_dev_t *)ctx;
}

uint32_t dal_display_get_count(void)
{
    return s_disp_reg_inited ? (uint32_t)dal_registry_count(&s_disp_reg) : 0;
}

dal_display_dev_t* dal_display_get_dev_by_index(uint32_t index)
{
    if (!s_disp_reg_inited) {
        return NULL;
    }

    uint32_t valid_idx = 0;
    uint16_t cap = s_disp_reg.capacity;

    for (uint16_t i = 0; i < cap; i++) {
        const char *name = NULL;
        void *ctx = NULL;

        dal_err_t err = dal_registry_get_entry(&s_disp_reg, i, &name, NULL, &ctx);
        if (err != DAL_OK || name == NULL) {
            continue;
        }

        if (valid_idx == index) {
            return (dal_display_dev_t *)ctx;
        }
        valid_idx++;
    }

    return NULL;
}

/* ========================================================================== */
/*                           生命周期管理                                        */
/* ========================================================================== */

dal_err_t dal_display_init(dal_display_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev)) {
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

dal_err_t dal_display_deinit(dal_display_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    /* 幂等：未初始化直接返回成功 */
    if (!dev->initialized) {
        return DAL_OK;
    }

    /* 先禁用事件中断 */
    if (dev->ops->set_event_irq_enable != NULL) {
        (void)dev->ops->set_event_irq_enable(dev, false);
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
/*                            诊断与测试                                         */
/* ========================================================================== */

dal_err_t dal_display_selftest(dal_display_dev_t *dev,
                               dal_display_selftest_result_t *result)
{
    if (dev == NULL || result == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops == NULL || dev->ops->selftest == NULL) {
        *result = DAL_DISPLAY_SELFTEST_NOT_IMPL;
        return DAL_OK;
    }
    return dev->ops->selftest(dev, result);
}

/* ========================================================================== */
/*                            核心绘图接口                                       */
/* ========================================================================== */

dal_err_t dal_display_draw(dal_display_dev_t *dev,
                           const dal_display_rect_t *rect,
                           const uint8_t *data, uint32_t len)
{
    if (dev == NULL || rect == NULL || data == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->draw == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    /*
     * 【越界检查责任说明】
     * DAL 框架不做坐标/尺寸裁剪，rect 有效性校验完全由 DRV 层负责。
     * DRV 层应在 draw 实现中检查 rect 是否超出当前逻辑分辨率，
     * 越界时返回 DAL_ERR_PARAM_INVALID。
     */
    return dev->ops->draw(dev, rect, data, len);
}

dal_err_t dal_display_flush(dal_display_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->flush == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    /*
     * 【并发互斥契约】
     * 若异步刷新正在进行，DRV 层的 flush 实现必须返回 DAL_ERR_BUSY，
     * 禁止阻塞等待。DAL 层在此不做额外状态检查，将并发判断权
     * 完全交给 DRV 层（DRV 层拥有最准确的硬件状态）。
     */
    return dev->ops->flush(dev);
}

dal_err_t dal_display_flush_async(dal_display_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->flush_async == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    /*
     * 【并发互斥契约】
     * 若上一次异步刷新尚未完成，DRV 层应返回 DAL_ERR_BUSY。
     */
    return dev->ops->flush_async(dev);
}

/* ========================================================================== */
/*                        段码屏/字符屏专用接口                                  */
/* ========================================================================== */

dal_err_t dal_display_set_segment(dal_display_dev_t *dev,
                                  uint32_t index, uint32_t value)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_segment == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_segment(dev, index, value);
}

/* ========================================================================== */
/*                          控制与配置接口                                       */
/* ========================================================================== */

dal_err_t dal_display_set_backlight(dal_display_dev_t *dev, uint8_t brightness)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_backlight == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_backlight(dev, brightness);
}

dal_err_t dal_display_set_rotation(dal_display_dev_t *dev,
                                   dal_display_rotation_t rotation)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_rotation == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_rotation(dev, rotation);
}

dal_err_t dal_display_set_power(dal_display_dev_t *dev, bool on)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_power == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_power(dev, on);
}

/* ========================================================================== */
/*                            状态查询接口                                       */
/* ========================================================================== */

dal_err_t dal_display_get_state(dal_display_dev_t *dev,
                                dal_display_state_t *state)
{
    if (dev == NULL || state == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_state(dev, state);
}

dal_err_t dal_display_get_fault(dal_display_dev_t *dev, uint32_t *fault)
{
    if (dev == NULL || fault == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_fault == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_fault(dev, fault);
}

dal_err_t dal_display_get_info(dal_display_dev_t *dev,
                               uint16_t *width, uint16_t *height,
                               dal_display_pixel_fmt_t *pixel_fmt,
                               dal_display_type_t *type,
                               uint32_t *capability)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_info == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_info(dev, width, height, pixel_fmt, type, capability);
}

/* ========================================================================== */
/*                          异步回调管理接口                                     */
/* ========================================================================== */

dal_err_t dal_display_set_event_callback(dal_display_dev_t *dev,
                                         dal_display_event_callback_t cb,
                                         void *user_data)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    dev->event_cb      = cb;
    dev->event_cb_data = user_data;
    return DAL_OK;
}

dal_err_t dal_display_set_event_irq_enable(dal_display_dev_t *dev, bool enable)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_disp_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops == NULL || dev->ops->set_event_irq_enable == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_event_irq_enable(dev, enable);
}

/* ========================================================================== */
/*                    BSP 层 ISR 调用接口（中断上下文）                           */
/* ========================================================================== */

void dal_display_notify_event(dal_display_dev_t *dev, uint32_t event)
{
    if (dev == NULL || dev->event_cb == NULL) {
        return;
    }

    /*
     * ISR 安全说明：_disp_dev_is_registered 内部调用 dal_registry_find_ops，
     * 该函数仅遍历静态数组、不持锁，可在中断/DMA完成上下文安全调用。
     */
    if (!_disp_dev_is_registered(dev)) {
        return;
    }

    dev->event_cb(dev, event, dev->event_cb_data);
}