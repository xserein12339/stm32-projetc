/**
 * @file    dal_encoder.c
 * @brief   编码器设备 DAL 层实现 v1.1
 * @note    依赖 dal_registry 提供的通用注册表管理。
 *          本文件【不提供】内部互斥，调用者需自行保证线程安全。
 * 
 * @author xserein
 * @version v1.1
 */
#include "dal_encoder.h"
#include "dal_registry.h"
#include <string.h>

/* ========================================================================== */
/*                               配置宏                                         */
/* ========================================================================== */

#ifndef DAL_ENCODER_REGISTRY_SIZE
#define DAL_ENCODER_REGISTRY_SIZE 4   /**< 默认最大编码器设备数 */
#endif

/* ========================================================================== */
/*                              内部数据                                        */
/* ========================================================================== */

static dal_reg_entry_t s_enc_reg_buf[DAL_ENCODER_REGISTRY_SIZE];
static dal_registry_t  s_enc_reg;
static bool            s_enc_reg_inited = false;

/* ========================================================================== */
/*                            内部辅助函数                                       */
/* ========================================================================== */

static dal_err_t _enc_reg_ensure_init(void)
{
    if (!s_enc_reg_inited) {
        dal_err_t err = dal_registry_init(&s_enc_reg, s_enc_reg_buf,
                                          DAL_ENCODER_REGISTRY_SIZE);
        if (err != DAL_OK) {
            return err;
        }
        s_enc_reg_inited = true;
    }
    return DAL_OK;
}

/**
 * @brief 校验设备实例是否确实存在于注册表中
 * @note  ISR 安全：dal_registry_find_ops 为纯只读静态数组遍历。
 */
static bool _enc_dev_is_registered(const dal_encoder_dev_t *dev)
{
    if (!s_enc_reg_inited || dev == NULL || dev->name == NULL) {
        return false;
    }

    void *ctx = NULL;
    const void *ops = dal_registry_find_ops(&s_enc_reg, dev->name, &ctx);
    return (ops != NULL && ctx == (void *)dev);
}

/* ========================================================================== */
/*                             注册 / 注销 / 查找                                */
/* ========================================================================== */

dal_err_t dal_encoder_register(dal_encoder_dev_t *dev)
{
    if (dev == NULL || dev->name == NULL || dev->name[0] == '\0' || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    dal_err_t err = _enc_reg_ensure_init();
    if (err != DAL_OK) {
        return err;
    }

    /* 查重：同名设备已存在 */
    void *ctx = NULL;
    if (dal_registry_find_ops(&s_enc_reg, dev->name, &ctx) != NULL) {
        return DAL_ERR_DUPLICATE;
    }

    /* 重置 DAL 内部状态（防止 re-register 后残留旧回调） */
    dev->initialized     = false;
    dev->event_cb        = NULL;
    dev->event_cb_data   = NULL;

    return dal_registry_register(&s_enc_reg, dev->name,
                                 (const void *)dev->ops, (void *)dev);
}

dal_err_t dal_encoder_unregister(dal_encoder_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!s_enc_reg_inited || !_enc_dev_is_registered(dev)) {
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

    return dal_registry_unregister(&s_enc_reg, dev->name);
}

dal_encoder_dev_t* dal_encoder_get_dev(const char *name)
{
    if (name == NULL || name[0] == '\0' || !s_enc_reg_inited) {
        return NULL;
    }

    void *ctx = NULL;
    dal_registry_find_ops(&s_enc_reg, name, &ctx);
    return (dal_encoder_dev_t *)ctx;
}

uint32_t dal_encoder_get_count(void)
{
    return s_enc_reg_inited ? (uint32_t)dal_registry_count(&s_enc_reg) : 0;
}

dal_encoder_dev_t* dal_encoder_get_dev_by_index(uint32_t index)
{
    if (!s_enc_reg_inited) {
        return NULL;
    }

    uint32_t valid_idx = 0;
    uint16_t cap = s_enc_reg.capacity;

    for (uint16_t i = 0; i < cap; i++) {
        const char *name = NULL;
        void *ctx = NULL;

        dal_err_t err = dal_registry_get_entry(&s_enc_reg, i, &name, NULL, &ctx);
        if (err != DAL_OK || name == NULL) {
            continue;
        }

        if (valid_idx == index) {
            return (dal_encoder_dev_t *)ctx;
        }
        valid_idx++;
    }

    return NULL;
}

/* ========================================================================== */
/*                           生命周期管理                                        */
/* ========================================================================== */

dal_err_t dal_encoder_init(dal_encoder_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev)) {
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

dal_err_t dal_encoder_deinit(dal_encoder_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev)) {
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

dal_err_t dal_encoder_selftest(dal_encoder_dev_t *dev,
                               dal_encoder_selftest_result_t *result)
{
    if (dev == NULL || result == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops == NULL || dev->ops->selftest == NULL) {
        *result = DAL_ENCODER_SELFTEST_NOT_IMPL;
        return DAL_OK;
    }
    return dev->ops->selftest(dev, result);
}

/* ========================================================================== */
/*                            核心读取接口                                       */
/* ========================================================================== */

dal_err_t dal_encoder_get_position(dal_encoder_dev_t *dev, int32_t *position)
{
    if (dev == NULL || position == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_position == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_position(dev, position);
}

dal_err_t dal_encoder_get_angle(dal_encoder_dev_t *dev, uint32_t *angle)
{
    if (dev == NULL || angle == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_angle == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_angle(dev, angle);
}

dal_err_t dal_encoder_get_velocity(dal_encoder_dev_t *dev, int32_t *velocity)
{
    if (dev == NULL || velocity == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_velocity == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_velocity(dev, velocity);
}

dal_err_t dal_encoder_get_direction(dal_encoder_dev_t *dev,
                                    dal_encoder_dir_t *direction)
{
    if (dev == NULL || direction == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_direction == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_direction(dev, direction);
}

/* ========================================================================== */
/*                          控制与校准接口                                       */
/* ========================================================================== */

dal_err_t dal_encoder_reset(dal_encoder_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->reset == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->reset(dev);
}

dal_err_t dal_encoder_find_zero(dal_encoder_dev_t *dev, uint32_t timeout_ms)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->find_zero == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->find_zero(dev, timeout_ms);
}

/* ========================================================================== */
/*                            状态查询接口                                       */
/* ========================================================================== */

dal_err_t dal_encoder_get_state(dal_encoder_dev_t *dev,
                                dal_encoder_state_t *state)
{
    if (dev == NULL || state == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_state(dev, state);
}

dal_err_t dal_encoder_get_fault(dal_encoder_dev_t *dev, uint32_t *fault)
{
    if (dev == NULL || fault == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_fault == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_fault(dev, fault);
}

dal_err_t dal_encoder_get_info(dal_encoder_dev_t *dev, uint32_t *resolution,
                               dal_encoder_type_t *type, uint32_t *capability)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_info == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_info(dev, resolution, type, capability);
}

/* ========================================================================== */
/*                          异步回调管理接口                                     */
/* ========================================================================== */

dal_err_t dal_encoder_set_event_callback(dal_encoder_dev_t *dev,
                                         dal_encoder_event_callback_t cb,
                                         void *user_data)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    dev->event_cb      = cb;
    dev->event_cb_data = user_data;
    return DAL_OK;
}

dal_err_t dal_encoder_set_event_irq_enable(dal_encoder_dev_t *dev, bool enable)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_enc_dev_is_registered(dev) || !dev->initialized) {
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

void dal_encoder_notify_event(dal_encoder_dev_t *dev, uint32_t event)
{
    if (dev == NULL || dev->event_cb == NULL) {
        return;
    }

    /*
     * ISR 安全说明：_enc_dev_is_registered 内部调用 dal_registry_find_ops，
     * 该函数仅遍历静态数组、不持锁，可在中断上下文安全调用。
     */
    if (!_enc_dev_is_registered(dev)) {
        return;
    }

    dev->event_cb(dev, event, dev->event_cb_data);
}