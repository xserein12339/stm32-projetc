/**
 * @file    dal_led.c
 * @brief   DAL LED 设备管理框架实现 
 * 
 * @author xserein
 * @version v1.0
 */
#include "dal_led.h"
#include "dal_registry.h"
#include <string.h>

/* ========================================================================== */
/*                               配置宏                                        */
/* ========================================================================== */

#ifndef DAL_LED_REGISTRY_SIZE
#define DAL_LED_REGISTRY_SIZE 8
#endif

/* ========================================================================== */
/*                              内部数据                                       */
/* ========================================================================== */

static dal_reg_entry_t s_led_reg_buf[DAL_LED_REGISTRY_SIZE];
static dal_registry_t  s_led_reg;
static bool            s_led_reg_inited = false;

/* ========================================================================== */
/*                            内部辅助函数                                     */
/* ========================================================================== */

static dal_err_t _led_reg_ensure_init(void)
{
    if (!s_led_reg_inited) {
        dal_err_t err = dal_registry_init(&s_led_reg, s_led_reg_buf,
                                          DAL_LED_REGISTRY_SIZE);
        if (err != DAL_OK) {
            return err;
        }
        s_led_reg_inited = true;
    }
    return DAL_OK;
}

static bool _led_dev_is_registered(const dal_led_dev_t *dev)
{
    if (!s_led_reg_inited || dev == NULL || dev->name == NULL) {
        return false;
    }
    void *ctx = NULL;
    const void *ops = dal_registry_find_ops(&s_led_reg, dev->name, &ctx);
    return (ops != NULL && ctx == (void *)dev);
}

/* ========================================================================== */
/*                             公共 API 实现                                   */
/* ========================================================================== */

dal_err_t dal_led_register(dal_led_dev_t *dev)
{
    if (dev == NULL || dev->name == NULL || dev->name[0] == '\0') {
        return DAL_ERR_PARAM_INVALID;
    }
    if (dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    dal_err_t err = _led_reg_ensure_init();
    if (err != DAL_OK) {
        return err;
    }

    /* 查重 */
    void *ctx = NULL;
    if (dal_registry_find_ops(&s_led_reg, dev->name, &ctx) != NULL) {
        return DAL_ERR_DUPLICATE;
    }

    dev->initialized = false;
    return dal_registry_register(&s_led_reg, dev->name,
                                 (const void *)dev->ops, (void *)dev);
}

dal_err_t dal_led_unregister(dal_led_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!s_led_reg_inited) {
        return DAL_ERR_NOT_READY;
    }
    if (!_led_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->initialized) {
        return DAL_ERR_BUSY;   ///< 需先 deinit
    }

    return dal_registry_unregister(&s_led_reg, dev->name);
}

dal_led_dev_t* dal_led_get_dev(const char *name)
{
    if (name == NULL || name[0] == '\0') {
        return NULL;
    }
    if (!s_led_reg_inited) {
        return NULL;
    }

    void *ctx = NULL;
    dal_registry_find_ops(&s_led_reg, name, &ctx);
    return (dal_led_dev_t *)ctx;
}

dal_err_t dal_led_init(dal_led_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_led_dev_is_registered(dev)) {
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

dal_err_t dal_led_deinit(dal_led_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_led_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    if (!dev->initialized) {
        return DAL_ERR_NOT_READY;
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

dal_err_t dal_led_selftest(dal_led_dev_t *dev, dal_led_selftest_result_t *result)
{
    if (dev == NULL || result == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (dev->ops == NULL) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->selftest == NULL) {
        *result = DAL_LED_SELFTEST_NOT_IMPL;
        return DAL_OK;
    }
    return dev->ops->selftest(dev, result);
}

dal_err_t dal_led_set_state(dal_led_dev_t *dev, dal_led_state_t state)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (state != DAL_LED_ON && state != DAL_LED_OFF) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_led_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    if (!dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    return dev->ops->set_state(dev, state);
}

dal_err_t dal_led_get_state(dal_led_dev_t *dev, dal_led_state_t *state)
{
    if (dev == NULL || state == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_led_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    if (!dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    return dev->ops->get_state(dev, state);
}