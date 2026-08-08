/**
 * @file dal_key.c
 * @brief 按键设备 DAL 层实现
 * @note  依赖 dal_registry 提供的通用注册表管理。
 *        本文件不提供内部互斥，register/unregister 需在任务上下文调用。
 * 
 * @author xserein
 * @version v1.0
 */
#include "dal_key.h"
#include "dal_registry.h"
#include <string.h>

/* ========================================================================== */
/*                               配置宏                                       */
/* ========================================================================== */

#ifndef DAL_KEY_REGISTRY_SIZE
#define DAL_KEY_REGISTRY_SIZE 4   /**< 默认最大按键设备数，可按项目调整 */
#endif

/* ========================================================================== */
/*                              内部数据                                      */
/* ========================================================================== */

static dal_reg_entry_t s_key_reg_buf[DAL_KEY_REGISTRY_SIZE];
static dal_registry_t  s_key_reg;
static bool            s_key_reg_inited = false;

/* ========================================================================== */
/*                            内部辅助函数                                     */
/* ========================================================================== */

/**
 * @brief 惰性初始化全局按键注册表
 */
static dal_err_t _key_reg_ensure_init(void)
{
    if (!s_key_reg_inited) {
        dal_err_t err = dal_registry_init(&s_key_reg, s_key_reg_buf,
                                          DAL_KEY_REGISTRY_SIZE);
        if (err != DAL_OK) {
            return err;
        }
        s_key_reg_inited = true;
    }
    return DAL_OK;
}

/**
 * @brief 校验设备实例是否确实存在于注册表中
 * @note  防止传入一个野指针或已注销的 dev 导致后续操作踩内存。
 */
static bool _key_dev_is_registered(const dal_key_dev_t *dev)
{
    if (!s_key_reg_inited || dev == NULL || dev->name == NULL) {
        return false;
    }

    void *ctx = NULL;
    const void *ops = dal_registry_find_ops(&s_key_reg, dev->name, &ctx);
    return (ops != NULL && ctx == (void *)dev);
}

/* ========================================================================== */
/*                             公共 API 实现                                   */
/* ========================================================================== */

dal_err_t dal_key_register(dal_key_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (dev->name == NULL || dev->name[0] == '\0') {
        return DAL_ERR_PARAM_INVALID;
    }
    if (dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    dal_err_t err = _key_reg_ensure_init();
    if (err != DAL_OK) {
        return err;
    }

    /* 查重：同名设备已存在 */
    void *ctx = NULL;
    if (dal_registry_find_ops(&s_key_reg, dev->name, &ctx) != NULL) {
        return DAL_ERR_DUPLICATE;
    }

    /* 重置 DAL 内部状态 */
    dev->initialized = false;

    /* 注册到通用注册表：ops 存驱动操作集，ctx 存设备实例指针 */
    return dal_registry_register(&s_key_reg, dev->name,
                                 (const void *)dev->ops, (void *)dev);
}

dal_err_t dal_key_unregister(dal_key_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!s_key_reg_inited) {
        return DAL_ERR_NOT_READY;
    }

    /* 确认该 dev 确实在注册表中 */
    if (!_key_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    /* 若硬件仍处初始化状态，先自动反初始化 */
    if (dev->initialized) {
        dal_err_t err = dal_key_deinit(dev);
        if (err != DAL_OK) {
            return DAL_ERR_BUSY;   ///< 硬件拒绝释放，不能强制注销 
        }
    }

    return dal_registry_unregister(&s_key_reg, dev->name);
}

dal_key_dev_t* dal_key_get_dev(const char *name)
{
    if (name == NULL || name[0] == '\0') {
        return NULL;
    }
    if (!s_key_reg_inited) {
        return NULL;
    }

    void *ctx = NULL;
    dal_registry_find_ops(&s_key_reg, name, &ctx);
    return (dal_key_dev_t *)ctx;
}

dal_err_t dal_key_init(dal_key_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    /* 必须是已注册设备，防止对野指针操作硬件 */
    if (!_key_dev_is_registered(dev)) {
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

dal_err_t dal_key_deinit(dal_key_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    if (!_key_dev_is_registered(dev)) {
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

dal_err_t dal_key_selftest(dal_key_dev_t *dev, dal_key_selftest_result_t *result)
{
    if (dev == NULL || result == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (dev->ops == NULL) {
        return DAL_ERR_NOT_READY;
    }

    /* 驱动未实现自检接口 */
    if (dev->ops->selftest == NULL) {
        *result = DAL_KEY_SELFTEST_NOT_IMPL;
        return DAL_OK;
    }

    return dev->ops->selftest(dev, result);
}

dal_err_t dal_key_get_state(dal_key_dev_t *dev, int *state)
{
    if (dev == NULL || state == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (dev->ops == NULL) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }

    return dev->ops->get_state(dev, state);
}