/**
 * @file    dal_led.c
 * @brief   LED 设备 DAL 层实现 v2.0
 * @note    依赖 dal_registry 提供的通用注册表管理。
 *          本文件【不提供】内部互斥，调用者需自行保证线程安全。
 * 
 * @author xserein
 * @version v2.0
 */
#include "dal_led.h"
#include "dal_registry.h"
#include <string.h>

/* ========================================================================== */
/*                               配置宏                                         */
/* ========================================================================== */

#ifndef DAL_LED_REGISTRY_SIZE
#define DAL_LED_REGISTRY_SIZE 8   /**< 默认最大 LED 设备数 */
#endif

/* ========================================================================== */
/*                              内部数据                                        */
/* ========================================================================== */

static dal_reg_entry_t s_led_reg_buf[DAL_LED_REGISTRY_SIZE];
static dal_registry_t  s_led_reg;
static bool            s_led_reg_inited = false;

/* ========================================================================== */
/*                            内部辅助函数                                       */
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

/**
 * @brief 校验设备实例是否确实存在于注册表中
 * @note  ISR 安全：dal_registry_find_ops 为纯只读静态数组遍历。
 */
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
/*                             注册 / 注销 / 查找                                */
/* ========================================================================== */

dal_err_t dal_led_register(dal_led_dev_t *dev)
{
    if (dev == NULL || dev->name == NULL || dev->name[0] == '\0' || dev->ops == NULL) {
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

    /* 重置 DAL 内部状态（LED 无回调字段，仅重置 initialized） */
    dev->initialized = false;

    return dal_registry_register(&s_led_reg, dev->name,
                                 (const void *)dev->ops, (void *)dev);
}

/**
 * @brief 从DAL框架注销LED设备实例
 * @note 【强制注销语义】
 *       注销前会自动执行 deinit（若已初始化）。
 *       即使 deinit 返回错误，设备仍会从注册表中移除，函数返回 DAL_OK。
 *       调用者【无法】通过返回值感知底层 deinit 是否失败。
 *       如需诊断 deinit 失败原因，请在调用 unregister 前单独调用 deinit。
 */
dal_err_t dal_led_unregister(dal_led_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!s_led_reg_inited || !_led_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    /* 若硬件仍处初始化状态，尝试反初始化（忽略错误，强制注销） */
    if (dev->initialized) {
        if (dev->ops != NULL && dev->ops->deinit != NULL) {
            (void)dev->ops->deinit(dev);
        }
        dev->initialized = false;
    }

    return dal_registry_unregister(&s_led_reg, dev->name);
}

dal_led_dev_t* dal_led_get_dev(const char *name)
{
    if (name == NULL || name[0] == '\0' || !s_led_reg_inited) {
        return NULL;
    }

    void *ctx = NULL;
    dal_registry_find_ops(&s_led_reg, name, &ctx);
    return (dal_led_dev_t *)ctx;
}

uint32_t dal_led_get_count(void)
{
    return s_led_reg_inited ? (uint32_t)dal_registry_count(&s_led_reg) : 0;
}

/**
 * @brief 按逻辑索引获取已注册的 LED 设备实例
 * @note  遍历物理数组跳过空闲槽位，返回第 (index+1) 个有效设备。
 */
dal_led_dev_t* dal_led_get_dev_by_index(uint32_t index)
{
    if (!s_led_reg_inited) {
        return NULL;
    }

    uint32_t valid_idx = 0;
    uint16_t cap = s_led_reg.capacity;

    for (uint16_t i = 0; i < cap; i++) {
        const char *name = NULL;
        void *ctx = NULL;

        dal_err_t err = dal_registry_get_entry(&s_led_reg, i, &name, NULL, &ctx);
        if (err != DAL_OK || name == NULL) {
            continue;
        }

        if (valid_idx == index) {
            return (dal_led_dev_t *)ctx;
        }
        valid_idx++;
    }

    return NULL;
}

/* ========================================================================== */
/*                           生命周期管理                                        */
/* ========================================================================== */

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

/**
 * @brief 反初始化LED设备硬件
 * @note 幂等设计：若未初始化则直接返回 DAL_OK。
 */
dal_err_t dal_led_deinit(dal_led_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_led_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    /* 幂等：未初始化直接返回成功 */
    if (!dev->initialized) {
        return DAL_OK;
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

/* ========================================================================== */
/*                            核心控制接口                                       */
/* ========================================================================== */

dal_err_t dal_led_set_state(dal_led_dev_t *dev, dal_led_state_t state)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (state != DAL_LED_ON && state != DAL_LED_OFF) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_led_dev_is_registered(dev) || !dev->initialized) {
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
    if (!_led_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_state(dev, state);
}