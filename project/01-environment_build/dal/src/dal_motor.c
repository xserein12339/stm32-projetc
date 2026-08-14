/**
 * @file    dal_motor.c
 * @brief   电机设备 DAL 层实现 v2.0
 * @note    依赖 dal_registry 提供的通用注册表管理。
 *          本文件【不提供】内部互斥，调用者需自行保证线程安全。
 * 
 * @author xserein
 * @version v2.0
 */
#include "dal_motor.h"
#include "dal_registry.h"
#include <string.h>

/* ========================================================================== */
/*                               配置宏                                         */
/* ========================================================================== */

#ifndef DAL_MOTOR_REGISTRY_SIZE
#define DAL_MOTOR_REGISTRY_SIZE 4   ///< 默认最大电机设备数
#endif

/* ========================================================================== */
/*                              内部数据                                        */
/* ========================================================================== */

static dal_reg_entry_t s_motor_reg_buf[DAL_MOTOR_REGISTRY_SIZE];
static dal_registry_t  s_motor_reg;
static bool            s_motor_reg_inited = false;

/* ========================================================================== */
/*                            内部辅助函数                                       */
/* ========================================================================== */

/**
 * @brief 惰性初始化全局电机注册表
 */
static dal_err_t _motor_reg_ensure_init(void)
{
    if (!s_motor_reg_inited) {
        dal_err_t err = dal_registry_init(&s_motor_reg, s_motor_reg_buf,
                                          DAL_MOTOR_REGISTRY_SIZE);
        if (err != DAL_OK) {
            return err;
        }
        s_motor_reg_inited = true;
    }
    return DAL_OK;
}

/**
 * @brief 校验设备实例是否确实存在于注册表中
 * @note  防止传入野指针或已注销的 dev 导致后续操作踩内存。
 */
static bool _motor_dev_is_registered(const dal_motor_dev_t *dev)
{
    if (!s_motor_reg_inited || dev == NULL || dev->name == NULL) {
        return false;
    }

    void *ctx = NULL;
    const void *ops = dal_registry_find_ops(&s_motor_reg, dev->name, &ctx);
    return (ops != NULL && ctx == (void *)dev);
}

/**
 * @brief 安全停止序列：get_state -> set_duty(0) -> brake -> disable
 * @note  用于 deinit/unregister 前自动清理，确保硬件处于安全状态。
 *        - 若设备已处于 DISABLED 状态，直接跳过（避免对已禁用设备
 *          执行 set_duty/brake 导致 BSP 层返回 ERR_DISABLED 或硬件异常）
 *        - 忽略中间步骤的错误码，尽力执行到最终 disable
 *        - 若 get_state 不可用或失败，仍执行完整停止序列（防御性兜底）
 */
static void _motor_safe_stop(dal_motor_dev_t *dev)
{
    if (dev->ops == NULL) {
        return;
    }

    /* 前置状态检查：已禁用则无需操作 */
    if (dev->ops->get_state != NULL) {
        dal_motor_state_t state;
        if (dev->ops->get_state(dev, &state) == DAL_OK) {
            if (state == DAL_MOTOR_STATE_DISABLED) {
                return;
            }
        }
        /* get_state 失败时不中断，继续执行完整停止序列作为兜底 */
    }

    if (dev->ops->set_duty != NULL) {
        (void)dev->ops->set_duty(dev, 0);
    }
    if (dev->ops->brake != NULL) {
        (void)dev->ops->brake(dev);
    }
    if (dev->ops->disable != NULL) {
        (void)dev->ops->disable(dev);
    }
}

/* ========================================================================== */
/*                          注册 / 注销 / 查找                                   */
/* ========================================================================== */

dal_err_t dal_motor_register(dal_motor_dev_t *dev)
{
    if (dev == NULL || dev->name == NULL || dev->name[0] == '\0' || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    dal_err_t err = _motor_reg_ensure_init();
    if (err != DAL_OK) {
        return err;
    }

    /* 查重：同名设备已存在 */
    void *ctx = NULL;
    if (dal_registry_find_ops(&s_motor_reg, dev->name, &ctx) != NULL) {
        return DAL_ERR_DUPLICATE;
    }

    /* 重置 DAL 内部状态 */
    dev->initialized = false;
    dev->fault_cb = NULL;
    dev->fault_cb_data = NULL;

    return dal_registry_register(&s_motor_reg, dev->name,
                                 (const void *)dev->ops, (void *)dev);
}

dal_err_t dal_motor_unregister(dal_motor_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!s_motor_reg_inited || !_motor_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    /* 若硬件仍处初始化状态，尝试安全停止 + 反初始化（忽略错误，强制注销） */
    if (dev->initialized) {
        /* 先禁用故障中断 */
        if (dev->ops->set_fault_irq_enable != NULL) {
            (void)dev->ops->set_fault_irq_enable(dev, false);
        }

        /* 安全停止 */
        _motor_safe_stop(dev);

        /* 尝试 deinit，忽略错误码 —— 强制注销 */
        if (dev->ops->deinit != NULL) {
            (void)dev->ops->deinit(dev);
        }
        dev->initialized = false;
    }

    /* 注销回调，防止悬垂指针 */
    dev->fault_cb = NULL;
    dev->fault_cb_data = NULL;

    /* 从注册表中移除（强制） */
    return dal_registry_unregister(&s_motor_reg, dev->name);
}

dal_motor_dev_t* dal_motor_get_dev(const char *name)
{
    if (name == NULL || name[0] == '\0' || !s_motor_reg_inited) {
        return NULL;
    }
    void *ctx = NULL;
    dal_registry_find_ops(&s_motor_reg, name, &ctx);
    return (dal_motor_dev_t *)ctx;
}

uint32_t dal_motor_get_count(void)
{
    return s_motor_reg_inited ? (uint32_t)dal_registry_count(&s_motor_reg) : 0;
}

dal_motor_dev_t* dal_motor_get_dev_by_index(uint32_t index)
{
    if (!s_motor_reg_inited) {
        return NULL;
    }
    uint32_t valid_idx = 0;
    uint16_t cap = s_motor_reg.capacity;

    for (uint16_t i = 0; i < cap; i++) {
        const char *name = NULL;
        void *ctx = NULL;

        dal_err_t err = dal_registry_get_entry(&s_motor_reg, i, &name, NULL, &ctx);
        if (err != DAL_OK || name == NULL) {
            continue;  /* 跳过空闲/已注销槽位 */
        }

        if (valid_idx == index) {
            return (dal_motor_dev_t *)ctx;
        }
        valid_idx++;
    }

    return NULL;  /* 索引越界 */
}

/* ========================================================================== */
/*                           生命周期管理                                        */
/* ========================================================================== */

dal_err_t dal_motor_init(dal_motor_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev)) {
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

dal_err_t dal_motor_deinit(dal_motor_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    /* 幂等：未初始化直接返回成功 */
    if (!dev->initialized) {
        return DAL_OK;
    }

    /* 先禁用故障中断 */
    if (dev->ops->set_fault_irq_enable != NULL) {
        (void)dev->ops->set_fault_irq_enable(dev, false);
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

dal_err_t dal_motor_selftest(dal_motor_dev_t *dev, dal_motor_selftest_result_t *result)
{
    if (dev == NULL || result == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (dev->ops == NULL) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->selftest == NULL) {
        *result = DAL_MOTOR_SELFTEST_NOT_IMPL;
        return DAL_OK;
    }
    return dev->ops->selftest(dev, result);
}

/* ========================================================================== */
/*                            核心控制接口                                       */
/* ========================================================================== */

dal_err_t dal_motor_set_duty(dal_motor_dev_t *dev, uint8_t duty_pct)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (duty_pct > 100) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_duty == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_duty(dev, duty_pct);
}

dal_err_t dal_motor_stop(dal_motor_dev_t *dev)
{
    /* stop 等效于 set_duty(0)，复用同一校验逻辑 */
    return dal_motor_set_duty(dev, 0);
}

dal_err_t dal_motor_set_direction(dal_motor_dev_t *dev, dal_motor_dir_t dir)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_direction == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_direction(dev, dir);
}

dal_err_t dal_motor_enable(dal_motor_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->enable == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->enable(dev);
}

dal_err_t dal_motor_disable(dal_motor_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->disable == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->disable(dev);
}

dal_err_t dal_motor_brake(dal_motor_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->brake == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->brake(dev);
}

/* ========================================================================== */
/*                            状态查询接口                                       */
/* ========================================================================== */

dal_err_t dal_motor_get_state(dal_motor_dev_t *dev, dal_motor_state_t *state)
{
    if (dev == NULL || state == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_state(dev, state);
}

dal_err_t dal_motor_get_fault(dal_motor_dev_t *dev, uint32_t *fault)
{
    if (dev == NULL || fault == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_fault == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_fault(dev, fault);
}

/* ========================================================================== */
/*                          异步回调管理接口                                     */
/* ========================================================================== */

dal_err_t dal_motor_set_fault_callback(dal_motor_dev_t *dev,
                                       dal_motor_fault_callback_t cb,
                                       void *user_data)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    dev->fault_cb = cb;
    dev->fault_cb_data = user_data;
    return DAL_OK;
}

dal_err_t dal_motor_set_fault_irq_enable(dal_motor_dev_t *dev, bool enable)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_motor_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_fault_irq_enable == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_fault_irq_enable(dev, enable);
}

/* ========================================================================== */
/*                    BSP 层 ISR 调用接口（中断上下文）                           */
/* ========================================================================== */

void dal_motor_notify_fault(dal_motor_dev_t *dev, uint32_t event)
{
    /* 极简校验：避免在中断中执行无效操作 */
    if (dev == NULL || dev->fault_cb == NULL) {
        return;
    }
    /*
     * ISR 安全说明：_motor_dev_is_registered 内部调用 dal_registry_find_ops，
     * 该函数仅遍历静态数组、不持锁，可在中断上下文安全调用。
     * 参见 dal_registry.h 中 dal_registry_find_ops 的 ISR 安全契约。
     */
    if (!_motor_dev_is_registered(dev)) {
        return;
    }

    /* 直接调用回调，调用者需保证 ISR 安全与可重入性 */
    dev->fault_cb(dev, event, dev->fault_cb_data);
}