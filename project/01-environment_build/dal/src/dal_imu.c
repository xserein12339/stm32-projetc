/**
 * @file    dal_imu.c
 * @brief   IMU 设备 DAL 层实现 v1.0
 * @note    依赖 dal_registry 提供的通用注册表管理。
 *          本文件【不提供】内部互斥，调用者需自行保证线程安全。
 * 
 * @author xserein
 * @version v1.0
 */
#include "dal_imu.h"
#include "dal_registry.h"
#include <string.h>

/* ========================================================================== */
/*                               配置宏                                         */
/* ========================================================================== */

#ifndef DAL_IMU_REGISTRY_SIZE
#define DAL_IMU_REGISTRY_SIZE 4   /**< 默认最大 IMU 设备数 */
#endif

/* ========================================================================== */
/*                              内部数据                                        */
/* ========================================================================== */

static dal_reg_entry_t s_imu_reg_buf[DAL_IMU_REGISTRY_SIZE];
static dal_registry_t  s_imu_reg;
static bool            s_imu_reg_inited = false;

/* ========================================================================== */
/*                            内部辅助函数                                       */
/* ========================================================================== */

static dal_err_t _imu_reg_ensure_init(void)
{
    if (!s_imu_reg_inited) {
        dal_err_t err = dal_registry_init(&s_imu_reg, s_imu_reg_buf,
                                          DAL_IMU_REGISTRY_SIZE);
        if (err != DAL_OK) {
            return err;
        }
        s_imu_reg_inited = true;
    }
    return DAL_OK;
}

/**
 * @brief 校验设备实例是否确实存在于注册表中
 * @note  ISR 安全：dal_registry_find_ops 为纯只读静态数组遍历。
 */
static bool _imu_dev_is_registered(const dal_imu_dev_t *dev)
{
    if (!s_imu_reg_inited || dev == NULL || dev->name == NULL) {
        return false;
    }

    void *ctx = NULL;
    const void *ops = dal_registry_find_ops(&s_imu_reg, dev->name, &ctx);
    return (ops != NULL && ctx == (void *)dev);
}

/* ========================================================================== */
/*                             注册 / 注销 / 查找                                */
/* ========================================================================== */

dal_err_t dal_imu_register(dal_imu_dev_t *dev)
{
    if (dev == NULL || dev->name == NULL || dev->name[0] == '\0' || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }

    dal_err_t err = _imu_reg_ensure_init();
    if (err != DAL_OK) {
        return err;
    }

    /* 查重：同名设备已存在 */
    void *ctx = NULL;
    if (dal_registry_find_ops(&s_imu_reg, dev->name, &ctx) != NULL) {
        return DAL_ERR_DUPLICATE;
    }

    /* 重置 DAL 内部状态（防止 re-register 后残留旧回调） */
    dev->initialized   = false;
    dev->event_cb      = NULL;
    dev->event_cb_data = NULL;

    return dal_registry_register(&s_imu_reg, dev->name,
                                 (const void *)dev->ops, (void *)dev);
}

dal_err_t dal_imu_unregister(dal_imu_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!s_imu_reg_inited || !_imu_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    /*
     * 【强制注销语义】
     * 即使 deinit 返回错误，设备仍会从注册表中移除。
     */
    if (dev->initialized) {
        /* 先禁用事件中断，防止注销过程中触发回调 */
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

    return dal_registry_unregister(&s_imu_reg, dev->name);
}

dal_imu_dev_t* dal_imu_get_dev(const char *name)
{
    if (name == NULL || name[0] == '\0' || !s_imu_reg_inited) {
        return NULL;
    }

    void *ctx = NULL;
    dal_registry_find_ops(&s_imu_reg, name, &ctx);
    return (dal_imu_dev_t *)ctx;
}

uint32_t dal_imu_get_count(void)
{
    return s_imu_reg_inited ? (uint32_t)dal_registry_count(&s_imu_reg) : 0;
}

dal_imu_dev_t* dal_imu_get_dev_by_index(uint32_t index)
{
    if (!s_imu_reg_inited) {
        return NULL;
    }

    uint32_t valid_idx = 0;
    uint16_t cap = s_imu_reg.capacity;

    for (uint16_t i = 0; i < cap; i++) {
        const char *name = NULL;
        void *ctx = NULL;

        dal_err_t err = dal_registry_get_entry(&s_imu_reg, i, &name, NULL, &ctx);
        if (err != DAL_OK || name == NULL) {
            continue;
        }

        if (valid_idx == index) {
            return (dal_imu_dev_t *)ctx;
        }
        valid_idx++;
    }

    return NULL;
}

/* ========================================================================== */
/*                           生命周期管理                                        */
/* ========================================================================== */

dal_err_t dal_imu_init(dal_imu_dev_t *dev)
{
    if (dev == NULL || dev->ops == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev)) {
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

dal_err_t dal_imu_deinit(dal_imu_dev_t *dev)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev)) {
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

dal_err_t dal_imu_selftest(dal_imu_dev_t *dev,
                           dal_imu_selftest_result_t *result)
{
    if (dev == NULL || result == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops == NULL || dev->ops->selftest == NULL) {
        *result = DAL_IMU_SELFTEST_NOT_IMPL;
        return DAL_OK;
    }
    return dev->ops->selftest(dev, result);
}

/* ========================================================================== */
/*                            核心数据读取接口                                    */
/* ========================================================================== */

dal_err_t dal_imu_read(dal_imu_dev_t *dev, dal_imu_data_t *data)
{
    if (dev == NULL || data == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->read == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    /*
     * 【并发互斥契约】
     * 若异步 FIFO 读取正在进行，DRV 层的 read 实现必须返回 DAL_ERR_BUSY，
     * 禁止阻塞等待。DAL 层在此不做额外状态检查，将并发判断权
     * 完全交给 DRV 层（DRV 层拥有最准确的总线/DMA 状态）。
     */
    return dev->ops->read(dev, data);
}

dal_err_t dal_imu_fifo_read(dal_imu_dev_t *dev, dal_imu_data_t *buf,
                            uint32_t max_count, uint32_t *actual)
{
    if (dev == NULL || buf == NULL || actual == NULL || max_count == 0) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->fifo_read == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    /*
     * 【并发互斥契约（与 read 对称）】
     * 若同步 read 正在执行，DRV 层的 fifo_read 实现必须返回 DAL_ERR_BUSY。
     * read 与 fifo_read 构成双向互斥对，任一操作进行中另一操作均须返回 BUSY。
     *
     * 【时间戳填充约定】
     * DRV 层仅需填充 buf[0].timestamp_us，后续样本时间戳由 SVC 层推算。
     */
    return dev->ops->fifo_read(dev, buf, max_count, actual);
}

/* ========================================================================== */
/*                          配置与控制接口                                       */
/* ========================================================================== */

dal_err_t dal_imu_set_odr(dal_imu_dev_t *dev, uint32_t module,
                          dal_imu_odr_t odr)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_odr == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_odr(dev, module, odr);
}

dal_err_t dal_imu_set_accel_range(dal_imu_dev_t *dev,
                                  dal_imu_accel_range_t range)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_accel_range == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_accel_range(dev, range);
}

dal_err_t dal_imu_set_gyro_range(dal_imu_dev_t *dev,
                                 dal_imu_gyro_range_t range)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_gyro_range == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_gyro_range(dev, range);
}

dal_err_t dal_imu_set_fifo_threshold(dal_imu_dev_t *dev, uint16_t threshold)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_fifo_threshold == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->set_fifo_threshold(dev, threshold);
}

dal_err_t dal_imu_set_calibration(dal_imu_dev_t *dev,
                                  const dal_imu_calibration_t *cal)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->set_calibration == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    /* cal 允许为 NULL（表示清除校准），不做非空检查 */
    return dev->ops->set_calibration(dev, cal);
}

dal_err_t dal_imu_set_power(dal_imu_dev_t *dev, bool on)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
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

dal_err_t dal_imu_get_state(dal_imu_dev_t *dev, dal_imu_state_t *state)
{
    if (dev == NULL || state == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_state == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_state(dev, state);
}

dal_err_t dal_imu_get_fault(dal_imu_dev_t *dev, uint32_t *fault)
{
    if (dev == NULL || fault == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_fault == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_fault(dev, fault);
}

dal_err_t dal_imu_get_info(dal_imu_dev_t *dev, uint32_t *modules,
                           uint32_t *capability, uint16_t *fifo_depth)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
        return DAL_ERR_NOT_READY;
    }
    if (dev->ops->get_info == NULL) {
        return DAL_ERR_NOT_SUPPORTED;
    }
    return dev->ops->get_info(dev, modules, capability, fifo_depth);
}

/* ========================================================================== */
/*                          异步回调管理接口                                     */
/* ========================================================================== */

dal_err_t dal_imu_set_event_callback(dal_imu_dev_t *dev,
                                     dal_imu_event_callback_t cb,
                                     void *user_data)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev)) {
        return DAL_ERR_NOT_READY;
    }

    dev->event_cb      = cb;
    dev->event_cb_data = user_data;
    return DAL_OK;
}

dal_err_t dal_imu_set_event_irq_enable(dal_imu_dev_t *dev, bool enable)
{
    if (dev == NULL) {
        return DAL_ERR_PARAM_INVALID;
    }
    if (!_imu_dev_is_registered(dev) || !dev->initialized) {
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

void dal_imu_notify_event(dal_imu_dev_t *dev, uint32_t event)
{
    if (dev == NULL || dev->event_cb == NULL) {
        return;
    }

    /*
     * ISR 安全说明：_imu_dev_is_registered 内部调用 dal_registry_find_ops，
     * 该函数仅遍历静态数组、不持锁，可在中断/DMA完成上下文安全调用。
     */
    if (!_imu_dev_is_registered(dev)) {
        return;
    }

    dev->event_cb(dev, event, dev->event_cb_data);
}