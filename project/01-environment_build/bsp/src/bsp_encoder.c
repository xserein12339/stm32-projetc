/**
 * @file    bsp_encoder.c
 * @brief   板级编码器 BSP 层实现（基于 STM32F1 HAL + FreeRTOS）
 * @note    - 硬件映射完全引用 board_v1_config.h（定时器 ID 宏）
 *          - 对外仅暴露 bsp_encoder_init()，内部静态管理所有实例
 *          - 依赖 bsp_timer 提供的编码器模式接口（精简版 v3.0）
 *          - GPIO 配置由 bsp_timer.c 中的 HAL_TIM_Encoder_MspInit 负责
 *          - 实现 dal_encoder_ops_t 并注册到 dal_encoder 框架
 *          - v1.1: get_velocity 软件差分实现（0.1 RPM），替代原 NOT_SUPPORTED 桩
 * @author  xserein
 * @version v1.1
 */

#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_encoder.h"
#include "dal_encoder.h"
#include "bsp_timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ========================================================================== */
/*                          内部配置                                            */
/* ========================================================================== */

/** @brief 编码器每转脉冲数（电机轴）
 *  @note  JGB37-520 霍尔磁编码器：11 PPR/相/电机轴转，
 *         4 倍频后 44 计数/电机轴转（参考：厂商手册/ATmega16 数据）
 */
#define ENCODER_PPR          (11U)

/** @brief 减速比（减速箱输出轴 : 电机轴）
 *  @note  1 = 速度/位置按电机轴计算。
 *         填实际减速比（如 30 表示 1:30）后，get_velocity 输出
 *         减速箱输出轴 RPM；get_position 仍为原始计数，不做换算。
 *         减速比可实测校准：见 bsp_enc_ops_get_velocity 注释。
 */
#ifndef ENCODER_GEAR_RATIO
#define ENCODER_GEAR_RATIO   (30U)
#endif

/* ========================================================================== */
/*                     硬件描述符与私有上下文                                   */
/* ========================================================================== */

/**
 * @brief 硬件描述符（直接引用 board_v1_config.h 中的宏）
 */
typedef struct {
    const char   *name;          ///< 设备名称（唯一标识）
    uint8_t       timer_id;      ///< 定时器编号，来自 BSP_ENCODER_*_TIM_ID
    bsp_timer_enc_mode_t enc_mode; ///< 编码器倍频模式
    bool          invert;        ///< 是否反转计数方向
    uint32_t      ppr;           ///< 每转脉冲数（物理线数）
} enc_hw_desc_t;

/**
 * @brief 私有上下文（每个编码器实例独立）
 */
typedef struct {
    bsp_timer_handle_t timer_handle; ///< 由 bsp_timer_open 返回的句柄
    int32_t            last_dir_count;   /**< get_direction 专用上次计数 */
    int32_t            last_state_count; /**< get_state 专用上次计数 */
    uint32_t           ppr;          ///< 每转脉冲数（缓存）
    bsp_timer_enc_mode_t enc_mode;   ///< 缓存倍频模式
    bool               zeroed;       ///< 是否已归零（用于 CAP_ZEROED 标志）
    bool               is_initialized; ///< 硬件是否已初始化

    /* 软件差分测速状态（get_velocity 专用，约定单任务调用） */
    int32_t            vel_last_count;   /**< 上次采样计数 */
    uint32_t           vel_last_tick;    /**< 上次采样 tick */
    int32_t            vel_last_result;  /**< 上次计算结果（dt=0 时返回） */
    bool               vel_seeded;       /**< 已完成首次采样 */
} bsp_encoder_priv_t;

/* ========================================================================== */
/*                        静态数据池                                            */
/* ========================================================================== */

/**
 * @brief 编码器硬件映射表
 * @note  直接使用 board_v1_config.h 中定义的宏，消除硬编码
 *        左编码器 -> BSP_ENCODER_LEFT_TIM_ID (通常为 2)
 *        右编码器 -> BSP_ENCODER_RIGHT_TIM_ID (通常为 3)
 */
static const enc_hw_desc_t s_enc_table[] = {
    {
        .name      = "encoder_left",
        .timer_id  = BSP_ENCODER_LEFT_TIM_ID,
        .enc_mode  = BSP_TIMER_ENC_MODE_X4,
        .invert    = false,
        .ppr       = ENCODER_PPR,
    },
    {
        .name      = "encoder_right",
        .timer_id  = BSP_ENCODER_RIGHT_TIM_ID,
        .enc_mode  = BSP_TIMER_ENC_MODE_X4,
        .invert    = false,
        .ppr       = ENCODER_PPR,
    },
};

#define ENC_COUNT  (sizeof(s_enc_table) / sizeof(s_enc_table[0]))

static bsp_encoder_priv_t s_priv_pool[ENC_COUNT];
static dal_encoder_dev_t  s_dev_pool[ENC_COUNT];

/* ========================================================================== */
/*                         内部辅助函数                                         */
/* ========================================================================== */

/**
 * @brief 根据倍频模式返回实际倍频系数
 */
static uint32_t _get_multiplier(bsp_timer_enc_mode_t mode)
{
    switch (mode) {
        case BSP_TIMER_ENC_MODE_X1: return 1U;
        case BSP_TIMER_ENC_MODE_X2: return 2U;
        case BSP_TIMER_ENC_MODE_X4: return 4U;
        default:                    return 4U;
    }
}

/**
 * @brief 通过设备名查找硬件描述符
 */
static const enc_hw_desc_t *_find_hw(const char *name)
{
    for (uint32_t i = 0; i < ENC_COUNT; i++) {
        if (strcmp(s_enc_table[i].name, name) == 0) {
            return &s_enc_table[i];
        }
    }
    return NULL;
}

/* ========================================================================== */
/*                         DAL ops 实现                                         */
/* ========================================================================== */

static dal_err_t bsp_enc_ops_init(dal_encoder_dev_t *dev)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv) return DAL_ERR_DEPENDENCY;

    const enc_hw_desc_t *hw = _find_hw(dev->name);
    if (!hw) return DAL_ERR_NOT_READY;

    /* 构建 bsp_timer 配置（编码器模式） */
    bsp_timer_config_t cfg = {
        .freq_hz    = BSP_TIMER_FREQ_DEFAULT,
        .mode       = BSP_TIMER_MODE_ENCODER,
        .enc_mode   = hw->enc_mode,
        .enc_invert = hw->invert,
        .enable_irq = true,   /* 必须使能中断用于 32 位溢出累加 */
    };

    bsp_timer_handle_t handle;
    bsp_err_t err = bsp_timer_open(hw->timer_id, &cfg, &handle);
    if (err != BSP_OK) {
        return DAL_ERR_DEPENDENCY;   /* 底层定时器打开失败 */
    }

    /* 启动编码器计数 */
    err = bsp_timer_start(handle, 0, 0);
    if (err != BSP_OK) {
        bsp_timer_close(handle);
        return DAL_ERR_DEPENDENCY;   /* 底层定时器启动失败 */
    }

    priv->timer_handle   = handle;
    priv->ppr            = hw->ppr;
    priv->enc_mode       = hw->enc_mode;
    priv->last_dir_count = 0;
    priv->last_state_count = 0;
    priv->zeroed         = false;
    priv->is_initialized = true;

    return DAL_OK;
}

static dal_err_t bsp_enc_ops_deinit(dal_encoder_dev_t *dev)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv || !priv->timer_handle) return DAL_ERR_NOT_READY;

    bsp_timer_stop(priv->timer_handle);
    bsp_timer_close(priv->timer_handle);
    priv->timer_handle   = NULL;
    priv->is_initialized = false;

    return DAL_OK;
}

static dal_err_t bsp_enc_ops_selftest(dal_encoder_dev_t *dev,
                                      dal_encoder_selftest_result_t *result)
{
    (void)dev;
    *result = DAL_ENCODER_SELFTEST_NOT_IMPL;
    return DAL_OK;
}

static dal_err_t bsp_enc_ops_get_position(dal_encoder_dev_t *dev, int32_t *position)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv || !priv->timer_handle || !priv->is_initialized)
        return DAL_ERR_NOT_READY;

    bsp_err_t err = bsp_timer_encoder_get_count(priv->timer_handle, position);
    return (err == BSP_OK) ? DAL_OK : DAL_ERR_DEPENDENCY;
}

static dal_err_t bsp_enc_ops_get_angle(dal_encoder_dev_t *dev, uint32_t *angle)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv || !priv->timer_handle || !priv->is_initialized)
        return DAL_ERR_NOT_READY;

    if (!priv->zeroed) return DAL_ERR_NOT_SUPPORTED;

    int32_t pos;
    bsp_err_t err = bsp_timer_encoder_get_count(priv->timer_handle, &pos);
    if (err != BSP_OK) return DAL_ERR_DEPENDENCY;

    uint32_t cpr = priv->ppr * _get_multiplier(priv->enc_mode);
    /* 取模运算处理负数位置 */
    int32_t mod = pos % (int32_t)cpr;
    if (mod < 0) mod += (int32_t)cpr;

    /* 转换为 0.01 度单位: angle = mod * 36000 / cpr */
    *angle = (uint32_t)((uint64_t)mod * 36000U / cpr);
    return DAL_OK;
}

/**
 * @brief 软件差分测速：v = Δcount / Δt
 * @note  - 单位契约（dal_encoder.h）：0.1 RPM，正 CW / 负 CCW
 *          - 依赖上层周期性轮询（如 20Hz 显示任务），两次调用间隔
 *            即为测速窗口；采样间隔过短（<1 tick）时返回上次结果
 *          - 计算式：vel[0.1RPM] = Δcount * 600000 / (Δms * CPR)
 *            其中 CPR = ppr * 倍频系数 * 减速比（ENCODER_GEAR_RATIO），
 *            表示减速箱输出轴每转的总计数
 *          - 减速比实测校准：复位计数后手转输出轴整整 1 圈，
 *            读 get_position，减速比 = 读数 / (ppr * 倍频系数)
 * @warning 差分状态无锁保护，约定仅从单一任务上下文调用；
 *          中断回调中严禁调用（DAL 事件契约）
 */
static dal_err_t bsp_enc_ops_get_velocity(dal_encoder_dev_t *dev, int32_t *velocity)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv || !priv->timer_handle || !priv->is_initialized)
        return DAL_ERR_NOT_READY;

    int32_t current;
    bsp_err_t err = bsp_timer_encoder_get_count(priv->timer_handle, &current);
    if (err != BSP_OK) return DAL_ERR_DEPENDENCY;

    uint32_t now = xTaskGetTickCount();
    int32_t result = 0;

    if (priv->vel_seeded) {
        uint32_t dt_ms = (now - priv->vel_last_tick) * portTICK_PERIOD_MS;
        if (dt_ms == 0) {
            result = priv->vel_last_result;
        } else {
            int32_t  dc  = current - priv->vel_last_count;
            uint32_t cpr = priv->ppr * _get_multiplier(priv->enc_mode)
                           * ENCODER_GEAR_RATIO;
            result = (int32_t)((int64_t)dc * 600000LL /
                               ((int64_t)dt_ms * (int64_t)cpr));
        }
    }

    priv->vel_last_count  = current;
    priv->vel_last_tick   = now;
    priv->vel_last_result = result;
    priv->vel_seeded      = true;

    *velocity = result;
    return DAL_OK;
}

static dal_err_t bsp_enc_ops_get_direction(dal_encoder_dev_t *dev,
                                           dal_encoder_dir_t *direction)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv || !priv->timer_handle || !priv->is_initialized)
        return DAL_ERR_NOT_READY;

    int32_t current;
    bsp_err_t err = bsp_timer_encoder_get_count(priv->timer_handle, &current);
    if (err != BSP_OK) return DAL_ERR_DEPENDENCY;

    int32_t delta = current - priv->last_dir_count;
    priv->last_dir_count = current;

    if (delta > 0)      *direction = DAL_ENCODER_DIR_CW;
    else if (delta < 0) *direction = DAL_ENCODER_DIR_CCW;
    else                *direction = DAL_ENCODER_DIR_UNKNOWN;

    return DAL_OK;
}

static dal_err_t bsp_enc_ops_reset(dal_encoder_dev_t *dev)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv || !priv->timer_handle || !priv->is_initialized)
        return DAL_ERR_NOT_READY;

    bsp_err_t err = bsp_timer_encoder_reset(priv->timer_handle);
    if (err != BSP_OK) return DAL_ERR_DEPENDENCY;

    priv->last_dir_count   = 0;
    priv->last_state_count = 0;
    /* 计数已清零，测速差分基准必须同步失效，
     * 否则下次 get_velocity 会用旧基准算出巨大假速度 */
    priv->vel_seeded = false;
    priv->zeroed           = true;
    return DAL_OK;
}

static dal_err_t bsp_enc_ops_find_zero(dal_encoder_dev_t *dev, uint32_t timeout_ms)
{
    (void)timeout_ms;
    /* 降级为软件归零；完整的硬件寻零序列应由 SVC 层编排 */
    return bsp_enc_ops_reset(dev);
}

static dal_err_t bsp_enc_ops_get_state(dal_encoder_dev_t *dev,
                                       dal_encoder_state_t *state)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv || !priv->timer_handle || !priv->is_initialized)
        return DAL_ERR_NOT_READY;

    int32_t current;
    bsp_err_t err = bsp_timer_encoder_get_count(priv->timer_handle, &current);
    if (err != BSP_OK) return DAL_ERR_DEPENDENCY;

    /* 使用独立的 last_state_count，不与 get_direction 互相干扰 */
    *state = (current != priv->last_state_count)
             ? DAL_ENCODER_STATE_RUNNING
             : DAL_ENCODER_STATE_IDLE;
    priv->last_state_count = current;

    return DAL_OK;
}

static dal_err_t bsp_enc_ops_get_fault(dal_encoder_dev_t *dev, uint32_t *fault)
{
    (void)dev;
    *fault = 0U;
    return DAL_OK;
}

static dal_err_t bsp_enc_ops_get_info(dal_encoder_dev_t *dev,
                                      uint32_t *resolution,
                                      dal_encoder_type_t *type,
                                      uint32_t *capability)
{
    bsp_encoder_priv_t *priv = (bsp_encoder_priv_t *)dev->drv_priv;
    if (!priv || !priv->is_initialized) return DAL_ERR_NOT_READY;

    if (resolution) {
        *resolution = priv->ppr * _get_multiplier(priv->enc_mode);
    }
    if (type) {
        *type = DAL_ENCODER_TYPE_INCREMENTAL;
    }
    if (capability) {
        uint32_t cap = DAL_ENCODER_CAP_MULTI_TURN;
        if (priv->zeroed) {
            cap |= DAL_ENCODER_CAP_ZEROED;
        }
        *capability = cap;
    }

    return DAL_OK;
}

static dal_err_t bsp_enc_ops_set_event_irq_enable(dal_encoder_dev_t *dev, bool enable)
{
    (void)dev;
    (void)enable;
    return DAL_ERR_NOT_SUPPORTED;
}

/* ========================================================================== */
/*                       dal_encoder_ops_t 实例                                 */
/* ========================================================================== */

static const dal_encoder_ops_t g_bsp_enc_ops = {
    .init                 = bsp_enc_ops_init,
    .deinit               = bsp_enc_ops_deinit,
    .selftest             = bsp_enc_ops_selftest,
    .get_position         = bsp_enc_ops_get_position,
    .get_angle            = bsp_enc_ops_get_angle,
    .get_velocity         = bsp_enc_ops_get_velocity,
    .get_direction        = bsp_enc_ops_get_direction,
    .reset                = bsp_enc_ops_reset,
    .find_zero            = bsp_enc_ops_find_zero,
    .get_state            = bsp_enc_ops_get_state,
    .get_fault            = bsp_enc_ops_get_fault,
    .get_info             = bsp_enc_ops_get_info,
    .set_event_irq_enable = bsp_enc_ops_set_event_irq_enable,
};

/* ========================================================================== */
/*                     BSP 公共接口                                            */
/* ========================================================================== */

bsp_err_t bsp_encoder_init(void)
{
    for (uint32_t i = 0; i < ENC_COUNT; i++) {
        const enc_hw_desc_t *hw = &s_enc_table[i];

        bsp_encoder_priv_t *priv = &s_priv_pool[i];
        memset(priv, 0, sizeof(bsp_encoder_priv_t));
        priv->ppr      = hw->ppr;
        priv->enc_mode = hw->enc_mode;
        priv->zeroed   = false;

        dal_encoder_dev_t *dev = &s_dev_pool[i];
        dev->name     = hw->name;
        dev->ops      = &g_bsp_enc_ops;
        dev->drv_priv = priv;

        dal_err_t ret = dal_encoder_register(dev);
        if (ret != DAL_OK) {
            /* 回滚已注册实例，保证原子性 */
            for (uint32_t j = 0; j < i; j++) {
                (void)dal_encoder_unregister(&s_dev_pool[j]);
            }
            return BSP_ERR_FAIL;
        }
    }

    return BSP_OK;
}