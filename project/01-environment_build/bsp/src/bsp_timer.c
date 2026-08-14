/**
 * @file    bsp_timer.c
 * @brief   BSP Timer 精简驱动 — 仅支持 PWM 与编码器模式 v3.0
 *
 * @details 支持 TIM1~TIM4，每实例独立管理。
 *          - 编码器模式：中断+软件累加实现 32 位有符号计数
 *          - PWM 模式：open 时预初始化全部 4 通道，支持运行时独立更新占空比
 *          - DWT CYCCNT 用于 delay_us 高精度微秒延时
 *          - GPIO 配置通过 HAL MspInit 回调完成，引用 board_v1_config.h
 *
 * @author  xserein
 * @version v3.0
 */
#include "bsp_timer.h"
#include "board_v1.h"
#include "board_v1_config.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ========================================================================== */
/*                         编译期配置                                          */
/* ========================================================================== */

#if defined(configMAX_SYSCALL_INTERRUPT_PRIORITY) && defined(BSP_TIMER_IRQ_PRIO_ENCODER)
    #if BSP_TIMER_IRQ_PRIO_ENCODER < configMAX_SYSCALL_INTERRUPT_PRIORITY
        #error "BSP_TIMER_IRQ_PRIO_ENCODER must be >= configMAX_SYSCALL_INTERRUPT_PRIORITY"
    #endif
#endif

#define BSP_TIMER_MAX_INSTANCES  4U
#define BSP_TIMER_APB_CLK_HZ     72000000U

#ifndef BSP_TIMER_IRQ_PRIO_ENCODER
#define BSP_TIMER_IRQ_PRIO_ENCODER   5U
#endif

#ifndef BSP_TIMER_IRQ_PRIO_PWM
#define BSP_TIMER_IRQ_PRIO_PWM       7U
#endif

/* DWT 寄存器 */
#define BSP_DWT_CTRL     (*(volatile uint32_t *)0xE0001000U)
#define BSP_DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004U)
#define BSP_DWT_CTRL_EN  (1U << 0)
#define BSP_DEMCR        (*(volatile uint32_t *)0xE000EDFCU)
#define BSP_DEMCR_TRCENA (1U << 24)

/* ========================================================================== */
/*                          内部类型定义                                        */
/* ========================================================================== */

typedef struct {
    TIM_HandleTypeDef    htim;
    TIM_TypeDef         *tim_periph;
    IRQn_Type            irqn;

    bsp_timer_config_t   config;
    uint32_t             timer_clk_hz;
    uint32_t             period_ticks;      ///< ARR 值（PWM 模式下有效）
    uint32_t             prescaler;         ///< PSC 值

    bsp_timer_callback_t callback;
    void                *callback_arg;
    bool                 irq_enabled;

    /* 编码器专用 */
    volatile int32_t     enc_accum;
    uint16_t             enc_last_cnt;      ///< 与硬件寄存器位宽一致

    uint8_t              id;
    bool                 in_use;
    bool                 running;
} bsp_timer_inst_t;

/* ========================================================================== */
/*                         静态数据                                            */
/* ========================================================================== */

static bsp_timer_inst_t s_timers[BSP_TIMER_MAX_INSTANCES];
static bool s_dwt_inited = false;

static const struct {
    TIM_TypeDef *periph;
    IRQn_Type    irqn;
} s_hw_map[BSP_TIMER_MAX_INSTANCES] = {
    { TIM1, TIM1_UP_IRQn },
    { TIM2, TIM2_IRQn    },
    { TIM3, TIM3_IRQn    },
    { TIM4, TIM4_IRQn    },
};

static const uint32_t s_hal_ch_map[4] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/* ========================================================================== */
/*                        内部辅助函数                                          */
/* ========================================================================== */

static bool _handle_valid(bsp_timer_handle_t h)
{
    if (h == NULL) return false;
    ptrdiff_t idx = (bsp_timer_inst_t *)h - s_timers;
    return (idx >= 0 && idx < BSP_TIMER_MAX_INSTANCES && ((bsp_timer_inst_t *)h)->in_use);
}

static void _enable_periph_clock(uint8_t id)
{
    switch (id) {
        case 1: __HAL_RCC_TIM1_CLK_ENABLE(); break;
        case 2: __HAL_RCC_TIM2_CLK_ENABLE(); break;
        case 3: __HAL_RCC_TIM3_CLK_ENABLE(); break;
        case 4: __HAL_RCC_TIM4_CLK_ENABLE(); break;
        default: break;
    }
}

static void _disable_periph_clock(uint8_t id)
{
    switch (id) {
        case 1: __HAL_RCC_TIM1_CLK_DISABLE(); break;
        case 2: __HAL_RCC_TIM2_CLK_DISABLE(); break;
        case 3: __HAL_RCC_TIM3_CLK_DISABLE(); break;
        case 4: __HAL_RCC_TIM4_CLK_DISABLE(); break;
        default: break;
    }
}

/**
 * @brief 计算预分频器和 ARR 值
 */
static bsp_err_t _calc_prescaler_arr(uint32_t clk_hz, uint32_t freq_hz,
                                     uint32_t period_us,
                                     uint32_t *psc, uint32_t *arr)
{
    uint32_t target_ticks;
    if (period_us > 0) {
        target_ticks = (uint32_t)((uint64_t)clk_hz * period_us / 1000000U);
    } else if (freq_hz > 0) {
        target_ticks = clk_hz / freq_hz;
    } else {
        return BSP_ERR_PARAM;
    }
    if (target_ticks == 0) return BSP_ERR_PARAM;

    for (uint32_t p = 0; p <= 65535U; p++) {
        uint32_t a = target_ticks / (p + 1U);
        if (a >= 1U && a <= 65535U) {
            *psc = p;
            *arr = a - 1U;
            return BSP_OK;
        }
    }
    return BSP_ERR_PARAM;
}

/* ========================================================================== */
/*                        模式配置函数                                          */
/* ========================================================================== */

static bsp_err_t _config_pwm_mode(bsp_timer_inst_t *inst,
                                  uint32_t psc, uint32_t arr)
{
    TIM_HandleTypeDef *htim = &inst->htim;
    htim->Instance = inst->tim_periph;
    htim->Init.Prescaler = psc;
    htim->Init.CounterMode = (inst->config.pwm_align == BSP_TIMER_PWM_CENTER_ALIGNED)
                             ? TIM_COUNTERMODE_CENTERALIGNED1
                             : TIM_COUNTERMODE_UP;
    htim->Init.Period = arr;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(htim) != HAL_OK) return BSP_ERR_IO;

    TIM_OC_InitTypeDef oc_cfg = {
        .OCMode     = TIM_OCMODE_PWM1,
        .Pulse      = 0,
        .OCPolarity = (inst->config.pwm_polarity == BSP_TIMER_PWM_ACTIVE_HIGH)
                      ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW,
        .OCFastMode = TIM_OCFAST_DISABLE,
    };

    /* 预初始化全部 4 通道，使后续 pwm_set_duty 可直接写 CCR */
    for (uint8_t ch = 0; ch < 4; ch++) {
        if (HAL_TIM_PWM_ConfigChannel(htim, &oc_cfg, s_hal_ch_map[ch]) != HAL_OK) {
            return BSP_ERR_IO;
        }
    }
    return BSP_OK;
}

/**
 * @brief 配置编码器模式
 * @note  STM32F1 硬件仅支持 TI1(1x) 和 TI12(4x) 两种编码器模式。
 *        BSP_TIMER_ENC_MODE_X2 降级映射为 TI12（实际 4 倍频），
 *        如需真正 2 倍频请在软件层对 4x 计数值除以 2。
 */
static bsp_err_t _config_encoder_mode(bsp_timer_inst_t *inst)
{
    TIM_HandleTypeDef *htim = &inst->htim;
    htim->Instance = inst->tim_periph;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    uint32_t enc_mode;
    switch (inst->config.enc_mode) {
        case BSP_TIMER_ENC_MODE_X1:
            enc_mode = TIM_ENCODERMODE_TI1;
            break;
        case BSP_TIMER_ENC_MODE_X2:
            /* F1 无硬件 X2，降级为 TI12 */
            enc_mode = TIM_ENCODERMODE_TI12;
            break;
        case BSP_TIMER_ENC_MODE_X4:
        default:
            enc_mode = TIM_ENCODERMODE_TI12;
            break;
    }

    uint32_t pol = inst->config.enc_invert
                   ? TIM_INPUTCHANNELPOLARITY_FALLING
                   : TIM_INPUTCHANNELPOLARITY_RISING;

    TIM_Encoder_InitTypeDef enc_cfg = {
        .EncoderMode  = enc_mode,
        .IC1Polarity  = pol,
        .IC1Selection = TIM_ICSELECTION_DIRECTTI,
        .IC1Prescaler = TIM_ICPSC_DIV1,
        .IC1Filter    = 0x0F,
        .IC2Polarity  = pol,  /* IC2 极性应与 IC1 一致 */
        .IC2Selection = TIM_ICSELECTION_DIRECTTI,
        .IC2Prescaler = TIM_ICPSC_DIV1,
        .IC2Filter    = 0x0F,
    };

    if (HAL_TIM_Encoder_Init(htim, &enc_cfg) != HAL_OK) return BSP_ERR_IO;
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    return BSP_OK;
}

static void _config_nvic(bsp_timer_inst_t *inst)
{
    uint8_t prio = (inst->config.mode == BSP_TIMER_MODE_ENCODER)
                   ? BSP_TIMER_IRQ_PRIO_ENCODER
                   : BSP_TIMER_IRQ_PRIO_PWM;
    HAL_NVIC_SetPriority(inst->irqn, prio, 0);
    HAL_NVIC_EnableIRQ(inst->irqn);
}

static void _disable_nvic(bsp_timer_inst_t *inst)
{
    HAL_NVIC_DisableIRQ(inst->irqn);
}

/* ========================================================================== */
/*                      HAL MspInit 回调（GPIO 配置）                           */
/* ========================================================================== */

/**
 * @brief HAL 编码器初始化 MSP 回调
 * @note  在这里配置编码器使用的 GPIO 引脚，直接引用 board_v1_config.h 的宏。
 *        根据定时器实例判断是左编码器（TIM2）还是右编码器（TIM3）。
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    if (htim->Instance == TIM2) {
        /* 左编码器：PA0(TIM2_CH1), PA1(TIM2_CH2) */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        gpio.Pin   = BSP_ENC_A_CH1_PIN | BSP_ENC_A_CH2_PIN;
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(BSP_ENC_A_CH1_PORT, &gpio);

        /* 中断已在 _config_nvic 中使能，此处不再重复使能 */
    } else if (htim->Instance == TIM3) {
        /* 右编码器：PA6(TIM3_CH1), PA7(TIM3_CH2) */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        gpio.Pin   = BSP_ENC_B_CH1_PIN | BSP_ENC_B_CH2_PIN;
        gpio.Mode  = GPIO_MODE_INPUT;
        gpio.Pull  = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(BSP_ENC_B_CH1_PORT, &gpio);
    }
    /* 其他定时器（如 TIM4）若有编码器需求可添加 */
}

/**
 * @brief HAL PWM 初始化 MSP 回调
 * @note  配置 PWM 输出引脚（TIM1_CH1/CH2 用于电机控制）
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    if (htim->Instance == TIM1) {
        /* TIM1_CH1 (PA8), TIM1_CH2 (PA9) 用于双电机 PWM */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* 配置 CH1 (PA8) */
        gpio.Pin   = BSP_TIM1_CH1_PIN;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(BSP_TIM1_CH1_PORT, &gpio);

        /* 配置 CH2 (PA9) */
        gpio.Pin   = BSP_TIM1_CH2_PIN;
        HAL_GPIO_Init(BSP_TIM1_CH2_PORT, &gpio);
    }
    /* 若其他定时器用于 PWM，可继续添加分支 */
}

/* ========================================================================== */
/*                      中断处理                                                */
/* ========================================================================== */

static void _timer_irq_handler(bsp_timer_inst_t *inst)
{
    TIM_HandleTypeDef *htim = &inst->htim;

    if (inst->config.mode == BSP_TIMER_MODE_ENCODER) {
        if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) &&
            __HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE)) {
            __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
            if (htim->Instance->CR1 & TIM_CR1_DIR) {
                inst->enc_accum -= 65536;
            } else {
                inst->enc_accum += 65536;
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    for (int i = 0; i < BSP_TIMER_MAX_INSTANCES; i++) {
        if (s_timers[i].in_use && s_timers[i].tim_periph == htim->Instance) {
            _timer_irq_handler(&s_timers[i]);
            return;
        }
    }
}

void TIM1_UP_IRQHandler(void) { if (s_timers[0].in_use) HAL_TIM_IRQHandler(&s_timers[0].htim); }
void TIM2_IRQHandler(void)    { if (s_timers[1].in_use) HAL_TIM_IRQHandler(&s_timers[1].htim); }
void TIM3_IRQHandler(void)    { if (s_timers[2].in_use) HAL_TIM_IRQHandler(&s_timers[2].htim); }
void TIM4_IRQHandler(void)    { if (s_timers[3].in_use) HAL_TIM_IRQHandler(&s_timers[3].htim); }

/* ========================================================================== */
/*                        DWT 微秒延时                                         */
/* ========================================================================== */

static void _dwt_init(void)
{
    if (!s_dwt_inited) {
        BSP_DEMCR |= BSP_DEMCR_TRCENA;
        BSP_DWT_CYCCNT = 0;
        BSP_DWT_CTRL |= BSP_DWT_CTRL_EN;
        s_dwt_inited = true;
    }
}

void bsp_timer_delay_us(uint32_t us)
{
    if (us == 0) return;
    _dwt_init();
    uint32_t ticks = us * (BSP_TIMER_APB_CLK_HZ / 1000000U);
    uint32_t start = BSP_DWT_CYCCNT;
    while ((BSP_DWT_CYCCNT - start) < ticks) {}
}

void bsp_timer_delay_ms(uint32_t ms)
{
    if (ms == 0) return;
#ifdef BSP_USE_RTOS
    extern void osDelay(uint32_t ticks);
    osDelay(ms);
#else
    while (ms >= 1000U) {
        bsp_timer_delay_us(1000000U);
        ms -= 1000U;
    }
    if (ms > 0) bsp_timer_delay_us(ms * 1000U);
#endif
}

/* ========================================================================== */
/*                         公共 API                                            */
/* ========================================================================== */

bsp_err_t bsp_timer_open(uint8_t id, const bsp_timer_config_t *cfg,
                         bsp_timer_handle_t *handle)
{
    if (cfg == NULL || handle == NULL) return BSP_ERR_PARAM;
    if (id < 1 || id > BSP_TIMER_MAX_INSTANCES) return BSP_ERR_PARAM;

    uint8_t idx = id - 1;
    bsp_timer_inst_t *inst = &s_timers[idx];
    if (inst->in_use) return BSP_ERR_BUSY;

    memset(inst, 0, sizeof(bsp_timer_inst_t));
    inst->id           = id;
    inst->in_use       = true;
    inst->config       = *cfg;
    inst->tim_periph   = s_hw_map[idx].periph;
    inst->irqn         = s_hw_map[idx].irqn;
    inst->timer_clk_hz = BSP_TIMER_APB_CLK_HZ;
    inst->irq_enabled  = cfg->enable_irq;

    _enable_periph_clock(id);

    bsp_err_t ret = BSP_OK;
    uint32_t psc = 0, arr = 0;

    switch (cfg->mode) {
        case BSP_TIMER_MODE_PWM: {
            uint32_t freq = (cfg->freq_hz == BSP_TIMER_FREQ_DEFAULT) ? 1000U : cfg->freq_hz;
            ret = _calc_prescaler_arr(inst->timer_clk_hz, freq, 0, &psc, &arr);
            if (ret != BSP_OK) break;
            inst->prescaler    = psc;
            inst->period_ticks = arr;
            ret = _config_pwm_mode(inst, psc, arr);
            break;
        }
        case BSP_TIMER_MODE_ENCODER:
            ret = _config_encoder_mode(inst);
            break;
        default:
            ret = BSP_ERR_PARAM;
            break;
    }

    if (ret != BSP_OK) {
        _disable_periph_clock(id);
        inst->in_use = false;
        return ret;
    }

    _config_nvic(inst);
    *handle = (bsp_timer_handle_t)inst;
    return BSP_OK;
}

bsp_err_t bsp_timer_close(bsp_timer_handle_t handle)
{
    if (!_handle_valid(handle)) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;

    if (inst->running) bsp_timer_stop(handle);

    if (inst->config.mode == BSP_TIMER_MODE_PWM) {
        HAL_TIM_PWM_DeInit(&inst->htim);
    } else {
        HAL_TIM_Encoder_DeInit(&inst->htim);
    }

    _disable_nvic(inst);
    _disable_periph_clock(inst->id);
    inst->callback     = NULL;
    inst->callback_arg = NULL;
    inst->in_use       = false;
    return BSP_OK;
}

/**
 * @brief 启动定时器
 * @note  PWM 模式下若传入 period_us > 0 修改频率，
 *        采用 stop → reconfig → restart 策略避免运行时改频毛刺。
 *        仅修改占空比请使用 bsp_timer_pwm_set_duty()。
 */
bsp_err_t bsp_timer_start(bsp_timer_handle_t handle,
                          uint32_t period_us, uint8_t duty_pct)
{
    if (!_handle_valid(handle)) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;

    if (inst->config.mode == BSP_TIMER_MODE_PWM) {
        if (duty_pct > 100) return BSP_ERR_PARAM;

        /* 动态改频：先停再改再启，避免毛刺 */
        if (period_us > 0) {
            if (inst->running) {
                HAL_TIM_PWM_Stop(&inst->htim, TIM_CHANNEL_1);
                inst->running = false;
            }
            uint32_t psc, arr;
            bsp_err_t ret = _calc_prescaler_arr(inst->timer_clk_hz, 0, period_us, &psc, &arr);
            if (ret != BSP_OK) return BSP_ERR_PARAM;
            __HAL_TIM_SET_PRESCALER(&inst->htim, psc);
            __HAL_TIM_SET_AUTORELOAD(&inst->htim, arr);
            inst->prescaler    = psc;
            inst->period_ticks = arr;
        }

        uint32_t ccr = (uint32_t)((uint64_t)(inst->period_ticks + 1U) * duty_pct / 100U);
        inst->htim.Instance->CCR1 = ccr;

        if (HAL_TIM_PWM_Start(&inst->htim, TIM_CHANNEL_1) != HAL_OK) return BSP_ERR_IO;
        inst->running = true;
        return BSP_OK;
    }

    /* 编码器模式 */
    inst->enc_accum    = 0;
    inst->enc_last_cnt = 0;
    __HAL_TIM_SET_COUNTER(&inst->htim, 0);

    if (HAL_TIM_Encoder_Start(&inst->htim, TIM_CHANNEL_ALL) != HAL_OK) return BSP_ERR_IO;

    /* 使能溢出中断用于 32 位累加 */
    __HAL_TIM_ENABLE_IT(&inst->htim, TIM_IT_UPDATE);
    inst->running = true;
    return BSP_OK;
}

bsp_err_t bsp_timer_stop(bsp_timer_handle_t handle)
{
    if (!_handle_valid(handle)) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (!inst->running) return BSP_OK;

    if (inst->config.mode == BSP_TIMER_MODE_PWM) {
        HAL_TIM_PWM_Stop(&inst->htim, TIM_CHANNEL_1);
    } else {
        __HAL_TIM_DISABLE_IT(&inst->htim, TIM_IT_UPDATE);
        HAL_TIM_Encoder_Stop(&inst->htim, TIM_CHANNEL_ALL);
    }

    inst->running = false;

    if (inst->irq_enabled && inst->callback != NULL) {
        bsp_timer_evt_info_t info = {0};
        info.event = BSP_TIMER_EVT_ABORT;
        inst->callback(handle, &info, inst->callback_arg);
    }
    return BSP_OK;
}

bsp_err_t bsp_timer_set_callback(bsp_timer_handle_t handle,
                                 bsp_timer_callback_t cb, void *user_data)
{
    if (!_handle_valid(handle)) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    inst->callback     = cb;
    inst->callback_arg = user_data;
    return BSP_OK;
}

bsp_err_t bsp_timer_is_running(bsp_timer_handle_t handle, bool *running)
{
    if (!_handle_valid(handle) || running == NULL) return BSP_ERR_PARAM;
    *running = ((bsp_timer_inst_t *)handle)->running;
    return BSP_OK;
}

bsp_err_t bsp_timer_get_count(bsp_timer_handle_t handle, uint32_t *count)
{
    if (!_handle_valid(handle) || count == NULL) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (inst->config.mode == BSP_TIMER_MODE_ENCODER) return BSP_ERR_UNSUPPORT;
    *count = __HAL_TIM_GET_COUNTER(&inst->htim);
    return BSP_OK;
}

/* ========================================================================== */
/*                          PWM 专用接口                                        */
/* ========================================================================== */

bsp_err_t bsp_timer_pwm_get_freq(bsp_timer_handle_t handle, uint32_t *freq_hz)
{
    if (!_handle_valid(handle) || freq_hz == NULL) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (inst->config.mode != BSP_TIMER_MODE_PWM) return BSP_ERR_UNSUPPORT;

    uint32_t psc = inst->htim.Instance->PSC;
    uint32_t arr = inst->htim.Instance->ARR;
    if ((psc + 1U) == 0 || (arr + 1U) == 0) return BSP_ERR_IO;
    *freq_hz = inst->timer_clk_hz / ((psc + 1U) * (arr + 1U));
    return BSP_OK;
}

bsp_err_t bsp_timer_pwm_set_duty(bsp_timer_handle_t handle,
                                 uint8_t channel, uint8_t duty_pct)
{
    if (!_handle_valid(handle)) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (inst->config.mode != BSP_TIMER_MODE_PWM) return BSP_ERR_UNSUPPORT;
    if (channel < 1 || channel > 4 || duty_pct > 100) return BSP_ERR_PARAM;

    uint32_t ccr = (uint32_t)((uint64_t)(inst->period_ticks + 1U) * duty_pct / 100U);
    switch (channel) {
        case 1: inst->htim.Instance->CCR1 = ccr; break;
        case 2: inst->htim.Instance->CCR2 = ccr; break;
        case 3: inst->htim.Instance->CCR3 = ccr; break;
        case 4: inst->htim.Instance->CCR4 = ccr; break;
        default: return BSP_ERR_PARAM;
    }
    return BSP_OK;
}

bsp_err_t bsp_timer_pwm_start_channel(bsp_timer_handle_t handle, uint8_t channel)
{
    if (!_handle_valid(handle)) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (inst->config.mode != BSP_TIMER_MODE_PWM) return BSP_ERR_UNSUPPORT;
    if (channel < 1 || channel > 4) return BSP_ERR_PARAM;
    if (HAL_TIM_PWM_Start(&inst->htim, s_hal_ch_map[channel - 1]) != HAL_OK) return BSP_ERR_IO;
    return BSP_OK;
}

bsp_err_t bsp_timer_pwm_start_all(bsp_timer_handle_t handle)
{
    if (!_handle_valid(handle)) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (inst->config.mode != BSP_TIMER_MODE_PWM) return BSP_ERR_UNSUPPORT;

    for (uint8_t ch = 0; ch < 4; ch++) {
        if (HAL_TIM_PWM_Start(&inst->htim, s_hal_ch_map[ch]) != HAL_OK) {
            return BSP_ERR_IO;
        }
    }
    return BSP_OK;
}

/* ========================================================================== */
/*                        编码器专用接口                                        */
/* ========================================================================== */

bsp_err_t bsp_timer_encoder_get_count(bsp_timer_handle_t handle, int32_t *count)
{
    if (!_handle_valid(handle) || count == NULL) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (inst->config.mode != BSP_TIMER_MODE_ENCODER) return BSP_ERR_UNSUPPORT;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t hw_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&inst->htim);
    int32_t delta = (int32_t)(int16_t)(hw_cnt - inst->enc_last_cnt);
    inst->enc_last_cnt = hw_cnt;
    inst->enc_accum += delta;

    *count = inst->enc_accum;
    __set_PRIMASK(primask);
    return BSP_OK;
}

bsp_err_t bsp_timer_encoder_reset(bsp_timer_handle_t handle)
{
    if (!_handle_valid(handle)) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (inst->config.mode != BSP_TIMER_MODE_ENCODER) return BSP_ERR_UNSUPPORT;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_SET_COUNTER(&inst->htim, 0);
    inst->enc_accum    = 0;
    inst->enc_last_cnt = 0;
    __set_PRIMASK(primask);
    return BSP_OK;
}

bsp_err_t bsp_timer_encoder_get_bit_width(bsp_timer_handle_t handle, uint8_t *bits)
{
    if (!_handle_valid(handle) || bits == NULL) return BSP_ERR_PARAM;
    bsp_timer_inst_t *inst = (bsp_timer_inst_t *)handle;
    if (inst->config.mode != BSP_TIMER_MODE_ENCODER) return BSP_ERR_UNSUPPORT;
    *bits = 16;
    return BSP_OK;
}