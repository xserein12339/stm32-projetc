/**
 * @file bsp_stm32_rcc.c
 * @brief RCC HAL 接口的 STM32F1 BSP 静态实现
 * 
 * @details STM32F1 关键适配点：
 *          - HSE 频率可配置（默认 8MHz），HSI 为 8MHz
 *          - PLL 输出频率范围：16MHz ~ 72MHz
 *          - AHB/APB 分频系数由 BSP 内部计算
 *          - 外设时钟使能和复位通过 RCC 寄存器位操作实现
 *          - APB 定时器自动倍频（APB 预分频 ≠ 1 时 ×2）
 * 
 * @author xserein
 * @version v1.2
 */

#include "hal_rcc.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stddef.h>

/* ========================================================================== */
/*                           内部宏与常量定义                                    */
/* ========================================================================== */

/** HSE 外部晶振频率（可根据板级修改） */
#ifndef BSP_RCC_HSE_FREQ
#define BSP_RCC_HSE_FREQ           8000000U   /* 8MHz */
#endif

/** HSI 内部 RC 频率 */
#define BSP_RCC_HSI_FREQ           8000000U

/** 默认系统配置（当 config 为 NULL 或字段为 0 时使用） */
#define BSP_RCC_DEFAULT_SYSCLK     72000000U
#define BSP_RCC_DEFAULT_AHB        BSP_RCC_DEFAULT_SYSCLK
#define BSP_RCC_DEFAULT_APB1       36000000U
#define BSP_RCC_DEFAULT_APB2       72000000U

/** 允许的 PLL 倍频范围 */
#define BSP_RCC_PLL_MIN            16000000U
#define BSP_RCC_PLL_MAX            72000000U

/** 允许的 APB1 最大频率 */
#define BSP_RCC_APB1_MAX           36000000U

/** 临界区保护宏 */
#define CRITICAL_ENTER()           uint32_t _primask = __get_PRIMASK(); __disable_irq()
#define CRITICAL_EXIT()            __set_PRIMASK(_primask)


/* ========================================================================== */
/*                     外设标识符定义（供应用层使用）                             */
/* ========================================================================== */

typedef enum {
    BSP_RCC_PERIPH_GPIOA = 0,
    BSP_RCC_PERIPH_GPIOB,
    BSP_RCC_PERIPH_GPIOC,
    BSP_RCC_PERIPH_GPIOD,
    BSP_RCC_PERIPH_GPIOE,
    BSP_RCC_PERIPH_GPIOF,
    BSP_RCC_PERIPH_GPIOG,
    BSP_RCC_PERIPH_AFIO,
    BSP_RCC_PERIPH_USART1,
    BSP_RCC_PERIPH_USART2,
    BSP_RCC_PERIPH_USART3,
    BSP_RCC_PERIPH_I2C1,
    BSP_RCC_PERIPH_I2C2,
    BSP_RCC_PERIPH_SPI1,
    BSP_RCC_PERIPH_SPI2,
    BSP_RCC_PERIPH_TIM1,
    BSP_RCC_PERIPH_TIM2,
    BSP_RCC_PERIPH_TIM3,
    BSP_RCC_PERIPH_TIM4,
    BSP_RCC_PERIPH_DMA1,
    BSP_RCC_PERIPH_DMA2,
    BSP_RCC_PERIPH_ADC1,
    BSP_RCC_PERIPH_ADC2,
    BSP_RCC_PERIPH_FLASH,
    BSP_RCC_PERIPH_CRC,
    BSP_RCC_PERIPH_COUNT
} bsp_rcc_periph_index_t;

#define HAL_RCC_PERIPH_GPIOA       ((hal_rcc_periph_t)BSP_RCC_PERIPH_GPIOA)
#define HAL_RCC_PERIPH_GPIOB       ((hal_rcc_periph_t)BSP_RCC_PERIPH_GPIOB)
#define HAL_RCC_PERIPH_GPIOC       ((hal_rcc_periph_t)BSP_RCC_PERIPH_GPIOC)
#define HAL_RCC_PERIPH_GPIOD       ((hal_rcc_periph_t)BSP_RCC_PERIPH_GPIOD)
#define HAL_RCC_PERIPH_GPIOE       ((hal_rcc_periph_t)BSP_RCC_PERIPH_GPIOE)
#define HAL_RCC_PERIPH_GPIOF       ((hal_rcc_periph_t)BSP_RCC_PERIPH_GPIOF)
#define HAL_RCC_PERIPH_GPIOG       ((hal_rcc_periph_t)BSP_RCC_PERIPH_GPIOG)
#define HAL_RCC_PERIPH_AFIO        ((hal_rcc_periph_t)BSP_RCC_PERIPH_AFIO)
#define HAL_RCC_PERIPH_USART1      ((hal_rcc_periph_t)BSP_RCC_PERIPH_USART1)
#define HAL_RCC_PERIPH_USART2      ((hal_rcc_periph_t)BSP_RCC_PERIPH_USART2)
#define HAL_RCC_PERIPH_USART3      ((hal_rcc_periph_t)BSP_RCC_PERIPH_USART3)
#define HAL_RCC_PERIPH_I2C1        ((hal_rcc_periph_t)BSP_RCC_PERIPH_I2C1)
#define HAL_RCC_PERIPH_I2C2        ((hal_rcc_periph_t)BSP_RCC_PERIPH_I2C2)
#define HAL_RCC_PERIPH_SPI1        ((hal_rcc_periph_t)BSP_RCC_PERIPH_SPI1)
#define HAL_RCC_PERIPH_SPI2        ((hal_rcc_periph_t)BSP_RCC_PERIPH_SPI2)
#define HAL_RCC_PERIPH_TIM1        ((hal_rcc_periph_t)BSP_RCC_PERIPH_TIM1)
#define HAL_RCC_PERIPH_TIM2        ((hal_rcc_periph_t)BSP_RCC_PERIPH_TIM2)
#define HAL_RCC_PERIPH_TIM3        ((hal_rcc_periph_t)BSP_RCC_PERIPH_TIM3)
#define HAL_RCC_PERIPH_TIM4        ((hal_rcc_periph_t)BSP_RCC_PERIPH_TIM4)
#define HAL_RCC_PERIPH_DMA1        ((hal_rcc_periph_t)BSP_RCC_PERIPH_DMA1)
#define HAL_RCC_PERIPH_DMA2        ((hal_rcc_periph_t)BSP_RCC_PERIPH_DMA2)
#define HAL_RCC_PERIPH_ADC1        ((hal_rcc_periph_t)BSP_RCC_PERIPH_ADC1)
#define HAL_RCC_PERIPH_ADC2        ((hal_rcc_periph_t)BSP_RCC_PERIPH_ADC2)
#define HAL_RCC_PERIPH_FLASH       ((hal_rcc_periph_t)BSP_RCC_PERIPH_FLASH)
#define HAL_RCC_PERIPH_CRC         ((hal_rcc_periph_t)BSP_RCC_PERIPH_CRC)


/* ========================================================================== */
/*                            内部数据结构                                      */
/* ========================================================================== */

typedef struct {
    uint8_t     bus_type;   /**< 0=AHB, 1=APB1, 2=APB2 */
    uint8_t     bit_pos;    /**< 使能/复位寄存器中的位号 */
    uint8_t     apb_timer;  /**< 1=APB 定时器（需倍频检查） */
} bsp_rcc_periph_map_t;

static struct {
    bool                    initialized;
    hal_rcc_config_t        active_config;
    uint32_t                ahb_freq;
    uint32_t                apb1_freq;
    uint32_t                apb2_freq;
    uint32_t                periph_freq_cache[BSP_RCC_PERIPH_COUNT];
} g_rcc = {0};

/**
 * @brief 外设映射表（索引必须与 bsp_rcc_periph_index_t 一致）
 * 
 * STM32F1 寄存器位参考：
 * - RCC_AHBENR:  DMA1(0), DMA2(1), SRAM(2), FLITF(4), CRC(6)
 * - RCC_APB1ENR: TIM2(0), TIM3(1), TIM4(2), WWDG(11), SPI2(14),
 *                 USART2(17), USART3(18), I2C1(21), I2C2(22)
 * - RCC_APB2ENR: AFIO(0), GPIOA(2), GPIOB(3), GPIOC(4), GPIOD(5),
 *                 GPIOE(6), GPIOF(7), GPIOG(8), ADC1(9), ADC2(10),
 *                 TIM1(11), SPI1(12), USART1(14)
 */
static const bsp_rcc_periph_map_t g_periph_map[BSP_RCC_PERIPH_COUNT] = {
    /* GPIOA  */ { .bus_type = 2, .bit_pos = 2,  .apb_timer = 0 },
    /* GPIOB  */ { .bus_type = 2, .bit_pos = 3,  .apb_timer = 0 },
    /* GPIOC  */ { .bus_type = 2, .bit_pos = 4,  .apb_timer = 0 },
    /* GPIOD  */ { .bus_type = 2, .bit_pos = 5,  .apb_timer = 0 },
    /* GPIOE  */ { .bus_type = 2, .bit_pos = 6,  .apb_timer = 0 },
    /* GPIOF  */ { .bus_type = 2, .bit_pos = 7,  .apb_timer = 0 },
    /* GPIOG  */ { .bus_type = 2, .bit_pos = 8,  .apb_timer = 0 },
    /* AFIO   */ { .bus_type = 2, .bit_pos = 0,  .apb_timer = 0 },
    /* USART1 */ { .bus_type = 2, .bit_pos = 14, .apb_timer = 0 },
    /* USART2 */ { .bus_type = 1, .bit_pos = 17, .apb_timer = 0 },
    /* USART3 */ { .bus_type = 1, .bit_pos = 18, .apb_timer = 0 },
    /* I2C1   */ { .bus_type = 1, .bit_pos = 21, .apb_timer = 0 },
    /* I2C2   */ { .bus_type = 1, .bit_pos = 22, .apb_timer = 0 },
    /* SPI1   */ { .bus_type = 2, .bit_pos = 12, .apb_timer = 0 },
    /* SPI2   */ { .bus_type = 1, .bit_pos = 14, .apb_timer = 0 },
    /* TIM1   */ { .bus_type = 2, .bit_pos = 11, .apb_timer = 1 },
    /* TIM2   */ { .bus_type = 1, .bit_pos = 0,  .apb_timer = 1 },
    /* TIM3   */ { .bus_type = 1, .bit_pos = 1,  .apb_timer = 1 },
    /* TIM4   */ { .bus_type = 1, .bit_pos = 2,  .apb_timer = 1 },
    /* DMA1   */ { .bus_type = 0, .bit_pos = 0,  .apb_timer = 0 },
    /* DMA2   */ { .bus_type = 0, .bit_pos = 1,  .apb_timer = 0 },
    /* ADC1   */ { .bus_type = 2, .bit_pos = 9,  .apb_timer = 0 },
    /* ADC2   */ { .bus_type = 2, .bit_pos = 10, .apb_timer = 0 },
    /* FLASH  */ { .bus_type = 0, .bit_pos = 4,  .apb_timer = 0 },
    /* CRC    */ { .bus_type = 0, .bit_pos = 6,  .apb_timer = 0 },
};


/* ========================================================================== */
/*                         内部辅助函数(私有)                                    */
/* ========================================================================== */

static inline bool is_valid_periph(hal_rcc_periph_t periph)
{
    return (periph != HAL_RCC_PERIPH_NONE) && (periph < BSP_RCC_PERIPH_COUNT);
}

static inline const bsp_rcc_periph_map_t* get_periph_map(hal_rcc_periph_t periph)
{
    if (!is_valid_periph(periph)) return NULL;
    return &g_periph_map[periph];
}

static inline uint32_t div_to_hal_rcc(uint32_t div)
{
    switch (div) {
        case 1:  return RCC_HCLK_DIV1;
        case 2:  return RCC_HCLK_DIV2;
        case 4:  return RCC_HCLK_DIV4;
        case 8:  return RCC_HCLK_DIV8;
        case 16: return RCC_HCLK_DIV16;
        default: return RCC_HCLK_DIV1;
    }
}

static int calc_prescaler(uint32_t freq, uint32_t src_freq, uint32_t max_div, uint32_t *div)
{
    if (freq == 0 || src_freq % freq != 0) return -1;
    uint32_t d = src_freq / freq;
    if (d > max_div || (d & (d - 1)) != 0) return -1;
    *div = d;
    return 0;
}

static int calc_pll_multiplier(uint32_t sysclk, uint32_t *pll_mul)
{
    uint32_t pll_in = BSP_RCC_HSE_FREQ;
    if (sysclk < BSP_RCC_PLL_MIN || sysclk > BSP_RCC_PLL_MAX) return -1;
    if (sysclk % pll_in != 0) return -1;
    uint32_t mul = sysclk / pll_in;
    if (mul < 2 || mul > 16) return -1;
    *pll_mul = mul;
    return 0;
}

static inline uint32_t pll_mul_to_hal(uint32_t mul)
{
    switch (mul) {
        case 2:  return RCC_PLL_MUL2;
        case 3:  return RCC_PLL_MUL3;
        case 4:  return RCC_PLL_MUL4;
        case 5:  return RCC_PLL_MUL5;
        case 6:  return RCC_PLL_MUL6;
        case 7:  return RCC_PLL_MUL7;
        case 8:  return RCC_PLL_MUL8;
        case 9:  return RCC_PLL_MUL9;
        case 10: return RCC_PLL_MUL10;
        case 11: return RCC_PLL_MUL11;
        case 12: return RCC_PLL_MUL12;
        case 13: return RCC_PLL_MUL13;
        case 14: return RCC_PLL_MUL14;
        case 15: return RCC_PLL_MUL15;
        case 16: return RCC_PLL_MUL16;
        default: return RCC_PLL_MUL2;
    }
}

static inline uint32_t get_flash_latency(uint32_t sysclk)
{
    if (sysclk > 48000000U) return FLASH_LATENCY_2;
    if (sysclk > 24000000U) return FLASH_LATENCY_1;
    return FLASH_LATENCY_0;
}

/**
 * @brief 填充默认配置到局部结构体
 */
static void fill_default_config(hal_rcc_config_t *out)
{
    out->sysclk_src = HAL_RCC_SRC_PLL;
    out->sysclk_hz  = BSP_RCC_DEFAULT_SYSCLK;
    out->ahb_hz     = BSP_RCC_DEFAULT_AHB;
    out->apb1_hz    = BSP_RCC_DEFAULT_APB1;
    out->apb2_hz    = BSP_RCC_DEFAULT_APB2;
}

/**
 * @brief 解析并校验应用层配置，输出最终生效的配置参数
 * @param config    应用层传入配置（可为 NULL）
 * @param out       输出解析后的完整配置
 * @param sysclk    输出最终系统时钟
 * @param ahb       输出最终 AHB 频率
 * @param apb1      输出最终 APB1 频率
 * @param apb2      输出最终 APB2 频率
 * @return HAL_SUCCESS 或错误码
 */
static hal_err_t resolve_config(const hal_rcc_config_t *config,
                                hal_rcc_config_t *out,
                                uint32_t *sysclk,
                                uint32_t *ahb,
                                uint32_t *apb1,
                                uint32_t *apb2)
{
    if (config == NULL) {
        fill_default_config(out);
    } else {
        *out = *config;
        if (out->sysclk_hz == 0) out->sysclk_hz = BSP_RCC_DEFAULT_SYSCLK;
        if (out->ahb_hz    == 0) out->ahb_hz    = BSP_RCC_DEFAULT_AHB;
        if (out->apb1_hz   == 0) out->apb1_hz   = BSP_RCC_DEFAULT_APB1;
        if (out->apb2_hz   == 0) out->apb2_hz   = BSP_RCC_DEFAULT_APB2;
    }

    *sysclk = out->sysclk_hz;
    *ahb    = out->ahb_hz;
    *apb1   = out->apb1_hz;
    *apb2   = out->apb2_hz;

    if (*ahb != *sysclk) return HAL_ERR_INVALID_PARAM;
    if (*apb1 > BSP_RCC_APB1_MAX) return HAL_RCC_ERR_BUS_FREQ;
    if (*apb2 > *sysclk) return HAL_RCC_ERR_BUS_FREQ;
    if (*sysclk % *apb1 != 0 || *sysclk % *apb2 != 0) return HAL_ERR_INVALID_PARAM;

    return HAL_SUCCESS;
}

/**
 * @brief 更新所有外设的频率缓存
 */
static void update_periph_freq_cache(void)
{
    for (uint32_t i = 0; i < BSP_RCC_PERIPH_COUNT; i++) {
        const bsp_rcc_periph_map_t *map = &g_periph_map[i];
        uint32_t base_freq = 0;

        if (map->bus_type == 0) {
            base_freq = g_rcc.ahb_freq;
        } else if (map->bus_type == 1) {
            base_freq = g_rcc.apb1_freq;
        } else if (map->bus_type == 2) {
            base_freq = g_rcc.apb2_freq;
        }

        /* APB 定时器倍频：若 APB 预分频 ≠ 1，定时器时钟 = APB × 2 */
        if (map->apb_timer) {
            uint32_t div = (map->bus_type == 1)
                         ? (g_rcc.ahb_freq / g_rcc.apb1_freq)
                         : (g_rcc.ahb_freq / g_rcc.apb2_freq);
            if (div != 1) {
                base_freq *= 2;
            }
        }
        g_rcc.periph_freq_cache[i] = base_freq;
    }
}

/**
 * @brief 根据解析后的配置填充 STM32 HAL 时钟结构体
 */
static hal_err_t build_hal_clock_config(const hal_rcc_config_t *resolved,
                                        uint32_t div_apb1,
                                        uint32_t div_apb2,
                                        RCC_OscInitTypeDef *osc,
                                        RCC_ClkInitTypeDef *clk)
{
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    switch (resolved->sysclk_src) {
        case HAL_RCC_SRC_HSE:
            if (resolved->sysclk_hz != BSP_RCC_HSE_FREQ) {
                return HAL_RCC_ERR_FREQ_INVALID;
            }
            osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
            osc_init.HSEState = RCC_HSE_ON;
            osc_init.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
            clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
            break;

        case HAL_RCC_SRC_HSI:
            if (resolved->sysclk_hz != BSP_RCC_HSI_FREQ) {
                return HAL_RCC_ERR_FREQ_INVALID;
            }
            osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
            osc_init.HSIState = RCC_HSI_ON;
            osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
            clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
            break;

        case HAL_RCC_SRC_PLL:
        {
            uint32_t pll_mul;
            if (calc_pll_multiplier(resolved->sysclk_hz, &pll_mul) != 0) {
                return HAL_RCC_ERR_FREQ_INVALID;
            }
            osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_PLL;
            osc_init.HSEState = RCC_HSE_ON;
            osc_init.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
            osc_init.PLL.PLLState = RCC_PLL_ON;
            osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
            osc_init.PLL.PLLMUL = pll_mul_to_hal(pll_mul);
            clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
            break;
        }

        default:
            return HAL_ERR_INVALID_PARAM;
    }

    clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = div_to_hal_rcc(div_apb1);
    clk_init.APB2CLKDivider = div_to_hal_rcc(div_apb2);

    *osc = osc_init;
    *clk = clk_init;
    return HAL_SUCCESS;
}


/* ========================================================================== */
/*                     HAL 公共 API 实现 (Public API)                           */
/* ========================================================================== */

hal_err_t hal_rcc_init(const hal_rcc_config_t *config)
{
    if (g_rcc.initialized) {
        return HAL_ERR_ALREADY_INIT;
    }

    hal_rcc_config_t resolved;
    uint32_t sysclk, ahb, apb1, apb2;
    uint32_t div_apb1, div_apb2;

    hal_err_t err = resolve_config(config, &resolved, &sysclk, &ahb, &apb1, &apb2);
    if (err != HAL_SUCCESS) return err;

    if (calc_prescaler(apb1, sysclk, 16, &div_apb1) != 0) return HAL_ERR_INVALID_PARAM;
    if (calc_prescaler(apb2, sysclk, 16, &div_apb2) != 0) return HAL_ERR_INVALID_PARAM;

    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};
    err = build_hal_clock_config(&resolved, div_apb1, div_apb2, &osc_init, &clk_init);
    if (err != HAL_SUCCESS) return err;

    /* HAL 时钟配置可能涉及超时等待，不在临界区内调用 */
    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
        if (resolved.sysclk_src == HAL_RCC_SRC_HSE) {
            return HAL_RCC_ERR_HSE_FAIL;
        }
        if (resolved.sysclk_src == HAL_RCC_SRC_PLL) {
            return HAL_RCC_ERR_PLL_LOCK;
        }
        return HAL_ERR_TIMEOUT;
    }

    uint32_t latency = get_flash_latency(sysclk);
    if (HAL_RCC_ClockConfig(&clk_init, latency) != HAL_OK) {
        return HAL_ERR_IO;
    }

    /* 更新全局状态（需临界区保护） */
    CRITICAL_ENTER();
    g_rcc.active_config = resolved;
    g_rcc.ahb_freq  = HAL_RCC_GetHCLKFreq();
    g_rcc.apb1_freq = HAL_RCC_GetPCLK1Freq();
    g_rcc.apb2_freq = HAL_RCC_GetPCLK2Freq();
    update_periph_freq_cache();
    g_rcc.initialized = true;
    CRITICAL_EXIT();

    return HAL_SUCCESS;
}

hal_err_t hal_rcc_deinit(void)
{
    if (!g_rcc.initialized) {
        return HAL_ERR_NOT_INITIALIZED;
    }

    /* 1. 先切到 HSI（关闭 PLL/HSE 前必须切换时钟源，否则系统崩溃） */
    RCC_OscInitTypeDef osc_hsi = {0};
    osc_hsi.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc_hsi.HSIState = RCC_HSI_ON;
    osc_hsi.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    if (HAL_RCC_OscConfig(&osc_hsi) != HAL_OK) {
        return HAL_ERR_TIMEOUT;
    }

    RCC_ClkInitTypeDef clk_hsi = {0};
    clk_hsi.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_hsi.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk_hsi.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_hsi.APB1CLKDivider = RCC_HCLK_DIV1;
    clk_hsi.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk_hsi, FLASH_LATENCY_0) != HAL_OK) {
        return HAL_ERR_IO;
    }

    /* 2. 关闭 PLL 和 HSE */
    RCC_OscInitTypeDef osc_off = {0};
    osc_off.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_PLL;
    osc_off.HSEState = RCC_HSE_OFF;
    osc_off.PLL.PLLState = RCC_PLL_OFF;
    HAL_RCC_OscConfig(&osc_off);

    /* 3. 清空全局状态 */
    CRITICAL_ENTER();
    memset(&g_rcc, 0, sizeof(g_rcc));
    CRITICAL_EXIT();

    return HAL_SUCCESS;
}

hal_err_t hal_rcc_periph_enable(hal_rcc_periph_t periph, bool enable)
{
    if (!g_rcc.initialized) {
        return HAL_ERR_NOT_INITIALIZED;
    }
    const bsp_rcc_periph_map_t *map = get_periph_map(periph);
    if (map == NULL) {
        return HAL_ERR_INVALID_PARAM;
    }

    uint32_t bit_mask = 1U << map->bit_pos;

    if (enable) {
        switch (map->bus_type) {
            case 0:  SET_BIT(RCC->AHBENR, bit_mask);  break;
            case 1:  SET_BIT(RCC->APB1ENR, bit_mask); break;
            case 2:  SET_BIT(RCC->APB2ENR, bit_mask); break;
            default: return HAL_ERR_INVALID_PARAM;
        }
    } else {
        switch (map->bus_type) {
            case 0:  CLEAR_BIT(RCC->AHBENR, bit_mask);  break;
            case 1:  CLEAR_BIT(RCC->APB1ENR, bit_mask); break;
            case 2:  CLEAR_BIT(RCC->APB2ENR, bit_mask); break;
            default: return HAL_ERR_INVALID_PARAM;
        }
    }
    return HAL_SUCCESS;
}

bool hal_rcc_periph_is_enabled(hal_rcc_periph_t periph)
{
    if (!g_rcc.initialized) return false;
    const bsp_rcc_periph_map_t *map = get_periph_map(periph);
    if (map == NULL) return false;

    uint32_t reg_val;
    switch (map->bus_type) {
        case 0:  reg_val = RCC->AHBENR;  break;
        case 1:  reg_val = RCC->APB1ENR; break;
        case 2:  reg_val = RCC->APB2ENR; break;
        default: return false;
    }
    return (reg_val & (1U << map->bit_pos)) != 0;
}

uint32_t hal_rcc_get_freq(hal_rcc_bus_t bus)
{
    if (!g_rcc.initialized) return 0;

    switch (bus) {
        case HAL_RCC_BUS_AHB:   return g_rcc.ahb_freq;
        case HAL_RCC_BUS_AHB2:  return 0;  /* F1 无 AHB2 */
        case HAL_RCC_BUS_AHB3:  return 0;  /* F1 无 AHB3 */
        case HAL_RCC_BUS_APB1:  return g_rcc.apb1_freq;
        case HAL_RCC_BUS_APB2:  return g_rcc.apb2_freq;
        case HAL_RCC_BUS_APB3:  return 0;  /* F1 无 APB3 */
        default: return 0;
    }
}

uint32_t hal_rcc_get_periph_freq(hal_rcc_periph_t periph)
{
    if (!g_rcc.initialized) return 0;
    if (!is_valid_periph(periph)) return 0;
    return g_rcc.periph_freq_cache[periph];
}

hal_rcc_source_t hal_rcc_get_source(void)
{
    if (!g_rcc.initialized) return HAL_RCC_SRC_HSI;

    uint32_t sws = (RCC->CFGR & RCC_CFGR_SWS) >> 2;
    switch (sws) {
        case 0:  return HAL_RCC_SRC_HSI;
        case 1:  return HAL_RCC_SRC_HSE;
        case 2:  return HAL_RCC_SRC_PLL;
        default: return HAL_RCC_SRC_HSI;
    }
}

hal_err_t hal_rcc_reset(hal_rcc_periph_t periph)
{
    if (!g_rcc.initialized) return HAL_ERR_NOT_INITIALIZED;
    const bsp_rcc_periph_map_t *map = get_periph_map(periph);
    if (map == NULL) return HAL_ERR_INVALID_PARAM;

    uint32_t bit_mask = 1U << map->bit_pos;

    switch (map->bus_type) {
        case 0:
            SET_BIT(RCC->AHBRSTR, bit_mask);
            CLEAR_BIT(RCC->AHBRSTR, bit_mask);
            break;
        case 1:
            SET_BIT(RCC->APB1RSTR, bit_mask);
            CLEAR_BIT(RCC->APB1RSTR, bit_mask);
            break;
        case 2:
            SET_BIT(RCC->APB2RSTR, bit_mask);
            CLEAR_BIT(RCC->APB2RSTR, bit_mask);
            break;
        default:
            return HAL_ERR_INVALID_PARAM;
    }
    return HAL_SUCCESS;
}

hal_err_t hal_rcc_enter_low_power(const hal_rcc_config_t *config)
{
    if (!g_rcc.initialized) return HAL_ERR_NOT_INITIALIZED;

    hal_rcc_config_t resolved;
    uint32_t sysclk, ahb, apb1, apb2;
    uint32_t div_apb1, div_apb2;

    if (config == NULL) {
        /* 默认低功耗配置：HSI 8MHz */
        resolved.sysclk_src = HAL_RCC_SRC_HSI;
        resolved.sysclk_hz  = BSP_RCC_HSI_FREQ;
        resolved.ahb_hz   = BSP_RCC_HSI_FREQ;
        resolved.apb1_hz  = BSP_RCC_HSI_FREQ;
        resolved.apb2_hz  = BSP_RCC_HSI_FREQ;
        sysclk = ahb = apb1 = apb2 = BSP_RCC_HSI_FREQ;
    } else {
        hal_err_t err = resolve_config(config, &resolved, &sysclk, &ahb, &apb1, &apb2);
        if (err != HAL_SUCCESS) return err;
    }

    if (calc_prescaler(apb1, sysclk, 16, &div_apb1) != 0) return HAL_ERR_INVALID_PARAM;
    if (calc_prescaler(apb2, sysclk, 16, &div_apb2) != 0) return HAL_ERR_INVALID_PARAM;

    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};
    hal_err_t err = build_hal_clock_config(&resolved, div_apb1, div_apb2, &osc_init, &clk_init);
    if (err != HAL_SUCCESS) return err;

    /* HAL 时钟配置不在临界区内 */
    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
        return HAL_ERR_TIMEOUT;
    }

    uint32_t latency = get_flash_latency(sysclk);
    if (HAL_RCC_ClockConfig(&clk_init, latency) != HAL_OK) {
        return HAL_ERR_IO;
    }

    /* 更新全局状态 */
    CRITICAL_ENTER();
    g_rcc.ahb_freq  = HAL_RCC_GetHCLKFreq();
    g_rcc.apb1_freq = HAL_RCC_GetPCLK1Freq();
    g_rcc.apb2_freq = HAL_RCC_GetPCLK2Freq();
    update_periph_freq_cache();
    CRITICAL_EXIT();

    return HAL_SUCCESS;
}

hal_err_t hal_rcc_exit_low_power(void)
{
    if (!g_rcc.initialized) return HAL_ERR_NOT_INITIALIZED;

    /* 恢复到 hal_rcc_init 时保存的配置 */
    hal_rcc_config_t resolved = g_rcc.active_config;

    /* 如果 active_config 无效（理论上不应发生），使用默认配置 */
    if (resolved.sysclk_hz == 0) {
        fill_default_config(&resolved);
    }

    uint32_t sysclk = resolved.sysclk_hz;
    uint32_t ahb    = resolved.ahb_hz;
    uint32_t apb1   = resolved.apb1_hz;
    uint32_t apb2   = resolved.apb2_hz;
    uint32_t div_apb1, div_apb2;

    if (calc_prescaler(apb1, sysclk, 16, &div_apb1) != 0) return HAL_ERR_INVALID_PARAM;
    if (calc_prescaler(apb2, sysclk, 16, &div_apb2) != 0) return HAL_ERR_INVALID_PARAM;

    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};
    hal_err_t err = build_hal_clock_config(&resolved, div_apb1, div_apb2, &osc_init, &clk_init);
    if (err != HAL_SUCCESS) return err;

    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
        return HAL_ERR_TIMEOUT;
    }

    uint32_t latency = get_flash_latency(sysclk);
    if (HAL_RCC_ClockConfig(&clk_init, latency) != HAL_OK) {
        return HAL_ERR_IO;
    }

    CRITICAL_ENTER();
    g_rcc.ahb_freq  = HAL_RCC_GetHCLKFreq();
    g_rcc.apb1_freq = HAL_RCC_GetPCLK1Freq();
    g_rcc.apb2_freq = HAL_RCC_GetPCLK2Freq();
    update_periph_freq_cache();
    CRITICAL_EXIT();

    return HAL_SUCCESS;
}