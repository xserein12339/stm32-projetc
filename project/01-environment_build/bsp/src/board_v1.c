#include "board_v1.h"
#include "bsp_gpio.h"
#include "stm32f1xx.h"

/**
 * @brief  配置系统时钟为 72MHz (HSE 8MHz × PLL9)
 * @note   必须在 bsp_init() 最早阶段调用
 */
void SystemClock_Config(void)
{
    /* 使能 HSE */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    /* 配置 Flash 等待周期（72MHz 需要 2 个等待周期） */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* AHB=72MHz, APB1=36MHz, APB2=72MHz */
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2   /* APB1 = HCLK/2 */

              | RCC_CFGR_PPRE2_DIV1   /* APB2 = HCLK   */
              | RCC_CFGR_HPRE_DIV1;   /* AHB  = SYSCLK */

    /* PLL: HSE × 9 = 72MHz */
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;

    /* 使能 PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* 切换系统时钟到 PLL */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

    SystemCoreClockUpdate();
}


int bsp_init(void)
{
    int ret;
    SystemClock_Config();

    HAL_Init(); 

    ret = bsp_gpio_init();
    if (ret != 0) return ret;

    return 0;
}