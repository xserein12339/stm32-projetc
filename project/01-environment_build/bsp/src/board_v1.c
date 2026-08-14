#include "board_v1.h"
#include "board_v1_config.h"

#include "bsp_i2c.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include "bsp_mpu6050.h"
#include "bsp_esp8266.h"


/**
 * @brief  配置系统时钟至 72MHz (HSE + PLL)
 * @note   根据实际晶振频率和原理图修改 RCC_OscInitTypeDef 参数
 * @retval BSP_OK 成功 / BSP_ERR_CLOCK 失败
 */
static bsp_err_t bsp_system_clock_config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 1. 配置 HSE + PLL → SYSCLK = 72MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;  /* 8MHz × 9 = 72MHz */

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return BSP_ERR_CLOCK;
    }

    /* 2. 配置总线时钟 AHB/APB1/APB2 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;    /* AHB  = 72MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;      /* APB1 = 36MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      /* APB2 = 72MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        return BSP_ERR_CLOCK;
    }

    return BSP_OK;
}

/* ========================================================================== */
/*                         BSP 统一初始化入口                                   */
/* ========================================================================== */


bsp_err_t bsp_init(void)
{
    HAL_Init();
    bsp_err_t ret;

    /* Phase 1: 系统基础 */
    ret = bsp_system_clock_config();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }

    bsp_key_init();      
    bsp_led_init();


    return BSP_OK;
}


void Error_Handler(void)
{
    /* 关闭全局中断，防止 ISR 干扰故障现场 */
    __disable_irq();

    /* 可选：翻转错误指示 LED / 记录故障码到备份寄存器 */
    // HAL_GPIO_WritePin(ERR_LED_PORT, ERR_LED_PIN, GPIO_PIN_SET);

    /* 永久停驻，等待调试器附加或硬件看门狗复位 */
    while (1) {
        __NOP();
    }
}
