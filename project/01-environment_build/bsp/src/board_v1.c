#include "board_v1.h"
#include "board_v1_config.h"

#include "bsp_i2c.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "bsp_dbg.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_encoder.h"
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

/**
 * @brief   阶段标记直写（零 libc / 零 mw_log 依赖，定位初始化死点用）
 * @note    sizeof("clk") == strlen("clk")+1 == strlen("clk\n")：拼接 "\n"
 *          后长度恰为 sizeof(t)，无需运行时求长。
 */
#define BSP_PHASE(t) \
    (void)bsp_dbg_write(t "\n", (uint32_t)sizeof(t))

bsp_err_t bsp_init(void)
{
    HAL_Init();
    bsp_err_t ret;

    /* Phase 0: 系统时钟（先于日志通道：USART2 波特率依赖 PCLK1，
     *          时钟失败时 Error_Handler 仅 PC13 LED 指示） */
    ret = bsp_system_clock_config();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }

    /* Phase 1: 调试通道（时钟就绪后尽早初始化，后续阶段 printf 取证可用） */
    ret = bsp_dbg_init();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    /* 二分诊断：首条 UART 输出在关中断下发送。
     * 完整出现 "P:dbg" -> 杀手是异步中断（唯一 ~1ms 周期源：SysTick）；
     * 仍被截断   -> CPU/时钟/总线级硬件问题（HSE 停振等），与软件中断无关 */
    __disable_irq();
    BSP_PHASE("P:dbg");
    __enable_irq();

    /* 400kHz fast mode：SSD1306/MPU6050 均支持，全屏刷新耗时降为 1/4 */
    ret = bsp_i2c_init(400000);
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    BSP_PHASE("P:i2c");

    ret = bsp_key_init();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    BSP_PHASE("P:key");

    ret = bsp_led_init();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    BSP_PHASE("P:led");

    ret = bsp_encoder_init();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    BSP_PHASE("P:enc");

    ret = bsp_motor_init();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    BSP_PHASE("P:mot");

    ret = bsp_oled_init();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    BSP_PHASE("P:oled");

    ret = bsp_mpu6050_init();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    BSP_PHASE("P:mpu");

    ret = bsp_esp8266_init();
    if (ret != BSP_OK) {
        Error_Handler();
        return ret;
    }
    BSP_PHASE("P:esp");

    return BSP_OK;
}

void Error_Handler(void)
{
    /* WHY 打印再关中断：轮询 TX 不依赖中断；初始化链哪一步死在这里，
     * 串口日志直接可见（否则 IWDG 复位循环里只能看到 boot 前几行） */
    static const char k_msg[] = "[F][bsp] Error_Handler\n";
    extern int32_t bsp_dbg_write(const char *data, uint32_t len);
    (void)bsp_dbg_write(k_msg, (uint32_t)(sizeof(k_msg) - 1U));

    __disable_irq();
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(0xFU << 20);
    GPIOC->CRH |= (0x2U << 20);
    GPIOC->ODR &= ~(1U << 13);

    while (1) { __NOP(); }
}
