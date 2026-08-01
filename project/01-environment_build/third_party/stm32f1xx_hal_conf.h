#ifndef __STM32F1XX_HAL_CONF_H
#define __STM32F1XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 模块使能（按需开启）======================== */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED      /* NVIC/SysTick */
#define HAL_RCC_MODULE_ENABLED         /* 时钟树 */
#define HAL_GPIO_MODULE_ENABLED        /* GPIO */
#define HAL_DMA_MODULE_ENABLED         /* 多数外设依赖 */
#define HAL_FLASH_MODULE_ENABLED       /* Flash 读写 */
#define HAL_PWR_MODULE_ENABLED         /* 低功耗/备份域 */
#define HAL_EXTI_MODULE_ENABLED        /* 外部中断 */

/* ↓↓↓ 用到哪个取消注释哪个 ↓↓↓ */
#define HAL_UART_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
// #define HAL_WWDG_MODULE_ENABLED
// #define HAL_IWDG_MODULE_ENABLED
// #define HAL_CAN_MODULE_ENABLED
// #define HAL_USB_MODULE_ENABLED         /* F103C8T6 有 USB，需要时开启 */

/* ======================== 振荡器配置 ======================== */
#if !defined(HSE_VALUE)
  #define HSE_VALUE    8000000U 
#endif
#if !defined(HSI_VALUE)
  #define HSI_VALUE    8000000U
#endif
#if !defined(LSI_VALUE)
  #define LSI_VALUE    40000U
#endif
#if !defined(LSE_VALUE)
  #define LSE_VALUE    32768U
#endif

#define HSE_STARTUP_TIMEOUT    100U
#define LSE_STARTUP_TIMEOUT    5000U

/* ======================== 系统配置 ======================== */
#define VDD_VALUE              3300U   /* VDD = 3.3V */
#define TICK_INT_PRIORITY      0x0FU   /* SysTick 优先级(最低) */
#define USE_RTOS               0U      /* 裸机设为0; 用FreeRTOS等改为1 */
#define PREFETCH_ENABLE        1U      /* Flash 预取缓冲 */

/* 回调函数开关（全部关闭以节省Flash）*/
#define USE_HAL_ADC_REGISTER_CALLBACKS         0U
#define USE_HAL_CAN_REGISTER_CALLBACKS         0U
#define USE_HAL_DAC_REGISTER_CALLBACKS         0U
#define USE_HAL_I2C_REGISTER_CALLBACKS         0U
#define USE_HAL_SPI_REGISTER_CALLBACKS         0U
#define USE_HAL_TIM_REGISTER_CALLBACKS         0U
#define USE_HAL_UART_REGISTER_CALLBACKS        0U
#define USE_HAL_USART_REGISTER_CALLBACKS       0U
#define USE_HAL_WWDG_REGISTER_CALLBACKS        0U

/* ======================== 断言控制 ======================== */
// #define USE_FULL_ASSERT    1U

/* ======================== SPI CRC ======================== */
#define USE_SPI_CRC            0U  

/* ======================== 头文件包含 ======================== */
#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f1xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f1xx_hal_gpio.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f1xx_hal_exti.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f1xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f1xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f1xx_hal_flash.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32f1xx_hal_pwr.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32f1xx_hal_uart.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
  #include "stm32f1xx_hal_spi.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
  #include "stm32f1xx_hal_i2c.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f1xx_hal_adc.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32f1xx_hal_tim.h"
#endif
#ifdef HAL_RTC_MODULE_ENABLED
  #include "stm32f1xx_hal_rtc.h"
#endif

/* ======================== Assert 实现 ======================== */
#ifdef USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1XX_HAL_CONF_H */