#ifndef BOARD_V1_CONFIG_H
#define BOARD_V1_CONFIG_H

#include "stm32f1xx_hal.h"

/* ======================== LED 定义 ======================== */
#define BSP_LED_GREEN_PORT          GPIOC
#define BSP_LED_GREEN_PIN           GPIO_PIN_13
#define BSP_LED_GREEN_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()

#define BSP_LED_ON_LEVEL            GPIO_PIN_RESET
#define BSP_LED_OFF_LEVEL           GPIO_PIN_SET

#endif /* BOARD_V1_CONFIG_H */