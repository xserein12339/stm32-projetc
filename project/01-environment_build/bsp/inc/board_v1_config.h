#ifndef BOARD_V1_CONFIG_H
#define BOARD_V1_CONFIG_H

#include "stm32f1xx_hal.h"

/* ========================================================================== */
/*                         GPIO 基础资源定义                                   */
/* ========================================================================== */
#define BSP_GPIOA_PORT              GPIOA
#define BSP_GPIOA_PIN0              GPIO_PIN_0      ///< TIM2_ENC_A_CH1
#define BSP_GPIOA_PIN1              GPIO_PIN_1      ///< TIM2_ENC_A_CH2
#define BSP_GPIOA_PIN2              GPIO_PIN_2      ///< LED2
#define BSP_GPIOA_PIN3              GPIO_PIN_3      ///< KEY3
#define BSP_GPIOA_PIN4              GPIO_PIN_4      ///< KEY1
#define BSP_GPIOA_PIN5              GPIO_PIN_5      ///< KEY2
#define BSP_GPIOA_PIN6              GPIO_PIN_6      ///< TIM3_ENC_B_CH1
#define BSP_GPIOA_PIN7              GPIO_PIN_7      ///< TIM3_ENC_B_CH2
#define BSP_GPIOA_PIN8              GPIO_PIN_8      ///< TIM1_CH1 PWMA
#define BSP_GPIOA_PIN9              GPIO_PIN_9      ///< TIM1_CH2 PWMB
#define BSP_GPIOA_PIN10             GPIO_PIN_10     ///< TB6612 MOTORB_IN1

#define BSP_GPIOB_PORT              GPIOB
#define BSP_GPIOB_PIN0              GPIO_PIN_0      ///< TB6612 MOTORA_IN1
#define BSP_GPIOB_PIN1              GPIO_PIN_1      ///< TB6612 MOTORA_IN2
#define BSP_GPIOB_PIN5              GPIO_PIN_5      ///< TB6612 MOTORB_IN2
#define BSP_GPIOB_PIN6              GPIO_PIN_6      ///< I2C1_SCL
#define BSP_GPIOB_PIN7              GPIO_PIN_7      ///< I2C1_SDA
#define BSP_GPIOB_PIN10             GPIO_PIN_10     ///< USART3_TX
#define BSP_GPIOB_PIN11             GPIO_PIN_11     ///< USART3_RX
#define BSP_GPIOB_PIN13             GPIO_PIN_13     ///< LED3 
#define BSP_GPIOB_PIN14             GPIO_PIN_14     ///< TB6612_STBY 

#define BSP_GPIOC_PORT              GPIOC
#define BSP_GPIOC_PIN13             GPIO_PIN_13     ///< LED1  

/* ========================================================================== */
/*                       I2C1 资源定义 (默认引脚 PB6/PB7 )                      */
/* ========================================================================== */
#define BSP_I2C1_PORT               I2C1
#define BSP_I2C1_GPIO_PORT          BSP_GPIOB_PORT
#define BSP_I2C1_SCL_PIN            BSP_GPIOB_PIN6
#define BSP_I2C1_SDA_PIN            BSP_GPIOB_PIN7

/* ========================================================================== */
/*                      USART3 资源定义 (默认引脚 PB10/PB11 )                   */
/* ========================================================================== */
#define BSP_USART3_PORT             USART3
#define BSP_USART3_TX_PIN           BSP_GPIOB_PIN10
#define BSP_USART3_TX_PORT          BSP_GPIOB_PORT
#define BSP_USART3_RX_PIN           BSP_GPIOB_PIN11
#define BSP_USART3_RX_PORT          BSP_GPIOB_PORT

/* ========================================================================== */
/*                         TIM1 双电机 PWM (仅PWMA/PWMB )                      */
/* ========================================================================== */
#define BSP_TIM1_PORT               TIM1

#define BSP_TIM1_CH1                TIM_CHANNEL_1   ///< PWMA
#define BSP_TIM1_CH1_PIN            BSP_GPIOA_PIN8
#define BSP_TIM1_CH1_PORT           BSP_GPIOA_PORT

#define BSP_TIM1_CH2                TIM_CHANNEL_2   ///< PWMB
#define BSP_TIM1_CH2_PIN            BSP_GPIOA_PIN9
#define BSP_TIM1_CH2_PORT           BSP_GPIOA_PORT

/* ========================================================================== */
/*                         TIM2 编码器A (默认引脚 PA0/PA1 )                     */
/* ========================================================================== */
#define BSP_TIM2_PORT               TIM2
#define BSP_ENC_A_CH1_PIN           BSP_GPIOA_PIN0
#define BSP_ENC_A_CH1_PORT          BSP_GPIOA_PORT
#define BSP_ENC_A_CH2_PIN           BSP_GPIOA_PIN1
#define BSP_ENC_A_CH2_PORT          BSP_GPIOA_PORT

/* ========================================================================== */
/*                         TIM3 编码器B (默认引脚 PA6/PA7 )                     */
/* ========================================================================== */
#define BSP_TIM3_PORT               TIM3
#define BSP_ENC_B_CH1_PIN           BSP_GPIOA_PIN6
#define BSP_ENC_B_CH1_PORT          BSP_GPIOA_PORT
#define BSP_ENC_B_CH2_PIN           BSP_GPIOA_PIN7
#define BSP_ENC_B_CH2_PORT          BSP_GPIOA_PORT

/* ========================================================================== */
/*                         编码器业务逻辑映射（非物理资源）                       */
/* ========================================================================== */
#define BSP_ENCODER_LEFT_TIM_ID     2   ///< 左编码器对应定时器编号（TIM2）
#define BSP_ENCODER_RIGHT_TIM_ID    3   ///< 右编码器对应定时器编号（TIM3）

/* ========================================================================== */
/*                           OLED / MPU6050 资源定义                           */
/* ========================================================================== */
#define BSP_OLED_I2C_PORT           BSP_I2C1_PORT
#define BSP_OLED_I2C_SCL_PIN        BSP_I2C1_SCL_PIN
#define BSP_OLED_I2C_SDA_PIN        BSP_I2C1_SDA_PIN

#define BSP_MPU6050_I2C_PORT        BSP_I2C1_PORT
#define BSP_MPU6050_I2C_SCL_PIN     BSP_I2C1_SCL_PIN
#define BSP_MPU6050_I2C_SDA_PIN     BSP_I2C1_SDA_PIN

/* ========================================================================== */
/*                             ESP8266 资源定义                                */
/* ========================================================================== */
#define BSP_ESP8266_USART_PORT      BSP_USART3_PORT
#define BSP_ESP8266_USART_TX_PIN    BSP_USART3_TX_PIN
#define BSP_ESP8266_USART_RX_PIN    BSP_USART3_RX_PIN

/* ========================================================================== */
/*                          TB6612 双电机驱动资源定义                            */
/* ========================================================================== */
#define BSP_TB6612_TIM              BSP_TIM1_PORT

/* ---- Motor A ---- */
#define BSP_TB6612_MOTORA_PWM_CH        BSP_TIM1_CH1
#define BSP_TB6612_MOTORA_PWM_PORT      BSP_TIM1_CH1_PORT
#define BSP_TB6612_MOTORA_PWM_PIN       BSP_TIM1_CH1_PIN
#define BSP_TB6612_MOTORA_IN1_PORT      BSP_GPIOB_PORT
#define BSP_TB6612_MOTORA_IN1_PIN       BSP_GPIOB_PIN0
#define BSP_TB6612_MOTORA_IN2_PORT      BSP_GPIOB_PORT
#define BSP_TB6612_MOTORA_IN2_PIN       BSP_GPIOB_PIN1

/* ---- Motor B ---- */
#define BSP_TB6612_MOTORB_PWM_CH        BSP_TIM1_CH2
#define BSP_TB6612_MOTORB_PWM_PORT      BSP_TIM1_CH2_PORT
#define BSP_TB6612_MOTORB_PWM_PIN       BSP_TIM1_CH2_PIN
#define BSP_TB6612_MOTORB_IN1_PORT      BSP_GPIOA_PORT
#define BSP_TB6612_MOTORB_IN1_PIN       BSP_GPIOA_PIN10
#define BSP_TB6612_MOTORB_IN2_PORT      BSP_GPIOB_PORT
#define BSP_TB6612_MOTORB_IN2_PIN       BSP_GPIOB_PIN5

/* ---- STBY 待机控制 ---- */
#define BSP_TB6612_STBY_PORT            BSP_GPIOB_PORT
#define BSP_TB6612_STBY_PIN             BSP_GPIOB_PIN14

/* ========================================================================== */
/*                          LED / KEY 资源定义                                 */
/* ========================================================================== */
#define BSP_LED1_PORT               BSP_GPIOC_PORT      
#define BSP_LED1_PIN                BSP_GPIOC_PIN13
#define BSP_LED2_PORT               BSP_GPIOA_PORT
#define BSP_LED2_PIN                BSP_GPIOA_PIN2
#define BSP_LED3_PORT               BSP_GPIOB_PORT       
#define BSP_LED3_PIN                BSP_GPIOB_PIN13

#define BSP_KEY1_PORT               BSP_GPIOA_PORT
#define BSP_KEY1_PIN                BSP_GPIOA_PIN3
#define BSP_KEY2_PORT               BSP_GPIOA_PORT
#define BSP_KEY2_PIN                BSP_GPIOA_PIN4
#define BSP_KEY3_PORT               BSP_GPIOA_PORT
#define BSP_KEY3_PIN                BSP_GPIOA_PIN5

/* ---- KEY 中断资源（新增） ---- */
#define BSP_KEY1_EXTI_LINE          EXTI_LINE_4
#define BSP_KEY1_IRQN               EXTI4_IRQn
#define BSP_KEY2_EXTI_LINE          EXTI_LINE_5
#define BSP_KEY2_IRQN               EXTI9_5_IRQn
#define BSP_KEY3_EXTI_LINE          EXTI_LINE_3
#define BSP_KEY3_IRQN               EXTI3_IRQn

#endif /* BOARD_V1_CONFIG_H */