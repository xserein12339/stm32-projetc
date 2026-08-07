/**
 * @file hal_err.h
 * @brief HAL层统一错误码定义，为所有外设抽象接口提供标准化的返回值语义
 * 
 * @author xserein
 * @version v1.0
 */

#ifndef HAL_ERR_H
#define HAL_ERR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            基础类型与约定                                    */
/* ========================================================================== */
/** @brief HAL 统一返回类型: >=0 成功, <0 失败 */
typedef int32_t hal_err_t;

#define HAL_SUCCESS              ((hal_err_t)0)
#define HAL_IS_SUCCESS(err)      ((err) >= 0)
#define HAL_IS_ERR(err)          ((err) < 0)

/* ========================================================================== */
/*                              编码规则                                        */
/* ========================================================================== */
/**
 * @brief 错误码 32-bit 布局
 * @note  [31]    : 符号位 (1=负数/错误)
 *        [30:16] : 模块 ID (区分错误来源, 最多 32767 个模块)
 *        [15:0]  : 错误序号 (0x0000 保留给 HAL_SUCCESS, 实际错误从 0x0001 起)
 * 
 *        构造方式：通过 -(int32_t) 取负。module 和 code 的组合值不超过 0x7FFFFFFF，
 *        取负后不会触及 INT32_MIN，无未定义行为。
 */
#define _HAL_MAKE_ERR(module, code) \
    ((hal_err_t)(-((int32_t)((((uint32_t)(module) & 0x7FFFU) << 16) | ((uint32_t)(code) & 0xFFFFU)))))

/**
 * @warning 仅在 HAL_IS_ERR(err) 为真时有效！
 *          对 HAL_SUCCESS 或正值调用将返回无意义结果。
 */
#define HAL_ERR_GET_MODULE(err) \
    ((uint16_t)(((uint32_t)(-(err)) >> 16) & 0x7FFFU))
#define HAL_ERR_GET_CODE(err) \
    ((uint16_t)((uint32_t)(-(err)) & 0xFFFFU))

/* ========================================================================== */
/*                             模块 ID 分配                                     */
/* ========================================================================== */
/**
 * @brief 模块编号注册表
 * @note  0x0000-0x001F : HAL 核心模块 (官方保留)
 *        0x0020-0x00FF : 项目自定义模块
 *        0x0100-0x7FFF : 扩展预留
 * @warning 禁止修改已有编号！
 */
#define HAL_MOD_COMMON      0x00    ///< 通用/跨模块错误
#define HAL_MOD_GPIO        0x01    ///< GPIO
#define HAL_MOD_UART        0x02    ///< UART / USART
#define HAL_MOD_SPI         0x03    ///< SPI
#define HAL_MOD_I2C         0x04    ///< I2C
#define HAL_MOD_ADC         0x05    ///< ADC
#define HAL_MOD_PWM         0x06    ///< PWM / Timer
#define HAL_MOD_DMA         0x07    ///< DMA
#define HAL_MOD_FLASH       0x08    ///< Internal Flash
#define HAL_MOD_RCC         0x09    ///< RCC (Reset and Clock Control)
#define HAL_MOD_CAN         0x0A    ///< CAN
#define HAL_MOD_RTC         0x0B    ///< RTC
#define HAL_MOD_WDT         0x0C    ///< Watchdog
#define HAL_MOD_CRC         0x0D    ///< CRC 硬件单元
#define HAL_MOD_RNG         0x0E    ///< 随机数发生器
#define HAL_MOD_USB         0x0F    ///< USB
#define HAL_MOD_ETH         0x10    ///< Ethernet
/* 0x11-0x1F 保留给未来 HAL 核心模块 */

/* ========================================================================== */
/*                          通用错误码 (COMMON)                                 */
/* ========================================================================== */
#define HAL_ERR_INVALID_PARAM       _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0001)
#define HAL_ERR_NOT_SUPPORTED       _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0002)
#define HAL_ERR_BUSY                _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0003)
#define HAL_ERR_TIMEOUT             _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0004)
#define HAL_ERR_NO_MEMORY           _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0005)
#define HAL_ERR_NOT_INITIALIZED     _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0006)
#define HAL_ERR_ALREADY_INIT        _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0007)
#define HAL_ERR_IO                  _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0008)
#define HAL_ERR_AGAIN               _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x0009)
#define HAL_ERR_ABORTED             _HAL_MAKE_ERR(HAL_MOD_COMMON, 0x000A)

/* ========================================================================== */
/*                          GPIO 模块专属错误码                                 */
/* ========================================================================== */
#define HAL_GPIO_ERR_INVALID_PIN        _HAL_MAKE_ERR(HAL_MOD_GPIO, 0x0001)
#define HAL_GPIO_ERR_IRQ_CONFLICT       _HAL_MAKE_ERR(HAL_MOD_GPIO, 0x0002)
#define HAL_GPIO_ERR_INVALID_MODE       _HAL_MAKE_ERR(HAL_MOD_GPIO, 0x0003)
#define HAL_GPIO_ERR_IRQ_NOT_REGISTERED _HAL_MAKE_ERR(HAL_MOD_GPIO, 0x0004)

/* ========================================================================== */
/*                         UART 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_UART_ERR_BAUDRATE       _HAL_MAKE_ERR(HAL_MOD_UART, 0x0001)
#define HAL_UART_ERR_FRAMING        _HAL_MAKE_ERR(HAL_MOD_UART, 0x0002)
#define HAL_UART_ERR_PARITY         _HAL_MAKE_ERR(HAL_MOD_UART, 0x0003)
#define HAL_UART_ERR_NOISE          _HAL_MAKE_ERR(HAL_MOD_UART, 0x0004)
#define HAL_UART_ERR_OVERRUN        _HAL_MAKE_ERR(HAL_MOD_UART, 0x0005)
#define HAL_UART_ERR_BREAK          _HAL_MAKE_ERR(HAL_MOD_UART, 0x0006)
#define HAL_UART_ERR_FLOW_CTRL      _HAL_MAKE_ERR(HAL_MOD_UART, 0x0007)

/* ========================================================================== */
/*                          SPI 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_SPI_ERR_MODE_FAULT      _HAL_MAKE_ERR(HAL_MOD_SPI, 0x0001)
#define HAL_SPI_ERR_CRC_MISMATCH    _HAL_MAKE_ERR(HAL_MOD_SPI, 0x0002)
#define HAL_SPI_ERR_FIFO_UNDERRUN   _HAL_MAKE_ERR(HAL_MOD_SPI, 0x0003)
#define HAL_SPI_ERR_FIFO_OVERRUN    _HAL_MAKE_ERR(HAL_MOD_SPI, 0x0004)
#define HAL_SPI_ERR_NSS_FAULT       _HAL_MAKE_ERR(HAL_MOD_SPI, 0x0005)
#define HAL_SPI_ERR_HALF_DUPLEX_DIR _HAL_MAKE_ERR(HAL_MOD_SPI, 0x0006)

/* ========================================================================== */
/*                          I2C 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_I2C_ERR_ARB_LOST        _HAL_MAKE_ERR(HAL_MOD_I2C, 0x0001)
#define HAL_I2C_ERR_NACK            _HAL_MAKE_ERR(HAL_MOD_I2C, 0x0002)
#define HAL_I2C_ERR_ADDR_NACK       _HAL_MAKE_ERR(HAL_MOD_I2C, 0x0003)
#define HAL_I2C_ERR_DATA_NACK       _HAL_MAKE_ERR(HAL_MOD_I2C, 0x0004)
#define HAL_I2C_ERR_BUS_ERR         _HAL_MAKE_ERR(HAL_MOD_I2C, 0x0005)
#define HAL_I2C_ERR_CLK_STRETCH     _HAL_MAKE_ERR(HAL_MOD_I2C, 0x0006)
#define HAL_I2C_ERR_PEC_MISMATCH    _HAL_MAKE_ERR(HAL_MOD_I2C, 0x0007)
#define HAL_I2C_ERR_OVERRUN         _HAL_MAKE_ERR(HAL_MOD_I2C, 0x0008)

/* ========================================================================== */
/*                          ADC 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_ADC_ERR_INVALID_CHANNEL   _HAL_MAKE_ERR(HAL_MOD_ADC, 0x0001)
#define HAL_ADC_ERR_INVALID_SEQ       _HAL_MAKE_ERR(HAL_MOD_ADC, 0x0002)
#define HAL_ADC_ERR_CALIBRATION       _HAL_MAKE_ERR(HAL_MOD_ADC, 0x0003)
#define HAL_ADC_ERR_VREF              _HAL_MAKE_ERR(HAL_MOD_ADC, 0x0004)
#define HAL_ADC_ERR_DMA_CONFLICT      _HAL_MAKE_ERR(HAL_MOD_ADC, 0x0005)
#define HAL_ADC_ERR_INJECTED_OVERRUN  _HAL_MAKE_ERR(HAL_MOD_ADC, 0x0006)
#define HAL_ADC_ERR_AWD               _HAL_MAKE_ERR(HAL_MOD_ADC, 0x0007)

/* ========================================================================== */
/*                       PWM / Timer 模块专属错误码                             */
/* ========================================================================== */
#define HAL_PWM_ERR_INVALID_CHANNEL _HAL_MAKE_ERR(HAL_MOD_PWM, 0x0001)
#define HAL_PWM_ERR_FREQ_RANGE      _HAL_MAKE_ERR(HAL_MOD_PWM, 0x0002)
#define HAL_PWM_ERR_DEAD_TIME       _HAL_MAKE_ERR(HAL_MOD_PWM, 0x0003)
#define HAL_PWM_ERR_FAULT           _HAL_MAKE_ERR(HAL_MOD_PWM, 0x0004)
#define HAL_PWM_ERR_CAPTURE_OVERRUN _HAL_MAKE_ERR(HAL_MOD_PWM, 0x0005)
#define HAL_PWM_ERR_DMA_BURST       _HAL_MAKE_ERR(HAL_MOD_PWM, 0x0006)

/* ========================================================================== */
/*                          DMA 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_DMA_ERR_INVALID_CHANNEL _HAL_MAKE_ERR(HAL_MOD_DMA, 0x0001)
#define HAL_DMA_ERR_CHANNEL_BUSY    _HAL_MAKE_ERR(HAL_MOD_DMA, 0x0002)
#define HAL_DMA_ERR_ADDR_ALIGN      _HAL_MAKE_ERR(HAL_MOD_DMA, 0x0003)
#define HAL_DMA_ERR_XFER_CONFIG     _HAL_MAKE_ERR(HAL_MOD_DMA, 0x0004)
#define HAL_DMA_ERR_BUS_FAULT       _HAL_MAKE_ERR(HAL_MOD_DMA, 0x0005)
#define HAL_DMA_ERR_FIFO_ERR        _HAL_MAKE_ERR(HAL_MOD_DMA, 0x0006)
#define HAL_DMA_ERR_DIRECT_MODE     _HAL_MAKE_ERR(HAL_MOD_DMA, 0x0007)

/* ========================================================================== */
/*                        Flash 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_FLASH_ERR_PROG_ALIGN    _HAL_MAKE_ERR(HAL_MOD_FLASH, 0x0001)
#define HAL_FLASH_ERR_PROG_VERIFY   _HAL_MAKE_ERR(HAL_MOD_FLASH, 0x0002)
#define HAL_FLASH_ERR_ERASE_VERIFY  _HAL_MAKE_ERR(HAL_MOD_FLASH, 0x0003)
#define HAL_FLASH_ERR_WRITE_PROTECT _HAL_MAKE_ERR(HAL_MOD_FLASH, 0x0004)
#define HAL_FLASH_ERR_OUT_OF_RANGE  _HAL_MAKE_ERR(HAL_MOD_FLASH, 0x0005)
#define HAL_FLASH_ERR_ECC           _HAL_MAKE_ERR(HAL_MOD_FLASH, 0x0006)
#define HAL_FLASH_ERR_VOLTAGE       _HAL_MAKE_ERR(HAL_MOD_FLASH, 0x0007)
#define HAL_FLASH_ERR_SEQ           _HAL_MAKE_ERR(HAL_MOD_FLASH, 0x0008)

/* ========================================================================== */
/*                          RCC 模块专属错误码                                   */
/* ========================================================================== */
#define HAL_RCC_ERR_HSE_FAIL              _HAL_MAKE_ERR(HAL_MOD_RCC, 0x0001)
#define HAL_RCC_ERR_PLL_LOCK              _HAL_MAKE_ERR(HAL_MOD_RCC, 0x0002)
#define HAL_RCC_ERR_FREQ_INVALID          _HAL_MAKE_ERR(HAL_MOD_RCC, 0x0003)
#define HAL_RCC_ERR_CLK_SWITCH            _HAL_MAKE_ERR(HAL_MOD_RCC, 0x0004)
#define HAL_RCC_ERR_BUS_FREQ              _HAL_MAKE_ERR(HAL_MOD_RCC, 0x0005)
#define HAL_RCC_ERR_LOW_POWER_UNSUPPORTED _HAL_MAKE_ERR(HAL_MOD_RCC, 0x0006)

/* ========================================================================== */
/*                          CAN 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_CAN_ERR_ARB_LOST        _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0001)
#define HAL_CAN_ERR_ACK_ERR         _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0002)
#define HAL_CAN_ERR_BIT_ERR         _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0003)
#define HAL_CAN_ERR_STUFF_ERR       _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0004)
#define HAL_CAN_ERR_CRC_ERR         _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0005)
#define HAL_CAN_ERR_FORM_ERR        _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0006)
#define HAL_CAN_ERR_BUS_OFF         _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0007)
#define HAL_CAN_ERR_RX_OVERRUN      _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0008)
#define HAL_CAN_ERR_MAILBOX_FULL    _HAL_MAKE_ERR(HAL_MOD_CAN, 0x0009)

/* ========================================================================== */
/*                          RTC 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_RTC_ERR_INVALID_DATE    _HAL_MAKE_ERR(HAL_MOD_RTC, 0x0001)
#define HAL_RTC_ERR_NOT_SYNC        _HAL_MAKE_ERR(HAL_MOD_RTC, 0x0002)
#define HAL_RTC_ERR_CALIBRATION     _HAL_MAKE_ERR(HAL_MOD_RTC, 0x0003)
#define HAL_RTC_ERR_WAKEUP_CONFIG   _HAL_MAKE_ERR(HAL_MOD_RTC, 0x0004)
#define HAL_RTC_ERR_TAMPER          _HAL_MAKE_ERR(HAL_MOD_RTC, 0x0005)

/* ========================================================================== */
/*                        Watchdog 模块专属错误码                               */
/* ========================================================================== */
#define HAL_WDT_ERR_INVALID_TIMEOUT _HAL_MAKE_ERR(HAL_MOD_WDT, 0x0001)
#define HAL_WDT_ERR_ALREADY_STARTED _HAL_MAKE_ERR(HAL_MOD_WDT, 0x0002)
#define HAL_WDT_ERR_NOT_ENABLED     _HAL_MAKE_ERR(HAL_MOD_WDT, 0x0003)

/* ========================================================================== */
/*                          CRC 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_CRC_ERR_INVALID_POLY    _HAL_MAKE_ERR(HAL_MOD_CRC, 0x0001)
#define HAL_CRC_ERR_INVALID_INIT    _HAL_MAKE_ERR(HAL_MOD_CRC, 0x0002)

/* ========================================================================== */
/*                          RNG 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_RNG_ERR_CLOCK_ERROR     _HAL_MAKE_ERR(HAL_MOD_RNG, 0x0001)
#define HAL_RNG_ERR_SEED_ERROR      _HAL_MAKE_ERR(HAL_MOD_RNG, 0x0002)

/* ========================================================================== */
/*                          USB 模块专属错误码                                  */
/* ========================================================================== */
#define HAL_USB_ERR_DISCONNECT      _HAL_MAKE_ERR(HAL_MOD_USB, 0x0001)
#define HAL_USB_ERR_ENUM_TIMEOUT    _HAL_MAKE_ERR(HAL_MOD_USB, 0x0002)
#define HAL_USB_ERR_EP_STALL        _HAL_MAKE_ERR(HAL_MOD_USB, 0x0003)
#define HAL_USB_ERR_FIFO_OVERRUN    _HAL_MAKE_ERR(HAL_MOD_USB, 0x0004)

/* ========================================================================== */
/*                         Ethernet 模块专属错误码                                */
/* ========================================================================== */
#define HAL_ETH_ERR_DMA_BUS         _HAL_MAKE_ERR(HAL_MOD_ETH, 0x0001)
#define HAL_ETH_ERR_RX_OVERRUN      _HAL_MAKE_ERR(HAL_MOD_ETH, 0x0002)
#define HAL_ETH_ERR_TX_UNDERRUN     _HAL_MAKE_ERR(HAL_MOD_ETH, 0x0003)

#ifdef __cplusplus
}
#endif

#endif /* HAL_ERR_H */