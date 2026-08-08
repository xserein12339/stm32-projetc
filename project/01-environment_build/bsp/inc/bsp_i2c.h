/**
 * @file bsp_i2c.h
 * @brief BSP I2C1 驱动接口 
 * 
 * @details 
 * 
 * @author xserein
 * @version v1.0
 */

#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include "bsp_err.h"
#include <stdint.h>
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 I2C1  
 * @retval BSP_OK           初始化成功或已初始化
 * @retval BSP_ERR_IO       硬件配置失败
 * @note   内部含幂等保护，多设备驱动可安全重复调用
 */
bsp_err_t bsp_i2c_init(void);

/**
 * @brief 反初始化 I2C1，关闭外设时钟并将引脚恢复为模拟输入
 * @retval BSP_OK           反初始化成功
 * @retval BSP_ERR_NOT_INIT 尚未初始化
 * @note   仅在系统进入低功耗或需要完全释放总线时调用
 */
bsp_err_t bsp_i2c_deinit(void);

/**
 * @brief 向 I2C 设备的 8 位寄存器写入数据
 * @param[in] dev_addr 7 位设备地址（无需手动左移）
 * @param[in] reg      8 位寄存器地址
 * @param[in] data     待写入数据缓冲区指针
 * @param[in] len      待写入数据长度（字节）
 * @retval BSP_OK           写入成功
 * @retval BSP_ERR_NOT_INIT 总线未初始化
 * @retval BSP_ERR_IO       传输失败或超时
 */
bsp_err_t bsp_i2c_write(uint8_t dev_addr, uint8_t reg,
                        const uint8_t *data, uint16_t len);

/**
 * @brief 从 I2C 设备的 8 位寄存器读取数据
 * @param[in]  dev_addr 7 位设备地址（无需手动左移）
 * @param[in]  reg      8 位寄存器地址
 * @param[out] buf      读取数据缓存指针
 * @param[in]  len      期望读取的数据长度（字节）
 * @retval BSP_OK           读取成功
 * @retval BSP_ERR_NOT_INIT 总线未初始化
 * @retval BSP_ERR_IO       传输失败或超时
 */
bsp_err_t bsp_i2c_read(uint8_t dev_addr, uint8_t reg,
                       uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_H__ */