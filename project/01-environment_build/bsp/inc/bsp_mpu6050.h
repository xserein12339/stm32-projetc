/**
 * @file    bsp_mpu6050.h
 * @brief   板级 MPU6050 BSP 层接口（I2C）
 * @note    - 硬件映射引用 board_v1_config.h
 *          - 实现 dal_imu_ops_t 并注册到 dal_imu 框架
 *          - 支持加速度计、陀螺仪，量程/ODR 可配置
 * @author  xserein
 * @version v1.0
 */

#ifndef __BSP_MPU6050_H__
#define __BSP_MPU6050_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 MPU6050 并注册到 DAL 框架
 * @note  自动探测设备地址（0x68/0x69），初始化硬件并注册
 * @retval BSP_OK           成功
 * @retval BSP_ERR_IO       I2C 通信失败或设备无响应
 * @retval BSP_ERR_FAIL     注册失败
 */
bsp_err_t bsp_mpu6050_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MPU6050_H__ */