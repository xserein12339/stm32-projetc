/**
 * @file    bsp_i2c.h
 * @brief   板级 I2C BSP 层接口（仅支持 I2C1）v1.1
 * @note    - 硬件映射引用 board_v1_config.h
 *          - 提供阻塞读写接口，带超时保护与总线自动恢复
 *          - 单实例设计，不支持多总线
 *
 * @par 线程安全契约
 *      本接口【非线程安全】。多任务环境下调用者须自行加锁，
 *      或使用 bsp_i2c_lock() / bsp_i2c_unlock() 提供的内置互斥量。
 *      同一事务内的 write_reg + read_reg 必须由调用者保证原子性。
 *
 * @author  xserein
 * @version v1.1
 */

#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                               常量定义                                       */
/* ========================================================================== */

/** @brief 默认超时时间(ms)，可通过 BSP_I2C_DEFAULT_TIMEOUT_MS 编译期覆盖 */
#ifndef BSP_I2C_DEFAULT_TIMEOUT_MS
#define BSP_I2C_DEFAULT_TIMEOUT_MS  (100U)
#endif

/** @brief 总线恢复时 SCL toggle 最大次数 */
#define BSP_I2C_BUS_RECOVERY_CLKS   (16U)

/* ========================================================================== */
/*                            生命周期接口                                      */
/* ========================================================================== */

/**
 * @brief 初始化 I2C1 外设
 * @param freq_hz  I2C 时钟频率(Hz)，常用: 100000 / 400000
 * @retval BSP_OK           成功
 * @retval BSP_ERR_IO       HAL 初始化失败
 * @retval BSP_ERR_BUSY     重复初始化（未先 deinit）
 */
bsp_err_t bsp_i2c_init(uint32_t freq_hz);

/**
 * @brief 反初始化 I2C1，释放资源
 * @note  若检测到总线异常（SDA 被拉低），会自动执行总线恢复序列
 * @retval BSP_OK           成功
 * @retval BSP_ERR_NOT_INIT 未初始化
 */
bsp_err_t bsp_i2c_deinit(void);

/**
 * @brief 手动触发总线恢复（SCL toggle + START/STOP 重建）
 * @note  当 I2C 外设因从机拉低 SDA 而锁死时调用。
 *        此操作会将 SCL/SDA 临时切换为 GPIO 输出模式，
 *        完成后自动重新初始化 I2C 外设。
 * @retval BSP_OK       恢复成功
 * @retval BSP_ERR_IO   恢复后 SDA 仍为低电平（硬件故障）
 */
bsp_err_t bsp_i2c_bus_recovery(void);

/* ========================================================================== */
/*                          互斥接口（FreeRTOS）                                */
/* ========================================================================== */

/**
 * @brief 获取 I2C 总线互斥锁
 * @param timeout_ms 等待超时(ms)，0 = 不等待
 * @retval BSP_OK        获取成功
 * @retval BSP_ERR_BUSY  超时未获取
 * @retval BSP_ERR_NOT_INIT 未初始化
 */
bsp_err_t bsp_i2c_lock(uint32_t timeout_ms);

/**
 * @brief 释放 I2C 总线互斥锁
 * @retval BSP_OK           释放成功
 * @retval BSP_ERR_NOT_INIT 未初始化或未持有锁
 */
bsp_err_t bsp_i2c_unlock(void);

/* ========================================================================== */
/*                     8 位寄存器地址 读写接口                                  */
/* ========================================================================== */

/**
 * @brief 向 I2C 设备寄存器写入数据（8位 reg addr）
 * @param dev_addr   7位设备地址（无需左移）
 * @param reg        8位寄存器地址
 * @param data       待写入数据缓冲区
 * @param len        数据长度(字节)
 * @param timeout_ms 超时(ms)，0 = 使用 BSP_I2C_DEFAULT_TIMEOUT_MS
 */
bsp_err_t bsp_i2c_write_reg(uint8_t dev_addr, uint8_t reg,
                            const uint8_t *data, uint16_t len,
                            uint32_t timeout_ms);

/**
 * @brief 从 I2C 设备寄存器读取数据（8位 reg addr）
 * @param dev_addr   7位设备地址（无需左移）
 * @param reg        8位寄存器地址
 * @param buf        接收缓冲区
 * @param len        期望读取长度(字节)
 * @param timeout_ms 超时(ms)，0 = 使用 BSP_I2C_DEFAULT_TIMEOUT_MS
 */
bsp_err_t bsp_i2c_read_reg(uint8_t dev_addr, uint8_t reg,
                           uint8_t *buf, uint16_t len,
                           uint32_t timeout_ms);

/* ========================================================================== */
/*                    16 位寄存器地址 读写接口                                   */
/* ========================================================================== */

/**
 * @brief 向 I2C 设备寄存器写入数据（16位 reg addr，大端序）
 * @note  适用于 EEPROM(AT24C256)、大容量传感器 FIFO 等
 */
bsp_err_t bsp_i2c_write_reg16(uint8_t dev_addr, uint16_t reg,
                              const uint8_t *data, uint16_t len,
                              uint32_t timeout_ms);

/**
 * @brief 从 I2C 设备寄存器读取数据（16位 reg addr，大端序）
 */
bsp_err_t bsp_i2c_read_reg16(uint8_t dev_addr, uint16_t reg,
                             uint8_t *buf, uint16_t len,
                             uint32_t timeout_ms);

/* ========================================================================== */
/*                        Raw 读写接口                                          */
/* ========================================================================== */

/**
 * @brief 直接向 I2C 设备写入数据（无寄存器地址）
 * @note  适用于命令下发、OLED 显存写入等场景
 */
bsp_err_t bsp_i2c_write_raw(uint8_t dev_addr,
                            const uint8_t *data, uint16_t len,
                            uint32_t timeout_ms);

/**
 * @brief 直接从 I2C 设备读取数据（无寄存器地址）
 */
bsp_err_t bsp_i2c_read_raw(uint8_t dev_addr,
                           uint8_t *buf, uint16_t len,
                           uint32_t timeout_ms);

/* ========================================================================== */
/*                          诊断接口                                            */
/* ========================================================================== */

/**
 * @brief 检测设备是否在线（发送地址字节检查 ACK）
 * @param dev_addr   7位设备地址
 * @param timeout_ms 超时(ms)
 * @retval BSP_OK        设备在线（ACK）
 * @retval BSP_ERR_IO    设备离线（NACK）或总线错误
 */
bsp_err_t bsp_i2c_probe(uint8_t dev_addr, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_H__ */