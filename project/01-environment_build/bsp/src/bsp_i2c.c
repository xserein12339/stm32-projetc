/**
 * @file    bsp_i2c.c
 * @brief   板级 I2C BSP 层实现 v1.5
 * @note    - [v1.5] read_reg 改用 HAL_I2C_Mem_Read，保证 RESTART 原子事务
 *          - [v1.5] write_reg 改用 HAL_I2C_Mem_Write，消除栈缓冲区拷贝
 *          - [v1.5] 总线恢复增加 SCL 钳位检测，防止误判恢复成功
 *          - [v1.4] 采用静态 MspInit/MspDeInit 注入，避免全局符号冲突
 *          - [v1.3] 阻塞模式 + 超时保护 + 总线自动恢复
 *          - [v1.3] FreeRTOS 互斥锁（init 时创建）
 *          - [v1.3] 所有读写接口内部自动加锁，保证事务原子性
 * @author  xserein
 * @version v1.5
 */

#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_i2c.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

/* ========================================================================== */
/*                         编译期配置                                          */
/* ========================================================================== */

#ifndef BSP_I2C_MAX_STACK_BUF
#define BSP_I2C_MAX_STACK_BUF  (64U)
#endif

/* ========================================================================== */
/*                         静态全局变量                                        */
/* ========================================================================== */

static I2C_HandleTypeDef s_hi2c;
static bool              s_initialized = false;
static SemaphoreHandle_t s_i2c_mutex   = NULL;

/* ========================================================================== */
/*              Static MspInit / MspDeInit（避免全局符号冲突）                   */
/* ========================================================================== */

static void _i2c_msp_init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {
            .Pin   = BSP_I2C1_SCL_PIN | BSP_I2C1_SDA_PIN,
            .Mode  = GPIO_MODE_AF_OD,
            .Pull  = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_HIGH,
        };
        HAL_GPIO_Init(BSP_I2C1_GPIO_PORT, &gpio);

        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

static void _i2c_msp_deinit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_DISABLE();
    }
}

/* ========================================================================== */
/*                        内部辅助函数                                          */
/* ========================================================================== */

static uint32_t _get_timeout(uint32_t timeout_ms)
{
    return (timeout_ms == 0) ? BSP_I2C_DEFAULT_TIMEOUT_MS : timeout_ms;
}

static void _config_gpio(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {
        .Pin   = BSP_I2C1_SCL_PIN | BSP_I2C1_SDA_PIN,
        .Mode  = GPIO_MODE_AF_OD,
        .Pull  = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(BSP_I2C1_GPIO_PORT, &gpio);
}

static void _gpio_recovery_mode(void)
{
    GPIO_InitTypeDef gpio = {
        .Mode  = GPIO_MODE_OUTPUT_OD,
        .Pull  = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };

    gpio.Pin = BSP_I2C1_SCL_PIN;
    HAL_GPIO_Init(BSP_I2C1_GPIO_PORT, &gpio);

    gpio.Pin = BSP_I2C1_SDA_PIN;
    HAL_GPIO_Init(BSP_I2C1_GPIO_PORT, &gpio);
}

static uint8_t _gpio_read_sda(void)
{
    return (HAL_GPIO_ReadPin(BSP_I2C1_GPIO_PORT, BSP_I2C1_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t _gpio_read_scl(void)
{
    return (HAL_GPIO_ReadPin(BSP_I2C1_GPIO_PORT, BSP_I2C1_SCL_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static void _gpio_write_scl(uint8_t level)
{
    HAL_GPIO_WritePin(BSP_I2C1_GPIO_PORT, BSP_I2C1_SCL_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void _gpio_write_sda(uint8_t level)
{
    HAL_GPIO_WritePin(BSP_I2C1_GPIO_PORT, BSP_I2C1_SDA_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void _gpio_delay_us(uint32_t us)
{
#if defined(BSP_TIMER_APB_CLK_HZ)
    extern void bsp_timer_delay_us(uint32_t us);
    bsp_timer_delay_us(us);
#else
    volatile uint32_t count = us * ((SystemCoreClock / 1000000U) / 4U);
    while (count--) { __NOP(); }
#endif
}

/* ========================================================================== */
/*               [v1.5] 纯 GPIO 级总线恢复（含 SCL 钳位检测）                   */
/* ========================================================================== */

/**
 * @brief [v1.5] 增加 SCL 钳位检测
 *
 * @par v1.4 遗留问题
 *      仅检测 SDA 电平。若从机将 SCL 钳位为低（Clock Stretching 异常或硬件故障），
 *      时钟翻转对 SDA 无效，循环结束后 SDA 仍为低，但代码仅在循环内检查 SDA，
 *      循环退出后未再次验证，可能误判为恢复成功。
 *
 * @par v1.5 修正
 *      1. 循环内同时检查 SDA 和 SCL
 *      2. 循环结束后强制验证 SDA == HIGH && SCL == HIGH
 *      3. 任一信号被钳位即返回 BSP_ERR_IO
 */
static bsp_err_t _bus_recovery_gpio_only(void)
{
    _gpio_recovery_mode();

    /* 先检查 SCL 是否已被钳位 */
    if (!_gpio_read_scl()) {
        return BSP_ERR_IO;  /* SCL 被外部器件拉低，无法通过软件恢复 */
    }

    uint8_t recovered = 0;
    for (uint8_t i = 0; i < BSP_I2C_BUS_RECOVERY_CLKS; i++) {
        if (_gpio_read_sda()) {
            recovered = 1;
            break;
        }
        _gpio_write_scl(0);
        _gpio_delay_us(5);
        _gpio_write_scl(1);
        _gpio_delay_us(5);

        /* [v1.5] 每个时钟周期也检查 SCL 是否被钳位 */
        if (!_gpio_read_scl()) {
            return BSP_ERR_IO;
        }
    }

    /* [v1.5] 最终验证：SDA 和 SCL 都必须为高 */
    if (!recovered || !_gpio_read_sda() || !_gpio_read_scl()) {
        return BSP_ERR_IO;
    }

    /* STOP 条件 */
    _gpio_write_sda(0);
    _gpio_delay_us(5);
    _gpio_write_scl(1);
    _gpio_delay_us(5);
    _gpio_write_sda(1);
    _gpio_delay_us(5);

    return BSP_OK;
}

/* ========================================================================== */
/*               完整总线恢复（含外设重建，供运行时调用）                        */
/* ========================================================================== */

bsp_err_t bsp_i2c_bus_recovery(void)
{
    if (s_initialized) {
        HAL_I2C_DeInit(&s_hi2c);
        __HAL_RCC_I2C1_CLK_DISABLE();
    }

    bsp_err_t ret = _bus_recovery_gpio_only();

    if (s_initialized) {
        __HAL_RCC_I2C1_CLK_ENABLE();
        _config_gpio();
        if (HAL_I2C_Init(&s_hi2c) != HAL_OK) {
            return BSP_ERR_IO;
        }
    }

    return ret;
}

/* ========================================================================== */
/*                        互斥锁接口                                            */
/* ========================================================================== */

bsp_err_t bsp_i2c_lock(uint32_t timeout_ms)
{
    if (!s_initialized || s_i2c_mutex == NULL) return BSP_ERR_NOT_INIT;

    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTake(s_i2c_mutex, ticks) == pdTRUE) ? BSP_OK : BSP_ERR_BUSY;
}

bsp_err_t bsp_i2c_unlock(void)
{
    if (!s_initialized || s_i2c_mutex == NULL) return BSP_ERR_NOT_INIT;
    return (xSemaphoreGive(s_i2c_mutex) == pdTRUE) ? BSP_OK : BSP_ERR_FAIL;
}

/* ========================================================================== */
/*                        生命周期接口                                          */
/* ========================================================================== */

bsp_err_t bsp_i2c_init(uint32_t freq_hz)
{
    if (s_initialized) return BSP_ERR_BUSY;

    s_hi2c.Instance             = BSP_I2C1_PORT;
    s_hi2c.MspInitCallback      = _i2c_msp_init;
    s_hi2c.MspDeInitCallback    = _i2c_msp_deinit;

    s_hi2c.Init.ClockSpeed      = (freq_hz == 0) ? 100000U : freq_hz;
    s_hi2c.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    s_hi2c.Init.OwnAddress1     = 0;
    s_hi2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    s_hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    s_hi2c.Init.OwnAddress2     = 0;
    s_hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    s_hi2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&s_hi2c) != HAL_OK) {
        __HAL_RCC_I2C1_CLK_DISABLE();
        return BSP_ERR_IO;
    }

    s_i2c_mutex = xSemaphoreCreateMutex();
    if (s_i2c_mutex == NULL) {
        HAL_I2C_DeInit(&s_hi2c);
        return BSP_ERR_NOMEM;
    }

    s_initialized = true;
    return BSP_OK;
}

bsp_err_t bsp_i2c_deinit(void)
{
    if (!s_initialized) return BSP_ERR_NOT_INIT;

    GPIO_InitTypeDef gpio_sda_check = {
        .Pin  = BSP_I2C1_SDA_PIN,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL,
    };
    HAL_GPIO_Init(BSP_I2C1_GPIO_PORT, &gpio_sda_check);

    if (HAL_GPIO_ReadPin(BSP_I2C1_GPIO_PORT, BSP_I2C1_SDA_PIN) == GPIO_PIN_RESET) {
        (void)_bus_recovery_gpio_only();
    }

    HAL_I2C_DeInit(&s_hi2c);

    GPIO_InitTypeDef gpio_default = {
        .Pin   = BSP_I2C1_SCL_PIN | BSP_I2C1_SDA_PIN,
        .Mode  = GPIO_MODE_INPUT,
        .Pull  = GPIO_NOPULL,
    };
    HAL_GPIO_Init(BSP_I2C1_GPIO_PORT, &gpio_default);

    if (s_i2c_mutex != NULL) {
        vSemaphoreDelete(s_i2c_mutex);
        s_i2c_mutex = NULL;
    }

    s_initialized = false;
    return BSP_OK;
}

/* ========================================================================== */
/*          [v1.5] 8 位寄存器地址 读写接口（Mem API + RESTART 原子事务）         */
/* ========================================================================== */

/**
 * @brief [v1.5] 改用 HAL_I2C_Mem_Write，消除栈缓冲区 + memcpy
 * @note  HAL_I2C_Mem_Write 内部直接将 reg_addr 作为首字节发送，
 *        无需用户手动拼接 tx_buf，减少一次内存拷贝
 */
bsp_err_t bsp_i2c_write_reg(uint8_t dev_addr, uint8_t reg,
                            const uint8_t *data, uint16_t len,
                            uint32_t timeout_ms)
{
    if (!s_initialized) return BSP_ERR_NOT_INIT;
    if (data == NULL || len == 0) return BSP_ERR_PARAM;

    uint32_t timeout = _get_timeout(timeout_ms);
    uint16_t addr = (uint16_t)(dev_addr << 1);

    if (bsp_i2c_lock(timeout_ms) != BSP_OK) return BSP_ERR_BUSY;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&s_hi2c, addr, reg,
                                                  I2C_MEMADD_SIZE_8BIT,
                                                  (uint8_t *)data, len, timeout);
    bsp_i2c_unlock();
    return (status == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}

/**
 * @brief [v1.5] 改用 HAL_I2C_Mem_Read，保证 RESTART 原子事务
 *
 * @par v1.4 问题
 *      使用 Transmit(reg) + Receive(data) 两步操作，中间产生 STOP → START。
 *      在多主环境下，STOP 后总线释放，其他主设备可能插入事务，
 *      导致目标从机的寄存器指针被重置或状态机错乱。
 *
 * @par v1.5 修正
 *      HAL_I2C_Mem_Read 内部使用 RESTART 条件连接写地址和读数据阶段，
 *      整个操作为单一原子事务，符合 I2C 协议规范的 Combined Format。
 */
bsp_err_t bsp_i2c_read_reg(uint8_t dev_addr, uint8_t reg,
                           uint8_t *buf, uint16_t len,
                           uint32_t timeout_ms)
{
    if (!s_initialized) return BSP_ERR_NOT_INIT;
    if (buf == NULL || len == 0) return BSP_ERR_PARAM;

    uint32_t timeout = _get_timeout(timeout_ms);
    uint16_t addr = (uint16_t)(dev_addr << 1);

    if (bsp_i2c_lock(timeout_ms) != BSP_OK) return BSP_ERR_BUSY;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&s_hi2c, addr, reg,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 buf, len, timeout);
    bsp_i2c_unlock();
    return (status == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}

/* ========================================================================== */
/*          [v1.5] 16 位寄存器地址 读写接口（Mem API + RESTART 原子事务）        */
/* ========================================================================== */

bsp_err_t bsp_i2c_write_reg16(uint8_t dev_addr, uint16_t reg,
                              const uint8_t *data, uint16_t len,
                              uint32_t timeout_ms)
{
    if (!s_initialized) return BSP_ERR_NOT_INIT;
    if (data == NULL || len == 0) return BSP_ERR_PARAM;

    uint32_t timeout = _get_timeout(timeout_ms);
    uint16_t addr = (uint16_t)(dev_addr << 1);

    if (bsp_i2c_lock(timeout_ms) != BSP_OK) return BSP_ERR_BUSY;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&s_hi2c, addr, reg,
                                                  I2C_MEMADD_SIZE_16BIT,
                                                  (uint8_t *)data, len, timeout);
    bsp_i2c_unlock();
    return (status == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}

bsp_err_t bsp_i2c_read_reg16(uint8_t dev_addr, uint16_t reg,
                             uint8_t *buf, uint16_t len,
                             uint32_t timeout_ms)
{
    if (!s_initialized) return BSP_ERR_NOT_INIT;
    if (buf == NULL || len == 0) return BSP_ERR_PARAM;

    uint32_t timeout = _get_timeout(timeout_ms);
    uint16_t addr = (uint16_t)(dev_addr << 1);

    if (bsp_i2c_lock(timeout_ms) != BSP_OK) return BSP_ERR_BUSY;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&s_hi2c, addr, reg,
                                                 I2C_MEMADD_SIZE_16BIT,
                                                 buf, len, timeout);
    bsp_i2c_unlock();
    return (status == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}

/* ========================================================================== */
/*                        Raw 读写接口                                          */
/* ========================================================================== */

bsp_err_t bsp_i2c_write_raw(uint8_t dev_addr,
                            const uint8_t *data, uint16_t len,
                            uint32_t timeout_ms)
{
    if (!s_initialized) return BSP_ERR_NOT_INIT;
    if (data == NULL || len == 0) return BSP_ERR_PARAM;

    uint32_t timeout = _get_timeout(timeout_ms);
    uint16_t addr = (uint16_t)(dev_addr << 1);

    if (bsp_i2c_lock(timeout_ms) != BSP_OK) return BSP_ERR_BUSY;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&s_hi2c, addr,
                                                        (uint8_t *)data, len, timeout);
    bsp_i2c_unlock();
    return (status == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}

bsp_err_t bsp_i2c_read_raw(uint8_t dev_addr,
                           uint8_t *buf, uint16_t len,
                           uint32_t timeout_ms)
{
    if (!s_initialized) return BSP_ERR_NOT_INIT;
    if (buf == NULL || len == 0) return BSP_ERR_PARAM;

    uint32_t timeout = _get_timeout(timeout_ms);
    uint16_t addr = (uint16_t)(dev_addr << 1);

    if (bsp_i2c_lock(timeout_ms) != BSP_OK) return BSP_ERR_BUSY;

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&s_hi2c, addr, buf, len, timeout);
    bsp_i2c_unlock();
    return (status == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}

/* ========================================================================== */
/*                          诊断接口                                            */
/* ========================================================================== */

bsp_err_t bsp_i2c_probe(uint8_t dev_addr, uint32_t timeout_ms)
{
    if (!s_initialized) return BSP_ERR_NOT_INIT;

    uint32_t timeout = _get_timeout(timeout_ms);
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(
        &s_hi2c, (uint16_t)(dev_addr << 1), 1, timeout);

    return (status == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}