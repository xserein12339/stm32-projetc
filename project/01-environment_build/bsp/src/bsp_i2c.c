/**
 * @file bsp_i2c.c
 * @brief BSP I2C1 驱动实现
 * 
 * @details 
 * 
 * @author xserein
 * @version v1.0
 */

#include "bsp_i2c.h"
#include "board_v1_config.h"


/* ========================================================================== */
/*                               全局变量                                      */
/* ========================================================================== */
/* I2C1句柄与初始化标志 */
static I2C_HandleTypeDef hi2c1;
static bool is_initialized = false;

/* ========================================================================== */
/*                               外部函数                                      */
/* ========================================================================== */
bsp_err_t bsp_i2c_init(void)
{
    if (is_initialized) {
        return BSP_OK;
    }
    /* 使能GPIOB时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置GPIO引脚复用且为上拉 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = BSP_I2C1_SCL_PIN | BSP_I2C1_SDA_PIN;
    gpio.Mode  = GPIO_MODE_AF_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BSP_I2C1_GPIO_PORT, &gpio);

    /* 使能I2C1时钟 */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* 初始化I2C */
    hi2c1.Instance             = BSP_I2C1_PORT;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        return BSP_ERR_IO;
    }

    is_initialized = true;
    return BSP_OK;
}

bsp_err_t bsp_i2c_deinit(void)
{
    if (!is_initialized) {
        return BSP_ERR_NOT_INIT;
    }

    /* 停止I2C外设并关闭时钟 */
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();

    /* 将SCL/SDA恢复为模拟输入，避免漏电 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &gpio);

    is_initialized = false;
    return BSP_OK;
}

bsp_err_t bsp_i2c_write(uint8_t dev_addr, uint8_t reg,
                        const uint8_t *data, uint16_t len)
{
    if (!is_initialized) {
        return BSP_ERR_NOT_INIT;
    }

    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1,
                                             (uint16_t)(dev_addr << 1),
                                             reg, I2C_MEMADD_SIZE_8BIT,
                                             (uint8_t *)data, len, 100);
    return (st == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}

bsp_err_t bsp_i2c_read(uint8_t dev_addr, uint8_t reg,
                       uint8_t *buf, uint16_t len)
{
    if (!is_initialized) {
        return BSP_ERR_NOT_INIT;
    }

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1,
                                            (uint16_t)(dev_addr << 1),
                                            reg, I2C_MEMADD_SIZE_8BIT,
                                            buf, len, 100);
    return (st == HAL_OK) ? BSP_OK : BSP_ERR_IO;
}