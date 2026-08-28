/**
 * @file    bsp_mpu6050.c
 * @brief   板级 MPU6050 BSP 层实现（I2C）v1.2
 * @note    - 依赖 bsp_i2c 进行 I2C 通信
 *          - 实现 dal_imu_ops_t 并注册到 dal_imu 框架
 *          - 严格遵循 datasheet 初始化序列（reset → delay → config）
 *          - 预计算整数缩放因子，避免运行时浮点运算
 *          - ODR 分频基于实际 DLPF 配置动态计算，不可实现时返回错误
 *          - set_power(wake) 完整恢复所有配置寄存器
 * @author  xserein
 * @version v1.2
 */

#include "board_v1.h"
#include "board_v1_config.h"
#include "bsp_mpu6050.h"
#include "bsp_i2c.h"
#include "dal_imu.h"
#include "bsp_timer.h"
#include <string.h>

/* ========================================================================== */
/*                             硬件配置                                          */
/* ========================================================================== */

#define MPU6050_I2C_ADDR_0      (0x68U)
#define MPU6050_I2C_ADDR_1      (0x69U)
#define MPU6050_I2C_TIMEOUT_MS  (100U)
#define MPU6050_WHO_AM_I_VAL    (0x68U)

#define MPU6050_DEFAULT_ACCEL_RANGE   DAL_IMU_ACCEL_RANGE_2G
#define MPU6050_DEFAULT_GYRO_RANGE    DAL_IMU_GYRO_RANGE_250DPS
#define MPU6050_DEFAULT_ODR           DAL_IMU_ODR_100
/*
 * DLPF=4：加速度计带宽 21Hz / 陀螺仪带宽 20Hz，延迟 8.3ms
 * [v1.2] WHY: 原 DLPF=6（5Hz 带宽）对 200Hz 采样的平衡控制过低，
 * 角速度相位滞后过大；DLPF=3（44Hz）在电机振动环境下噪声偏高，
 * 折中取 4（参见《需求分析》FR-ATT-001）
 */
#define MPU6050_DEFAULT_DLPF          (0x04U)

/* ========================================================================== */
/*                         MPU6050 寄存器定义                                    */
/* ========================================================================== */

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_INT_ENABLE    0x38U
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U

#define MPU6050_PWR_DEVICE_RESET  (0x80U)
#define MPU6050_PWR_SLEEP         (0x40U)
#define MPU6050_PWR_CLK_SEL_XGYRO (0x01U)

/* ========================================================================== */
/*                        私有数据结构                                          */
/* ========================================================================== */

typedef struct {
    uint8_t               dev_addr;
    bool                  is_initialized;
    dal_imu_accel_range_t accel_range;
    dal_imu_gyro_range_t  gyro_range;
    dal_imu_odr_t         odr;
    uint8_t               dlpf_cfg;

    int32_t               accel_scale_num;
    int32_t               gyro_scale_num;

    bool                  cal_valid;
    dal_imu_calibration_t cal;
} bsp_mpu6050_priv_t;

static bsp_mpu6050_priv_t  s_mpu_priv;
static dal_imu_dev_t       s_imu_dev;

/* ========================================================================== */
/*                   I2C 读写辅助函数                                            */
/* ========================================================================== */

static dal_err_t _mpu_read_reg(uint8_t reg, uint8_t *val)
{
    bsp_err_t ret = bsp_i2c_read_reg(s_mpu_priv.dev_addr, reg, val, 1,
                                     MPU6050_I2C_TIMEOUT_MS);
    return (ret == BSP_OK) ? DAL_OK : DAL_ERR_FAIL;
}

static dal_err_t _mpu_write_reg(uint8_t reg, uint8_t val)
{
    bsp_err_t ret = bsp_i2c_write_reg(s_mpu_priv.dev_addr, reg, &val, 1,
                                      MPU6050_I2C_TIMEOUT_MS);
    return (ret == BSP_OK) ? DAL_OK : DAL_ERR_FAIL;
}

/**
 * @brief [v1.2] 增加 len 有效性校验
 */
static dal_err_t _mpu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len == 0) return DAL_ERR_PARAM_INVALID;

    bsp_err_t ret = bsp_i2c_read_reg(s_mpu_priv.dev_addr, reg, buf, len,
                                     MPU6050_I2C_TIMEOUT_MS);
    return (ret == BSP_OK) ? DAL_OK : DAL_ERR_FAIL;
}

static uint8_t _mpu_probe_addr(void)
{
    if (bsp_i2c_probe(MPU6050_I2C_ADDR_0, 50) == BSP_OK) return MPU6050_I2C_ADDR_0;
    if (bsp_i2c_probe(MPU6050_I2C_ADDR_1, 50) == BSP_OK) return MPU6050_I2C_ADDR_1;
    return 0;
}

static bool _mpu_verify(void)
{
    uint8_t whoami = 0;
    if (_mpu_read_reg(MPU6050_REG_WHO_AM_I, &whoami) != DAL_OK) return false;
    return (whoami == MPU6050_WHO_AM_I_VAL);
}

/* ========================================================================== */
/*               预计算缩放因子 & ODR 分频                                        */
/* ========================================================================== */

static void _update_accel_scale(bsp_mpu6050_priv_t *priv)
{
    switch (priv->accel_range) {
        case DAL_IMU_ACCEL_RANGE_2G:  priv->accel_scale_num = 2000;  break;
        case DAL_IMU_ACCEL_RANGE_4G:  priv->accel_scale_num = 4000;  break;
        case DAL_IMU_ACCEL_RANGE_8G:  priv->accel_scale_num = 8000;  break;
        case DAL_IMU_ACCEL_RANGE_16G: priv->accel_scale_num = 16000; break;
        default:                      priv->accel_scale_num = 2000;  break;
    }
}

static void _update_gyro_scale(bsp_mpu6050_priv_t *priv)
{
    switch (priv->gyro_range) {
        case DAL_IMU_GYRO_RANGE_250DPS:  priv->gyro_scale_num = 250000;  break;
        case DAL_IMU_GYRO_RANGE_500DPS:  priv->gyro_scale_num = 500000;  break;
        case DAL_IMU_GYRO_RANGE_1000DPS: priv->gyro_scale_num = 1000000; break;
        case DAL_IMU_GYRO_RANGE_2000DPS: priv->gyro_scale_num = 2000000; break;
        default:                         priv->gyro_scale_num = 250000;  break;
    }
}

static uint16_t _get_gyro_internal_rate(uint8_t dlpf_cfg)
{
    return (dlpf_cfg == 0) ? 8000U : 1000U;
}

/**
 * @brief [v1.2] ODR 分频计算，不可实现时通过 out_div 返回实际值供调用者判断
 * @param[out] out_div  计算出的 SMPLRT_DIV 值
 * @retval true   ODR 可精确实现或合理近似
 * @retval false  请求的 ODR 超出硬件能力，无法实现
 */
static bool _odr_to_smplrt_div(dal_imu_odr_t odr, uint8_t dlpf_cfg, uint8_t *out_div)
{
    uint16_t internal_rate = _get_gyro_internal_rate(dlpf_cfg);
    uint16_t target_hz;

    switch (odr) {
        case DAL_IMU_ODR_1:    target_hz = 1;    break;
        case DAL_IMU_ODR_10:   target_hz = 10;   break;
        case DAL_IMU_ODR_25:   target_hz = 25;   break;
        case DAL_IMU_ODR_50:   target_hz = 50;   break;
        case DAL_IMU_ODR_100:  target_hz = 100;  break;
        case DAL_IMU_ODR_200:  target_hz = 200;  break;
        case DAL_IMU_ODR_400:  target_hz = 400;  break;
        case DAL_IMU_ODR_800:  target_hz = 800;  break;
        case DAL_IMU_ODR_1600: target_hz = 1600; break;
        default:               target_hz = 100;  break;
    }

    /* 请求 ODR 超过内部采样率，硬件无法实现 */
    if (target_hz > internal_rate) {
        *out_div = 0;
        return false;
    }

    uint16_t div = (internal_rate / target_hz) - 1;
    if (div > 255) div = 255;
    *out_div = (uint8_t)div;

    /* 检查取整后的实际 ODR 与请求值偏差是否过大（>25%） */
    uint16_t actual_hz = internal_rate / (1 + div);
    if (actual_hz < target_hz * 3 / 4) {
        return false;
    }

    return true;
}

/* ========================================================================== */
/*            [v1.2] 配置重载辅助函数（用于 wake 恢复）                           */
/* ========================================================================== */

/**
 * @brief 将当前 priv 中保存的配置全部写入硬件寄存器
 * @note  用于 init 和 set_power(wake) 共用，消除重复代码
 */
static dal_err_t _mpu_apply_all_config(bsp_mpu6050_priv_t *priv)
{
    dal_err_t ret;

    /* DLPF */
    ret = _mpu_write_reg(MPU6050_REG_CONFIG, priv->dlpf_cfg);
    if (ret != DAL_OK) return ret;

    /* 加速度计量程 */
    uint8_t accel_reg = ((uint8_t)priv->accel_range & 0x03U) << 3;
    ret = _mpu_write_reg(MPU6050_REG_ACCEL_CONFIG, accel_reg);
    if (ret != DAL_OK) return ret;

    /* 陀螺仪量程 */
    uint8_t gyro_reg = ((uint8_t)priv->gyro_range & 0x03U) << 3;
    ret = _mpu_write_reg(MPU6050_REG_GYRO_CONFIG, gyro_reg);
    if (ret != DAL_OK) return ret;

    /* ODR */
    uint8_t smplrt_div;
    if (!_odr_to_smplrt_div(priv->odr, priv->dlpf_cfg, &smplrt_div)) {
        return DAL_ERR_PARAM_INVALID;
    }
    ret = _mpu_write_reg(MPU6050_REG_SMPLRT_DIV, smplrt_div);
    if (ret != DAL_OK) return ret;

    return DAL_OK;
}

/* ========================================================================== */
/*                         DAL ops 实现                                         */
/* ========================================================================== */

static dal_err_t bsp_mpu_ops_init(dal_imu_dev_t *dev)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;
    dal_err_t ret;

    priv->dev_addr = _mpu_probe_addr();
    if (priv->dev_addr == 0) return DAL_ERR_DEPENDENCY;

    if (!_mpu_verify()) return DAL_ERR_NOT_FOUND;

    /* Step A: 设备复位 */
    ret = _mpu_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_DEVICE_RESET);
    if (ret != DAL_OK) return ret;
    bsp_timer_delay_ms(100);

    /* Step B: 唤醒 + X-Gyro PLL */
    ret = _mpu_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_CLK_SEL_XGYRO);
    if (ret != DAL_OK) return ret;
    bsp_timer_delay_ms(10);

    /* Step C: 设置默认参数 */
    priv->accel_range = MPU6050_DEFAULT_ACCEL_RANGE;
    priv->gyro_range  = MPU6050_DEFAULT_GYRO_RANGE;
    priv->odr         = MPU6050_DEFAULT_ODR;
    priv->dlpf_cfg    = MPU6050_DEFAULT_DLPF;

    /* Step D: [v1.2] 统一使用配置重载函数 */
    ret = _mpu_apply_all_config(priv);
    if (ret != DAL_OK) return ret;

    _update_accel_scale(priv);
    _update_gyro_scale(priv);

    priv->is_initialized = true;
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_deinit(dal_imu_dev_t *dev)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;
    if (!priv->is_initialized) return DAL_OK;

    _mpu_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_SLEEP);
    priv->is_initialized = false;
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_selftest(dal_imu_dev_t *dev,
                                      dal_imu_selftest_result_t *result)
{
    (void)dev;
    *result = DAL_IMU_SELFTEST_NOT_IMPL;
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_read(dal_imu_dev_t *dev, dal_imu_data_t *data)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;
    if (!priv->is_initialized) return DAL_ERR_NOT_READY;

    uint8_t raw[14];
    dal_err_t ret = _mpu_read_bytes(MPU6050_REG_ACCEL_XOUT_H, raw, 14);
    if (ret != DAL_OK) return ret;

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t temp = (int16_t)((raw[6] << 8) | raw[7]);
    int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    int32_t accel_x_mg = (ax * priv->accel_scale_num) / 32768;
    int32_t accel_y_mg = (ay * priv->accel_scale_num) / 32768;
    int32_t accel_z_mg = (az * priv->accel_scale_num) / 32768;
    int32_t gyro_x_mdps = (gx * priv->gyro_scale_num) / 32768;
    int32_t gyro_y_mdps = (gy * priv->gyro_scale_num) / 32768;
    int32_t gyro_z_mdps = (gz * priv->gyro_scale_num) / 32768;

    if (priv->cal_valid) {
        accel_x_mg -= priv->cal.accel_offset_mg.x;
        accel_y_mg -= priv->cal.accel_offset_mg.y;
        accel_z_mg -= priv->cal.accel_offset_mg.z;
        gyro_x_mdps -= priv->cal.gyro_offset_mdps.x;
        gyro_y_mdps -= priv->cal.gyro_offset_mdps.y;
        gyro_z_mdps -= priv->cal.gyro_offset_mdps.z;
    }

    data->accel_mg.x = accel_x_mg;
    data->accel_mg.y = accel_y_mg;
    data->accel_mg.z = accel_z_mg;
    data->gyro_mdps.x = gyro_x_mdps;
    data->gyro_mdps.y = gyro_y_mdps;
    data->gyro_mdps.z = gyro_z_mdps;
    data->temp_mc = (int16_t)(((int32_t)temp * 1000) / 340 + 36530);
    data->timestamp_us = 0;
    data->valid_mask = DAL_IMU_MODULE_ACCEL | DAL_IMU_MODULE_GYRO;

    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_fifo_read(dal_imu_dev_t *dev, dal_imu_data_t *buf,
                                       uint32_t max_count, uint32_t *actual)
{
    (void)dev; (void)buf; (void)max_count; (void)actual;
    return DAL_ERR_NOT_SUPPORTED;
}

/**
 * @brief [v1.2] set_odr 对不可实现的 ODR 返回 DAL_ERR_PARAM_INVALID
 */
static dal_err_t bsp_mpu_ops_set_odr(dal_imu_dev_t *dev, uint32_t module,
                                     dal_imu_odr_t odr)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;
    if (!priv->is_initialized) return DAL_ERR_NOT_READY;
    if (module & DAL_IMU_MODULE_MAG) return DAL_ERR_NOT_SUPPORTED;

    uint8_t smplrt_div;
    if (!_odr_to_smplrt_div(odr, priv->dlpf_cfg, &smplrt_div)) {
        return DAL_ERR_PARAM_INVALID;
    }

    dal_err_t ret = _mpu_write_reg(MPU6050_REG_SMPLRT_DIV, smplrt_div);
    if (ret != DAL_OK) return ret;

    priv->odr = odr;
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_set_accel_range(dal_imu_dev_t *dev,
                                             dal_imu_accel_range_t range)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;
    if (!priv->is_initialized) return DAL_ERR_NOT_READY;

    uint8_t reg_val = ((uint8_t)range & 0x03U) << 3;
    dal_err_t ret = _mpu_write_reg(MPU6050_REG_ACCEL_CONFIG, reg_val);
    if (ret != DAL_OK) return ret;

    priv->accel_range = range;
    _update_accel_scale(priv);
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_set_gyro_range(dal_imu_dev_t *dev,
                                            dal_imu_gyro_range_t range)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;
    if (!priv->is_initialized) return DAL_ERR_NOT_READY;

    uint8_t reg_val = ((uint8_t)range & 0x03U) << 3;
    dal_err_t ret = _mpu_write_reg(MPU6050_REG_GYRO_CONFIG, reg_val);
    if (ret != DAL_OK) return ret;

    priv->gyro_range = range;
    _update_gyro_scale(priv);
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_set_fifo_threshold(dal_imu_dev_t *dev, uint16_t threshold)
{
    (void)dev; (void)threshold;
    return DAL_ERR_NOT_SUPPORTED;
}

static dal_err_t bsp_mpu_ops_set_calibration(dal_imu_dev_t *dev,
                                             const dal_imu_calibration_t *cal)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;
    if (!priv->is_initialized) return DAL_ERR_NOT_READY;

    if (cal != NULL) {
        priv->cal = *cal;
        priv->cal_valid = true;
    } else {
        priv->cal_valid = false;
        memset(&priv->cal, 0, sizeof(priv->cal));
    }
    return DAL_OK;
}

/**
 * @brief [v1.2] wake 时完整恢复所有配置寄存器
 * @note  虽然 MPU6050 datasheet 表明睡眠期间寄存器值保持，
 *        但显式重载可防御以下场景：
 *        1. 外部干扰导致寄存器位翻转
 *        2. 未来更换为寄存器不保持的兼容型号（如某些国产替代）
 *        3. 确保 priv 状态与硬件始终一致
 */
static dal_err_t bsp_mpu_ops_set_power(dal_imu_dev_t *dev, bool on)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;
    if (!priv->is_initialized) return DAL_ERR_NOT_READY;

    if (on) {
        /* 唤醒 + 恢复时钟源 */
        dal_err_t ret = _mpu_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_CLK_SEL_XGYRO);
        if (ret != DAL_OK) return ret;
        bsp_timer_delay_ms(10);

        /* [v1.2] 完整重载所有配置 */
        ret = _mpu_apply_all_config(priv);
        if (ret != DAL_OK) return ret;
    } else {
        dal_err_t ret = _mpu_write_reg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_SLEEP);
        if (ret != DAL_OK) return ret;
    }
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_get_state(dal_imu_dev_t *dev, dal_imu_state_t *state)
{
    (void)dev;
    bsp_mpu6050_priv_t *priv = &s_mpu_priv;

    if (!priv->is_initialized) { *state = DAL_IMU_STATE_OFF; return DAL_OK; }

    uint8_t pwr;
    if (_mpu_read_reg(MPU6050_REG_PWR_MGMT_1, &pwr) != DAL_OK) {
        *state = DAL_IMU_STATE_FAULT;
        return DAL_OK;
    }
    *state = (pwr & MPU6050_PWR_SLEEP) ? DAL_IMU_STATE_OFF : DAL_IMU_STATE_IDLE;
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_get_fault(dal_imu_dev_t *dev, uint32_t *fault)
{
    (void)dev;
    *fault = 0U;
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_get_info(dal_imu_dev_t *dev, uint32_t *modules,
                                      uint32_t *capability, uint16_t *fifo_depth)
{
    (void)dev;
    if (!s_mpu_priv.is_initialized) return DAL_ERR_NOT_READY;

    if (modules)     *modules     = DAL_IMU_MODULE_ACCEL | DAL_IMU_MODULE_GYRO;
    if (capability)  *capability  = 0;
    if (fifo_depth)  *fifo_depth  = 0;
    return DAL_OK;
}

static dal_err_t bsp_mpu_ops_set_event_irq_enable(dal_imu_dev_t *dev, bool enable)
{
    (void)dev; (void)enable;
    return DAL_ERR_NOT_SUPPORTED;
}

/* ========================================================================== */
/*                       dal_imu_ops_t 实例                                     */
/* ========================================================================== */

static const dal_imu_ops_t g_bsp_mpu_ops = {
    .init                 = bsp_mpu_ops_init,
    .deinit               = bsp_mpu_ops_deinit,
    .selftest             = bsp_mpu_ops_selftest,
    .read                 = bsp_mpu_ops_read,
    .fifo_read            = bsp_mpu_ops_fifo_read,
    .set_odr              = bsp_mpu_ops_set_odr,
    .set_accel_range      = bsp_mpu_ops_set_accel_range,
    .set_gyro_range       = bsp_mpu_ops_set_gyro_range,
    .set_fifo_threshold   = bsp_mpu_ops_set_fifo_threshold,
    .set_calibration      = bsp_mpu_ops_set_calibration,
    .set_power            = bsp_mpu_ops_set_power,
    .get_state            = bsp_mpu_ops_get_state,
    .get_fault            = bsp_mpu_ops_get_fault,
    .get_info             = bsp_mpu_ops_get_info,
    .set_event_irq_enable = bsp_mpu_ops_set_event_irq_enable,
};

/* ========================================================================== */
/*                     BSP 公共接口                                            */
/* ========================================================================== */

bsp_err_t bsp_mpu6050_init(void)
{
    memset(&s_mpu_priv, 0, sizeof(s_mpu_priv));
    s_mpu_priv.accel_range = MPU6050_DEFAULT_ACCEL_RANGE;
    s_mpu_priv.gyro_range  = MPU6050_DEFAULT_GYRO_RANGE;
    s_mpu_priv.odr         = MPU6050_DEFAULT_ODR;
    s_mpu_priv.dlpf_cfg    = MPU6050_DEFAULT_DLPF;

    s_imu_dev.name     = "mpu6050";
    s_imu_dev.ops      = &g_bsp_mpu_ops;
    s_imu_dev.drv_priv = &s_mpu_priv;

    dal_err_t ret = dal_imu_register(&s_imu_dev);
    if (ret != DAL_OK) {
        (void)dal_imu_unregister(&s_imu_dev);
        return BSP_ERR_IO;
    }
    return BSP_OK;
}