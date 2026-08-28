/**
 * @file    mw_filter.h
 * @brief   常用定点数字滤波器中间件 
 * @note    支持：一阶低通滤波、滑动平均滤波、一维卡尔曼滤波。
 *
 * @author  xserein
 * @version v1.0
 */
#ifndef __MW_FILTER_H__
#define __MW_FILTER_H__

#include <stdint.h>
#include <stdbool.h>
#include "q15_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            一阶低通滤波器 (LPF)                              */
/* ========================================================================== */
/**
 * @brief 一阶低通滤波器结构体 (指数加权移动平均)
 * @note  y[n] = α * x[n] + (1-α) * y[n-1]
 */
typedef struct {
    q15_t alpha;      ///< 滤波系数 (0~1, Q15), 越大响应越快
    q15_t prev_out;   ///< 上一拍输出值 (Q15)
} mw_filter_lpf1_t;

/**
 * @brief 初始化一阶低通滤波器
 * @param f     滤波器对象指针
 * @param alpha 滤波系数 (Q15), 建议范围 [0.01, 1.0]
 */
void mw_filter_lpf1_init(mw_filter_lpf1_t *f, q15_t alpha);

/**
 * @brief 更新一阶低通滤波器
 * @param f  滤波器对象指针
 * @param in 本次输入值 (Q15)
 * @return   Q15 滤波输出
 */
q15_t mw_filter_lpf1_update(mw_filter_lpf1_t *f, q15_t in);

/* ========================================================================== */
/*                            滑动平均滤波器 (MAVG)                             */
/* ========================================================================== */

/**
 * @brief 滑动平均滤波器结构体 (循环缓冲区)
 * @note  输出 = 窗口内所有采样值的算术平均
 *        内部 sum 为 Q15 原始值的整数累加和 (非 Q30 定点格式)
 */
typedef struct {
    q15_t  *buffer;   ///< 循环缓冲区指针 (外部提供)
    uint16_t size;    ///< 窗口大小 (必须 > 0, 建议 ≤ 65535 防 sum 溢出)
    uint16_t index;   ///< 当前写入位置 (内部管理)
    q30_t   sum;      ///< 窗口内 Q15 原始值的整数累加和
    bool    is_full;  ///< 缓冲区是否已填满 (内部管理)
} mw_filter_mavg_t;

/**
 * @brief 初始化滑动平均滤波器
 * @param f      滤波器对象指针
 * @param buf    外部提供的缓冲区 (数组长度至少为 size)
 * @param size   窗口大小 (建议 2 的幂次以优化除法；最大 65535)
 */
void mw_filter_mavg_init(mw_filter_mavg_t *f, q15_t *buf, uint16_t size);

/**
 * @brief 更新滑动平均滤波器
 * @param f  滤波器对象指针
 * @param in 本次输入值 (Q15)
 * @return   Q15 滤波输出 (窗口平均值)
 */
q15_t mw_filter_mavg_update(mw_filter_mavg_t *f, q15_t in);

/**
 * @brief 重置滑动平均滤波器 (清空缓冲区)
 * @param f  滤波器对象指针
 */
void mw_filter_mavg_reset(mw_filter_mavg_t *f);

/* ========================================================================== */
/*                           一维卡尔曼滤波器 (Kalman 1D)                      */
/* ========================================================================== */

/**
 * @brief 一维卡尔曼滤波器结构体 (标量)
 * @note  适用于单变量传感器融合 (温度、角度、电压等)。
 *        输入输出接口为 Q15，内部协方差 P 使用 Q30 全精度运算，
 *        避免 Q15 动态范围不足导致的饱和与精度丢失。
 *
 * @par 参数物理含义:
 *   - Q: 过程噪声协方差，反映模型不确定度。越大越信任测量值。
 *   - R: 测量噪声协方差，反映传感器噪声水平。越大越信任预测值。
 *   - Q/R 比值决定滤波器带宽，建议通过 float_to_q15() 配置。
 */
typedef struct {
    /* --- 可调参数 (Q15) --- */
    q15_t q;          ///< 过程噪声协方差 (Q15), 建议 0.001 ~ 0.1
    q15_t r;          ///< 测量噪声协方差 (Q15), 建议 0.01 ~ 0.5

    /* --- 运行状态 (内部管理) --- */
    q15_t x;          ///< 当前状态估计值 (Q15)
    q30_t p;          ///< 当前估计协方差 (Q30), 内部全精度运算
} mw_filter_kalman1d_t;

/**
 * @brief 初始化一维卡尔曼滤波器
 * @param f      滤波器对象指针
 * @param q      过程噪声协方差 (Q15)
 * @param r      测量噪声协方差 (Q15)
 * @param init_x 初始状态估计值 (Q15)
 */
void mw_filter_kalman1d_init(mw_filter_kalman1d_t *f, q15_t q, q15_t r, q15_t init_x);

/**
 * @brief 重置一维卡尔曼滤波器 (保留参数，重置状态及协方差)
 * @param f      滤波器对象指针
 * @param init_x 初始状态估计值 (Q15)
 */
void mw_filter_kalman1d_reset(mw_filter_kalman1d_t *f, q15_t init_x);

/**
 * @brief 执行一步一维卡尔曼滤波
 * @param f  滤波器对象指针
 * @param z  本次测量值 (Q15)
 * @return   Q15 最优估计值
 */
q15_t mw_filter_kalman1d_update(mw_filter_kalman1d_t *f, q15_t z);

#ifdef __cplusplus
}
#endif

#endif /* __MW_FILTER_H__ */