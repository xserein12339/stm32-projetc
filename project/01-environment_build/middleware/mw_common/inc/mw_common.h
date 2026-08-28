/**
 * @file    mw_common.h
 * @brief   中间件通用数学工具库
 *
 * 提供与平台无关的通用数学函数（整数域实现，无浮点依赖）。
 * 典型使用场景：姿态解算所需的三角函数近似、定点数辅助换算等。
 *
 * 参考文档：本项目《开发手册》5.6 定点运算约定
 * @author  xserein
 * @version v1.0
 */
#ifndef __MW_COMMON_H__
#define __MW_COMMON_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          整数域三角函数                                       */
/* ========================================================================== */

/**
 * @brief   反正切查表插值表长度（覆盖比值区间 [0, 1]，等分 16 段）
 */
#define MW_MATH_ATAN2_TABLE_SIZE   (17U)

/**
 * @brief   整数 atan2（四象限），输出单位为毫度 (milli-degree)
 *
 * 采用 17 点查表 + 线性插值近似 atan，配合象限归约实现四象限 atan2。
 * 输入为两个任意比例的整数（无需归一化，内部按比值计算），
 * 典型输入为加速度计两轴原始读数 (mg)。
 *
 * @param[in]  y  Y 轴分量（任意比例整数，如 mg）
 * @param[in]  x  X 轴分量（任意比例整数，如 mg）
 * @return     atan2(y, x)，单位毫度，范围 (-180000, 180000]
 *
 * @note      最大近似误差约 0.2°（查表插值引入），
 *            作为互补滤波中加速度计角度基准足够（陀螺仪主导短时精度）。
 * @note      线程安全：纯函数，无内部状态，可重入。
 */
int32_t mw_math_atan2_mdg(int32_t y, int32_t x);

#ifdef __cplusplus
}
#endif

#endif /* __MW_COMMON_H__ */
