#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "bsp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机驱动并注册到 DAL 框架
 * @retval BSP_OK          成功
 * @retval BSP_ERR_FAIL    注册失败
 * @retval BSP_ERR_IO      定时器或 GPIO 初始化失败
 */
bsp_err_t bsp_motor_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MOTOR_H__ */