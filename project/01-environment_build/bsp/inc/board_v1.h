#ifndef BOARD_V1_H
#define BOARD_V1_H

#include "bsp_err.h"

void Error_Handler(void);

/**
 * @brief 板级统一初始化入口
 * @return 0: 成功; 非0: 失败
 */
bsp_err_t bsp_init(void);

#endif /* BOARD_V1_H */