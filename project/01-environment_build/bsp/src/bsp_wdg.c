/**
 * @file    bsp_wdg.c
 * @brief   独立看门狗（IWDG）BSP 层实现
 *
 * 时序计算（RM0008 §18.2）：
 *   LSI ≈ 40kHz，PRE=64（IWDG_PR=0b100）-> 计数时钟 ≈ 625Hz
 *   重装值 = timeout_ms * 625 / 1000，上限 0xFFF（约 10.5s）
 *
 * @author  xserein
 * @version v1.0
 */
#include "bsp_wdg.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>

/** IWDG 句柄（单实例，静态分配） */
static IWDG_HandleTypeDef s_hiwdg;

/** 初始化完成标志（refresh 未初始化时静默返回） */
static bool s_initialized = false;

bsp_err_t bsp_wdg_init(uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        timeout_ms = BSP_WDG_DEFAULT_TIMEOUT_MS;
    }

    /* 重装值上限 0xFFF：625Hz 下约 10.5s */
    uint32_t reload = (timeout_ms * 625U + 999U) / 1000U;
    if (reload > 0xFFFU) {
        return BSP_ERR_PARAM;
    }

    s_hiwdg.Instance = IWDG;
    s_hiwdg.Init.Prescaler = IWDG_PRESCALER_64;   /* 40kHz/64 = 625Hz */
    s_hiwdg.Init.Reload    = (uint32_t)reload;

    if (HAL_IWDG_Init(&s_hiwdg) != HAL_OK) {
        return BSP_ERR_FAIL;
    }

    s_initialized = true;
    return BSP_OK;
}

bsp_err_t bsp_wdg_refresh(void)
{
    if (!s_initialized) {
        return BSP_OK;
    }

    /* HAL_IWDG_Refresh 仅写 KR=0xAAAA，单寄存器访问，ISR 安全 */
    HAL_IWDG_Refresh(&s_hiwdg);
    return BSP_OK;
}
