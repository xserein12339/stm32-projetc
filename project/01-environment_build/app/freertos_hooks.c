/**
 * @file    freertos_hooks.c
 * @brief   FreeRTOS 移植钩子实现 v1.1
 *
 * v1.1：vApplicationTickHook（configUSE_TICK_HOOK=1）承担两项职责：
 *   1. HAL_IncTick -- FreeRTOS 接管 SysTick 后 HAL 时基冻结，所有
 *      HAL_*_Mem_Read / 超时判断失效，此处在 tick hook 恢复时基；
 *   2. bsp_key_tick_scan -- 按键消抖确认（EXTI 仅记边沿，稳定确认
 *      依赖 1ms 周期采样，见 bsp_key.c v2.2）。
 *
 * @author  xserein
 * @version v1.1
 */
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include "bsp_key.h"

/*
 * vApplicationGetIdleTaskMemory
 */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
static StaticTask_t xIdleTaskTCB;
static StackType_t  uxIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint16_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}
#endif

/**
 * @brief   1ms tick 钩子（SysTick ISR 上下文，保持极短）
 * @warning 严禁调用阻塞 API；FromISR 类调用须带 portYIELD_FROM_ISR
 */
void vApplicationTickHook(void)
{
    HAL_IncTick();          /* 恢复 HAL 时基（I2C/UART 等超时依赖） */
    bsp_key_tick_scan();    /* 按键消抖确认（2 键 × 数条指令） */
}

/*
 * vApplicationStackOverflowHook
 * 使用内联汇编替代 __BKPT，避免依赖 CMSIS 头文件
 */
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    /* 栈溢出是致命错误，触发硬件断点等待调试器捕获 */
    __asm volatile("bkpt #0");

    while (1) {}
}
#endif
