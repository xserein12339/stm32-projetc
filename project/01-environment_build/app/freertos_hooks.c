#include "FreeRTOS.h"
#include "task.h"

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