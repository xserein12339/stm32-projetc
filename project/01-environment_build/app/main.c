#include "board_v1.h"
#include "app_hmi.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hal_gpio.h"

/* ======================== 主入口 ======================== */
int main(void)
{
    /* 板级硬件初始化（RCC、GPIO、外设等） */
    if (bsp_init() != 0) {
        for (;;) {}
    }
    

    /* 启动 RTOS 调度器 */
    vTaskStartScheduler();

    for (;;) {}
}