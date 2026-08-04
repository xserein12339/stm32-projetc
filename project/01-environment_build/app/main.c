#include "board_v1.h"
#include "bsp_gpio.h"
#include "app_hmi.h"

#include "FreeRTOS.h"
#include "task.h"

/* ======================== 启动自检任务 ======================== */
/**
 * @brief 上电自检任务：确认 BSP 初始化正常后自动删除自身
 * @note  该任务仅用于启动阶段验证，不属于常驻业务
 */
static void startup_selftest_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* 闪烁 3 次 */
    for (int i = 0; i < 3; i++) {
        bsp_led_toggle(BSP_LED_ID_GREEN);       // 亮
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));

        bsp_led_toggle(BSP_LED_ID_GREEN);       // 灭
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
    }

    /* 确保最终状态为灭 */
    bsp_led_set(BSP_LED_ID_GREEN, 0);

    vTaskDelete(NULL);
}

/* ======================== 主入口 ======================== */
int main(void)
{
    /* 板级硬件初始化（RCC、GPIO、外设等） */
    if (bsp_init() != 0) {
        for (;;) {}
    }

    /* 创建启动自检任务 */
    xTaskCreate(startup_selftest_task,
                "selftest",
                configMINIMAL_STACK_SIZE,   /* 自检任务栈需求极小 */
                NULL,
                tskIDLE_PRIORITY + 1,       /* 略高于空闲优先级即可 */
                NULL);

    /* 启动 RTOS 调度器 */
    vTaskStartScheduler();

    for (;;) {}
}