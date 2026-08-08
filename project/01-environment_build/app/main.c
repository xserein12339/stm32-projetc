#include "board_v1.h"
#include "dal_key.h"
#include "dal_led.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 应用启动任务：按键控制LED亮灭
 */
void app_start_task(void *arg)
{
    /* 获取并初始化按键和LED设备 */
    dal_key_dev_t *key1 = dal_key_get_dev("key1");
    if (key1) {
        dal_key_init(key1);
    }

    dal_led_dev_t *led1 = dal_led_get_dev("led1");
    if (led1) {
        dal_led_init(led1);
    }

    int key_state = 0;
    dal_led_state_t led_state = DAL_LED_OFF;

    while (1) {
        /* 读取按键逻辑状态（1=按下，0=释放）*/
        if (key1 && dal_key_get_state(key1, &key_state) == DAL_OK) {
            /* 按下则点亮，释放则熄灭 */
            led_state = (key_state == 1) ? DAL_LED_ON : DAL_LED_OFF;
            dal_led_set_state(led1, led_state);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main(void)
{
    if (bsp_init() != BSP_OK) {
        Error_Handler();
    }

    xTaskCreate(app_start_task, "AppStart", 256, NULL, 2, NULL);

    vTaskStartScheduler();
    Error_Handler(); 
    return 0;
}