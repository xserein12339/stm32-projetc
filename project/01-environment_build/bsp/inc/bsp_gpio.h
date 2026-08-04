#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>

/**
 * @brief 板载 LED 编号枚举 (逻辑名称，与物理引脚解耦)
 */
typedef enum {
    BSP_LED_ID_GREEN = 0,
    BSP_LED_ID_MAX       /* 哨兵值，用于参数校验 */
} bsp_led_id_t;

/**
 * @brief 初始化所有板级 GPIO (LED、按键等)
 * @return 0: 成功; -1: 失败
 */
int bsp_gpio_init(void);

/**
 * @brief 控制 LED 亮灭
 * @param led_id  LED 逻辑编号
 * @param on      1=亮, 0=灭
 */
void bsp_led_set(bsp_led_id_t led_id, uint8_t on);

/**
 * @brief 翻转 LED 状态
 * @param led_id  LED 逻辑编号
 */
void bsp_led_toggle(bsp_led_id_t led_id);

#endif /* BSP_GPIO_H */