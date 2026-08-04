#include "bsp_gpio.h"
#include "board_v1_config.h"


/* ======================== 内部 LED 描述符表 ======================== */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    GPIO_PinState on_level;
} led_desc_t;

static const led_desc_t led_table[BSP_LED_ID_MAX] = {
    [BSP_LED_ID_GREEN] = {
        .port     = BSP_LED_GREEN_PORT,
        .pin      = BSP_LED_GREEN_PIN,
        .on_level = BSP_LED_ON_LEVEL,
    },
};

/* ======================== 接口实现 ======================== */

int bsp_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    /* 使能 GPIO 时钟 */
    BSP_LED_GREEN_CLK_ENABLE();

    /* 配置 PC13 为推挽输出 */
    gpio_init.Pin   = BSP_LED_GREEN_PIN;
    gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull  = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;  /* LED 不需要高速切换 */
    HAL_GPIO_Init(BSP_LED_GREEN_PORT, &gpio_init);

    /* 默认关闭 LED */
    HAL_GPIO_WritePin(BSP_LED_GREEN_PORT, BSP_LED_GREEN_PIN, BSP_LED_OFF_LEVEL);

    return 0;
}

void bsp_led_set(bsp_led_id_t led_id, uint8_t on)
{
    if (led_id >= BSP_LED_ID_MAX) return;

    const led_desc_t *desc = &led_table[led_id];
    GPIO_PinState level = on ? desc->on_level
                             : (desc->on_level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(desc->port, desc->pin, level);
}

void bsp_led_toggle(bsp_led_id_t led_id)
{
    if (led_id >= BSP_LED_ID_MAX) return;

    const led_desc_t *desc = &led_table[led_id];
    HAL_GPIO_TogglePin(desc->port, desc->pin);
}