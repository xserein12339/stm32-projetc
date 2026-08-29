
# BSP静态库
add_library(bsp STATIC
    src/board_v1.c
    src/bsp_i2c.c
    src/bsp_timer.c
    src/bsp_uart.c
    src/bsp_mpu6050.c
    src/bsp_oled.c
    src/bsp_esp8266.c
    src/bsp_motor.c
    src/bsp_encoder.c
    src/bsp_key.c
    src/bsp_led.c
    src/bsp_wdg.c
    src/bsp_dbg.c
)

target_include_directories(bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_compile_options(bsp PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# 添加BSP依赖
target_link_libraries(bsp
    PUBLIC
        chip_runtime
        freertos_kernel
        dal
    PRIVATE
        stm32f1_hal
)