
# drv静态库
add_library(drv STATIC
    src/drv_mpu6050.c
    src/drv_oled.c
    src/drv_motor.c
    src/drv_led.c
    src/drv_key.c
    src/drv_esp8266.c
)

target_include_directories(drv PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 添加BSP依赖
target_link_libraries(drv
    PUBLIC
        hal
)