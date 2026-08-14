
# dal静态库
add_library(dal STATIC
    src/dal_registry.c
    src/dal_key.c
    src/dal_led.c
    src/dal_motor.c
    src/dal_encoder.c
    src/dal_display.c
    src/dal_imu.c
    src/dal_wifi.c
)

target_include_directories(dal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
