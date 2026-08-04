
# BSP静态库
add_library(bsp STATIC
    src/board_v1.c
    src/bsp_gpio.c

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
        dal_display
        freertos_kernel
    PRIVATE
        stm32f1_hal
)