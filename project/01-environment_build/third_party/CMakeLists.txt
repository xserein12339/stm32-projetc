# ==================== FreeRTOS 内核库 ====================
set(FREERTOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS)

add_library(freertos_kernel STATIC
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM3/port.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

target_include_directories(freertos_kernel PUBLIC
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM3
    ${FREERTOS_DIR}/Config
)

# 获取芯片型号宏
target_link_libraries(freertos_kernel PUBLIC stm32f103c8tx_target)

# FreeRTOS 全局编译开关
target_compile_definitions(freertos_kernel PUBLIC
    USE_FREERTOS
)

# ==================== STM32F1xx HAL 驱动 ====================
add_library(stm32f1_hal STATIC
    # --- 基础核心模块 ---
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_cortex.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_rcc.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_gpio.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_dma.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_flash.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_pwr.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_exti.c

    # --- 已启用的外设模块 ---
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_uart.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_spi.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_i2c.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_adc.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_tim.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_rtc.c
    ${HAL_DRIVER_DIR}/Src/stm32f1xx_hal_iwdg.c
)

# 添加芯片依赖宏
target_link_libraries(stm32f1_hal PUBLIC
    stm32f103c8tx_target    
)

target_include_directories(stm32f1_hal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}          # stm32f1xx_hal_conf.h 所在目录
    ${HAL_DRIVER_DIR}/Inc                # HAL 驱动头文件
)

target_link_libraries(stm32f1_hal PUBLIC
    stm32f1_cmsis_device                 # 自动传递 CMSIS 路径
)

target_compile_definitions(stm32f1_hal PUBLIC
    USE_HAL_DRIVER                       # HAL 驱动总开关
)

# 编译设置
target_compile_options(stm32f1_hal PRIVATE
    -Wall
    -Wno-unused-parameter
)


