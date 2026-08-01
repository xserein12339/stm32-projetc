
# 设置目录
set(HAL_DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/STM32F1xx_HAL/STM32F1xx_HAL_Driver)
set(CMSIS_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/STM32F1xx_HAL/CMSIS/Core)
set(CMSIS_DEVICE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/STM32F1xx_HAL/CMSIS/Device/ST/STM32F1xx)

# CMSIS Core (纯头文件接口库)
add_library(cmsis_core INTERFACE)
target_include_directories(cmsis_core INTERFACE
    ${CMSIS_CORE_DIR}/Include            # ✅ core_cm3.h, cmsis_gcc.h 等
)

# CMSIS Device (芯片头文件 + 型号宏)
add_library(stm32f1_cmsis_device INTERFACE)
target_include_directories(stm32f1_cmsis_device INTERFACE
    ${CMSIS_DEVICE_DIR}/Include
)
target_link_libraries(stm32f1_cmsis_device INTERFACE cmsis_core)
target_compile_definitions(stm32f1_cmsis_device INTERFACE
    STM32F103xB                       
)


# STM32F1xx HAL Driver
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
)

target_include_directories(stm32f1_hal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}          # stm32f1xx_hal_conf.h 所在目录
    ${HAL_DRIVER_DIR}/Inc                # HAL 驱动头文件
)

target_link_libraries(stm32f1_hal PUBLIC
    stm32f1_cmsis_device                 # 自动传递 CMSIS 路径 + STM32F103xB 宏
)

target_compile_definitions(stm32f1_hal PUBLIC
    USE_HAL_DRIVER                       # HAL 驱动总开关
)

target_compile_options(stm32f1_hal PRIVATE
    -Wall
    -Wno-unused-parameter
)