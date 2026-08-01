# =============================================================================
# bsp/CMakeLists.txt - Board Support Package
# =============================================================================

# System Init (system_stm32f1xx.c)
get_target_property(CMSIS_DEVICE_INC stm32f1_cmsis_device INTERFACE_INCLUDE_DIRECTORIES)

add_library(stm32f1_system_init STATIC
    ${CMSIS_DEVICE_INC}/../Source/Templates/system_stm32f1xx.c
)

# 链接 stm32f1_hal
target_link_libraries(stm32f1_system_init PUBLIC
    stm32f1_hal
)

# BSP 静态库
add_library(bsp STATIC
    
    src/bsp_uart.c

    # 启动文件
    ${CMAKE_CURRENT_SOURCE_DIR}/startup/startup_stm32f103xb.s
)

target_include_directories(bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 3. 导出链接脚本
set(BSP_LINKER_SCRIPT
    "${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F103C8Tx_FLASH.ld"
    CACHE FILEPATH "STM32F103C8Tx Linker Script"
)

# 4. BSP 依赖关系
target_link_libraries(bsp PUBLIC
    stm32f1_hal
    stm32f1_system_init
)