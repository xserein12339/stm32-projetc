
# 芯片目标接口库 
add_library(stm32f103c8tx_target INTERFACE)

# CPU架构与浮点选项
target_compile_options(stm32f103c8tx_target INTERFACE
    -mcpu=cortex-m3
    -mthumb
)

# 芯片型号编译宏定义
target_compile_definitions(stm32f103c8tx_target INTERFACE
    STM32F103xB
)

# 链接脚本与链接器选项
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld"
    CACHE FILEPATH "Linker script for STM32F103C8Tx")

target_link_options(stm32f103c8tx_target INTERFACE
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# 芯片运行时库 (STATIC，承接启动文件与系统初始化)
add_library(chip_runtime STATIC
    startup_stm32f103xb.s
    system_stm32f1xx.c
    fault_handlers.c
)

# 获取芯片宏和架构标志
target_link_libraries(chip_runtime 
    PUBLIC 
        stm32f103c8tx_target
        stm32f1_cmsis_device
        )

target_include_directories(chip_runtime PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}            # 本地 system_stm32f1xx.h
)
