
add_library(stm32f103c8tx_target INTERFACE)

# 芯片型号编译宏定义
target_compile_definitions(stm32f103c8tx_target INTERFACE
    STM32F103xB 
)

# 启动文件 + 链接脚本 + CPU 架构...
target_sources(stm32f103c8tx_target INTERFACE startup/startup_stm32f103c8tx.s)
target_link_options(stm32f103c8tx_target INTERFACE -T${LINKER_SCRIPT})
target_compile_options(stm32f103c8tx_target INTERFACE -mcpu=cortex-m3)