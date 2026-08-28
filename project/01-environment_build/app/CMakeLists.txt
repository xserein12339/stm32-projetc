# 添加子模块
add_subdirectory(app_hmi)

# 生成最终可执行文件
add_executable(${PROJECT_NAME}.elf
    main.c              # 主程序入口
    freertos_hooks.c
)

# 链接应用模块与BSP,MilldeWare,svc库
target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    app_hmi
    svc_att_algo              # 姿态解算服务
    svc_mot_ctrl              # 电机控制服务
    bsp                     # 提供驱动、HAL、SystemInit、启动文件、链接脚本
)

# 链接选项 (从 BSP 获取链接脚本)
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -Wl,--gc-sections  
    --specs=nano.specs      # newlib-nano (20KB RAM 必须)
    --specs=nosys.specs     # 无系统调用
    -Wl,--print-memory-usage
    -Wl,--wrap=malloc 
    -Wl,--wrap=free  
)

# 生成 hex/bin 固件 & 打印大小
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE}    $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating firmware and printing size..."
)

# 设置烧录
add_custom_target(flash
    COMMAND openocd
        -f interface/stlink.cfg
        -f target/stm32f1x.cfg
        -c "program $<TARGET_FILE:${PROJECT_NAME}.elf> verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing ${PROJECT_NAME}.elf via OpenOCD..."
)