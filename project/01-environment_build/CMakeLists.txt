cmake_minimum_required(VERSION 3.20)

# =============================================================================
# 工具链文件 (必须在 project() 之前)
# =============================================================================
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32f1-gcc.cmake"
        CACHE FILEPATH "STM32F1 GCC toolchain file")
endif()

project(stm32f103_test LANGUAGES C ASM)

# =============================================================================
# 全局编译配置
# =============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# 全局编译选项 (所有子模块自动继承)
add_compile_options(
    -fno-common
    -Wall
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Og -g3 -gdwarf-2)
    add_compile_definitions(DEBUG)
else()
    add_compile_options(-Os)
    add_compile_definitions(NDEBUG)
endif()

# =============================================================================
# 第三方路径变量 (target，third_party依赖)
# =============================================================================
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
set(HAL_DRIVER_DIR  ${THIRD_PARTY_DIR}/STM32F1xx_HAL/STM32F1xx_HAL_Driver)
set(CMSIS_CORE_DIR  ${THIRD_PARTY_DIR}/STM32F1xx_HAL/CMSIS/Core)
set(CMSIS_DEVICE_DIR ${THIRD_PARTY_DIR}/STM32F1xx_HAL/CMSIS/Device/ST/STM32F1xx)

# =============================================================================
# CMSIS 纯头文件接口库 (零编译，被 target/ 和 third_party/ 共同依赖)
# =============================================================================
add_library(cmsis_core INTERFACE)
target_include_directories(cmsis_core INTERFACE
    ${CMSIS_CORE_DIR}/Include            # core_cm3.h, cmsis_gcc.h 等
)

add_library(stm32f1_cmsis_device INTERFACE)
target_include_directories(stm32f1_cmsis_device INTERFACE
    ${CMSIS_DEVICE_DIR}/Include          # stm32f1xx.h, system_stm32f1xx.h 等
)
target_link_libraries(stm32f1_cmsis_device INTERFACE cmsis_core)

# =============================================================================
# 模块添加 (严格按依赖顺序，自底向上)
# =============================================================================
add_subdirectory(target/stm32_f103c8t6) # 目标库 (启动文件、链接脚本)
add_subdirectory(third_party)           # HAL 驱动 + FreeRTOS  
add_subdirectory(bsp)           # 板级支持包 (依赖 third_party)
add_subdirectory(dal)           # 设备抽象层
add_subdirectory(middleware)    # 中间件
add_subdirectory(svc)           # 服务层 (依赖 middleware/dal)
add_subdirectory(app)           # 应用入口 (定义最终 ELF + hex/bin)
