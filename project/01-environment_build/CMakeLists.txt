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

# ⭐ 全局编译选项 (所有子模块自动继承)
add_compile_options(
    -ffunction-sections
    -fdata-sections
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
# 模块添加 (严格按依赖顺序，自底向上)
# =============================================================================
add_subdirectory(third_party)   # CMSIS + HAL (无外部依赖)
add_subdirectory(bsp)           # 板级支持包 (依赖 third_party)
# add_subdirectory(dal)           # 数据访问层 (依赖 bsp)
# add_subdirectory(middleware)    # 中间件 (依赖 dal/bsp)
# add_subdirectory(svc)           # 服务层 (依赖 middleware/dal)
add_subdirectory(app)           # 应用入口 (定义最终 ELF + hex/bin)

# =============================================================================
# 注意: 不要在此处再定义 add_executable!
# 最终固件目标 ${PROJECT_NAME}.elf 已在 app/CMakeLists.txt 中完整定义
# 包括: main.c、链接脚本、nano.specs、hex/bin 生成
# =============================================================================