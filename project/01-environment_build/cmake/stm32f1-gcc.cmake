# 适用于 STM32F1xx
# 设置目标平台与架构
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 交叉编译工具链查找
set(TOOLCHAIN_PREFIX arm-none-eabi-)    # 前缀
find_program(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc     HINTS ${TOOLCHAIN_PATH} REQUIRED)    #编译器
find_program(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc     HINTS ${TOOLCHAIN_PATH} REQUIRED)    #汇编器
find_program(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy HINTS ${TOOLCHAIN_PATH} REQUIRED)    #生成.bin/hex
find_program(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size    HINTS ${TOOLCHAIN_PATH} REQUIRED)    #打印内存占用
find_program(CMAKE_GDB          ${TOOLCHAIN_PREFIX}gdb     HINTS ${TOOLCHAIN_PATH})             #调试

# 只编译不链接
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 搜索路径策略，防止主机环境污染
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# MCU 标志
set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft")

set(CMAKE_C_FLAGS_INIT     "${MCU_FLAGS} -Wall -Wextra -Wshadow -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS_INIT   "${MCU_FLAGS} -Wall -Wextra -Wshadow -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS_INIT   "${MCU_FLAGS} -x assembler-with-cpp")

# 构建类型与优化级别
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_FLAGS_DEBUG          "-Og -g3 -gdwarf-4"      CACHE INTERNAL "")
set(CMAKE_C_FLAGS_RELEASE        "-O2 -DNDEBUG"           CACHE INTERNAL "")
set(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG"           CACHE INTERNAL "")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3 -DNDEBUG"       CACHE INTERNAL "")
