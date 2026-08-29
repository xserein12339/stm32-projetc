# =============================================================================
# 宿主机（PC）单元测试 - middleware 模块
#
# 独立于交叉编译工程：使用系统 gcc，直接编译 middleware 源文件。
# 运行：
#   cmake -S tests -B tests/build
#   cmake --build tests/build
#   ./tests/build/mw_tests
# =============================================================================
cmake_minimum_required(VERSION 3.20)

project(mw_tests LANGUAGES C)

# 与固件构建一致的最小告警集
add_compile_options(-Wall -Wextra -fno-common)

# 测试 CMake 不读交叉工具链（防止缓存穿透）
unset(CMAKE_TOOLCHAIN_FILE CACHE)

# middleware 头文件路径
set(MW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../middleware)

add_executable(mw_tests
    test_main.c
    test_q15_math.c
    test_mw_math.c
    test_mw_pid.c
    test_mw_filter.c
    test_mw_log.c
    # 被测源码（直接编入，不依赖固件工程的目标库）
    ${MW_DIR}/mw_common/src/mw_common.c
    ${MW_DIR}/mw_pid/src/mw_pid.c
    ${MW_DIR}/mw_filter/src/mw_filter.c
    ${MW_DIR}/mw_log/src/mw_log.c
)

target_include_directories(mw_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MW_DIR}/mw_common/inc
    ${MW_DIR}/mw_pid/inc
    ${MW_DIR}/mw_filter/inc
    ${MW_DIR}/mw_log/inc
)

# test_mw_math 使用浮点 atan2/sin/cos 作为基准
target_link_libraries(mw_tests PRIVATE m)

# 消除 -Werror 潜在干扰（测试中 main 返回值即用例结果，无需拦截）
enable_testing()
add_test(NAME mw_tests COMMAND mw_tests)
