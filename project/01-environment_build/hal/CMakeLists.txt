
# hal抽象接口库
add_library(hal INTERFACE)

target_include_directories(hal INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)