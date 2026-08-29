# 日志模块（无外部依赖；输出通道由 app 注入）
add_library(mw_log STATIC
    src/mw_log.c
)

target_include_directories(mw_log PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# 添加依赖，无依赖
