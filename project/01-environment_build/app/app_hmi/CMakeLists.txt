# 人机界面模块（OLED 文本渲染，业务数据格式化在 app_ctrl）
add_library(app_hmi STATIC
    src/app_hmi.c
)

target_include_directories(app_hmi PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(app_hmi PRIVATE
    dal
)
