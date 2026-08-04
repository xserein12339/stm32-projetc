
# 生成dal_display库
add_library(dal_display STATIC      
    src/dal_display.c
)

target_include_directories(dal_display PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
)


# 添加依赖，无依赖