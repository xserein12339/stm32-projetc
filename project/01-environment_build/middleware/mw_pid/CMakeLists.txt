
# 生成pid库
add_library(mw_pid STATIC      
    src/mw_pid.c
)

target_include_directories(mw_pid PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
)

# 添加依赖，无依赖