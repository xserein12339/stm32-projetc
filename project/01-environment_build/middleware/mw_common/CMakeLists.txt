
# 生成pid库
add_library(mw_common STATIC      
    src/mw_common.c
)

target_include_directories(mw_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
)

# 添加依赖，无依赖