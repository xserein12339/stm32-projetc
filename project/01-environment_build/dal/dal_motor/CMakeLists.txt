
# 生成dal_motor库
add_library(dal_motor STATIC      
    src/dal_motor.c
)

target_include_directories(dal_motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
)

# 添加依赖，无依赖