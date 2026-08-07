
# 生成控制服务库
add_library(svc_mot_ctrl STATIC      
    src/svc_mot_ctrl.c
)

target_include_directories(svc_mot_ctrl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
)

# 添加依赖
target_link_libraries(svc_mot_ctrl 
    PRIVATE
        freertos_kernel

)