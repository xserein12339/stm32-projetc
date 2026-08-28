
# 生成控制服务库
add_library(svc_att_algo STATIC      
    src/svc_att_algo.c
)

target_include_directories(svc_att_algo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
)

# 添加依赖
target_link_libraries(svc_att_algo 
    PRIVATE
        freertos_kernel
        mw_common
        mw_filter
        dal
)