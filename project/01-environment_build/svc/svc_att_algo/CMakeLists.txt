# 生成姿态解算服务库
add_library(svc_att_algo STATIC
    src/svc_att_algo.c
)

target_include_directories(svc_att_algo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc    # svc_err.h 服务层公共错误码
)

# 添加依赖
# mw_common 为 PUBLIC：公开头文件的 q15_t 类型与配置项需要其头文件路径
target_link_libraries(svc_att_algo
    PUBLIC
        mw_common
    PRIVATE
        freertos_kernel
        mw_filter
        dal
)
