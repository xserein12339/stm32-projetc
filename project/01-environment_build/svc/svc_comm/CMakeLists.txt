
# 通信服务库（依赖 dal_wifi RAW_SERIAL 通道）
add_library(svc_comm STATIC
    src/svc_comm.c
)

# 公共错误码头文件在 svc/inc，需对外可见
target_include_directories(svc_comm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc
)

target_link_libraries(svc_comm
    PUBLIC
        freertos_kernel
    PRIVATE
        dal
)
