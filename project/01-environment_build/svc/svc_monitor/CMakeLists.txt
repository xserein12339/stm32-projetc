
# 系统监控服务库（叶子服务：仅依赖 FreeRTOS，不依赖其他 svc/dal）
add_library(svc_monitor STATIC
    src/svc_monitor.c
)

# 公共错误码头文件在 svc/inc，需对外可见
target_include_directories(svc_monitor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc
)

target_link_libraries(svc_monitor
    PUBLIC
        freertos_kernel
)
