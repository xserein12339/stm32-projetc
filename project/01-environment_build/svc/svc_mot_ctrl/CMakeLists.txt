
# 运动控制服务库（依赖姿态服务快照与 mw_pid）
add_library(svc_mot_ctrl STATIC
    src/svc_mot_ctrl.c
)

# 公共错误码头文件在 svc/inc，需对外可见
target_include_directories(svc_mot_ctrl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc
)

# svc_err.h 为 PUBLIC 头，mw_common 提供的 q15 数学需传递给上层
target_link_libraries(svc_mot_ctrl
    PUBLIC
        svc_att_algo
        mw_pid
    PRIVATE
        freertos_kernel
        mw_common
        dal
)
