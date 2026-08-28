
# 应用控制模块（模式状态机 + 服务装配，依赖倒置顶层）
add_library(app_ctrl STATIC
    src/app_ctrl.c
)

target_include_directories(app_ctrl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(app_ctrl
    PUBLIC
        svc_att_algo
        svc_mot_ctrl
        svc_monitor
        svc_comm
        bsp
    PRIVATE
        freertos_kernel
        dal
        mw_pid
)
