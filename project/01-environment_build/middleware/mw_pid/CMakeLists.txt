
# 生成pid库
add_library(mw_pid STATIC      
    src/mw_pid.c
)

target_include_directories(mw_pid PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
)

# 添加 MW_COMMON 依赖
target_link_libraries(mw_pid
    PUBLIC
        mw_common
)