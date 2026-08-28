
# 生成filter库
add_library(mw_filter STATIC      
    src/mw_filter.c
)

target_include_directories(mw_filter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc    
)

# 添加 MW_COMMON 依赖
target_link_libraries(mw_filter
    PUBLIC
        mw_common
)