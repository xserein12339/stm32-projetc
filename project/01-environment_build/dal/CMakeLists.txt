
# dal静态库
add_library(dal STATIC
    src/dal_core.c
)

target_include_directories(dal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
