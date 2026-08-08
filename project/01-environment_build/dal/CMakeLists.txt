
# dal静态库
add_library(dal STATIC
    src/dal_registry.c
    src/dal_key.c
    src/dal_led.c
)

target_include_directories(dal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
