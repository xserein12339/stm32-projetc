add_library(app_test STATIC
    src/app_test.c
)

target_include_directories(app_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)


target_link_libraries(app_test PUBLIC bsp)