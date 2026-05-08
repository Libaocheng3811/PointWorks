# test_common — 测试公共工具库（静态库）
set(TEST_COMMON_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/test_helpers.cpp
)

add_library(test_common STATIC ${TEST_COMMON_SRCS})

target_include_directories(test_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/libs
)

target_link_libraries(test_common
    PUBLIC
        pw_core
    PRIVATE
        GTest::gtest
)
