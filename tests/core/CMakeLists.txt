set(TEST_CORE_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/test_cloud.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_cloudtype.cpp
)

add_executable(test_core ${TEST_CORE_SRCS})

target_link_libraries(test_core
    PRIVATE
        ct_core
        test_common
        GTest::gtest
        GTest::gtest_main
)

target_include_directories(test_core PRIVATE
    ${CMAKE_SOURCE_DIR}/libs
)

gtest_discover_tests(test_core)
