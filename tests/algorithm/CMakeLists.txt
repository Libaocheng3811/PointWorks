file(GLOB TEST_ALGO_SRCS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(test_algorithm ${TEST_ALGO_SRCS})

target_link_libraries(test_algorithm
    PRIVATE
        ct_algorithm
        ct_core
        test_common
        GTest::gtest
        GTest::gtest_main
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
)

target_include_directories(test_algorithm PRIVATE
    ${CMAKE_SOURCE_DIR}/libs
)

gtest_discover_tests(test_algorithm)
