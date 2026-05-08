set(TEST_IO_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/test_fileio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_formats.cpp
)

# IO 测试需要自定义 main（创建 QCoreApplication），不使用 gtest_main
add_executable(test_io ${TEST_IO_SRCS})

target_link_libraries(test_io
    PRIVATE
        pw_io
        pw_core
        pw_algorithm
        test_common
        GTest::gtest
        Qt5::Core
        Qt5::Widgets
    PRIVATE
        ${PCL_LIBRARIES}
)

target_include_directories(test_io PRIVATE
    ${CMAKE_SOURCE_DIR}/libs
)

gtest_discover_tests(test_io)
