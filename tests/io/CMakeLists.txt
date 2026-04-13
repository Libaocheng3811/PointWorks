set(TEST_IO_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/test_fileio.cpp
)

add_executable(test_io ${TEST_IO_SRCS})

target_link_libraries(test_io
    PRIVATE
        ct_io
        ct_core
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
