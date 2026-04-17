# MSVC: 确保 GTest 使用与项目一致的 CRT（/MD）
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)

# 使用本地 3rdparty 目录中的 googletest
add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/googletest ${CMAKE_BINARY_DIR}/3rdparty/googletest)

message(STATUS "Google Test ready (local)")

# 通过 CTest 注册所有测试（必须在 add_subdirectory 之前加载）
include(GoogleTest)

# 测试公共工具库
add_subdirectory(common)

# 各模块测试
add_subdirectory(core)
add_subdirectory(algorithm)
add_subdirectory(io)
