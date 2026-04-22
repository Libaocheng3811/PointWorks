cmake_minimum_required(VERSION 3.28)

project(PointWorks)

if (POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif ()

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

# 启用 Qt 的自动处理功能
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif ()

# Debug 版本库后缀 "d" (例如 ct_cored.lib)
set(CMAKE_DEBUG_POSTFIX "d")

# 统一输出目录，无论 Debug 还是 Release，都输出到统一的 bin 和 lib 目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib) # .lib (static)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) # .dll
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) # .exe

foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${CMAKE_BINARY_DIR}/bin)
endforeach()

if (MSVC)
    # 设置 默认C++ 编译器标志
    set(CMAKE_CXX_FLAGS_DEFAULT "/DWIN32 /D_WINDOWS /W3 /GR /EHsc")

    # 若现有的 C++ 编译器标志与默认标志相同，则追加一些额外的标志
    if ("${CMAKE_BUILD_TYPE}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
        string(APPEND CMAKE_CXX_FLAGS " /fp:precise /bigobj /wd4996 /source-charset:utf-8 ")
        # 将 MSVC_MP 变量设置为 CPU 核心数，用于启用多处理器编译
        set(MSVC_MP ${CPUCores})

        if (CMAKE_VERSION VERSION_LESS 3.11.0)
            if (MSVC_MP EQUAL 0)
                string(APPEND CMAKE_CXX_FLAGS " /MP")
            elseif (MSVC_MP GREATER 1)
                string(APPEND CMAKE_CXX_FLAGS " /MP${MSVC_MP}")
            endif ()
        else ()
            if (MSVC_MP EQUAL 0)
                add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP>)
            elseif (MSVC_MP GREATER 1)
                add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP${MSVC_MP}>)
            endif ()
        endif ()
    endif ()
    # 对于GNU编译器
elseif (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEFAULT "")
    if (CMAKE_COMPILER_VERSION VERSION_GREATER 7)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi=11")
    else ()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
    endif ()
endif ()

option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)

if (WITH_OPENMP)
    find_package(OpenMP COMPONENTS C CXX)
endif ()

# 如果找到OpenMP，追加 OpenMP 编译器标志
if (OpenMP_FOUND)
    string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
    string(APPEND CMAKE_CXX_FLAGS " ${OpenMP_CXX_FLAGS}")
    string(APPEND CMAKE_EXE_LINKER_FLAGS " ${OpenMP_EXE_LINKER_FLAGS}")

    if (${CMAKE_VERSION} VERSION_LESS 3.7)
        message(STATUS "Found OpenMP")
    else ()
        message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")
    endif ()
else ()
    message(STATUS "Not found OpenMP")
endif ()

# 标准python环境，为避免多个python版本冲突，指定路径
set(MY_NATIVE_PYTHON_DIR "F:/Program Files/Python39")
set(Python3_ROOT_DIR ${MY_NATIVE_PYTHON_DIR})
set(Python3_FIND_STRATEGY LOCATION)
set(Python3_FIND_REGISTRY NEVER)

find_package(Python3 3.9 EXACT COMPONENTS Interpreter Development.Embed REQUIRED)

message(STATUS "Python Include: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python Libs: ${Python3_LIBRARIES}")

link_directories(${Python3_LIBRARY_DIRS})
set(PYBIND11_FINDPYTHON ON CACHE BOOL "" FORCE)

# QT路径
#set(CMAKE_PREFIX_PATH "D:/LibCommunity/Qt5.15/5.15.2/msvc2019_64")
find_package(QT NAMES Qt5 COMPONENTS Core Widgets OpenGL Gui Concurrent Charts REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets Concurrent Charts REQUIRED)
find_package(Qt5LinguistTools REQUIRED)

if (QT_FOUND)
    message(STATUS "Found QT version: ${Qt5_VERSION}.")
    message(STATUS "Qt5_DIR = ${Qt5_DIR}")
    message(STATUS "Qt5Widgets_DIR = ${Qt5Widgets_DIR}")
    message(STATUS "Qt5Core_DIR = ${Qt5Core_DIR}")
else ()
    message(STATUS "Qt5 is not found.")
endif ()

# vtk
find_package(VTK REQUIRED)
if (VTK_FOUND)
    message(STATUS "Found VTK version: ${VTK_VERSION}.")
else ()
    message(STATUS "VTK is not found.")
endif ()

# pcl
find_package(PCL REQUIRED)
if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    message(STATUS "Found PCL version: ${PCL_VERSION}.")
else ()
    message(STATUS "PCL is not found.")
endif ()

find_package(OpenMP REQUIRED)

message(STATUS "Build Type: " ${CMAKE_BUILD_TYPE})

# add_definitions 命令用于向编译器添加编译选项或定义宏
file(TO_CMAKE_PATH "${PROJECT_SOURCE_DIR}" PROJECT_ROOT_PATH)
add_definitions(-DROOT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(-DDATA_PATH="${PROJECT_SOURCE_DIR}/data/")
add_definitions(-DPYTHON_HOME="${MY_NATIVE_PYTHON_DIR}")

# 3rdParty 编译选项
set(LASZIP_BUILD_STATIC ON CACHE BOOL "Build static laszip" FORCE)
set(LASLIB_BUILD_STATIC ON CACHE BOOL "Build static laslib" FORCE)
set(BUILD_TOOLS OFF CACHE BOOL "Build tools" FORCE)

# Xerces-C + libE57：使用预编译静态库
set(LIBE57_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libE57")

# xerces-c 静态库
add_library(xerces-c STATIC IMPORTED)
set_target_properties(xerces-c PROPERTIES
    IMPORTED_LOCATION "${LIBE57_DIR}/lib/xerces-c_3.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${LIBE57_DIR}/include"
)
# xerces-c 依赖的 Windows 系统库
target_link_libraries(xerces-c INTERFACE ws2_32 advapi32)

# E57Format 静态库
add_library(E57Format STATIC IMPORTED)
set_target_properties(E57Format PROPERTIES
    IMPORTED_LOCATION "${LIBE57_DIR}/lib/E57Format.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${LIBE57_DIR}/include"
    INTERFACE_COMPILE_DEFINITIONS "E57FORMAT_STATIC_DEFINE"
)
target_link_libraries(E57Format INTERFACE xerces-c)

add_subdirectory(3rdparty/pybind11)
add_subdirectory(3rdparty/LASTools)
add_subdirectory(3rdparty/CSF)

# libs/ 作为跨库 include 根路径 (e.g. #include "core/cloud.h")
include_directories(${PROJECT_SOURCE_DIR}/libs)

# Libraries
add_subdirectory(libs/core)
add_subdirectory(libs/viz)
add_subdirectory(libs/io)
add_subdirectory(libs/algorithm)
add_subdirectory(libs/ui)
add_subdirectory(libs/python)

# Application
add_subdirectory(src)

# Testing
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
