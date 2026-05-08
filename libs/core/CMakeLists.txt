file(GLOB LibSrcs  CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB LibHdrs CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

# LASlib头文件路径
set(LAS_INCLUDE_DIRS
    "${CMAKE_SOURCE_DIR}/3rdparty/LAStools/LASlib/inc"
    "${CMAKE_SOURCE_DIR}/3rdparty/LAStools/LASzip/src"
)

add_library(pw_core SHARED ${LibSrcs} ${LibHdrs})

target_include_directories(pw_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(pw_core PUBLIC ${LAS_INCLUDE_DIRS})

target_link_libraries(pw_core
        PUBLIC
        ${PCL_LIBRARIES}
        PRIVATE
        LASlib
        OpenMP::OpenMP_CXX)

target_compile_definitions(pw_core PRIVATE PW_BUILDING_PW_CORE)

set_target_properties(pw_core PROPERTIES PUBLIC_HEADER "${LibHdrs}")
