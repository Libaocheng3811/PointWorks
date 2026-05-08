file(GLOB IoSrcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB IoHdrs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(pw_io SHARED ${IoSrcs} ${IoHdrs})

target_include_directories(pw_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pw_io
    PUBLIC
        pw_core
        Qt${QT_VERSION_MAJOR}::Core
    PRIVATE
        LASlib
        E57Format
)

target_compile_definitions(pw_io PRIVATE PW_BUILDING_PW_IO)
