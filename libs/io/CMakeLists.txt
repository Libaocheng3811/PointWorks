file(GLOB IoSrcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB IoHdrs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(ct_io SHARED ${IoSrcs} ${IoHdrs})

target_include_directories(ct_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_io
    PUBLIC
        ct_core
        Qt${QT_VERSION_MAJOR}::Widgets
    PRIVATE
        LASlib
        E57Format
)

target_compile_definitions(ct_io PRIVATE CT_BUILDING_CT_IO)
