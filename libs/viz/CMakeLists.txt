file(GLOB VizSrcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB VizHdrs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

find_package(VTK COMPONENTS
    vtkGUISupportQt
    vtkGUISupportQtOpenGL
    vtkRenderingGL2PSOpenGL2
    vtkRenderingVolumeOpenGL2
    vtkIOImage)

add_library(pw_viz SHARED ${VizSrcs} ${VizHdrs})

target_include_directories(pw_viz PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pw_viz
    PUBLIC
        pw_core
        Qt${QT_VERSION_MAJOR}::Widgets
        ${VTK_LIBRARIES}
        ${PCL_LIBRARIES}
    PRIVATE
        pw_io
)

target_compile_definitions(pw_viz PRIVATE PW_BUILDING_PW_VIZ)
