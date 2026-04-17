file(GLOB VizSrcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB VizHdrs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

find_package(VTK COMPONENTS
    vtkGUISupportQt
    vtkGUISupportQtOpenGL
    vtkRenderingGL2PSOpenGL2
    vtkRenderingVolumeOpenGL2
    vtkIOImage)

add_library(ct_viz SHARED ${VizSrcs} ${VizHdrs})

target_include_directories(ct_viz PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_viz
    PUBLIC
        ct_core
        ct_io
        Qt${QT_VERSION_MAJOR}::Widgets
        ${VTK_LIBRARIES}
        ${PCL_LIBRARIES}
)

target_compile_definitions(ct_viz PRIVATE CT_BUILDING_CT_VIZ)
