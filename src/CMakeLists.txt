set(Srcs
    app/main.cpp
    app/mainwindow.cpp
    edit/color.cpp
    edit/boundingbox.cpp
    edit/transformation.cpp
    edit/normals.cpp
    tool/cutting.cpp
    tool/pickpoints.cpp
    tool/filters.cpp
    tool/keypoints.cpp
    tool/segmentation.cpp
    tool/surface.cpp
    tool/boundary.cpp
    tool/rangeimage.cpp
    tool/registration.cpp
    tool/descriptor.cpp
    tool/sampling.cpp
    python/python_console.cpp
    python/python_editor.cpp
    plugins/csfplugin.cpp
    plugins/vegplugin.cpp
    plugins/changedetectplugin.cpp
    app/projectmanager.cpp
    )

set(Hdrs
    app/mainwindow.h
    app/projectmanager.h
    app/recentprojects.h
    edit/color.h
    edit/boundingbox.h
    edit/transformation.h
    edit/normals.h
    tool/cutting.h
    tool/pickpoints.h
    tool/filters.h
    tool/keypoints.h
    tool/segmentation.h
    tool/surface.h
    tool/boundary.h
    tool/rangeimage.h
    tool/registration.h
    tool/descriptor.h
    tool/sampling.h
    python/python_console.h
    python/python_editor.h
    plugins/csfplugin.h
    plugins/vegplugin.h
    plugins/changedetectplugin.h
    )

set(UIS
    tool/registration.ui
    tool/filters.ui
    tool/segmentation.ui
    tool/surface.ui
    tool/boundary.ui
    tool/descriptor.ui
    edit/color.ui
    edit/boundingbox.ui
    edit/transformation.ui
    edit/normals.ui
)

set(QRCs resources/res.qrc)
set(IRCs resources/logo/logo.rc)

find_package(VTK COMPONENTS
    vtkGUISupportQt
    vtkGUISupportQtOpenGL
    vtkRenderingGL2PSOpenGL2
    vtkRenderingVolumeOpenGL2)

add_executable(cloudtool WIN32 ${Srcs} ${Hdrs} ${UIS} ${QRCs} ${IRCs})

target_include_directories(cloudtool PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(cloudtool PRIVATE
    ct_core
    ct_viz
    ct_io
    ct_algorithm
    ct_ui_base
    ct_python
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Concurrent
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    pybind11::embed
    Python3::Python
)
