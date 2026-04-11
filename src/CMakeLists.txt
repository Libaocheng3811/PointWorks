set(Srcs
    app/main.cpp
    app/mainwindow.cpp
    edit/color.cpp
    edit/boundingbox.cpp
    edit/transformation.cpp
    edit/normals.cpp
    edit/scale.cpp
    edit/coordinate.cpp
    tool/cutting.cpp
    tool/pickpoints.cpp
    tool/filters.cpp
    tool/segmentation.cpp
    tool/surface.cpp
    tool/boundary.cpp
    tool/rangeimage.cpp
    tool/align_by_centers.cpp
    tool/global_registration.cpp
    tool/fine_registration.cpp
    tool/point_pairs_alignment.cpp
    tool/segmentation/shape_detection_dialog.cpp
    tool/segmentation/morphological_filter_dialog.cpp
    tool/segmentation/region_growing_dialog.cpp
    tool/segmentation/clustering_dialog.cpp
    tool/segmentation/supervoxel_dialog.cpp
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
    edit/scale.h
    edit/coordinate.h
    tool/cutting.h
    tool/pickpoints.h
    tool/filters.h
    tool/segmentation.h
    tool/surface.h
    tool/boundary.h
    tool/rangeimage.h
    tool/align_by_centers.h
    tool/global_registration.h
    tool/fine_registration.h
    tool/point_pairs_alignment.h
    tool/segmentation/shape_detection_dialog.h
    tool/segmentation/morphological_filter_dialog.h
    tool/segmentation/region_growing_dialog.h
    tool/segmentation/clustering_dialog.h
    tool/segmentation/supervoxel_dialog.h
    tool/sampling.h
    python/python_console.h
    python/python_editor.h
    plugins/csfplugin.h
    plugins/vegplugin.h
    plugins/changedetectplugin.h
    )

set(UIS
    tool/filters.ui
    tool/segmentation.ui
    tool/surface.ui
    tool/boundary.ui
    edit/color.ui
    edit/boundingbox.ui
    edit/transformation.ui
    edit/normals.ui
    edit/scale.ui
    edit/coordinate.ui
)

set(QRCs resources/res.qrc)
set(IRCs resources/logo/logo.rc)

find_package(VTK COMPONENTS
    vtkGUISupportQt
    vtkGUISupportQtOpenGL
    vtkRenderingGL2PSOpenGL2
    vtkRenderingVolumeOpenGL2)

add_executable(pointworks WIN32 ${Srcs} ${Hdrs} ${UIS} ${QRCs} ${IRCs})

target_include_directories(pointworks PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pointworks PRIVATE
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
