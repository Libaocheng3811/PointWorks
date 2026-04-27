set(Srcs
    app/main.cpp
    app/mainwindow.cpp
    help/help_launcher.cpp
    app/python_connections.cpp
    edit/color.cpp
    options/displaysettings.cpp
    options/displaysettings_colors_page.cpp
    options/python_settings_page.cpp
    edit/boundingbox.cpp
    edit/transformation.cpp
    edit/normals.cpp
    edit/scale.cpp
    edit/coordinate.cpp
    tool/cutting.cpp
    tool/pickpoints.cpp
    tool/filters.cpp
    tool/rangeimage.cpp
    tool/align/align_by_centers.cpp
    tool/align/global_registration.cpp
    tool/align/fine_registration.cpp
    tool/align/point_pairs_alignment.cpp
    tool/segmentation/shape_detection_dialog.cpp
    tool/segmentation/morphological_filter_dialog.cpp
    tool/segmentation/region_growing_dialog.cpp
    tool/segmentation/clustering_dialog.cpp
    tool/segmentation/supervoxel_dialog.cpp
    tool/mesh/compute_hull_dialog.cpp
    tool/mesh/extract_boundary_dialog.cpp
    tool/mesh/reconstruct_surface_dialog.cpp
    tool/distance/cloud_cloud_dist_dialog.cpp
    tool/distance/cloud_mesh_dist_dialog.cpp
    tool/distance/cloud_primitive_dist_dialog.cpp
    tool/distance/closest_point_set_dialog.cpp
    tool/sampling.cpp
    tool/measure.cpp
    python/python_console.cpp
    python/python_editor.cpp
    plugins/csfplugin.cpp
    plugins/vegplugin.cpp
    plugins/changedetectplugin.cpp
    plugins/m3c2plugin.cpp
    app/projectmanager.cpp
    )

set(Hdrs
    app/mainwindow.h
    help/help_launcher.h
    app/python_connections.h
    app/projectmanager.h
    app/recentprojects.h
    edit/color.h
    options/displaysettings.h
    options/displaysettings_page.h
    options/displaysettings_colors_page.h
    options/python_settings_page.h
    edit/boundingbox.h
    edit/transformation.h
    edit/normals.h
    edit/scale.h
    edit/coordinate.h
    tool/cutting.h
    tool/pickpoints.h
    tool/filters.h
    tool/rangeimage.h
    tool/align/align_by_centers.h
    tool/align/global_registration.h
    tool/align/fine_registration.h
    tool/align/point_pairs_alignment.h
    tool/segmentation/shape_detection_dialog.h
    tool/segmentation/morphological_filter_dialog.h
    tool/segmentation/region_growing_dialog.h
    tool/segmentation/clustering_dialog.h
    tool/segmentation/supervoxel_dialog.h
    tool/mesh/compute_hull_dialog.h
    tool/mesh/extract_boundary_dialog.h
    tool/mesh/reconstruct_surface_dialog.h
    tool/distance/cloud_cloud_dist_dialog.h
    tool/distance/cloud_mesh_dist_dialog.h
    tool/distance/cloud_primitive_dist_dialog.h
    tool/distance/closest_point_set_dialog.h
    tool/sampling.h
    tool/measure.h
    python/python_console.h
    python/python_editor.h
    plugins/csfplugin.h
    plugins/vegplugin.h
    plugins/changedetectplugin.h
    plugins/m3c2plugin.h
    )

set(UIS
    tool/filters.ui
    tool/measure.ui
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

qt5_add_translation(QM_FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/trans/zh_CN.ts)

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
