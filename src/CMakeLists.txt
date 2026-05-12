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
    pw_core
    pw_algorithm
    pw_ui_base
    pw_python
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Concurrent
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    pybind11::embed
    Python3::Python
)

# Post-build: Deploy third-party DLLs for running from build directory
if(WIN32 AND NOT CMAKE_CROSSCOMPILING)

    # --- Qt: locate windeployqt from Qt5::Core DLL path ---
    get_target_property(_Qt5_DLL Qt5::Core IMPORTED_LOCATION_RELEASE)
    if(NOT _Qt5_DLL)
        get_target_property(_Qt5_DLL Qt5::Core IMPORTED_LOCATION_DEBUG)
    endif()
    if(_Qt5_DLL)
        get_filename_component(_Qt5_BIN "${_Qt5_DLL}" DIRECTORY)
        find_program(WINDEPLOYQT_EXE windeployqt HINTS "${_Qt5_BIN}" NO_DEFAULT_PATH)
    endif()
    if(WINDEPLOYQT_EXE)
        add_custom_command(TARGET pointworks POST_BUILD
            COMMAND "${WINDEPLOYQT_EXE}"
                --$<LOWER_CASE:$<CONFIG>>
                --no-translations --no-opengl-sw
                "$<TARGET_FILE:pointworks>"
            COMMENT "Deploying Qt DLLs..."
        )
    else()
        message(WARNING "windeployqt not found - Qt DLLs won't be auto-deployed")
    endif()

    # --- PCL: derive root from PCL_DIR ---
    get_filename_component(_PCL_ROOT "${PCL_DIR}" DIRECTORY)
    if(NOT EXISTS "${_PCL_ROOT}/include/pcl-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
        set(_PCL_ROOT "${PCL_DIR}")
    endif()

    # --- VTK: derive bin dir ---
    # Method 1: from imported target (VTK 9.x uses VTK:: prefix)
    set(_VTK_BIN "")
    if(TARGET VTK::CommonCore)
        get_target_property(_vtk_lib VTK::CommonCore IMPORTED_LOCATION_RELEASE)
        if(NOT _vtk_lib)
            get_target_property(_vtk_lib VTK::CommonCore IMPORTED_LOCATION_DEBUG)
        endif()
    elseif(TARGET vtkCommonCore)
        get_target_property(_vtk_lib vtkCommonCore IMPORTED_LOCATION_RELEASE)
        if(NOT _vtk_lib)
            get_target_property(_vtk_lib vtkCommonCore IMPORTED_LOCATION_DEBUG)
        endif()
    endif()
    if(_vtk_lib)
        get_filename_component(_vtk_lib_dir "${_vtk_lib}" DIRECTORY)
        get_filename_component(_vtk_root "${_vtk_lib_dir}" DIRECTORY)
        if(EXISTS "${_vtk_root}/bin")
            set(_VTK_BIN "${_vtk_root}/bin")
        endif()
    endif()
    # Method 2: PCL All-in-One standard layout
    if(NOT _VTK_BIN AND EXISTS "${_PCL_ROOT}/3rdParty/VTK/bin")
        set(_VTK_BIN "${_PCL_ROOT}/3rdParty/VTK/bin")
    endif()

    # --- Python: exact DLL paths from find_package(Python3) ---
    set(_PYTHON_DLL_RELEASE "${Python3_RUNTIME_LIBRARY_RELEASE}")
    set(_PYTHON_DLL_DEBUG "${Python3_RUNTIME_LIBRARY_DEBUG}")
    set(_PYTHON_STABLE_DLL "")
    if(_PYTHON_DLL_RELEASE)
        get_filename_component(_PYTHON_BIN "${_PYTHON_DLL_RELEASE}" DIRECTORY)
        if(EXISTS "${_PYTHON_BIN}/python3.dll")
            set(_PYTHON_STABLE_DLL "${_PYTHON_BIN}/python3.dll")
        endif()
    endif()

    # --- Collect DLL source directories ---
    set(_DLL_DIRS "")
    if(EXISTS "${_PCL_ROOT}/bin")
        list(APPEND _DLL_DIRS "${_PCL_ROOT}/bin")
    endif()
    if(_VTK_BIN)
        list(APPEND _DLL_DIRS "${_VTK_BIN}")
    endif()
    foreach(_lib FLANN Qhull Boost)
        if(EXISTS "${_PCL_ROOT}/3rdParty/${_lib}/bin")
            list(APPEND _DLL_DIRS "${_PCL_ROOT}/3rdParty/${_lib}/bin")
        endif()
    endforeach()

    string(REPLACE ";" "|" _DLL_DIRS_STR "${_DLL_DIRS}")

    # --- Write config file (avoids cmd.exe quoting issues with spaces in paths) ---
    file(WRITE "${CMAKE_BINARY_DIR}/deploy_config.cmake"
        "set(DLL_DIRS \"${_DLL_DIRS_STR}\")\n"
        "set(PYTHON_DLL_RELEASE \"${_PYTHON_DLL_RELEASE}\")\n"
        "set(PYTHON_DLL_DEBUG \"${_PYTHON_DLL_DEBUG}\")\n"
        "set(PYTHON_STABLE_DLL \"${_PYTHON_STABLE_DLL}\")\n"
    )

    add_custom_command(TARGET pointworks POST_BUILD
        COMMAND ${CMAKE_COMMAND}
            -DOUT_DIR="$<TARGET_FILE_DIR:pointworks>"
            -DCONFIG="$<CONFIG>"
            -DCONFIG_FILE="${CMAKE_BINARY_DIR}/deploy_config.cmake"
            -P "${CMAKE_SOURCE_DIR}/cmake/deploy_dlls.cmake"
        COMMENT "Deploying PCL/VTK/Python DLLs..."
    )

endif()
