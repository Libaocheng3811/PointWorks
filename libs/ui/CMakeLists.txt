# Dialog components
file(GLOB Dialog_SOURCES CONFIGURE_DEPENDS "dialog/*.cpp")
file(GLOB Dialog_HEADERS CONFIGURE_DEPENDS "dialog/*.h")
file(GLOB Dialog_FORMS CONFIGURE_DEPENDS "dialog/*.ui")

add_library(pw_ui_dialog STATIC ${Dialog_SOURCES} ${Dialog_HEADERS} ${Dialog_FORMS})

target_include_directories(pw_ui_dialog PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pw_ui_dialog PUBLIC
    pw_core
    Qt${QT_VERSION_MAJOR}::Widgets
)

# Base widget classes + cloudtree
set(Base_Srcs
    base/customtree.cpp
    base/cloudtree.cpp
    base/cloud_io_controller.cpp
    base/cloudregistry.cpp
    base/dialog_registry.cpp
    base/language_manager.cpp
    base/progress_manager.cpp
    base/viewport_manager.cpp
    widgets/sf_histogram_chart.cpp
    widgets/sf_display_panel.cpp
)

set(Base_Hdrs
    base/customtree.h
    base/cloudtree.h
    base/cloud_io_controller.h
    base/cloudregistry.h
    base/customdock.h
    base/customdialog.h
    base/dialog_registry.h
    base/language_manager.h
    base/progress_manager.h
    base/paramsnapshot.h
    base/viewport_manager.h
    widgets/sf_histogram_chart.h
    widgets/sf_display_panel.h
)

add_library(pw_ui_base STATIC ${Base_Srcs} ${Base_Hdrs})

target_include_directories(pw_ui_base PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pw_ui_base
    PUBLIC
        pw_ui_dialog
        pw_core
        pw_viz
    PRIVATE
        pw_io
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Charts
        Qt${QT_VERSION_MAJOR}::Concurrent
)
