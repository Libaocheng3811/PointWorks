# Dialog components
file(GLOB Dialog_SOURCES CONFIGURE_DEPENDS "dialog/*.cpp")
file(GLOB Dialog_HEADERS CONFIGURE_DEPENDS "dialog/*.h")
file(GLOB Dialog_FORMS CONFIGURE_DEPENDS "dialog/*.ui")

add_library(ct_ui_dialog STATIC ${Dialog_SOURCES} ${Dialog_HEADERS} ${Dialog_FORMS})

target_include_directories(ct_ui_dialog PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_ui_dialog PUBLIC
    ct_core
    Qt${QT_VERSION_MAJOR}::Widgets
)

# Base widget classes + cloudtree
set(Base_Srcs
    base/customtree.cpp
    base/cloudtree.cpp
    widgets/sf_histogram_chart.cpp
    widgets/sf_display_panel.cpp
)

set(Base_Hdrs
    base/customtree.h
    base/cloudtree.h
    base/customdock.h
    base/customdialog.h
    base/paramsnapshot.h
    widgets/sf_histogram_chart.h
    widgets/sf_display_panel.h
)

add_library(ct_ui_base STATIC ${Base_Srcs} ${Base_Hdrs})

target_include_directories(ct_ui_base PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_ui_base
    PUBLIC
        ct_ui_dialog
        ct_core
        ct_viz
        ct_io
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Charts
)
