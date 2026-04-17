#include "bind_view.h"

void registerViewBindings(py::module_& m)
{
    m.def("refresh_view", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->refreshView();
    }, "Refresh the 3D view");

    m.def("reset_camera", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->resetCamera();
    }, "Reset camera to default position");

    m.def("zoom_to_bounds", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->zoomToBounds();
    }, "Zoom to fit all visible clouds");

    m.def("set_auto_render", [](bool enable) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setAutoRender(enable);
    }, py::arg("enable"), "Enable or disable auto rendering");

    m.def("zoom_to_selected", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->zoomToSelected();
    }, "Zoom to selected clouds");

    m.def("set_top_view", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setTopView();
    }, "Set camera to top view");

    m.def("set_front_view", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setFrontView();
    }, "Set camera to front view");

    m.def("set_back_view", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setBackView();
    }, "Set camera to back view");

    m.def("set_left_view", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setLeftSideView();
    }, "Set camera to left side view");

    m.def("set_right_view", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setRightSideView();
    }, "Set camera to right side view");

    m.def("set_bottom_view", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setBottomView();
    }, "Set camera to bottom view");
}
