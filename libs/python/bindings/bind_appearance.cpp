#include "bind_appearance.h"
#include "python_bridge.h"

void registerAppearanceBindings(py::module_& m)
{
    // 点云外观
    m.def("set_point_size", [](const std::string& id, float size) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setPointSize(QString::fromStdString(id), size);
    }, py::arg("id"), py::arg("size"), "Set point size for a cloud");

    m.def("set_opacity", [](const std::string& id, float value) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setOpacity(QString::fromStdString(id), value);
    }, py::arg("id"), py::arg("value"), "Set cloud opacity (0.0 - 1.0)");

    m.def("set_cloud_color", [](const std::string& id, float r, float g, float b) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setCloudColorRGB(QString::fromStdString(id), r, g, b);
    }, py::arg("id"), py::arg("r"), py::arg("g"), py::arg("b"),
       "Set cloud color by RGB (0.0 - 1.0)");

    m.def("set_color_by_axis", [](const std::string& id, const std::string& axis) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setCloudColorByAxis(QString::fromStdString(id), QString::fromStdString(axis));
    }, py::arg("id"), py::arg("axis"), "Color cloud by axis (X/Y/Z)");

    m.def("reset_cloud_color", [](const std::string& id) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->resetCloudColor(QString::fromStdString(id));
    }, py::arg("id"), "Reset cloud to original colors");

    m.def("set_cloud_visibility", [](const std::string& id, bool visible) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setCloudVisibility(QString::fromStdString(id), visible);
    }, py::arg("id"), py::arg("visible"), "Show or hide a cloud");

    // 场景外观
    m.def("set_background_color", [](float r, float g, float b) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setBackgroundColor(r, g, b);
    }, py::arg("r"), py::arg("g"), py::arg("b"),
       "Set background color (0.0 - 1.0)");

    m.def("reset_background_color", []() {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->resetBackgroundColor();
    }, "Reset background to default color");

    // 显示开关
    m.def("show_id", [](bool show) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->showId(show);
    }, py::arg("show"), "Show or hide cloud IDs");

    m.def("show_axes", [](bool show) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->showAxes(show);
    }, py::arg("show"), "Show or hide coordinate axes");

    m.def("show_fps", [](bool show) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->showFPS(show);
    }, py::arg("show"), "Show or hide FPS counter");

    m.def("show_info", [](const std::string& text) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->showInfo(QString::fromStdString(text));
    }, py::arg("text"), "Show info text overlay");

    m.def("clear_info", []() {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->clearInfo();
    }, "Clear all info text overlays");
}
