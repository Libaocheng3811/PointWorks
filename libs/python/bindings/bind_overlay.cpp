#include "bind_overlay.h"
#include "python_bridge.h"

void registerOverlayBindings(py::module_& m)
{
    // 基础叠加物
    m.def("add_cube", [](float cx, float cy, float cz, float size, const std::string& id) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->addCube(cx, cy, cz, size, QString::fromStdString(id));
    }, py::arg("cx"), py::arg("cy"), py::arg("cz"),
       py::arg("size"), py::arg("id") = "cube",
       "Add a cube overlay at center (cx,cy,cz) with given size");

    m.def("add_3d_label", [](const std::string& text, float x, float y, float z, const std::string& id) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->add3DLabel(QString::fromStdString(text), x, y, z, QString::fromStdString(id));
    }, py::arg("text"), py::arg("x"), py::arg("y"), py::arg("z"),
       py::arg("id") = "label",
       "Add a 3D text label at position (x,y,z)");

    m.def("remove_shape", [](const std::string& id) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->removeShape(QString::fromStdString(id));
    }, py::arg("id"), "Remove a shape/overlay by ID");

    m.def("remove_all_shapes", []() {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->removeAllShapes();
    }, "Remove all shapes/overlays");

    // 高级叠加物
    m.def("add_arrow", [](float x1, float y1, float z1, float x2, float y2, float z2,
                           float r, float g, float b, const std::string& id) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->addArrow(x1, y1, z1, x2, y2, z2, QString::fromStdString(id), r, g, b);
    }, py::arg("x1"), py::arg("y1"), py::arg("z1"),
       py::arg("x2"), py::arg("y2"), py::arg("z2"),
       py::arg("r") = 1.0, py::arg("g") = 1.0, py::arg("b") = 1.0,
       py::arg("id") = "arrow",
       "Add arrow between two 3D points. Color r/g/b in 0.0-1.0");

    m.def("add_polygon", [](const std::string& cloud_id, float r, float g, float b,
                             const std::string& id) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->addPolygonCloud(QString::fromStdString(cloud_id), QString::fromStdString(id), r, g, b);
    }, py::arg("cloud_id"), py::arg("r") = 1.0, py::arg("g") = 1.0, py::arg("b") = 1.0,
       py::arg("id") = "polygon",
       "Add polygon (convex hull) for a cloud. Color r/g/b in 0.0-1.0");

    m.def("set_shape_color", [](const std::string& id, float r, float g, float b) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeColor(QString::fromStdString(id), r, g, b);
    }, py::arg("id"), py::arg("r") = 1.0, py::arg("g") = 1.0, py::arg("b") = 1.0,
       "Set shape color. r/g/b in 0.0-1.0");

    m.def("set_shape_size", [](const std::string& id, float size) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeSize(QString::fromStdString(id), size);
    }, py::arg("id"), py::arg("size"), "Set shape point size");

    m.def("set_shape_opacity", [](const std::string& id, float value) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeOpacity(QString::fromStdString(id), value);
    }, py::arg("id"), py::arg("value"), "Set shape opacity (0.0-1.0)");

    m.def("set_shape_line_width", [](const std::string& id, float value) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeLineWidth(QString::fromStdString(id), value);
    }, py::arg("id"), py::arg("value"), "Set shape line width");

    m.def("set_shape_font_size", [](const std::string& id, float value) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeFontSize(QString::fromStdString(id), value);
    }, py::arg("id"), py::arg("value"), "Set shape font size");

    m.def("set_shape_representation", [](const std::string& id, int type) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeRepresentation(QString::fromStdString(id), type);
    }, py::arg("id"), py::arg("type"),
       "Set shape representation: 0=points, 1=wireframe, 2=surface");

    m.def("zoom_to_bounds_xyz", [](float min_x, float min_y, float min_z,
                                    float max_x, float max_y, float max_z) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->zoomToBoundsXYZ(min_x, min_y, min_z, max_x, max_y, max_z);
    }, py::arg("min_x"), py::arg("min_y"), py::arg("min_z"),
       py::arg("max_x"), py::arg("max_y"), py::arg("max_z"),
       "Zoom camera to specific bounding box");

    m.def("invalidate_cloud_render", [](const std::string& id) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->invalidateCloudRender(QString::fromStdString(id));
    }, py::arg("id"), "Force re-render a specific cloud");

    m.def("set_interactor_enable", [](bool enable) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->setInteractorEnable(enable);
    }, py::arg("enable"), "Enable or disable mouse interaction");
}
