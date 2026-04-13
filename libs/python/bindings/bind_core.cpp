#include "bind_core.h"

void registerCoreBindings(py::module_& m)
{
    // --- GUI Console 输出函数 ---
    m.def("printI", [](const std::string& msg) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->log(0, QString::fromStdString(msg));
    }, "Print info message to GUI Console");

    m.def("printW", [](const std::string& msg) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->log(1, QString::fromStdString(msg));
    }, "Print warning message to GUI Console");

    m.def("printE", [](const std::string& msg) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->log(2, QString::fromStdString(msg));
    }, "Print error message to GUI Console");

    // --- 工厂函数：从 capsule 中提取 Cloud::Ptr 并构造 PyCloud ---
    m.def("_wrap_cloud", [](py::capsule cap) -> py::object {
        auto* cloud_ptr = static_cast<ct::Cloud::Ptr*>(cap);
        if (!cloud_ptr || !*cloud_ptr)
            throw std::runtime_error("Invalid cloud capsule");
        return py::cast(PyCloud(*cloud_ptr));
    }, py::arg("cap"), "Internal: wrap Cloud::Ptr into ct.Cloud");

    // --- 按名称获取点云 ---
    m.def("get_cloud", [](const std::string& name) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");

        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) return py::none();

        bridge->holdCloud(cloud);
        bridge->markCloudInUse(QString::fromStdString(cloud->id()));

        return py::cast(PyCloud(cloud));
    }, py::arg("name"), "Get cloud by name, returns ct.Cloud or None");

    // --- add_cloud ---
    m.def("add_cloud", [](const std::string& name,
                          py::array_t<float> xyz,
                          py::object colors_obj) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");

        auto buf = xyz.request();
        if (buf.ndim != 2 || buf.shape[1] != 3)
            throw std::runtime_error("XYZ array must have shape (N, 3)");

        size_t count = static_cast<size_t>(buf.shape[0]);
        if (count == 0) throw std::runtime_error("XYZ array is empty");

        const float* data = static_cast<const float*>(buf.ptr);

        auto cloud = std::make_shared<ct::Cloud>();
        cloud->setId(name);

        auto config = ct::Cloud::calculateAdaptiveConfig(count);
        cloud->setConfig(config);

        float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
        float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
        for (size_t i = 0; i < count; ++i) {
            float x = data[i*3], y = data[i*3+1], z = data[i*3+2];
            if (x < min_x) min_x = x; if (x > max_x) max_x = x;
            if (y < min_y) min_y = y; if (y > max_y) max_y = y;
            if (z < min_z) min_z = z; if (z > max_z) max_z = z;
        }
        ct::Box globalBox;
        globalBox.translation = Eigen::Vector3f(
            (min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f, (min_z + max_z) * 0.5f);
        globalBox.width  = max_x - min_x;
        globalBox.height = max_y - min_y;
        globalBox.depth  = max_z - min_z;

        cloud->initOctree(globalBox);

        std::vector<ct::PointXYZ> points(count);
        for (size_t i = 0; i < count; ++i) {
            points[i].x = data[i*3];
            points[i].y = data[i*3+1];
            points[i].z = data[i*3+2];
        }

        bool has_colors = !colors_obj.is_none();
        std::vector<ct::ColorRGB> colors;
        if (has_colors) {
            auto color_arr = py::cast<py::array_t<uint8_t>>(colors_obj);
            auto cbuf = color_arr.request();
            if (cbuf.ndim != 2 || cbuf.shape[1] != 3)
                throw std::runtime_error("Color array must have shape (N, 3)");
            if (static_cast<size_t>(cbuf.shape[0]) != count)
                throw std::runtime_error("Color array length must match XYZ");
            colors.resize(count);
            const uint8_t* cdata = static_cast<const uint8_t*>(cbuf.ptr);
            for (size_t i = 0; i < count; ++i) {
                colors[i].r = cdata[i*3];
                colors[i].g = cdata[i*3+1];
                colors[i].b = cdata[i*3+2];
            }
            cloud->enableColors();
        }

        if (has_colors)
            cloud->addPoints(points, &colors);
        else
            cloud->addPoints(points);

        cloud->makeAdaptive();
        cloud->update();

        bridge->insertCloud(cloud);

        return py::cast(PyCloud(cloud));
    }, py::arg("name"), py::arg("xyz"), py::arg("colors") = py::none(),
       "Create a new cloud from numpy arrays and add to scene");

    m.def("insert_cloud", [](ct::Cloud::Ptr cloud) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->insertCloud(cloud);
    }, py::arg("cloud"), "Insert a cloud into the tree and view");

    m.def("remove_selected_clouds", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->removeSelectedClouds();
    }, "Remove currently selected clouds");

    // --- PyCloud class ---
    py::class_<PyCloud>(m, "Cloud")
        .def(py::init<ct::Cloud::Ptr>())
        .def("size", &PyCloud::size, "Total number of points")
        .def("num_blocks", &PyCloud::numBlocks, "Number of data blocks")
        .def("name", &PyCloud::name, "Get cloud name")
        .def("set_name", &PyCloud::setName, "Set cloud name")
        .def("block_size", &PyCloud::blockSize, "Number of points in block[i]")
        .def("block_to_numpy", &PyCloud::blockToNumpy, "Zero-copy NumPy view of block[i] XYZ, shape (M, 3)")
        .def("block_get_colors", &PyCloud::blockGetColors, "Zero-copy NumPy view of block[i] colors, shape (M, 3)")
        .def("block_set_colors", &PyCloud::blockSetColors, "Set block[i] colors from NumPy array (N, 3)")
        .def("block_set_numpy", &PyCloud::blockSetNumpy, "Set block[i] XYZ from NumPy array (N, 3)")
        .def("block_mark_dirty", &PyCloud::blockMarkDirty, "Mark block[i] dirty and recalculate its bounding box")
        .def("refresh", &PyCloud::refresh, "Trigger VTK render update")
        .def("to_numpy", &PyCloud::toNumpy, "Copy all blocks into one contiguous array, shape (N, 3)")
        .def("get_colors", &PyCloud::getColors, "Copy all block colors into one array, shape (N, 3)")
        .def("has_colors", &PyCloud::hasColors, "Check if cloud has color data")
        .def("has_normals", &PyCloud::hasNormals, "Check if cloud has normal data")
        .def("bounding_box", &PyCloud::boundingBox, "Get bounding box as dict {cx, cy, cz, width, height, depth}")
        .def("center", &PyCloud::center, "Get center point as [x, y, z]")
        .def("resolution", &PyCloud::res, "Get point resolution")
        .def("volume", &PyCloud::vol, "Get bounding box volume")
        .def("filepath", &PyCloud::filepath, "Get source file path")
        .def("add_scalar_field", &PyCloud::addScalarField, "Add a scalar field from numpy array", py::arg("name"), py::arg("data"))
        .def("get_scalar_field", &PyCloud::getScalarField, "Get scalar field data as numpy array", py::arg("name"))
        .def("get_scalar_field_names", &PyCloud::getScalarFieldNames, "Get names of all scalar fields")
        .def("remove_scalar_field", &PyCloud::removeScalarField, "Remove a scalar field by name", py::arg("name"))
        .def("clear_scalar_fields", &PyCloud::clearScalarFields, "Remove all scalar fields")
        .def("has_scalar_field", &PyCloud::hasScalarField, "Check if scalar field exists", py::arg("name"))
        .def("update_color_by_field", &PyCloud::updateColorByField, "Colorize cloud by scalar field values", py::arg("field_name"));
}
