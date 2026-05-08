#include "bind_cloud_mgmt.h"
#include "bind_core.h"
#include "python_bridge.h"

void registerCloudMgmtBindings(py::module_& m)
{
    // In-place update cloud data
    m.def("update_cloud", [](const std::string& name, py::array_t<float> xyz,
                              py::object colors_obj) {
        auto& registry = getRegistry();
        auto old_cloud = registry.getCloud(name);
        if (!old_cloud) throw std::runtime_error("Cloud not found: " + name);

        auto buf = xyz.request();
        if (buf.ndim != 2 || buf.shape[1] != 3)
            throw std::runtime_error("XYZ array must have shape (N, 3)");
        size_t count = static_cast<size_t>(buf.shape[0]);
        const float* data = static_cast<const float*>(buf.ptr);

        auto new_cloud = std::make_shared<pw::Cloud>();
        new_cloud->setId(old_cloud->id());
        auto config = pw::Cloud::calculateAdaptiveConfig(count);
        new_cloud->setConfig(config);

        float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
        float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
        for (size_t i = 0; i < count; ++i) {
            float x = data[i*3], y = data[i*3+1], z = data[i*3+2];
            if (x < min_x) min_x = x; if (x > max_x) max_x = x;
            if (y < min_y) min_y = y; if (y > max_y) max_y = y;
            if (z < min_z) min_z = z; if (z > max_z) max_z = z;
        }
        pw::Box globalBox;
        globalBox.translation = Eigen::Vector3f((min_x+max_x)*0.5f, (min_y+max_y)*0.5f, (min_z+max_z)*0.5f);
        globalBox.width = max_x - min_x;
        globalBox.height = max_y - min_y;
        globalBox.depth = max_z - min_z;
        new_cloud->initOctree(globalBox);

        std::vector<pw::PointXYZ> points(count);
        for (size_t i = 0; i < count; ++i) {
            points[i].x = data[i*3]; points[i].y = data[i*3+1]; points[i].z = data[i*3+2];
        }

        bool has_colors = !colors_obj.is_none();
        std::vector<pw::ColorRGB> colors;
        if (has_colors) {
            auto color_arr = py::cast<py::array_t<uint8_t>>(colors_obj);
            auto cbuf = color_arr.request();
            if (cbuf.ndim != 2 || cbuf.shape[1] != 3)
                throw std::runtime_error("Color array must have shape (N, 3)");
            colors.resize(count);
            const uint8_t* cdata = static_cast<const uint8_t*>(cbuf.ptr);
            for (size_t i = 0; i < count; ++i) {
                colors[i].r = cdata[i*3]; colors[i].g = cdata[i*3+1]; colors[i].b = cdata[i*3+2];
            }
            new_cloud->enableColors();
        }

        if (has_colors)
            new_cloud->addPoints(points, &colors);
        else
            new_cloud->addPoints(points);
        new_cloud->generateLOD();
        new_cloud->update();

        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->updateCloud(QString::fromStdString(name), new_cloud);
    }, py::arg("name"), py::arg("xyz"), py::arg("colors") = py::none(),
       "Replace cloud data in-place with new xyz (and optional colors) arrays");

    // Get names of all registered clouds
    m.def("get_all_cloud_names", []() -> py::list {
        auto names = getRegistry().getCloudNames();
        py::list result;
        for (const auto& n : names)
            result.append(py::cast(n));
        return result;
    }, "Get names of all registered clouds");

    // Remove a cloud by name
    m.def("remove_cloud", [](const std::string& name) {
        auto& registry = getRegistry();
        auto cloud = registry.getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        registry.unregisterCloud(name);
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->removeCloud(QString::fromStdString(name));
    }, py::arg("name"), "Remove a cloud by name");

    // Remove all clouds
    m.def("remove_all_clouds", []() {
        auto& registry = getRegistry();
        auto names = registry.getCloudNames();
        for (const auto& n : names)
            registry.unregisterCloud(n);
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->removeAllClouds();
    }, "Remove all clouds from the scene");

    // Clear all Python-generated data
    m.def("clear_all", []() {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->requestClearAll();
    }, "Clear all Python-generated data (clouds and meshes) from the scene");

    // Clear script-generated unmounted data only
    m.def("clear_script_data", []() {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->clearScriptData();
    }, "Clear script-generated data that has not been explicitly shown via .show() or add_to_scene()");

    // Explicitly add a cloud to the scene
    m.def("add_to_scene", [](PyCloud& cloud, const std::string& name) {
        auto cloud_ptr = cloud.cloudPtr();
        if (!name.empty()) cloud_ptr->setId(name);
        getRegistry().registerCloud(cloud_ptr);
        getRegistry().markSceneMounted(cloud_ptr->id());
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->insertCloud(cloud_ptr);
    }, py::arg("cloud"), py::arg("name") = "",
       "Explicitly add a ct.Cloud to the scene tree and view. Equivalent to cloud.show(name).");

    // Clone a cloud
    m.def("clone_cloud", [](const std::string& name) -> py::object {
        auto& registry = getRegistry();
        auto cloud = registry.getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto cloned = cloud->clone();
        cloned->setId("clone-" + cloud->id());
        cloned->makeAdaptive();
        registry.registerCloud(cloned);
        registry.holdCloud(cloned);
        if (shouldAutoInsert()) {
            auto* bridge = pw::PythonManager::instance().bridge();
            if (bridge) bridge->insertCloud(cloned);
        }

        return py::cast(PyCloud(cloned));
    }, py::arg("name"), "Clone a cloud, returns new ct.Cloud");

    // Merge multiple clouds
    m.def("merge_clouds", [](py::list name_list) -> py::object {
        auto& registry = getRegistry();
        if (name_list.size() < 2)
            throw std::runtime_error("Need at least 2 clouds to merge");

        std::vector<pw::Cloud::Ptr> clouds;
        for (auto item : name_list) {
            std::string n = py::cast<std::string>(item);
            auto c = registry.getCloud(n);
            if (!c) throw std::runtime_error("Cloud not found: " + n);
            clouds.push_back(c);
        }

        auto merged = std::make_shared<pw::Cloud>();
        merged->setId("merge-" + clouds[0]->id());
        auto config = pw::Cloud::calculateAdaptiveConfig(
            [&]() {
                size_t total = 0;
                for (auto& c : clouds) total += c->size();
                return total;
            }()
        );
        merged->setConfig(config);

        float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
        float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
        for (auto& c : clouds) {
            auto b = c->box();
            auto cen = b.translation;
            float hx = b.width * 0.5f, hy = b.height * 0.5f, hz = b.depth * 0.5f;
            min_x = std::min(min_x, cen.x() - hx); max_x = std::max(max_x, cen.x() + hx);
            min_y = std::min(min_y, cen.y() - hy); max_y = std::max(max_y, cen.y() + hy);
            min_z = std::min(min_z, cen.z() - hz); max_z = std::max(max_z, cen.z() + hz);
        }
        pw::Box globalBox;
        globalBox.translation = Eigen::Vector3f((min_x+max_x)*0.5f, (min_y+max_y)*0.5f, (min_z+max_z)*0.5f);
        globalBox.width = max_x - min_x;
        globalBox.height = max_y - min_y;
        globalBox.depth = max_z - min_z;

        merged->initOctree(globalBox);
        if (clouds[0]->hasColors()) merged->enableColors();

        for (auto& c : clouds) {
            for (auto& block : c->getBlocks()) {
                std::vector<pw::PointXYZ> pts = block->m_points;
                std::vector<pw::ColorRGB> colors;
                if (c->hasColors() && block->m_colors) {
                    colors = *block->m_colors;
                    merged->addPoints(pts, &colors);
                } else {
                    merged->addPoints(pts);
                }
            }
        }
        merged->generateLOD();
        merged->update();
        merged->makeAdaptive();

        registry.registerCloud(merged);
        registry.holdCloud(merged);
        if (shouldAutoInsert()) {
            auto* bridge = pw::PythonManager::instance().bridge();
            if (bridge) bridge->insertCloud(merged);
        }

        return py::cast(PyCloud(merged));
    }, py::arg("names"), "Merge multiple clouds by name list, returns new ct.Cloud");

    // Select a cloud
    m.def("select_cloud", [](const std::string& name) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->selectCloud(QString::fromStdString(name));
    }, py::arg("name"), "Select a cloud by name in the tree");

    // Load a cloud file
    m.def("load_cloud", [](const std::string& filepath) -> py::object {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->loadCloud(QString::fromStdString(filepath));
        return py::none();
    }, py::arg("filepath"), "Load a point cloud file into the scene (async)");

    // Save a cloud to file
    m.def("save_cloud", [](const std::string& name, const std::string& filepath, bool binary) {
        auto& registry = getRegistry();
        auto cloud = registry.getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->saveCloud(QString::fromStdString(name), QString::fromStdString(filepath), binary);
    }, py::arg("name"), py::arg("filepath"), py::arg("binary") = true,
       "Save a cloud to file. binary=True for binary, False for ASCII (async)");
}
