#include "bind_cloud_mgmt.h"
#include "bind_core.h"

void registerCloudMgmtBindings(py::module_& m)
{
    // 就地更新点云数据
    m.def("update_cloud", [](const std::string& name, py::array_t<float> xyz,
                              py::object colors_obj) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        auto old_cloud = bridge->getCloud(QString::fromStdString(name));
        if (!old_cloud) throw std::runtime_error("Cloud not found: " + name);

        auto buf = xyz.request();
        if (buf.ndim != 2 || buf.shape[1] != 3)
            throw std::runtime_error("XYZ array must have shape (N, 3)");
        size_t count = static_cast<size_t>(buf.shape[0]);
        const float* data = static_cast<const float*>(buf.ptr);

        auto new_cloud = std::make_shared<ct::Cloud>();
        new_cloud->setId(old_cloud->id());
        auto config = ct::Cloud::calculateAdaptiveConfig(count);
        new_cloud->setConfig(config);

        float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
        float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
        for (size_t i = 0; i < count; ++i) {
            float x = data[i*3], y = data[i*3+1], z = data[i*3+2];
            if (x < min_x) min_x = x; if (x > max_x) max_x = x;
            if (y < min_y) min_y = y; if (y > max_y) max_y = y;
            if (z < min_z) min_z = z; if (z > max_z) max_z = z;
        }
        ct::Box globalBox;
        globalBox.translation = Eigen::Vector3f((min_x+max_x)*0.5f, (min_y+max_y)*0.5f, (min_z+max_z)*0.5f);
        globalBox.width = max_x - min_x;
        globalBox.height = max_y - min_y;
        globalBox.depth = max_z - min_z;
        new_cloud->initOctree(globalBox);

        std::vector<ct::PointXYZ> points(count);
        for (size_t i = 0; i < count; ++i) {
            points[i].x = data[i*3]; points[i].y = data[i*3+1]; points[i].z = data[i*3+2];
        }

        bool has_colors = !colors_obj.is_none();
        std::vector<ct::ColorRGB> colors;
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

        bridge->updateCloud(QString::fromStdString(name), new_cloud);
    }, py::arg("name"), py::arg("xyz"), py::arg("colors") = py::none(),
       "Replace cloud data in-place with new xyz (and optional colors) arrays");

    // 获取所有已注册点云的名称列表
    m.def("get_all_cloud_names", []() -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) return py::list();
        auto names = bridge->getCloudNames();
        py::list result;
        for (const auto& n : names)
            result.append(py::cast(n.toStdString()));
        return result;
    }, "Get names of all registered clouds");

    // 按名称移除单个点云
    m.def("remove_cloud", [](const std::string& name) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        bridge->unregisterCloud(QString::fromStdString(name));
        bridge->removeCloud(QString::fromStdString(name));
    }, py::arg("name"), "Remove a cloud by name");

    // 移除所有点云
    m.def("remove_all_clouds", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        auto names = bridge->getCloudNames();
        for (const auto& n : names)
            bridge->unregisterCloud(n);
        bridge->removeAllClouds();
    }, "Remove all clouds from the scene");

    // 清理所有 Python 生成的数据（点云+网格）
    m.def("clear_all", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        bridge->requestClearAll();
    }, "Clear all Python-generated data (clouds and meshes) from the scene");

    // 克隆点云
    m.def("clone_cloud", [](const std::string& name) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto cloned = cloud->clone();
        cloned->setId("clone-" + cloud->id());
        cloned->makeAdaptive();
        bridge->registerCloud(cloned);
        bridge->holdCloud(cloned);
        bridge->insertCloud(cloned);

        return py::cast(PyCloud(cloned));
    }, py::arg("name"), "Clone a cloud, returns new ct.Cloud");

    // 合并多个点云
    m.def("merge_clouds", [](py::list name_list) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        if (name_list.size() < 2)
            throw std::runtime_error("Need at least 2 clouds to merge");

        std::vector<ct::Cloud::Ptr> clouds;
        for (auto item : name_list) {
            std::string n = py::cast<std::string>(item);
            auto c = bridge->getCloud(QString::fromStdString(n));
            if (!c) throw std::runtime_error("Cloud not found: " + n);
            clouds.push_back(c);
        }

        auto merged = std::make_shared<ct::Cloud>();
        merged->setId("merge-" + clouds[0]->id());
        auto config = ct::Cloud::calculateAdaptiveConfig(
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
        ct::Box globalBox;
        globalBox.translation = Eigen::Vector3f((min_x+max_x)*0.5f, (min_y+max_y)*0.5f, (min_z+max_z)*0.5f);
        globalBox.width = max_x - min_x;
        globalBox.height = max_y - min_y;
        globalBox.depth = max_z - min_z;

        merged->initOctree(globalBox);
        if (clouds[0]->hasColors()) merged->enableColors();

        for (auto& c : clouds) {
            for (auto& block : c->getBlocks()) {
                std::vector<ct::PointXYZ> pts = block->m_points;
                std::vector<ct::ColorRGB> colors;
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

        bridge->registerCloud(merged);
        bridge->holdCloud(merged);
        bridge->insertCloud(merged);

        return py::cast(PyCloud(merged));
    }, py::arg("names"), "Merge multiple clouds by name list, returns new ct.Cloud");

    // 选中点云
    m.def("select_cloud", [](const std::string& name) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        bridge->selectCloud(QString::fromStdString(name));
    }, py::arg("name"), "Select a cloud by name in the tree");

    // 加载点云文件
    m.def("load_cloud", [](const std::string& filepath) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        bridge->loadCloud(QString::fromStdString(filepath));
        return py::none();
    }, py::arg("filepath"), "Load a point cloud file into the scene (async)");

    // 保存点云到文件
    m.def("save_cloud", [](const std::string& name, const std::string& filepath, bool binary) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        bridge->saveCloud(QString::fromStdString(name), QString::fromStdString(filepath), binary);
    }, py::arg("name"), py::arg("filepath"), py::arg("binary") = true,
       "Save a cloud to file. binary=True for binary, False for ASCII (async)");
}
