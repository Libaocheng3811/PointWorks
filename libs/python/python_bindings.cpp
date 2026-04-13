#include "python_manager.h"
#include "python_bridge.h"

#include "core/cloud.h"
#include "core/octree.h"

#include "algorithm/filters.h"
#include "algorithm/csffilter.h"
#include "algorithm/vegfilter.h"
#include "algorithm/distancecalculator.h"
#include "algorithm/features.h"
#include "algorithm/registration.h"
#include "core/field_types.h"

// Qt 的 <QObject> 定义了 slots 宏，与 Python 的 object.h 冲突
#undef slots
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// ============================================================================
// PyCloud — Python 端的点云访问包装
// 注意：不放在 namespace ct 中，避免与 PYBIND11_EMBEDDED_MODULE(ct, ...) 冲突
// ============================================================================
class PyCloud
{
public:
    explicit PyCloud(ct::Cloud::Ptr cloud) : m_cloud(cloud) {}

    size_t size() const { return m_cloud->size(); }

    int numBlocks() const { return static_cast<int>(m_cloud->getBlocks().size()); }

    std::string name() const { return m_cloud->id(); }

    void setName(const std::string& name) { m_cloud->setId(name); }

    size_t blockSize(int idx) const
    {
        auto& blocks = m_cloud->getBlocks();
        if (idx < 0 || idx >= static_cast<int>(blocks.size()))
            throw py::index_error("Block index out of range");
        return blocks[idx]->size();
    }

    // === 按 Block 零拷贝：XYZ 坐标 ===

    py::array_t<float> blockToNumpy(int idx)
    {
        auto& blocks = m_cloud->getBlocks();
        if (idx < 0 || idx >= static_cast<int>(blocks.size()))
            throw py::index_error("Block index out of range");

        auto& pts = blocks[idx]->m_points;

        // 将 shared_ptr<Cloud> 藏入 capsule，绑定生命周期
        auto* holder = new ct::Cloud::Ptr(m_cloud);
        auto capsule = py::capsule(holder, [](void* ptr) {
            delete reinterpret_cast<ct::Cloud::Ptr*>(ptr);
        });

        // pcl::PointXYZ 内存布局: {float x, y, z, padding} = 16 bytes
        return py::array_t<float>(
            {static_cast<py::ssize_t>(pts.size()), static_cast<py::ssize_t>(3)},
            {static_cast<py::ssize_t>(sizeof(pcl::PointXYZ)),
             static_cast<py::ssize_t>(sizeof(float))},
            reinterpret_cast<const float*>(pts.data()),
            capsule
        );
    }

    // === 按 Block 零拷贝：颜色 ===

    py::array_t<uint8_t> blockGetColors(int idx)
    {
        auto& blocks = m_cloud->getBlocks();
        if (idx < 0 || idx >= static_cast<int>(blocks.size()))
            throw py::index_error("Block index out of range");

        auto& block = blocks[idx];
        if (!block->m_colors)
            throw std::runtime_error("This cloud has no color data");

        auto& colors = *block->m_colors;

        auto* holder = new ct::Cloud::Ptr(m_cloud);
        auto capsule = py::capsule(holder, [](void* ptr) {
            delete reinterpret_cast<ct::Cloud::Ptr*>(ptr);
        });

        // ct::ColorRGB 内存布局: {uint8_t r, g, b} = 3 bytes
        return py::array_t<uint8_t>(
            {static_cast<py::ssize_t>(colors.size()), static_cast<py::ssize_t>(3)},
            {static_cast<py::ssize_t>(sizeof(ct::ColorRGB)),
             static_cast<py::ssize_t>(sizeof(uint8_t))},
            reinterpret_cast<const uint8_t*>(colors.data()),
            capsule
        );
    }

    // === 按 Block 设置颜色（从 NumPy 拷贝） ===

    void blockSetColors(int idx, py::array_t<uint8_t> array)
    {
        auto& blocks = m_cloud->getBlocks();
        if (idx < 0 || idx >= static_cast<int>(blocks.size()))
            throw py::index_error("Block index out of range");

        auto buf = array.request();
        if (buf.ndim != 2 || buf.shape[1] != 3)
            throw std::runtime_error("Color array must have shape (N, 3)");

        auto& block = blocks[idx];
        if (!block->m_colors)
            block->m_colors = std::make_unique<std::vector<ct::ColorRGB>>();

        auto& colors = *block->m_colors;
        auto count = static_cast<size_t>(buf.shape[0]);
        colors.resize(count);

        const uint8_t* src = static_cast<const uint8_t*>(buf.ptr);
        for (size_t i = 0; i < count; ++i) {
            colors[i].r = src[i * 3];
            colors[i].g = src[i * 3 + 1];
            colors[i].b = src[i * 3 + 2];
        }
        block->m_is_dirty = true;
    }

    // === 按 Block 写回 XYZ（从 NumPy 拷贝） ===

    void blockSetNumpy(int idx, py::array_t<float> array)
    {
        auto& blocks = m_cloud->getBlocks();
        if (idx < 0 || idx >= static_cast<int>(blocks.size()))
            throw py::index_error("Block index out of range");

        auto buf = array.request();
        if (buf.ndim != 2 || buf.shape[1] != 3)
            throw std::runtime_error("XYZ array must have shape (N, 3)");

        auto& pts = blocks[idx]->m_points;
        auto count = static_cast<size_t>(buf.shape[0]);
        pts.resize(count);

        const float* src = static_cast<const float*>(buf.ptr);
        if (buf.strides[0] == sizeof(float) * 3 && buf.strides[1] == sizeof(float)) {
            for (size_t i = 0; i < count; ++i) {
                pts[i].x = src[i * 3];
                pts[i].y = src[i * 3 + 1];
                pts[i].z = src[i * 3 + 2];
            }
        } else {
            for (size_t i = 0; i < count; ++i) {
                auto row = reinterpret_cast<const float*>(
                    reinterpret_cast<const char*>(src) + i * buf.strides[0]);
                pts[i].x = row[0];
                pts[i].y = row[1];
                pts[i].z = row[2];
            }
        }
        blocks[idx]->m_is_dirty = true;
    }

    // === 标记 Block 脏 + 重算包围盒 ===

    void blockMarkDirty(int idx)
    {
        auto& blocks = m_cloud->getBlocks();
        if (idx < 0 || idx >= static_cast<int>(blocks.size()))
            throw py::index_error("Block index out of range");

        auto& block = blocks[idx];
        block->m_is_dirty = true;

        if (!block->m_points.empty()) {
            float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
            float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
            for (auto& pt : block->m_points) {
                if (pt.x < min_x) min_x = pt.x; if (pt.x > max_x) max_x = pt.x;
                if (pt.y < min_y) min_y = pt.y; if (pt.y > max_y) max_y = pt.y;
                if (pt.z < min_z) min_z = pt.z; if (pt.z > max_z) max_z = pt.z;
            }
            block->m_box.translation = Eigen::Vector3f(
                (min_x + max_x) * 0.5f,
                (min_y + max_y) * 0.5f,
                (min_z + max_z) * 0.5f);
            block->m_box.width  = max_x - min_x;
            block->m_box.height = max_y - min_y;
            block->m_box.depth  = max_z - min_z;
        }
    }

    // === 刷新视图：重建 Cloud 元数据 + 失效渲染缓存 + 触发重绘 ===

    void refresh()
    {
        m_cloud->update();                 // 重算总点数、包围盒、分辨率
        m_cloud->invalidateRenderCache();  // 失效 VTK 渲染缓存

        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) {
            bridge->cloudChanged(QString::fromStdString(name()));
            bridge->refreshView();
        }
    }

    // === 全量拷贝：合并所有 Block 为一个连续 NumPy 数组 ===

    py::array_t<float> toNumpy()
    {
        size_t total = m_cloud->size();
        auto result = py::array_t<float>({static_cast<py::ssize_t>(total),
                                          static_cast<py::ssize_t>(3)});
        auto buf = result.request();
        float* dst = static_cast<float*>(buf.ptr);

        for (auto& block : m_cloud->getBlocks()) {
            for (auto& pt : block->m_points) {
                *dst++ = pt.x;
                *dst++ = pt.y;
                *dst++ = pt.z;
            }
        }
        return result;
    }

    // === 全量拷贝：合并所有 Block 颜色 ===

    py::array_t<uint8_t> getColors()
    {
        size_t total = m_cloud->size();
        auto result = py::array_t<uint8_t>({static_cast<py::ssize_t>(total),
                                            static_cast<py::ssize_t>(3)});
        auto buf = result.request();
        uint8_t* dst = static_cast<uint8_t*>(buf.ptr);

        for (auto& block : m_cloud->getBlocks()) {
            if (block->m_colors) {
                for (auto& c : *block->m_colors) {
                    *dst++ = c.r;
                    *dst++ = c.g;
                    *dst++ = c.b;
                }
            } else {
                for (size_t i = 0; i < block->m_points.size(); ++i) {
                    *dst++ = 255;
                    *dst++ = 255;
                    *dst++ = 255;
                }
            }
        }
        return result;
    }

    bool hasColors() const { return m_cloud->hasColors(); }
    bool hasNormals() const { return m_cloud->hasNormals(); }

    // === 元数据属性 ===

    py::dict boundingBox() const {
        auto b = m_cloud->box();
        py::dict dict;
        dict["cx"] = static_cast<double>(b.translation.x());
        dict["cy"] = static_cast<double>(b.translation.y());
        dict["cz"] = static_cast<double>(b.translation.z());
        dict["width"]  = b.width;
        dict["height"] = b.height;
        dict["depth"]  = b.depth;
        return dict;
    }

    std::vector<double> center() const {
        auto c = m_cloud->center();
        return {static_cast<double>(c.x()), static_cast<double>(c.y()), static_cast<double>(c.z())};
    }

    float res() const { return m_cloud->resolution(); }

    double vol() const { return m_cloud->volume(); }

    std::string filepath() const { return m_cloud->filepath(); }

    // === 标量场 ===

    void addScalarField(const std::string& name, py::array_t<float> array) {
        auto buf = array.request();
        auto count = static_cast<size_t>(buf.shape[0]);
        const float* src = static_cast<const float*>(buf.ptr);
        std::vector<float> data(src, src + count);
        m_cloud->addScalarField(name, data);
    }

    py::array_t<float> getScalarField(const std::string& name) const {
        const auto* data = m_cloud->getScalarField(name);
        if (!data) throw std::runtime_error("Scalar field not found: " + name);
        auto count = static_cast<py::ssize_t>(data->size());
        auto arr = py::array_t<float>(count);
        std::memcpy(arr.request().ptr, data->data(), count * sizeof(float));
        return arr;
    }

    std::vector<std::string> getScalarFieldNames() const {
        return m_cloud->getScalarFieldNames();
    }

    bool removeScalarField(const std::string& name) {
        return m_cloud->removeScalarField(name);
    }

    void clearScalarFields() { m_cloud->clearScalarFields(); }

    bool hasScalarField(const std::string& name) const {
        return m_cloud->hasScalarField(name);
    }

    void updateColorByField(const std::string& field_name) {
        m_cloud->updateColorByField(field_name);
    }

private:
    ct::Cloud::Ptr m_cloud;
};

// ============================================================================
// Python 模块注册
// ============================================================================
PYBIND11_EMBEDDED_MODULE(ct, m)
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
    // C++ 侧无法直接构造 PyCloud（类型不导出），通过 capsule 传递 shared_ptr
    m.def("_wrap_cloud", [](py::capsule cap) -> py::object {
        auto* cloud_ptr = static_cast<ct::Cloud::Ptr*>(cap);
        if (!cloud_ptr || !*cloud_ptr)
            throw std::runtime_error("Invalid cloud capsule");
        return py::cast(PyCloud(*cloud_ptr));
    }, py::arg("cap"), "Internal: wrap Cloud::Ptr into ct.Cloud");

    // --- 按名称获取点云（线程安全，自动持有引用 + 标记 in-use） ---
    m.def("get_cloud", [](const std::string& name) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");

        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) return py::none();

        // 持有引用 + 标记 in-use
        bridge->holdCloud(cloud);
        bridge->markCloudInUse(QString::fromStdString(cloud->id()));

        return py::cast(PyCloud(cloud));
    }, py::arg("name"), "Get cloud by name, returns ct.Cloud or None");

    // ================================================================
    // 视图控制
    // ================================================================

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

    // ================================================================
    // 点云外观
    // ================================================================

    m.def("set_point_size", [](const std::string& id, float size) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setPointSize(QString::fromStdString(id), size);
    }, py::arg("id"), py::arg("size"), "Set point size for a cloud");

    m.def("set_opacity", [](const std::string& id, float value) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setOpacity(QString::fromStdString(id), value);
    }, py::arg("id"), py::arg("value"), "Set cloud opacity (0.0 - 1.0)");

    m.def("set_cloud_color", [](const std::string& id, float r, float g, float b) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setCloudColorRGB(QString::fromStdString(id), r, g, b);
    }, py::arg("id"), py::arg("r"), py::arg("g"), py::arg("b"),
       "Set cloud color by RGB (0.0 - 1.0)");

    m.def("set_color_by_axis", [](const std::string& id, const std::string& axis) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setCloudColorByAxis(QString::fromStdString(id), QString::fromStdString(axis));
    }, py::arg("id"), py::arg("axis"), "Color cloud by axis (X/Y/Z)");

    m.def("reset_cloud_color", [](const std::string& id) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->resetCloudColor(QString::fromStdString(id));
    }, py::arg("id"), "Reset cloud to original colors");

    m.def("set_cloud_visibility", [](const std::string& id, bool visible) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setCloudVisibility(QString::fromStdString(id), visible);
    }, py::arg("id"), py::arg("visible"), "Show or hide a cloud");

    // ================================================================
    // 场景外观
    // ================================================================

    m.def("set_background_color", [](float r, float g, float b) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setBackgroundColor(r, g, b);
    }, py::arg("r"), py::arg("g"), py::arg("b"),
       "Set background color (0.0 - 1.0)");

    m.def("reset_background_color", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->resetBackgroundColor();
    }, "Reset background to default color");

    // ================================================================
    // 显示开关
    // ================================================================

    m.def("show_id", [](bool show) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->showId(show);
    }, py::arg("show"), "Show or hide cloud IDs");

    m.def("show_axes", [](bool show) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->showAxes(show);
    }, py::arg("show"), "Show or hide coordinate axes");

    m.def("show_fps", [](bool show) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->showFPS(show);
    }, py::arg("show"), "Show or hide FPS counter");

    m.def("show_info", [](const std::string& text) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->showInfo(QString::fromStdString(text));
    }, py::arg("text"), "Show info text overlay");

    m.def("clear_info", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->clearInfo();
    }, "Clear all info text overlays");

    // ================================================================
    // 叠加物
    // ================================================================

    m.def("add_cube", [](float cx, float cy, float cz, float size, const std::string& id) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->addCube(cx, cy, cz, size, QString::fromStdString(id));
    }, py::arg("cx"), py::arg("cy"), py::arg("cz"),
       py::arg("size"), py::arg("id") = "cube",
       "Add a cube overlay at center (cx,cy,cz) with given size");

    m.def("add_3d_label", [](const std::string& text, float x, float y, float z, const std::string& id) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->add3DLabel(QString::fromStdString(text), x, y, z, QString::fromStdString(id));
    }, py::arg("text"), py::arg("x"), py::arg("y"), py::arg("z"),
       py::arg("id") = "label",
       "Add a 3D text label at position (x,y,z)");

    m.def("remove_shape", [](const std::string& id) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->removeShape(QString::fromStdString(id));
    }, py::arg("id"), "Remove a shape/overlay by ID");

    m.def("remove_all_shapes", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->removeAllShapes();
    }, "Remove all shapes/overlays");

    // ================================================================
    // 点云管理
    // ================================================================

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

        // 1. Create cloud
        auto cloud = std::make_shared<ct::Cloud>();
        cloud->setId(name);

        // 2. Adaptive config
        auto config = ct::Cloud::calculateAdaptiveConfig(count);
        cloud->setConfig(config);

        // 3. Calculate bounding box from xyz
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

        // 4. Build point vectors
        std::vector<ct::PointXYZ> points(count);
        for (size_t i = 0; i < count; ++i) {
            points[i].x = data[i*3];
            points[i].y = data[i*3+1];
            points[i].z = data[i*3+2];
        }

        // 5. Build color vectors (optional)
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

        // 6. Add points to octree
        if (has_colors)
            cloud->addPoints(points, &colors);
        else
            cloud->addPoints(points);

        // 7. Generate LOD + adaptive config + update metadata
        cloud->makeAdaptive();
        cloud->update();

        // 8. Insert into scene
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

    // ================================================================
    // 进度
    // ================================================================

    m.def("show_progress", [](const std::string& title) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->showProgress(QString::fromStdString(title));
    }, py::arg("title"), "Show a progress dialog");

    m.def("set_progress", [](int percent) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setProgress(percent);
    }, py::arg("percent"), "Update progress (0-100)");

    m.def("close_progress", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->closeProgress();
    }, "Close the progress dialog");

    // --- PyCloud ---
    py::class_<PyCloud>(m, "Cloud")
        .def(py::init<ct::Cloud::Ptr>())
        .def("size", &PyCloud::size,
             "Total number of points")
        .def("num_blocks", &PyCloud::numBlocks,
             "Number of data blocks")
        .def("name", &PyCloud::name,
             "Get cloud name")
        .def("set_name", &PyCloud::setName,
             "Set cloud name")
        .def("block_size", &PyCloud::blockSize,
             "Number of points in block[i]")
        .def("block_to_numpy", &PyCloud::blockToNumpy,
             "Zero-copy NumPy view of block[i] XYZ, shape (M, 3)")
        .def("block_get_colors", &PyCloud::blockGetColors,
             "Zero-copy NumPy view of block[i] colors, shape (M, 3)")
        .def("block_set_colors", &PyCloud::blockSetColors,
             "Set block[i] colors from NumPy array (N, 3)")
        .def("block_set_numpy", &PyCloud::blockSetNumpy,
             "Set block[i] XYZ from NumPy array (N, 3)")
        .def("block_mark_dirty", &PyCloud::blockMarkDirty,
             "Mark block[i] dirty and recalculate its bounding box")
        .def("refresh", &PyCloud::refresh,
             "Trigger VTK render update")
        .def("to_numpy", &PyCloud::toNumpy,
             "Copy all blocks into one contiguous array, shape (N, 3)")
        .def("get_colors", &PyCloud::getColors,
             "Copy all block colors into one array, shape (N, 3)")
        .def("has_colors", &PyCloud::hasColors,
             "Check if cloud has color data")
        .def("has_normals", &PyCloud::hasNormals,
             "Check if cloud has normal data")
        .def("bounding_box", &PyCloud::boundingBox,
             "Get bounding box as dict {cx, cy, cz, width, height, depth}")
        .def("center", &PyCloud::center,
             "Get center point as [x, y, z]")
        .def("resolution", &PyCloud::res,
             "Get point resolution")
        .def("volume", &PyCloud::vol,
             "Get bounding box volume")
        .def("filepath", &PyCloud::filepath,
             "Get source file path")
        .def("add_scalar_field", &PyCloud::addScalarField,
             "Add a scalar field from numpy array", py::arg("name"), py::arg("data"))
        .def("get_scalar_field", &PyCloud::getScalarField,
             "Get scalar field data as numpy array", py::arg("name"))
        .def("get_scalar_field_names", &PyCloud::getScalarFieldNames,
             "Get names of all scalar fields")
        .def("remove_scalar_field", &PyCloud::removeScalarField,
             "Remove a scalar field by name", py::arg("name"))
        .def("clear_scalar_fields", &PyCloud::clearScalarFields,
             "Remove all scalar fields")
        .def("has_scalar_field", &PyCloud::hasScalarField,
             "Check if scalar field exists", py::arg("name"))
        .def("update_color_by_field", &PyCloud::updateColorByField,
             "Colorize cloud by scalar field values", py::arg("field_name"));

    // ================================================================
    // Phase 2: 算法绑定 — 过滤 / 分割 / 距离计算 / 特征
    // ================================================================

    // --- 辅助：将 FilterResult 的 result_cloud 插入场景并返回 PyCloud ---
    auto insertFilterResult = [](const ct::FilterResult& fr, const std::string& base_name) -> py::object {
        if (!fr.result_cloud) throw std::runtime_error("Filter produced no result");
        auto* bridge = ct::PythonManager::instance().bridge();
        fr.result_cloud->setId(base_name);
        fr.result_cloud->makeAdaptive();            // 生成 LOD + 配置渲染参数（颜色依赖此步）
        bridge->registerCloud(fr.result_cloud);
        bridge->holdCloud(fr.result_cloud);
        bridge->insertCloud(fr.result_cloud);
        return py::cast(PyCloud(fr.result_cloud));
    };

    // ---- 过滤 ----

    m.def("voxel_grid", [insertFilterResult](const std::string& name, float lx, float ly, float lz,
                           bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::VoxelGrid(cloud, lx, ly, lz, negative);
        return insertFilterResult(fr, "voxel-" + name);
    }, py::arg("name"), py::arg("lx"), py::arg("ly"), py::arg("lz"),
       py::arg("negative") = false,
       "Voxel grid downsampling, returns new ct.Cloud");

    m.def("approx_voxel_grid", [insertFilterResult](const std::string& name, float lx, float ly, float lz,
                                   bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::ApproximateVoxelGrid(cloud, lx, ly, lz, negative);
        return insertFilterResult(fr, "approx_voxel-" + name);
    }, py::arg("name"), py::arg("lx"), py::arg("ly"), py::arg("lz"),
       py::arg("negative") = false,
       "Approximate voxel grid downsampling, returns new ct.Cloud");

    m.def("statistical_outlier_removal", [insertFilterResult](const std::string& name, int nr_k,
                                             double stddev_mult, bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::StatisticalOutlierRemoval(cloud, nr_k, stddev_mult, negative);
        return insertFilterResult(fr, "sor-" + name);
    }, py::arg("name"), py::arg("nr_k") = 30, py::arg("stddev_mult") = 2.0,
       py::arg("negative") = false,
       "Statistical outlier removal, returns new ct.Cloud");

    m.def("radius_outlier_removal", [insertFilterResult](const std::string& name, double radius,
                                        int min_pts, bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::RadiusOutlierRemoval(cloud, radius, min_pts, negative);
        return insertFilterResult(fr, "ror-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("min_pts") = 2,
       py::arg("negative") = false,
       "Radius outlier removal, returns new ct.Cloud");

    m.def("pass_through", [insertFilterResult](const std::string& name, const std::string& field_name,
                              float limit_min, float limit_max, bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::PassThrough(cloud, field_name, limit_min, limit_max, negative);
        return insertFilterResult(fr, "passthrough-" + name);
    }, py::arg("name"), py::arg("field_name"), py::arg("limit_min"), py::arg("limit_max"),
       py::arg("negative") = false,
       "Pass-through filter on a field (x/y/z), returns new ct.Cloud");

    m.def("grid_minimum", [insertFilterResult](const std::string& name, float resolution,
                              bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::GridMinimun(cloud, resolution, negative);
        return insertFilterResult(fr, "gridmin-" + name);
    }, py::arg("name"), py::arg("resolution") = 1.0, py::arg("negative") = false,
       "Grid minimum filter, returns new ct.Cloud");

    m.def("local_maximum", [insertFilterResult](const std::string& name, float radius,
                               bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::LocalMaximum(cloud, radius, negative);
        return insertFilterResult(fr, "localmax-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("negative") = false,
       "Local maximum filter, returns new ct.Cloud");

    m.def("shadow_points", [insertFilterResult](const std::string& name, float threshold,
                               bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::ShadowPoints(cloud, threshold, negative);
        return insertFilterResult(fr, "shadow-" + name);
    }, py::arg("name"), py::arg("threshold") = 0.1, py::arg("negative") = false,
       "Shadow points removal, returns new ct.Cloud");

    // ---- 采样 ----

    m.def("down_sampling", [insertFilterResult](const std::string& name, float radius,
                               bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::DownSampling(cloud, radius, negative);
        return insertFilterResult(fr, "downsample-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("negative") = false,
       "Down sampling, returns new ct.Cloud");

    m.def("uniform_sampling", [insertFilterResult](const std::string& name, float radius,
                                  bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::UniformSampling(cloud, radius, negative);
        return insertFilterResult(fr, "uniform-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("negative") = false,
       "Uniform sampling, returns new ct.Cloud");

    m.def("random_sampling", [insertFilterResult](const std::string& name, int sample, int seed,
                                 bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::RandomSampling(cloud, sample, seed, negative);
        return insertFilterResult(fr, "random-" + name);
    }, py::arg("name"), py::arg("sample") = 1000, py::arg("seed") = 42,
       py::arg("negative") = false,
       "Random sampling, returns new ct.Cloud");

    m.def("resampling", [insertFilterResult](const std::string& name, float radius, int polynomial_order,
                            bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::ReSampling(cloud, radius, polynomial_order, negative);
        return insertFilterResult(fr, "resample-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("polynomial_order") = 2,
       py::arg("negative") = false,
       "MLS resampling, returns new ct.Cloud");

    m.def("normal_space_sampling", [insertFilterResult](const std::string& name, int sample, int seed,
                                       int bin, bool negative) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::NormalSpaceSampling(cloud, sample, seed, bin, negative);
        return insertFilterResult(fr, "nss-" + name);
    }, py::arg("name"), py::arg("sample") = 1000, py::arg("seed") = 42,
       py::arg("bin") = 10, py::arg("negative") = false,
       "Normal space sampling, returns new ct.Cloud");

    // ---- CSF 地面分割 ----

    m.def("csf_filter", [](const std::string& name,
                            bool smooth, float time_step, double class_threshold,
                            double cloth_resolution, int rigidness, int iterations) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto result = ct::CSFFilter::apply(cloud, smooth, time_step, class_threshold,
                                            cloth_resolution, rigidness, iterations);

        if (result.ground_cloud) {
            result.ground_cloud->setId("ground-" + name);
            result.ground_cloud->makeAdaptive();
            bridge->registerCloud(result.ground_cloud);
            bridge->holdCloud(result.ground_cloud);
            bridge->insertCloud(result.ground_cloud);
        }
        if (result.off_ground_cloud) {
            result.off_ground_cloud->setId("offground-" + name);
            result.off_ground_cloud->makeAdaptive();
            bridge->registerCloud(result.off_ground_cloud);
            bridge->holdCloud(result.off_ground_cloud);
            bridge->insertCloud(result.off_ground_cloud);
        }

        py::dict dict;
        dict["ground"] = result.ground_cloud ? py::cast(PyCloud(result.ground_cloud)) : py::none();
        dict["off_ground"] = result.off_ground_cloud ? py::cast(PyCloud(result.off_ground_cloud)) : py::none();
        dict["time_ms"] = result.time_ms;
        return dict;
    }, py::arg("name"), py::arg("smooth") = true, py::arg("time_step") = 1.0f,
       py::arg("class_threshold") = 0.5, py::arg("cloth_resolution") = 2.0,
       py::arg("rigidness") = 3, py::arg("iterations") = 500,
       "CSF ground segmentation, returns dict with 'ground' and 'off_ground' clouds");

    // ---- 植被分割 ----

    m.def("veg_filter", [](const std::string& name, int index_type,
                            double threshold) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto result = ct::VegetationFilter::apply(cloud, index_type, threshold);

        if (result.veg_cloud) {
            result.veg_cloud->setId("veg-" + name);
            result.veg_cloud->makeAdaptive();
            bridge->registerCloud(result.veg_cloud);
            bridge->holdCloud(result.veg_cloud);
            bridge->insertCloud(result.veg_cloud);
        }
        if (result.non_veg_cloud) {
            result.non_veg_cloud->setId("nonveg-" + name);
            result.non_veg_cloud->makeAdaptive();
            bridge->registerCloud(result.non_veg_cloud);
            bridge->holdCloud(result.non_veg_cloud);
            bridge->insertCloud(result.non_veg_cloud);
        }

        py::dict dict;
        dict["vegetation"] = result.veg_cloud ? py::cast(PyCloud(result.veg_cloud)) : py::none();
        dict["non_vegetation"] = result.non_veg_cloud ? py::cast(PyCloud(result.non_veg_cloud)) : py::none();
        dict["time_ms"] = result.time_ms;
        return dict;
    }, py::arg("name"), py::arg("index_type") = 0,
       py::arg("threshold") = 0.35,
       "Vegetation segmentation. index_type: 0=ExG_ExR, 1=ExG, 2=NGRDI, 3=CIVE. Returns dict with 'vegetation' and 'non_vegetation' clouds");

    // ---- 距离计算 ----

    m.def("calculate_distance", [](const std::string& ref_name,
                                    const std::string& comp_name,
                                    int method, int k_knn,
                                    double radius, bool flip_normals) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto ref_cloud = bridge->getCloud(QString::fromStdString(ref_name));
        if (!ref_cloud) throw std::runtime_error("Cloud not found: " + ref_name);
        auto comp_cloud = bridge->getCloud(QString::fromStdString(comp_name));
        if (!comp_cloud) throw std::runtime_error("Cloud not found: " + comp_name);

        ct::DistanceParams params;
        params.method = static_cast<ct::DistanceParams::Method>(method);
        params.k_knn = k_knn;
        params.radius = radius;
        params.flip_normals = flip_normals;

        auto result = ct::DistanceCalculator::calculate(ref_cloud, comp_cloud, params);
        if (!result.success) throw std::runtime_error(result.error_msg);

        // Convert distances to numpy array
        auto count = static_cast<py::ssize_t>(result.distances.size());
        auto arr = py::array_t<float>(count);
        auto buf = arr.request();
        std::memcpy(buf.ptr, result.distances.data(), count * sizeof(float));

        py::dict dict;
        dict["distances"] = arr;
        dict["time_ms"] = result.time_ms;
        return dict;
    }, py::arg("ref_name"), py::arg("comp_name"),
       py::arg("method") = 0, py::arg("k_knn") = 6,
       py::arg("radius") = 0.5, py::arg("flip_normals") = false,
       "Calculate distances between two clouds. method: 0=C2C_NEAREST, 1=C2C_KNN_MEAN, 2=C2C_RADIUS_MEAN, 3=C2M_SIGNED, 4=M3C2. Returns dict with 'distances' array");

    // ---- 特征：包围盒 ----

    m.def("bounding_box_aabb", [](const std::string& name) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto box = ct::Features::boundingBoxAABB(cloud);
        py::dict dict;
        dict["center_x"] = static_cast<double>(box.translation.x());
        dict["center_y"] = static_cast<double>(box.translation.y());
        dict["center_z"] = static_cast<double>(box.translation.z());
        dict["width"]  = box.width;
        dict["height"] = box.height;
        dict["depth"]  = box.depth;
        return dict;
    }, py::arg("name"), "Compute axis-aligned bounding box, returns dict with center and dimensions");

    m.def("bounding_box_obb", [](const std::string& name) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto box = ct::Features::boundingBoxOBB(cloud);
        py::dict dict;
        dict["center_x"] = static_cast<double>(box.translation.x());
        dict["center_y"] = static_cast<double>(box.translation.y());
        dict["center_z"] = static_cast<double>(box.translation.z());
        dict["width"]  = box.width;
        dict["height"] = box.height;
        dict["depth"]  = box.depth;
        return dict;
    }, py::arg("name"), "Compute oriented bounding box, returns dict with center and dimensions");

    // ================================================================
    // ================================================================
    // 脚本模式控制
    // ================================================================

    m.def("set_script_mode", [](bool enabled) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setScriptMode(enabled);
    }, py::arg("enabled"),
       "Enable script mode: skip file dialogs (field mapping, global shift), use defaults automatically");

    // ================================================================
    // Phase 3: 高级叠加物 + 视图控制
    // ================================================================

    m.def("add_arrow", [](float x1, float y1, float z1, float x2, float y2, float z2,
                           float r, float g, float b, const std::string& id) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->addArrow(x1, y1, z1, x2, y2, z2, QString::fromStdString(id), r, g, b);
    }, py::arg("x1"), py::arg("y1"), py::arg("z1"),
       py::arg("x2"), py::arg("y2"), py::arg("z2"),
       py::arg("r") = 1.0, py::arg("g") = 1.0, py::arg("b") = 1.0,
       py::arg("id") = "arrow",
       "Add arrow between two 3D points. Color r/g/b in 0.0-1.0");

    m.def("add_polygon", [](const std::string& cloud_id, float r, float g, float b,
                             const std::string& id) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->addPolygonCloud(QString::fromStdString(cloud_id), QString::fromStdString(id), r, g, b);
    }, py::arg("cloud_id"), py::arg("r") = 1.0, py::arg("g") = 1.0, py::arg("b") = 1.0,
       py::arg("id") = "polygon",
       "Add polygon (convex hull) for a cloud. Color r/g/b in 0.0-1.0");

    m.def("set_shape_color", [](const std::string& id, float r, float g, float b) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeColor(QString::fromStdString(id), r, g, b);
    }, py::arg("id"), py::arg("r") = 1.0, py::arg("g") = 1.0, py::arg("b") = 1.0,
       "Set shape color. r/g/b in 0.0-1.0");

    m.def("set_shape_size", [](const std::string& id, float size) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeSize(QString::fromStdString(id), size);
    }, py::arg("id"), py::arg("size"), "Set shape point size");

    m.def("set_shape_opacity", [](const std::string& id, float value) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeOpacity(QString::fromStdString(id), value);
    }, py::arg("id"), py::arg("value"), "Set shape opacity (0.0-1.0)");

    m.def("set_shape_line_width", [](const std::string& id, float value) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeLineWidth(QString::fromStdString(id), value);
    }, py::arg("id"), py::arg("value"), "Set shape line width");

    m.def("set_shape_font_size", [](const std::string& id, float value) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeFontSize(QString::fromStdString(id), value);
    }, py::arg("id"), py::arg("value"), "Set shape font size");

    m.def("set_shape_representation", [](const std::string& id, int type) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setShapeRepresentation(QString::fromStdString(id), type);
    }, py::arg("id"), py::arg("type"),
       "Set shape representation: 0=points, 1=wireframe, 2=surface");

    m.def("zoom_to_bounds_xyz", [](float min_x, float min_y, float min_z,
                                    float max_x, float max_y, float max_z) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->zoomToBoundsXYZ(min_x, min_y, min_z, max_x, max_y, max_z);
    }, py::arg("min_x"), py::arg("min_y"), py::arg("min_z"),
       py::arg("max_x"), py::arg("max_y"), py::arg("max_z"),
       "Zoom camera to specific bounding box");

    m.def("invalidate_cloud_render", [](const std::string& id) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->invalidateCloudRender(QString::fromStdString(id));
    }, py::arg("id"), "Force re-render a specific cloud");

    m.def("set_interactor_enable", [](bool enable) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setInteractorEnable(enable);
    }, py::arg("enable"), "Enable or disable mouse interaction");

    // ================================================================
    // Phase 3: 配准算法
    // ================================================================

    // --- 辅助：构建基础 RegistrationContext ---
    auto makeRegContext = [](const std::string& src_name, const std::string& tgt_name,
                             int max_iter, double corr_dist) -> ct::RegistrationContext {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto src = bridge->getCloud(QString::fromStdString(src_name));
        if (!src) throw std::runtime_error("Source cloud not found: " + src_name);
        auto tgt = bridge->getCloud(QString::fromStdString(tgt_name));
        if (!tgt) throw std::runtime_error("Target cloud not found: " + tgt_name);
        ct::RegistrationContext ctx;
        ctx.source_cloud = src;
        ctx.target_cloud = tgt;
        ctx.params.max_iterations = max_iter;
        ctx.params.distance_threshold = corr_dist;
        return ctx;
    };

    // --- 辅助：处理 RegistrationResult → py::dict ---
    auto regResultToDict = [](const ct::RegistrationResult& result) -> py::object {
        if (!result.success) return py::none();
        // Insert aligned cloud into scene
        auto* bridge = ct::PythonManager::instance().bridge();
        result.aligned_cloud->makeAdaptive();
        bridge->registerCloud(result.aligned_cloud);
        bridge->holdCloud(result.aligned_cloud);
        bridge->insertCloud(result.aligned_cloud);

        py::dict dict;
        dict["aligned"] = py::cast(PyCloud(result.aligned_cloud));
        dict["score"] = result.score;
        dict["time_ms"] = result.time_ms;
        // Matrix as list of 4 lists of 4 floats
        py::list rows;
        for (int i = 0; i < 4; ++i) {
            py::list row;
            for (int j = 0; j < 4; ++j)
                row.append(static_cast<double>(result.matrix(i, j)));
            rows.append(row);
        }
        dict["matrix"] = rows;
        return dict;
    };

    m.def("icp", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                      int max_iter, double corr_dist,
                                                      bool use_reciprocal) -> py::object {
        auto ctx = makeRegContext(src, tgt, max_iter, corr_dist);
        auto result = ct::Registration::IterativeClosestPoint(ctx, use_reciprocal);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("max_iterations") = 50, py::arg("correspondence_distance") = 1.0,
       py::arg("use_reciprocal") = false,
       "ICP registration. Returns dict with 'aligned', 'score', 'matrix', 'time_ms' or None");

    m.def("icp_with_normals", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                                    int max_iter, double corr_dist,
                                                                    bool use_reciprocal,
                                                                    bool use_symmetric,
                                                                    bool enforce_same_direction) -> py::object {
        auto ctx = makeRegContext(src, tgt, max_iter, corr_dist);
        auto result = ct::Registration::IterativeClosestPointWithNormals(
            ctx, use_reciprocal, use_symmetric, enforce_same_direction);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("max_iterations") = 50, py::arg("correspondence_distance") = 1.0,
       py::arg("use_reciprocal") = false, py::arg("use_symmetric") = false,
       py::arg("enforce_same_direction") = false,
       "ICP with normals registration. Returns dict or None");

    m.def("icp_nonlinear", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                                int max_iter, double corr_dist,
                                                                bool use_reciprocal) -> py::object {
        auto ctx = makeRegContext(src, tgt, max_iter, corr_dist);
        auto result = ct::Registration::IterativeClosestPointNonLinear(ctx, use_reciprocal);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("max_iterations") = 50, py::arg("correspondence_distance") = 1.0,
       py::arg("use_reciprocal") = false,
       "Non-linear ICP registration. Returns dict or None");

    m.def("gicp", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                       int max_iter, int k, double tra_tol,
                                                       double rol_tol, bool use_reciprocal) -> py::object {
        auto ctx = makeRegContext(src, tgt, max_iter, std::sqrt(std::numeric_limits<double>::max()));
        auto result = ct::Registration::GeneralizedIterativeClosestPoint(
            ctx, k, max_iter, tra_tol, rol_tol, use_reciprocal);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("max_iterations") = 200, py::arg("k") = 30,
       py::arg("translation_tolerance") = 1e-6, py::arg("rotation_tolerance") = 1e-6,
       py::arg("use_reciprocal") = false,
       "Generalized ICP registration. Returns dict or None");

    m.def("ndt", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                      float resolution, double step_size,
                                                      double outlier_ratio) -> py::object {
        auto ctx = makeRegContext(src, tgt, 35, std::sqrt(std::numeric_limits<double>::max()));
        auto result = ct::Registration::NormalDistributionsTransform(ctx, resolution, step_size, outlier_ratio);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("resolution") = 1.0, py::arg("step_size") = 0.1, py::arg("outlier_ratio") = 0.05,
       "Normal Distributions Transform registration. Returns dict or None");

    m.def("fpcs", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                       float delta, float approx_overlap,
                                                       float score_threshold, int nr_samples,
                                                       float max_norm_diff, int max_runtime) -> py::object {
        auto ctx = makeRegContext(src, tgt, 0, std::sqrt(std::numeric_limits<double>::max()));
        auto result = ct::Registration::FPCSInitialAlignment(
            ctx, delta, true, approx_overlap, score_threshold, nr_samples, max_norm_diff, max_runtime);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("delta") = 1.0, py::arg("approx_overlap") = 0.1,
       py::arg("score_threshold") = 0.6, py::arg("nr_samples") = 3000,
       py::arg("max_norm_diff") = 0.1, py::arg("max_runtime") = 60,
       "FPCS initial alignment. Returns dict or None");

    m.def("kfpcs", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                        float delta, float approx_overlap,
                                                        float score_threshold, int nr_samples,
                                                        float max_norm_diff, int max_runtime,
                                                        float upper_trl, float lower_trl,
                                                        float lambda) -> py::object {
        auto ctx = makeRegContext(src, tgt, 0, std::sqrt(std::numeric_limits<double>::max()));
        auto result = ct::Registration::KFPCSInitialAlignment(
            ctx, delta, true, approx_overlap, score_threshold, nr_samples,
            max_norm_diff, max_runtime, upper_trl, lower_trl, lambda);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("delta") = 1.0, py::arg("approx_overlap") = 0.1,
       py::arg("score_threshold") = 0.6, py::arg("nr_samples") = 3000,
       py::arg("max_norm_diff") = 0.1, py::arg("max_runtime") = 60,
       py::arg("upper_trl_boundary") = 2.0, py::arg("lower_trl_boundary") = 0.05,
       py::arg("lambda") = 0.5,
       "KFPCS initial alignment. Returns dict or None");

    // ================================================================
    // Phase 1: 核心 Python API — 点云管理扩展
    // ================================================================

    // --- 就地更新点云数据（Phase 3） ---
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

    // --- 获取所有已注册点云的名称列表 ---
    m.def("get_all_cloud_names", []() -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) return py::list();
        auto names = bridge->getCloudNames();
        py::list result;
        for (const auto& n : names)
            result.append(py::cast(n.toStdString()));
        return result;
    }, "Get names of all registered clouds");

    // --- 按名称移除单个点云 ---
    m.def("remove_cloud", [](const std::string& name) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) {
            throw std::runtime_error("Cloud not found: " + name);
        }
        bridge->unregisterCloud(QString::fromStdString(name));
        bridge->removeCloud(QString::fromStdString(name));
    }, py::arg("name"), "Remove a cloud by name");

    // --- 移除所有点云 ---
    m.def("remove_all_clouds", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        auto names = bridge->getCloudNames();
        for (const auto& n : names)
            bridge->unregisterCloud(n);
        bridge->removeAllClouds();
    }, "Remove all clouds from the scene");

    // --- 克隆点云 ---
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

    // --- 合并多个点云 ---
    m.def("merge_clouds", [](py::list name_list) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        if (name_list.size() < 2)
            throw std::runtime_error("Need at least 2 clouds to merge");

        // Collect clouds
        std::vector<ct::Cloud::Ptr> clouds;
        for (auto item : name_list) {
            std::string n = py::cast<std::string>(item);
            auto c = bridge->getCloud(QString::fromStdString(n));
            if (!c) throw std::runtime_error("Cloud not found: " + n);
            clouds.push_back(c);
        }

        // Merge: append all into a new cloud
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

        // Compute global bounding box
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

        // Copy points from each cloud
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

    // --- 选中点云 ---
    m.def("select_cloud", [](const std::string& name) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        bridge->selectCloud(QString::fromStdString(name));
    }, py::arg("name"), "Select a cloud by name in the tree");

    // --- 加载点云文件 ---
    m.def("load_cloud", [](const std::string& filepath) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        bridge->loadCloud(QString::fromStdString(filepath));
        return py::none();  // async load, use get_all_cloud_names() to check
    }, py::arg("filepath"), "Load a point cloud file into the scene (async)");

    // --- 保存点云到文件 ---
    m.def("save_cloud", [](const std::string& name, const std::string& filepath, bool binary) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (!bridge) throw std::runtime_error("Python bridge not initialized");
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        bridge->saveCloud(QString::fromStdString(name), QString::fromStdString(filepath), binary);
    }, py::arg("name"), py::arg("filepath"), py::arg("binary") = true,
       "Save a cloud to file. binary=True for binary, False for ASCII (async)");
}
