#pragma once

#include "bind_common.h"

// PyCloud — Python 端的点云访问包装
// 不放在 namespace ct 中，避免与 PYBIND11_EMBEDDED_MODULE(ct, ...) 冲突
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

        auto* holder = new ct::Cloud::Ptr(m_cloud);
        auto capsule = py::capsule(holder, [](void* ptr) {
            delete reinterpret_cast<ct::Cloud::Ptr*>(ptr);
        });

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

    // === 刷新视图 ===

    void refresh()
    {
        m_cloud->update();
        m_cloud->invalidateRenderCache();

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

    ct::Cloud::Ptr cloudPtr() const { return m_cloud; }

private:
    ct::Cloud::Ptr m_cloud;
};

// 注册核心绑定：PyCloud 类 + get_cloud + add_cloud + insert_cloud + remove_selected_clouds
void registerCoreBindings(py::module_& m);
