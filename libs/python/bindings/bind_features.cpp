#include "algorithm/features.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <cstring>

#include "bind_features.h"
#include "bind_core.h"

#undef slots

// 辅助：从 FeatureType 中提取描述子数据为 numpy 数组
// PCL 描述子类型都是 pcl::PointCloud<Descriptor>，每个 point 有 .histogram[] 数组
// 通过 pcl::PCLPointCloud2 泛型提取 float 数据
static py::object extractDescriptor(const ct::FeatureType::Ptr& feature)
{
    if (!feature) return py::none();

    // 按优先级尝试各描述子类型
    pcl::PCLPointCloud2 cloud2;

    if (feature->fpfh) {
        pcl::toPCLPointCloud2(*feature->fpfh, cloud2);
    } else if (feature->shot) {
        pcl::toPCLPointCloud2(*feature->shot, cloud2);
    } else if (feature->shotc) {
        pcl::toPCLPointCloud2(*feature->shotc, cloud2);
    } else if (feature->pfh) {
        pcl::toPCLPointCloud2(*feature->pfh, cloud2);
    } else if (feature->vfh) {
        pcl::toPCLPointCloud2(*feature->vfh, cloud2);
    } else if (feature->esf) {
        pcl::toPCLPointCloud2(*feature->esf, cloud2);
    } else if (feature->usc) {
        pcl::toPCLPointCloud2(*feature->usc, cloud2);
    } else if (feature->sc3d) {
        pcl::toPCLPointCloud2(*feature->sc3d, cloud2);
    } else if (feature->gasd) {
        pcl::toPCLPointCloud2(*feature->gasd, cloud2);
    } else if (feature->gasdc) {
        pcl::toPCLPointCloud2(*feature->gasdc, cloud2);
    } else if (feature->grsd) {
        pcl::toPCLPointCloud2(*feature->grsd, cloud2);
    } else if (feature->rsd) {
        pcl::toPCLPointCloud2(*feature->rsd, cloud2);
    } else if (feature->crh) {
        // CRH 使用 pcl::Histogram<90> 泛型模板，未注册 POINT_CLOUD_REGISTER_POINT_STRUCT
        // 不能用 toPCLPointCloud2，直接从 histogram[] 提取
        int n = static_cast<int>(feature->crh->size());
        if (n == 0) return py::none();
        constexpr int dim = 90;
        auto arr = py::array_t<float>({n, dim});
        auto buf = arr.request();
        float* dst = static_cast<float*>(buf.ptr);
        for (int i = 0; i < n; ++i)
            std::memcpy(dst + i * dim, feature->crh->points[i].histogram, dim * sizeof(float));
        return py::object(arr);
    } else {
        return py::none();
    }

    int total_size = static_cast<int>(cloud2.width * cloud2.height);
    if (total_size == 0) return py::none();

    // 查找描述子字段（非 x, y, z 的字段）
    int field_offset = -1;
    int descriptor_dim = 0;
    for (auto& field : cloud2.fields) {
        if (field.name == "x" || field.name == "y" || field.name == "z")
            continue;
        // 只取第一个非坐标字段
        field_offset = field.offset;
        descriptor_dim = field.count;
        break;
    }

    if (field_offset < 0 || descriptor_dim == 0) return py::none();

    // 提取为 (N, dim) 的 float numpy 数组
    auto arr = py::array_t<float>({total_size, descriptor_dim});
    auto buf = arr.request();
    const uint8_t* data_ptr = static_cast<const uint8_t*>(cloud2.data.data());
    int field_size = descriptor_dim * static_cast<int>(sizeof(float));

    for (int i = 0; i < total_size; ++i) {
        std::memcpy(static_cast<float*>(buf.ptr) + i * descriptor_dim,
                     data_ptr + i * cloud2.point_step + field_offset,
                     field_size);
    }
    return py::object(arr);
}

void registerFeatureBindings(py::module_& m)
{
    // ========== 包围盒（原有） ==========

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

    // ========== 描述子估计（新增） ==========

    m.def("fpfh", [](const std::string& name, int k, double radius,
                     bool surface) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        ct::Cloud::Ptr surface_cloud = surface ? cloud : nullptr;
        auto fr = ct::Features::FPFHEstimation(cloud, k, radius, surface_cloud);

        py::dict dict;
        dict["descriptor"] = extractDescriptor(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("k") = 30,
       py::arg("radius") = 0.05,
       py::arg("surface") = false,
       "Compute FPFH (Fast Point Feature Histograms) descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("shot", [](const std::string& name, float radius,
                     bool surface) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        ct::Cloud::Ptr surface_cloud = surface ? cloud : nullptr;
        auto fr = ct::Features::SHOTEstimation(cloud, nullptr, radius, surface_cloud);

        py::dict dict;
        dict["descriptor"] = extractDescriptor(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("radius") = 0.05f,
       py::arg("surface") = false,
       "Compute SHOT (Signature of Histograms of OrienTations) descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("shot_color", [](const std::string& name, float radius,
                           bool surface) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        ct::Cloud::Ptr surface_cloud = surface ? cloud : nullptr;
        auto fr = ct::Features::SHOTColorEstimation(cloud, nullptr, radius, surface_cloud);

        py::dict dict;
        dict["descriptor"] = extractDescriptor(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("radius") = 0.05f,
       py::arg("surface") = false,
       "Compute SHOT Color descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    // ========== 边界估计（新增） ==========

    m.def("boundary_estimation", [](const std::string& name,
                                     int k, double radius,
                                     double angle) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto result = ct::Features::BoundaryEstimation(cloud, k, radius, angle);
        if (!result) throw std::runtime_error("Boundary estimation produced no result");

        result->setId("boundary-" + name);
        result->makeAdaptive();
        bridge->registerCloud(result);
        bridge->holdCloud(result);
        bridge->insertCloud(result);

        return py::cast(PyCloud(result));
    }, py::arg("name"),
       py::arg("k") = 30,
       py::arg("radius") = 0.05,
       py::arg("angle") = 30.0,
       "Estimate boundary points (requires normals). Returns new ct.Cloud of boundary points.");

    // ========== 局部参考帧（新增） ==========

    m.def("shot_lrf", [](const std::string& name,
                         float radius) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto lr = ct::Features::SHOTLocalReferenceFrameEstimation(cloud, radius);

        py::dict dict;
        dict["time_ms"] = lr.time_ms;

        if (lr.lrf && !lr.lrf->empty()) {
            // LRF (ReferenceFrame) 每个点包含一个 3x3 旋转矩阵
            // 提取为 (N, 3, 3) 的 float numpy 数组
            int n = static_cast<int>(lr.lrf->size());
            auto arr = py::array_t<float>({n, 3, 3});
            auto buf = arr.request();
            float* dst = static_cast<float*>(buf.ptr);
            for (int i = 0; i < n; ++i) {
                const auto& rf = lr.lrf->at(i);
                // 每列是一个轴：x_axis, y_axis, z_axis
                for (int row = 0; row < 3; ++row) {
                    dst[i * 9 + row * 3 + 0] = rf.x_axis[row];
                    dst[i * 9 + row * 3 + 1] = rf.y_axis[row];
                    dst[i * 9 + row * 3 + 2] = rf.z_axis[row];
                }
            }
            dict["lrf"] = py::object(arr);
        } else {
            dict["lrf"] = py::none();
        }

        return dict;
    }, py::arg("name"),
       py::arg("radius") = 0.05f,
       "Compute SHOT Local Reference Frame. Returns dict with 'lrf' (numpy array of shape N x 3 x 3) and 'time_ms'.");
}
