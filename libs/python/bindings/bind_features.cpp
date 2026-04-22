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
py::object extractDescriptorToPy(const ct::FeatureType::Ptr& feature)
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
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
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
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
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
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
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
        if (shouldAutoInsert()) bridge->insertCloud(result);

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
            int n = static_cast<int>(lr.lrf->size());
            auto arr = py::array_t<float>({n, 3, 3});
            auto buf = arr.request();
            float* dst = static_cast<float*>(buf.ptr);
            for (int i = 0; i < n; ++i) {
                const auto& rf = lr.lrf->at(i);
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

    // ========== 补充缺失的特征描述子 ==========

    m.def("pfh", [](const std::string& name, int k, double radius,
                     bool surface) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        ct::Cloud::Ptr surface_cloud = surface ? cloud : nullptr;
        auto fr = ct::Features::PFHEstimation(cloud, k, radius, surface_cloud);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("k") = 30,
       py::arg("radius") = 0.05,
       py::arg("surface") = false,
       "Compute PFH (Point Feature Histograms) descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("vfh", [](const std::string& name,
                     double dir_x, double dir_y, double dir_z,
                     bool surface) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Vector3f dir(static_cast<float>(dir_x), static_cast<float>(dir_y), static_cast<float>(dir_z));
        ct::Cloud::Ptr surface_cloud = surface ? cloud : nullptr;
        auto fr = ct::Features::VFHEstimation(cloud, dir, surface_cloud);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("dir_x") = 0.0, py::arg("dir_y") = 0.0, py::arg("dir_z") = 0.0,
       py::arg("surface") = false,
       "Compute VFH (Viewpoint Feature Histogram) descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("esf", [](const std::string& name) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto fr = ct::Features::ESFEstimation(cloud);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       "Compute ESF (Ensemble of Shape Functions) descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("gasd", [](const std::string& name,
                      double dir_x, double dir_y, double dir_z,
                      int shgs, int shs, int interp) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Vector3f dir(static_cast<float>(dir_x), static_cast<float>(dir_y), static_cast<float>(dir_z));
        auto fr = ct::Features::GASDEstimation(cloud, dir, shgs, shs, interp);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("dir_x") = 0.0, py::arg("dir_y") = 0.0, py::arg("dir_z") = 0.0,
       py::arg("shgs") = 5, py::arg("shs") = 3, py::arg("interp") = 0,
       "Compute GASD descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("gasd_color", [](const std::string& name,
                            double dir_x, double dir_y, double dir_z,
                            int shgs, int shs, int interp,
                            int chgs, int chs, int cinterp) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Vector3f dir(static_cast<float>(dir_x), static_cast<float>(dir_y), static_cast<float>(dir_z));
        auto fr = ct::Features::GASDColorEstimation(cloud, dir, shgs, shs, interp, chgs, chs, cinterp);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("dir_x") = 0.0, py::arg("dir_y") = 0.0, py::arg("dir_z") = 0.0,
       py::arg("shgs") = 5, py::arg("shs") = 3, py::arg("interp") = 0,
       py::arg("chgs") = 5, py::arg("chs") = 3, py::arg("cinterp") = 0,
       "Compute GASD Color descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("rsd", [](const std::string& name, int nr_subdiv, double plane_radius) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto fr = ct::Features::RSDEstimation(cloud, nr_subdiv, plane_radius);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("nr_subdiv") = 5,
       py::arg("plane_radius") = 0.1,
       "Compute RSD (RoPS) descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("grsd", [](const std::string& name) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto fr = ct::Features::GRSDEstimation(cloud);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       "Compute GRSD descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("crh", [](const std::string& name,
                     double dir_x, double dir_y, double dir_z) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Vector3f dir(static_cast<float>(dir_x), static_cast<float>(dir_y), static_cast<float>(dir_z));
        auto fr = ct::Features::CRHEstimation(cloud, dir);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("dir_x") = 0.0, py::arg("dir_y") = 0.0, py::arg("dir_z") = 0.0,
       "Compute CRH (Camera Roll Histogram) descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("cvfh", [](const std::string& name,
                      double dir_x, double dir_y, double dir_z,
                      float radius_normals, float d1, float d2, float d3,
                      int min_points, bool normalize) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Vector3f dir(static_cast<float>(dir_x), static_cast<float>(dir_y), static_cast<float>(dir_z));
        auto fr = ct::Features::CVFHEstimation(cloud, dir, radius_normals, d1, d2, d3, min_points, normalize);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("dir_x") = 0.0, py::arg("dir_y") = 0.0, py::arg("dir_z") = 0.0,
       py::arg("radius_normals") = 0.05f,
       py::arg("d1") = 0.02f, py::arg("d2") = 0.04f, py::arg("d3") = 0.06f,
       py::arg("min_points") = 50,
       py::arg("normalize") = true,
       "Compute CVFH descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("shape_context_3d", [](const std::string& name,
                                  double min_radius, double radius) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto fr = ct::Features::ShapeContext3DEstimation(cloud, min_radius, radius);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("min_radius") = 0.005,
       py::arg("radius") = 0.05,
       "Compute 3D Shape Context descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("unique_shape_context", [](const std::string& name,
                                      double lrf_radius, double radius,
                                      double loc_radius) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto fr = ct::Features::UniqueShapeContext(cloud, nullptr, lrf_radius, radius, loc_radius);

        py::dict dict;
        dict["descriptor"] = extractDescriptorToPy(fr.feature);
        dict["time_ms"] = fr.time_ms;
        return dict;
    }, py::arg("name"),
       py::arg("lrf_radius") = 0.015,
       py::arg("radius") = 0.025,
       py::arg("loc_radius") = 0.075,
       "Compute Unique Shape Context descriptor. Returns dict with 'descriptor' (numpy array) and 'time_ms'.");

    m.def("board_lrf", [](const std::string& name,
                            float radius, bool find_holes,
                            float margin_thresh, int size,
                            float prob_thresh, float steep_thresh) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto lr = ct::Features::BOARDLocalReferenceFrameEstimation(
            cloud, radius, find_holes, margin_thresh, size, prob_thresh, steep_thresh);

        py::dict dict;
        dict["time_ms"] = lr.time_ms;

        if (lr.lrf && !lr.lrf->empty()) {
            int n = static_cast<int>(lr.lrf->size());
            auto arr = py::array_t<float>({n, 3, 3});
            auto buf = arr.request();
            float* dst = static_cast<float*>(buf.ptr);
            for (int i = 0; i < n; ++i) {
                const auto& rf = lr.lrf->at(i);
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
       py::arg("radius") = 0.03f,
       py::arg("find_holes") = true,
       py::arg("margin_thresh") = 0.001f,
       py::arg("size") = 0,
       py::arg("prob_thresh") = 0.001f,
       py::arg("steep_thresh") = 0.5f,
       "Compute BOARD Local Reference Frame. Returns dict with 'lrf' (numpy array N x 3 x 3) and 'time_ms'.");

    m.def("flare_lrf", [](const std::string& name,
                            float radius, float margin_thresh,
                            int min_neighbors_normal, int min_neighbors_tangent) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto lr = ct::Features::FLARELocalReferenceFrameEstimation(
            cloud, radius, margin_thresh, min_neighbors_normal, min_neighbors_tangent);

        py::dict dict;
        dict["time_ms"] = lr.time_ms;

        if (lr.lrf && !lr.lrf->empty()) {
            int n = static_cast<int>(lr.lrf->size());
            auto arr = py::array_t<float>({n, 3, 3});
            auto buf = arr.request();
            float* dst = static_cast<float*>(buf.ptr);
            for (int i = 0; i < n; ++i) {
                const auto& rf = lr.lrf->at(i);
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
       py::arg("radius") = 0.03f,
       py::arg("margin_thresh") = 0.02f,
       py::arg("min_neighbors_normal") = 5,
       py::arg("min_neighbors_tangent") = 5,
       "Compute FLARE Local Reference Frame. Returns dict with 'lrf' (numpy array N x 3 x 3) and 'time_ms'.");
}
