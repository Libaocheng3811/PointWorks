#include "algorithm/distancecalculator.h"
#include "core/field_types.h"
#include "bind_distance.h"
#include "bind_core.h"
#include "bind_surface.h"

void registerDistanceBindings(py::module_& m)
{
    // 辅助：DistanceResult → py::dict
    auto distResultToDict = [](const ct::DistanceResult& dr) -> py::dict {
        py::dict dict;
        if (dr.success && !dr.distances.empty()) {
            auto count = static_cast<py::ssize_t>(dr.distances.size());
            auto arr = py::array_t<float>(count);
            auto buf = arr.request();
            std::memcpy(buf.ptr, dr.distances.data(), count * sizeof(float));
            dict["distances"] = arr;
        } else {
            dict["distances"] = py::none();
        }
        dict["time_ms"] = dr.time_ms;
        if (!dr.success) dict["error"] = dr.error_msg;
        return dict;
    };

    // ================================================================
    // 新版 C2C（替代废弃的 calculate_distance）
    // ================================================================
    m.def("cloud_cloud_distance", [distResultToDict](
            const std::string& ref_name, const std::string& comp_name,
            int method, int k_knn, double radius) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto ref = bridge->getCloud(QString::fromStdString(ref_name));
        if (!ref) throw std::runtime_error("Cloud not found: " + ref_name);
        auto comp = bridge->getCloud(QString::fromStdString(comp_name));
        if (!comp) throw std::runtime_error("Cloud not found: " + comp_name);

        ct::C2CParams params;
        params.method = static_cast<ct::C2CParams::Method>(method);
        params.k_knn = k_knn;
        params.radius = radius;

        auto dr = ct::DistanceCalculator::calculateC2C(ref, comp, params);
        return distResultToDict(dr);
    }, py::arg("ref_name"), py::arg("comp_name"),
       py::arg("method") = 0,
       py::arg("k_knn") = 6,
       py::arg("radius") = 0.5,
       "Cloud-to-Cloud distance. method: 0=C2C_NEAREST, 1=C2C_KNN_MEAN, 2=C2C_RADIUS_MEAN. "
       "Returns dict with 'distances' (numpy array), 'time_ms'.");

    // ================================================================
    // C2M：点云对网格有符号距离
    // ================================================================
    m.def("cloud_mesh_distance", [distResultToDict](
            const std::string& cloud_name, const std::string& mesh_name,
            bool signed_distance, bool flip_normals,
            double search_radius) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto source = bridge->getCloud(QString::fromStdString(cloud_name));
        if (!source) throw std::runtime_error("Cloud not found: " + cloud_name);
        auto mesh_cloud = bridge->getCloud(QString::fromStdString(mesh_name));
        if (!mesh_cloud) throw std::runtime_error("Cloud not found: " + mesh_name);

        // TODO: 支持从 ct.Mesh 对象获取 PolygonMesh
        throw std::runtime_error("cloud_mesh_distance: mesh input not yet supported via Python. "
                                  "Use the Cloud-to-Cloud distance as a workaround.");
    }, py::arg("cloud_name"), py::arg("mesh_name"),
       py::arg("signed_distance") = true,
       py::arg("flip_normals") = false,
       py::arg("search_radius") = 0.0,
       "Cloud-to-Mesh signed distance. (NOT YET IMPLEMENTED - mesh input needed)");

    // ================================================================
    // Closest Point Set 提取
    // ================================================================
    m.def("closest_point_set", [](const std::string& source_name,
                                    const std::string& target_name,
                                    double max_distance) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto source = bridge->getCloud(QString::fromStdString(source_name));
        if (!source) throw std::runtime_error("Cloud not found: " + source_name);
        auto target = bridge->getCloud(QString::fromStdString(target_name));
        if (!target) throw std::runtime_error("Cloud not found: " + target_name);

        ct::CPSParams params;
        params.max_distance = max_distance;

        auto cr = ct::DistanceCalculator::extractClosestPoints(source, target, params);
        if (!cr.success) throw std::runtime_error(cr.error_msg);
        if (!cr.projected_cloud) throw std::runtime_error("Closest point set produced no result");

        cr.projected_cloud->setId("cps-" + source_name);
        cr.projected_cloud->makeAdaptive();
        bridge->registerCloud(cr.projected_cloud);
        bridge->holdCloud(cr.projected_cloud);
        if (shouldAutoInsert()) bridge->insertCloud(cr.projected_cloud);

        return py::cast(PyCloud(cr.projected_cloud));
    }, py::arg("source_name"), py::arg("target_name"),
       py::arg("max_distance") = 1.0,
       "Extract closest point set between two clouds. Returns new ct.Cloud of projected points.");

    // ================================================================
    // 废弃接口（保留向后兼容）
    // ================================================================
    m.def("calculate_distance", [distResultToDict](
            const std::string& ref_name,
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
        return distResultToDict(result);
    }, py::arg("ref_name"), py::arg("comp_name"),
       py::arg("method") = 0, py::arg("k_knn") = 6,
       py::arg("radius") = 0.5, py::arg("flip_normals") = false,
       "[DEPRECATED] Use cloud_cloud_distance instead.");
}
