#include "algorithm/keypoints.h"
#include "bind_keypoints.h"
#include "bind_core.h"

void registerKeypointBindings(py::module_& m)
{
    // 辅助：将 KeypointResult 的 cloud 插入场景并返回 PyCloud
    auto insertKeypointResult = [](const ct::KeypointResult& kr, const std::string& base_name) -> py::object {
        if (!kr.cloud) throw std::runtime_error("Keypoint detection produced no result");
        auto* bridge = ct::PythonManager::instance().bridge();
        kr.cloud->setId(base_name);
        kr.cloud->makeAdaptive();
        bridge->registerCloud(kr.cloud);
        bridge->holdCloud(kr.cloud);
        bridge->insertCloud(kr.cloud);
        return py::cast(PyCloud(kr.cloud));
    };

    m.def("iss_keypoints", [insertKeypointResult](const std::string& name,
                               double resolution,
                               double gamma_21,
                               double gamma_32,
                               int min_neighbors,
                               float angle,
                               int k,
                               double radius) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto kr = ct::Keypoints::ISSKeypoint3D(cloud, resolution, gamma_21, gamma_32,
                                                 min_neighbors, angle, k, radius);
        return insertKeypointResult(kr, "iss-" + name);
    }, py::arg("name"),
       py::arg("resolution") = 0.1,
       py::arg("gamma_21") = 0.975,
       py::arg("gamma_32") = 0.975,
       py::arg("min_neighbors") = 5,
       py::arg("angle") = 0.52f,
       py::arg("k") = 10,
       py::arg("radius") = 0.1,
       "ISS 3D keypoint detection. Returns new ct.Cloud of keypoints.");

    m.def("harris_keypoints", [insertKeypointResult](const std::string& name,
                                  int response_method,
                                  float threshold,
                                  bool non_maxima,
                                  bool do_refine,
                                  int k,
                                  double radius) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto kr = ct::Keypoints::HarrisKeypoint3D(cloud, response_method, threshold,
                                                    non_maxima, do_refine, k, radius);
        return insertKeypointResult(kr, "harris-" + name);
    }, py::arg("name"),
       py::arg("response_method") = 0,
       py::arg("threshold") = 0.001f,
       py::arg("non_maxima") = true,
       py::arg("do_refine") = false,
       py::arg("k") = 10,
       py::arg("radius") = 0.01,
       "Harris 3D keypoint detection. response_method: 1=HARRIS 2=NOBLE 3=LOWE 4=TOMASI 5=CURVATURE. Returns new ct.Cloud of keypoints.");

    m.def("sift_keypoints", [insertKeypointResult](const std::string& name,
                                 float min_scale,
                                 int nr_octaves,
                                 int nr_scales_per_octave,
                                 float min_contrast,
                                 int k,
                                 double radius) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto kr = ct::Keypoints::SIFTKeypoint(cloud, min_scale, nr_octaves,
                                                nr_scales_per_octave, min_contrast,
                                                k, radius);
        return insertKeypointResult(kr, "sift-" + name);
    }, py::arg("name"),
       py::arg("min_scale") = 0.01f,
       py::arg("nr_octaves") = 6,
       py::arg("nr_scales_per_octave") = 3,
       py::arg("min_contrast") = 0.01f,
       py::arg("k") = 10,
       py::arg("radius") = 0.05,
       "SIFT 3D keypoint detection. Returns new ct.Cloud of keypoints.");
}
