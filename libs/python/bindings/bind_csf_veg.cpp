#include "algorithm/csffilter.h"
#include "algorithm/vegfilter.h"
#include "bind_csf_veg.h"
#include "bind_core.h"

void registerCsfVegBindings(py::module_& m)
{
    m.def("csf_filter", [](const std::string& name,
                            bool smooth, float time_step, double class_threshold,
                            double cloth_resolution, int rigidness, int iterations) -> py::dict {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto result = ct::CSFFilter::apply(cloud, smooth, time_step, class_threshold,
                                            cloth_resolution, rigidness, iterations);

        if (result.ground_cloud) {
            result.ground_cloud->setId("ground-" + name);
            result.ground_cloud->makeAdaptive();
            getRegistry().registerCloud(result.ground_cloud);
            getRegistry().holdCloud(result.ground_cloud);
            if (shouldAutoInsert()) {
                auto* bridge = ct::PythonManager::instance().bridge();
                if (bridge) bridge->insertCloud(result.ground_cloud);
            }
        }
        if (result.off_ground_cloud) {
            result.off_ground_cloud->setId("offground-" + name);
            result.off_ground_cloud->makeAdaptive();
            getRegistry().registerCloud(result.off_ground_cloud);
            getRegistry().holdCloud(result.off_ground_cloud);
            if (shouldAutoInsert()) {
                auto* bridge = ct::PythonManager::instance().bridge();
                if (bridge) bridge->insertCloud(result.off_ground_cloud);
            }
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

    m.def("veg_filter", [](const std::string& name, int index_type,
                            double threshold) -> py::dict {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto result = ct::VegetationFilter::apply(cloud, index_type, threshold);

        if (result.veg_cloud) {
            result.veg_cloud->setId("veg-" + name);
            result.veg_cloud->makeAdaptive();
            getRegistry().registerCloud(result.veg_cloud);
            getRegistry().holdCloud(result.veg_cloud);
            if (shouldAutoInsert()) {
                auto* bridge = ct::PythonManager::instance().bridge();
                if (bridge) bridge->insertCloud(result.veg_cloud);
            }
        }
        if (result.non_veg_cloud) {
            result.non_veg_cloud->setId("nonveg-" + name);
            result.non_veg_cloud->makeAdaptive();
            getRegistry().registerCloud(result.non_veg_cloud);
            getRegistry().holdCloud(result.non_veg_cloud);
            if (shouldAutoInsert()) {
                auto* bridge = ct::PythonManager::instance().bridge();
                if (bridge) bridge->insertCloud(result.non_veg_cloud);
            }
        }

        py::dict dict;
        dict["vegetation"] = result.veg_cloud ? py::cast(PyCloud(result.veg_cloud)) : py::none();
        dict["non_vegetation"] = result.non_veg_cloud ? py::cast(PyCloud(result.non_veg_cloud)) : py::none();
        dict["time_ms"] = result.time_ms;
        return dict;
    }, py::arg("name"), py::arg("index_type") = 0,
       py::arg("threshold") = 0.35,
       "Vegetation segmentation. index_type: 0=ExG_ExR, 1=ExG, 2=NGRDI, 3=CIVE. Returns dict with 'vegetation' and 'non_vegetation' clouds");
}
