#include "algorithm/normals.h"
#include "bind_normals.h"
#include "bind_core.h"

void registerNormalBindings(py::module_& m)
{
    m.def("estimate_normals", [](const std::string& name,
                                  int k_search, double radius_search,
                                  float vpx, float vpy, float vpz,
                                  bool reverse) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        auto result = ct::Normals::estimate(cloud, k_search, radius_search, vpx, vpy, vpz, reverse);
        if (!result.cloud) throw std::runtime_error(result.error_msg);

        result.cloud->setId("normals-" + name);
        result.cloud->makeAdaptive();
        bridge->registerCloud(result.cloud);
        bridge->holdCloud(result.cloud);
        if (shouldAutoInsert()) bridge->insertCloud(result.cloud);

        return py::cast(PyCloud(result.cloud));
    }, py::arg("name"),
       py::arg("k_search") = 30,
       py::arg("radius_search") = 0.0,
       py::arg("vpx") = 0.0f,
       py::arg("vpy") = 0.0f,
       py::arg("vpz") = 0.0f,
       py::arg("reverse") = false,
       "Estimate normals for a cloud. Use k_search (KNN) or radius_search (radius). Returns new ct.Cloud with normals.");
}
