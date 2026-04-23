#include "algorithm/filters.h"
#include "bind_filters.h"
#include "bind_core.h"
#include "python_bridge.h"

void registerFilterBindings(py::module_& m)
{
    // 辅助：将 FilterResult 的 result_cloud 插入场景并返回 PyCloud
    auto insertFilterResult = [](const ct::FilterResult& fr, const std::string& base_name) -> py::object {
        if (!fr.result_cloud) throw std::runtime_error("Filter produced no result");
        auto& registry = getRegistry();
        fr.result_cloud->setId(base_name);
        fr.result_cloud->makeAdaptive();
        registry.registerCloud(fr.result_cloud);
        registry.holdCloud(fr.result_cloud);
        if (shouldAutoInsert()) {
            auto* bridge = ct::PythonManager::instance().bridge();
            if (bridge) bridge->insertCloud(fr.result_cloud);
        }
        return py::cast(PyCloud(fr.result_cloud));
    };

    m.def("voxel_grid", [insertFilterResult](const std::string& name, float lx, float ly, float lz,
                           bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::VoxelGrid(cloud, lx, ly, lz, negative);
        return insertFilterResult(fr, "voxel-" + name);
    }, py::arg("name"), py::arg("lx"), py::arg("ly"), py::arg("lz"),
       py::arg("negative") = false,
       "Voxel grid downsampling, returns new ct.Cloud");

    m.def("approx_voxel_grid", [insertFilterResult](const std::string& name, float lx, float ly, float lz,
                                   bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::ApproximateVoxelGrid(cloud, lx, ly, lz, negative);
        return insertFilterResult(fr, "approx_voxel-" + name);
    }, py::arg("name"), py::arg("lx"), py::arg("ly"), py::arg("lz"),
       py::arg("negative") = false,
       "Approximate voxel grid downsampling, returns new ct.Cloud");

    m.def("statistical_outlier_removal", [insertFilterResult](const std::string& name, int nr_k,
                                             double stddev_mult, bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::StatisticalOutlierRemoval(cloud, nr_k, stddev_mult, negative);
        return insertFilterResult(fr, "sor-" + name);
    }, py::arg("name"), py::arg("nr_k") = 30, py::arg("stddev_mult") = 2.0,
       py::arg("negative") = false,
       "Statistical outlier removal, returns new ct.Cloud");

    m.def("radius_outlier_removal", [insertFilterResult](const std::string& name, double radius,
                                        int min_pts, bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::RadiusOutlierRemoval(cloud, radius, min_pts, negative);
        return insertFilterResult(fr, "ror-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("min_pts") = 2,
       py::arg("negative") = false,
       "Radius outlier removal, returns new ct.Cloud");

    m.def("pass_through", [insertFilterResult](const std::string& name, const std::string& field_name,
                              float limit_min, float limit_max, bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::PassThrough(cloud, field_name, limit_min, limit_max, negative);
        return insertFilterResult(fr, "passthrough-" + name);
    }, py::arg("name"), py::arg("field_name"), py::arg("limit_min"), py::arg("limit_max"),
       py::arg("negative") = false,
       "Pass-through filter on a field (x/y/z), returns new ct.Cloud");

    m.def("grid_minimum", [insertFilterResult](const std::string& name, float resolution,
                              bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::GridMinimun(cloud, resolution, negative);
        return insertFilterResult(fr, "gridmin-" + name);
    }, py::arg("name"), py::arg("resolution") = 1.0, py::arg("negative") = false,
       "Grid minimum filter, returns new ct.Cloud");

    m.def("local_maximum", [insertFilterResult](const std::string& name, float radius,
                               bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::LocalMaximum(cloud, radius, negative);
        return insertFilterResult(fr, "localmax-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("negative") = false,
       "Local maximum filter, returns new ct.Cloud");

    m.def("shadow_points", [insertFilterResult](const std::string& name, float threshold,
                               bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::ShadowPoints(cloud, threshold, negative);
        return insertFilterResult(fr, "shadow-" + name);
    }, py::arg("name"), py::arg("threshold") = 0.1, py::arg("negative") = false,
       "Shadow points removal, returns new ct.Cloud");

    m.def("down_sampling", [insertFilterResult](const std::string& name, float radius,
                               bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::DownSampling(cloud, radius, negative);
        return insertFilterResult(fr, "downsample-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("negative") = false,
       "Down sampling, returns new ct.Cloud");

    m.def("uniform_sampling", [insertFilterResult](const std::string& name, float radius,
                                  bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::UniformSampling(cloud, radius, negative);
        return insertFilterResult(fr, "uniform-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("negative") = false,
       "Uniform sampling, returns new ct.Cloud");

    m.def("random_sampling", [insertFilterResult](const std::string& name, int sample, int seed,
                                 bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::RandomSampling(cloud, sample, seed, negative);
        return insertFilterResult(fr, "random-" + name);
    }, py::arg("name"), py::arg("sample") = 1000, py::arg("seed") = 42,
       py::arg("negative") = false,
       "Random sampling, returns new ct.Cloud");

    m.def("resampling", [insertFilterResult](const std::string& name, float radius, int polynomial_order,
                            bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::ReSampling(cloud, radius, polynomial_order, negative);
        return insertFilterResult(fr, "resample-" + name);
    }, py::arg("name"), py::arg("radius") = 1.0, py::arg("polynomial_order") = 2,
       py::arg("negative") = false,
       "MLS resampling, returns new ct.Cloud");

    m.def("normal_space_sampling", [insertFilterResult](const std::string& name, int sample, int seed,
                                       int bin, bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::NormalSpaceSampling(cloud, sample, seed, bin, negative);
        return insertFilterResult(fr, "nss-" + name);
    }, py::arg("name"), py::arg("sample") = 1000, py::arg("seed") = 42,
       py::arg("bin") = 10, py::arg("negative") = false,
       "Normal space sampling, returns new ct.Cloud");

    m.def("sampling_surface_normal", [insertFilterResult](const std::string& name,
                                       int sample, int seed, float ratio,
                                       bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto fr = ct::Filters::SamplingSurfaceNormal(cloud, sample, seed, ratio, negative);
        return insertFilterResult(fr, "ssn-" + name);
    }, py::arg("name"), py::arg("sample") = 10000, py::arg("seed") = 42,
       py::arg("ratio") = 0.5f, py::arg("negative") = false,
       "Sampling using surface normal information (requires normals), returns new ct.Cloud");
}
