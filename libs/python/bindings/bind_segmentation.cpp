#include "algorithm/segmentation.h"
#include "bind_segmentation.h"
#include "bind_core.h"

void registerSegmentationBindings(py::module_& m)
{
    // ---------------------------------------------------------------------------
    // Helper: insert SegmentationResult clouds into scene, return py::list of PyCloud
    // ---------------------------------------------------------------------------
    auto insertSegResult = [](const ct::SegmentationResult& result,
                               const std::string& base_name) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        py::list cloud_list;
        int i = 0;
        for (auto& c : result.clouds) {
            c->setId(base_name + "-" + std::to_string(i++));
            c->makeAdaptive();
            bridge->registerCloud(c);
            bridge->holdCloud(c);
            if (shouldAutoInsert()) bridge->insertCloud(c);
            cloud_list.append(py::cast(PyCloud(c)));
        }
        return cloud_list;
    };

    // Helper: build a py::dict from ModelCoefficients values
    auto coeffToDict = [](const pcl::ModelCoefficients::Ptr& coeff) -> py::dict {
        py::dict d;
        if (coeff) {
            py::list vals;
            for (auto v : coeff->values)
                vals.append(static_cast<double>(v));
            d["values"] = vals;
        }
        return d;
    };

    // Helper: wrap a multi-cloud result into a dict with clouds, time_ms, coefficients
    auto segResultToDict = [insertSegResult, coeffToDict](
        const ct::SegmentationResult& result,
        const std::string& base_name) -> py::dict {
        py::dict d;
        d["clouds"] = insertSegResult(result, base_name);
        d["time_ms"] = static_cast<double>(result.time_ms);
        d["coefficients"] = coeffToDict(result.coefficients);
        return d;
    };

    // ===========================================================================
    // 1. SACSegmentation
    // ===========================================================================
    m.def("sac_segmentation", [segResultToDict](
            const std::string& name, bool negative,
            int model, int method, double threshold,
            int max_iterations, double probability,
            bool optimize, double min_radius, double max_radius) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::SACSegmentation(
            cloud, negative, model, method, threshold,
            max_iterations, probability, optimize, min_radius, max_radius);
        return segResultToDict(result, "sac-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("model") = 0,
       py::arg("method") = 0,
       py::arg("threshold") = 1.0,
       py::arg("max_iterations") = 50,
       py::arg("probability") = 0.99,
       py::arg("optimize") = true,
       py::arg("min_radius") = 0.0,
       py::arg("max_radius") = 0.0,
       "SAC model segmentation. Returns dict with 'clouds', 'time_ms', 'coefficients'.");

    // ===========================================================================
    // 2. SACSegmentationFromNormals
    // ===========================================================================
    m.def("sac_segmentation_from_normals", [segResultToDict](
            const std::string& name, bool negative,
            int model, int method, double threshold,
            int max_iterations, double probability,
            bool optimize, double min_radius, double max_radius,
            double distance_weight, double d) -> py::dict {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::SACSegmentationFromNormals(
            cloud, negative, model, method, threshold,
            max_iterations, probability, optimize, min_radius, max_radius,
            distance_weight, d);
        return segResultToDict(result, "sacnorm-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("model") = 0,
       py::arg("method") = 0,
       py::arg("threshold") = 1.0,
       py::arg("max_iterations") = 50,
       py::arg("probability") = 0.99,
       py::arg("optimize") = true,
       py::arg("min_radius") = 0.0,
       py::arg("max_radius") = 0.0,
       py::arg("distance_weight") = 0.1,
       py::arg("d") = 0.0,
       "SAC segmentation from normals. Returns dict with 'clouds', 'time_ms', 'coefficients'.");

    // ===========================================================================
    // 3. EuclideanClusterExtraction
    // ===========================================================================
    m.def("euclidean_cluster", [insertSegResult](
            const std::string& name, bool negative,
            double tolerance, int min_cluster_size, int max_cluster_size) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::EuclideanClusterExtraction(
            cloud, negative, tolerance, min_cluster_size, max_cluster_size);
        return insertSegResult(result, "euc-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("tolerance") = 1.0,
       py::arg("min_cluster_size") = 1,
       py::arg("max_cluster_size") = std::numeric_limits<int>::max(),
       "Euclidean cluster extraction. Returns list of ct.Cloud.");

    // ===========================================================================
    // 4. DBSCANClusterExtraction
    // ===========================================================================
    m.def("dbscan_cluster", [insertSegResult](
            const std::string& name, bool negative,
            double eps, int min_pts, int min_cluster_size, int max_cluster_size,
            double normal_weight, double color_weight) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::DBSCANClusterExtraction(
            cloud, negative, eps, min_pts, min_cluster_size, max_cluster_size,
            normal_weight, color_weight);
        return insertSegResult(result, "dbscan-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("eps") = 1.0,
       py::arg("min_pts") = 2,
       py::arg("min_cluster_size") = 1,
       py::arg("max_cluster_size") = std::numeric_limits<int>::max(),
       py::arg("normal_weight") = 0.0,
       py::arg("color_weight") = 0.0,
       "DBSCAN clustering. Returns list of ct.Cloud.");

    // ===========================================================================
    // 5. KMeansClusterExtraction
    // ===========================================================================
    m.def("kmeans_cluster", [insertSegResult](
            const std::string& name, int k, int max_iterations,
            double normal_weight, double color_weight) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::KMeansClusterExtraction(
            cloud, k, max_iterations, normal_weight, color_weight);
        return insertSegResult(result, "kmeans-" + name);
    }, py::arg("name"),
       py::arg("k") = 8,
       py::arg("max_iterations") = 100,
       py::arg("normal_weight") = 0.0,
       py::arg("color_weight") = 0.0,
       "K-Means clustering. Returns list of ct.Cloud.");

    // ===========================================================================
    // 6. RegionGrowing
    // ===========================================================================
    m.def("region_growing", [insertSegResult](
            const std::string& name, bool negative,
            int min_cluster_size, int max_cluster_size,
            bool smooth_mode, bool curvature_test, bool residual_test,
            float smoothness_threshold, float residual_threshold,
            float curvature_threshold, int neighbours) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::RegionGrowing(
            cloud, negative, min_cluster_size, max_cluster_size,
            smooth_mode, curvature_test, residual_test,
            smoothness_threshold, residual_threshold,
            curvature_threshold, neighbours);
        return insertSegResult(result, "rg-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("min_cluster_size") = 50,
       py::arg("max_cluster_size") = std::numeric_limits<int>::max(),
       py::arg("smooth_mode") = true,
       py::arg("curvature_test") = false,
       py::arg("residual_test") = false,
       py::arg("smoothness_threshold") = 30.0f,
       py::arg("residual_threshold") = 0.05f,
       py::arg("curvature_threshold") = 0.05f,
       py::arg("neighbours") = 30,
       "Region growing segmentation. Returns list of ct.Cloud.");

    // ===========================================================================
    // 7. RegionGrowingFromSeed
    // ===========================================================================
    m.def("region_growing_from_seed", [insertSegResult](
            const std::string& name, bool negative,
            int seed_index,
            int min_cluster_size, int max_cluster_size,
            bool smooth_mode, bool curvature_test, bool residual_test,
            float smoothness_threshold, float residual_threshold,
            float curvature_threshold, int neighbours) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::RegionGrowingFromSeed(
            cloud, negative, seed_index,
            min_cluster_size, max_cluster_size,
            smooth_mode, curvature_test, residual_test,
            smoothness_threshold, residual_threshold,
            curvature_threshold, neighbours);
        return insertSegResult(result, "rgseed-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("seed_index") = 0,
       py::arg("min_cluster_size") = 50,
       py::arg("max_cluster_size") = std::numeric_limits<int>::max(),
       py::arg("smooth_mode") = true,
       py::arg("curvature_test") = false,
       py::arg("residual_test") = false,
       py::arg("smoothness_threshold") = 30.0f,
       py::arg("residual_threshold") = 0.05f,
       py::arg("curvature_threshold") = 0.05f,
       py::arg("neighbours") = 30,
       "Region growing from seed point. Returns list of ct.Cloud.");

    // ===========================================================================
    // 8. RegionGrowingRGB
    // ===========================================================================
    m.def("region_growing_rgb", [insertSegResult](
            const std::string& name, bool negative,
            int min_cluster_size, int max_cluster_size,
            bool smooth_mode, bool curvature_test, bool residual_test,
            float smoothness_threshold, float residual_threshold,
            float curvature_threshold, int neighbours,
            float pt_thresh, float re_thresh,
            float dis_thresh, int nghbr_number) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::RegionGrowingRGB(
            cloud, negative, min_cluster_size, max_cluster_size,
            smooth_mode, curvature_test, residual_test,
            smoothness_threshold, residual_threshold,
            curvature_threshold, neighbours,
            pt_thresh, re_thresh, dis_thresh, nghbr_number);
        return insertSegResult(result, "rgrgb-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("min_cluster_size") = 50,
       py::arg("max_cluster_size") = std::numeric_limits<int>::max(),
       py::arg("smooth_mode") = true,
       py::arg("curvature_test") = false,
       py::arg("residual_test") = false,
       py::arg("smoothness_threshold") = 30.0f,
       py::arg("residual_threshold") = 0.05f,
       py::arg("curvature_threshold") = 0.05f,
       py::arg("neighbours") = 30,
       py::arg("pt_thresh") = 6.0f,
       py::arg("re_thresh") = 5.0f,
       py::arg("dis_thresh") = 30000.0f,
       py::arg("nghbr_number") = 30,
       "Region growing with RGB color. Returns list of ct.Cloud.");

    // ===========================================================================
    // 9. SupervoxelClustering
    // ===========================================================================
    m.def("supervoxel", [insertSegResult](
            const std::string& name,
            float voxel_resolution, float seed_resolution,
            float color_importance, float spatial_importance,
            float normal_importance, bool camera_transform) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::SupervoxelClustering(
            cloud, voxel_resolution, seed_resolution,
            color_importance, spatial_importance,
            normal_importance, camera_transform);
        return insertSegResult(result, "sv-" + name);
    }, py::arg("name"),
       py::arg("voxel_resolution") = 0.008f,
       py::arg("seed_resolution") = 0.1f,
       py::arg("color_importance") = 0.2f,
       py::arg("spatial_importance") = 0.4f,
       py::arg("normal_importance") = 1.0f,
       py::arg("camera_transform") = false,
       "Supervoxel clustering. Returns list of ct.Cloud.");

    // ===========================================================================
    // 10. DonSegmentation
    // ===========================================================================
    m.def("don_segmentation", [insertSegResult](
            const std::string& name, bool negative,
            double mean_radius, double scale1, double scale2,
            double threshold, double segradius,
            int minClusterSize, int maxClusterSize) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::DonSegmentation(
            cloud, negative, mean_radius, scale1, scale2,
            threshold, segradius, minClusterSize, maxClusterSize);
        return insertSegResult(result, "don-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("mean_radius") = 1.0,
       py::arg("scale1") = 0.5,
       py::arg("scale2") = 2.0,
       py::arg("threshold") = 0.5,
       py::arg("segradius") = 1.5,
       py::arg("minClusterSize") = 50,
       py::arg("maxClusterSize") = std::numeric_limits<int>::max(),
       "Difference of Normals segmentation. Returns list of ct.Cloud.");

    // ===========================================================================
    // 11. MinCutSegmentation
    // ===========================================================================
    m.def("min_cut_segmentation", [insertSegResult](
            const std::string& name,
            double sigma, double radius, double weight,
            int neighbour_number) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::MinCutSegmentation(
            cloud, sigma, radius, weight, neighbour_number);
        return insertSegResult(result, "mincut-" + name);
    }, py::arg("name"),
       py::arg("sigma") = 0.25,
       py::arg("radius") = 0.04,
       py::arg("weight") = 0.8,
       py::arg("neighbour_number") = 14,
       "Min-Cut graph-based segmentation. Returns list of ct.Cloud.");

    // ===========================================================================
    // 12. MorphologicalFilter
    // ===========================================================================
    m.def("morphological_filter", [insertSegResult](
            const std::string& name, bool negative,
            int max_window_size, float slope,
            float max_distance, float initial_distance,
            float cell_size, float base) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::MorphologicalFilter(
            cloud, negative, max_window_size, slope,
            max_distance, initial_distance, cell_size, base);
        return insertSegResult(result, "morph-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("max_window_size") = 33,
       py::arg("slope") = 1.0f,
       py::arg("max_distance") = 3.0f,
       py::arg("initial_distance") = 0.5f,
       py::arg("cell_size") = 1.0f,
       py::arg("base") = 2.0f,
       "Progressive morphological filter for ground segmentation. Returns list of ct.Cloud.");

    // ===========================================================================
    // 13. SeededHueSegmentation
    // ===========================================================================
    m.def("seeded_hue_segmentation", [insertSegResult](
            const std::string& name, bool negative,
            double tolerance, float delta_hue) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto result = ct::Segmentation::SeededHueSegmentation(
            cloud, negative, tolerance, delta_hue);
        return insertSegResult(result, "hue-" + name);
    }, py::arg("name"),
       py::arg("negative") = false,
       py::arg("tolerance") = 5.0,
       py::arg("delta_hue") = 10.0f,
       "Seeded hue-based segmentation. Returns list of ct.Cloud.");

    // ===========================================================================
    // 14. SegmentDifferences  (two clouds)
    // ===========================================================================
    m.def("segment_differences", [insertSegResult](
            const std::string& name, const std::string& tar_name,
            double sqr_threshold) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto tar_cloud = bridge->getCloud(QString::fromStdString(tar_name));
        if (!tar_cloud) throw std::runtime_error("Target cloud not found: " + tar_name);
        auto result = ct::Segmentation::SegmentDifferences(
            cloud, tar_cloud, sqr_threshold);
        return insertSegResult(result, "diff-" + name);
    }, py::arg("name"),
       py::arg("target"),
       py::arg("sqr_threshold") = 0.001,
       "Segment differences between two aligned clouds. Returns list of ct.Cloud.");

    // ===========================================================================
    // 15. ExtractPolygonalPrismData  (two clouds)
    // ===========================================================================
    m.def("extract_polygonal_prism_data", [insertSegResult](
            const std::string& name, const std::string& hull_name,
            bool negative, double height_min, double height_max,
            float vpx, float vpy, float vpz) -> py::list {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        auto hull = bridge->getCloud(QString::fromStdString(hull_name));
        if (!hull) throw std::runtime_error("Hull cloud not found: " + hull_name);
        auto result = ct::Segmentation::ExtractPolygonalPrismData(
            cloud, negative, hull, height_min, height_max, vpx, vpy, vpz);
        return insertSegResult(result, "prism-" + name);
    }, py::arg("name"),
       py::arg("hull"),
       py::arg("negative") = false,
       py::arg("height_min") = 0.0,
       py::arg("height_max") = 10.0,
       py::arg("vpx") = 0.0f,
       py::arg("vpy") = 0.0f,
       py::arg("vpz") = 0.0f,
       "Extract points inside a polygonal prism defined by a hull cloud. Returns list of ct.Cloud.");
}
