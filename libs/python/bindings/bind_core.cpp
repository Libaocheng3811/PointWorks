#include "bind_core.h"
#include "python_bridge.h"
#include "bind_surface.h"
#include "mesh_utils.h"

#include <pcl/common/transforms.h>
#include "core/common.h"
#include "algorithm/filters.h"
#include "algorithm/normals.h"
#include "algorithm/features.h"
#include "algorithm/keypoints.h"
#include "bind_features.h"

void registerCoreBindings(py::module_& m)
{
    // --- GUI Console output ---
    m.def("printI", [](const std::string& msg) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->log(0, QString::fromStdString(msg));
    }, "Print info message to GUI Console");

    m.def("printW", [](const std::string& msg) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->log(1, QString::fromStdString(msg));
    }, "Print warning message to GUI Console");

    m.def("printE", [](const std::string& msg) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->log(2, QString::fromStdString(msg));
    }, "Print error message to GUI Console");

    // --- 工厂函数：从 capsule 中提取 Cloud::Ptr 并构造 PyCloud ---
    m.def("_wrap_cloud", [](py::capsule cap) -> py::object {
        auto* cloud_ptr = static_cast<pw::Cloud::Ptr*>(cap);
        if (!cloud_ptr || !*cloud_ptr)
            throw std::runtime_error("Invalid cloud capsule");
        return py::cast(PyCloud(*cloud_ptr));
    }, py::arg("cap"), "Internal: wrap Cloud::Ptr into ct.Cloud");

    // --- 按名称获取点云 ---
    m.def("get_cloud", [](const std::string& name) -> py::object {
        auto& registry = getRegistry();
        auto cloud = registry.getCloud(name);
        if (!cloud) return py::none();

        registry.holdCloud(cloud);
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->markCloudInUse(QString::fromStdString(cloud->id()));

        return py::cast(PyCloud(cloud));
    }, py::arg("name"), "Get cloud by name, returns ct.Cloud or None");

    // --- add_cloud ---
    m.def("add_cloud", [](const std::string& name,
                          py::array_t<float> xyz,
                          py::object colors_obj) -> py::object {
        auto buf = xyz.request();
        if (buf.ndim != 2 || buf.shape[1] != 3)
            throw std::runtime_error("XYZ array must have shape (N, 3)");

        size_t count = static_cast<size_t>(buf.shape[0]);
        if (count == 0) throw std::runtime_error("XYZ array is empty");

        const float* data = static_cast<const float*>(buf.ptr);

        auto cloud = std::make_shared<pw::Cloud>();
        cloud->setId(name);

        auto config = pw::Cloud::calculateAdaptiveConfig(count);
        cloud->setConfig(config);

        float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
        float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;
        for (size_t i = 0; i < count; ++i) {
            float x = data[i*3], y = data[i*3+1], z = data[i*3+2];
            if (x < min_x) min_x = x; if (x > max_x) max_x = x;
            if (y < min_y) min_y = y; if (y > max_y) max_y = y;
            if (z < min_z) min_z = z; if (z > max_z) max_z = z;
        }
        pw::Box globalBox;
        globalBox.translation = Eigen::Vector3f(
            (min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f, (min_z + max_z) * 0.5f);
        globalBox.width  = max_x - min_x;
        globalBox.height = max_y - min_y;
        globalBox.depth  = max_z - min_z;

        cloud->initOctree(globalBox);

        std::vector<pw::PointXYZ> points(count);
        for (size_t i = 0; i < count; ++i) {
            points[i].x = data[i*3];
            points[i].y = data[i*3+1];
            points[i].z = data[i*3+2];
        }

        bool has_colors = !colors_obj.is_none();
        std::vector<pw::ColorRGB> colors;
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

        if (has_colors)
            cloud->addPoints(points, &colors);
        else
            cloud->addPoints(points);

        cloud->makeAdaptive();
        cloud->update();

        getRegistry().registerCloud(cloud);
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->insertCloud(cloud);

        return py::cast(PyCloud(cloud));
    }, py::arg("name"), py::arg("xyz"), py::arg("colors") = py::none(),
       "Create a new cloud from numpy arrays and add to scene");

    m.def("insert_cloud", [](pw::Cloud::Ptr cloud) {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->insertCloud(cloud);
    }, py::arg("cloud"), "Insert a cloud into the tree and view");

    m.def("remove_selected_clouds", []() {
        auto* bridge = pw::PythonManager::instance().bridge();
        if (bridge) bridge->removeSelectedClouds();
    }, "Remove currently selected clouds");

    // --- PyCloud class ---
    py::class_<PyCloud>(m, "Cloud")
        .def(py::init<pw::Cloud::Ptr>())
        // ===== 基础属性 =====
        .def("size", &PyCloud::size, "Total number of points")
        .def("num_blocks", &PyCloud::numBlocks, "Number of data blocks")
        .def("name", &PyCloud::name, "Get cloud name")
        .def("set_name", &PyCloud::setName, "Set cloud name")
        .def("block_size", &PyCloud::blockSize, "Number of points in block[i]")
        .def("block_to_numpy", &PyCloud::blockToNumpy, "Zero-copy NumPy view of block[i] XYZ, shape (M, 3)")
        .def("block_get_colors", &PyCloud::blockGetColors, "Zero-copy NumPy view of block[i] colors, shape (M, 3)")
        .def("block_set_colors", &PyCloud::blockSetColors, "Set block[i] colors from NumPy array (N, 3)")
        .def("block_set_numpy", &PyCloud::blockSetNumpy, "Set block[i] XYZ from NumPy array (N, 3)")
        .def("block_mark_dirty", &PyCloud::blockMarkDirty, "Mark block[i] dirty and recalculate its bounding box")
        .def("refresh", &PyCloud::refresh, "Trigger VTK render update")
        .def("to_numpy", &PyCloud::toNumpy, "Copy all blocks into one contiguous array, shape (N, 3)")
        .def("get_colors", &PyCloud::getColors, "Copy all block colors into one array, shape (N, 3)")
        .def("has_colors", &PyCloud::hasColors, "Check if cloud has color data")
        .def("has_normals", &PyCloud::hasNormals, "Check if cloud has normal data")
        .def("bounding_box", &PyCloud::boundingBox, "Get bounding box as dict {cx, cy, cz, width, height, depth}")
        .def("center", &PyCloud::center, "Get center point as [x, y, z]")
        .def("resolution", &PyCloud::res, "Get point resolution")
        .def("volume", &PyCloud::vol, "Get bounding box volume")
        .def("filepath", &PyCloud::filepath, "Get source file path")
        .def("add_scalar_field", &PyCloud::addScalarField, "Add a scalar field from numpy array", py::arg("name"), py::arg("data"))
        .def("get_scalar_field", &PyCloud::getScalarField, "Get scalar field data as numpy array", py::arg("name"))
        .def("get_scalar_field_names", &PyCloud::getScalarFieldNames, "Get names of all scalar fields")
        .def("remove_scalar_field", &PyCloud::removeScalarField, "Remove a scalar field by name", py::arg("name"))
        .def("clear_scalar_fields", &PyCloud::clearScalarFields, "Remove all scalar fields")
        .def("has_scalar_field", &PyCloud::hasScalarField, "Check if scalar field exists", py::arg("name"))
        .def("update_color_by_field", &PyCloud::updateColorByField, "Colorize cloud by scalar field values", py::arg("field_name"))
        .def("show", &PyCloud::show, py::arg("name") = "", "Add this cloud to the scene tree and view. Optionally set a new name.")

        // ===== 变换（便捷方法） =====
        .def("translate", [](PyCloud& self, double tx, double ty, double tz) -> py::object {
            auto cloud = self.cloudPtr();
            Eigen::Affine3f trans = Eigen::Affine3f::Identity();
            trans.translation() = Eigen::Vector3f(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));
            auto pcl_cloud = cloud->toPCL_XYZRGBN();
            pcl::PointCloud<pw::PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<pw::PointXYZRGBN>);
            pcl::transformPointCloud(*pcl_cloud, *pcl_result, trans);
            auto result = pw::Cloud::fromPCL_XYZRGBN(*pcl_result);
            result->setId("translated-" + cloud->id());
            result->setHasColors(cloud->hasColors());
            result->setHasNormals(cloud->hasNormals());
            result->makeAdaptive();
            getRegistry().registerCloud(result);
            getRegistry().holdCloud(result);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(result); }
            return py::cast(PyCloud(result));
        }, py::arg("tx"), py::arg("ty"), py::arg("tz"),
           "Translate cloud by (tx, ty, tz). Returns new ct.Cloud.")

        .def("rotate", [](PyCloud& self, double rx, double ry, double rz) -> py::object {
            auto cloud = self.cloudPtr();
            Eigen::Affine3f trans = pw::getTransformation(0, 0, 0,
                static_cast<float>(rx), static_cast<float>(ry), static_cast<float>(rz));
            auto pcl_cloud = cloud->toPCL_XYZRGBN();
            pcl::PointCloud<pw::PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<pw::PointXYZRGBN>);
            pcl::transformPointCloud(*pcl_cloud, *pcl_result, trans);
            auto result = pw::Cloud::fromPCL_XYZRGBN(*pcl_result);
            result->setId("rotated-" + cloud->id());
            result->setHasColors(cloud->hasColors());
            result->setHasNormals(cloud->hasNormals());
            result->makeAdaptive();
            getRegistry().registerCloud(result);
            getRegistry().holdCloud(result);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(result); }
            return py::cast(PyCloud(result));
        }, py::arg("rx"), py::arg("ry"), py::arg("rz"),
           "Rotate cloud by Euler angles (degrees). Returns new ct.Cloud.")

        .def("rotate_axis", [](PyCloud& self, double angle, double ax, double ay, double az) -> py::object {
            auto cloud = self.cloudPtr();
            Eigen::Affine3f trans = pw::getTransformation(
                static_cast<float>(angle), static_cast<float>(ax), static_cast<float>(ay), static_cast<float>(az), 0, 0, 0);
            auto pcl_cloud = cloud->toPCL_XYZRGBN();
            pcl::PointCloud<pw::PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<pw::PointXYZRGBN>);
            pcl::transformPointCloud(*pcl_cloud, *pcl_result, trans);
            auto result = pw::Cloud::fromPCL_XYZRGBN(*pcl_result);
            result->setId("rotated-" + cloud->id());
            result->setHasColors(cloud->hasColors());
            result->setHasNormals(cloud->hasNormals());
            result->makeAdaptive();
            getRegistry().registerCloud(result);
            getRegistry().holdCloud(result);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(result); }
            return py::cast(PyCloud(result));
        }, py::arg("angle"), py::arg("ax"), py::arg("ay"), py::arg("az"),
           "Rotate cloud by angle (degrees) around axis. Returns new ct.Cloud.")

        .def("scale", [](PyCloud& self, double sx, double sy, double sz) -> py::object {
            auto cloud = self.cloudPtr();
            Eigen::Affine3f trans = Eigen::Affine3f::Identity();
            trans(0, 0) = static_cast<float>(sx);
            trans(1, 1) = static_cast<float>(sy);
            trans(2, 2) = static_cast<float>(sz);
            auto pcl_cloud = cloud->toPCL_XYZRGBN();
            pcl::PointCloud<pw::PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<pw::PointXYZRGBN>);
            pcl::transformPointCloud(*pcl_cloud, *pcl_result, trans);
            auto result = pw::Cloud::fromPCL_XYZRGBN(*pcl_result);
            result->setId("scaled-" + cloud->id());
            result->setHasColors(cloud->hasColors());
            result->setHasNormals(cloud->hasNormals());
            result->makeAdaptive();
            getRegistry().registerCloud(result);
            getRegistry().holdCloud(result);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(result); }
            return py::cast(PyCloud(result));
        }, py::arg("sx"), py::arg("sy"), py::arg("sz"),
           "Scale cloud by (sx, sy, sz). Returns new ct.Cloud.")

        .def("apply_matrix", [](PyCloud& self, py::list matrix) -> py::object {
            auto cloud = self.cloudPtr();
            if (matrix.size() != 4) throw std::runtime_error("Matrix must have 4 rows");
            Eigen::Affine3f trans = Eigen::Affine3f::Identity();
            for (int i = 0; i < 4; ++i) {
                py::list row = matrix[i].cast<py::list>();
                if (row.size() != 4) throw std::runtime_error("Matrix must have 4 columns");
                for (int j = 0; j < 4; ++j)
                    trans(i, j) = row[j].cast<float>();
            }
            auto pcl_cloud = cloud->toPCL_XYZRGBN();
            pcl::PointCloud<pw::PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<pw::PointXYZRGBN>);
            pcl::transformPointCloud(*pcl_cloud, *pcl_result, trans);
            auto result = pw::Cloud::fromPCL_XYZRGBN(*pcl_result);
            result->setId("transformed-" + cloud->id());
            result->setHasColors(cloud->hasColors());
            result->setHasNormals(cloud->hasNormals());
            result->makeAdaptive();
            getRegistry().registerCloud(result);
            getRegistry().holdCloud(result);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(result); }
            return py::cast(PyCloud(result));
        }, py::arg("matrix"),
           "Apply 4x4 transformation matrix. Returns new ct.Cloud.")

        // ===== 滤波（便捷方法） =====
        .def("voxel_down_sample", [](PyCloud& self, float lx, float ly, float lz) -> py::object {
            auto cloud = self.cloudPtr();
            auto fr = pw::Filters::VoxelGrid(cloud, lx, ly, lz, false);
            if (!fr.result_cloud) throw std::runtime_error("Voxel grid produced no result");
            fr.result_cloud->setId("voxel-" + cloud->id());
            fr.result_cloud->makeAdaptive();
            getRegistry().registerCloud(fr.result_cloud);
            getRegistry().holdCloud(fr.result_cloud);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(fr.result_cloud); }
            return py::cast(PyCloud(fr.result_cloud));
        }, py::arg("lx"), py::arg("ly"), py::arg("lz"),
           "Voxel grid downsampling. Returns new ct.Cloud.")

        .def("remove_outliers", [](PyCloud& self, int nr_k, double stddev_mult) -> py::object {
            auto cloud = self.cloudPtr();
            auto fr = pw::Filters::StatisticalOutlierRemoval(cloud, nr_k, stddev_mult, false);
            if (!fr.result_cloud) throw std::runtime_error("Outlier removal produced no result");
            fr.result_cloud->setId("sor-" + cloud->id());
            fr.result_cloud->makeAdaptive();
            getRegistry().registerCloud(fr.result_cloud);
            getRegistry().holdCloud(fr.result_cloud);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(fr.result_cloud); }
            return py::cast(PyCloud(fr.result_cloud));
        }, py::arg("nr_k") = 30, py::arg("stddev_mult") = 2.0,
           "Statistical outlier removal. Returns new ct.Cloud.")

        .def("crop_by_box", [](PyCloud& self,
                                 double min_x, double min_y, double min_z,
                                 double max_x, double max_y, double max_z) -> py::object {
            auto cloud = self.cloudPtr();
            auto fr_x = pw::Filters::PassThrough(cloud, "x", static_cast<float>(min_x), static_cast<float>(max_x), false);
            if (!fr_x.result_cloud) throw std::runtime_error("Crop produced no result on X axis");
            auto fr_y = pw::Filters::PassThrough(fr_x.result_cloud, "y", static_cast<float>(min_y), static_cast<float>(max_y), false);
            if (!fr_y.result_cloud) throw std::runtime_error("Crop produced no result on Y axis");
            auto fr_z = pw::Filters::PassThrough(fr_y.result_cloud, "z", static_cast<float>(min_z), static_cast<float>(max_z), false);
            if (!fr_z.result_cloud) throw std::runtime_error("Crop produced no result on Z axis");
            fr_z.result_cloud->setId("cropped-" + cloud->id());
            fr_z.result_cloud->makeAdaptive();
            getRegistry().registerCloud(fr_z.result_cloud);
            getRegistry().holdCloud(fr_z.result_cloud);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(fr_z.result_cloud); }
            return py::cast(PyCloud(fr_z.result_cloud));
        }, py::arg("min_x"), py::arg("min_y"), py::arg("min_z"),
           py::arg("max_x"), py::arg("max_y"), py::arg("max_z"),
           "Crop by axis-aligned bounding box. Returns new ct.Cloud.")

        // ===== 法线（便捷方法） =====
        .def("estimate_normals", [](PyCloud& self, int k, double radius) -> py::object {
            auto cloud = self.cloudPtr();
            auto result = pw::Normals::estimate(cloud, k, radius, 0, 0, 0, false);
            if (!result.cloud) throw std::runtime_error(result.error_msg);
            result.cloud->setId("normals-" + cloud->id());
            result.cloud->makeAdaptive();
            getRegistry().registerCloud(result.cloud);
            getRegistry().holdCloud(result.cloud);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(result.cloud); }
            return py::cast(PyCloud(result.cloud));
        }, py::arg("k_search") = 30, py::arg("radius_search") = 0.0,
           "Estimate normals. Returns new ct.Cloud with normals.")

        // ===== 特征（便捷方法） =====
        .def("fpfh", [](PyCloud& self, int k, double radius) -> py::dict {
            auto cloud = self.cloudPtr();
            auto fr = pw::Features::FPFHEstimation(cloud, k, radius, nullptr);
            py::dict dict;
            dict["descriptor"] = extractDescriptorToPy(fr.feature);
            dict["time_ms"] = fr.time_ms;
            return dict;
        }, py::arg("k") = 30, py::arg("radius") = 0.05,
           "Compute FPFH descriptor. Returns dict with 'descriptor' and 'time_ms'.")

        .def("shot", [](PyCloud& self, float radius) -> py::dict {
            auto cloud = self.cloudPtr();
            auto fr = pw::Features::SHOTEstimation(cloud, nullptr, radius, nullptr);
            py::dict dict;
            dict["descriptor"] = extractDescriptorToPy(fr.feature);
            dict["time_ms"] = fr.time_ms;
            return dict;
        }, py::arg("radius") = 0.05f,
           "Compute SHOT descriptor. Returns dict with 'descriptor' and 'time_ms'.")

        .def("boundary_estimation", [](PyCloud& self, int k, double radius, double angle) -> py::object {
            auto cloud = self.cloudPtr();
            auto result = pw::Features::BoundaryEstimation(cloud, k, radius, angle);
            if (!result) throw std::runtime_error("Boundary estimation produced no result");
            result->setId("boundary-" + cloud->id());
            result->makeAdaptive();
            getRegistry().registerCloud(result);
            getRegistry().holdCloud(result);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(result); }
            return py::cast(PyCloud(result));
        }, py::arg("k") = 30, py::arg("radius") = 0.05, py::arg("angle") = 30.0,
           "Estimate boundary points (requires normals). Returns new ct.Cloud.")

        // ===== 曲面重建（便捷方法） =====
        .def("convex_hull", [](PyCloud& self, bool compute_area_volume) -> py::object {
            auto cloud = self.cloudPtr();
            std::string err;
            auto mesh = surfaceConvexHull(cloud, compute_area_volume, 3, err);
            if (!mesh) throw std::runtime_error(err.empty() ? "Convex hull failed" : err);
            return py::cast(PyMesh(mesh));
        }, py::arg("compute_area_volume") = false,
           "Convex hull reconstruction. Returns ct.Mesh.")

        .def("concave_hull", [](PyCloud& self, double alpha) -> py::object {
            auto cloud = self.cloudPtr();
            std::string err;
            auto mesh = surfaceConcaveHull(cloud, alpha, false, 3, err);
            if (!mesh) throw std::runtime_error(err.empty() ? "Concave hull failed" : err);
            return py::cast(PyMesh(mesh));
        }, py::arg("alpha") = 0.1,
           "Concave hull (alpha shape) reconstruction. Returns ct.Mesh.")

        .def("poisson", [](PyCloud& self, int depth, int min_depth, float point_weight,
                            float scale, int solver_divide, int iso_divide,
                            float samples_per_node, bool confidence,
                            bool output_polygons, bool manifold) -> py::object {
            auto cloud = self.cloudPtr();
            std::string err;
            auto mesh = surfacePoisson(cloud, depth, min_depth, point_weight, scale,
                                       solver_divide, iso_divide, samples_per_node,
                                       confidence, output_polygons, manifold, err);
            if (!mesh) throw std::runtime_error(err.empty() ? "Poisson failed" : err);
            return py::cast(PyMesh(mesh));
        }, py::arg("depth") = 8, py::arg("min_depth") = 5,
           py::arg("point_weight") = 4.0f, py::arg("scale") = 1.1f,
           py::arg("solver_divide") = 8, py::arg("iso_divide") = 8,
           py::arg("samples_per_node") = 1.5f,
           py::arg("confidence") = false, py::arg("output_polygons") = false, py::arg("manifold") = false,
           "Poisson surface reconstruction. Returns ct.Mesh.")

        // ===== 关键点（便捷方法） =====
        .def("iss_keypoints", [](PyCloud& self, double resolution,
                                   double gamma_21, double gamma_32,
                                   int min_neighbors, float angle,
                                   int k, double radius) -> py::object {
            auto cloud = self.cloudPtr();
            auto kr = pw::Keypoints::ISSKeypoint3D(cloud, resolution, gamma_21, gamma_32,
                                                     min_neighbors, angle, k, radius);
            if (!kr.cloud) throw std::runtime_error("ISS keypoint detection produced no result");
            kr.cloud->setId("iss-" + cloud->id());
            kr.cloud->makeAdaptive();
            getRegistry().registerCloud(kr.cloud);
            getRegistry().holdCloud(kr.cloud);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(kr.cloud); }
            return py::cast(PyCloud(kr.cloud));
        }, py::arg("resolution") = 0.1,
           py::arg("gamma_21") = 0.975, py::arg("gamma_32") = 0.975,
           py::arg("min_neighbors") = 5, py::arg("angle") = 0.52f,
           py::arg("k") = 10, py::arg("radius") = 0.1,
           "ISS 3D keypoint detection. Returns new ct.Cloud of keypoints.")

        .def("clone", [](PyCloud& self) -> py::object {
            auto cloud = self.cloudPtr();
            auto result = cloud->clone();
            result->setId("clone-" + cloud->id());
            getRegistry().registerCloud(result);
            getRegistry().holdCloud(result);
            if (shouldAutoInsert()) { auto* bridge = pw::PythonManager::instance().bridge(); if (bridge) bridge->insertCloud(result); }
            return py::cast(PyCloud(result));
        }, "Clone this cloud. Returns new ct.Cloud.");
}
