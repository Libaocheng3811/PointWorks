#include "bind_transform.h"
#include "bind_core.h"

#include <pcl/common/transforms.h>
#include "core/common.h"
#include "algorithm/filters.h"

static ct::Cloud::Ptr transformCloud(const ct::Cloud::Ptr& cloud,
                                      const Eigen::Affine3f& trans,
                                      const std::string& output_name)
{
    auto pcl_cloud = cloud->toPCL_XYZRGBN();
    pcl::PointCloud<ct::PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<ct::PointXYZRGBN>);
    pcl::transformPointCloud(*pcl_cloud, *pcl_result, trans);
    auto result = ct::Cloud::fromPCL_XYZRGBN(*pcl_result);
    result->setId(output_name);
    result->setHasColors(cloud->hasColors());
    result->setHasNormals(cloud->hasNormals());
    result->makeAdaptive();
    return result;
}

static py::object insertTransformedCloud(const ct::Cloud::Ptr& result,
                                          const std::string& output_name)
{
    getRegistry().registerCloud(result);
    getRegistry().holdCloud(result);
    if (shouldAutoInsert()) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->insertCloud(result);
    }
    return py::cast(PyCloud(result));
}

void registerTransformBindings(py::module_& m)
{
    // ================================================================
    // 平移
    // ================================================================
    m.def("translate", [](const std::string& name,
                           double tx, double ty, double tz) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        trans.translation() = Eigen::Vector3f(
            static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));

        auto result = transformCloud(cloud, trans, "translated-" + name);
        return insertTransformedCloud(result, result->id());
    }, py::arg("name"), py::arg("tx"), py::arg("ty"), py::arg("tz"),
       "Translate a cloud by (tx, ty, tz). Returns new ct.Cloud.");

    // ================================================================
    // 旋转（欧拉角，度）
    // ================================================================
    m.def("rotate", [](const std::string& name,
                         double rx, double ry, double rz) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Affine3f trans = ct::getTransformation(0, 0, 0,
            static_cast<float>(rx), static_cast<float>(ry), static_cast<float>(rz));

        auto result = transformCloud(cloud, trans, "rotated-" + name);
        return insertTransformedCloud(result, result->id());
    }, py::arg("name"), py::arg("rx"), py::arg("ry"), py::arg("rz"),
       "Rotate a cloud by Euler angles (rx, ry, rz) in degrees. Returns new ct.Cloud.");

    // ================================================================
    // 旋转（轴角）
    // ================================================================
    m.def("rotate_axis", [](const std::string& name,
                              double angle, double ax, double ay, double az,
                              double tx, double ty, double tz) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Affine3f trans = ct::getTransformation(
            static_cast<float>(angle),
            static_cast<float>(ax), static_cast<float>(ay), static_cast<float>(az),
            static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));

        auto result = transformCloud(cloud, trans, "rotated-" + name);
        return insertTransformedCloud(result, result->id());
    }, py::arg("name"), py::arg("angle"), py::arg("ax"), py::arg("ay"), py::arg("az"),
       py::arg("tx") = 0.0, py::arg("ty") = 0.0, py::arg("tz") = 0.0,
       "Rotate a cloud by angle (degrees) around axis (ax, ay, az), optional translation. Returns new ct.Cloud.");

    // ================================================================
    // 缩放
    // ================================================================
    m.def("scale", [](const std::string& name,
                       double sx, double sy, double sz,
                       bool use_center) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        trans(0, 0) = static_cast<float>(sx);
        trans(1, 1) = static_cast<float>(sy);
        trans(2, 2) = static_cast<float>(sz);

        if (use_center) {
            Eigen::Vector3f c = cloud->center();
            Eigen::Affine3f toOrigin = Eigen::Affine3f::Identity();
            toOrigin.translation() = -c;
            Eigen::Affine3f fromOrigin = Eigen::Affine3f::Identity();
            fromOrigin.translation() = c;
            trans = fromOrigin * trans * toOrigin;
        }

        auto result = transformCloud(cloud, trans, "scaled-" + name);
        return insertTransformedCloud(result, result->id());
    }, py::arg("name"), py::arg("sx"), py::arg("sy"), py::arg("sz"),
       py::arg("use_center") = false,
       "Scale a cloud by (sx, sy, sz). use_center=True scales around cloud center. Returns new ct.Cloud.");

    // ================================================================
    // 应用 4x4 矩阵
    // ================================================================
    m.def("apply_matrix", [](const std::string& name,
                               py::list matrix) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        if (matrix.size() != 4)
            throw std::runtime_error("Matrix must have 4 rows");

        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        for (int i = 0; i < 4; ++i) {
            py::list row = matrix[i].cast<py::list>();
            if (row.size() != 4)
                throw std::runtime_error("Matrix must have 4 columns");
            for (int j = 0; j < 4; ++j)
                trans(i, j) = row[j].cast<float>();
        }

        auto result = transformCloud(cloud, trans, "transformed-" + name);
        return insertTransformedCloud(result, result->id());
    }, py::arg("name"), py::arg("matrix"),
       "Apply a 4x4 transformation matrix to a cloud. matrix is a list of 4 lists of 4 floats. Returns new ct.Cloud.");

    // ================================================================
    // 包围盒裁剪
    // ================================================================
    m.def("crop_by_box", [](const std::string& name,
                              double min_x, double min_y, double min_z,
                              double max_x, double max_y, double max_z,
                              bool negative) -> py::object {
        auto cloud = getRegistry().getCloud(name);
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);

        // 使用 PassThrough 滤波器实现三轴裁剪
        // 先裁剪 X，再裁剪 Y，再裁剪 Z
        ct::FilterResult fr_x = ct::Filters::PassThrough(cloud, "x",
            static_cast<float>(min_x), static_cast<float>(max_x), negative);
        if (!fr_x.result_cloud) throw std::runtime_error("Crop produced no result on X axis");

        ct::FilterResult fr_y = ct::Filters::PassThrough(fr_x.result_cloud, "y",
            static_cast<float>(min_y), static_cast<float>(max_y), negative);
        if (!fr_y.result_cloud) throw std::runtime_error("Crop produced no result on Y axis");

        ct::FilterResult fr_z = ct::Filters::PassThrough(fr_y.result_cloud, "z",
            static_cast<float>(min_z), static_cast<float>(max_z), negative);
        if (!fr_z.result_cloud) throw std::runtime_error("Crop produced no result on Z axis");

        fr_z.result_cloud->setId("cropped-" + name);
        fr_z.result_cloud->makeAdaptive();
        getRegistry().registerCloud(fr_z.result_cloud);
        getRegistry().holdCloud(fr_z.result_cloud);
        if (shouldAutoInsert()) {
            auto* bridge = ct::PythonManager::instance().bridge();
            if (bridge) bridge->insertCloud(fr_z.result_cloud);
        }

        return py::cast(PyCloud(fr_z.result_cloud));
    }, py::arg("name"),
       py::arg("min_x"), py::arg("min_y"), py::arg("min_z"),
       py::arg("max_x"), py::arg("max_y"), py::arg("max_z"),
       py::arg("negative") = false,
       "Crop a cloud by axis-aligned bounding box. negative=True keeps points outside the box. Returns new ct.Cloud.");
}
