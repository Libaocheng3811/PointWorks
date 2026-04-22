// 本文件 PCL 代码和 pybind11 代码共存
// 关键：先通过 core/cloud.h → cloudtype.h 正确设置 PCL 环境
// 然后 #undef slots，最后 include bind_surface.h 引入 pybind11

#include "core/cloud.h"          // → cloudtype.h: #define _USE_MATH_DEFINES + PCL 基础头文件
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include "algorithm/surface.h"

#undef slots                     // Qt slots 宏与 Python 冲突

#include "bind_surface.h"        // → bind_common.h → pybind11 (在 #undef slots 之后)
#include "mesh_utils.h"

// ==================== PyMesh 方法实现 ====================

PyMesh::PyMesh(std::shared_ptr<pcl::PolygonMesh> mesh) : m_mesh(mesh) {}
PyMesh::~PyMesh() = default;

py::array_t<float> PyMesh::vertices() const
{
    size_t n = 0;
    auto data = extractMeshVertices(*m_mesh, n);
    py::ssize_t rows = static_cast<py::ssize_t>(n);
    py::ssize_t cols = 3;

    py::array_t<float> result({rows, cols});
    auto buf = result.request();
    std::memcpy(buf.ptr, data.data(), data.size() * sizeof(float));
    return result;
}

py::array_t<int32_t> PyMesh::faces() const
{
    size_t n = 0;
    auto data = extractMeshFaces(*m_mesh, n);
    py::ssize_t rows = static_cast<py::ssize_t>(n);
    py::ssize_t cols = 3;

    py::array_t<int32_t> result({rows, cols});
    auto buf = result.request();
    std::memcpy(buf.ptr, data.data(), data.size() * sizeof(int32_t));
    return result;
}

size_t PyMesh::numVertices() const { return meshNumVertices(*m_mesh); }
size_t PyMesh::numFaces() const { return meshNumFaces(*m_mesh); }
std::shared_ptr<pcl::PolygonMesh> PyMesh::meshPtr() const { return m_mesh; }

// ==================== 数据提取 ====================

std::vector<float> extractMeshVertices(const pcl::PolygonMesh& mesh, size_t& out_count)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    out_count = cloud.size();

    std::vector<float> data(out_count * 3);
    for (size_t i = 0; i < out_count; ++i) {
        data[i * 3]     = cloud.points[i].x;
        data[i * 3 + 1] = cloud.points[i].y;
        data[i * 3 + 2] = cloud.points[i].z;
    }
    return data;
}

std::vector<int32_t> extractMeshFaces(const pcl::PolygonMesh& mesh, size_t& out_count)
{
    out_count = mesh.polygons.size();

    std::vector<int32_t> data(out_count * 3);
    for (size_t i = 0; i < out_count; ++i) {
        const auto& poly = mesh.polygons[i];
        data[i * 3]     = static_cast<int32_t>(poly.vertices[0]);
        data[i * 3 + 1] = static_cast<int32_t>(poly.vertices[1]);
        data[i * 3 + 2] = static_cast<int32_t>(poly.vertices[2]);
    }
    return data;
}

size_t meshNumVertices(const pcl::PolygonMesh& mesh)
{
    return mesh.cloud.width * mesh.cloud.height;
}

size_t meshNumFaces(const pcl::PolygonMesh& mesh)
{
    return mesh.polygons.size();
}

// ==================== 曲面重建算法 ====================

std::shared_ptr<pcl::PolygonMesh> surfacePoisson(
    const std::shared_ptr<ct::Cloud>& cloud,
    int depth, int min_depth, float point_weight,
    float scale, int solver_divide, int iso_divide,
    float samples_per_node, bool confidence,
    bool output_polygons, bool manifold,
    std::string& error_msg)
{
    auto sr = ct::Surface::Poisson(cloud, depth, min_depth, point_weight, scale,
                                    solver_divide, iso_divide, samples_per_node,
                                    confidence, output_polygons, manifold);
    if (!sr.mesh && !sr.error_msg.empty()) error_msg = sr.error_msg;
    return sr.mesh;
}

std::shared_ptr<pcl::PolygonMesh> surfaceGreedyTriangulation(
    const std::shared_ptr<ct::Cloud>& cloud,
    double mu, int nnn, double radius,
    double min_angle, double max_angle, double ep,
    bool consistent, bool consistent_ordering,
    std::string& error_msg)
{
    auto sr = ct::Surface::GreedyProjectionTriangulation(
        cloud, mu, nnn, radius, min_angle, max_angle, ep, consistent, consistent_ordering);
    if (!sr.mesh && !sr.error_msg.empty()) error_msg = sr.error_msg;
    return sr.mesh;
}

std::shared_ptr<pcl::PolygonMesh> surfaceMarchingCubesHoppe(
    const std::shared_ptr<ct::Cloud>& cloud,
    float iso_level, int res_x, int res_y, int res_z,
    float percentage, float dist_ignore,
    std::string& error_msg)
{
    auto sr = ct::Surface::MarchingCubesHoppe(
        cloud, iso_level, res_x, res_y, res_z, percentage, dist_ignore);
    if (!sr.mesh && !sr.error_msg.empty()) error_msg = sr.error_msg;
    return sr.mesh;
}

std::shared_ptr<pcl::PolygonMesh> surfaceConvexHull(
    const std::shared_ptr<ct::Cloud>& cloud,
    bool compute_area_volume, int dimension,
    std::string& error_msg)
{
    auto sr = ct::Surface::ConvexHull(cloud, compute_area_volume, dimension);
    if (!sr.mesh && !sr.error_msg.empty()) error_msg = sr.error_msg;
    return sr.mesh;
}

std::shared_ptr<pcl::PolygonMesh> surfaceConcaveHull(
    const std::shared_ptr<ct::Cloud>& cloud,
    double alpha, bool keep_information, int dimension,
    std::string& error_msg)
{
    auto sr = ct::Surface::ConcaveHull(cloud, alpha, keep_information, dimension);
    if (!sr.mesh && !sr.error_msg.empty()) error_msg = sr.error_msg;
    return sr.mesh;
}

std::shared_ptr<pcl::PolygonMesh> surfaceMarchingCubesRBF(
    const std::shared_ptr<ct::Cloud>& cloud,
    float iso_level, int res_x, int res_y, int res_z,
    float percentage, float epsilon,
    std::string& error_msg)
{
    auto sr = ct::Surface::MarchingCubesRBF(
        cloud, iso_level, res_x, res_y, res_z, percentage, epsilon);
    if (!sr.mesh && !sr.error_msg.empty()) error_msg = sr.error_msg;
    return sr.mesh;
}

std::shared_ptr<pcl::PolygonMesh> surfaceGridProjection(
    const std::shared_ptr<ct::Cloud>& cloud,
    double resolution, int padding_size, int k, int max_binary_search_level,
    std::string& error_msg)
{
    auto sr = ct::Surface::GridProjection(
        cloud, resolution, padding_size, k, max_binary_search_level);
    if (!sr.mesh && !sr.error_msg.empty()) error_msg = sr.error_msg;
    return sr.mesh;
}
