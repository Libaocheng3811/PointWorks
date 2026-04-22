#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace pcl { struct PolygonMesh; }
namespace ct { class Cloud; }

// ========== Data Extraction ==========

std::vector<float> extractMeshVertices(const pcl::PolygonMesh& mesh, size_t& out_count);
std::vector<int32_t> extractMeshFaces(const pcl::PolygonMesh& mesh, size_t& out_count);
size_t meshNumVertices(const pcl::PolygonMesh& mesh);
size_t meshNumFaces(const pcl::PolygonMesh& mesh);

// ========== Surface Reconstruction Algorithms ==========

std::shared_ptr<pcl::PolygonMesh> surfacePoisson(
    const std::shared_ptr<ct::Cloud>& cloud,
    int depth, int min_depth, float point_weight,
    float scale, int solver_divide, int iso_divide,
    float samples_per_node, bool confidence,
    bool output_polygons, bool manifold,
    std::string& error_msg);

std::shared_ptr<pcl::PolygonMesh> surfaceGreedyTriangulation(
    const std::shared_ptr<ct::Cloud>& cloud,
    double mu, int nnn, double radius,
    double min_angle, double max_angle, double ep,
    bool consistent, bool consistent_ordering,
    std::string& error_msg);

std::shared_ptr<pcl::PolygonMesh> surfaceMarchingCubesHoppe(
    const std::shared_ptr<ct::Cloud>& cloud,
    float iso_level, int res_x, int res_y, int res_z,
    float percentage, float dist_ignore,
    std::string& error_msg);

std::shared_ptr<pcl::PolygonMesh> surfaceConvexHull(
    const std::shared_ptr<ct::Cloud>& cloud,
    bool compute_area_volume, int dimension,
    std::string& error_msg);

std::shared_ptr<pcl::PolygonMesh> surfaceConcaveHull(
    const std::shared_ptr<ct::Cloud>& cloud,
    double alpha, bool keep_information, int dimension,
    std::string& error_msg);

std::shared_ptr<pcl::PolygonMesh> surfaceMarchingCubesRBF(
    const std::shared_ptr<ct::Cloud>& cloud,
    float iso_level, int res_x, int res_y, int res_z,
    float percentage, float epsilon,
    std::string& error_msg);

std::shared_ptr<pcl::PolygonMesh> surfaceGridProjection(
    const std::shared_ptr<ct::Cloud>& cloud,
    double resolution, int padding_size, int k, int max_binary_search_level,
    std::string& error_msg);
