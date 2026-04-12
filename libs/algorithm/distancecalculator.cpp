//
// Distance calculator implementations.
//

#include "distancecalculator.h"

#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <Eigen/Dense>

#include <omp.h>
#include <cmath>
#include <limits>

namespace ct {

    // ================================================================
    // Internal helpers: C2C
    // ================================================================

    static void computeNearest(const Cloud::Ptr& ref, const Cloud::Ptr& comp,
                               std::vector<float>& dists,
                               std::atomic<bool>* cancel,
                               const std::function<void(int)>& on_progress,
                               double max_distance) {
        auto refCloud = ref->toPCL_XYZ();
        auto compCloud = comp->toPCL_XYZ();
        pcl::search::KdTree<PointXYZ> tree;
        tree.setInputCloud(refCloud);

        const float max_sq = (max_distance > 0) ? static_cast<float>(max_distance * max_distance)
                                                  : std::numeric_limits<float>::max();
        size_t n_points = comp->size();
        int progress_counter = 0;

#pragma omp parallel for schedule(dynamic, 1024)
        for (int i = 0; i < static_cast<int>(n_points); ++i) {
            if (cancel && cancel->load()) continue;

            std::vector<int> indices(1);
            std::vector<float> sqrt_dists(1);

            if (tree.nearestKSearch(compCloud->points[i], 1, indices, sqrt_dists) > 0) {
                float d = std::sqrt(sqrt_dists[0]);
                dists[i] = (d > max_sq) ? std::numeric_limits<float>::quiet_NaN() : d;
            } else {
                dists[i] = std::numeric_limits<float>::quiet_NaN();
            }

#pragma omp atomic
            progress_counter++;
            if (omp_get_thread_num() == 0 && progress_counter % (static_cast<int>(n_points) / 50 + 1) == 0) {
                if (on_progress) on_progress((progress_counter * 100) / static_cast<int>(n_points));
            }
        }
    }

    static void computeKnnMean(const Cloud::Ptr& ref, const Cloud::Ptr& comp, int k,
                               std::vector<float>& dists,
                               std::atomic<bool>* cancel,
                               const std::function<void(int)>& on_progress,
                               double max_distance) {
        if (k < 1) k = 1;
        auto refCloud = ref->toPCL_XYZ();
        auto compCloud = comp->toPCL_XYZ();

        pcl::search::KdTree<PointXYZ> tree;
        tree.setInputCloud(refCloud);

        const float max_d = (max_distance > 0) ? static_cast<float>(max_distance)
                                                  : std::numeric_limits<float>::max();
        size_t n_points = comp->size();
        int progress_counter = 0;

#pragma omp parallel for schedule(dynamic, 1024)
        for (int i = 0; i < static_cast<int>(n_points); ++i) {
            if (cancel && cancel->load()) continue;

            std::vector<int> indices(k);
            std::vector<float> sqr_dists(k);

            int found = tree.nearestKSearch(compCloud->points[i], k, indices, sqr_dists);
            if (found > 0) {
                double sum = 0.0;
                for (float sq : sqr_dists) {
                    float d = std::sqrt(sq);
                    if (d > max_d) { sum = -1; break; }
                    sum += d;
                }
                dists[i] = (sum < 0) ? std::numeric_limits<float>::quiet_NaN()
                                      : static_cast<float>(sum / found);
            } else {
                dists[i] = std::numeric_limits<float>::quiet_NaN();
            }

#pragma omp atomic
            progress_counter++;
            if (omp_get_thread_num() == 0 && progress_counter % (static_cast<int>(n_points) / 50 + 1) == 0) {
                if (on_progress) on_progress((progress_counter * 100) / static_cast<int>(n_points));
            }
        }
    }

    static void computeRadiusMean(const Cloud::Ptr& ref, const Cloud::Ptr& comp, double r,
                                  std::vector<float>& dists,
                                  std::atomic<bool>* cancel,
                                  const std::function<void(int)>& on_progress,
                                  double max_distance) {
        auto refCloud = ref->toPCL_XYZ();
        auto compCloud = comp->toPCL_XYZ();

        pcl::search::KdTree<PointXYZ> tree;
        tree.setInputCloud(refCloud);

        const float max_d = (max_distance > 0) ? static_cast<float>(max_distance)
                                                  : std::numeric_limits<float>::max();
        size_t n_points = comp->size();
        int progress_counter = 0;

#pragma omp parallel for schedule(dynamic, 1024)
        for (int i = 0; i < static_cast<int>(n_points); ++i) {
            if (cancel && cancel->load()) continue;

            std::vector<int> indices;
            std::vector<float> sqr_dists;

            int found = tree.radiusSearch(compCloud->points[i], r, indices, sqr_dists);
            if (found > 0) {
                double sum = 0.0;
                bool exceeded = false;
                for (float sq : sqr_dists) {
                    float d = std::sqrt(sq);
                    if (d > max_d) { exceeded = true; break; }
                    sum += d;
                }
                dists[i] = exceeded ? std::numeric_limits<float>::quiet_NaN()
                                    : static_cast<float>(sum / found);
            } else {
                dists[i] = std::numeric_limits<float>::quiet_NaN();
            }

#pragma omp atomic
            progress_counter++;
            if (omp_get_thread_num() == 0 && progress_counter % (static_cast<int>(n_points) / 50 + 1) == 0) {
                if (on_progress) on_progress((progress_counter * 100) / static_cast<int>(n_points));
            }
        }
    }

    // ================================================================
    // Internal helpers: C2P (analytical distance to geometric primitives)
    // ================================================================

    static float distToPlane(float px, float py, float pz,
                             const C2PParams& params) {
        // |ax + by + cz + d| / sqrt(a²+b²+c²)
        const auto& p = params.plane_params;
        double len = std::sqrt(p.a * p.a + p.b * p.b + p.c * p.c);
        if (len < 1e-12) return std::numeric_limits<float>::quiet_NaN();
        return static_cast<float>(std::abs(p.a * px + p.b * py + p.c * pz + p.d) / len);
    }

    static float distToSphere(float px, float py, float pz,
                              const C2PParams& params) {
        // |dist(P, Center) - R|
        const auto& s = params.sphere_params;
        double dx = px - s.cx, dy = py - s.cy, dz = pz - s.cz;
        return static_cast<float>(std::abs(std::sqrt(dx * dx + dy * dy + dz * dz) - s.radius));
    }

    static float distToCylinder(float px, float py, float pz,
                                const C2PParams& params) {
        // Point-to-axis distance minus radius
        const auto& c = params.cylinder_params;
        // Axis direction (normalized)
        double ax = c.axis_x, ay = c.axis_y, az = c.axis_z;
        double alen = std::sqrt(ax * ax + ay * ay + az * az);
        if (alen < 1e-12) return std::numeric_limits<float>::quiet_NaN();
        ax /= alen; ay /= alen; az /= alen;

        // Vector from axis point to query point
        double dx = px - c.cx, dy = py - c.cy, dz = pz - c.cz;
        // Project onto axis
        double t = dx * ax + dy * ay + dz * az;
        // Perpendicular distance
        double perp_sq = dx * dx + dy * dy + dz * dz - t * t;
        if (perp_sq < 0) perp_sq = 0;
        return static_cast<float>(std::sqrt(perp_sq) - c.radius);
    }

    static float distToCone(float px, float py, float pz,
                            const C2PParams& params) {
        const auto& cn = params.cone_params;
        // Axis direction (normalized)
        double ax = cn.axis_x, ay = cn.axis_y, az = cn.axis_z;
        double alen = std::sqrt(ax * ax + ay * ay + az * az);
        if (alen < 1e-12) return std::numeric_limits<float>::quiet_NaN();
        ax /= alen; ay /= alen; az /= alen;

        // Vector from apex to query point
        double dx = px - cn.apex_x, dy = py - cn.apex_y, dz = pz - cn.apex_z;
        // Projection length along axis
        double t = dx * ax + dy * ay + dz * az;
        if (t < 0) {
            // Point is behind the apex — distance to apex
            return static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        // Perpendicular distance from axis
        double perp_sq = dx * dx + dy * dy + dz * dz - t * t;
        if (perp_sq < 0) perp_sq = 0;
        double perp = std::sqrt(perp_sq);
        // Theoretical radius at projection point
        double R = t * std::tan(cn.half_angle);
        return static_cast<float>(perp - R);
    }

    // ================================================================
    // Public API
    // ================================================================

    DistanceResult DistanceCalculator::calculateC2C(const Cloud::Ptr& ref, const Cloud::Ptr& comp,
                                                     const C2CParams& params,
                                                     std::atomic<bool>* cancel,
                                                     std::function<void(int)> on_progress) {
        if (cancel) cancel->store(false);

        if (!ref || !comp || ref->empty() || comp->empty()) {
            return {{}, 0, false, "Invalid input clouds"};
        }

        pcl::console::TicToc timer;
        timer.tic();

        std::vector<float> distances(comp->size(), std::numeric_limits<float>::quiet_NaN());

        try {
            switch (params.method) {
                case C2CParams::C2C_NEAREST:
                    computeNearest(ref, comp, distances, cancel, on_progress, params.max_distance);
                    break;
                case C2CParams::C2C_KNN_MEAN:
                    computeKnnMean(ref, comp, params.k_knn, distances, cancel, on_progress, params.max_distance);
                    break;
                case C2CParams::C2C_RADIUS_MEAN:
                    computeRadiusMean(ref, comp, params.radius, distances, cancel, on_progress, params.max_distance);
                    break;
                default:
                    return {{}, 0, false, "Invalid C2C method"};
            }
        } catch (const std::exception& e) {
            return {{}, 0, false, std::string("C2C error: ") + e.what()};
        }

        if (cancel && cancel->load()) {
            return {{}, 0, false, "C2C calculation canceled"};
        }

        return {distances, static_cast<float>(timer.toc()), true, ""};
    }

    DistanceResult DistanceCalculator::calculateC2M(const Cloud::Ptr& source,
                                                     const pcl::PolygonMesh::Ptr& target_mesh,
                                                     const C2MParams& params,
                                                     std::atomic<bool>* cancel,
                                                     std::function<void(int)> on_progress) {
        if (cancel) cancel->store(false);

        if (!source || source->empty()) {
            return {{}, 0, false, "Invalid source cloud"};
        }
        if (!target_mesh || target_mesh->polygons.empty()) {
            return {{}, 0, false, "Invalid target mesh"};
        }

        pcl::console::TicToc timer;
        timer.tic();

        // Convert source to PCL for iteration
        auto sourceCloud = source->toPCL_XYZ();
        size_t n_points = sourceCloud->size();
        std::vector<float> distances(n_points, std::numeric_limits<float>::quiet_NaN());

        // Extract mesh vertices as PCL point cloud
        pcl::PointCloud<PointXYZ>::Ptr meshCloud(new pcl::PointCloud<PointXYZ>);
        pcl::fromPCLPointCloud2(target_mesh->cloud, *meshCloud);

        // Build KD-Tree on mesh vertices
        pcl::search::KdTree<PointXYZ> tree;
        tree.setInputCloud(meshCloud);

        // Precompute per-vertex accumulated normals from mesh faces (for signed distance)
        std::vector<Eigen::Vector3f> vertex_normals(meshCloud->size(), Eigen::Vector3f::Zero());
        for (const auto& poly : target_mesh->polygons) {
            if (poly.vertices.size() < 3) continue;
            // Fan triangulation
            for (size_t j = 1; j < poly.vertices.size() - 1; ++j) {
                uint32_t i0 = poly.vertices[0];
                uint32_t i1 = poly.vertices[j];
                uint32_t i2 = poly.vertices[j + 1];
                if (i0 >= meshCloud->size() || i1 >= meshCloud->size() || i2 >= meshCloud->size())
                    continue;
                const auto& p0 = meshCloud->points[i0];
                const auto& p1 = meshCloud->points[i1];
                const auto& p2 = meshCloud->points[i2];
                Eigen::Vector3f e1(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
                Eigen::Vector3f e2(p2.x - p0.x, p2.y - p0.y, p2.z - p0.z);
                Eigen::Vector3f n = e1.cross(e2);
                vertex_normals[i0] += n;
                vertex_normals[i1] += n;
                vertex_normals[i2] += n;
            }
        }
        // Normalize vertex normals
        for (auto& n : vertex_normals) {
            float len = n.norm();
            if (len > 1e-12f) n /= len;
        }

        const double max_d = (params.max_distance > 0) ? params.max_distance : 1e18;
        const float max_sq = static_cast<float>(max_d * max_d);

        int progress_counter = 0;

#pragma omp parallel for schedule(dynamic, 1024)
        for (int i = 0; i < static_cast<int>(n_points); ++i) {
            if (cancel && cancel->load()) continue;

            const auto& pt = sourceCloud->points[i];
            std::vector<int> indices(1);
            std::vector<float> sq_dists(1);

            if (tree.nearestKSearch(pt, 1, indices, sq_dists) > 0) {
                float dist = std::sqrt(sq_dists[0]);
                if (dist > max_d) {
                    distances[i] = std::numeric_limits<float>::quiet_NaN();
                } else if (params.signed_distance) {
                    int idx = indices[0];
                    Eigen::Vector3f n = vertex_normals[idx];
                    if (params.flip_normals) n = -n;
                    Eigen::Vector3f v(pt.x - meshCloud->points[idx].x,
                                     pt.y - meshCloud->points[idx].y,
                                     pt.z - meshCloud->points[idx].z);
                    float sign = v.dot(n);
                    distances[i] = (sign >= 0 ? 1.0f : -1.0f) * dist;
                } else {
                    distances[i] = dist;
                }
            }

#pragma omp atomic
            progress_counter++;
            if (omp_get_thread_num() == 0 && progress_counter % (static_cast<int>(n_points) / 50 + 1) == 0) {
                if (on_progress) on_progress((progress_counter * 100) / static_cast<int>(n_points));
            }
        }

        if (cancel && cancel->load()) {
            return {{}, 0, false, "C2M calculation canceled"};
        }

        return {distances, static_cast<float>(timer.toc()), true, ""};
    }

    DistanceResult DistanceCalculator::calculateC2P(const Cloud::Ptr& source,
                                                     const C2PParams& params,
                                                     std::atomic<bool>* cancel,
                                                     std::function<void(int)> on_progress) {
        if (cancel) cancel->store(false);

        if (!source || source->empty()) {
            return {{}, 0, false, "Invalid source cloud"};
        }

        pcl::console::TicToc timer;
        timer.tic();

        auto sourceCloud = source->toPCL_XYZ();
        size_t n_points = sourceCloud->size();
        std::vector<float> distances(n_points);

        const float max_d = (params.max_distance > 0) ? static_cast<float>(params.max_distance)
                                                        : std::numeric_limits<float>::max();
        int progress_counter = 0;

        // Select distance function based on primitive type
        std::function<float(float, float, float)> distFunc;
        switch (params.primitive_type) {
            case PrimitiveType::PLANE:
                distFunc = [&](float x, float y, float z) { return distToPlane(x, y, z, params); };
                break;
            case PrimitiveType::SPHERE:
                distFunc = [&](float x, float y, float z) { return distToSphere(x, y, z, params); };
                break;
            case PrimitiveType::CYLINDER:
                distFunc = [&](float x, float y, float z) { return distToCylinder(x, y, z, params); };
                break;
            case PrimitiveType::CONE:
                distFunc = [&](float x, float y, float z) { return distToCone(x, y, z, params); };
                break;
            default:
                return {{}, 0, false, "Unknown primitive type"};
        }

#pragma omp parallel for schedule(static)
        for (int i = 0; i < static_cast<int>(n_points); ++i) {
            if (cancel && cancel->load()) continue;

            const auto& pt = sourceCloud->points[i];
            float d = distFunc(pt.x, pt.y, pt.z);
            distances[i] = (d > max_d) ? std::numeric_limits<float>::quiet_NaN() : d;

#pragma omp atomic
            progress_counter++;
            if (omp_get_thread_num() == 0 && progress_counter % (static_cast<int>(n_points) / 50 + 1) == 0) {
                if (on_progress) on_progress((progress_counter * 100) / static_cast<int>(n_points));
            }
        }

        if (cancel && cancel->load()) {
            return {{}, 0, false, "C2P calculation canceled"};
        }

        return {distances, static_cast<float>(timer.toc()), true, ""};
    }

    CPSResult DistanceCalculator::extractClosestPoints(const Cloud::Ptr& source,
                                                        const Cloud::Ptr& target,
                                                        const CPSParams& params,
                                                        std::atomic<bool>* cancel,
                                                        std::function<void(int)> on_progress) {
        if (cancel) cancel->store(false);

        if (!source || !target || source->empty() || target->empty()) {
            return {nullptr, 0, false, "Invalid input clouds"};
        }

        pcl::console::TicToc timer;
        timer.tic();

        auto sourceCloud = source->toPCL_XYZ();
        auto targetCloud = target->toPCL_XYZ();

        // Build KD-Tree on target
        pcl::search::KdTree<PointXYZ> tree;
        tree.setInputCloud(targetCloud);

        size_t n_points = source->size();
        const float max_sq = (params.max_distance > 0)
                                 ? static_cast<float>(params.max_distance * params.max_distance)
                                 : std::numeric_limits<float>::max();

        // Collect projected coordinates
        std::vector<PointXYZ> projected(n_points);
        std::vector<bool> valid(n_points, false);
        int progress_counter = 0;

#pragma omp parallel for schedule(dynamic, 1024)
        for (int i = 0; i < static_cast<int>(n_points); ++i) {
            if (cancel && cancel->load()) continue;

            std::vector<int> indices(1);
            std::vector<float> sqr_dists(1);

            if (tree.nearestKSearch(sourceCloud->points[i], 1, indices, sqr_dists) > 0) {
                if (sqr_dists[0] <= max_sq) {
                    projected[i] = targetCloud->points[indices[0]];
                    valid[i] = true;
                }
            }

#pragma omp atomic
            progress_counter++;
            if (omp_get_thread_num() == 0 && progress_counter % (static_cast<int>(n_points) / 50 + 1) == 0) {
                if (on_progress) on_progress((progress_counter * 100) / static_cast<int>(n_points));
            }
        }

        if (cancel && cancel->load()) {
            return {nullptr, 0, false, "CPS extraction canceled"};
        }

        // Build the result cloud
        auto result = std::make_shared<Cloud>();
        std::vector<PointXYZ> batch_pts;

        for (size_t i = 0; i < n_points; ++i) {
            if (!valid[i]) continue;
            batch_pts.push_back(projected[i]);
        }

        if (batch_pts.empty()) {
            return {nullptr, static_cast<float>(timer.toc()), true, "No points within max distance"};
        }

        // TODO: copy colors / intensity / scalar fields from source if params keep_* is true
        // This requires reading source block data by global index — left for Phase 4 dialog integration

        result->addPoints(batch_pts);
        result->update();

        return {result, static_cast<float>(timer.toc()), true, ""};
    }

    // ================================================================
    // Backward-compatible wrapper (DEPRECATED)
    // ================================================================

    DistanceResult DistanceCalculator::calculate(const Cloud::Ptr& ref, const Cloud::Ptr& comp,
                                                   const DistanceParams& params,
                                                   std::atomic<bool>* cancel,
                                                   std::function<void(int)> on_progress) {
        // Forward to the new C2C interface, mapping legacy fields
        C2CParams c2c_params;
        c2c_params.method = static_cast<C2CParams::Method>(params.method);
        c2c_params.k_knn = params.k_knn;
        c2c_params.radius = params.radius;
        c2c_params.search_method = params.search_method;
        c2c_params.max_distance = params.max_distance;

        return calculateC2C(ref, comp, c2c_params, cancel, on_progress);
    }

} // namespace ct
