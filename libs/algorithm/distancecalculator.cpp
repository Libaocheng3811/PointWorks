//
// Distance calculator implementations.
//

#include "distancecalculator.h"

#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <omp.h>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <random>

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

    // Debug log helper
    static FILE* m3c2_log() {
        static FILE* f = fopen("m3c2_debug.log", "a");
        return f;
    }
    #define M3C2_DBG(fmt, ...) do { if(auto*_f=m3c2_log()){fprintf(_f,fmt,__VA_ARGS__);fflush(_f);} } while(0)

    // ================================================================
    // Internal helpers: M3C2
    // ================================================================

    static double estimateMeanPointSpacing(const pcl::PointCloud<PointXYZ>::Ptr& cloud,
                                            size_t sample_count = 10000) {
        if (cloud->empty()) return 1.0;

        size_t n = std::min(cloud->size(), sample_count);
        std::vector<size_t> indices(cloud->size());
        std::iota(indices.begin(), indices.end(), 0);
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(indices.begin(), indices.end(), g);
        indices.resize(n);

        pcl::search::KdTree<PointXYZ> tree;
        tree.setInputCloud(cloud);

        double total_dist = 0.0;
        size_t valid = 0;
        for (size_t idx : indices) {
            std::vector<int> ki(2);
            std::vector<float> sd(2);
            if (tree.nearestKSearch(cloud->points[idx], 2, ki, sd) >= 2) {
                total_dist += std::sqrt(sd[1]);
                valid++;
            }
        }
        return (valid > 0) ? (total_dist / valid) : 1.0;
    }

    static void estimateNormalsManual(
        const pcl::PointCloud<PointXYZ>::Ptr& cloud,
        double radius,
        pcl::PointCloud<pcl::Normal>::Ptr& out_normals,
        std::atomic<bool>* cancel,
        const std::function<void(int)>& on_progress) {

        M3C2_DBG("estimateNormalsManual: n=%zu radius=%.4f\n", cloud->size(), radius);

        size_t n = cloud->size();
        out_normals.reset(new pcl::PointCloud<pcl::Normal>);
        out_normals->width = n;
        out_normals->height = 1;
        out_normals->points.resize(n, pcl::Normal());

        if (n == 0) return;

        M3C2_DBG("estimateNormalsManual: building KdTree...\n");
        pcl::search::KdTree<PointXYZ> tree;
        tree.setInputCloud(cloud);
        M3C2_DBG("estimateNormalsManual: KdTree built, starting loop\n");

        if (on_progress) on_progress(30);
        if (cancel && cancel->load()) return;

        size_t report_interval = n / 50 + 1;
        size_t log_interval = n / 10 + 1;

        for (size_t i = 0; i < n; ++i) {
            if (cancel && cancel->load()) return;

            std::vector<int> indices;
            std::vector<float> sqr_dists;
            int found = tree.radiusSearch(cloud->points[i], radius, indices, sqr_dists);

            if (found < 3) continue;

            // Compute centroid
            float cx = 0, cy = 0, cz = 0;
            for (int k = 0; k < found; ++k) {
                cx += cloud->points[indices[k]].x;
                cy += cloud->points[indices[k]].y;
                cz += cloud->points[indices[k]].z;
            }
            float inv_n = 1.0f / static_cast<float>(found);
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            // Compute covariance matrix (3x3)
            float cov[9] = {0};
            for (int k = 0; k < found; ++k) {
                float dx = cloud->points[indices[k]].x - cx;
                float dy = cloud->points[indices[k]].y - cy;
                float dz = cloud->points[indices[k]].z - cz;
                cov[0] += dx * dx; cov[1] += dx * dy; cov[2] += dx * dz;
                cov[4] += dy * dy; cov[5] += dy * dz;
                cov[8] += dz * dz;
            }
            cov[3] = cov[1]; cov[6] = cov[2]; cov[7] = cov[5];

            // Solve for eigenvectors
            Eigen::Matrix3f mat;
            mat << cov[0], cov[1], cov[2],
                   cov[3], cov[4], cov[5],
                   cov[6], cov[7], cov[8];
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(mat);
            Eigen::Vector3f normal = solver.eigenvectors().col(0);

            out_normals->points[i].normal_x = normal.x();
            out_normals->points[i].normal_y = normal.y();
            out_normals->points[i].normal_z = normal.z();

            if (i % log_interval == 0) {
                M3C2_DBG("normals: %zu/%zu done\n", i, n);
            }

            if (on_progress && i % report_interval == 0)
                on_progress(30 + static_cast<int>(i * 30 / n));
        }

        M3C2_DBG("estimateNormalsManual DONE\n");
        if (on_progress) on_progress(60);
    }

    static void orientNormalsTowardComp(
        const pcl::PointCloud<PointXYZ>::Ptr& ref_cloud,
        pcl::PointCloud<pcl::Normal>::Ptr& normals,
        const pcl::PointCloud<PointXYZ>::Ptr& comp_cloud) {

        if (!normals || normals->size() != ref_cloud->size()) return;

        Eigen::Vector3d comp_center = Eigen::Vector3d::Zero();
        size_t comp_n = comp_cloud->size();
        if (comp_n == 0) return;

        size_t sample = std::min(comp_n, static_cast<size_t>(10000));
        for (size_t i = 0; i < sample; ++i) {
            const auto& p = comp_cloud->points[i];
            comp_center += Eigen::Vector3d(p.x, p.y, p.z);
        }
        comp_center /= static_cast<double>(sample);

        Eigen::Vector3f cf(comp_center.cast<float>());

        size_t n = ref_cloud->size();
        for (size_t i = 0; i < n; ++i) {
            const auto& p = ref_cloud->points[i];
            float tx = cf.x() - p.x, ty = cf.y() - p.y, tz = cf.z() - p.z;
            float nx = normals->points[i].normal_x;
            float ny = normals->points[i].normal_y;
            float nz = normals->points[i].normal_z;
            if (nx * tx + ny * ty + nz * tz < 0) {
                normals->points[i].normal_x = -nx;
                normals->points[i].normal_y = -ny;
                normals->points[i].normal_z = -nz;
            }
        }
    }

    static void computeM3C2Distances(
        const pcl::PointCloud<PointXYZ>::Ptr& ref_cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& ref_normals,
        const pcl::PointCloud<PointXYZ>::Ptr& comp_cloud,
        double proj_radius, double max_depth_val,
        std::vector<float>& signed_distances,
        std::vector<float>& neighbor_counts,
        std::atomic<bool>* cancel,
        const std::function<void(int)>& on_progress) {

        size_t n = ref_cloud->size();
        if (n == 0 || !ref_normals || ref_normals->size() != n || !comp_cloud || comp_cloud->empty()) return;

        signed_distances.assign(n, std::numeric_limits<float>::quiet_NaN());
        neighbor_counts.assign(n, 0.0f);

        pcl::search::KdTree<PointXYZ> comp_tree;
        comp_tree.setInputCloud(comp_cloud);

        float max_depth = (max_depth_val > 0) ? static_cast<float>(max_depth_val)
                                              : std::numeric_limits<float>::max();
        size_t report_interval = n / 50 + 1;

        for (size_t i = 0; i < n; ++i) {
            if (cancel && cancel->load()) return;

            const float nx = ref_normals->points[i].normal_x;
            const float ny = ref_normals->points[i].normal_y;
            const float nz = ref_normals->points[i].normal_z;
            float nlen = std::sqrt(nx*nx + ny*ny + nz*nz);
            if (nlen < 1e-6f) continue; // skip invalid normals

            // normalize
            float inv_len = 1.0f / nlen;
            float dx = nx * inv_len, dy = ny * inv_len, dz = nz * inv_len;

            const auto& pt = ref_cloud->points[i];

            std::vector<int> indices;
            std::vector<float> sqr_dists;
            comp_tree.radiusSearch(pt, proj_radius, indices, sqr_dists);

            if (indices.empty()) {
                if (on_progress && i % report_interval == 0)
                    on_progress(70 + static_cast<int>(i * 20 / n));
                continue;
            }

            double sum_proj = 0.0;
            int count = 0;

            for (size_t j = 0; j < indices.size(); ++j) {
                const auto& cp = comp_cloud->points[indices[j]];
                float vx = cp.x - pt.x;
                float vy = cp.y - pt.y;
                float vz = cp.z - pt.z;
                float proj = vx*dx + vy*dy + vz*dz;

                if (std::abs(proj) > max_depth) continue;
                sum_proj += proj;
                ++count;
            }

            if (count > 0) {
                signed_distances[i] = static_cast<float>(sum_proj / count);
                neighbor_counts[i] = static_cast<float>(count);
            }

            if (on_progress && i % report_interval == 0)
                on_progress(70 + static_cast<int>(i * 20 / n));
        }

        if (on_progress) on_progress(90);
    }

    static void computeLODValues(
        const pcl::PointCloud<PointXYZ>::Ptr& ref_cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr& ref_normals,
        const pcl::PointCloud<PointXYZ>::Ptr& comp_cloud,
        double proj_radius,
        std::vector<float>& lod_values,
        std::atomic<bool>* cancel,
        const std::function<void(int)>& on_progress) {

        size_t n = ref_cloud->size();
        if (n == 0 || !ref_normals || ref_normals->size() != n) return;

        lod_values.assign(n, std::numeric_limits<float>::quiet_NaN());

        pcl::search::KdTree<PointXYZ> ref_tree;
        ref_tree.setInputCloud(ref_cloud);

        pcl::search::KdTree<PointXYZ> comp_tree;
        comp_tree.setInputCloud(comp_cloud);

        size_t report_interval = n / 50 + 1;

        for (size_t i = 0; i < n; ++i) {
            if (cancel && cancel->load()) return;

            const float nx = ref_normals->points[i].normal_x;
            const float ny = ref_normals->points[i].normal_y;
            const float nz = ref_normals->points[i].normal_z;
            float nlen = std::sqrt(nx*nx + ny*ny + nz*nz);
            if (nlen < 1e-6f) continue;

            float inv_len = 1.0f / nlen;
            float dx = nx * inv_len, dy = ny * inv_len, dz = nz * inv_len;

            const auto& pt = ref_cloud->points[i];

            // sigma_ref
            std::vector<int> ri;
            std::vector<float> rd;
            ref_tree.radiusSearch(pt, proj_radius, ri, rd);
            if (ri.size() < 3) continue;

            float cx = 0, cy = 0, cz = 0;
            for (int idx : ri) {
                cx += ref_cloud->points[idx].x;
                cy += ref_cloud->points[idx].y;
                cz += ref_cloud->points[idx].z;
            }
            float inv_n = 1.0f / static_cast<float>(ri.size());
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            double sum_sq = 0.0;
            for (int idx : ri) {
                float vx = ref_cloud->points[idx].x - cx;
                float vy = ref_cloud->points[idx].y - cy;
                float vz = ref_cloud->points[idx].z - cz;
                float d = vx*dx + vy*dy + vz*dz;
                sum_sq += d * d;
            }
            double sigma_ref = std::sqrt(sum_sq / (ri.size() - 1));

            // sigma_comp
            std::vector<int> ci;
            std::vector<float> cd;
            comp_tree.radiusSearch(pt, proj_radius, ci, cd);
            if (ci.size() < 3) continue;

            float ccx = 0, ccy = 0, ccz = 0;
            for (int idx : ci) {
                ccx += comp_cloud->points[idx].x;
                ccy += comp_cloud->points[idx].y;
                ccz += comp_cloud->points[idx].z;
            }
            float inv_nc = 1.0f / static_cast<float>(ci.size());
            ccx *= inv_nc; ccy *= inv_nc; ccz *= inv_nc;

            double sum_sq_c = 0.0;
            for (int idx : ci) {
                float vx = comp_cloud->points[idx].x - ccx;
                float vy = comp_cloud->points[idx].y - ccy;
                float vz = comp_cloud->points[idx].z - ccz;
                float d = vx*dx + vy*dy + vz*dz;
                sum_sq_c += d * d;
            }
            double sigma_comp = std::sqrt(sum_sq_c / (ci.size() - 1));

            int nr = static_cast<int>(ri.size());
            int nc = static_cast<int>(ci.size());

            lod_values[i] = 1.96f * static_cast<float>(
                std::sqrt(sigma_ref * sigma_ref / nr + sigma_comp * sigma_comp / nc));

            if (on_progress && i % report_interval == 0)
                on_progress(92 + static_cast<int>(i * 8 / n));
        }

        if (on_progress) on_progress(100);
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
    // M3C2 Public API
    // ================================================================

    static M3C2Result calculateM3C2Impl(
            const pcl::PointCloud<PointXYZ>::Ptr& refCloud,
            const pcl::PointCloud<PointXYZ>::Ptr& compCloud,
            const pcl::PointCloud<pcl::Normal>::Ptr& existingNormals,
            const M3C2Params& params,
            std::atomic<bool>* cancel,
            std::function<void(int)> on_progress) {

        if (cancel) cancel->store(false);

        if (!refCloud || !compCloud || refCloud->empty() || compCloud->empty()) {
            M3C2_DBG("EXIT: invalid input\n");
            return {{}, {}, {}, 0, false, "Invalid input clouds"};
        }

        M3C2_DBG("ref=%zu comp=%zu\n", refCloud->size(), compCloud->size());

        pcl::console::TicToc timer;
        timer.tic();

        size_t n_ref = refCloud->size();

        // Step 0: Auto-estimate parameters
        M3C2_DBG("Step0: estimateMeanPointSpacing\n");
        double mean_spacing = estimateMeanPointSpacing(refCloud);
        M3C2_DBG("mean_spacing=%.4f\n", mean_spacing);
        double normal_scale = (params.normal_max_scale > 0) ? params.normal_max_scale : mean_spacing * 5.0;
        double proj_radius = (params.proj_radius > 0) ? params.proj_radius : normal_scale * 2.0;

        // Auto-estimate max depth: use merged bounding box diagonal
        double max_depth = params.max_distance;
        if (max_depth <= 0) {
            Eigen::Vector4f ref_min, ref_max, comp_min, comp_max;
            pcl::getMinMax3D(*refCloud, ref_min, ref_max);
            pcl::getMinMax3D(*compCloud, comp_min, comp_max);
            ref_min = ref_min.cwiseMin(comp_min);
            ref_max = ref_max.cwiseMax(comp_max);
            float diag = (ref_max - ref_min).head<3>().norm();
            max_depth = diag * 0.1;
        }

        if (on_progress) on_progress(5);

        // Step 1: Normal estimation
        M3C2_DBG("Step1: use_existing=%d normals=%p size=%zu\n",
                params.use_existing_normals, (void*)existingNormals.get(),
                existingNormals ? existingNormals->size() : 0);
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        std::vector<float> normals_quality;

        if (params.use_existing_normals && existingNormals && existingNormals->size() == n_ref) {
            M3C2_DBG("Step1: using existing normals\n");
            normals = existingNormals;
            normals_quality.assign(n_ref, 1.0f);
            if (on_progress) on_progress(60);
        } else {
            M3C2_DBG("Step1: estimateNormalsManual radius=%.4f\n", normal_scale);
            estimateNormalsManual(refCloud, normal_scale, normals, cancel, on_progress);
            normals_quality.assign(n_ref, 1.0f);
        }
        M3C2_DBG("Step1 DONE\n");

        if (cancel && cancel->load()) {
            return {{}, {}, {}, 0, false, "M3C2 canceled during normal estimation"};
        }

        // Step 2: Orient normals toward compared cloud
        M3C2_DBG("Step2: orient normals\n");
        orientNormalsTowardComp(refCloud, normals, compCloud);
        M3C2_DBG("Step2 DONE\n");

        if (on_progress) on_progress(70);

        // Step 3: Compute projection distances
        M3C2_DBG("Step3: distances\n");
        std::vector<float> signed_distances;
        std::vector<float> neighbor_counts;
        computeM3C2Distances(refCloud, normals, compCloud,
                              proj_radius, max_depth,
                              signed_distances, neighbor_counts,
                              cancel, on_progress);
        M3C2_DBG("Step3 DONE\n");

        if (cancel && cancel->load()) {
            return {{}, {}, {}, 0, false, "M3C2 canceled during distance computation"};
        }

        // Step 4: Compute LOD
        M3C2_DBG("Step4: LOD\n");
        std::vector<float> lod_values;
        if (params.compute_lod) {
            computeLODValues(refCloud, normals, compCloud,
                              proj_radius, lod_values,
                              cancel, on_progress);
        }
        M3C2_DBG("Step4 DONE, returning success\n");

        if (cancel && cancel->load()) {
            return {{}, {}, {}, 0, false, "M3C2 canceled during LOD computation"};
        }

        return {signed_distances, lod_values, normals_quality,
                static_cast<float>(timer.toc()), true, ""};
    }

    M3C2Result DistanceCalculator::calculateM3C2(
            const pcl::PointCloud<PointXYZ>::Ptr& refCloud,
            const pcl::PointCloud<PointXYZ>::Ptr& compCloud,
            const pcl::PointCloud<pcl::Normal>::Ptr& existingNormals,
            const M3C2Params& params,
            std::atomic<bool>* cancel,
            std::function<void(int)> on_progress) {
        M3C2_DBG(">>> calculateM3C2 ENTRY\n");
        auto result = calculateM3C2Impl(refCloud, compCloud, existingNormals, params, cancel, on_progress);
        M3C2_DBG("<<< calculateM3C2 EXIT success=%d\n", result.success);
        return result;
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
