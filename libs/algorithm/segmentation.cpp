#include <cmath>
#include <random>
#include <numeric>

#include "segmentation.h"

#include <pcl/console/time.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/don.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/segmentation/impl/progressive_morphological_filter.hpp>
#include <pcl/features/impl/don.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/extract_polygonal_prism_data.hpp>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/impl/region_growing_rgb.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle3d.hpp>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <pcl/sample_consensus/impl/sac_model_cone.hpp>
#include <pcl/sample_consensus/impl/sac_model_stick.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
#include <pcl/segmentation/impl/seeded_hue_segmentation.hpp>
#include <pcl/segmentation/impl/segment_differences.hpp>

namespace ct
{

    static inline bool isCanceled(std::atomic<bool>* cancel) {
        return cancel && cancel->load();
    }

    static inline void reportProgress(std::atomic<bool>* cancel,
                                      std::function<void(int)> on_progress,
                                      int pct) {
        if (!isCanceled(cancel) && on_progress) on_progress(pct);
    }

    static std::vector<Cloud::Ptr> extractIndices(
        const pcl::PointCloud<PointXYZRGBN>::Ptr& pcl_cloud,
        const PointIndicesPtr& indices, bool negative)
    {
        std::vector<Cloud::Ptr> segmented_clouds;
        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_segmented(new pcl::PointCloud<PointXYZRGBN>);
        pcl::ExtractIndices<PointXYZRGBN> extract;
        extract.setInputCloud(pcl_cloud);
        extract.setIndices(indices);
        extract.setNegative(negative);
        extract.filter(*pcl_segmented);
        segmented_clouds.push_back(Cloud::fromPCL_XYZRGBN(*pcl_segmented));
        return segmented_clouds;
    }

    static std::vector<Cloud::Ptr> extractClusters(
        const pcl::PointCloud<PointXYZRGBN>::Ptr& pcl_cloud,
        const IndicesClustersPtr& clusters, bool negative)
    {
        std::vector<Cloud::Ptr> segmented_clouds;
        PointIndicesPtr segmented_indices(new PointIndices);
        for (IndicesClusters::const_iterator it = clusters->begin(); it != clusters->end(); ++it)
        {
            Cloud::Ptr cloud_cluster = Cloud::fromPCL_XYZRGBN(
                pcl::PointCloud<PointXYZRGBN>(*pcl_cloud, it->indices));
            cloud_cluster->setId("cluster");
            segmented_clouds.push_back(cloud_cluster);
            segmented_indices->indices.insert(segmented_indices->indices.end(),
                                               it->indices.begin(), it->indices.end());
        }
        if (negative)
            return extractIndices(pcl_cloud, segmented_indices, negative);
        else
            return segmented_clouds;
    }

    SegmentationResult Segmentation::SACSegmentation(const Cloud::Ptr& cloud, bool negative,
                                                     int model, int method, double threshold,
                                                     int max_iterations, double probability,
                                                     bool optimize, double min_radius, double max_radius,
                                                     std::atomic<bool>* cancel,
                                                     std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::PointIndicesPtr indices(new pcl::PointIndices);
        ModelCoefficients::Ptr cofes(new ModelCoefficients);

        pcl::SACSegmentation<PointXYZRGBN> sacseg;
        sacseg.setInputCloud(pcl_cloud);
        sacseg.setModelType(model);
        sacseg.setMethodType(method);
        sacseg.setDistanceThreshold(threshold);
        sacseg.setMaxIterations(max_iterations);
        sacseg.setOptimizeCoefficients(optimize);
        sacseg.setRadiusLimits(min_radius, max_radius);
        sacseg.setProbability(probability);
        sacseg.setNumberOfThreads(12);

        sacseg.segment(*indices, *cofes);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto clouds = extractIndices(pcl_cloud, indices, negative);

        reportProgress(cancel, on_progress, 100);
        return {clouds, static_cast<float>(time.toc()), cofes};
    }

    SegmentationResult Segmentation::SACSegmentationFromNormals(const Cloud::Ptr& cloud, bool negative,
                                                                 int model, int method, double threshold,
                                                                 int max_iterations, double probability,
                                                                 bool optimize, double min_radius, double max_radius,
                                                                 double distance_weight, double d,
                                                                 std::atomic<bool>* cancel,
                                                                 std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::PointIndicesPtr indices(new pcl::PointIndices);
        ModelCoefficients::Ptr cofes(new ModelCoefficients);

        pcl::SACSegmentationFromNormals<PointXYZRGBN, PointXYZRGBN> sacseg;
        sacseg.setInputCloud(pcl_cloud);
        sacseg.setInputNormals(pcl_cloud);
        sacseg.setModelType(model);
        sacseg.setMethodType(method);
        sacseg.setDistanceThreshold(threshold);
        sacseg.setMaxIterations(max_iterations);
        sacseg.setOptimizeCoefficients(optimize);
        sacseg.setRadiusLimits(min_radius, max_radius);
        sacseg.setProbability(probability);
        sacseg.setNormalDistanceWeight(distance_weight);
        sacseg.setDistanceFromOrigin(d);
        sacseg.setNumberOfThreads(12);

        sacseg.segment(*indices, *cofes);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto clouds = extractIndices(pcl_cloud, indices, negative);

        reportProgress(cancel, on_progress, 100);
        return {clouds, static_cast<float>(time.toc()), cofes};
    }

    SegmentationResult Segmentation::EuclideanClusterExtraction(const Cloud::Ptr& cloud, bool negative,
                                                                 double tolerance, int min_cluster_size,
                                                                 int max_cluster_size,
                                                                 std::atomic<bool>* cancel,
                                                                 std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        IndicesClustersPtr clusters(new IndicesClusters);

        pcl::EuclideanClusterExtraction<PointXYZRGBN> seg;
        seg.setInputCloud(pcl_cloud);
        seg.setSearchMethod(tree);
        seg.setClusterTolerance(tolerance);
        seg.setMinClusterSize(min_cluster_size);
        seg.setMaxClusterSize(max_cluster_size);
        seg.extract(*clusters);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto segmented_clouds = extractClusters(pcl_cloud, clusters, negative);

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::DBSCANClusterExtraction(const Cloud::Ptr& cloud, bool negative,
                                                               double eps, int min_pts,
                                                               int min_cluster_size, int max_cluster_size,
                                                               double normal_weight, double color_weight,
                                                               std::atomic<bool>* cancel,
                                                               std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        tree->setInputCloud(pcl_cloud);
        IndicesClustersPtr clusters(new IndicesClusters);

        // 纯 XYZ 距离的 DBSCAN（PCL 原生）
        // TODO: 支持法线/颜色加权的自定义距离 DBSCAN
        pcl::EuclideanClusterExtraction<PointXYZRGBN> seg;
        seg.setInputCloud(pcl_cloud);
        seg.setSearchMethod(tree);
        seg.setClusterTolerance(eps);
        seg.setMinClusterSize(min_pts);
        seg.setMaxClusterSize(max_cluster_size);
        seg.extract(*clusters);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        // 过滤 min_cluster_size
        std::vector<Cloud::Ptr> segmented_clouds;
        for (IndicesClusters::const_iterator it = clusters->begin(); it != clusters->end(); ++it)
        {
            if (static_cast<int>(it->indices.size()) < min_cluster_size) continue;
            if (static_cast<int>(it->indices.size()) > max_cluster_size) continue;
            Cloud::Ptr cluster = Cloud::fromPCL_XYZRGBN(
                pcl::PointCloud<PointXYZRGBN>(*pcl_cloud, it->indices));
            cluster->setId("cluster");
            segmented_clouds.push_back(cluster);
        }

        if (negative) {
            PointIndicesPtr all_indices(new PointIndices);
            for (IndicesClusters::const_iterator it = clusters->begin(); it != clusters->end(); ++it) {
                if (static_cast<int>(it->indices.size()) < min_cluster_size) continue;
                if (static_cast<int>(it->indices.size()) > max_cluster_size) continue;
                all_indices->indices.insert(all_indices->indices.end(),
                                             it->indices.begin(), it->indices.end());
            }
            return {extractIndices(pcl_cloud, all_indices, true),
                    static_cast<float>(time.toc()), nullptr};
        }

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::KMeansClusterExtraction(const Cloud::Ptr& cloud,
                                                              int k, int max_iterations,
                                                              double normal_weight, double color_weight,
                                                              std::atomic<bool>* cancel,
                                                              std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        size_t point_count = pcl_cloud->size();

        if (point_count == 0 || k <= 0) return {};

        // 初始化聚类中心（随机选取 k 个点）
        std::vector<Eigen::VectorXf> centers(k);
        std::vector<int> assignments(point_count, -1);
        std::mt19937 rng(42);

        std::vector<size_t> indices(point_count);
        for (size_t i = 0; i < point_count; i++) indices[i] = i;
        std::shuffle(indices.begin(), indices.end(), rng);
        for (int c = 0; c < k; c++) {
            const auto& pt = pcl_cloud->at(indices[c]);
            // 特征维度: xyz(3) + normal(3)*normal_weight + rgb(3)*color_weight
            int dim = 3;
            if (normal_weight > 0.0) dim += 3;
            if (color_weight > 0.0) dim += 3;
            Eigen::VectorXf feat(dim);
            feat[0] = pt.x; feat[1] = pt.y; feat[2] = pt.z;
            int off = 3;
            if (normal_weight > 0.0) {
                feat[off] = pt.normal_x * normal_weight;
                feat[off+1] = pt.normal_y * normal_weight;
                feat[off+2] = pt.normal_z * normal_weight;
                off += 3;
            }
            if (color_weight > 0.0) {
                feat[off] = pt.r / 255.0f * color_weight;
                feat[off+1] = pt.g / 255.0f * color_weight;
                feat[off+2] = pt.b / 255.0f * color_weight;
            }
            centers[c] = feat;
        }

        // K-Means 迭代
        for (int iter = 0; iter < max_iterations; iter++) {
            if (isCanceled(cancel)) return {};

            // Assignment step
            for (size_t i = 0; i < point_count; i++) {
                const auto& pt = pcl_cloud->at(i);
                int dim = static_cast<int>(centers[0].size());
                Eigen::VectorXf feat(dim);
                feat[0] = pt.x; feat[1] = pt.y; feat[2] = pt.z;
                int off = 3;
                if (normal_weight > 0.0) {
                    feat[off] = pt.normal_x * normal_weight;
                    feat[off+1] = pt.normal_y * normal_weight;
                    feat[off+2] = pt.normal_z * normal_weight;
                    off += 3;
                }
                if (color_weight > 0.0) {
                    feat[off] = pt.r / 255.0f * color_weight;
                    feat[off+1] = pt.g / 255.0f * color_weight;
                    feat[off+2] = pt.b / 255.0f * color_weight;
                }

                float min_dist = std::numeric_limits<float>::max();
                int best_cluster = 0;
                for (int c = 0; c < k; c++) {
                    float dist = (feat - centers[c]).squaredNorm();
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_cluster = c;
                    }
                }
                assignments[i] = best_cluster;
            }

            // Update step
            std::vector<Eigen::VectorXf> new_centers(k, Eigen::VectorXf::Zero(centers[0].size()));
            std::vector<int> counts(k, 0);
            for (size_t i = 0; i < point_count; i++) {
                const auto& pt = pcl_cloud->at(i);
                int c = assignments[i];
                int dim = static_cast<int>(new_centers[0].size());
                Eigen::VectorXf feat(dim);
                feat[0] = pt.x; feat[1] = pt.y; feat[2] = pt.z;
                int off = 3;
                if (normal_weight > 0.0) {
                    feat[off] = pt.normal_x * normal_weight;
                    feat[off+1] = pt.normal_y * normal_weight;
                    feat[off+2] = pt.normal_z * normal_weight;
                    off += 3;
                }
                if (color_weight > 0.0) {
                    feat[off] = pt.r / 255.0f * color_weight;
                    feat[off+1] = pt.g / 255.0f * color_weight;
                    feat[off+2] = pt.b / 255.0f * color_weight;
                }
                new_centers[c] += feat;
                counts[c]++;
            }
            for (int c = 0; c < k; c++) {
                if (counts[c] > 0) {
                    centers[c] = new_centers[c] / counts[c];
                }
            }

            reportProgress(cancel, on_progress, 10 + static_cast<int>(90.0 * (iter + 1) / max_iterations));
        }

        if (isCanceled(cancel)) return {};

        // 提取聚类
        std::vector<pcl::PointIndices> cluster_indices(k);
        for (size_t i = 0; i < point_count; i++) {
            cluster_indices[assignments[i]].indices.push_back(static_cast<int>(i));
        }

        std::vector<Cloud::Ptr> segmented_clouds;
        for (int c = 0; c < k; c++) {
            if (cluster_indices[c].indices.empty()) continue;
            Cloud::Ptr cluster = Cloud::fromPCL_XYZRGBN(
                pcl::PointCloud<PointXYZRGBN>(*pcl_cloud, cluster_indices[c].indices));
            cluster->setId("cluster");
            segmented_clouds.push_back(cluster);
        }

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::RegionGrowing(const Cloud::Ptr& cloud, bool negative,
                                                   int min_cluster_size, int max_cluster_size,
                                                   bool smooth_mode, bool curvature_test, bool residual_test,
                                                   float smoothness_threshold, float residual_threshold,
                                                   float curvature_threshold, int neighbours,
                                                   std::atomic<bool>* cancel,
                                                   std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        IndicesClustersPtr clusters(new IndicesClusters);

        pcl::RegionGrowing<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputCloud(pcl_cloud);
        reg.setInputNormals(pcl_cloud);
        reg.setSearchMethod(tree);
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSmoothModeFlag(smooth_mode);
        reg.setSmoothnessThreshold(pcl::deg2rad(smoothness_threshold));
        reg.setCurvatureTestFlag(curvature_test);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.setResidualTestFlag(residual_test);
        reg.setResidualThreshold(residual_threshold);
        reg.setNumberOfNeighbours(neighbours);
        reg.extract(*clusters);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto segmented_clouds = extractClusters(pcl_cloud, clusters, negative);

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::RegionGrowingFromSeed(const Cloud::Ptr& cloud, bool negative,
                                                           int seed_index,
                                                           int min_cluster_size, int max_cluster_size,
                                                           bool smooth_mode, bool curvature_test, bool residual_test,
                                                           float smoothness_threshold, float residual_threshold,
                                                           float curvature_threshold, int neighbours,
                                                           std::atomic<bool>* cancel,
                                                           std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        IndicesClustersPtr clusters(new IndicesClusters);

        pcl::RegionGrowing<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputCloud(pcl_cloud);
        reg.setInputNormals(pcl_cloud);
        reg.setSearchMethod(tree);
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSmoothModeFlag(smooth_mode);
        reg.setSmoothnessThreshold(pcl::deg2rad(smoothness_threshold));
        reg.setCurvatureTestFlag(curvature_test);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.setResidualTestFlag(residual_test);
        reg.setResidualThreshold(residual_threshold);
        reg.setNumberOfNeighbours(neighbours);
        reg.extract(*clusters);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        // 从种子点获取其所属的 segment
        pcl::PointIndices seed_cluster;
        reg.getSegmentFromPoint(seed_index, seed_cluster);

        if (seed_cluster.indices.empty()) return {};

        std::vector<Cloud::Ptr> segmented_clouds;
        if (negative) {
            // 返回种子点所属 segment 之外的所有点
            PointIndicesPtr seg_indices(new PointIndices);
            seg_indices->indices = seed_cluster.indices;
            segmented_clouds = extractIndices(pcl_cloud, seg_indices, true);
        } else {
            // 返回种子点所属 segment
            Cloud::Ptr cluster_cloud = Cloud::fromPCL_XYZRGBN(
                pcl::PointCloud<PointXYZRGBN>(*pcl_cloud, seed_cluster.indices));
            cluster_cloud->setId("cluster");
            segmented_clouds.push_back(cluster_cloud);
        }

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::RegionGrowingRGB(const Cloud::Ptr& cloud, bool negative,
                                                      int min_cluster_size, int max_cluster_size,
                                                      bool smooth_mode, bool curvature_test, bool residual_test,
                                                      float smoothness_threshold, float residual_threshold,
                                                      float curvature_threshold, int neighbours,
                                                      float pt_thresh, float re_thresh,
                                                      float dis_thresh, int nghbr_number,
                                                      std::atomic<bool>* cancel,
                                                      std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        IndicesClustersPtr clusters(new IndicesClusters);

        pcl::RegionGrowingRGB<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputCloud(pcl_cloud);
        reg.setInputNormals(pcl_cloud);
        reg.setSearchMethod(tree);
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSmoothModeFlag(smooth_mode);
        reg.setSmoothnessThreshold(pcl::deg2rad(smoothness_threshold));
        reg.setCurvatureTestFlag(curvature_test);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.setResidualTestFlag(residual_test);
        reg.setResidualThreshold(residual_threshold);
        reg.setNumberOfNeighbours(neighbours);
        reg.setPointColorThreshold(pt_thresh);
        reg.setRegionColorThreshold(re_thresh);
        reg.setDistanceThreshold(dis_thresh);
        reg.setNumberOfRegionNeighbours(nghbr_number);
        reg.extract(*clusters);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto segmented_clouds = extractClusters(pcl_cloud, clusters, negative);

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::SupervoxelClustering(const Cloud::Ptr& cloud,
                                                          float voxel_resolution, float seed_resolution,
                                                          float color_importance, float spatial_importance,
                                                          float normal_importance, bool camera_transform,
                                                          std::atomic<bool>* cancel,
                                                          std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*cloud->toPCL_XYZRGB(), *cloud_xyzrgba);

        pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);
        super.setInputCloud(cloud_xyzrgba);
        super.setVoxelResolution(voxel_resolution);
        super.setSeedResolution(seed_resolution);
        super.setColorImportance(color_importance);
        super.setSpatialImportance(spatial_importance);
        super.setNormalImportance(normal_importance);
        super.setUseSingleCameraTransform(camera_transform);

        std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters;
        super.extract(supervoxel_clusters);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
        super.getSupervoxelAdjacency(supervoxel_adjacency);

        std::vector<Cloud::Ptr> segmented_clouds;

        std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
        for (; label_itr != supervoxel_adjacency.end();)
        {
            uint32_t supervoxel_label = label_itr->first;
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxels_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*supervoxel->voxels_, *voxels_rgb);
            segmented_clouds.push_back(Cloud::fromPCL_XYZRGB(*voxels_rgb));
            label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
        }

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::DonSegmentation(const Cloud::Ptr& cloud, bool negative,
                                                     double mean_radius, double scale1, double scale2,
                                                     double threshold, double segradius,
                                                     int minClusterSize, int maxClusterSize,
                                                     std::atomic<bool>* cancel,
                                                     std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        IndicesClustersPtr cluster_indices(new IndicesClusters);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        scale1 *= mean_radius;
        scale2 *= mean_radius;
        segradius *= mean_radius;

        pcl::NormalEstimationOMP<PointXYZRGBN, PointN> ne;
        ne.setInputCloud(pcl_cloud);
        ne.setSearchMethod(tree);
        ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

        pcl::PointCloud<PointN>::Ptr normals_small_scale(new pcl::PointCloud<PointN>);
        pcl::PointCloud<PointN>::Ptr normals_large_scale(new pcl::PointCloud<PointN>);

        ne.setNumberOfThreads(12);
        ne.setRadiusSearch(scale1);
        ne.compute(*normals_small_scale);
        ne.setRadiusSearch(scale2);
        ne.compute(*normals_large_scale);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 40);

        pcl::PointCloud<PointN>::Ptr doncloud(new pcl::PointCloud<PointN>);
        pcl::DifferenceOfNormalsEstimation<PointXYZRGBN, PointN, PointN> don;
        don.setInputCloud(pcl_cloud);
        don.setNormalScaleLarge(normals_large_scale);
        don.setNormalScaleSmall(normals_small_scale);
        don.computeFeature(*doncloud);

        pcl::ConditionOr<PointN>::Ptr range_cond(new pcl::ConditionOr<PointN>);
        range_cond->addComparison(pcl::FieldComparison<PointN>::ConstPtr(
            new pcl::FieldComparison<PointN>("curvature", pcl::ComparisonOps::GT, threshold)));

        pcl::ConditionalRemoval<PointN> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(doncloud);
        pcl::PointCloud<PointN>::Ptr doncloud_filtered(new pcl::PointCloud<PointN>);
        condrem.filter(*doncloud_filtered);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        pcl::search::KdTree<PointN>::Ptr segtree(new pcl::search::KdTree<PointN>);
        segtree->setInputCloud(doncloud_filtered);

        pcl::EuclideanClusterExtraction<PointN> ecc;
        ecc.setClusterTolerance(segradius);
        ecc.setMinClusterSize(minClusterSize);
        ecc.setMaxClusterSize(maxClusterSize);
        ecc.setSearchMethod(segtree);
        ecc.setInputCloud(doncloud_filtered);
        ecc.extract(*cluster_indices);

        auto segmented_clouds = extractClusters(pcl_cloud, cluster_indices, negative);

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::MinCutSegmentation(const Cloud::Ptr& cloud,
                                                        double sigma, double radius, double weight,
                                                        int neighbour_number,
                                                        std::atomic<bool>* cancel,
                                                        std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        IndicesClustersPtr cluster_indices(new IndicesClusters);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::PointCloud<PointXYZRGBN>::Ptr foreground_points(new pcl::PointCloud<PointXYZRGBN>);
        PointXYZRGBN center_pt;
        center_pt.x = cloud->center()[0];
        center_pt.y = cloud->center()[1];
        center_pt.z = cloud->center()[2];
        foreground_points->push_back(center_pt);

        pcl::MinCutSegmentation<PointXYZRGBN> mseg;
        mseg.setInputCloud(pcl_cloud);
        mseg.setSigma(sigma);
        mseg.setRadius(radius);
        mseg.setSourceWeight(weight);
        mseg.setSearchMethod(tree);
        mseg.setForegroundPoints(foreground_points);
        mseg.setNumberOfNeighbours(neighbour_number);
        mseg.extract(*cluster_indices);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto segmented_clouds = extractClusters(pcl_cloud, cluster_indices, false);

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::MorphologicalFilter(const Cloud::Ptr& cloud, bool negative,
                                                         int max_window_size, float slope,
                                                         float max_distance, float initial_distance,
                                                         float cell_size, float base,
                                                         std::atomic<bool>* cancel,
                                                         std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        PointIndicesPtr inliers(new PointIndices);

        pcl::ProgressiveMorphologicalFilter<PointXYZRGBN> pmf;
        pmf.setInputCloud(pcl_cloud);
        pmf.setMaxWindowSize(max_window_size);
        pmf.setSlope(slope);
        pmf.setMaxDistance(max_distance);
        pmf.setInitialDistance(initial_distance);
        pmf.setCellSize(cell_size);
        pmf.setBase(base);
        pmf.extract(inliers->indices);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto segmented_clouds = extractIndices(pcl_cloud, inliers, negative);

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::SeededHueSegmentation(const Cloud::Ptr& cloud, bool negative,
                                                           double tolerance, float delta_hue,
                                                           std::atomic<bool>* cancel,
                                                           std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = cloud->toPCL_XYZRGB();
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::PointIndicesPtr indices_in(new pcl::PointIndices);
        pcl::PointIndicesPtr indices_out(new pcl::PointIndices);

        pcl::SeededHueSegmentation seg;
        seg.setInputCloud(cloudrgb);
        seg.setSearchMethod(tree);
        seg.setClusterTolerance(tolerance);
        seg.setDeltaHue(delta_hue);
        seg.segment(*indices_in, *indices_out);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        auto segmented_clouds = extractIndices(pcl_cloud, indices_out, negative);

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::SegmentDifferences(const Cloud::Ptr& cloud,
                                                         const Cloud::Ptr& tar_cloud,
                                                         double sqr_threshold,
                                                         std::atomic<bool>* cancel,
                                                         std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        auto pcl_tar = tar_cloud->toPCL_XYZRGBN();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::PointCloud<PointXYZRGBN>::Ptr diff(new pcl::PointCloud<PointXYZRGBN>);

        pcl::SegmentDifferences<PointXYZRGBN> seg;
        seg.setInputCloud(pcl_cloud);
        seg.setSearchMethod(tree);
        seg.setTargetCloud(pcl_tar);
        seg.setDistanceThreshold(sqr_threshold);
        seg.segment(*diff);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        std::vector<Cloud::Ptr> segmented_clouds;
        segmented_clouds.push_back(Cloud::fromPCL_XYZRGBN(*diff));

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

    SegmentationResult Segmentation::ExtractPolygonalPrismData(const Cloud::Ptr& cloud, bool negative,
                                                                const Cloud::Ptr& hull,
                                                                double height_min, double height_max,
                                                                float vpx, float vpy, float vpz,
                                                                std::atomic<bool>* cancel,
                                                                std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        auto pcl_hull = hull->toPCL_XYZRGBN();
        pcl::PointIndicesPtr indices(new pcl::PointIndices);

        pcl::ExtractPolygonalPrismData<PointXYZRGBN> seg;
        seg.setInputCloud(pcl_cloud);
        seg.setInputPlanarHull(pcl_hull);
        seg.setHeightLimits(height_min, height_max);
        seg.setViewPoint(vpx, vpy, vpz);
        seg.segment(*indices);

        if (isCanceled(cancel)) return {};
        reportProgress(cancel, on_progress, 70);

        auto segmented_clouds = extractIndices(pcl_cloud, indices, negative);

        reportProgress(cancel, on_progress, 100);
        return {segmented_clouds, static_cast<float>(time.toc()), nullptr};
    }

}  // namespace ct
