//
// Created by LBC on 2024/12/26.
//

#include "filters.h"

#include <pcl/console/time.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

#include <pcl/filters/impl/local_maximum.hpp>
#include <pcl/filters/impl/project_inliers.hpp>

namespace ct
{
    // helper: early-return if cancel is requested
    static inline bool isCanceled(std::atomic<bool>* cancel) {
        return cancel && cancel->load();
    }

    // helper: report progress (skip if canceled)
    static inline void reportProgress(std::atomic<bool>* cancel,
                                      std::function<void(int)> on_progress,
                                      int pct) {
        if (!isCanceled(cancel) && on_progress) on_progress(pct);
    }

    // helper: synchronize properties and custom fields
    void syncCloudProperties(const Cloud::Ptr& source, Cloud::Ptr& target){
        target->setId(source->id());
        target->setHasColors(source->hasColors());

        // TODO: PCL filters cannot return indices, so custom field info cannot be extracted via indices. Discarding custom fields for now.

        target->backupColors();
    }

    FilterResult Filters::PassThrough(const Cloud::Ptr& cloud, const std::string& field_name,
                                       float limit_min, float limit_max, bool negative,
                                       std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        // convert input cloud
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::PassThrough<PointXYZRGBN> pfilter;
        pfilter.setInputCloud(pcl_cloud);
        pfilter.setFilterFieldName(field_name);
        pfilter.setFilterLimits(limit_min, limit_max);
        pfilter.setNegative(negative);
        pfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        // construct Cloud from PCL result
        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::VoxelGrid(const Cloud::Ptr& cloud, float lx, float ly, float lz,
                                     bool negative,
                                     std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        // create a VoxelGrid filter object
        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::VoxelGrid<PointXYZRGBN> vfilter;
        vfilter.setInputCloud(pcl_cloud);
        vfilter.setLeafSize(lx, ly, lz);
        vfilter.setFilterLimitsNegative(negative);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 20);

        vfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::ApproximateVoxelGrid(const Cloud::Ptr& cloud, float lx, float ly, float lz,
                                                bool negative,
                                                std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::ApproximateVoxelGrid<PointXYZRGBN> avfilter;
        avfilter.setInputCloud(pcl_cloud);
        avfilter.setLeafSize(lx, ly, lz);
        avfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        if (negative)
        {
            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_neg_filtered(new pcl::PointCloud<PointXYZRGBN>);
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(pcl_cloud);
            extract.setIndices(avfilter.getRemovedIndices());
            extract.filter(*pcl_neg_filtered);
            pcl_filtered = pcl_neg_filtered;
        }

        if (isCanceled(cancel)) return {nullptr, 0};

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::StatisticalOutlierRemoval(const Cloud::Ptr& cloud, int nr_k, double stddev_mult,
                                                     bool negative,
                                                     std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::StatisticalOutlierRemoval<PointXYZRGBN> sfilter;
        sfilter.setInputCloud(pcl_cloud);
        sfilter.setMeanK(nr_k);
        sfilter.setStddevMulThresh(stddev_mult);
        sfilter.setNegative(negative);
        sfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::RadiusOutlierRemoval(const Cloud::Ptr& cloud, double radius, int min_pts,
                                               bool negative,
                                               std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(pcl_cloud);
        rfilter.setRadiusSearch(radius);
        rfilter.setMinNeighborsInRadius(min_pts);
        rfilter.setNegative(negative);
        rfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::ConditionalRemoval(const Cloud::Ptr& cloud, ConditionBase::Ptr con,
                                             bool negative,
                                             std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::ConditionalRemoval<PointXYZRGBN> bfilter;
        bfilter.setInputCloud(pcl_cloud);
        bfilter.setCondition(con);
        bfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        if (negative)
        {
            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_neg_filtered(new pcl::PointCloud<PointXYZRGBN>);
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(pcl_cloud);
            extract.setIndices(bfilter.getRemovedIndices());
            extract.filter(*pcl_neg_filtered);
            pcl_filtered = pcl_neg_filtered;
        }

        if (isCanceled(cancel)) return {nullptr, 0};

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::GridMinimun(const Cloud::Ptr& cloud, float resolution,
                                       bool negative,
                                       std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::GridMinimum<PointXYZRGBN> gfilter(resolution);
        gfilter.setInputCloud(pcl_cloud);
        gfilter.setResolution(resolution);
        gfilter.setNegative(negative);
        gfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::LocalMaximum(const Cloud::Ptr& cloud, float radius,
                                        bool negative,
                                        std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::LocalMaximum<PointXYZRGBN> lfilter;
        lfilter.setInputCloud(pcl_cloud);
        lfilter.setRadius(radius);
        lfilter.setNegative(negative);
        lfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::ShadowPoints(const Cloud::Ptr& cloud, float threshold,
                                        bool negative,
                                        std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::ShadowPoints<PointXYZRGBN, PointXYZRGBN> sfilter;
        sfilter.setInputCloud(pcl_cloud);
        sfilter.setNormals(pcl_cloud);
        sfilter.setThreshold(threshold);
        sfilter.setNegative(negative);
        sfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        syncCloudProperties(cloud, cloud_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::DownSampling(const Cloud::Ptr& cloud, float radius,
                                        bool negative,
                                        std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (!cloud) return {nullptr, 0};
        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        // convert input cloud
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::VoxelGrid<PointXYZRGBN> vfilter;
        vfilter.setInputCloud(pcl_cloud);
        vfilter.setLeafSize(radius, radius, radius);
        vfilter.setFilterLimitsNegative(negative);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 20);

        vfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        if (pcl_filtered->empty())
        {
            reportProgress(cancel, on_progress, 100);
            return {nullptr, 0};
        }

        // construct Cloud from PCL result
        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        if (!cloud_filtered || cloud_filtered->empty())
        {
            reportProgress(cancel, on_progress, 100);
            return {nullptr, 0};
        }

        syncCloudProperties(cloud, cloud_filtered);
        cloud_filtered->setHasNormals(cloud->hasNormals());

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::UniformSampling(const Cloud::Ptr& cloud, float radius,
                                           bool negative,
                                           std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (!cloud) return {nullptr, 0};
        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        // convert input cloud
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::UniformSampling<PointXYZRGBN> sfilter;
        sfilter.setInputCloud(pcl_cloud);
        sfilter.setRadiusSearch(radius);
        sfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        if (negative)
        {
            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_neg_filtered(new pcl::PointCloud<PointXYZRGBN>);
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(pcl_cloud);
            extract.setIndices(sfilter.getRemovedIndices());
            extract.filter(*pcl_neg_filtered);
            pcl_filtered = pcl_neg_filtered;
        }

        if (isCanceled(cancel)) return {nullptr, 0};

        // construct Cloud from PCL result
        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud->id());
        cloud_filtered->setHasColors(cloud->hasColors());
        cloud_filtered->setHasNormals(cloud->hasNormals());
        cloud_filtered->backupColors();

        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::RandomSampling(const Cloud::Ptr& cloud, int sample, int seed,
                                         bool negative,
                                         std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (!cloud) return {nullptr, 0};
        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        // convert input cloud
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::RandomSample<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(pcl_cloud);
        rfilter.setSample(sample);
        rfilter.setSeed(seed);
        rfilter.setNegative(negative);
        rfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        // construct Cloud from PCL result
        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud->id());
        cloud_filtered->setHasColors(cloud->hasColors());
        cloud_filtered->setHasNormals(cloud->hasNormals());
        cloud_filtered->backupColors();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::ReSampling(const Cloud::Ptr& cloud, float radius, int polynomial_order,
                                      bool negative,
                                      std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (!cloud) return {nullptr, 0};
        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        // convert input cloud
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::MovingLeastSquares<PointXYZRGBN, PointXYZRGBN> mfilter;
        mfilter.setInputCloud(pcl_cloud);
        mfilter.setSearchRadius(radius);
        if (!cloud->hasNormals()) mfilter.setComputeNormals(true);
        mfilter.setPolynomialOrder(polynomial_order);
        mfilter.setSearchMethod(tree);
        mfilter.setNumberOfThreads(14);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 20);

        mfilter.process(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        // construct Cloud from PCL result
        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud->id());
        cloud_filtered->setHasColors(cloud->hasColors());
        cloud_filtered->setHasNormals(true); // MLS computes normals
        cloud_filtered->backupColors();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::SamplingSurfaceNormal(const Cloud::Ptr& cloud, int sample, int seed, float ratio,
                                                bool negative,
                                                std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (!cloud) return {nullptr, 0};
        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        // convert input cloud
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::SamplingSurfaceNormal<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(pcl_cloud);
        rfilter.setSample(sample);
        rfilter.setSeed(seed);
        rfilter.setRatio(ratio);
        rfilter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        if (negative)
        {
            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_neg_filtered(new pcl::PointCloud<PointXYZRGBN>);
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(pcl_cloud);
            extract.setIndices(rfilter.getRemovedIndices());
            extract.filter(*pcl_neg_filtered);
            pcl_filtered = pcl_neg_filtered;
        }

        if (isCanceled(cancel)) return {nullptr, 0};

        // construct Cloud from PCL result
        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud->id());
        cloud_filtered->setHasColors(cloud->hasColors());
        cloud_filtered->setHasNormals(cloud->hasNormals());
        cloud_filtered->backupColors();

        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }

    FilterResult Filters::NormalSpaceSampling(const Cloud::Ptr& cloud, int sample, int seed, int bin,
                                              bool negative,
                                              std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();

        if (!cloud) return {nullptr, 0};
        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        // convert input cloud
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::NormalSpaceSampling<PointXYZRGBN, PointXYZRGBN> filter;
        filter.setInputCloud(pcl_cloud);
        filter.setNormals(pcl_cloud);
        filter.setSample(sample);
        filter.setSeed(seed);
        filter.setBins(bin, bin, bin);
        filter.setNegative(negative);
        filter.filter(*pcl_filtered);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 80);

        // construct Cloud from PCL result
        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud->id());
        cloud_filtered->setHasColors(cloud->hasColors());
        cloud_filtered->setHasNormals(cloud->hasNormals());
        cloud_filtered->backupColors();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 100);

        return {cloud_filtered, (float)time.toc()};
    }
}

// 强制实例化 PCL 模板，解决 Windows 下 LNK2001
#include <pcl/sample_consensus/model_types.h>

// 基础几何模型
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <pcl/sample_consensus/impl/sac_model_cone.hpp>

// 带有法线约束的几何模型
#include <pcl/sample_consensus/impl/sac_model_normal_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
