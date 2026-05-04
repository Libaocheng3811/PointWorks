#include <cmath>

#include "keypoints.h"

#include <pcl/console/time.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/trajkovic_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>

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

    KeypointResult Keypoints::NarfKeypoint(const Cloud::Ptr& cloud,
                                            const RangeImage::Ptr& range_image,
                                            float support_size,
                                            std::atomic<bool>* cancel,
                                            std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        pcl::RangeImageBorderExtractor range_image_border_extractor;
        pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
        narf_keypoint_detector.setRangeImage(&(*range_image));
        narf_keypoint_detector.getParameters().support_size = support_size;

        pcl::PointCloud<int> keypoint_indices;
        narf_keypoint_detector.compute(keypoint_indices);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 70);

        // convert indices to point cloud
        pcl::PointCloud<PointXYZRGB>::Ptr pcl_keypoints(new pcl::PointCloud<PointXYZRGB>);
        pcl_keypoints->reserve(keypoint_indices.size());
        for (size_t i = 0; i < keypoint_indices.size(); ++i) {
            PointXYZRGB pt;
            pt.getVector3fMap() = range_image->at(keypoint_indices[i]).getVector3fMap();
            pt.r = 0; pt.g = 255; pt.b = 0;
            pcl_keypoints->push_back(pt);
        }

        auto result = Cloud::fromPCL_XYZRGB(*pcl_keypoints, cloud->getGlobalShift());
        result->setId(cloud->id());
        result->setPointSize(cloud->pointSize() + 2);

        reportProgress(cancel, on_progress, 100);
        return {result, static_cast<float>(time.toc())};
    }

    KeypointResult Keypoints::HarrisKeypoint3D(const Cloud::Ptr& cloud,
                                                int response_method, float threshold,
                                                bool non_maxima, bool do_refine,
                                                int k, double radius,
                                                std::atomic<bool>* cancel,
                                                std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::PointCloud<PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<PointXYZI>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::HarrisKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN> est;
        est.setInputCloud(pcl_cloud);
        est.setNormals(pcl_cloud);
        est.setSearchMethod(tree);
        // Harris3D 内部使用 radiusSearch，不能同时设置 k 和 radius。
        // 使用 resolution 作为搜索半径。
        if (radius > 0) est.setRadiusSearch(radius);
        else est.setRadiusSearch(0.01); // 默认兜底
        est.setMethod(pcl::HarrisKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN>::ResponseMethod(response_method));
        est.setThreshold(threshold);
        est.setNonMaxSupression(non_maxima);
        est.setRefine(do_refine);
        est.setNumberOfThreads(1);
        est.compute(*keypoints_temp);
        pcl_cloud.reset();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 70);

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<PointXYZRGBN>);
        pcl_result->reserve(keypoints_temp->size());
        for (const auto& pt : keypoints_temp->points) {
            PointXYZRGBN p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            p.r = 0; p.g = 255; p.b = 0;
            pcl_result->push_back(p);
        }

        auto result = Cloud::fromPCL_XYZRGBN(*pcl_result, cloud->getGlobalShift());
        result->setId(cloud->id());
        result->setPointSize(cloud->pointSize() + 2);

        reportProgress(cancel, on_progress, 100);
        return {result, static_cast<float>(time.toc())};
    }

    KeypointResult Keypoints::ISSKeypoint3D(const Cloud::Ptr& cloud,
                                             double resolution,
                                             double gamma_21, double gamma_32,
                                             int min_neighbors, float angle,
                                             int k, double radius,
                                             std::atomic<bool>* cancel,
                                             std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::PointCloud<PointXYZRGBN>::Ptr keypoints_temp(new pcl::PointCloud<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::ISSKeypoint3D<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN> est;
        est.setInputCloud(pcl_cloud);
        est.setNormals(pcl_cloud);
        est.setSearchMethod(tree);
        // ISS 使用自己的 salientRadius/nonMaxRadius 控制搜索范围。
        // 基类 Keypoint::initCompute() 要求 k 或 radius 至少一个非零，
        // 这里设一个极小的 radius 满足基类检查（具体值不影响 ISS 内部行为）。
        est.setRadiusSearch(0.001);
        est.setSalientRadius(6 * resolution);
        est.setNonMaxRadius(4 * resolution);
        est.setNormalRadius(4 * resolution);
        est.setBorderRadius(4 * resolution);
        est.setThreshold21(gamma_21);
        est.setThreshold32(gamma_32);
        est.setMinNeighbors(min_neighbors);
        est.setAngleThreshold(pcl::deg2rad(angle));
        est.setNumberOfThreads(1);
        est.compute(*keypoints_temp);
        pcl_cloud.reset();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 70);

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<PointXYZRGBN>);
        pcl_result->reserve(keypoints_temp->size());
        for (const auto& pt : keypoints_temp->points) {
            PointXYZRGBN p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            p.normal_x = pt.normal_x; p.normal_y = pt.normal_y; p.normal_z = pt.normal_z;
            p.r = 0; p.g = 255; p.b = 0;
            pcl_result->push_back(p);
        }

        auto result = Cloud::fromPCL_XYZRGBN(*pcl_result, cloud->getGlobalShift());
        result->setId(cloud->id());
        result->setPointSize(cloud->pointSize() + 2);

        reportProgress(cancel, on_progress, 100);
        return {result, static_cast<float>(time.toc())};
    }

    KeypointResult Keypoints::SIFTKeypoint(const Cloud::Ptr& cloud,
                                            float min_scale, int nr_octaves,
                                            int nr_scales_per_octave, float min_contrast,
                                            int k, double radius,
                                            std::atomic<bool>* cancel,
                                            std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud_xyzrgb = cloud->toPCL_XYZRGB();
        pcl::PointCloud<PointWithScale>::Ptr result_temp(new pcl::PointCloud<PointWithScale>);
        pcl::search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>);

        pcl::SIFTKeypoint<PointXYZRGB, PointWithScale> est;
        est.setInputCloud(pcl_cloud_xyzrgb);
        est.setSearchMethod(tree);
        // SIFT 重写了 initCompute()，内部自管理搜索参数，
        // 不应设置 setKSearch 或 setRadiusSearch
        est.setScales(min_scale, nr_octaves, nr_scales_per_octave);
        est.setMinimumContrast(min_contrast);
        est.compute(*result_temp);

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 70);

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<PointXYZRGBN>);
        pcl_result->reserve(result_temp->size());
        for (const auto& pt : result_temp->points) {
            PointXYZRGBN p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            p.r = 0; p.g = 255; p.b = 0;
            pcl_result->push_back(p);
        }

        auto result = Cloud::fromPCL_XYZRGBN(*pcl_result, cloud->getGlobalShift());
        result->setId(cloud->id());
        result->setPointSize(cloud->pointSize() + 2);

        reportProgress(cancel, on_progress, 100);
        return {result, static_cast<float>(time.toc())};
    }

    KeypointResult Keypoints::TrajkovicKeypoint3D(const Cloud::Ptr& cloud,
                                                    int compute_method, int window_size,
                                                    float first_threshold, float second_threshold,
                                                    int k, double radius,
                                                    std::atomic<bool>* cancel,
                                                    std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);
        TicToc time;
        time.tic();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::PointCloud<PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<PointXYZI>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::TrajkovicKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN> est;
        est.setInputCloud(pcl_cloud);
        est.setNormals(pcl_cloud);
        est.setSearchMethod(tree);
        // Trajkovic 重写了 initCompute()，使用 setWindowSize 和像素偏移访问，
        // 不应设置 setKSearch 或 setRadiusSearch
        est.setMethod(pcl::TrajkovicKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN>::ComputationMethod(compute_method));
        est.setWindowSize(window_size);
        est.setFirstThreshold(first_threshold);
        est.setSecondThreshold(second_threshold);
        est.setNumberOfThreads(1);
        est.compute(*keypoints_temp);
        pcl_cloud.reset();

        if (isCanceled(cancel)) return {nullptr, 0};
        reportProgress(cancel, on_progress, 70);

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_result(new pcl::PointCloud<PointXYZRGBN>);
        pcl_result->reserve(keypoints_temp->size());
        for (const auto& pt : keypoints_temp->points) {
            PointXYZRGBN p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            p.r = 0; p.g = 255; p.b = 0;
            pcl_result->push_back(p);
        }

        auto result = Cloud::fromPCL_XYZRGBN(*pcl_result, cloud->getGlobalShift());
        result->setId(cloud->id());
        result->setPointSize(cloud->pointSize() + 2);

        reportProgress(cancel, on_progress, 100);
        return {result, static_cast<float>(time.toc())};
    }

}  // namespace ct
