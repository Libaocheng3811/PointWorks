//
// Created by LBC on 2024/11/13.
//

#include "features.h"

#include <pcl/features/boundary.h>
#include <pcl/common/angles.h>
#include <pcl/features/impl/3dsc.hpp>
#include <pcl/features/impl/board.hpp>
#include <pcl/features/impl/crh.hpp>
#include <pcl/features/impl/cvfh.hpp>
#include <pcl/features/impl/don.hpp>
#include <pcl/features/impl/esf.hpp>
#include <pcl/features/impl/flare.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/impl/fpfh_omp.hpp>
#include <pcl/features/impl/gasd.hpp>
#include <pcl/features/impl/grsd.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/impl/rsd.hpp>
#include <pcl/features/impl/shot.hpp>
#include <pcl/features/impl/shot_lrf.hpp>
#include <pcl/features/impl/shot_lrf_omp.hpp>
#include <pcl/features/impl/shot_omp.hpp>
#include <pcl/features/impl/usc.hpp>
#include <pcl/features/impl/vfh.hpp>

namespace pw
{
    Box Features::boundingBoxAABB(const Cloud::Ptr& cloud)
    {
        auto pcl_cloud = cloud->toPCL_XYZ();
        PointXYZ min, max;
        pcl::getMinMax3D(*pcl_cloud, min, max);
        Eigen::Vector3f cloud_center =
                0.5f * (min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd;
        whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f affine = pcl::getTransformation(
                cloud_center[0], cloud_center[1], cloud_center[2], 0, 0, 0);
        return { whd(0), whd(1), whd(2), affine, cloud_center,
                 Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
    }

    Box Features::boundingBoxOBB(const Cloud::Ptr& cloud)
    {
        auto pcl_cloud = cloud->toPCL_XYZ();

        // 质心
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*pcl_cloud, pcaCentroid);
        // 协方差
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*pcl_cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f transform_inv = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        transform.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>());
        transform_inv = transform.inverse();

        pcl::PointCloud<PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<PointXYZ>);
        transformPointCloud(*pcl_cloud, *transformedCloud, transform);

        PointXYZ min, max;
        Eigen::Vector3f cloud_center, tcloud_center;
        pcl::getMinMax3D(*transformedCloud, min, max);
        cloud_center = 0.5f * (max.getVector3fMap() + min.getVector3fMap());
        Eigen::Vector3f whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f transform_inv_aff(transform_inv);
        pcl::transformPoint(cloud_center, tcloud_center, transform_inv_aff);
        return {whd(0), whd(1), whd(2), transform_inv_aff, tcloud_center,
                Eigen::Quaternionf(transform_inv.block<3, 3>(0, 0))};

    }

    Box Features::boundingBoxAdjust(const Cloud::Ptr& cloud, const Eigen::Affine3f &t)
    {
        auto pcl_cloud = cloud->toPCL_XYZ();

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform = t.matrix();
        Eigen::Matrix4f transform_inv = Eigen::Matrix4f::Identity();
        transform_inv = transform.inverse();

        pcl::PointCloud<PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<PointXYZ>);
        transformPointCloud(*pcl_cloud, *transformedCloud, transform);

        PointXYZ min, max;
        Eigen::Vector3f cloud_center, tcloud_center;
        pcl::getMinMax3D(*transformedCloud, min, max);
        cloud_center = 0.5f * (min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f transform_inv_aff(transform_inv);
        pcl::transformPoint(cloud_center, tcloud_center, transform_inv_aff);
        return {whd(0), whd(1), whd(2), transform_inv_aff, tcloud_center,
                Eigen::Quaternionf(transform_inv.block<3, 3>(0, 0)) };
    }

    FeatureResult Features::PFHEstimation(const Cloud::Ptr& cloud, int k, double radius,
                                           const Cloud::Ptr& surface,
                                           std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->pfh.reset(new PFHFeature);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        auto pcl_surface = surface ? surface->toPCL_XYZRGBN() : pcl_cloud;

        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::PFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::PFHSignature125> pfh;
        pfh.setSearchMethod(tree);
        pfh.setSearchSurface(pcl_surface);
        pfh.setInputCloud(pcl_cloud);
        pfh.setInputNormals(pcl_cloud);
        // PFH 同样不能同时设置 k 和 radius
        if (radius > 0) pfh.setRadiusSearch(radius);
        else if (k > 0) pfh.setKSearch(k);

        _progress(20);

        pfh.compute(*feature->pfh);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::FPFHEstimation(const Cloud::Ptr& cloud, int k, double radius,
                                            const Cloud::Ptr& surface,
                                            std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->fpfh.reset(new FPFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        auto pcl_surface = surface ? surface->toPCL_XYZRGBN() : pcl_cloud;

        pcl::FPFHEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::FPFHSignature33> fpfh;
        fpfh.setSearchMethod(tree);
        fpfh.setInputCloud(pcl_cloud);
        fpfh.setInputNormals(pcl_cloud);
        fpfh.setSearchSurface(pcl_surface);
        // FPFH 与关键点一样，k 和 radius 不能同时设置。
        // radius 控制邻域大小，是 FPFH 的核心参数；k 作为备用不应同时使用。
        if (radius > 0) fpfh.setRadiusSearch(radius);
        else if (k > 0) fpfh.setKSearch(k);
        fpfh.setNumberOfThreads(12);

        _progress(30);

        fpfh.compute(*feature->fpfh);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::VFHEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                          const Cloud::Ptr& surface,
                                          std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->vfh.reset(new VFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        auto pcl_surface = surface ? surface->toPCL_XYZRGBN() : pcl_cloud;

        pcl::VFHEstimation<PointXYZRGBN , PointXYZRGBN , pcl::VFHSignature308> vfh;
        vfh.setSearchMethod(tree);
        vfh.setInputCloud(pcl_cloud);
        vfh.setInputNormals(pcl_cloud);
        vfh.setSearchSurface(pcl_surface);
        vfh.setViewPoint(dir[0], dir[1], dir[2]);

        _progress(30);

        vfh.compute(*feature->vfh);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::ESFEstimation(const Cloud::Ptr& cloud,
                                          std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->esf.reset(new ESFFeature);
        pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::ESFEstimation<PointXYZRGBN, pcl::ESFSignature640> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchMethod(tree);

        _progress(30);

        est.compute(*feature->esf);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::GASDEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                           int shgs, int shs, int interp,
                                           std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->gasd.reset(new GASDFeature);
        pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::GASDEstimation<PointXYZRGBN, pcl::GASDSignature512> est;
        est.setSearchMethod(tree);
        est.setInputCloud(pcl_cloud);
        est.setViewDirection(dir);
        est.setShapeHalfGridSize(shgs);
        est.setShapeHistsSize(shs);
        est.setShapeHistsInterpMethod(pcl::HistogramInterpolationMethod(interp));

        _progress(30);

        est.compute(*feature->gasd);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::GASDColorEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                                int shgs, int shs, int interp,
                                                int chgs, int chs, int cinterp,
                                                std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->gasdc.reset(new GASDCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::GASDColorEstimation<PointXYZRGBN , pcl::GASDSignature984> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchMethod(tree);
        est.setViewDirection(dir);
        est.setShapeHalfGridSize(shgs);
        est.setShapeHistsSize(shs);
        est.setShapeHistsInterpMethod(pcl::HistogramInterpolationMethod(interp));
        est.setColorHalfGridSize(chgs);
        est.setColorHistsSize(chs);
        est.setColorHistsInterpMethod(pcl::HistogramInterpolationMethod(cinterp));

        _progress(40);

        est.compute(*feature->gasdc);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::RSDEstimation(const Cloud::Ptr& cloud, int nr_subdiv, double plane_radius,
                                          std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->rsd.reset(new RSDFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::RSDEstimation<PointXYZRGBN, PointXYZRGBN, pcl::PrincipalRadiiRSD> est;
        est.setSearchMethod(tree);
        est.setInputCloud(pcl_cloud);
        est.setInputNormals(pcl_cloud);
        est.setNrSubdivisions(nr_subdiv);
        est.setPlaneRadius(plane_radius);

        _progress(30);

        est.compute(*feature->rsd);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::GRSDEstimation(const Cloud::Ptr& cloud,
                                           std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->grsd.reset(new GRSDFeature);
        pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::GRSDEstimation<PointXYZRGBN, PointXYZRGBN, pcl::GRSDSignature21> est;
        est.setInputCloud(pcl_cloud);
        est.setInputNormals(pcl_cloud);
        est.setSearchMethod(tree);

        _progress(30);

        est.compute(*feature->grsd);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::CRHEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                          std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->crh.reset(new CRHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::CRHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::Histogram<90>> est;
        est.setInputCloud(pcl_cloud);
        est.setInputNormals(pcl_cloud);
        est.setSearchMethod(tree);
        est.setViewPoint(dir[0], dir[1], dir[2]);

        _progress(30);

        est.compute(*feature->crh);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::CVFHEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                           float radius_normals, float d1, float d2, float d3,
                                           int min, bool normalize,
                                           std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->vfh.reset(new VFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::CVFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::VFHSignature308> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchMethod(tree);
        est.setInputNormals(pcl_cloud);
        est.setViewPoint(dir[0], dir[1], dir[2]);
        est.setRadiusNormals(radius_normals);
        est.setClusterTolerance(d1);
        est.setEPSAngleThreshold(d2);
        est.setCurvatureThreshold(d3);
        est.setMinPoints(min);
        est.setNormalizeBins(normalize);

        _progress(40);

        est.compute(*feature->vfh);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::ShapeContext3DEstimation(const Cloud::Ptr& cloud,
                                                     double min_radius, double radius,
                                                     std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->sc3d.reset(new SC3DFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::ShapeContext3DEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ShapeContext1980> est;
        est.setInputCloud(pcl_cloud);
        est.setInputNormals(pcl_cloud);
        est.setSearchMethod(tree);
        est.setMinimalRadius(min_radius);
        est.setPointDensityRadius(radius);

        _progress(30);

        est.compute(*feature->sc3d);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::SHOTEstimation(const Cloud::Ptr& cloud,
                                           const ReferenceFrame::Ptr& lrf, float radius,
                                           const Cloud::Ptr& surface,
                                           std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->shot.reset(new SHOTFeature );
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        auto pcl_surface = surface ? surface->toPCL_XYZRGBN() : pcl_cloud;

        pcl::SHOTEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::SHOT352> shot;
        shot.setInputCloud(pcl_cloud);
        shot.setInputNormals(pcl_cloud);
        shot.setSearchMethod(tree);
        shot.setSearchSurface(pcl_surface);
        shot.setRadiusSearch(radius);
        shot.setNumberOfThreads(1);
        shot.setLRFRadius(radius);
        shot.setInputReferenceFrames(lrf);

        _progress(30);

        shot.compute(*feature->shot);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::SHOTColorEstimation(const Cloud::Ptr& cloud,
                                                const ReferenceFrame::Ptr& lrf, float radius,
                                                const Cloud::Ptr& surface,
                                                std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->shotc.reset(new SHOTCFeature);
        pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        auto pcl_surface = surface ? surface->toPCL_XYZRGBN() : pcl_cloud;

        pcl::SHOTColorEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::SHOT1344> shot;
        shot.setSearchMethod(tree);
        shot.setSearchSurface(pcl_surface);
        shot.setInputCloud(pcl_cloud);
        shot.setInputNormals(pcl_cloud);
        shot.setRadiusSearch(radius);
        shot.setNumberOfThreads(1);
        shot.setLRFRadius(radius);
        shot.setInputReferenceFrames(lrf);

        _progress(40);

        shot.compute(*feature->shotc);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    FeatureResult Features::UniqueShapeContext(const Cloud::Ptr& cloud,
                                               const ReferenceFrame::Ptr& lrf,
                                               double min_radius, double pt_radius, double loc_radius,
                                               std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->usc.reset(new USCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::UniqueShapeContext<PointXYZRGBN, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> est;
        est.setSearchMethod(tree);
        est.setInputCloud(pcl_cloud);
        est.setInputReferenceFrames(lrf);
        est.setMinimalRadius(min_radius);
        est.setPointDensityRadius(pt_radius);
        est.setLocalRadius(loc_radius);

        _progress(30);

        est.compute(*feature->usc);

        _progress(100);

        return {cloud->id(), feature, (float)time.toc()};
    }

    LRFResult Features::BOARDLocalReferenceFrameEstimation(const Cloud::Ptr& cloud,
                                                            float radius, bool find_holes,
                                                            float margin_thresh, int size,
                                                            float prob_thresh, float steep_thresh,
                                                            std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        ReferenceFrame::Ptr lrf(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::BOARDLocalReferenceFrameEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(pcl_cloud);
        est.setInputNormals(pcl_cloud);
        est.setSearchMethod(tree);
        est.setTangentRadius(radius);
        est.setFindHoles(find_holes);
        est.setMarginThresh(margin_thresh);
        est.setCheckMarginArraySize(size);
        est.setHoleSizeProbThresh(prob_thresh);
        est.setSteepThresh(steep_thresh);

        _progress(40);

        est.compute(*lrf);

        _progress(100);

        return {cloud->id(), lrf, (float)time.toc()};
    }

    LRFResult Features::FLARELocalReferenceFrameEstimation(const Cloud::Ptr& cloud,
                                                            float radius, float margin_thresh,
                                                            int min_neighbors_for_normal_axis,
                                                            int min_neighbors_for_tangent_axis,
                                                            std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        ReferenceFrame::Ptr lrf(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::FLARELocalReferenceFrameEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(pcl_cloud);
        est.setInputNormals(pcl_cloud);
        est.setSearchMethod(tree);
        est.setTangentRadius(radius);
        est.setMarginThresh(margin_thresh);
        est.setMinNeighboursForNormalAxis(min_neighbors_for_normal_axis);
        est.setMinNeighboursForTangentAxis(min_neighbors_for_tangent_axis);

        _progress(40);

        est.compute(*lrf);

        _progress(100);

        return {cloud->id(), lrf, (float)time.toc()};
    }

    LRFResult Features::SHOTLocalReferenceFrameEstimation(const Cloud::Ptr& cloud,
                                                           float radius,
                                                           std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        if (cancel) cancel->store(false);

        TicToc time;
        time.tic();
        ReferenceFrame::Ptr lrf(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        pcl::SHOTLocalReferenceFrameEstimationOMP<PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchMethod(tree);
        est.setNumberOfThreads(1);
        if (radius > 0) est.setRadiusSearch(radius);
        else est.setRadiusSearch(0.01f);

        _progress(40);

        est.compute(*lrf);

        _progress(100);

        return {cloud->id(), lrf, (float)time.toc()};
    }

    Cloud::Ptr Features::BoundaryEstimation(const Cloud::Ptr& cloud,
                                             int k, double radius, double angle,
                                             std::atomic<bool>* cancel,
                                             std::function<void(int)> on_progress)
    {
        if (cancel && cancel->load()) return nullptr;
        if (!cloud->hasNormals()) return nullptr;

        auto _progress = [&](int pct) { if (cancel && cancel->load()) return; if (on_progress) on_progress(pct); };
        _progress(10);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        _progress(30);

        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::PointCloud<pcl::Boundary>::Ptr boundary(new pcl::PointCloud<pcl::Boundary>);

        pcl::BoundaryEstimation<PointXYZRGBN, PointXYZRGBN, pcl::Boundary> be;
        be.setInputCloud(pcl_cloud);
        be.setInputNormals(pcl_cloud);
        be.setSearchMethod(tree);
        // Boundary 不能同时设置 k 和 radius
        if (radius > 0) be.setRadiusSearch(radius);
        else if (k > 0) be.setKSearch(k);
        be.setAngleThreshold(static_cast<float>(pcl::deg2rad(angle)));

        _progress(50);
        be.compute(*boundary);
        _progress(80);

        // 提取边界点
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_pts(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t i = 0; i < boundary->size(); ++i)
        {
            if (boundary->points[i].boundary_point > 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = pcl_cloud->points[i].x;
                pt.y = pcl_cloud->points[i].y;
                pt.z = pcl_cloud->points[i].z;
                pt.r = pcl_cloud->points[i].r;
                pt.g = pcl_cloud->points[i].g;
                pt.b = pcl_cloud->points[i].b;
                boundary_pts->push_back(pt);
            }
        }

        auto result = Cloud::fromPCL_XYZRGB(*boundary_pts);
        result->setId(cloud->id());
        result->setPointSize(cloud->pointSize() + 2);

        _progress(100);
        return result;
    }

} // namespace pw
