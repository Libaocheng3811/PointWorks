//
// Created by LBC on 2024/11/13.
//

#ifndef POINTWORKS_FEATURES_H
#define POINTWORKS_FEATURES_H

#include "core/exports.h"
#include "core/cloud.h"

#include <pcl/point_types.h>

#include <functional>
#include <atomic>
#include <string>

namespace pw
{
    typedef pcl::PointCloud<pcl::ShapeContext1980>          SC3DFeature;
    typedef pcl::PointCloud<pcl::Histogram<90>>             CRHFeature;
    typedef pcl::PointCloud<pcl::VFHSignature308>           VFHFeature;
    typedef pcl::PointCloud<pcl::ESFSignature640>           ESFFeature;
    typedef pcl::PointCloud<pcl::FPFHSignature33>           FPFHFeature;
    typedef pcl::PointCloud<pcl::GASDSignature512>          GASDFeature;
    typedef pcl::PointCloud<pcl::GASDSignature984>          GASDCFeature;
    typedef pcl::PointCloud<pcl::GRSDSignature21>           GRSDFeature;
    typedef pcl::PointCloud<pcl::PFHSignature125>           PFHFeature;
    typedef pcl::PointCloud<pcl::PrincipalRadiiRSD>         RSDFeature;
    typedef pcl::PointCloud<pcl::SHOT352>                   SHOTFeature;
    typedef pcl::PointCloud<pcl::SHOT1344>                  SHOTCFeature;
    typedef pcl::PointCloud<pcl::UniqueShapeContext1960>    USCFeature;
    typedef pcl::PointCloud<pcl::ReferenceFrame>            ReferenceFrame;

    struct FeatureType
    {
        using Ptr = std::shared_ptr<FeatureType>;
        using ConstPtr = std::shared_ptr<const FeatureType>;

        PFHFeature::Ptr     pfh = nullptr;
        FPFHFeature::Ptr    fpfh = nullptr;
        VFHFeature::Ptr     vfh = nullptr;
        ESFFeature::Ptr     esf = nullptr;
        SC3DFeature::Ptr    sc3d = nullptr;
        GASDFeature::Ptr    gasd = nullptr;
        CRHFeature::Ptr     crh = nullptr;
        GRSDFeature::Ptr    grsd = nullptr;
        GASDCFeature::Ptr   gasdc = nullptr;
        RSDFeature::Ptr     rsd = nullptr;
        SHOTFeature::Ptr    shot = nullptr;
        SHOTCFeature::Ptr   shotc = nullptr;
        USCFeature::Ptr     usc = nullptr;
    };

    struct FeatureResult {
        std::string id;
        FeatureType::Ptr feature;
        float time_ms = 0;
    };

    struct LRFResult {
        std::string id;
        ReferenceFrame::Ptr lrf;
        float time_ms = 0;
    };

    class Features
    {
    public:
        static Box boundingBoxAABB(const Cloud::Ptr& cloud);
        static Box boundingBoxOBB(const Cloud::Ptr& cloud);
        static Box boundingBoxAdjust(const Cloud::Ptr& cloud, const Eigen::Affine3f& t);

        static FeatureResult PFHEstimation(const Cloud::Ptr& cloud, int k, double radius,
                                             const Cloud::Ptr& surface = nullptr,
                                             std::atomic<bool>* cancel = nullptr,
                                             std::function<void(int)> on_progress = nullptr);

        static FeatureResult FPFHEstimation(const Cloud::Ptr& cloud, int k, double radius,
                                              const Cloud::Ptr& surface = nullptr,
                                              std::atomic<bool>* cancel = nullptr,
                                              std::function<void(int)> on_progress = nullptr);

        static FeatureResult VFHEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                            const Cloud::Ptr& surface = nullptr,
                                            std::atomic<bool>* cancel = nullptr,
                                            std::function<void(int)> on_progress = nullptr);

        static FeatureResult ESFEstimation(const Cloud::Ptr& cloud,
                                             std::atomic<bool>* cancel = nullptr,
                                             std::function<void(int)> on_progress = nullptr);

        static FeatureResult GASDEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                              int shgs, int shs, int interp,
                                              std::atomic<bool>* cancel = nullptr,
                                              std::function<void(int)> on_progress = nullptr);

        static FeatureResult GASDColorEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                                   int shgs, int shs, int interp,
                                                   int chgs, int chs, int cinterp,
                                                   std::atomic<bool>* cancel = nullptr,
                                                   std::function<void(int)> on_progress = nullptr);

        static FeatureResult RSDEstimation(const Cloud::Ptr& cloud, int nr_subdiv, double plane_radius,
                                            std::atomic<bool>* cancel = nullptr,
                                            std::function<void(int)> on_progress = nullptr);

        static FeatureResult GRSDEstimation(const Cloud::Ptr& cloud,
                                             std::atomic<bool>* cancel = nullptr,
                                             std::function<void(int)> on_progress = nullptr);

        static FeatureResult CRHEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                            std::atomic<bool>* cancel = nullptr,
                                            std::function<void(int)> on_progress = nullptr);

        static FeatureResult CVFHEstimation(const Cloud::Ptr& cloud, const Eigen::Vector3f& dir,
                                             float radius_normals, float d1, float d2, float d3,
                                             int min, bool normalize,
                                             std::atomic<bool>* cancel = nullptr,
                                             std::function<void(int)> on_progress = nullptr);

        static FeatureResult ShapeContext3DEstimation(const Cloud::Ptr& cloud,
                                                       double min_radius, double radius,
                                                       std::atomic<bool>* cancel = nullptr,
                                                       std::function<void(int)> on_progress = nullptr);

        static FeatureResult SHOTEstimation(const Cloud::Ptr& cloud,
                                             const ReferenceFrame::Ptr& lrf, float radius,
                                             const Cloud::Ptr& surface = nullptr,
                                             std::atomic<bool>* cancel = nullptr,
                                             std::function<void(int)> on_progress = nullptr);

        static FeatureResult SHOTColorEstimation(const Cloud::Ptr& cloud,
                                                   const ReferenceFrame::Ptr& lrf, float radius,
                                                   const Cloud::Ptr& surface = nullptr,
                                                   std::atomic<bool>* cancel = nullptr,
                                                   std::function<void(int)> on_progress = nullptr);

        static FeatureResult UniqueShapeContext(const Cloud::Ptr& cloud,
                                                 const ReferenceFrame::Ptr& lrf,
                                                 double min_radius, double pt_radius, double loc_radius,
                                                 std::atomic<bool>* cancel = nullptr,
                                                 std::function<void(int)> on_progress = nullptr);

        static LRFResult BOARDLocalReferenceFrameEstimation(const Cloud::Ptr& cloud,
                                                              float radius, bool find_holes,
                                                              float margin_thresh, int size,
                                                              float prob_thresh, float steep_thresh,
                                                              std::atomic<bool>* cancel = nullptr,
                                                              std::function<void(int)> on_progress = nullptr);

        static LRFResult FLARELocalReferenceFrameEstimation(const Cloud::Ptr& cloud,
                                                               float radius, float margin_thresh,
                                                               int min_neighbors_for_normal_axis,
                                                               int min_neighbors_for_tangent_axis,
                                                               std::atomic<bool>* cancel = nullptr,
                                                               std::function<void(int)> on_progress = nullptr);

        static LRFResult SHOTLocalReferenceFrameEstimation(const Cloud::Ptr& cloud,
                                                             float radius = 0.0f,
                                                             std::atomic<bool>* cancel = nullptr,
                                                             std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 边界点估计
         * @param cloud 输入点云（需要法线）
         * @param k 最近邻搜索数量
         * @param radius 搜索半径
         * @param angle 角度阈值（度）
         */
        static Cloud::Ptr BoundaryEstimation(const Cloud::Ptr& cloud,
                                              int k, double radius, double angle,
                                              std::atomic<bool>* cancel = nullptr,
                                              std::function<void(int)> on_progress = nullptr);
    };


} // namespace pw

#endif //POINTWORKS_FEATURES_H
