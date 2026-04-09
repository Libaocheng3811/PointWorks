//
// Created by LBC on 2025/1/10.
//

#ifndef MODULES_REGISTRATION_H
#define MODULES_REGISTRATION_H

#include "core/cloud.h"
#include "core/exports.h"
#include "features.h"

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation.h>

#include <functional>
#include <atomic>
#include <limits>
#include <vector>

namespace ct {
    typedef pcl::registration::TransformationEstimation<PointXYZRGBN, PointXYZRGBN, float> TransEst;
    typedef pcl::registration::CorrespondenceEstimationBase<PointXYZRGBN, PointXYZRGBN, float> CorreEst;
    typedef pcl::registration::CorrespondenceRejector CorreRej;
    typedef pcl::CorrespondencesPtr CorrespondencesPtr;

    // --- Result structs ---

    struct RegistrationResult {
        bool success = false;
        Cloud::Ptr aligned_cloud;
        double score = 0;
        Eigen::Matrix4f matrix;
        float time_ms = 0;
    };

    struct CorrespondenceResult {
        CorrespondencesPtr correspondences;
        CorreEst::Ptr ce;
        float time_ms = 0;
    };

    struct RejectorResult {
        CorrespondencesPtr correspondences;
        CorreRej::Ptr cr;
        float time_ms = 0;
    };

    struct TransformationResult {
        Eigen::Matrix4f matrix;
        TransEst::Ptr te;
        float time_ms = 0;
    };

    // --- Configuration ---

    struct RegistrationParams {
        int max_iterations = 10;
        int ransac_iterations = 0;
        double inlier_threshold = 0.05;
        double distance_threshold = std::sqrt(std::numeric_limits<double>::max());
        double transformation_epsilon = 0.0;
        double transformation_rotation_epsilon = 0.0;
        double euclidean_fitness_epsilon = -std::numeric_limits<double>::max();
    };

    struct RegistrationContext {
        Cloud::Ptr source_cloud;
        Cloud::Ptr target_cloud;
        CorrespondencesPtr correspondences;
        TransEst::Ptr transformation_estimation;
        CorreEst::Ptr correspondence_estimation;
        std::map<int, CorreRej::Ptr> correspondence_rejectors;
        RegistrationParams params;
    };

    // --- Constrained Point-Pairs Alignment ---

    enum class RotationConstraint {
        NONE,       // 无旋转（纯平移）
        X_ONLY,     // 仅绕 X 轴旋转
        Y_ONLY,     // 仅绕 Y 轴旋转
        Z_ONLY,     // 仅绕 Z 轴旋转
        XY,         // 绕 X、Y 轴旋转
        XZ,         // 绕 X、Z 轴旋转
        YZ,         // 绕 Y、Z 轴旋转
        XYZ         // 全自由度旋转（默认）
    };

    struct ConstrainedTransformParams {
        RotationConstraint rotation = RotationConstraint::XYZ;
        bool tx_enabled = true;
        bool ty_enabled = true;
        bool tz_enabled = true;
        bool adjust_scale = false; // 7 DOF 相似变换
    };

    struct PointPairErrorResult {
        Eigen::Matrix4f matrix;
        double rms = 0.0;
        std::vector<Eigen::Vector3f> deltas;  // 每对的 Delta XYZ
        std::vector<double> errors;            // 每对的欧氏距离误差
        double scale = 1.0;                    // 缩放因子
    };

    class Registration {
    public:
        // --- Correspondence Estimation (non-template) ---

        static CorrespondenceResult CorrespondenceEstimationBackProjection(const RegistrationContext& ctx, int k,
                                                                           std::atomic<bool>* cancel = nullptr,
                                                                           std::function<void(int)> on_progress = nullptr);

        static CorrespondenceResult CorrespondenceEstimationNormalShooting(const RegistrationContext& ctx, int k,
                                                                            std::atomic<bool>* cancel = nullptr,
                                                                            std::function<void(int)> on_progress = nullptr);

        static CorrespondenceResult CorrespondenceEstimationOrganizedProjection(
                const RegistrationContext& ctx, float fx, float fy, float cx, float cy,
                const Eigen::Matrix4f& src_to_tgt_trans, float depth_threshold,
                std::atomic<bool>* cancel = nullptr, std::function<void(int)> on_progress = nullptr);

        // --- Correspondence Estimation (template) ---

        template<typename Type>
        static CorrespondenceResult CorrespondenceEstimation(
                const typename pcl::PointCloud<Type>::Ptr& source,
                const typename pcl::PointCloud<Type>::Ptr& target)
        {
            TicToc time;
            time.tic();
            pcl::registration::CorrespondenceEstimation<Type, Type, float> ce;
            pcl::CorrespondencesPtr corr(new pcl::Correspondences);
            ce.setInputSource(source);
            ce.setInputTarget(target);
            typename pcl::search::KdTree<Type>::Ptr target_tree(new pcl::search::KdTree<Type>);
            typename pcl::search::KdTree<Type>::Ptr source_tree(new pcl::search::KdTree<Type>);
            ce.setSearchMethodSource(source_tree);
            ce.setSearchMethodTarget(target_tree);
            ce.determineCorrespondences(*corr);
            return {corr, nullptr, (float)time.toc()};
        }

        // --- Correspondence Rejection (non-template) ---

        static RejectorResult CorrespondenceRejectorDistance(const RegistrationContext& ctx,
                                                             const CorrespondencesPtr& input_corr, float distance,
                                                             std::atomic<bool>* cancel = nullptr,
                                                             std::function<void(int)> on_progress = nullptr);

        static RejectorResult CorrespondenceRejectorMedianDistance(const RegistrationContext& ctx,
                                                                     const CorrespondencesPtr& input_corr, double factor,
                                                                     std::atomic<bool>* cancel = nullptr,
                                                                     std::function<void(int)> on_progress = nullptr);

        static RejectorResult CorrespondenceRejectorOneToOne(const RegistrationContext& ctx,
                                                               const CorrespondencesPtr& input_corr,
                                                               std::atomic<bool>* cancel = nullptr,
                                                               std::function<void(int)> on_progress = nullptr);

        static RejectorResult CorrespondenceRejectionOrganizedBoundary(const RegistrationContext& ctx,
                                                                        const CorrespondencesPtr& input_corr, int val,
                                                                        std::atomic<bool>* cancel = nullptr,
                                                                        std::function<void(int)> on_progress = nullptr);

        static RejectorResult CorrespondenceRejectorPoly(const RegistrationContext& ctx,
                                                           const CorrespondencesPtr& input_corr,
                                                           int cardinality, float similarity_threshold, int iterations,
                                                           std::atomic<bool>* cancel = nullptr,
                                                           std::function<void(int)> on_progress = nullptr);

        static RejectorResult CorrespondenceRejectorSampleConsensus(const RegistrationContext& ctx,
                                                                      const CorrespondencesPtr& input_corr,
                                                                      double threshold, int max_iterations, bool refine,
                                                                      std::atomic<bool>* cancel = nullptr,
                                                                      std::function<void(int)> on_progress = nullptr);

        static RejectorResult CorrespondenceRejectorSurfaceNormal(const RegistrationContext& ctx,
                                                                    const CorrespondencesPtr& input_corr, double threshold,
                                                                    std::atomic<bool>* cancel = nullptr,
                                                                    std::function<void(int)> on_progress = nullptr);

        static RejectorResult CorrespondenceRejectorTrimmed(const RegistrationContext& ctx,
                                                              const CorrespondencesPtr& input_corr,
                                                              float ratio, int min_corre,
                                                              std::atomic<bool>* cancel = nullptr,
                                                              std::function<void(int)> on_progress = nullptr);

        static RejectorResult CorrespondenceRejectorVarTrimmed(const RegistrationContext& ctx,
                                                                const CorrespondencesPtr& input_corr,
                                                                double min_ratio, double max_ratio,
                                                                std::atomic<bool>* cancel = nullptr,
                                                                std::function<void(int)> on_progress = nullptr);

        // --- Correspondence Rejection (template) ---

        template<typename Feature>
        static RejectorResult CorrespondenceRejectorFeatures(
                const RegistrationContext& ctx,
                const typename pcl::PointCloud<PointXYZRGBN>::Ptr& source,
                const typename pcl::PointCloud<PointXYZRGBN>::Ptr& target,
                double thresh, const std::string& key,
                const CorrespondencesPtr& input_corr)
        {
            TicToc time;
            time.tic();
            typename pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
            pcl::CorrespondencesPtr corr(new pcl::Correspondences);

            pcl::registration::CorrespondenceRejectorFeatures::Ptr cj(new pcl::registration::CorrespondenceRejectorFeatures);
            cj->setSourceFeature<Feature>(source, ctx.source_cloud->id());
            cj->setTargetFeature<Feature>(target, ctx.target_cloud->id());
            cj->setDistanceThreshold<Feature>(thresh, key);
            cj->setInputCorrespondences(input_corr);
            cj->getRemainingCorrespondences(*input_corr, *corr);
            return {corr, time.toc(), cj};
        }

        // --- Transformation Estimation ---

        static TransformationResult TransformationEstimation2D(const RegistrationContext& ctx,
                                                                 const CorrespondencesPtr& input_corr,
                                                                 std::atomic<bool>* cancel = nullptr,
                                                                 std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimation3Point(const RegistrationContext& ctx,
                                                                     const CorrespondencesPtr& input_corr,
                                                                     std::atomic<bool>* cancel = nullptr,
                                                                     std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimationDualQuaternion(const RegistrationContext& ctx,
                                                                             const CorrespondencesPtr& input_corr,
                                                                             std::atomic<bool>* cancel = nullptr,
                                                                             std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimationLM(const RegistrationContext& ctx,
                                                                 const CorrespondencesPtr& input_corr,
                                                                 std::atomic<bool>* cancel = nullptr,
                                                                 std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimationPointToPlane(const RegistrationContext& ctx,
                                                                          const CorrespondencesPtr& input_corr,
                                                                          std::atomic<bool>* cancel = nullptr,
                                                                          std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimationPointToPlaneLLS(const RegistrationContext& ctx,
                                                                             const CorrespondencesPtr& input_corr,
                                                                             std::atomic<bool>* cancel = nullptr,
                                                                             std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimationPointToPlaneLLSWeighted(const RegistrationContext& ctx,
                                                                                      const CorrespondencesPtr& input_corr,
                                                                                      std::atomic<bool>* cancel = nullptr,
                                                                                      std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimationPointToPlaneWeighted(const RegistrationContext& ctx,
                                                                                 const CorrespondencesPtr& input_corr,
                                                                                 std::atomic<bool>* cancel = nullptr,
                                                                                 std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimationSVD(const RegistrationContext& ctx,
                                                                  const CorrespondencesPtr& input_corr,
                                                                  std::atomic<bool>* cancel = nullptr,
                                                                  std::function<void(int)> on_progress = nullptr);

        static TransformationResult TransformationEstimationSymmetricPointToPlaneLLS(
                const RegistrationContext& ctx, const CorrespondencesPtr& input_corr,
                bool enforce_same_direction_normals,
                std::atomic<bool>* cancel = nullptr, std::function<void(int)> on_progress = nullptr);

        static void TransformationValidationEuclidean(const RegistrationContext& ctx,
                                                       const pcl::Correspondence& corr, bool from_normals);

        // --- Registration ---

        static RegistrationResult IterativeClosestPoint(const RegistrationContext& ctx,
                                                         bool use_recip_corre,
                                                         std::atomic<bool>* cancel = nullptr,
                                                         std::function<void(int)> on_progress = nullptr);

        static RegistrationResult IterativeClosestPointWithNormals(const RegistrationContext& ctx,
                                                                     bool use_recip_corre,
                                                                     bool use_symmetric_objective,
                                                                     bool enforce_same_direction_normals,
                                                                     std::atomic<bool>* cancel = nullptr,
                                                                     std::function<void(int)> on_progress = nullptr);

        static RegistrationResult IterativeClosestPointNonLinear(const RegistrationContext& ctx,
                                                                  bool use_recip_corre,
                                                                  std::atomic<bool>* cancel = nullptr,
                                                                  std::function<void(int)> on_progress = nullptr);

        static RegistrationResult GeneralizedIterativeClosestPoint(const RegistrationContext& ctx,
                                                                     int k, int max,
                                                                     double tra_tolerance, double rol_tolerance,
                                                                     bool use_recip_corre,
                                                                     std::atomic<bool>* cancel = nullptr,
                                                                     std::function<void(int)> on_progress = nullptr);

        static RegistrationResult FPCSInitialAlignment(const RegistrationContext& ctx,
                                                         float delta, bool normalize, float approx_overlap,
                                                         float score_threshold, int nr_samples,
                                                         float max_norm_diff, int max_runtime,
                                                         std::atomic<bool>* cancel = nullptr,
                                                         std::function<void(int)> on_progress = nullptr);

        static RegistrationResult KFPCSInitialAlignment(const RegistrationContext& ctx,
                                                          float delta, bool normalize, float approx_overlap,
                                                          float score_threshold, int nr_samples,
                                                          float max_norm_diff, int max_runtime,
                                                          float upper_trl_boundary, float lower_trl_boundary,
                                                          float lambda,
                                                          std::atomic<bool>* cancel = nullptr,
                                                          std::function<void(int)> on_progress = nullptr);

        static RegistrationResult NormalDistributionsTransform(const RegistrationContext& ctx,
                                                                float resolution, double step_size,
                                                                double outlier_ratio,
                                                                std::atomic<bool>* cancel = nullptr,
                                                                std::function<void(int)> on_progress = nullptr);

        // --- Registration (template) ---

        template <typename Feature>
        static RegistrationResult SampleConsensusInitialAlignment(
                const RegistrationContext& ctx,
                const typename pcl::PointCloud<Feature>::Ptr& source,
                const typename pcl::PointCloud<Feature>::Ptr& target,
                float min_sample_distance, int nr_samples, int k)
        {
            TicToc time;
            time.tic();
            Cloud::Ptr ail_cloud(new Cloud);
            pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
            pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
            pcl::SampleConsensusInitialAlignment<PointXYZRGBN, PointXYZRGBN, Feature> reg;

            reg.setInputTarget(ctx.target_cloud->toPCL_XYZRGBN());
            reg.setInputSource(ctx.source_cloud->toPCL_XYZRGBN());
            reg.setSourceFeatures(source);
            reg.setTargetFeatures(target);
            reg.setSearchMethodTarget(target_tree);
            reg.setSearchMethodSource(source_tree);
            if(ctx.transformation_estimation!=nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
            if(ctx.correspondence_estimation!=nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);
            for (auto& cj : ctx.correspondence_rejectors) reg.addCorrespondenceRejector(cj.second);
            reg.setMaximumIterations(ctx.params.max_iterations);
            reg.setRANSACIterations(ctx.params.ransac_iterations);
            reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
            reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
            reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
            reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
            reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

            reg.setCorrespondenceRandomness(k);
            reg.setMinSampleDistance(min_sample_distance);
            reg.setNumberOfSamples(nr_samples);

            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_aligned(new pcl::PointCloud<PointXYZRGBN>);
            reg.align(*pcl_aligned);

            ail_cloud = Cloud::fromPCL_XYZRGBN(*pcl_aligned);
            return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                    reg.getFinalTransformation(), (float)time.toc()};
        }

        template<typename Feature>
        static RegistrationResult SampleConsensusPrerejective(
                const RegistrationContext& ctx,
                const typename pcl::PointCloud<Feature>::Ptr& source,
                const typename pcl::PointCloud<Feature>::Ptr& target,
                int nr_samples, int k, float similarity_threshold, float inlier_fraction)
        {
            TicToc time;
            time.tic();
            pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
            pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
            pcl::SampleConsensusPrerejective<PointXYZRGBN, PointXYZRGBN, Feature> reg;

            reg.setInputSource(ctx.source_cloud->toPCL_XYZRGBN());
            reg.setInputTarget(ctx.target_cloud->toPCL_XYZRGBN());
            reg.setSourceFeatures(source);
            reg.setTargetFeatures(target);
            reg.setSearchMethodTarget(target_tree);
            reg.setSearchMethodSource(source_tree);
            if (ctx.transformation_estimation != nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
            if (ctx.correspondence_estimation != nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);
            for (auto &cj : ctx.correspondence_rejectors)
                reg.addCorrespondenceRejector(cj.second);
            reg.setMaximumIterations(ctx.params.max_iterations);
            reg.setRANSACIterations(ctx.params.ransac_iterations);
            reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
            reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
            reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
            reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
            reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

            reg.setCorrespondenceRandomness(k);
            reg.setSimilarityThreshold(similarity_threshold);
            reg.setNumberOfSamples(nr_samples);
            reg.setInlierFraction(inlier_fraction);

            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_aligned(new pcl::PointCloud<PointXYZRGBN>);
            reg.align(*pcl_aligned);

            Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*pcl_aligned);
            return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                    reg.getFinalTransformation(), (float)time.toc()};
        }

        // Helper: DataContainer
        static double DataContainer(const RegistrationContext& ctx,
                                    const pcl::Correspondence& corr, bool from_normals);

        // --- Constrained Point-Pairs Registration ---

        /**
         * @brief 带约束的点对配准变换估计（SVD + 旋转/平移约束投影）
         * @param source_points 源点坐标
         * @param target_points 目标点坐标（与 source 一一对应）
         * @param params 约束参数
         * @return 变换矩阵、RMS、逐点误差
         */
        static PointPairErrorResult ConstrainedPointPairsRegistration(
            const std::vector<Eigen::Vector3f>& source_points,
            const std::vector<Eigen::Vector3f>& target_points,
            const ConstrainedTransformParams& params);
    };
}

#endif //MODULES_REGISTRATION_H
