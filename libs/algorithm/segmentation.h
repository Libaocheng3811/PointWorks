#ifndef CT_MODULES_SEGMENTATION_H
#define CT_MODULES_SEGMENTATION_H

#include "core/cloud.h"

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

#include <functional>
#include <atomic>

namespace ct
{
    typedef pcl::PointNormal                                    PointN;
    typedef pcl::PointIndices                                   PointIndices;
    typedef pcl::PointIndicesPtr                                PointIndicesPtr;
    typedef pcl::ModelCoefficients                              ModelCoefficients;
    typedef std::vector<PointIndices>                           IndicesClusters;
    typedef std::shared_ptr<std::vector<pcl::PointIndices>>     IndicesClustersPtr;
    typedef std::function<bool(const PointXYZRGBN&, const PointXYZRGBN&, float)> ConditionFunction;

    struct SegmentationResult {
        std::vector<Cloud::Ptr> clouds;
        std::vector<float> labels;  // per-point cluster label, size == cloud->pointCount()
        float time_ms = 0;
        ModelCoefficients::Ptr coefficients;
    };

    class Segmentation
    {
    public:
        /**
         * @brief 表示 Sample Consensus 方法和模型的 Nodelet
         * 分割类，从某种意义上说，它只是为基于 SAC 的通用分割 创建了一个 Nodelet
         * 包装器
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param model 要使用的模型类型（用户给定参数）
         * @param method 要使用的样本共识方法的类型（用户给定的参数）
         * @param threshold 到模型阈值的距离（用户给定参数）
         * @param max_iterations 设置放弃前的最大迭代次数
         * @param probability 设置至少选择一个没有异常值的样本的概率
         * @param optimize 如果需要进行系数细化，则设置为 true
         * @param min_radius 设置模型的最小允许半径限制（适用于估计半径的模型）
         * @param max_radius 设置模型的最大允许半径限制（适用于估计半径的模型）
         */
        static SegmentationResult SACSegmentation(const Cloud::Ptr& cloud, bool negative,
                                                   int model, int method, double threshold,
                                                   int max_iterations, double probability,
                                                   bool optimize, double min_radius, double max_radius,
                                                   std::atomic<bool>* cancel = nullptr,
                                                   std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 表示 Sample Consensus 方法和模型的 Nodelet
         * 分割类，从某种意义上说，它只是为基于 SAC 的通用分割 创建了一个 Nodelet
         * 包装器（基于法线）
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param model 要使用的模型类型（用户给定参数）
         * @param method 要使用的样本共识方法的类型（用户给定的参数）
         * @param threshold 到模型阈值的距离（用户给定参数）
         * @param max_iterations 设置放弃前的最大迭代次数
         * @param probability 设置至少选择一个没有异常值的样本的概率
         * @param optimize 如果需要进行系数细化，则设置为 true
         * @param min_radius 设置模型的最小允许半径限制（适用于估计半径的模型）
         * @param max_radius 设置模型的最大允许半径限制（适用于估计半径的模型）
         * @param distance_weight 设置相对权重（介于 0 和 1 之间）以给出点法线和平面法线之间的角距离（0 到 pi/2）
         * @param d 设置期望平面模型与原点的距离
         */
        static SegmentationResult SACSegmentationFromNormals(const Cloud::Ptr& cloud, bool negative,
                                                              int model, int method, double threshold,
                                                              int max_iterations, double probability,
                                                              bool optimize, double min_radius, double max_radius,
                                                              double distance_weight, double d,
                                                              std::atomic<bool>* cancel = nullptr,
                                                              std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 表示用于欧几里得意义上的聚类提取的分割类
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param tolerance 将空间聚类容差设置为 L2 欧几里得空间中的度量
         * @param min_cluster_size 设置集群需要包含的最小点数才能被视为有效
         * @param max_cluster_size 设置集群需要包含的最大点数才能被视为有效
         */
        static SegmentationResult EuclideanClusterExtraction(const Cloud::Ptr& cloud, bool negative,
                                                              double tolerance, int min_cluster_size,
                                                              int max_cluster_size,
                                                              std::atomic<bool>* cancel = nullptr,
                                                              std::function<void(int)> on_progress = nullptr);

        /**
         * @brief DBSCAN 聚类
         * @param cloud 输入点云
         * @param negative 设置是否反转
         * @param eps 邻域半径
         * @param min_pts 核心点最小邻居数
         * @param min_cluster_size 最小聚类大小
         * @param max_cluster_size 最大聚类大小
         * @param normal_weight 法线距离权重
         * @param color_weight 颜色距离权重
         */
        static SegmentationResult DBSCANClusterExtraction(const Cloud::Ptr& cloud, bool negative,
                                                           double eps, int min_pts,
                                                           int min_cluster_size, int max_cluster_size,
                                                           double normal_weight = 0.0,
                                                           double color_weight = 0.0,
                                                           std::atomic<bool>* cancel = nullptr,
                                                           std::function<void(int)> on_progress = nullptr);

        /**
         * @brief K-Means 聚类
         * @param cloud 输入点云
         * @param k 聚类数
         * @param max_iterations 最大迭代次数
         * @param normal_weight 法线距离权重
         * @param color_weight 颜色距离权重
         */
        static SegmentationResult KMeansClusterExtraction(const Cloud::Ptr& cloud,
                                                          int k, int max_iterations,
                                                          double normal_weight = 0.0,
                                                          double color_weight = 0.0,
                                                          std::atomic<bool>* cancel = nullptr,
                                                          std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 实现用于分割的众所周知的区域增长算法
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param min_cluster_size 设置集群需要包含的最小点数才能被视为有效
         * @param max_cluster_size 设置集群需要包含的最大点数才能被视为有效
         * @param smooth_mode 允许打开/关闭平滑约束
         * @param curvature_test 允许打开/关闭曲率测试
         * @param residual_test 允许打开/关闭剩余测试
         * @param smoothness_threshold 允许设置用于测试点的平滑度阈值
         * @param residual_threshold 允许设置用于测试点的残差阈值
         * @param curvature_threshold 允许设置用于测试点的曲率阈值
         * @param neighbours 允许设置邻居的数量
         */
        static SegmentationResult RegionGrowing(const Cloud::Ptr& cloud, bool negative,
                                                 int min_cluster_size, int max_cluster_size,
                                                 bool smooth_mode, bool curvature_test, bool residual_test,
                                                 float smoothness_threshold, float residual_threshold,
                                                 float curvature_threshold, int neighbours,
                                                 std::atomic<bool>* cancel = nullptr,
                                                 std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 从指定种子点进行区域生长
         * @param cloud 输入点云
         * @param negative 设置是否反转
         * @param seed_index 种子点索引
         * @param min_cluster_size 最小聚类大小
         * @param max_cluster_size 最大聚类大小
         * @param smooth_mode 允许打开/关闭平滑约束
         * @param curvature_test 允许打开/关闭曲率测试
         * @param residual_test 允许打开/关闭剩余测试
         * @param smoothness_threshold 平滑度阈值
         * @param residual_threshold 残差阈值
         * @param curvature_threshold 曲率阈值
         * @param neighbours 邻居数
         */
        static SegmentationResult RegionGrowingFromSeed(const Cloud::Ptr& cloud, bool negative,
                                                         int seed_index,
                                                         int min_cluster_size, int max_cluster_size,
                                                         bool smooth_mode, bool curvature_test, bool residual_test,
                                                         float smoothness_threshold, float residual_threshold,
                                                         float curvature_threshold, int neighbours,
                                                         std::atomic<bool>* cancel = nullptr,
                                                         std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 实现用于基于点颜色进行分割的众所周知的区域增长算法
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param min_cluster_size 设置集群需要包含的最小点数才能被视为有效
         * @param max_cluster_size 设置集群需要包含的最大点数才能被视为有效
         * @param smooth_mode 允许打开/关闭平滑约束
         * @param curvature_test 允许打开/关闭曲率测试
         * @param residual_test 允许打开/关闭剩余测试
         * @param smoothness_threshold 允许设置用于测试点的平滑度阈值
         * @param residual_threshold 允许设置用于测试点的残差阈值
         * @param curvature_threshold 允许设置用于测试点的曲率阈值
         * @param neighbours 允许设置邻居的数量
         * @param pt_thresh 指定点之间颜色测试的阈值
         * @param re_thresh 指定区域之间颜色测试的阈值
         * @param dis_thresh 允许设置距离阈值
         * @param nghbr_number 允许设置用于查找相邻段的邻居数
         */
        static SegmentationResult RegionGrowingRGB(const Cloud::Ptr& cloud, bool negative,
                                                    int min_cluster_size, int max_cluster_size,
                                                    bool smooth_mode, bool curvature_test, bool residual_test,
                                                    float smoothness_threshold, float residual_threshold,
                                                    float curvature_threshold, int neighbours,
                                                    float pt_thresh, float re_thresh,
                                                    float dis_thresh, int nghbr_number,
                                                    std::atomic<bool>* cancel = nullptr,
                                                    std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 实现基于体素结构、法线和 rgb 值的超体素算法。
         * @param cloud 输入点云
         * @param voxel_resolution 设置八叉树体素的分辨率
         * @param seed_resolution 设置八叉树种子体素的分辨率
         * @param color_importance 设置颜色对超体素的重要性
         * @param spatial_importance 设置空间距离对超体素的重要性
         * @param normal_importance 设置超体素标量正态积的重要性
         * @param camera_transform 设置是否使用单相机变换
         */
        static SegmentationResult SupervoxelClustering(const Cloud::Ptr& cloud,
                                                        float voxel_resolution, float seed_resolution,
                                                        float color_importance, float spatial_importance,
                                                        float normal_importance, bool camera_transform,
                                                        std::atomic<bool>* cancel = nullptr,
                                                        std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 基于欧几里得距离和用户定义的聚类执行分割（DoN 法线差分）
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param mean_radius 平均半径
         * @param scale1 小尺度
         * @param scale2 大尺度
         * @param threshold 阈值
         * @param segradius 分割半径
         * @param minClusterSize 最小聚类点数
         * @param maxClusterSize 最大聚类点数
         */
        static SegmentationResult DonSegmentation(const Cloud::Ptr& cloud, bool negative,
                                                   double mean_radius, double scale1, double scale2,
                                                   double threshold, double segradius,
                                                   int minClusterSize, int maxClusterSize,
                                                   std::atomic<bool>* cancel = nullptr,
                                                   std::function<void(int)> on_progress = nullptr);

        /**
         * @brief MinCut 分割
         * @param cloud 输入点云
         * @param sigma
         * @param radius
         * @param weight
         * @param neighbour_number
         */
        static SegmentationResult MinCutSegmentation(const Cloud::Ptr& cloud,
                                                      double sigma, double radius, double weight,
                                                      int neighbour_number,
                                                      std::atomic<bool>* cancel = nullptr,
                                                      std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 渐进式形态学滤波（地面分割）
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param max_window_size 最大窗口大小
         * @param slope 用于计算高度阈值的坡度值
         * @param max_distance 被视为地面回波的最大高度
         * @param initial_distance 被视为地面回波的初始高度
         * @param cell_size 单元格大小
         * @param base 用于计算渐进窗口大小的基数
         */
        static SegmentationResult MorphologicalFilter(const Cloud::Ptr& cloud, bool negative,
                                                       int max_window_size, float slope,
                                                       float max_distance, float initial_distance,
                                                       float cell_size, float base,
                                                       std::atomic<bool>* cancel = nullptr,
                                                       std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 种子色调分割
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param tolerance 将空间聚类容差设置为 L2 欧几里得空间中的度量
         * @param delta_hue 设置色相的容差
         */
        static SegmentationResult SeededHueSegmentation(const Cloud::Ptr& cloud, bool negative,
                                                         double tolerance, float delta_hue,
                                                         std::atomic<bool>* cancel = nullptr,
                                                         std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 获得两个空间对齐的点云之间的差异，并返回最大给定距离阈值的它们之间的差异
         * @param cloud 输入点云
         * @param tar_cloud 提供指向目标数据集的指针，将与输入云进行比较
         * @param sqr_threshold 设置两个输入数据集中对应点之间的最大距离容差（平方）
         */
        static SegmentationResult SegmentDifferences(const Cloud::Ptr& cloud,
                                                      const Cloud::Ptr& tar_cloud,
                                                      double sqr_threshold,
                                                      std::atomic<bool>* cancel = nullptr,
                                                      std::function<void(int)> on_progress = nullptr);

        /**
         * @brief 使用一组表示平面模型的点索引，并与给定的高度一起生成 3D 多边形棱柱
         * @param cloud 输入点云
         * @param negative 设置是应用点过滤的常规条件，还是应用倒置条件
         * @param hull 提供指向输入平面船体数据集的指针
         * @param height_min height_max 设置高度限制
         * @param vpx vpy vpz 设置视点
         */
        static SegmentationResult ExtractPolygonalPrismData(const Cloud::Ptr& cloud, bool negative,
                                                             const Cloud::Ptr& hull,
                                                             double height_min, double height_max,
                                                             float vpx, float vpy, float vpz,
                                                             std::atomic<bool>* cancel = nullptr,
                                                             std::function<void(int)> on_progress = nullptr);
    };

}  // namespace ct

#endif  // CT_MODULES_SEGMENTATION_H
