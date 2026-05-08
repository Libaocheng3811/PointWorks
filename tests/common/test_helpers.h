#pragma once

#include <cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/model_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include <vector>
#include <cmath>

namespace test_helpers {

// ============ 基础几何体生成 ============

/**
 * @brief 生成 XY 平面上均匀分布的点云
 * @param n       点数
 * @param width   X 方向范围
 * @param depth   Y 方向范围
 * @param noise   高斯噪声标准差（0 = 无噪声，仅影响 Z）
 * @param seed    随机种子
 */
pw::Cloud::Ptr makePlane(int n = 1000,
                         float width = 10.0f, float depth = 10.0f,
                         float noise = 0.0f, unsigned seed = 42);

/**
 * @brief 生成球面均匀分布的点云
 * @param n       点数
 * @param radius  球半径
 * @param noise   高斯噪声标准差
 * @param seed    随机种子
 */
pw::Cloud::Ptr makeSphere(int n = 1000,
                          float radius = 5.0f,
                          float noise = 0.0f, unsigned seed = 42);

/**
 * @brief 生成立方体表面均匀分布的点云
 * @param n       点数
 * @param size    立方体边长
 * @param noise   高斯噪声标准差
 * @param seed    随机种子
 */
pw::Cloud::Ptr makeCube(int n = 1000,
                        float size = 10.0f,
                        float noise = 0.0f, unsigned seed = 42);

/**
 * @brief 生成圆柱体表面均匀分布的点云
 * @param n       点数
 * @param radius  圆柱半径
 * @param height  圆柱高度
 * @param noise   高斯噪声标准差
 * @param seed    随机种子
 */
pw::Cloud::Ptr makeCylinder(int n = 1000,
                            float radius = 5.0f, float height = 10.0f,
                            float noise = 0.0f, unsigned seed = 42);

// ============ 特殊场景生成 ============

/**
 * @brief 生成两个部分重叠的球体（用于配准测试）
 */
struct OverlappingPair {
    pw::Cloud::Ptr source;
    pw::Cloud::Ptr target;
    Eigen::Matrix4f ground_truth_transform;  // source -> target 的真实变换
};

OverlappingPair makeOverlappingSpheres(int n = 500,
                                       float radius = 5.0f,
                                       float overlap = 0.7f,
                                       float angle_deg = 30.0f,
                                       unsigned seed = 42);

/**
 * @brief 给点云添加离群点
 * @param cloud         原始点云
 * @param n_outliers    离群点数量
 * @param outlier_range 离群点坐标范围
 * @param seed          随机种子
 */
pw::Cloud::Ptr addOutliers(pw::Cloud::Ptr cloud,
                           int n_outliers = 100,
                           float outlier_range = 50.0f,
                           unsigned seed = 42);

/**
 * @brief 生成两个聚类（用于分割测试）
 */
struct ClusterPair {
    pw::Cloud::Ptr cloud;         // 合并的点云
    std::vector<int> labels;      // 每个点的真实标签 (0 或 1)
};

ClusterPair makeTwoClusters(int n1 = 500, int n2 = 500,
                            float separation = 5.0f,
                            float noise = 0.1f, unsigned seed = 42);

/**
 * @brief 对点云施加已知变换（用于配准测试）
 */
pw::Cloud::Ptr applyTransform(pw::Cloud::Ptr cloud,
                              const Eigen::Matrix4f& transform);

// ============ 验证辅助 ============

/**
 * @brief 检查点云是否近似共面（拟合平面后计算 RMS 残差）
 * @param max_residual  允许的最大 RMS 残差
 */
bool isApproximatelyPlanar(pw::Cloud::Ptr cloud, float max_residual = 0.5f);

/**
 * @brief 检查点云是否近似在球面上
 * @param expected_radius 期望的球面半径
 * @param tolerance       半径允许偏差
 */
bool isApproximatelySpherical(pw::Cloud::Ptr cloud, float expected_radius,
                              float tolerance = 0.5f);

/**
 * @brief 计算点云质心
 */
Eigen::Vector4f computeCentroid(pw::Cloud::Ptr cloud);

/**
 * @brief 计算 RMSE
 */
float computeRMSE(const std::vector<float>& errors);

/**
 * @brief 矩阵近似相等（4x4）
 */
bool matrixApproxEqual(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b,
                       float tolerance = 1e-3f);

}  // namespace test_helpers
