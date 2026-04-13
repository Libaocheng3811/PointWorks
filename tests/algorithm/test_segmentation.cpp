#include <gtest/gtest.h>
#include <segmentation.h>
#include "test_helpers.h"
#include "macros.h"

using namespace ct;

// ===== 欧式聚类 =====

TEST(SegmentationTest, Euclidean_TwoClusters) {
    auto pair = test_helpers::makeTwoClusters(500, 500, 10.0f, 0.1f, 42);

    auto result = Segmentation::EuclideanClusterExtraction(
        pair.cloud, false, 3.0, 10, 100000);

    // 应识别出至少 2 个簇
    EXPECT_GE(result.clouds.size(), 2u);

    // 总点数应与原始一致
    size_t total = 0;
    for (const auto& c : result.clouds) total += c->size();
    EXPECT_EQ(total, pair.cloud->size());
}

TEST(SegmentationTest, Euclidean_SingleCluster) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);

    auto result = Segmentation::EuclideanClusterExtraction(
        cloud, false, 3.0, 10, 100000);

    EXPECT_EQ(result.clouds.size(), 1u);
    EXPECT_EQ(result.clouds[0]->size(), cloud->size());
}

TEST(SegmentationTest, Euclidean_MinSizeFilter) {
    auto pair = test_helpers::makeTwoClusters(500, 500, 10.0f, 0.1f, 42);

    // min_cluster_size=600 → 小簇被过滤
    auto result = Segmentation::EuclideanClusterExtraction(
        pair.cloud, false, 3.0, 600, 100000);

    // 只保留大簇
    for (const auto& c : result.clouds) {
        EXPECT_GE(c->size(), 600u);
    }
}

// ===== SAC 分割 =====

TEST(SegmentationTest, SACSegmentation_Plane) {
    // 平面 + 噪声
    auto plane = test_helpers::makePlane(900, 10.0f, 10.0f, 0.0f, 42);
    auto noisy = test_helpers::addOutliers(plane, 100, 20.0f, 42);

    // PCL SAC 模型类型: 0=SACMODEL_PLANE
    // PCL SAC 方法类型: 0=SAC_RANSAC
    auto result = Segmentation::SACSegmentation(
        noisy, false, 0, 0, 0.01, 1000, 0.99, true, 0, 0);

    // 应返回至少一个分割结果
    ASSERT_GE(result.clouds.size(), 1u);
    EXPECT_GT(result.clouds[0]->size(), 0u);

    // 分割结果应有更少的点（离群点被去除）
    EXPECT_LT(result.clouds[0]->size(), noisy->size());
}

// ===== DBSCAN =====

TEST(SegmentationTest, DBSCAN_TwoClusters) {
    auto pair = test_helpers::makeTwoClusters(500, 500, 10.0f, 0.1f, 42);

    auto result = Segmentation::DBSCANClusterExtraction(
        pair.cloud, false, 2.0, 5, 10, 100000);

    EXPECT_GE(result.clouds.size(), 2u);
}

// ===== K-Means =====

TEST(SegmentationTest, KMeans_TwoClusters) {
    auto pair = test_helpers::makeTwoClusters(500, 500, 10.0f, 0.1f, 42);

    auto result = Segmentation::KMeansClusterExtraction(
        pair.cloud, 2, 100);

    // K=2 应返回 2 个簇
    EXPECT_EQ(result.clouds.size(), 2u);

    // 总点数应一致
    size_t total = 0;
    for (const auto& c : result.clouds) total += c->size();
    EXPECT_EQ(total, pair.cloud->size());
}

// ===== 区域生长（需要法线） =====

TEST(SegmentationTest, RegionGrowing_PlaneWithNormals) {
    // 生成平面并估计法线
    auto cloud = test_helpers::makePlane(500, 10.0f, 10.0f, 0.0f, 42);

    auto normals_result = Normals::estimate(cloud, 30, 0.0, 0, 0, 0);
    ASSERT_TRUE(normals_result.error_msg.empty());

    auto result = Segmentation::RegionGrowing(
        normals_result.cloud, false,
        10, 100000,
        false, false, false,
        30.0f, 0.0f, 0.0f, 30);

    // 平面应被识别为一个整体簇
    ASSERT_GE(result.clouds.size(), 1u);
}

// ===== SAC 球面分割 =====

TEST(SegmentationTest, SACSegmentation_Sphere) {
    auto sphere = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);
    auto noisy = test_helpers::addOutliers(sphere, 50, 30.0f, 42);

    // PCL SAC 模型: 2=SACMODEL_SPHERE
    auto result = Segmentation::SACSegmentation(
        noisy, false, 2, 0, 0.05, 1000, 0.99, true, 4.0, 6.0);

    ASSERT_GE(result.clouds.size(), 1u);
    EXPECT_GT(result.clouds[0]->size(), 0u);
}

// ===== 取消支持 =====

TEST(SegmentationTest, Cancel_NoCrash) {
    auto cloud = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);
    std::atomic<bool> cancel(true);

    auto result = Segmentation::EuclideanClusterExtraction(
        cloud, false, 3.0, 10, 100000, &cancel);
    // 不崩溃即可
}
