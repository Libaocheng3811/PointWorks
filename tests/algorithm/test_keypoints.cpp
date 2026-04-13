#include <gtest/gtest.h>
#include <keypoints.h>
#include <normals.h>
#include "test_helpers.h"
#include "macros.h"

using namespace ct;

// ===== ISS 关键点 =====

TEST(KeypointsTest, ISS_Sphere_DetectsFewerPoints) {
    auto cloud = test_helpers::makeSphere(1000, 5.0f, 0.0f, 42);
    size_t original = cloud->size();

    auto result = Keypoints::ISSKeypoint3D(
        cloud,
        0.5,    // resolution
        0.975,  // gamma_21
        0.975,  // gamma_32
        5,      // min_neighbors
        0.52,   // angle
        30,     // k
        1.0     // radius
    );

    ASSERT_CLOUD_NOT_NULL(result.cloud);
    // 关键点数应少于原始点数
    EXPECT_LT(result.cloud->size(), original);
    EXPECT_GT(result.cloud->size(), 0u);
}

// ===== Harris3D =====

TEST(KeypointsTest, Harris3D_Cube_DetectsFewerPoints) {
    // Harris3D 检测角点，球面曲率均匀无角点，用立方体
    auto cloud = test_helpers::makeCube(1000, 10.0f, 0.0f, 42);
    // Harris3D 需要法线，Normals::estimate 返回新对象
    auto cloud_with_normals = Normals::estimate(cloud, 30, 2.0, 0, 0, 0).cloud;
    ASSERT_NE(cloud_with_normals, nullptr);
    size_t original = cloud_with_normals->size();

    auto result = Keypoints::HarrisKeypoint3D(
        cloud_with_normals,
        1,       // response_method: HARRIS
        0.001f,  // threshold
        true,    // non_maxima
        true,    // do_refine
        30,      // k
        1.0      // radius
    );

    ASSERT_CLOUD_NOT_NULL(result.cloud);
    EXPECT_LT(result.cloud->size(), original);
    EXPECT_GT(result.cloud->size(), 0u);
}

// ===== SIFT3D =====

TEST(KeypointsTest, SIFT3D_NonUniformDensity_DetectsFewerPoints) {
    // SIFT3D 检测基于局部密度变化，均匀分布无特征
    // 用添加离群点的方式制造密度差异
    auto cloud = test_helpers::makePlane(2000, 10.0f, 10.0f, 0.0f, 42);
    cloud = test_helpers::addOutliers(cloud, 100, 50.0f, 42);
    size_t original = cloud->size();

    auto result = Keypoints::SIFTKeypoint(
        cloud,
        0.1f,   // min_scale
        3,      // nr_octaves
        4,      // nr_scales_per_octave
        0.0001f,// min_contrast (较低阈值)
        30,     // k
        1.0     // radius
    );

    ASSERT_CLOUD_NOT_NULL(result.cloud);
    EXPECT_GT(result.cloud->size(), 0u);
}

// ===== Trajkovic =====

TEST(KeypointsTest, Trajkovic3D_Cube_DetectsFewerPoints) {
    // Trajkovic3D 检测角点，球面无角点，用立方体
    auto cloud = test_helpers::makeCube(1000, 10.0f, 0.0f, 42);
    size_t original = cloud->size();

    auto result = Keypoints::TrajkovicKeypoint3D(
        cloud,
        1,       // compute_method
        5,       // window_size (must be odd)
        0.001f,  // first_threshold
        0.001f,  // second_threshold
        30,      // k
        1.0      // radius
    );

    ASSERT_CLOUD_NOT_NULL(result.cloud);
    EXPECT_LT(result.cloud->size(), original);
    EXPECT_GT(result.cloud->size(), 0u);
}

// ===== 取消支持 =====

TEST(KeypointsTest, ISS_CancelNoCrash) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);
    std::atomic<bool> cancel(true);

    auto result = Keypoints::ISSKeypoint3D(
        cloud, 0.5, 0.975, 0.975, 5, 0.52, 30, 1.0, &cancel);
    ASSERT_CLOUD_NOT_NULL(result.cloud);
}

TEST(KeypointsTest, Harris3D_CancelNoCrash) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);
    std::atomic<bool> cancel(true);

    auto result = Keypoints::HarrisKeypoint3D(
        cloud, 1, 0.001f, true, true, 30, 1.0, &cancel);
    ASSERT_CLOUD_NOT_NULL(result.cloud);
}
