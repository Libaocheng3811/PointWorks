#include <gtest/gtest.h>
#include <keypoints.h>
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

TEST(KeypointsTest, Harris3D_Sphere_DetectsFewerPoints) {
    auto cloud = test_helpers::makeSphere(1000, 5.0f, 0.0f, 42);
    size_t original = cloud->size();

    auto result = Keypoints::HarrisKeypoint3D(
        cloud,
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

TEST(KeypointsTest, SIFT3D_Sphere_DetectsFewerPoints) {
    auto cloud = test_helpers::makeSphere(1000, 5.0f, 0.0f, 42);
    size_t original = cloud->size();

    auto result = Keypoints::SIFTKeypoint(
        cloud,
        0.01f,  // min_scale
        3,      // nr_octaves
        4,      // nr_scales_per_octave
        0.001f, // min_contrast
        30,     // k
        1.0     // radius
    );

    ASSERT_CLOUD_NOT_NULL(result.cloud);
    EXPECT_LT(result.cloud->size(), original);
    EXPECT_GT(result.cloud->size(), 0u);
}

// ===== Trajkovic =====

TEST(KeypointsTest, Trajkovic3D_Sphere_DetectsFewerPoints) {
    auto cloud = test_helpers::makeSphere(1000, 5.0f, 0.0f, 42);
    size_t original = cloud->size();

    auto result = Keypoints::TrajkovicKeypoint3D(
        cloud,
        1,       // compute_method
        4,       // window_size
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
