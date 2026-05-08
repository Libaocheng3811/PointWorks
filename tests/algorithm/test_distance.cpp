#include <gtest/gtest.h>
#include <distancecalculator.h>
#include "test_helpers.h"
#include "macros.h"

#include <cmath>

using namespace pw;

// ===== C2C 距离计算 =====

TEST(DistanceC2CTest, IdenticalClouds_ZeroDistance) {
    // 相同点云对，距离应为 0
    auto cloud = test_helpers::makePlane(500, 10.0f, 10.0f, 0.0f, 42);

    C2CParams params;
    params.method = C2CParams::C2C_NEAREST;

    auto result = DistanceCalculator::calculateC2C(cloud, cloud, params);
    EXPECT_TRUE(result.success) << result.error_msg;

    // 所有距离应接近 0
    for (float d : result.distances) {
        EXPECT_NEAR(d, 0.0f, 0.01f);
    }
}

TEST(DistanceC2CTest, KnownOffset_Translation) {
    // 将平面沿 Z 平移 2.0，距离应约为 2.0
    auto source = test_helpers::makePlane(500, 10.0f, 10.0f, 0.0f, 42);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(2, 3) = 2.0f;
    auto target = test_helpers::applyTransform(source, transform);

    C2CParams params;
    params.method = C2CParams::C2C_NEAREST;

    auto result = DistanceCalculator::calculateC2C(source, target, params);
    EXPECT_TRUE(result.success) << result.error_msg;
    ASSERT_FALSE(result.distances.empty());

    // 计算平均距离
    float sum = 0;
    for (float d : result.distances) sum += d;
    float mean = sum / result.distances.size();

    EXPECT_NEAR(mean, 2.0f, 0.5f)
        << "Expected mean distance ~2.0, got " << mean;
}

TEST(DistanceC2CTest, DistanceCountMatchesSource) {
    auto source = test_helpers::makePlane(100, 10.0f, 10.0f, 0.0f, 42);
    auto target = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 99);

    C2CParams params;
    params.method = C2CParams::C2C_NEAREST;

    // calculateC2C(ref, comp): 为 comp 的每个点计算到 ref 的最近距离
    auto result = DistanceCalculator::calculateC2C(target, source, params);
    EXPECT_TRUE(result.success) << result.error_msg;
    EXPECT_EQ(result.distances.size(), source->size());
}

// ===== C2C 方法变体 =====

TEST(DistanceC2CTest, KNN_Method) {
    auto source = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);
    auto target = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);

    C2CParams params;
    params.method = C2CParams::C2C_KNN_MEAN;
    params.k_knn = 5;

    auto result = DistanceCalculator::calculateC2C(source, target, params);
    EXPECT_TRUE(result.success) << result.error_msg;
    EXPECT_EQ(result.distances.size(), source->size());
}

TEST(DistanceC2CTest, Radius_Method) {
    auto source = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);
    auto target = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);

    C2CParams params;
    params.method = C2CParams::C2C_RADIUS_MEAN;
    params.radius = 2.0;

    auto result = DistanceCalculator::calculateC2C(source, target, params);
    EXPECT_TRUE(result.success) << result.error_msg;
    EXPECT_EQ(result.distances.size(), source->size());
}

// ===== CPS（最近点集）=====

TEST(CPSTest, IdenticalClouds) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);

    CPSParams params;

    auto result = DistanceCalculator::extractClosestPoints(cloud, cloud, params);
    EXPECT_TRUE(result.success) << result.error_msg;
    ASSERT_CLOUD_NOT_NULL(result.projected_cloud);
    EXPECT_EQ(result.projected_cloud->size(), cloud->size());
}

// ===== 取消支持 =====

TEST(DistanceCancelTest, C2C_CancelNoCrash) {
    auto source = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);
    auto target = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);

    std::atomic<bool> cancel(true);
    C2CParams params;
    params.method = C2CParams::C2C_NEAREST;

    auto result = DistanceCalculator::calculateC2C(source, target, params, &cancel);
    // 不崩溃即可
    EXPECT_TRUE(result.success);
}
