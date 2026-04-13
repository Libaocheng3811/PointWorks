#include <gtest/gtest.h>
#include <filters.h>
#include "test_helpers.h"
#include "macros.h"

using namespace ct;

// ========== PassThrough ==========

class PassThroughTest : public ::testing::Test {
protected:
    void SetUp() override {
        cloud_ = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);
    }
    Cloud::Ptr cloud_;
};

TEST_F(PassThroughTest, XAxis_FilterRange) {
    auto result = Filters::PassThrough(cloud_, "x", -3.0f, 3.0f);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_GT(result.result_cloud->size(), 0u);
    EXPECT_LT(result.result_cloud->size(), cloud_->size());

    auto pcl_cloud = result.result_cloud->toPCL_XYZ();
    for (const auto& pt : pcl_cloud->points) {
        EXPECT_GE(pt.x, -3.0f);
        EXPECT_LE(pt.x, 3.0f);
    }
}

TEST_F(PassThroughTest, Negative_KeepsOutside) {
    auto result = Filters::PassThrough(cloud_, "x", -3.0f, 3.0f, true);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);

    auto pcl_cloud = result.result_cloud->toPCL_XYZ();
    for (const auto& pt : pcl_cloud->points) {
        EXPECT_TRUE(pt.x < -3.0f || pt.x > 3.0f);
    }
}

TEST_F(PassThroughTest, FullRange_NoFilter) {
    auto result = Filters::PassThrough(cloud_, "x", -100.0f, 100.0f);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_EQ(result.result_cloud->size(), cloud_->size());
}

// ========== VoxelGrid ==========

class VoxelGridTest : public ::testing::Test {
protected:
    void SetUp() override {
        cloud_ = test_helpers::makePlane(10000, 10.0f, 10.0f, 0.0f, 42);
    }
    Cloud::Ptr cloud_;
};

TEST_F(VoxelGridTest, ReducesPoints) {
    size_t original = cloud_->size();
    auto result = Filters::VoxelGrid(cloud_, 0.5f, 0.5f, 0.5f);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_LT(result.result_cloud->size(), original);
    EXPECT_GT(result.result_cloud->size(), 0u);
}

TEST_F(VoxelGridTest, PreservesPlanarity) {
    auto result = Filters::VoxelGrid(cloud_, 0.1f, 0.1f, 0.1f);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_TRUE(test_helpers::isApproximatelyPlanar(result.result_cloud, 0.2f));
}

TEST_F(VoxelGridTest, LargeVoxel_MinimalPoints) {
    auto result = Filters::VoxelGrid(cloud_, 10.0f, 10.0f, 10.0f);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    // 大体素应只保留很少的点
    EXPECT_LT(result.result_cloud->size(), 10u);
}

// ========== StatisticalOutlierRemoval ==========

TEST(SORTest, RemovesOutliers) {
    auto clean = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);
    auto noisy = test_helpers::addOutliers(clean, 100, 50.0f, 42);

    size_t before = noisy->size();
    auto result = Filters::StatisticalOutlierRemoval(noisy, 30, 2.0);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);

    // 应移除一些点（离群点）
    EXPECT_LT(result.result_cloud->size(), before);
    // 但应保留大部分正常点
    EXPECT_GT(result.result_cloud->size(), clean->size() * 0.7);
}

TEST(SORTest, HighStddevMul_NoRemoval) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);
    size_t original = cloud->size();

    // stddev_mult 很大时不应移除任何点
    auto result = Filters::StatisticalOutlierRemoval(cloud, 30, 1000.0);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_EQ(result.result_cloud->size(), original);
}

// ========== RadiusOutlierRemoval ==========

TEST(RORTest, RemovesIsolatedPoints) {
    auto clean = test_helpers::makePlane(500, 10.0f, 10.0f, 0.0f, 42);
    auto noisy = test_helpers::addOutliers(clean, 50, 100.0f, 42);

    auto result = Filters::RadiusOutlierRemoval(noisy, 1.0, 5);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_LT(result.result_cloud->size(), noisy->size());
}

TEST(RORTest, PreservesDenseRegion) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);
    size_t original = cloud->size();

    auto result = Filters::RadiusOutlierRemoval(cloud, 1.0, 3);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    // 球面点云密集，应保留绝大部分
    EXPECT_GT(result.result_cloud->size(), original * 0.9);
}

// ========== GridMinimum ==========

TEST(GridMinimumTest, ReducesPoints) {
    auto cloud = test_helpers::makePlane(5000, 10.0f, 10.0f, 0.0f, 42);
    size_t original = cloud->size();

    auto result = Filters::GridMinimun(cloud, 0.5f);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_LT(result.result_cloud->size(), original);
    EXPECT_GT(result.result_cloud->size(), 0u);
}

TEST(GridMinimumTest, PreservesPlanarity) {
    auto cloud = test_helpers::makePlane(5000, 10.0f, 10.0f, 0.0f, 42);
    auto result = Filters::GridMinimun(cloud, 0.5f);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_TRUE(test_helpers::isApproximatelyPlanar(result.result_cloud, 0.3f));
}

// ========== RandomSampling ==========

TEST(RandomSamplingTest, ExactCount) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);
    auto result = Filters::RandomSampling(cloud, 500, 42);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_EQ(result.result_cloud->size(), 500u);
}

TEST(RandomSamplingTest, Deterministic) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);
    auto r1 = Filters::RandomSampling(cloud, 100, 42);
    auto r2 = Filters::RandomSampling(cloud, 100, 42);

    ASSERT_CLOUD_NOT_NULL(r1.result_cloud);
    ASSERT_CLOUD_NOT_NULL(r2.result_cloud);
    EXPECT_EQ(r1.result_cloud->size(), r2.result_cloud->size());
}

// ========== ApproximateVoxelGrid ==========

TEST(ApproxVoxelGridTest, ReducesPoints) {
    auto cloud = test_helpers::makeSphere(2000, 5.0f, 0.0f, 42);
    size_t original = cloud->size();

    auto result = Filters::ApproximateVoxelGrid(cloud, 0.5f, 0.5f, 0.5f);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
    EXPECT_LT(result.result_cloud->size(), original);
    EXPECT_GT(result.result_cloud->size(), 0u);
}

// ========== 取消支持（不崩溃即可） ==========

TEST(CancelTest, VoxelGrid_CancelNoCrash) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);
    std::atomic<bool> cancel(true);  // 直接取消

    auto result = Filters::VoxelGrid(cloud, 0.1f, 0.1f, 0.1f, false, &cancel);
    // 不崩溃即可
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
}

TEST(CancelTest, SOR_CancelNoCrash) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);
    std::atomic<bool> cancel(true);

    auto result = Filters::StatisticalOutlierRemoval(cloud, 30, 2.0, false, &cancel);
    ASSERT_CLOUD_NOT_NULL(result.result_cloud);
}
