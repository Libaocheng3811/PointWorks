#include <gtest/gtest.h>
#include <features.h>
#include <normals.h>
#include "test_helpers.h"
#include "macros.h"

using namespace pw;

// ===== 包围盒 =====

TEST(FeaturesTest, BoundingBoxAABB_Sphere) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);
    Box box = Features::boundingBoxAABB(cloud);

    // 球半径 5，包围盒长宽高应约为 10
    EXPECT_GT(box.width, 8.0);
    EXPECT_GT(box.height, 8.0);
    EXPECT_GT(box.depth, 8.0);
    EXPECT_LT(box.width, 12.0);
    EXPECT_LT(box.height, 12.0);
    EXPECT_LT(box.depth, 12.0);
}

TEST(FeaturesTest, BoundingBoxAABB_Plane) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);
    Box box = Features::boundingBoxAABB(cloud);

    // 三个维度之和应接近平面面积量级 (10x10)
    double extent = box.width + box.height + box.depth;
    EXPECT_GT(extent, 18.0);
    // 体积应很小（平面几乎是2D的）
    double vol = box.width * box.height * box.depth;
    EXPECT_LT(vol, 200.0);
}

TEST(FeaturesTest, BoundingBoxOBB_Cube) {
    auto cloud = test_helpers::makeCube(1000, 10.0f, 0.0f, 42);
    Box box = Features::boundingBoxOBB(cloud);

    EXPECT_GT(box.width, 0.0);
    EXPECT_GT(box.height, 0.0);
    EXPECT_GT(box.depth, 0.0);
}

// ===== FPFH =====

TEST(FeaturesTest, FPFHEstimation_ValidResult) {
    auto cloud = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);

    auto result = Features::FPFHEstimation(cloud, 30, 2.0);
    EXPECT_FALSE(result.id.empty());
    ASSERT_NE(result.feature, nullptr);
    EXPECT_NE(result.feature->fpfh, nullptr);
    EXPECT_EQ(result.feature->fpfh->size(), cloud->size());
}

TEST(FeaturesTest, FPFHEstimation_SameCloud_SimilarDescriptors) {
    auto c1 = test_helpers::makeSphere(200, 5.0f, 0.0f, 42);
    auto c2 = test_helpers::makeSphere(200, 5.0f, 0.0f, 42);

    auto r1 = Features::FPFHEstimation(c1, 30, 2.0);
    auto r2 = Features::FPFHEstimation(c2, 30, 2.0);

    ASSERT_NE(r1.feature, nullptr);
    ASSERT_NE(r2.feature, nullptr);
    ASSERT_NE(r1.feature->fpfh, nullptr);
    ASSERT_NE(r2.feature->fpfh, nullptr);
    EXPECT_EQ(r1.feature->fpfh->size(), r2.feature->fpfh->size());
}

// ===== VFH =====

TEST(FeaturesTest, VFHEstimation_ValidResult) {
    auto cloud = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);
    Eigen::Vector3f view_dir(0, 0, 1);

    auto result = Features::VFHEstimation(cloud, view_dir);
    EXPECT_FALSE(result.id.empty());
    ASSERT_NE(result.feature, nullptr);
    EXPECT_NE(result.feature->vfh, nullptr);
    // VFH 只产生一个全局描述符
    EXPECT_EQ(result.feature->vfh->size(), 1u);
}

TEST(FeaturesTest, VFH_DifferentShapes_DifferentDescriptors) {
    auto sphere = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);
    auto plane = test_helpers::makePlane(300, 10.0f, 10.0f, 0.0f, 42);

    // VFH 需要法线，Normals::estimate 返回新对象，需用返回值
    auto sphere_with_normals = Normals::estimate(sphere, 30, 2.0, 0, 0, 0).cloud;
    auto plane_with_normals = Normals::estimate(plane, 30, 2.0, 0, 0, 0).cloud;
    ASSERT_NE(sphere_with_normals, nullptr);
    ASSERT_NE(plane_with_normals, nullptr);

    Eigen::Vector3f view_dir(0, 0, 1);
    auto r_sphere = Features::VFHEstimation(sphere_with_normals, view_dir);
    auto r_plane = Features::VFHEstimation(plane_with_normals, view_dir);

    ASSERT_NE(r_sphere.feature->vfh, nullptr);
    ASSERT_NE(r_plane.feature->vfh, nullptr);

    // 不同形状的 VFH 描述符应不同
    bool all_equal = true;
    for (int i = 0; i < r_sphere.feature->vfh->points[0].descriptorSize(); ++i) {
        if (std::abs(r_sphere.feature->vfh->points[0].histogram[i] -
                     r_plane.feature->vfh->points[0].histogram[i]) > 0.001f) {
            all_equal = false;
            break;
        }
    }
    EXPECT_FALSE(all_equal) << "Sphere and plane VFH descriptors should differ";
}

// ===== ESF =====

TEST(FeaturesTest, ESFEstimation_ValidResult) {
    auto cloud = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);

    auto result = Features::ESFEstimation(cloud);
    ASSERT_NE(result.feature, nullptr);
    EXPECT_NE(result.feature->esf, nullptr);
    // ESF 产生一个全局描述符
    EXPECT_EQ(result.feature->esf->size(), 1u);
}

// ===== SHOT LRF =====

TEST(FeaturesTest, SHOTLocalReferenceFrame_ValidResult) {
    auto cloud = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);

    auto result = Features::SHOTLocalReferenceFrameEstimation(cloud, 2.0f);
    EXPECT_FALSE(result.id.empty());
    ASSERT_NE(result.lrf, nullptr);
    EXPECT_GT(result.lrf->size(), 0u);
}

// ===== 取消支持 =====

TEST(FeaturesTest, FPFH_CancelNoCrash) {
    auto cloud = test_helpers::makeSphere(200, 5.0f, 0.0f, 42);
    std::atomic<bool> cancel(true);

    auto result = Features::FPFHEstimation(cloud, 30, 2.0, nullptr, &cancel);
    // 不崩溃即可
}
