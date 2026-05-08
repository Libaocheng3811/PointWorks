#include <gtest/gtest.h>
#include <registration.h>
#include "test_helpers.h"
#include "macros.h"

using namespace pw;

// ===== ICP 配准 =====

TEST(RegistrationTest, ICP_PerfectAlignment) {
    // 相同点云配准 → 变换应为单位矩阵
    auto cloud = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);

    RegistrationContext ctx;
    ctx.source_cloud = cloud;
    ctx.target_cloud = cloud;
    ctx.params.max_iterations = 50;
    ctx.params.transformation_epsilon = 1e-6;
    ctx.params.euclidean_fitness_epsilon = 1e-6;

    auto result = Registration::IterativeClosestPoint(ctx, false);
    EXPECT_TRUE(result.success);

    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    EXPECT_TRUE(test_helpers::matrixApproxEqual(result.matrix, identity, 0.05f));
}

TEST(RegistrationTest, ICP_SmallTranslation) {
    // 小幅度平移，ICP 应能恢复
    auto cloud = test_helpers::makePlane(500, 10.0f, 10.0f, 0.0f, 42);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = 0.5f;
    transform(1, 3) = 0.3f;
    transform(2, 3) = 0.1f;

    auto target = test_helpers::applyTransform(cloud, transform);

    RegistrationContext ctx;
    ctx.source_cloud = cloud;
    ctx.target_cloud = target;
    ctx.params.max_iterations = 100;
    ctx.params.transformation_epsilon = 1e-6;
    ctx.params.euclidean_fitness_epsilon = 1e-6;
    ctx.params.distance_threshold = 3.0;

    auto result = Registration::IterativeClosestPoint(ctx, false);
    EXPECT_TRUE(result.success);

    // 估计变换应接近真实变换
    EXPECT_TRUE(test_helpers::matrixApproxEqual(result.matrix, transform, 0.1f));
}

TEST(RegistrationTest, ICP_CancelNoCrash) {
    auto cloud = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);

    RegistrationContext ctx;
    ctx.source_cloud = cloud;
    ctx.target_cloud = cloud;
    ctx.params.max_iterations = 1000;

    std::atomic<bool> cancel(true);
    auto result = Registration::IterativeClosestPoint(ctx, false, &cancel);
    // 取消后可能返回 nullptr，不崩溃即可
}

// ===== ICP NonLinear =====

TEST(RegistrationTest, ICPNonLinear_PerfectAlignment) {
    auto cloud = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);

    RegistrationContext ctx;
    ctx.source_cloud = cloud;
    ctx.target_cloud = cloud;
    ctx.params.max_iterations = 50;

    auto result = Registration::IterativeClosestPointNonLinear(ctx, false);
    EXPECT_TRUE(result.success);

    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    EXPECT_TRUE(test_helpers::matrixApproxEqual(result.matrix, identity, 0.05f));
}

// ===== NDT 配准 =====

TEST(RegistrationTest, NDT_PerfectAlignment) {
    auto cloud = test_helpers::makePlane(500, 10.0f, 10.0f, 0.0f, 42);

    RegistrationContext ctx;
    ctx.source_cloud = cloud;
    ctx.target_cloud = cloud;

    auto result = Registration::NormalDistributionsTransform(ctx, 1.0f, 0.1, 0.05);
    EXPECT_TRUE(result.success);

    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    EXPECT_TRUE(test_helpers::matrixApproxEqual(result.matrix, identity, 0.1f));
}

TEST(RegistrationTest, NDT_SmallTranslation) {
    auto cloud = test_helpers::makePlane(500, 10.0f, 10.0f, 0.0f, 42);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = 1.0f;
    transform(1, 3) = 0.5f;

    auto target = test_helpers::applyTransform(cloud, transform);

    RegistrationContext ctx;
    ctx.source_cloud = cloud;
    ctx.target_cloud = target;

    auto result = Registration::NormalDistributionsTransform(ctx, 1.0f, 0.1, 0.05);
    EXPECT_TRUE(result.success);
}

// ===== 约束变换配准 =====

TEST(RegistrationTest, ConstrainedTransform_Identity) {
    // 相同点对，约束变换应返回单位矩阵
    std::vector<Eigen::Vector3d> src_pts, tgt_pts;
    for (int i = 0; i < 10; ++i) {
        Eigen::Vector3d pt(i * 0.1, 0, 0);
        src_pts.push_back(pt);
        tgt_pts.push_back(pt);
    }

    ConstrainedTransformParams params;
    params.rotation = RotationConstraint::NONE;  // 纯平移

    auto result = Registration::ConstrainedPointPairsRegistration(src_pts, tgt_pts, params);
    EXPECT_NEAR(result.rms, 0.0, 1e-6);

    // 平移分量应接近零
    EXPECT_NEAR(result.matrix(0, 3), 0.0, 1e-4);
    EXPECT_NEAR(result.matrix(1, 3), 0.0, 1e-4);
    EXPECT_NEAR(result.matrix(2, 3), 0.0, 1e-4);
}

TEST(RegistrationTest, ConstrainedTransform_KnownTranslation) {
    std::vector<Eigen::Vector3d> src_pts, tgt_pts;
    for (int i = 0; i < 10; ++i) {
        src_pts.push_back({i * 0.1, 0, 0});
        tgt_pts.push_back({i * 0.1 + 2.0, 1.0, 0.5});
    }

    ConstrainedTransformParams params;
    params.rotation = RotationConstraint::NONE;
    params.tx_enabled = true;
    params.ty_enabled = true;
    params.tz_enabled = true;

    auto result = Registration::ConstrainedPointPairsRegistration(src_pts, tgt_pts, params);
    EXPECT_NEAR(result.rms, 0.0, 1e-6);
    EXPECT_NEAR(result.matrix(0, 3), 2.0, 1e-4);
    EXPECT_NEAR(result.matrix(1, 3), 1.0, 1e-4);
    EXPECT_NEAR(result.matrix(2, 3), 0.5, 1e-4);
}

TEST(RegistrationTest, ConstrainedTransform_XOnlyRotation) {
    // 沿 Y 平移 + 绕 X 旋转
    std::vector<Eigen::Vector3d> src_pts, tgt_pts;
    src_pts.push_back({0, 0, 0});
    src_pts.push_back({0, 1, 0});
    src_pts.push_back({0, 2, 0});
    // 绕 X 轴旋转 90 度: Y->Z, Z->-Y
    tgt_pts.push_back({0, 0, 0});
    tgt_pts.push_back({0, 0, 1});
    tgt_pts.push_back({0, 0, 2});

    ConstrainedTransformParams params;
    params.rotation = RotationConstraint::X_ONLY;

    auto result = Registration::ConstrainedPointPairsRegistration(src_pts, tgt_pts, params);
    EXPECT_NEAR(result.rms, 0.0, 1e-3);
}
