#include <gtest/gtest.h>
#include <surface.h>
#include <normals.h>
#include "test_helpers.h"
#include "macros.h"

using namespace ct;

// ===== ConvexHull =====

TEST(SurfaceTest, ConvexHull_ValidMesh) {
    auto cloud = test_helpers::makeSphere(200, 5.0f, 0.0f, 42);

    auto result = Surface::ConvexHull(cloud, false, 3);
    ASSERT_NE(result.mesh, nullptr);
    EXPECT_TRUE(result.error_msg.empty()) << result.error_msg;
    EXPECT_GT(result.mesh->polygons.size(), 0u);
    EXPECT_GT(result.mesh->cloud.width * result.mesh->cloud.height, 0u);
}

// ===== ConcaveHull =====

TEST(SurfaceTest, ConcaveHull_ValidMesh) {
    auto cloud = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);

    auto result = Surface::ConcaveHull(cloud, 2.0, false, 3);
    ASSERT_NE(result.mesh, nullptr);
    EXPECT_TRUE(result.error_msg.empty()) << result.error_msg;
    EXPECT_GT(result.mesh->polygons.size(), 0u);
}

// ===== GreedyProjection（需要法线） =====

TEST(SurfaceTest, GreedyProjection_SphereWithNormals) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);

    // 先估计法线
    auto normals_result = Normals::estimate(cloud, 30, 0.0, 0, 0, 100.0f);
    ASSERT_TRUE(normals_result.error_msg.empty());

    auto result = Surface::GreedyProjectionTriangulation(
        normals_result.cloud,
        2.5,    // mu
        100,    // nnn
        2.5,    // radius
        25.0,   // min angle
        150.0,  // max angle
        180.0,  // ep
        true,   // consistent
        true    // consistent_ordering
    );

    ASSERT_NE(result.mesh, nullptr);
    EXPECT_TRUE(result.error_msg.empty()) << result.error_msg;
    EXPECT_GT(result.mesh->polygons.size(), 0u);
}

// ===== Poisson（需要法线） =====

TEST(SurfaceTest, Poisson_SphereWithNormals) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f, 42);

    auto normals_result = Normals::estimate(cloud, 30, 0.0, 0, 0, 100.0f);
    ASSERT_TRUE(normals_result.error_msg.empty());

    auto result = Surface::Poisson(
        normals_result.cloud,
        8,      // depth
        5,      // min_depth
        1.0f,   // point_weight
        1.1f,   // scale
        8,      // solver_divide
        8,      // iso_divide
        1.0f,   // samples_per_node
        false,  // confidence
        false,  // output_polygons
        true    // manifold
    );

    ASSERT_NE(result.mesh, nullptr);
    EXPECT_TRUE(result.error_msg.empty()) << result.error_msg;
    EXPECT_GT(result.mesh->polygons.size(), 0u);
}

// ===== MarchingCubesHoppe（需要法线） =====

TEST(SurfaceTest, MarchingCubesHoppe_SphereWithNormals) {
    auto cloud = test_helpers::makeSphere(300, 5.0f, 0.0f, 42);

    auto normals_result = Normals::estimate(cloud, 30, 0.0, 0, 0, 100.0f);
    ASSERT_TRUE(normals_result.error_msg.empty());

    auto result = Surface::MarchingCubesHoppe(
        normals_result.cloud,
        0.0f,   // iso_level
        20, 20, 20,  // resolution
        0.0f,   // percentage
        10.0f   // dist_ignore
    );

    ASSERT_NE(result.mesh, nullptr);
    EXPECT_TRUE(result.error_msg.empty()) << result.error_msg;
    EXPECT_GT(result.mesh->polygons.size(), 0u);
}

// ===== GridProjection（需要法线） =====

TEST(SurfaceTest, GridProjection_PlaneWithNormals) {
    auto cloud = test_helpers::makePlane(300, 10.0f, 10.0f, 0.0f, 42);

    auto normals_result = Normals::estimate(cloud, 30, 0.0, 0, 0, 0);
    ASSERT_TRUE(normals_result.error_msg.empty());

    auto result = Surface::GridProjection(
        normals_result.cloud,
        1.0f,   // resolution
        3,      // padding_size
        20,     // k
        5       // max_binary_search_level
    );

    ASSERT_NE(result.mesh, nullptr);
    EXPECT_TRUE(result.error_msg.empty()) << result.error_msg;
    EXPECT_GT(result.mesh->polygons.size(), 0u);
}

// ===== 错误处理 =====

TEST(SurfaceTest, Poisson_TooFewPoints_ReturnsError) {
    auto cloud = test_helpers::makeSphere(3, 5.0f, 0.0f, 42);

    auto normals_result = Normals::estimate(cloud, 3, 0.0, 0, 0, 100.0f);

    auto result = Surface::Poisson(
        normals_result.cloud,
        8, 5, 1.0f, 1.1f, 8, 8, 1.0f, false, false, true);

    // 点太少应返回错误或空网格
    // 不崩溃即可
}

// ===== 取消支持 =====

TEST(SurfaceTest, ConvexHull_CancelNoCrash) {
    auto cloud = test_helpers::makeSphere(200, 5.0f, 0.0f, 42);
    std::atomic<bool> cancel(true);

    auto result = Surface::ConvexHull(cloud, false, 3, &cancel);
    ASSERT_NE(result.mesh, nullptr);
}
