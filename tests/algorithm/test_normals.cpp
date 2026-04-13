#include <gtest/gtest.h>
#include <normals.h>
#include "test_helpers.h"
#include "macros.h"

using namespace ct;

// ===== 法线估计 =====

TEST(NormalsTest, Plane_NormalDirection) {
    // XY 平面，法线应指向 Z 方向
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);

    auto result = Normals::estimate(cloud, 30, 0.0, 0.0f, 0.0f, 0.0f);
    ASSERT_CLOUD_NOT_NULL(result.cloud);
    EXPECT_TRUE(result.error_msg.empty()) << "Error: " << result.error_msg;
    EXPECT_TRUE(result.cloud->hasNormals());

    // 转为 PCL 检查法线方向
    auto pcl_cloud = result.cloud->toPCL_XYZRGBN();
    ASSERT_FALSE(pcl_cloud->empty());

    // 计算平均法线方向
    float avg_nx = 0, avg_ny = 0, avg_nz = 0;
    for (const auto& pt : pcl_cloud->points) {
        avg_nx += pt.normal_x;
        avg_ny += pt.normal_y;
        avg_nz += pt.normal_z;
    }
    avg_nx /= pcl_cloud->size();
    avg_ny /= pcl_cloud->size();
    avg_nz /= pcl_cloud->size();

    // 平均法线应接近 Z 轴（可能正或负）
    float horizontal_component = std::abs(avg_nx) + std::abs(avg_ny);
    float vertical_component = std::abs(avg_nz);
    EXPECT_GT(vertical_component, horizontal_component * 5.0f)
        << "Expected normal near Z-axis, got (" << avg_nx << ", " << avg_ny << ", " << avg_nz << ")";
}

TEST(NormalsTest, Plane_WithViewPoint) {
    // 设置视点在 Z 正上方，法线应朝向视点
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);

    auto result = Normals::estimate(cloud, 30, 0.0, 0.0f, 0.0f, 100.0f);
    ASSERT_CLOUD_NOT_NULL(result.cloud);
    EXPECT_TRUE(result.error_msg.empty());

    auto pcl_cloud = result.cloud->toPCL_XYZRGBN();
    float avg_nz = 0;
    for (const auto& pt : pcl_cloud->points) {
        avg_nz += pt.normal_z;
    }
    avg_nz /= pcl_cloud->size();

    // 视点在 Z=100，法线应指向正 Z
    EXPECT_GT(avg_nz, 0.5f) << "Expected normals toward viewpoint, avg_nz=" << avg_nz;
}

TEST(NormalsTest, RadiusSearch) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f, 42);

    // 使用半径搜索
    auto result = Normals::estimate(cloud, 0, 2.0, 0.0f, 0.0f, 0.0f);
    ASSERT_CLOUD_NOT_NULL(result.cloud);
    EXPECT_TRUE(result.error_msg.empty());
    EXPECT_TRUE(result.cloud->hasNormals());
}

TEST(NormalsTest, EmptyCloud_ReturnsError) {
    auto cloud = std::make_shared<Cloud>();

    auto result = Normals::estimate(cloud, 30, 0.0, 0.0f, 0.0f, 0.0f);
    // 空点云应返回错误或空结果
    EXPECT_FALSE(result.error_msg.empty());
}

TEST(NormalsTest, Cancel_NoCrash) {
    auto cloud = test_helpers::makePlane(500, 10.0f, 10.0f, 0.0f, 42);
    std::atomic<bool> cancel(true);

    auto result = Normals::estimate(cloud, 30, 0.0, 0.0f, 0.0f, 0.0f, false, &cancel);
    // 不崩溃即可
    ASSERT_CLOUD_NOT_NULL(result.cloud);
}
