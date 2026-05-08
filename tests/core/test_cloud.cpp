#include <gtest/gtest.h>
#include <cloud.h>
#include "test_helpers.h"
#include "macros.h"

using namespace pw;

// ===== 基础构造与容量 =====

TEST(CloudTest, EmptyCloud) {
    auto cloud = std::make_shared<Cloud>();
    ASSERT_NE(nullptr, cloud);
    EXPECT_EQ(cloud->size(), 0u);
    EXPECT_TRUE(cloud->empty());
}

TEST(CloudTest, AddPoints_Basic) {
    auto cloud = test_helpers::makePlane(1000);
    ASSERT_CLOUD_SIZE_AT_LEAST(cloud, 1000);
}

TEST(CloudTest, AddPoints_WithColor) {
    auto cloud = std::make_shared<Cloud>();
    cloud->enableColors();

    std::vector<PointXYZ> pts(100);
    std::vector<ColorRGB> colors(100, {255, 0, 0});
    cloud->addPoints(pts, &colors);
    cloud->update();

    EXPECT_EQ(cloud->size(), 100u);
    EXPECT_TRUE(cloud->hasColors());
}

TEST(CloudTest, AddPoints_WithNormals) {
    auto cloud = std::make_shared<Cloud>();
    cloud->enableNormals();

    std::vector<PointXYZ> pts(100);
    Eigen::Vector3f normal_dir(0, 0, 1);
    std::vector<CompressedNormal> normals(100);
    for (auto& n : normals) n.set(normal_dir);

    cloud->addPoints(pts, nullptr, &normals);
    cloud->update();

    EXPECT_EQ(cloud->size(), 100u);
    EXPECT_TRUE(cloud->hasNormals());
}

TEST(CloudTest, AddPoints_WithColorAndNormals) {
    auto cloud = std::make_shared<Cloud>();
    cloud->enableColors();
    cloud->enableNormals();

    std::vector<PointXYZ> pts(50);
    std::vector<ColorRGB> colors(50, {128, 64, 32});
    std::vector<CompressedNormal> normals(50);
    Eigen::Vector3f n(0, 1, 0);
    for (auto& cn : normals) cn.set(n);

    cloud->addPoints(pts, &colors, &normals);
    cloud->update();

    EXPECT_EQ(cloud->size(), 50u);
    EXPECT_TRUE(cloud->hasColors());
    EXPECT_TRUE(cloud->hasNormals());
}

TEST(CloudTest, Clear) {
    auto cloud = test_helpers::makePlane(500);
    ASSERT_FALSE(cloud->empty());

    cloud->clear();
    EXPECT_EQ(cloud->size(), 0u);
    EXPECT_TRUE(cloud->empty());
}

// ===== 属性管理 =====

TEST(CloudTest, ColorEnableDisable) {
    auto cloud = std::make_shared<Cloud>();
    EXPECT_FALSE(cloud->hasColors());

    cloud->enableColors();
    EXPECT_TRUE(cloud->hasColors());

    cloud->disableColors();
    EXPECT_FALSE(cloud->hasColors());
}

TEST(CloudTest, NormalEnableDisable) {
    auto cloud = std::make_shared<Cloud>();
    EXPECT_FALSE(cloud->hasNormals());

    cloud->enableNormals();
    EXPECT_TRUE(cloud->hasNormals());

    cloud->disableNormals();
    EXPECT_FALSE(cloud->hasNormals());
}

// ===== 几何计算 =====

TEST(CloudTest, Box_Calculation) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f);
    Box box = cloud->box();

    // XY 平面：width 和 height 应较大，depth 接近 0
    EXPECT_GT(box.width, 8.0);
    EXPECT_GT(box.height, 8.0);
    EXPECT_LT(box.depth, 1.0);
}

TEST(CloudTest, Center_Calculation) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f);
    Eigen::Vector3f center = cloud->center();

    // center() 返回八叉树包围盒的偏移点，在点的范围内
    EXPECT_GT(center.x(), -6.0f);
    EXPECT_LT(center.x(), 6.0f);
    EXPECT_GT(center.y(), -6.0f);
    EXPECT_LT(center.y(), 6.0f);
    EXPECT_NEAR(center.z(), 0.0f, 1.0f);
}

TEST(CloudTest, MinMax) {
    auto cloud = test_helpers::makeSphere(500, 5.0f, 0.0f);
    auto mn = cloud->min();
    auto mx = cloud->max();

    // 球半径 5，min/max 应在 [-5, 5] 附近
    EXPECT_LT(mn.x, -4.0f);
    EXPECT_LT(mn.y, -4.0f);
    EXPECT_LT(mn.z, -4.0f);
    EXPECT_GT(mx.x, 4.0f);
    EXPECT_GT(mx.y, 4.0f);
    EXPECT_GT(mx.z, 4.0f);
}

TEST(CloudTest, Volume) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f);
    double vol = cloud->volume();
    EXPECT_GT(vol, 0.0);
}

// ===== PCL 互操作 =====

TEST(CloudTest, PCLInterop_XYZ) {
    auto cloud = test_helpers::makeSphere(500, 5.0f);
    auto pcl_cloud = cloud->toPCL_XYZ();

    ASSERT_NE(pcl_cloud, nullptr);
    EXPECT_EQ(pcl_cloud->size(), cloud->size());
}

TEST(CloudTest, PCLInterop_XYZRGB) {
    auto cloud = std::make_shared<Cloud>();
    cloud->enableColors();
    std::vector<PointXYZ> pts(100);
    std::vector<ColorRGB> colors(100, {255, 128, 0});
    cloud->addPoints(pts, &colors);
    cloud->update();

    auto pcl_cloud = cloud->toPCL_XYZRGB();
    ASSERT_NE(pcl_cloud, nullptr);
    EXPECT_EQ(pcl_cloud->size(), 100u);
}

// ===== 标量场 =====

TEST(CloudTest, ScalarFields_AddRemove) {
    auto cloud = test_helpers::makePlane(100);

    std::vector<float> field(100, 1.0f);
    cloud->addScalarField("height", field);

    EXPECT_TRUE(cloud->hasScalarField("height"));
    auto* retrieved = cloud->getScalarField("height");
    ASSERT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->size(), 100u);
    EXPECT_FLOAT_EQ((*retrieved)[0], 1.0f);

    cloud->removeScalarField("height");
    EXPECT_FALSE(cloud->hasScalarField("height"));
}

TEST(CloudTest, ScalarFields_MultipleFields) {
    auto cloud = test_helpers::makePlane(50);
    std::vector<float> f1(50, 1.0f);
    std::vector<float> f2(50, 2.0f);
    cloud->addScalarField("a", f1);
    cloud->addScalarField("b", f2);

    EXPECT_TRUE(cloud->hasScalarField("a"));
    EXPECT_TRUE(cloud->hasScalarField("b"));

    auto names = cloud->getScalarFieldNames();
    EXPECT_EQ(names.size(), 2u);

    cloud->clearScalarFields();
    EXPECT_FALSE(cloud->hasScalarField("a"));
    EXPECT_FALSE(cloud->hasScalarField("b"));
}

TEST(CloudTest, ScalarFields_Nonexistent) {
    auto cloud = test_helpers::makePlane(10);
    EXPECT_FALSE(cloud->hasScalarField("nonexistent"));
    EXPECT_EQ(cloud->getScalarField("nonexistent"), nullptr);
    EXPECT_FALSE(cloud->removeScalarField("nonexistent"));
}

// ===== 克隆 =====

TEST(CloudTest, Clone_Independence) {
    auto original = test_helpers::makePlane(100, 10.0f, 10.0f);
    auto cloned = original->clone();

    ASSERT_CLOUD_NOT_NULL(cloned);
    EXPECT_EQ(cloned->size(), original->size());

    // 修改克隆对象不应影响原始
    cloned->clear();
    EXPECT_EQ(cloned->size(), 0u);
    EXPECT_GT(original->size(), 0u);
}

// ===== 追加 =====

TEST(CloudTest, Append) {
    auto c1 = test_helpers::makePlane(100, 10.0f, 10.0f, 0.0f, 42);
    auto c2 = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 99);
    size_t s1 = c1->size();

    c1->append(*c2);
    EXPECT_EQ(c1->size(), s1 + c2->size());
}

// ===== 元数据 =====

TEST(CloudTest, Id) {
    auto cloud = std::make_shared<Cloud>();
    EXPECT_EQ(cloud->id(), "cloud");
    cloud->setId("test_cloud");
    EXPECT_EQ(cloud->id(), "test_cloud");
}

TEST(CloudTest, Filepath) {
    auto cloud = std::make_shared<Cloud>();
    cloud->setFilepath("/path/to/file.pcd");
    EXPECT_EQ(cloud->filepath(), "/path/to/file.pcd");
}

TEST(CloudTest, PointSize) {
    auto cloud = std::make_shared<Cloud>();
    EXPECT_EQ(cloud->pointSize(), 1);
    cloud->setPointSize(3);
    EXPECT_EQ(cloud->pointSize(), 3);
}

TEST(CloudTest, Opacity) {
    auto cloud = std::make_shared<Cloud>();
    EXPECT_FLOAT_EQ(cloud->opacity(), 1.0f);
    cloud->setOpacity(0.5f);
    EXPECT_FLOAT_EQ(cloud->opacity(), 0.5f);
}

TEST(CloudTest, GlobalShift) {
    auto cloud = test_helpers::makePlane(100);
    Eigen::Vector3d shift(500000.0, 600000.0, 0.0);
    cloud->setGlobalShift(shift);

    auto retrieved = cloud->getGlobalShift();
    EXPECT_DOUBLE_EQ(retrieved.x(), 500000.0);
    EXPECT_DOUBLE_EQ(retrieved.y(), 600000.0);
    EXPECT_DOUBLE_EQ(retrieved.z(), 0.0);
}

// ===== 自适应配置 =====

TEST(CloudTest, AdaptiveConfig_SmallCloud) {
    auto config = Cloud::calculateAdaptiveConfig(1000);
    EXPECT_FALSE(config.enableOctree);
}

TEST(CloudTest, AdaptiveConfig_LargeCloud) {
    auto config = Cloud::calculateAdaptiveConfig(20000000);
    EXPECT_TRUE(config.enableOctree);
    EXPECT_GT(config.maxPointsPerBlock, 0u);
}

// ===== 颜色操作 =====

TEST(CloudTest, SetCloudColor_Solid) {
    auto cloud = test_helpers::makePlane(100);
    cloud->enableColors();

    cloud->setCloudColor(pw::ColorRGB{255, 0, 0});
    EXPECT_TRUE(cloud->hasColors());
}

TEST(CloudTest, ColorBackupRestore) {
    auto cloud = std::make_shared<Cloud>();
    cloud->enableColors();
    std::vector<PointXYZ> pts(10);
    std::vector<ColorRGB> colors(10, {255, 0, 0});
    cloud->addPoints(pts, &colors);
    cloud->update();

    cloud->backupColors();
    cloud->setCloudColor(pw::ColorRGB{0, 255, 0});
    EXPECT_TRUE(cloud->isColorModified());

    cloud->restoreColors();
    // restore 后颜色模式应恢复
    EXPECT_FALSE(cloud->isColorModified());
}

// ===== test_helpers 验证 =====

TEST(TestHelpers, MakePlane_IsPlanar) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f);
    EXPECT_TRUE(test_helpers::isApproximatelyPlanar(cloud, 0.1f));
}

TEST(TestHelpers, MakePlane_NoisyNotPerfectlyPlanar) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 2.0f);
    // 有噪声时应不满足严格平面条件
    EXPECT_FALSE(test_helpers::isApproximatelyPlanar(cloud, 0.1f));
}

TEST(TestHelpers, MakeSphere_IsSpherical) {
    auto cloud = test_helpers::makeSphere(1000, 5.0f);
    EXPECT_TRUE(test_helpers::isApproximatelySpherical(cloud, 5.0f, 0.2f));
}

TEST(TestHelpers, MatrixApproxEqual) {
    Eigen::Matrix4f a = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f b = Eigen::Matrix4f::Identity();

    EXPECT_TRUE(test_helpers::matrixApproxEqual(a, b, 1e-3f));

    b(0, 3) += 0.001f;
    EXPECT_TRUE(test_helpers::matrixApproxEqual(a, b, 0.01f));
    EXPECT_FALSE(test_helpers::matrixApproxEqual(a, b, 0.0001f));
}

TEST(TestHelpers, TwoClusters_Labels) {
    auto pair = test_helpers::makeTwoClusters(500, 500, 10.0f);
    EXPECT_EQ(pair.cloud->size(), 1000u);
    EXPECT_EQ(pair.labels.size(), 1000u);

    int count0 = 0, count1 = 0;
    for (int label : pair.labels) {
        if (label == 0) count0++;
        else if (label == 1) count1++;
    }
    EXPECT_EQ(count0, 500);
    EXPECT_EQ(count1, 500);
}

TEST(TestHelpers, AddOutliers) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f);
    auto noisy = test_helpers::addOutliers(cloud, 50, 100.0f);
    EXPECT_EQ(noisy->size(), 1050u);
}

TEST(TestHelpers, ComputeCentroid) {
    auto cloud = test_helpers::makePlane(1000, 10.0f, 10.0f, 0.0f);
    auto centroid = test_helpers::computeCentroid(cloud);
    EXPECT_NEAR(centroid[0], 0.0f, 1.0f);
    EXPECT_NEAR(centroid[1], 0.0f, 1.0f);
    EXPECT_NEAR(centroid[2], 0.0f, 0.1f);
}

TEST(TestHelpers, ApplyTransform) {
    auto cloud = test_helpers::makePlane(100, 10.0f, 10.0f, 0.0f);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = 10.0f;  // 平移 X +10

    auto shifted = test_helpers::applyTransform(cloud, transform);
    auto centroid = test_helpers::computeCentroid(shifted);
    EXPECT_NEAR(centroid[0], 10.0f, 1.0f);
}
