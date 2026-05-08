#include <gtest/gtest.h>
#include <cloudtype.h>
#include <cmath>

using namespace pw;

// ===== CompressedNormal =====

TEST(CompressedNormalTest, EncodeDecode_Identity) {
    // Z 轴方向 (0,0,1)
    CompressedNormal cn;
    cn.set(Eigen::Vector3f(0, 0, 1));
    auto decoded = cn.get();

    EXPECT_NEAR(decoded.x(), 0.0f, 0.02f);
    EXPECT_NEAR(decoded.y(), 0.0f, 0.02f);
    EXPECT_NEAR(decoded.z(), 1.0f, 0.02f);
}

TEST(CompressedNormalTest, EncodeDecode_NegativeZ) {
    // -Z 方向 (0,0,-1)
    CompressedNormal cn;
    cn.set(Eigen::Vector3f(0, 0, -1));
    auto decoded = cn.get();

    EXPECT_NEAR(decoded.x(), 0.0f, 0.02f);
    EXPECT_NEAR(decoded.y(), 0.0f, 0.02f);
    EXPECT_NEAR(decoded.z(), -1.0f, 0.02f);
}

TEST(CompressedNormalTest, EncodeDecode_XAxis) {
    CompressedNormal cn;
    cn.set(Eigen::Vector3f(1, 0, 0));
    auto decoded = cn.get();

    EXPECT_NEAR(decoded.x(), 1.0f, 0.02f);
    EXPECT_NEAR(decoded.y(), 0.0f, 0.02f);
    EXPECT_NEAR(std::abs(decoded.z()), 0.0f, 0.02f);
}

TEST(CompressedNormalTest, EncodeDecode_Diagonal) {
    Eigen::Vector3f dir(1, 1, 1);
    dir.normalize();

    CompressedNormal cn;
    cn.set(dir);
    auto decoded = cn.get();
    decoded.normalize();  // 解码后重新归一化

    EXPECT_NEAR(decoded.x(), dir.x(), 0.05f);
    EXPECT_NEAR(decoded.y(), dir.y(), 0.05f);
    EXPECT_NEAR(decoded.z(), dir.z(), 0.05f);
}

TEST(CompressedNormalTest, ZeroVector) {
    CompressedNormal cn;
    cn.set(Eigen::Vector3f(0, 0, 0));
    EXPECT_TRUE(cn.isZero());

    auto decoded = cn.get();
    EXPECT_EQ(decoded.x(), 0.0f);
    EXPECT_EQ(decoded.y(), 0.0f);
    EXPECT_EQ(decoded.z(), 1.0f);  // 默认方向
}

TEST(CompressedNormalTest, NonZeroAfterSet) {
    CompressedNormal cn;
    EXPECT_TRUE(cn.isZero());

    cn.set(Eigen::Vector3f(0, 1, 0));
    EXPECT_FALSE(cn.isZero());
}

// ===== ColorRGB =====

TEST(ColorRGBTest, DefaultWhite) {
    ColorRGB c;
    EXPECT_EQ(c.r, 255);
    EXPECT_EQ(c.g, 255);
    EXPECT_EQ(c.b, 255);
}

TEST(ColorRGBTest, ConstructFromValues) {
    ColorRGB c(128, 64, 32);
    EXPECT_EQ(c.r, 128);
    EXPECT_EQ(c.g, 64);
    EXPECT_EQ(c.b, 32);
}

TEST(ColorRGBTest, FloatConversion) {
    ColorRGB c(255, 128, 0);
    EXPECT_DOUBLE_EQ(c.rf(), 1.0);
    EXPECT_NEAR(c.gf(), 128.0 / 255.0, 1e-6);
    EXPECT_DOUBLE_EQ(c.bf(), 0.0);
}

// ===== Box =====

TEST(BoxTest, DefaultValues) {
    Box box;
    EXPECT_DOUBLE_EQ(box.width, 0.0);
    EXPECT_DOUBLE_EQ(box.height, 0.0);
    EXPECT_DOUBLE_EQ(box.depth, 0.0);
}

TEST(BoxTest, IdentityPose) {
    Box box;
    EXPECT_TRUE(box.pose.matrix().isApprox(Eigen::Affine3f::Identity().matrix()));
    EXPECT_TRUE(box.translation.isApprox(Eigen::Vector3f::Zero()));
}

// ===== Coord =====

TEST(CoordTest, DefaultValues) {
    Coord coord;
    EXPECT_TRUE(coord.id.empty());
    EXPECT_DOUBLE_EQ(coord.scale, 1.0);
}

TEST(CoordTest, ConstructFromParams) {
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.translation() = Eigen::Vector3f(1, 2, 3);
    Coord coord("test", 100.0, pose);

    EXPECT_EQ(coord.id, "test");
    EXPECT_DOUBLE_EQ(coord.scale, 100.0);
    EXPECT_NEAR(coord.pose.translation().x(), 1.0f, 1e-6f);
}

// ===== CloudConfig =====

TEST(CloudConfigTest, Defaults) {
    CloudConfig config;
    EXPECT_TRUE(config.enableOctree);
    EXPECT_GT(config.maxPointsPerBlock, 0u);
    EXPECT_GT(config.maxLODPoints, 0u);
    EXPECT_GT(config.pointBudget, 0u);
    EXPECT_GT(config.maxDepth, 0);
}

TEST(CloudConfigTest, DISABLED_NonDefaults) {
    // 预留测试：自定义配置
}

// ===== AutoOctreeConfig =====

TEST(AutoOctreeConfigTest, Constants) {
    EXPECT_GT(AutoOctreeConfig::MIN_POINTS_FOR_OCTREE, 0u);
    EXPECT_GT(AutoOctreeConfig::DEFAULT_BLOCK_SIZE, 0u);
    EXPECT_GT(AutoOctreeConfig::TARGET_BLOCK_COUNT, 0u);
    EXPECT_GT(AutoOctreeConfig::MIN_BLOCK_SIZE, 0u);
    EXPECT_GT(AutoOctreeConfig::MAX_BLOCK_SIZE, AutoOctreeConfig::MIN_BLOCK_SIZE);
    EXPECT_GT(AutoOctreeConfig::LOD_POINT_RATIO, 0.0f);
    EXPECT_GT(AutoOctreeConfig::MIN_LOD_SIZE, 0u);
    EXPECT_GT(AutoOctreeConfig::MAX_LOD_SIZE, AutoOctreeConfig::MIN_LOD_SIZE);
}

// ===== Color 常量 =====

TEST(ColorTest, PrimaryColors) {
    EXPECT_EQ(Color::Red.r, 255);
    EXPECT_EQ(Color::Red.g, 0);
    EXPECT_EQ(Color::Red.b, 0);

    EXPECT_EQ(Color::Green.r, 0);
    EXPECT_EQ(Color::Green.g, 255);
    EXPECT_EQ(Color::Green.b, 0);

    EXPECT_EQ(Color::Blue.r, 0);
    EXPECT_EQ(Color::Blue.g, 0);
    EXPECT_EQ(Color::Blue.b, 255);
}

TEST(ColorTest, BlackAndWhite) {
    EXPECT_EQ(Color::White.r, 255);
    EXPECT_EQ(Color::Black.r, 0);
}
