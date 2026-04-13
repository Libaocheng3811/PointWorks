#include <gtest/gtest.h>
#include <QDir>
#include <QTemporaryDir>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "test_helpers.h"
#include "macros.h"

using namespace ct;

// ===== PCD 格式兼容性 =====

TEST(PcdFormatTest, SaveAndLoad_ASCII) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/ascii.pcd";

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back({1.0f, 2.0f, 3.0f});
    cloud.push_back({4.0f, 5.0f, 6.0f});

    ASSERT_EQ(pcl::io::savePCDFile(path.toStdString(), cloud, false), 0);

    pcl::PointCloud<pcl::PointXYZ> loaded;
    ASSERT_EQ(pcl::io::loadPCDFile(path.toStdString(), loaded), 0);
    ASSERT_EQ(loaded.size(), 2u);
    EXPECT_FLOAT_EQ(loaded[0].x, 1.0f);
    EXPECT_FLOAT_EQ(loaded[0].y, 2.0f);
    EXPECT_FLOAT_EQ(loaded[0].z, 3.0f);
}

TEST(PcdFormatTest, SaveAndLoad_Binary) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/binary.pcd";

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back({-1.5f, 100.0f, -200.0f});
    cloud.push_back({0.001f, 0.002f, 0.003f});

    ASSERT_EQ(pcl::io::savePCDFile(path.toStdString(), cloud, true), 0);

    pcl::PointCloud<pcl::PointXYZ> loaded;
    ASSERT_EQ(pcl::io::loadPCDFile(path.toStdString(), loaded), 0);
    ASSERT_EQ(loaded.size(), 2u);

    EXPECT_NEAR(loaded[0].x, -1.5f, 1e-5f);
    EXPECT_NEAR(loaded[0].y, 100.0f, 1e-5f);
    EXPECT_NEAR(loaded[0].z, -200.0f, 1e-5f);
}

TEST(PcdFormatTest, SaveAndLoad_XYZRGB) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/xyzrgb.pcd";

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointXYZRGB pt;
    pt.x = 1; pt.y = 2; pt.z = 3;
    pt.r = 255; pt.g = 128; pt.b = 0;
    cloud.push_back(pt);

    ASSERT_EQ(pcl::io::savePCDFile(path.toStdString(), cloud, true), 0);

    pcl::PointCloud<pcl::PointXYZRGB> loaded;
    ASSERT_EQ(pcl::io::loadPCDFile(path.toStdString(), loaded), 0);
    ASSERT_EQ(loaded.size(), 1u);
    EXPECT_EQ(loaded[0].r, 255);
    EXPECT_EQ(loaded[0].g, 128);
    EXPECT_EQ(loaded[0].b, 0);
}

// ===== PLY 格式兼容性 =====

TEST(PlyFormatTest, SaveAndLoad_Binary) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/binary.ply";

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < 100; ++i) {
        cloud.push_back({float(i), float(i * 2), float(i * 3)});
    }

    ASSERT_EQ(pcl::io::savePLYFile(path.toStdString(), cloud, true), 0);

    pcl::PointCloud<pcl::PointXYZ> loaded;
    ASSERT_EQ(pcl::io::loadPLYFile(path.toStdString(), loaded), 0);
    ASSERT_EQ(loaded.size(), 100u);

    EXPECT_NEAR(loaded[50].x, 50.0f, 1e-5f);
    EXPECT_NEAR(loaded[50].y, 100.0f, 1e-5f);
    EXPECT_NEAR(loaded[50].z, 150.0f, 1e-5f);
}

TEST(PlyFormatTest, SaveAndLoad_ASCII) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/ascii.ply";

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back({1.0f, 2.0f, 3.0f});

    ASSERT_EQ(pcl::io::savePLYFile(path.toStdString(), cloud, false), 0);

    pcl::PointCloud<pcl::PointXYZ> loaded;
    ASSERT_EQ(pcl::io::loadPLYFile(path.toStdString(), loaded), 0);
    ASSERT_EQ(loaded.size(), 1u);
    EXPECT_FLOAT_EQ(loaded[0].x, 1.0f);
}

TEST(PlyFormatTest, SaveAndLoad_XYZRGB) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/xyzrgb.ply";

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointXYZRGB pt;
    pt.x = 10; pt.y = 20; pt.z = 30;
    pt.r = 0; pt.g = 255; pt.b = 128;
    cloud.push_back(pt);

    ASSERT_EQ(pcl::io::savePLYFile(path.toStdString(), cloud, true), 0);

    pcl::PointCloud<pcl::PointXYZRGB> loaded;
    ASSERT_EQ(pcl::io::loadPLYFile(path.toStdString(), loaded), 0);
    ASSERT_EQ(loaded.size(), 1u);
    EXPECT_EQ(loaded[0].r, 0);
    EXPECT_EQ(loaded[0].g, 255);
    EXPECT_EQ(loaded[0].b, 128);
}

// ===== PCL PCD 二进制格式变体 =====

TEST(PcdFormatTest, SaveAndLoad_BinaryCompressed) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/compressed.pcd";

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < 50; ++i) {
        cloud.push_back({float(i), float(i), float(i)});
    }

    // 保存为 binary_compressed
    ASSERT_EQ(pcl::io::savePCDFile(path.toStdString(), cloud, true), 0);

    pcl::PointCloud<pcl::PointXYZ> loaded;
    ASSERT_EQ(pcl::io::loadPCDFile(path.toStdString(), loaded), 0);
    ASSERT_EQ(loaded.size(), 50u);
}

// ===== PCLPointCloud2 (blob) 接口 =====

TEST(PcdFormatTest, PCLPointCloud2_LoadAndConvert) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/blob.pcd";

    pcl::PointCloud<pcl::PointXYZRGB> original;
    pcl::PointXYZRGB pt;
    pt.x = 5; pt.y = 6; pt.z = 7; pt.r = 200; pt.g = 100; pt.b = 50;
    original.push_back(pt);

    ASSERT_EQ(pcl::io::savePCDFile(path.toStdString(), original, true), 0);

    // 通过 PCLPointCloud2 加载
    pcl::PCLPointCloud2 blob;
    ASSERT_EQ(pcl::io::loadPCDFile(path.toStdString(), blob), 0);
    EXPECT_EQ(blob.width * blob.height, 1u);

    // 转换回结构化点云
    pcl::PointCloud<pcl::PointXYZRGB> converted;
    pcl::fromPCLPointCloud2(blob, converted);
    ASSERT_EQ(converted.size(), 1u);
    EXPECT_EQ(converted[0].r, 200);
    EXPECT_EQ(converted[0].g, 100);
    EXPECT_EQ(converted[0].b, 50);
}

// ===== 大坐标（GlobalShift 场景模拟） =====

TEST(PcdFormatTest, LargeCoordinates_Precision) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());
    QString path = tmpDir.path() + "/large_coords.pcd";

    // 模拟 UTM 大坐标
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back({500123.456f, 300456.789f, 10.5f});
    cloud.push_back({500124.456f, 300457.789f, 11.0f});

    ASSERT_EQ(pcl::io::savePCDFile(path.toStdString(), cloud, true), 0);

    pcl::PointCloud<pcl::PointXYZ> loaded;
    ASSERT_EQ(pcl::io::loadPCDFile(path.toStdString(), loaded), 0);
    ASSERT_EQ(loaded.size(), 2u);

    // 大坐标精度：float 精度约 7 位有效数字
    EXPECT_NEAR(loaded[0].x, 500123.456f, 0.1f);
    EXPECT_NEAR(loaded[0].y, 300456.789f, 0.1f);
    EXPECT_NEAR(loaded[0].z, 10.5f, 1e-3f);
}
