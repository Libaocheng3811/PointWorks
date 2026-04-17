#include <gtest/gtest.h>
#include <QCoreApplication>
#include <QDir>
#include <QTemporaryDir>
#include <fileio.h>
#include "test_helpers.h"
#include "macros.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace ct;

// ===== 测试辅助：处理 FileIO 的阻塞信号 =====

// 临时 QCoreApplication，在第一个 IO 测试中创建
static QCoreApplication* s_app = nullptr;

/**
 * @brief 创建一个配置好信号连接的 FileIO 实例
 * FileIO 的 loadPointCloud 使用阻塞信号 requestFieldMapping/requestGlobalShift，
 * 必须连接 slot 才能正常工作。
 */
static std::unique_ptr<FileIO> createTestFileIO() {
    auto fileio = std::make_unique<FileIO>();

    // 连接 requestFieldMapping：对标准字段提供默认映射
    QObject::connect(fileio.get(), &FileIO::requestFieldMapping,
        [](const QList<ct::FieldInfo>& fields,
           std::map<std::string, std::string>& result)
    {
        for (const auto& f : fields) {
            const std::string& name = f.name;
            if (name == "x" || name == "y" || name == "z") {
                result[name] = "Ignore";  // XYZ 始终被提取，映射值不影响坐标
            }
            else if (name == "rgb" || name == "rgba") {
                result[name] = "Color";
            }
            else if (name == "normal_x" || name == "normal_x") {
                result[name] = "Normal X";
            }
            else if (name == "normal_y") {
                result[name] = "Normal Y";
            }
            else if (name == "normal_z") {
                result[name] = "Normal Z";
            }
            else if (name == "r" || name == "red") {
                result[name] = "Red";
            }
            else if (name == "g" || name == "green") {
                result[name] = "Green";
            }
            else if (name == "b" || name == "blue") {
                result[name] = "Blue";
            }
            else {
                result[name] = "Ignore";
            }
        }
    });

    // 连接 requestGlobalShift：始终跳过
    QObject::connect(fileio.get(), &FileIO::requestGlobalShift,
        [](const Eigen::Vector3d&, Eigen::Vector3d&, bool& is_skipped) {
            is_skipped = true;
        });

    // 连接 requestTxtImportSetup：使用默认空格分隔
    QObject::connect(fileio.get(), &FileIO::requestTxtImportSetup,
        [](const QStringList&, ct::TxtImportParams& params) {
            params.separator = ' ';
            params.col_map[0] = "X";
            params.col_map[1] = "Y";
            params.col_map[2] = "Z";
        });

    return fileio;
}

/**
 * @brief 用 PCL 直接保存 PCD 文件（绕过 FileIO 信号机制）
 */
static bool saveTestPCD(const QString& path, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         bool binary = true) {
    return pcl::io::savePCDFile(path.toStdString(), *cloud, binary) == 0;
}

static bool saveTestPCDXYZRGB(const QString& path, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                bool binary = true) {
    return pcl::io::savePCDFile(path.toStdString(), *cloud, binary) == 0;
}

// ===== 测试 Fixture =====

class FileIOTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 确保 QCoreApplication 存在
        if (!s_app) {
            static int argc = 1;
            static char argv0[] = "test_io";
            static char* argv[] = { argv0 };
            s_app = new QCoreApplication(argc, argv);
        }
    }

    void TearDown() override {
        // 清理临时文件
        for (const auto& f : temp_files_) {
            if (QFile::exists(f)) QFile::remove(f);
        }
    }

    void trackTempFile(const QString& path) { temp_files_.push_back(path); }

private:
    QStringList temp_files_;
};

// ===== PCD 加载 =====

TEST_F(FileIOTest, LoadPCD_Binary) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());

    QString path = tmpDir.path() + "/test_binary.pcd";
    trackTempFile(path);

    // 用 PCL 创建测试文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud->push_back({1.0f, 2.0f, 3.0f});
    pcl_cloud->push_back({4.0f, 5.0f, 6.0f});
    pcl_cloud->push_back({7.0f, 8.0f, 9.0f});
    ASSERT_TRUE(saveTestPCD(path, pcl_cloud, true));

    // 通过 FileIO 加载
    auto fileio = createTestFileIO();
    Cloud::Ptr loaded_cloud;
    bool success = false;

    QObject::connect(fileio.get(), &FileIO::loadCloudResult,
        [&loaded_cloud, &success](bool ok, const Cloud::Ptr& cloud, float) {
            loaded_cloud = cloud;
            success = ok;
        });

    fileio->loadPointCloud(path);

    // 处理事件（DirectConnection 不需要，但保险起见）
    QCoreApplication::processEvents();

    ASSERT_TRUE(success);
    ASSERT_CLOUD_SIZE_EQ(loaded_cloud, 3);
}

TEST_F(FileIOTest, LoadPCD_ASCII) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());

    QString path = tmpDir.path() + "/test_ascii.pcd";
    trackTempFile(path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 100; ++i) {
        pcl_cloud->push_back({float(i), float(i * 2), float(i * 3)});
    }
    ASSERT_TRUE(saveTestPCD(path, pcl_cloud, false));

    auto fileio = createTestFileIO();
    Cloud::Ptr loaded_cloud;
    bool success = false;

    QObject::connect(fileio.get(), &FileIO::loadCloudResult,
        [&loaded_cloud, &success](bool ok, const Cloud::Ptr& cloud, float) {
            loaded_cloud = cloud;
            success = ok;
        });

    fileio->loadPointCloud(path);
    QCoreApplication::processEvents();

    ASSERT_TRUE(success);
    ASSERT_CLOUD_SIZE_EQ(loaded_cloud, 100);
}

// ===== PLY 加载 =====

TEST_F(FileIOTest, LoadPLY_Basic) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());

    QString path = tmpDir.path() + "/test.ply";
    trackTempFile(path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 50; ++i) {
        pcl_cloud->push_back({float(i), 0.0f, 0.0f});
    }
    ASSERT_EQ(pcl::io::savePLYFile(path.toStdString(), *pcl_cloud, true), 0);

    auto fileio = createTestFileIO();
    Cloud::Ptr loaded_cloud;
    bool success = false;

    QObject::connect(fileio.get(), &FileIO::loadCloudResult,
        [&loaded_cloud, &success](bool ok, const Cloud::Ptr& cloud, float) {
            loaded_cloud = cloud;
            success = ok;
        });

    fileio->loadPointCloud(path);
    QCoreApplication::processEvents();

    ASSERT_TRUE(success);
    ASSERT_CLOUD_SIZE_EQ(loaded_cloud, 50);
}

// ===== 保存与加载 Roundtrip =====

TEST_F(FileIOTest, PCD_SaveAndLoad_Roundtrip) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());

    // 创建 Cloud 并保存
    auto cloud = test_helpers::makePlane(200, 10.0f, 10.0f, 0.0f, 42);

    QString path = tmpDir.path() + "/roundtrip.pcd";
    trackTempFile(path);

    auto fileio = createTestFileIO();
    bool save_ok = false;

    QObject::connect(fileio.get(), &FileIO::saveCloudResult,
        [&save_ok](bool ok, const QString&, float) {
            save_ok = ok;
        });

    fileio->savePointCloud(cloud, path, true);
    QCoreApplication::processEvents();

    ASSERT_TRUE(save_ok) << "Save failed";
    ASSERT_TRUE(QFile::exists(path));

    // 重新加载
    Cloud::Ptr loaded_cloud;
    bool load_ok = false;

    QObject::connect(fileio.get(), &FileIO::loadCloudResult,
        [&loaded_cloud, &load_ok](bool ok, const Cloud::Ptr& c, float) {
            loaded_cloud = c;
            load_ok = ok;
        });

    fileio->loadPointCloud(path);
    QCoreApplication::processEvents();

    ASSERT_TRUE(load_ok);
    ASSERT_CLOUD_SIZE_EQ(loaded_cloud, 200);
}

TEST_F(FileIOTest, PLY_SaveAndLoad_Roundtrip) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());

    auto cloud = test_helpers::makeSphere(100, 5.0f, 0.0f, 42);
    QString path = tmpDir.path() + "/roundtrip.ply";
    trackTempFile(path);

    auto fileio = createTestFileIO();
    bool save_ok = false;

    QObject::connect(fileio.get(), &FileIO::saveCloudResult,
        [&save_ok](bool ok, const QString&, float) {
            save_ok = ok;
        });

    fileio->savePointCloud(cloud, path, true);
    QCoreApplication::processEvents();

    ASSERT_TRUE(save_ok);

    Cloud::Ptr loaded_cloud;
    bool load_ok = false;

    QObject::connect(fileio.get(), &FileIO::loadCloudResult,
        [&loaded_cloud, &load_ok](bool ok, const Cloud::Ptr& c, float) {
            loaded_cloud = c;
            load_ok = ok;
        });

    fileio->loadPointCloud(path);
    QCoreApplication::processEvents();

    ASSERT_TRUE(load_ok);
    ASSERT_CLOUD_SIZE_EQ(loaded_cloud, 100);
}

// ===== 错误处理 =====

TEST_F(FileIOTest, LoadNonexistentFile) {
    auto fileio = createTestFileIO();
    bool success = false;

    QObject::connect(fileio.get(), &FileIO::loadCloudResult,
        [&success](bool ok, const Cloud::Ptr&, float) {
            success = ok;
        });

    fileio->loadPointCloud("/nonexistent/path/file.pcd");
    QCoreApplication::processEvents();

    EXPECT_FALSE(success);
}

TEST_F(FileIOTest, LoadUnsupportedFormat) {
    QTemporaryDir tmpDir;
    ASSERT_TRUE(tmpDir.isValid());

    QString path = tmpDir.path() + "/test.xyzzy";
    trackTempFile(path);

    // 创建一个空文件
    QFile f(path);
    f.open(QIODevice::WriteOnly);
    f.write("some random content");
    f.close();

    auto fileio = createTestFileIO();
    bool success = false;

    QObject::connect(fileio.get(), &FileIO::loadCloudResult,
        [&success](bool ok, const Cloud::Ptr&, float) {
            success = ok;
        });

    fileio->loadPointCloud(path);
    QCoreApplication::processEvents();

    // 不支持的格式应失败
    EXPECT_FALSE(success);
}

// ===== 自定义 main（QCoreApplication 必须在使用 Qt 信号前创建） =====

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    s_app = &app;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
