#ifndef POINTWORKS_FILEIO_H
#define POINTWORKS_FILEIO_H

#include "core/cloud.h"
#include "core/field_types.h"
#include "core/exports.h"

#include <atomic>
#include <QObject>
#include <QString>
#include <QMap>
#include <vector>
#include <map>
#include <string>

namespace ct
{
    /**
    * @brief 流式加载缓冲区
    * @details 用于在读取文件时暂存一批数据，满额后一次性提交给 Cloud 进行八叉树插入
    */
    struct CloudBatch {
        std::vector<PointXYZ> points;
        std::vector<RGB> colors;
        std::vector<CompressedNormal> normals;
        std::map<std::string, std::vector<float>> scalars;

        // 预留大小，避免频繁扩容
        void reserve(size_t size) {
            points.reserve(size);
            colors.reserve(size);
            normals.reserve(size);
            // scalars 在运行时动态添加，无法预先 reserve，但在 addPoints 时会自动处理
        }

        void clear() {
            points.clear();
            colors.clear();
            normals.clear();
            // 清空数据但保留 Key，避免重复构造 Map
            for (auto it = scalars.begin(); it != scalars.end(); ++it) {
                it->second.clear();
            }
        }

        // 检查是否为空
        bool empty() const { return points.empty(); }

        // 提交数据到 Cloud
        void flushTo(Cloud::Ptr& cloud) {
            if (points.empty()) return;

            cloud->addPoints(points,
                             colors.empty() ? nullptr : &colors,
                             normals.empty() ? nullptr : &normals,
                             scalars.empty() ? nullptr : &scalars);
            clear();
        }
    };

    class CT_IO_EXPORT FileIO : public QObject
    {
        Q_OBJECT
    public:
        explicit FileIO(QObject *parent = nullptr) : QObject(parent) {}

    signals:
        /**
         * @brief 加载点云文件的结果
         * @note const Cloud::Ptr &cloud 表示参数cloud是对Cloud::Ptr 类型的智能指针的引用，且这个引用是常量
         */
        void loadCloudResult(bool success, const Cloud::Ptr &cloud, float time);

        /**
         * @brief 保存点云文件的结果
         */
        void saveCloudResult(bool success, const QString &filename, float time);

        /**
        * @brief 请求UI显示映射对话框 (阻塞式)
        * @param fields 文件中探测到的字段列表
        * @param result 用户选择的映射结果 (引用传出)
        */
        void requestFieldMapping(const QList<ct::FieldInfo>& fields, std::map<std::string, std::string>& result);

        /**
         * @brief 显示映射对话框 (阻塞式)
         * @param preview_lines 文件中探测到的字段列表
         * @param params 用户选择的映射结果 (引用传入)
         */
         void requestTxtImportSetup(const QStringList& preview_lines, ct::TxtImportParams& params);

         /**
          * @brief 显示导出对话框 (阻塞式)
          * @param available_fields 告诉UI 可用的字段列表
          * @param params 接收用户配置
          */
         void requestTxtExportSetup(const QStringList& available_fields, ct::TxtExportParams& params);

         /**
          * @brief 进度信号
          */
         void progress(int percent);

         /**
          * @brief 请求全局偏移设置
          * @param bounding_min 原始数据的最小点 (用于显示)
          * @param suggested_shift 建议的偏移值
          * @param is_skipped 用户是否选择跳过大坐标偏移，使用大坐标显示
          */
         void requestGlobalShift(const Eigen::Vector3d& bounding_min, Eigen::Vector3d& suggested_shift, bool& is_skipped);

    public slots:
        /**
         * @brief 加载点云文件
         * @param filename 点云文件路径
         */
        void loadPointCloud(const QString &filename);

        /**
         * @brief 保存点云文件
         * @param cloud 点云数据
         * @param filename 保存的文件路径
         */
        void savePointCloud(const Cloud::Ptr &cloud, const QString &filename, bool isBinary);

        /**
         * @brief 取消当前操作
         */
        void cancel() { m_is_canceled = true;};

    private:
        bool loadLAS(const QString &filename, Cloud::Ptr &cloud);
        bool loadPLY_PCD(const QString &filename, Cloud::Ptr &cloud); // 支持自定义字段
        bool loadTXT(const QString &filename, Cloud::Ptr &cloud); // 支持交互
        bool loadGeneralPCL(const QString &filename, Cloud::Ptr &cloud); // OBJ, IFS 等标准格式

        bool saveLAS(const Cloud::Ptr &cloud, const QString &filename);
        bool saveTXT(const Cloud::Ptr &cloud, const QString &filename);
        bool savePCL(const Cloud::Ptr &cloud, const QString &filename, bool isBinary);

    private:
        std::atomic<bool> m_is_canceled{false};

        // 批处理大小 (例如 50万点提交一次)
        const size_t BATCH_SIZE = 500000;
    };
}



#endif //POINTWORKS_FILEIO_H
