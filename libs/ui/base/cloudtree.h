#ifndef POINTWORKS_CLOUDTREE_H
#define POINTWORKS_CLOUDTREE_H

#include "customtree.h"
#include "dialog/processingdialog.h"
#include "dialog/globalshiftdialog.h"
#include "io/fileio.h"

#include <QMenu>
#include <QMap>
#include <QSet>
#include <QInputDialog>
#include <QThread>

#include <pcl/PolygonMesh.h>

#define CLONE_ADD_FLAG "clone-"
#define MERGE_ADD_FLAG "merge-"

namespace ct
{
    /// 点云挂载策略
    enum class MountStrategy
    {
        Auto,       // 自动判断（默认）
        Sibling,    // 策略一：作为兄弟节点挂载
        Child,      // 策略三：作为子节点挂载
        Group,      // 策略二：创建分组挂载
        Root        // 直接作为根节点
    };

    class CloudTree : public CustomTree
    {
    Q_OBJECT
    public:
        explicit CloudTree(QWidget* parent = nullptr);

        ~CloudTree() override;

        /**
         * @brief 添加点云 (弹出文件对话框)
         */
        void addCloud();

        /**
         * @brief 加载点云文件 (程序化调用，无对话框)
         * @param filepath 文件路径
         */
        void loadCloudFile(const QString& filepath);

        /**
         * @brief 加载点云文件到指定父节点下（用于项目恢复）
         * @param filepath 文件路径
         * @param targetParent 目标父节点
         */
        void loadCloudFile(const QString& filepath, QTreeWidgetItem* targetParent);

        /**
         * @brief 保存点云文件 (程序化调用，无对话框)
         * @param cloud 点云对象
         * @param filepath 文件路径
         * @param isBinary 是否二进制格式
         */
        void saveCloudFile(const Cloud::Ptr& cloud, const QString& filepath, bool isBinary = true);

        /**
        * @brief 更新点云数据
        */
        void updateCloud(const Cloud::Ptr& cloud, const Cloud::Ptr& new_cloud, bool update_name = false);

        /**
         * @brief 获取Item对应的Cloud
         */
        Cloud::Ptr getCloud(QTreeWidgetItem* item);

        /**
         * @brief 根据ID获取Item
         */
        QTreeWidgetItem* getItemById(const QString& id);

        /**
        * @brief 插入点云到树中 (核心函数)
        * @param cloud 点云对象
        * @param parentItem 父节点项。如果为 nullptr，则作为根节点(或文件夹下的子节点)
        * @param selected 是否选中新节点
        * @param strategy 挂载策略
        */
        void insertCloud(const Cloud::Ptr& cloud, QTreeWidgetItem* parent = nullptr,
                         bool selected = false, MountStrategy strategy = MountStrategy::Auto);

        /**
         * @brief 策略一：将结果作为兄弟节点挂载（滤波、采样、配准等）
         * @param sourceCloud 源点云
         * @param resultCloud 结果点云
         * @param suffix 结果点云 ID 后缀
         */
        void addSiblingCloud(const Cloud::Ptr& sourceCloud, const Cloud::Ptr& resultCloud,
                             const QString& suffix = "-result");

        /**
         * @brief 获取选中的所有点云对象
         */
        std::vector<Cloud::Ptr> getSelectedClouds();

        /**
         * @brief 获取所有点云对象
         */
        std::vector<Cloud::Ptr> getAllClouds();

        /**
         * @brief 移除选中的点云
         */
        void removeSelectedClouds();

        /**
         * @@brief 移除所有点云
         */
        void removeAllClouds();

        /**
         * @@brief 保存选中的点云
         */
        void saveSelectedClouds();

        /**
         * @brief 合并选中的点云项目
         */
        void mergeSelectedClouds();

        /**
         * @brief 克隆选中的点云项目
         */
        void cloneSelectedClouds();

        /**
         * @brief 设置勾选点云,设置点云的选中状态
         */
        void setCloudChecked(const Cloud::Ptr& cloud, bool checked = true);

        /**
         * @brief 设置选中点云
         */
        void setCloudSelected(const Cloud::Ptr& cloud, bool selected = true);

        /**
         * @brief 显示模态进度条
         */
        void showProgress(const QString& message);

        /**
         * @brief 关闭进度条
         */
        void closeProgress();

        /**
         * @brief 设置进度条百分比
         */
        void setProgress(int percent);

        /**
         * @brief 将任意后台 Worker (如 FileIO, GroundFilter) 绑定到当前进度条
         */
        void bindWorker(QObject* worker);

        /**
         * @brief 添加点云处理后，得到的新结果数据项
         * @param originCloud 原始数据
         * @param results 新结果数据
         * @param groupName 新结果数据项的组名
         */
        void addResultGroup(const Cloud::Ptr& originCloud, const std::vector<Cloud::Ptr>& results, const QString& groupName);

        /**
         * @brief 注册 PolygonMesh 到树节点（与节点可见性联动）
         * @param cloudId 关联的 Cloud UUID（已通过 insertCloud 添加的节点）
         * @param mesh PolygonMesh 数据
         */
        void registerMesh(const QString& cloudId, const pcl::PolygonMesh::Ptr& mesh);

        /**
         * @brief 注册带纹理的 PolygonMesh 到树节点
         */
        void registerTexturedMesh(const QString& cloudId, const TexturedMeshPtr& texturedMesh);

        /**
         * @brief 移除已注册的纹理 PolygonMesh
         */
        void unregisterTexturedMesh(const QString& cloudId);

        /**
         * @brief 移除已注册的 PolygonMesh
         */
        void unregisterMesh(const QString& cloudId);

        /**
         * @brief 获取所有已注册的 PolygonMesh 列表
         * @return QList of (cloudId, mesh) pairs
         */
        QList<QPair<QString, pcl::PolygonMesh::Ptr>> getLoadedMeshes() const;

        /**
         * @brief 查询指定点云是否关联了 PolygonMesh
         */
        bool hasMesh(const QString& cloudId) const;

        /**
         * @brief 聚焦视图到选中点云
         * 如果有选中项 -> 聚焦选中项的并集
         * 如果无选中项 -> 聚焦所有可见点云
         */
        void zoomToSelected();

    protected:
        /**
        * @brief 右键上下文菜单
        */
        void contextMenuEvent(QContextMenuEvent* event) override;

        /**
        * @brief 移除指定节点及其数据 (递归删除子节点)
        */
        void removeCloudItem(QTreeWidgetItem* item);

        /**
         * @brief 保存指定节点的数据
         */
        void saveCloudItem(QTreeWidgetItem* item);

        /**
         * @brief 克隆指定节点
         */
        void cloneCloudItem(QTreeWidgetItem* item);

        /**
         * @brief 重命名
         */
        void renameCloudItem(QTreeWidgetItem* item, const QString& name);

    signals:
        /**
         * @brief加载点云文件
         */
        void loadPointCloud(const QString& filename );

        /**
         * @brief 保存点云文件
         */
        void savePointCloud(const Cloud::Ptr& cloud, const QString& filename, bool isBinary);

        /**
         * @brief 保存 mesh 文件
         */
        void saveMeshFile(const pcl::PolygonMesh::Ptr& mesh, const QString& filename);

        /**
         * @brief 删除点云的ID
         */
        void removedCloudId(const QString&);

        /**
         * @brief 点云插入完成（Bridge 用于同步注册表）
         */
        void cloudInserted(Cloud::Ptr cloud);

    private slots:
        /**
         * @brief 加载点云文件的结果（含 mesh）
         */
        void loadCloudResult(bool success, const Cloud::Ptr& cloud, const pcl::PolygonMesh::Ptr& mesh, float time);

        /**
         * @brief 加载带纹理网格的结果
         */
        void loadTexturedMeshResult(const QString& cloudId, const QString& objFilePath);

    public slots:
        /// 标记点云为"脚本使用中"
        void markCloudInUse(const QString& id);
        /// 取消"脚本使用中"标记
        void unmarkCloudInUse(const QString& id);
        /// 释放所有 in-use 标记
        void releaseAllInUse();

        /**
         * @brief 保存点云文件的结果
         */
        void saveCloudResult(bool success, const QString& path, float time);

        /**
         * @brief 保存 mesh 文件的结果
         */
        void saveMeshResult(bool success, const QString& path, float time);

        /**
         * @brief 处理复选框状态改变事件
         */
        void itemChangedEvent(QTreeWidgetItem* item, int column);

        /**
         * @brief 项目选中改变事件
         * @note 函数重写（覆盖），重写基类CustomTree中的itemSelectionChangedEvent函数
         */
         void itemSelectionChangedEvent();

         void onFieldMappingRequested(const QList<ct::FieldInfo>& fields, std::map<std::string, std::string>& result);

         void onTxtImportRequested(const QStringList& preview_lines, ct::TxtImportParams& params);

         void onTxtExportRequested(const QStringList& available_fields, ct::TxtExportParams& params);

        /**
        * @brief 请求全局偏移设置 (阻塞式信号)
        * @param bounding_min 原始数据的最小点 (用于显示)
        * @param shift_suggest 建议的 Shift 值 (输入/输出)
        * @param is_skipped 用户是否点击了 No (跳过 Shift)
        */
         void onGlobalFilterRequested(const Eigen::Vector3d& min_pt, Eigen::Vector3d& suggested_shift, bool& skipped);


    private:
        QMap<QTreeWidgetItem*, Cloud::Ptr> m_cloud_map;
        QMap<QString, QTreeWidgetItem*> m_item_by_id;  // uuid -> item 反向索引
        QMap<QString, pcl::PolygonMesh::Ptr> m_mesh_map; // cloudId -> PolygonMesh
        QMap<QString, TexturedMeshPtr> m_textured_mesh_map; // cloudId -> TexturedMesh
        QString m_path;
        QThread m_thread;
        FileIO* m_fileio;
        QMenu* m_tree_menu;
        QMap<QString, QTreeWidgetItem*> m_pending_parents;  // filepath → 目标父节点（恢复模式）

    public:
        ProcessingDialog* m_processing_dialog = nullptr;

        Eigen::Vector3d m_last_shift = Eigen::Vector3d::Zero();
        bool m_hasLastShift = false;

        // 用于记录加载队列中的点云数量
        int m_loading_queue_count = 0;

        // 被脚本使用中的点云 ID 集合（删除保护）
        QSet<QString> m_clouds_in_use;

        /// 脚本模式：跳过弹窗，自动使用默认值
        bool m_script_mode = false;
        void setScriptMode(bool enabled) { m_script_mode = enabled; }
    };
}

// Qt meta-type 声明 (跨线程信号/槽需要)
Q_DECLARE_METATYPE(ct::Cloud::Ptr)
using ct_StringFieldMap = std::map<std::string, std::string>;
Q_DECLARE_METATYPE(ct_StringFieldMap)
Q_DECLARE_METATYPE(ct::TxtImportParams)
Q_DECLARE_METATYPE(ct::TxtExportParams)
Q_DECLARE_METATYPE(Eigen::Vector3d)
Q_DECLARE_METATYPE(ct::TexturedMeshPtr)

#endif //POINTWORKS_CLOUDTREE_H
