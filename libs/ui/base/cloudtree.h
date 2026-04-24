#ifndef POINTWORKS_CLOUDTREE_H
#define POINTWORKS_CLOUDTREE_H

#include "customtree.h"
#include "base/scenenodetype.h"
#include "core/cloud.h"
#include "core/textured_mesh.h"
#include "base/progress_manager.h"
#include "base/cloudregistry.h"
#include "base/cloud_io_controller.h"

#include <QMenu>
#include <QInputDialog>

#include <pcl/PolygonMesh.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

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

        void addCloud();
        void loadCloudFile(const QString& filepath);
        void loadCloudFile(const QString& filepath, QTreeWidgetItem* targetParent);
        void saveCloudFile(const Cloud::Ptr& cloud, const QString& filepath, bool isBinary = true);

        void updateCloud(const Cloud::Ptr& cloud, const Cloud::Ptr& new_cloud, bool update_name = false);

        Cloud::Ptr getCloud(QTreeWidgetItem* item);
        QTreeWidgetItem* getItemById(const QString& id);

        void insertCloud(const Cloud::Ptr& cloud, QTreeWidgetItem* parent = nullptr,
                         bool selected = false, MountStrategy strategy = MountStrategy::Auto,
                         SceneNodeType nodeType = NodeCloud);

        void addSiblingCloud(const Cloud::Ptr& sourceCloud, const Cloud::Ptr& resultCloud,
                             const QString& suffix = "-result");

        std::vector<Cloud::Ptr> getSelectedClouds();
        std::vector<Cloud::Ptr> getSelectedCloudsOnly() const;
        std::vector<Cloud::Ptr> getAllClouds();

        SelectionInfo getSelectionInfo() const;
        int getTotalCloudCount() const;
        int getTotalMeshCount() const;

        void removeSelectedClouds();
        void removeAllClouds();
        void saveSelectedClouds();
        void mergeSelectedClouds();
        void cloneSelectedClouds();

        void setCloudChecked(const Cloud::Ptr& cloud, bool checked = true);
        void setCloudSelected(const Cloud::Ptr& cloud, bool selected = true);

        QList<int> getItemViewports(QTreeWidgetItem* item) const;
        void setItemViewports(QTreeWidgetItem* item, const QList<int>& indices);
        QList<CloudView*> resolveViews(QTreeWidgetItem* item) const;
        CloudView* resolveView(QTreeWidgetItem* item) const;
        void assignCloudToViewport(QTreeWidgetItem* item, int viewportIndex);
        void repopulateAllViews();
        void updateItemViewportLabel(QTreeWidgetItem* item);

        void showProgress(const QString& message);
        void closeProgress();
        void setProgress(int percent);
        void bindWorker(QObject* worker);

        ProgressManager* progressManager() { return m_progress; }
        CloudRegistry* registry() { return m_registry; }
        const CloudRegistry* registry() const { return m_registry; }
        CloudIOController* ioController() { return m_io; }

        void addResultGroup(const Cloud::Ptr& originCloud, const std::vector<Cloud::Ptr>& results, const QString& groupName);

        void registerMesh(const QString& cloudId, const pcl::PolygonMesh::Ptr& mesh);
        void registerMeshPrebuilt(const QString& cloudId, const pcl::PolygonMesh::Ptr& mesh,
                                   vtkSmartPointer<vtkPolyData> polydata);
        void registerTexturedMesh(const QString& cloudId, const TexturedMeshPtr& texturedMesh);
        void unregisterTexturedMesh(const QString& cloudId);
        void unregisterMesh(const QString& cloudId);

        void registerShape(const QString& parentCloudId, const QString& shapeId,
                           const QString& displayName,
                           const pcl::PolygonMesh::Ptr& mesh = nullptr);
        void registerCloudPolyline(const QString& parentCloudId, const QString& shapeId,
                                   const QString& displayName, const Cloud::Ptr& cloud);

        QList<QPair<QString, pcl::PolygonMesh::Ptr>> getLoadedMeshes() const;
        bool hasMesh(const QString& cloudId) const;

        void zoomToSelected();

        void setScriptMode(bool enabled) { m_io->setScriptMode(enabled); }

    protected:
        void contextMenuEvent(QContextMenuEvent* event) override;
        void removeCloudItem(QTreeWidgetItem* item);
        void saveCloudItem(QTreeWidgetItem* item);
        void cloneCloudItem(QTreeWidgetItem* item);
        void renameCloudItem(QTreeWidgetItem* item, const QString& name);
        QString makeUniqueName(const QString& desiredName);

    signals:
        void removedCloudId(const QString&);
        void cloudInserted(Cloud::Ptr cloud);

    private slots:
        void handleCloudLoaded(const Cloud::Ptr& cloud, const pcl::PolygonMesh::Ptr& mesh,
                               QTreeWidgetItem* targetParent, float time);
        void handleTexturedMeshLoaded(const QString& cloudId, const QString& objFilePath);
        void handleSaveComplete(bool success, const QString& path, float time);
        void handleMeshSaveComplete(bool success, const QString& path, float time);

        void itemChangedEvent(QTreeWidgetItem* item, int column);
        void itemSelectionChangedEvent();

    public slots:
        void markCloudInUse(const QString& id);
        void unmarkCloudInUse(const QString& id);
        void releaseAllInUse();

    private:
        CloudRegistry* m_registry;
        CloudIOController* m_io;
        QMenu* m_tree_menu;
        ProgressManager* m_progress;

        int countNodesOfType(QTreeWidgetItem* parent, SceneNodeType type) const;
    };
}

Q_DECLARE_METATYPE(ct::Cloud::Ptr)

#endif //POINTWORKS_CLOUDTREE_H
