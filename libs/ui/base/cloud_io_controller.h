#ifndef POINTWORKS_CLOUD_IO_CONTROLLER_H
#define POINTWORKS_CLOUD_IO_CONTROLLER_H

#include "core/cloud.h"
#include "core/textured_mesh.h"
#include "core/field_types.h"

#include <QObject>
#include <QThread>
#include <QMap>

#include <pcl/PolygonMesh.h>
#include <Eigen/Core>

class QTreeWidgetItem;
class QWidget;

namespace ct
{
    class FileIO;
    class ProgressManager;

    class CloudIOController : public QObject
    {
        Q_OBJECT
    public:
        explicit CloudIOController(QObject* parent = nullptr);
        ~CloudIOController();

        void init(QWidget* parentWidget, ProgressManager* progress);

        void addCloud();
        void loadCloudFile(const QString& filepath);
        void loadCloudFile(const QString& filepath, QTreeWidgetItem* targetParent);
        void saveCloudFile(const Cloud::Ptr& cloud, const QString& filepath, bool isBinary = true);
        void saveMeshFile(const pcl::PolygonMesh::Ptr& mesh, const QString& filename);

        void setScriptMode(bool enabled) { m_script_mode = enabled; }
        bool scriptMode() const { return m_script_mode; }
        void setPath(const QString& path) { m_path = path; }

        void setPendingParents(const QMap<QString, QTreeWidgetItem*>& parents) { m_pending_parents = parents; }
        void clearPendingParents() { m_pending_parents.clear(); }

        Eigen::Vector3d lastShift() const { return m_last_shift; }
        bool hasLastShift() const { return m_hasLastShift; }

    signals:
        void cloudLoaded(const Cloud::Ptr& cloud, const pcl::PolygonMesh::Ptr& mesh,
                         QTreeWidgetItem* targetParent, float loadTime);
        void texturedMeshLoaded(const QString& cloudId, const QString& objFilePath);
        void saveComplete(bool success, const QString& path, float time);
        void meshSaveComplete(bool success, const QString& path, float time);

    private slots:
        void onLoadCloudResult(bool success, const Cloud::Ptr& cloud, const pcl::PolygonMesh::Ptr& mesh, float time);
        void onSaveCloudResult(bool success, const QString& path, float time);
        void onSaveMeshResult(bool success, const QString& path, float time);
        void onLoadTexturedMeshResult(const QString& cloudId, const QString& objFilePath);
        void onFieldMappingRequested(const QList<ct::FieldInfo>& fields, std::map<std::string, std::string>& result);
        void onTxtImportRequested(const QStringList& preview_lines, ct::TxtImportParams& params);
        void onTxtExportRequested(const QStringList& available_fields, ct::TxtExportParams& params);
        void onGlobalFilterRequested(const Eigen::Vector3d& min_pt, Eigen::Vector3d& suggested_shift, bool& skipped);

    private:
        QWidget* m_parent_widget = nullptr;
        ProgressManager* m_progress = nullptr;
        QString m_path;
        QThread m_thread;
        FileIO* m_fileio = nullptr;
        QMap<QString, QTreeWidgetItem*> m_pending_parents;
        Eigen::Vector3d m_last_shift = Eigen::Vector3d::Zero();
        bool m_hasLastShift = false;
        bool m_script_mode = false;
    };
}

#endif // POINTWORKS_CLOUD_IO_CONTROLLER_H
