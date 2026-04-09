#ifndef POINTWORKS_PROJECT_MANAGER_H
#define POINTWORKS_PROJECT_MANAGER_H

#include <QObject>
#include <QString>
#include <QTreeWidgetItem>

#include "io/projectfile.h"

namespace ct { class CloudTree; }
namespace ct { class CloudView; }

/// 项目管理器：保存/打开项目，跟踪修改状态
class ProjectManager : public QObject
{
    Q_OBJECT
public:
    explicit ProjectManager(QObject* parent = nullptr);

    bool saveProject(const QString& path, ct::CloudTree* tree, ct::CloudView* view);
    bool openProject(const QString& path, ct::CloudTree* tree, ct::CloudView* view);

    QString currentProjectPath() const { return m_current_path; }
    void setCurrentPath(const QString& path) { m_current_path = path; }
    bool isModified() const { return m_modified; }

    void markModified() { m_modified = true; emit modificationChanged(true); }
    void clearModified() { m_modified = false; emit modificationChanged(false); }

signals:
    void modificationChanged(bool modified);
    void projectOpened(const QString& path);
    void projectSaved(const QString& path);
    void loadProgress(const QString& msg, int percent);
    void loadError(const QString& msg);

private:
    void collectCloudEntries(ct::CloudTree* tree, ct::CloudView* view,
                             const QString& projectDir, ct::ProjectData& data);
    void collectTreeNodes(ct::CloudTree* tree, QList<ct::TreeNode>& roots);
    ct::TreeNode treeNodeFromItem(QTreeWidgetItem* item, ct::CloudTree* tree);
    QTreeWidgetItem* rebuildTreeNode(QTreeWidgetItem* parent, const ct::TreeNode& node, ct::CloudTree* tree);

    QString m_current_path;
    bool m_modified = false;
    int m_pending_loads = 0;
    int m_total_loads = 0;
};

#endif // POINTWORKS_PROJECT_MANAGER_H
