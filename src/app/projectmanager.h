#ifndef POINTWORKS_PROJECT_MANAGER_H
#define POINTWORKS_PROJECT_MANAGER_H

#include <QObject>
#include <QString>
#include <QTreeWidgetItem>

#include "io/projectfile.h"
#include "recentprojects.h"

class QMenu;
class QWidget;
namespace ct { class CloudTree; }
namespace ct { class CloudView; }

/// 项目管理器：自包含控制器，管理项目保存/打开/新建/最近文件
class ProjectManager : public QObject
{
    Q_OBJECT
public:
    /// 构造即完成所有信号连接
    ProjectManager(ct::CloudTree* tree, ct::CloudView* view,
                   QMenu* recentMenu, QWidget* parentWidget);

    QString currentProjectPath() const { return m_current_path; }
    bool isModified() const { return m_modified; }

    /// 返回当前应显示的窗口标题字符串
    QString windowTitle() const;

    /// closeEvent 中调用，返回 true 表示允许关闭
    bool confirmClose();

    void markModified();
    void clearModified();

signals:
    void windowTitleChanged(const QString& title);
    void loadError(const QString& msg);

public slots:
    void onNewProject();
    void onOpenProject();
    void onSaveProject();
    void onSaveProjectAs();
    void onOpenRecentProject();
    void updateRecentMenu();

private:
    void connectSignals();

    // 项目文件读写
    bool saveProject(const QString& path);
    bool openProject(const QString& path);

    void collectCloudEntries(const QString& projectDir, ct::ProjectData& data);
    void collectTreeNodes(QList<ct::TreeNode>& roots);
    ct::TreeNode treeNodeFromItem(QTreeWidgetItem* item);
    QTreeWidgetItem* rebuildTreeNode(QTreeWidgetItem* parent, const ct::TreeNode& node);

    void updateWindowTitle();

    // 外部依赖（构造时注入，不持有所有权）
    ct::CloudTree* m_tree;
    ct::CloudView* m_view;
    QMenu* m_recent_menu;
    QWidget* m_parent_widget;

    RecentProjects m_recent_projects;
    QString m_current_path;
    bool m_modified = false;
    int m_pending_loads = 0;
    int m_total_loads = 0;
};

#endif // POINTWORKS_PROJECT_MANAGER_H
