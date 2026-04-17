#include "projectmanager.h"

#include "base/cloudtree.h"
#include "base/scenenodetype.h"
#include "viz/cloudview.h"
#include "core/cloud.h"

#include <QFileInfo>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>
#include <QTreeWidget>
#include <QAction>

// ================================================================
// 构造与信号连接
// ================================================================

ProjectManager::ProjectManager(ct::CloudTree* tree, ct::CloudView* view,
                               QMenu* recentMenu, QWidget* parentWidget)
    : QObject(parentWidget), m_tree(tree), m_view(view),
      m_recent_menu(recentMenu), m_parent_widget(parentWidget)
{
    connectSignals();
    updateRecentMenu();
    updateWindowTitle();
}

void ProjectManager::connectSignals()
{
    // 点云增删 → 标记已修改
    connect(m_tree, &ct::CloudTree::cloudInserted, this, [this](ct::Cloud::Ptr cloud) {
        markModified();
        if (!cloud->filepath().empty())
            m_recent_projects.addProject(QString::fromStdString(cloud->filepath()));
    });
    connect(m_tree, &ct::CloudTree::removedCloudId, this, [this](const QString&) {
        markModified();
    });

    // 加载错误 → 弹窗
    connect(this, &ProjectManager::loadError, this, [](const QString& msg) {
        QMessageBox::warning(nullptr, "Project Error", msg);
    });
}

// ================================================================
// 修改状态与窗口标题
// ================================================================

void ProjectManager::markModified()
{
    m_modified = true;
    updateWindowTitle();
}

void ProjectManager::clearModified()
{
    m_modified = false;
    updateWindowTitle();
}

QString ProjectManager::windowTitle() const
{
    QString projName = "Untitled";
    if (!m_current_path.isEmpty())
        projName = QFileInfo(m_current_path).completeBaseName();
    return (m_modified ? "* " : "") + projName + " - PointWorks";
}

void ProjectManager::updateWindowTitle()
{
    QString projName = "Untitled";
    if (!m_current_path.isEmpty()) {
        projName = QFileInfo(m_current_path).completeBaseName();
    }
    QString title = (m_modified ? "* " : "") + projName + " - PointWorks";
    emit windowTitleChanged(title);
}

// ================================================================
// closeEvent 委托
// ================================================================

bool ProjectManager::confirmClose()
{
    if (!m_modified) return true;

    auto ret = QMessageBox::question(m_parent_widget, "Save Project",
        "Current project has unsaved changes. Save before closing?",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    if (ret == QMessageBox::Cancel) return false;
    if (ret == QMessageBox::Save) onSaveProject();
    return true;
}

// ================================================================
// 项目操作槽函数（从 MainWindow 搬入）
// ================================================================

void ProjectManager::onNewProject()
{
    if (m_modified) {
        auto ret = QMessageBox::question(m_parent_widget, "Save Project",
            "Current project has unsaved changes. Save before creating new project?",
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        if (ret == QMessageBox::Cancel) return;
        if (ret == QMessageBox::Save) onSaveProject();
    }
    m_tree->removeAllClouds();
    m_current_path.clear();
    clearModified();
}

void ProjectManager::onOpenProject()
{
    if (m_modified) {
        auto ret = QMessageBox::question(m_parent_widget, "Save Project",
            "Current project has unsaved changes. Save before opening?",
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        if (ret == QMessageBox::Cancel) return;
        if (ret == QMessageBox::Save) onSaveProject();
    }

    QString path = QFileDialog::getOpenFileName(m_parent_widget, "Open Project", QString(),
        ct::ProjectFile::fileFilter());
    if (path.isEmpty()) return;
    openProject(path);
}

void ProjectManager::onSaveProject()
{
    if (m_current_path.isEmpty()) {
        onSaveProjectAs();
        return;
    }
    saveProject(m_current_path);
}

void ProjectManager::onSaveProjectAs()
{
    QString path = QFileDialog::getSaveFileName(m_parent_widget, "Save Project As", QString(),
        ct::ProjectFile::fileFilter());
    if (path.isEmpty()) return;
    saveProject(path);
}

void ProjectManager::onOpenRecentProject()
{
    auto* action = qobject_cast<QAction*>(sender());
    if (!action) return;
    QString path = action->data().toString();
    if (path.isEmpty()) return;

    if (path.endsWith(".pwproj", Qt::CaseInsensitive)) {
        openProject(path);
    } else {
        m_tree->loadCloudFile(path);
    }
}

// ================================================================
// 最近项目菜单
// ================================================================

void ProjectManager::updateRecentMenu()
{
    m_recent_menu->clear();
    QStringList items = m_recent_projects.projects();
    if (items.isEmpty()) {
        auto* placeholder = m_recent_menu->addAction("No Recent Files");
        placeholder->setEnabled(false);
        return;
    }
    for (const auto& p : items) {
        auto* action = m_recent_menu->addAction(p);
        action->setData(p);
        connect(action, &QAction::triggered, this, &ProjectManager::onOpenRecentProject);
    }
    m_recent_menu->addSeparator();
    auto* clearAction = m_recent_menu->addAction("Clear Recent Files");
    connect(clearAction, &QAction::triggered, this, [this]() {
        m_recent_projects.clear();
        updateRecentMenu();
    });
}

// ================================================================
// 保存项目
// ================================================================

bool ProjectManager::saveProject(const QString& path)
{
    if (!m_tree || !m_view) return false;

    ct::ProjectData data;
    data.created_at = QDateTime::currentDateTime();
    data.modified_at = data.created_at;

    QString projectDir = QFileInfo(path).absolutePath();
    collectCloudEntries(projectDir, data);
    collectTreeNodes(data.tree_roots);

    data.camera = m_view->getCameraParams();
    data.view_options = m_view->getViewOptions();

    if (!ct::ProjectFile::save(path, data)) {
        emit loadError("Failed to save project file: " + path);
        return false;
    }

    m_current_path = path;
    m_recent_projects.addProject(path);
    updateRecentMenu();
    clearModified();
    return true;
}

// ================================================================
// 打开项目
// ================================================================

bool ProjectManager::openProject(const QString& path)
{
    if (!m_tree || !m_view) return false;

    ct::ProjectData data;
    if (!ct::ProjectFile::load(path, data)) {
        emit loadError("Failed to load project file: " + path);
        return false;
    }

    QString projectDir = QFileInfo(path).absolutePath();
    m_tree->removeAllClouds();

    // 第一遍：重建树骨架（只创建文件夹/分组节点，不加载点云）
    struct PendingFile { QString filepath; QTreeWidgetItem* parentNode; };
    QList<PendingFile> pendingFiles;

    for (const auto& root : data.tree_roots) {
        QTreeWidgetItem* item = rebuildTreeNode(nullptr, root);
        if (item) m_tree->addTopLevelItem(item);

        // 递归收集所有 FileNode
        std::function<void(QTreeWidgetItem*, const ct::TreeNode&)> collectFiles;
        collectFiles = [&](QTreeWidgetItem* treeItem, const ct::TreeNode& node) {
            if ((node.type == "file" || node.type == "folder") && !node.filepath.isEmpty()) {
                QString resolved = ct::ProjectFile::resolveFilePath(projectDir, node.filepath);
                if (!resolved.isEmpty()) {
                    pendingFiles.append({resolved, treeItem});
                }
            }
            for (const auto& child : node.children) {
                collectFiles(treeItem, child);
            }
        };
        collectFiles(item, root);
    }

    // 检查文件是否存在
    QStringList missing;
    QStringList validPaths;
    for (const auto& pf : pendingFiles) {
        if (!QFileInfo::exists(pf.filepath)) {
            missing << QFileInfo(pf.filepath).fileName();
        } else {
            validPaths << pf.filepath;
        }
    }

    if (!missing.isEmpty()) {
        QString msg = "The following cloud files are missing:\n\n" + missing.join("\n") +
                     "\n\nDo you want to continue loading the available files?";
        auto ret = QMessageBox::warning(nullptr, "Missing Files", msg,
                                        QMessageBox::Yes | QMessageBox::No);
        if (ret == QMessageBox::No)
            return false;
    }

    if (validPaths.isEmpty()) {
        emit loadError("No cloud files could be found for this project.");
        return false;
    }

    m_tree->setScriptMode(true);
    m_total_loads = 0;
    for (const auto& pf : pendingFiles) {
        if (QFileInfo::exists(pf.filepath))
            ++m_total_loads;
    }
    m_pending_loads = 0;

    // 信号驱动：每插入一个点云计数，全部完成时恢复视图
    auto* conn = new QMetaObject::Connection;
    *conn = connect(m_tree, &ct::CloudTree::cloudInserted, this,
        [this, data, conn](ct::Cloud::Ptr) {
            ++m_pending_loads;
            if (m_pending_loads >= m_total_loads) {
                disconnect(*conn);
                delete conn;
                m_tree->setScriptMode(false);
                m_view->setViewOptions(data.view_options);
                m_view->setCameraParams(data.camera);
                m_view->refresh();
            }
        });

    // 第二遍：加载所有点云文件到对应的 FileNode 下
    for (const auto& pf : pendingFiles) {
        if (!QFileInfo::exists(pf.filepath)) continue;
        m_tree->loadCloudFile(pf.filepath, pf.parentNode);
    }

    m_current_path = path;
    m_recent_projects.addProject(path);
    updateRecentMenu();
    clearModified();
    return true;
}

// ================================================================
// 辅助：收集点云条目
// ================================================================

void ProjectManager::collectCloudEntries(const QString& projectDir, ct::ProjectData& data)
{
    auto allClouds = m_tree->getAllClouds();
    for (const auto& cloud : allClouds) {
        ct::CloudEntry entry;
        entry.uuid = QString::fromStdString(cloud->id());
        entry.file_path = QString::fromStdString(cloud->filepath());
        entry.display_name = entry.uuid;

        if (!entry.file_path.isEmpty()) {
            QFileInfo fi(entry.file_path);
            entry.display_name = fi.fileName();
            entry.file_path = ct::ProjectFile::toRelativePath(projectDir, entry.file_path);
        }

        entry.global_shift = cloud->getGlobalShift();
        entry.color_mode = QString::fromStdString(cloud->currentColorMode());
        entry.point_size = cloud->pointSize();
        entry.opacity = cloud->opacity();

        auto* item = m_tree->getItemById(entry.uuid);
        entry.is_visible = item ? (item->checkState(0) != Qt::Unchecked) : true;

        // 扩展元数据
        entry.has_mesh = m_tree->hasMesh(entry.uuid);
        entry.has_colors = cloud->hasColors();
        entry.has_normals = cloud->hasNormals();
        entry.cloud_type = QString::fromStdString(cloud->type());

        auto sfNames = cloud->getScalarFieldNames();
        for (const auto& name : sfNames)
            entry.scalar_fields.append(QString::fromStdString(name));

        data.clouds.append(entry);
    }
}

// ================================================================
// 辅助：收集树结构
// ================================================================

void ProjectManager::collectTreeNodes(QList<ct::TreeNode>& roots)
{
    for (int i = 0; i < m_tree->topLevelItemCount(); ++i) {
        roots.append(treeNodeFromItem(m_tree->topLevelItem(i)));
    }
}

ct::TreeNode ProjectManager::treeNodeFromItem(QTreeWidgetItem* item)
{
    ct::TreeNode node;
    ct::SceneNodeType type = ct::CustomTree::getNodeType(item);

    node.type = (type == ct::NodeFile)  ? "file" :
                (type == ct::NodeGroup) ? "group" : "cloud";
    node.text = item->text(0);
    node.expanded = item->isExpanded();
    node.is_visible = (item->checkState(0) != Qt::Unchecked);

    if (type == ct::NodeCloud) {
        ct::Cloud::Ptr cloud = m_tree->getCloud(item);
        if (cloud) node.uuid = QString::fromStdString(cloud->id());
    }
    if (type == ct::NodeFile) {
        node.filepath = item->data(0, ct::NodeFilePathRole).toString();
    }

    for (int i = 0; i < item->childCount(); ++i) {
        node.children.append(treeNodeFromItem(item->child(i)));
    }

    return node;
}

// ================================================================
// 从 TreeNode 重建 QTreeWidgetItem 树骨架
// ================================================================

QTreeWidgetItem* ProjectManager::rebuildTreeNode(QTreeWidgetItem* parent, const ct::TreeNode& node)
{
    // cloud 类型节点由 loadCloudResult → insertCloud 动态创建，不在此处重建骨架
    if (node.type == "cloud") return nullptr;

    ct::SceneNodeType type = (node.type == "file" || node.type == "folder") ? ct::NodeFile :
                             (node.type == "group") ? ct::NodeGroup :
                                                       ct::NodeCloud;

    QTreeWidgetItem* item;
    if (parent) {
        item = new QTreeWidgetItem(parent);
    } else {
        item = new QTreeWidgetItem();
    }

    item->setText(0, node.text);
    item->setData(0, ct::NodeTypeRole, static_cast<int>(type));
    item->setIcon(0, m_tree->iconForType(type));
    item->setExpanded(node.expanded);
    item->setCheckState(0, node.is_visible ? Qt::Checked : Qt::Unchecked);

    if (type == ct::NodeFile) {
        item->setData(0, ct::NodeFilePathRole, node.filepath);
    }

    // 递归子节点
    for (const auto& child : node.children) {
        rebuildTreeNode(item, child);
    }

    return item;
}
