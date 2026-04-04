#include "projectmanager.h"

#include "base/cloudtree.h"
#include "viz/cloudview.h"

#include <QFileInfo>
#include <QDir>
#include <QMessageBox>
#include <QTreeWidget>

ProjectManager::ProjectManager(QObject* parent)
    : QObject(parent)
{
}

// ================================================================
// 保存项目
// ================================================================

bool ProjectManager::saveProject(const QString& path, ct::CloudTree* tree, ct::CloudView* view)
{
    if (!tree || !view) return false;

    ct::ProjectData data;
    data.created_at = QDateTime::currentDateTime();
    data.modified_at = data.created_at;

    QString projectDir = QFileInfo(path).absolutePath();
    collectCloudEntries(tree, view, projectDir, data);
    collectTreeNodes(tree, data.tree_roots);

    data.camera = view->getCameraParams();
    data.view_options = view->getViewOptions();

    if (!ct::ProjectFile::save(path, data)) {
        emit loadError("Failed to save project file: " + path);
        return false;
    }

    m_current_path = path;
    m_modified = false;
    emit modificationChanged(false);
    emit projectSaved(path);
    return true;
}

// ================================================================
// 打开项目
// ================================================================

bool ProjectManager::openProject(const QString& path, ct::CloudTree* tree, ct::CloudView* view)
{
    if (!tree || !view) return false;

    ct::ProjectData data;
    if (!ct::ProjectFile::load(path, data)) {
        emit loadError("Failed to load project file: " + path);
        return false;
    }

    QString projectDir = QFileInfo(path).absolutePath();

    tree->removeAllClouds();

    QStringList missing;
    QStringList validPaths;
    for (const auto& entry : data.clouds) {
        QString resolved = ct::ProjectFile::resolveFilePath(projectDir, entry.file_path);
        if (!QFileInfo::exists(resolved)) {
            missing << entry.display_name + " (" + entry.file_path + ")";
        } else {
            validPaths << resolved;
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

    tree->setScriptMode(true);
    m_total_loads = validPaths.size();
    m_pending_loads = 0;

    // 信号驱动：每插入一个点云计数，全部完成时恢复视图
    auto* conn = new QMetaObject::Connection;
    *conn = connect(tree, &ct::CloudTree::cloudInserted, this,
        [this, tree, view, data, conn](ct::Cloud::Ptr) {
            ++m_pending_loads;
            if (m_pending_loads >= m_total_loads) {
                disconnect(*conn);
                delete conn;
                tree->setScriptMode(false);
                view->setViewOptions(data.view_options);
                view->setCameraParams(data.camera);
                view->refresh();
                emit loadProgress("Project loaded successfully.", 100);
            }
        });

    for (const auto& resolvedPath : validPaths) {
        emit loadProgress("Loading " + QFileInfo(resolvedPath).fileName() + "...", 0);
        tree->loadCloudFile(resolvedPath);
    }

    m_current_path = path;
    m_modified = false;
    emit modificationChanged(false);
    emit projectOpened(path);
    return true;
}

// ================================================================
// 辅助：收集点云条目
// ================================================================

void ProjectManager::collectCloudEntries(ct::CloudTree* tree, ct::CloudView* /*view*/,
                                         const QString& projectDir, ct::ProjectData& data)
{
    auto allClouds = tree->getAllClouds();
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

        auto* item = tree->getItemById(entry.uuid);
        entry.is_visible = item ? (item->checkState(0) != Qt::Unchecked) : true;

        data.clouds.append(entry);
    }
}

// ================================================================
// 辅助：收集树结构
// ================================================================

void ProjectManager::collectTreeNodes(ct::CloudTree* tree, QList<ct::TreeNode>& roots)
{
    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        ct::TreeNode node;
        node.type = "folder";
        node.text = tree->topLevelItem(i)->text(0);
        node.expanded = tree->topLevelItem(i)->isExpanded();
        collectTreeChildren(tree->topLevelItem(i), node.children);
        roots.append(node);
    }
}

void ProjectManager::collectTreeChildren(QTreeWidgetItem* parent, QList<ct::TreeNode>& children)
{
    for (int i = 0; i < parent->childCount(); ++i) {
        QTreeWidgetItem* child = parent->child(i);
        ct::TreeNode node;
        if (child->childCount() > 0) {
            node.type = "folder";
            node.text = child->text(0);
            node.expanded = child->isExpanded();
            collectTreeChildren(child, node.children);
        } else {
            node.type = "cloud";
            node.text = child->text(0);
        }
        children.append(node);
    }
}
