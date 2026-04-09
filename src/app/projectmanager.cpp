#include "projectmanager.h"

#include "base/cloudtree.h"
#include "base/scenenodetype.h"
#include "viz/cloudview.h"
#include "core/cloud.h"

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

    // 第一遍：重建树骨架（只创建文件夹/分组节点，不加载点云）
    // 收集所有需要加载的 FileNode 及其路径
    struct PendingFile { QString filepath; QTreeWidgetItem* parentNode; };
    QList<PendingFile> pendingFiles;

    for (const auto& root : data.tree_roots) {
        QTreeWidgetItem* item = rebuildTreeNode(nullptr, root, tree);
        if (item) tree->addTopLevelItem(item);

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

    tree->setScriptMode(true);
    m_total_loads = 0;
    for (const auto& pf : pendingFiles) {
        if (QFileInfo::exists(pf.filepath))
            ++m_total_loads;
    }
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

    // 第二遍：加载所有点云文件到对应的 FileNode 下
    for (const auto& pf : pendingFiles) {
        if (!QFileInfo::exists(pf.filepath)) continue;
        emit loadProgress("Loading " + QFileInfo(pf.filepath).fileName() + "...", 0);
        tree->loadCloudFile(pf.filepath, pf.parentNode);
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
        ct::TreeNode node = treeNodeFromItem(tree->topLevelItem(i), tree);
        roots.append(node);
    }
}

ct::TreeNode ProjectManager::treeNodeFromItem(QTreeWidgetItem* item, ct::CloudTree* tree)
{
    ct::TreeNode node;
    ct::SceneNodeType type = ct::CustomTree::getNodeType(item);

    node.type = (type == ct::NodeFile)  ? "file" :
                (type == ct::NodeGroup) ? "group" : "cloud";
    node.text = item->text(0);
    node.expanded = item->isExpanded();
    node.is_visible = (item->checkState(0) != Qt::Unchecked);

    if (type == ct::NodeCloud) {
        ct::Cloud::Ptr cloud = tree->getCloud(item);
        if (cloud) node.uuid = QString::fromStdString(cloud->id());
    }
    if (type == ct::NodeFile) {
        node.filepath = item->data(0, ct::NodeFilePathRole).toString();
    }

    for (int i = 0; i < item->childCount(); ++i) {
        node.children.append(treeNodeFromItem(item->child(i), tree));
    }

    return node;
}

// ================================================================
// 从 TreeNode 重建 QTreeWidgetItem 树骨架
// ================================================================

QTreeWidgetItem* ProjectManager::rebuildTreeNode(QTreeWidgetItem* parent, const ct::TreeNode& node, ct::CloudTree* tree)
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
    item->setIcon(0, tree->iconForType(type));
    item->setExpanded(node.expanded);
    item->setCheckState(0, node.is_visible ? Qt::Checked : Qt::Unchecked);

    if (type == ct::NodeFile) {
        item->setData(0, ct::NodeFilePathRole, node.filepath);
    }

    // 递归子节点
    for (const auto& child : node.children) {
        rebuildTreeNode(item, child, tree);
    }

    return item;
}
