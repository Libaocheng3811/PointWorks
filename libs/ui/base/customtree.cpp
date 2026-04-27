#include "customtree.h"

#include "viz/cloudview.h"
#include "viz/console.h"

#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QMimeData>

namespace ct
{

void CustomTree::printI(const QString& message) { m_console->print(LOG_INFO, message); }
void CustomTree::printW(const QString& message) { m_console->print(LOG_WARNING, message); }
void CustomTree::printE(const QString& message) { m_console->print(LOG_ERROR, message); }

CustomTree::CustomTree(QWidget *parent)
        : QTreeWidget(parent),
        m_cloudview(nullptr),
        m_console(nullptr),
        m_table(nullptr)
    {
        /** QAbstractItemView 是 Qt 模型/视图框架中的一个抽象基类，提供了显示和编辑模型（Model）中数据的通用接口。
         *  setSelectionMode 函数接受一个 QAbstractItemView::SelectionMode 枚举值，定义了选择行为。
         *  QAbstractItemView::SingleSelection表示单选模式，QAbstractItemView::MultiSelection表示多选模式
         *  QAbstractItemView::ExtendedSelection表示扩展模式
         */
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);

        // 信号itemSelectionChanged和itemClicked继承自QTreeWidget
        connect(this, &CustomTree::itemSelectionChanged, this, &CustomTree::itemSelectionChangedEvent);
        connect(this, &CustomTree::itemClicked, this, &CustomTree::itemClickedEvent);
        this->setAcceptDrops(true);
    }

    // ================================================================
    // 图标选择
    // ================================================================

    const QIcon& CustomTree::iconForType(SceneNodeType type) const
    {
        switch (type) {
        case NodeFile:  return m_file_icon.isNull() ? m_parent_icon : m_file_icon;
        case NodeCloud: return m_cloud_icon.isNull() ? m_child_icon  : m_cloud_icon;
        case NodeGroup: return m_group_icon;
        case NodeShape: return m_shape_icon;
        case NodeMesh:  return m_mesh_icon.isNull() ? m_child_icon  : m_mesh_icon;
        case NodeBoundary: return m_shape_icon;
        default:        return m_cloud_icon.isNull() ? m_child_icon : m_cloud_icon;
        }
    }

    // ================================================================
    // addItem — 向后兼容接口
    // ================================================================

    QTreeWidgetItem* CustomTree::addItem(QTreeWidgetItem *parent, const QString &text, bool selected)
    {
        return addItem(parent, text, NodeCloud, selected);
    }

    // ================================================================
    // addItem — 带节点类型的完整接口
    // ================================================================

    QTreeWidgetItem* CustomTree::addItem(QTreeWidgetItem *parent, const QString &text,
                                          SceneNodeType type, bool selected)
    {
        QTreeWidgetItem* new_item;
        if (parent == nullptr){
            new_item = new QTreeWidgetItem(this);
        }
        else{
            new_item = new QTreeWidgetItem(parent);
        }

        new_item->setText(0, text);
        new_item->setIcon(0, iconForType(type));
        new_item->setCheckState(0, Qt::Checked);
        new_item->setData(0, NodeTypeRole, static_cast<int>(type));

        // 展开父节点
        if (parent) parent->setExpanded(true);
        else this->expandItem(new_item);

        if (selected){
            this->clearSelection();
            new_item->setSelected(true);
        }
        return new_item;
    }

    // ================================================================
    // 节点类型查询
    // ================================================================

    SceneNodeType CustomTree::getNodeType(QTreeWidgetItem* item)
    {
        if (!item) return NodeCloud;
        bool ok = false;
        int val = item->data(0, NodeTypeRole).toInt(&ok);
        if (!ok || val < 0) return NodeCloud;
        if (val < 0) return NodeCloud; // 未设置类型的旧节点默认为云节点
        return static_cast<SceneNodeType>(val);
    }

    bool CustomTree::isFolderNode(QTreeWidgetItem* item)
    {
        SceneNodeType type = getNodeType(item);
        return type == NodeFile || type == NodeGroup;
    }

    bool CustomTree::isCloudNode(QTreeWidgetItem* item)
    {
        SceneNodeType type = getNodeType(item);
        return type == NodeCloud || type == NodeMesh;
    }

    QTreeWidgetItem* CustomTree::findFileParent(QTreeWidgetItem* item)
    {
        if (!item) return nullptr;
        QTreeWidgetItem* current = item->parent();
        while (current) {
            if (getNodeType(current) == NodeFile)
                return current;
            current = current->parent();
        }
        return nullptr;
    }

    // ================================================================
    // 原有方法
    // ================================================================

    QList<QTreeWidgetItem*> CustomTree::getSelectedItems() {
        return this->selectedItems();
    }

    void CustomTree::getAllChildItems(QTreeWidgetItem* parent, QList<QTreeWidgetItem*>& lists){
        if (!parent) return;
        for (int i = 0; i < parent->childCount(); ++i){
            QTreeWidgetItem* child = parent->child(i);
            lists.append(child);
            getAllChildItems(child, lists); // 递归获取所有子节点
        }
    }

    void CustomTree::setItemAndChildrenCheckState(QTreeWidgetItem *item, Qt::CheckState state) {
        if (!item) return;
        const bool wasBlocked = this->blockSignals(true);

        item->setCheckState(0, state);
        for (int i = 0; i < item->childCount(); ++i){
            setItemAndChildrenCheckState(item->child(i), state);
        }
        this->blockSignals(wasBlocked);
    }

    void CustomTree::removeItem(QTreeWidgetItem *item) {
        if (item == nullptr) return;

        //如果有父节点，从父节点移除，否则从TreeWidget移除
        if (item->parent()){
            item->parent()->removeChild(item);
        }
        else {
            int idx = indexOfTopLevelItem(item);
            this->takeTopLevelItem(idx);
        }
        delete item;
    }

    void CustomTree::itemSelectionChangedEvent() {}

    void CustomTree::itemClickedEvent(QTreeWidgetItem* item, int) {}

    // ================================================================
    // 拖拽支持
    // ================================================================

    void CustomTree::dragEnterEvent(QDragEnterEvent* event)
    {
        if (event->mimeData()->hasUrls()) return;
        event->acceptProposedAction();
    }

    void CustomTree::dragMoveEvent(QDragMoveEvent* event)
    {
        if (event->mimeData()->hasUrls()) return;
        event->acceptProposedAction();
    }

    void CustomTree::dropEvent(QDropEvent* event)
    {
        if (event->mimeData()->hasUrls()) return;
        QTreeWidgetItem* target = itemAt(event->pos());
        QList<QTreeWidgetItem*> dragged = selectedItems();

        if (dragged.isEmpty()) return;

        // 不能拖拽自身或拖拽父节点到子节点
        for (QTreeWidgetItem* src : dragged) {
            if (src == target) return;
            // 检查 target 是否是 src 的子孙
            QTreeWidgetItem* p = target;
            while (p) {
                if (p == src) return;
                p = p->parent();
            }
        }

        SceneNodeType targetType = target ? getNodeType(target) : NodeCloud;

        for (QTreeWidgetItem* src : dragged) {
            SceneNodeType srcType = getNodeType(src);

            // 规则：FileNode 不能拖入 FileNode 或 GroupNode
            if (srcType == NodeFile && target && isFolderNode(target)) continue;

            // 规则：FileNode 不能拖入 CloudNode
            if (srcType == NodeFile && isCloudNode(target)) continue;

            if (target) {
                // 拖到文件夹/分组节点下
                if (isFolderNode(target)) {
                    if (src->parent())
                        src->parent()->removeChild(src);
                    else
                        takeTopLevelItem(indexOfTopLevelItem(src));
                    target->addChild(src);
                    target->setExpanded(true);
                }
                // 拖到点云节点旁（同级排序）
                else if (isCloudNode(target) && target->parent()) {
                    QTreeWidgetItem* parent = target->parent();
                    if (src->parent())
                        src->parent()->removeChild(src);
                    else
                        takeTopLevelItem(indexOfTopLevelItem(src));
                    int idx = parent->indexOfChild(target);
                    parent->insertChild(idx + 1, src);
                }
            } else {
                // 拖到空白区域：FileNode 不能成为独立根节点
                if (srcType == NodeFile) continue;
                // 从原位置取出
                if (src->parent())
                    src->parent()->removeChild(src);
                else
                    takeTopLevelItem(indexOfTopLevelItem(src));
                addTopLevelItem(src);
            }
        }

        event->acceptProposedAction();
    }
}
