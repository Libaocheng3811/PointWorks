#ifndef POINTWORKS_CUSTORMTREE_H
#define POINTWORKS_CUSTORMTREE_H

#include "core/exports.h"
#include "base/scenenodetype.h"

#include <QTreeWidget>
#include <QTableWidget>

namespace ct
{
    class CloudView;
    class Console;

    class CustomTree : public QTreeWidget
    {
        Q_OBJECT
    public:
        explicit CustomTree(QWidget* parent = nullptr);

        /**
         * @brief 设置点云视图
         */
        void setCloudView(CloudView* cloudview) {m_cloudview = cloudview;}

        /**
         * @brief 设置属性显示窗口
         */
        void setPropertiesTable(QTableWidget* table) {m_table = table;}

        /**
         * @brief 设置输出窗口
         */
        void setConsole(Console* console) {m_console = console;}

        /**
         * @brief 设置文件节点的图标 (NodeFile)
         */
        void setFileIcon(const QIcon& icon) {m_file_icon = icon;}

        /**
         * @brief 设置点云节点的图标 (NodeCloud)
         */
        void setCloudIcon(const QIcon& icon) {m_cloud_icon = icon;}

        /**
         * @brief 设置分组节点的图标 (NodeGroup)
         */
        void setGroupIcon(const QIcon& icon) {m_group_icon = icon;}

        /**
         * @brief 设置附属物节点的图标 (NodeShape)
         */
        void setShapeIcon(const QIcon& icon) {m_shape_icon = icon;}

        /**
         * @brief 设置网格模型节点的图标 (NodeMesh)
         */
        void setMeshIcon(const QIcon& icon) {m_mesh_icon = icon;}

        /**
         * @brief 向后兼容：设置父类项目的图标（等同于 setFileIcon）
         */
        void setParentIcon(const QIcon& icon) {m_file_icon = icon;}

        /**
         * @brief 向后兼容：设置子类项目的图标（等同于 setCloudIcon）
         */
        void setChildIcon(const QIcon& icon) {m_cloud_icon = icon;}

        /**
         * @brief 添加节点（向后兼容接口，默认类型为 NodeCloud）
         */
        QTreeWidgetItem* addItem(QTreeWidgetItem* parent, const QString& text, bool selected = false);

        /**
         * @brief 添加指定类型的节点
         * @param parent 父节点，为空则作为根节点
         * @param text 显示文本
         * @param type 节点类型
         * @param selected 是否选中
         */
        QTreeWidgetItem* addItem(QTreeWidgetItem* parent, const QString& text,
                                 SceneNodeType type, bool selected = false);

        /**
         * @brief 获取节点的场景类型
         */
        static SceneNodeType getNodeType(QTreeWidgetItem* item);

        /**
         * @brief 判断是否为文件夹类型节点（FileNode 或 GroupNode）
         */
        static bool isFolderNode(QTreeWidgetItem* item);

        /**
         * @brief 判断是否为点云类型节点
         */
        static bool isCloudNode(QTreeWidgetItem* item);

        /**
         * @brief 向上查找最近的文件节点
         */
        static QTreeWidgetItem* findFileParent(QTreeWidgetItem* item);

    protected:
        /**
         * @brief 打印日志
         */
        void printI(const QString& message);
        void printW(const QString& message);
        void printE(const QString& message);

        /**
         * @brief 获取选中的Item
         */
         QList<QTreeWidgetItem*> getSelectedItems();

         /**
          * @brief 递归获取某个父级item下的所有子孙Item
          */
         void getAllChildItems(QTreeWidgetItem* parent, QList<QTreeWidgetItem*>& lists);

         /**
         * @brief 递归设置勾选状态
         */
         void setItemAndChildrenCheckState(QTreeWidgetItem* item, Qt::CheckState state);

         /**
          * @brief 移除节点及其数据
          */
         void removeItem(QTreeWidgetItem* item);

         /**
          * @brief 拖拽事件
          */
         void dragEnterEvent(QDragEnterEvent* event) override;
         void dragMoveEvent(QDragMoveEvent* event) override;
         void dropEvent(QDropEvent* event) override;

    private slots:

        /**
        * @brief 项目选中改变事件（高亮选中）
         * 触发时机：当用户点击了树节点的 文字部分（背景变蓝），或者按住了 Ctrl/Shift 多选，改变了当前高亮的节点集合时
         * 业务逻辑：控制 "焦点/属性/包围盒"
         * 选中：
            在 3D 视图中给选中的点云加上 白色线框包围盒 (addBox)，提示用户"你现在操作的是这个"。
            在右侧 属性面板 (Properties) 中刷新数据显示（点数、分辨率、ID等）。
         * 取消选中：移除包围盒，清空属性面板。
         * 选中（变蓝）并不等于勾选（打钩）。你可以选中一个未勾选（不显示）的点云来查看它的属性（比如文件路径、点数），
         * 但这通常不显示包围盒（因为点云本身都没显示）
        */
        virtual void itemSelectionChangedEvent();

        /**
        * @brief 项目点击事件
         * 触发时机：当用户点击了树节点前面的 复选框 (CheckBox)，或者你在代码里调用了 setItemCheckState / setText 等改变节点内容的操作时
         * 业务逻辑：控制 "显示/隐藏"
         * 勾选：在 3D 视图中把点云画出来 (addPointCloud)。
         * 取消勾选：从 3D 视图中移除点云 (removePointCloud)。
         * 父子联动：如果勾选了文件夹，文件夹下的所有文件都应该被勾选并显
        */
        virtual void itemClickedEvent(QTreeWidgetItem* itme, int column);

    public:
        CloudView* m_cloudview;
        Console* m_console;
        QTableWidget* m_table;

    protected:
        // 四种节点类型的图标
        QIcon m_file_icon;     // NodeFile
        QIcon m_cloud_icon;    // NodeCloud
        QIcon m_group_icon;    // NodeGroup
        QIcon m_shape_icon;    // NodeShape
        QIcon m_mesh_icon;     // NodeMesh

        // 向后兼容（仅在不设置新图标时作为 fallback）
        QIcon m_parent_icon;
        QIcon m_child_icon;

    public:
        /**
         * @brief 根据节点类型获取对应的图标
         */
        const QIcon& iconForType(SceneNodeType type) const;

    };
}


#endif //POINTWORKS_CUSTORMTREE_H
