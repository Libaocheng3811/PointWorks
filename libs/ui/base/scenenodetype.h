#ifndef POINTWORKS_SCENENODETYPE_H
#define POINTWORKS_SCENENODETYPE_H

#include <QtGlobal>

namespace ct
{

/// 场景节点类型
enum SceneNodeType
{
    NodeFile  = 0,  // 原始文件节点（加载文件时创建的根节点）
    NodeCloud = 1,  // 点云数据节点
    NodeGroup = 2,  // 逻辑分组节点（算法结果容器，如 CSF、聚类结果）
    NodeShape = 3   // 附属物节点（包围盒、拟合平面等）
};

/// QTreeWidgetItem 数据角色 — 存储节点类型
constexpr int NodeTypeRole      = Qt::UserRole + 1;
/// QTreeWidgetItem 数据角色 — 存储关联的 Cloud UUID（仅 NodeCloud）
constexpr int NodeUuidRole      = Qt::UserRole + 2;
/// QTreeWidgetItem 数据角色 — 存储原始文件路径（仅 NodeFile）
constexpr int NodeFilePathRole  = Qt::UserRole + 3;
/// QTreeWidgetItem 数据角色 — 存储关联的 PolygonMesh visual ID
constexpr int NodeMeshIdRole    = Qt::UserRole + 4;

} // namespace ct

#endif // POINTWORKS_SCENENODETYPE_H
