#ifndef POINTWORKS_SCENENODETYPE_H
#define POINTWORKS_SCENENODETYPE_H

#include <QtGlobal>

#include <QtCore/qnamespace.h>

namespace ct
{

/// 场景节点类型
enum SceneNodeType
{
    NodeFile  = 0,  // 原始文件节点（加载文件时创建的根节点）
    NodeCloud = 1,  // 点云数据节点
    NodeGroup = 2,  // 逻辑分组节点（算法结果容器，如 CSF、聚类结果）
    NodeShape = 3,  // 附属物节点（包围盒、拟合平面等）
    NodeMesh  = 4,  // 网格模型节点（OBJ/STL 等带面片的模型）
    NodeBoundary = 5 // 附属形状节点（边界多段线等，作为父节点的子节点）
};

/// 选择统计信息（驱动 UI Action 的 enable/disable）
struct SelectionInfo
{
    int totalSelected = 0;

    int cloudCount = 0;
    int meshCount  = 0;
    int shapeCount = 0;
    int groupCount = 0;
    int fileCount = 0;
    int boundaryCount = 0;

    bool hasCloud()       const { return cloudCount > 0; }
    bool hasMesh()        const { return meshCount > 0; }
    bool hasShape()       const { return shapeCount > 0; }
    bool hasOnlyCloud()   const { return cloudCount > 0 && meshCount == 0 && shapeCount == 0; }
    bool hasOnlyMesh()    const { return meshCount > 0 && cloudCount == 0; }
    bool hasCloudOrMesh() const { return cloudCount > 0 || meshCount > 0; }
    bool hasAnySelection() const { return totalSelected > 0; }
    bool hasAny() const { return totalSelected > 0; }
    bool isSingleSelection() const { return totalSelected == 1; }
    bool isSingleCloud()  const { return totalSelected == 1 && cloudCount == 1; }
};

/// QTreeWidgetItem 数据角色 — 存储节点类型
constexpr int NodeTypeRole      = Qt::UserRole + 1;
/// QTreeWidgetItem 数据角色 — 存储关联的 Cloud UUID（仅 NodeCloud）
constexpr int NodeUuidRole      = Qt::UserRole + 2;
/// QTreeWidgetItem 数据角色 — 存储原始文件路径（仅 NodeFile）
constexpr int NodeFilePathRole  = Qt::UserRole + 3;
/// QTreeWidgetItem 数据角色 — 存储关联的 PolygonMesh visual ID
constexpr int NodeMeshIdRole    = Qt::UserRole + 4;
/// QTreeWidgetItem 数据角色 — 存储关联的 VTK shape ID（边界多段线等）
constexpr int NodeShapeIdRole   = Qt::UserRole + 5;
/// QTreeWidgetItem 数据角色 — 存储目标视窗索引（-1 = 所有视窗）
constexpr int NodeViewportIndexRole = Qt::UserRole + 6;
/// 特殊值：点云绑定到所有视窗
constexpr int VIEWPORT_ALL = -1;

} // namespace ct

#endif // POINTWORKS_SCENENODETYPE_H
