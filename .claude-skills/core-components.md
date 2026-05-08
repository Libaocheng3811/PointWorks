# 核心组件详解

PointWorks 的核心数据结构、文件 I/O 与渲染系统。分布在 `libs/core/`、`libs/io/`、`libs/viz/` 目录。

## Cloud (libs/core/cloud.h)

主要点云数据结构，采用 **AOS (Array of Structures)** 格式以节省内存。

**关键特性**:
- 基于八叉树的空间索引，`CloudBlock` 叶子节点最多存储 6 万个点
- 支持多种点类型：XYZ、XYZRGB、XYZNormal、XYZRGBNormal
- 标量字段管理 (`m_scalar_cache`)，支持自定义属性
- PCL 互操作性：`toPCL_XYZRGB()`、`fromPCL_XYZRGB()` 等方法
- 提供 `forEachPoint()`、`getFirstPoint()` 等 API 避免外部直接访问 CloudBlock 内部

**核心数据成员**:
```cpp
class Cloud {
    OctreeNode::Ptr m_octree_root;          // 八叉树根节点
    std::vector<CloudBlock::Ptr> m_all_blocks;  // 扁平化 Block 列表
    CloudConfig m_config;                   // 配置参数
    QMap<QString, std::vector<float>> m_scalar_cache;  // 标量场缓存
    Eigen::Vector3d m_global_shift;         // 全局坐标偏移
};
```

**自适应配置** (`calculateAdaptiveConfig`):
- 点数 < 1000 万：直通模式（禁用八叉树）
- 点数 >= 1000 万：八叉树模式，自动计算 Block 大小和 LOD 参数

## CloudBlock (libs/core/cloud.h)

八叉树叶子节点负载，存储实际点云数据。

**数据结构**:
```cpp
class CloudBlock {
    std::vector<pcl::PointXYZ> m_points;     // 3D 坐标（必需）
    std::unique_ptr<std::vector<RGB>> m_colors;  // RGB 颜色（可选）
    std::unique_ptr<std::vector<CompressedNormal>> m_normals;  // 压缩法线
    QMap<QString, std::vector<float>> m_scalar_fields;  // 标量场

    Box m_box;                               // 包围盒
    std::shared_ptr<void> m_vtk_polydata;    // VTK 缓存（类型擦除，设计权衡）
    bool m_is_dirty = true;                  // 脏标记
};
```

> `m_vtk_polydata` 使用 `shared_ptr<void>` 类型擦除避免核心层对 VTK 的编译依赖。
> 这使得渲染缓存与 block 共存可实现 O(1) 查找，是可接受的务实权衡。

## Octree (libs/core/octree.h)

空间分割与 LOD 生成。

**OctreeNode**:
```cpp
class OctreeNode {
    OctreeNode* m_parent;                    // 父节点
    OctreeNode* m_children[8];               // 8 个子节点
    CloudBlock::Ptr m_block;                 // 叶子节点数据
    std::vector<pcl::PointXYZRGB> m_lod_points;  // LOD 数据
    Box m_box;                               // 包围盒
    int m_depth;                             // 深度
    size_t m_total_points_in_node;           // 点数统计
};
```

**LOD 生成策略** (蓄水池采样 Reservoir Sampling):
```cpp
// 流式插入时实时更新 LOD
if (node->m_lod_points.size() < capacity) {
    node->m_lod_points.push_back(point);
} else {
    // 随机替换概率 = k/n
    if (rand() % current_n < capacity) {
        node->m_lod_points[rand() % capacity] = point;
    }
}
```

## OctreeRenderer (libs/viz/octreerenderer.h)

高性能渲染器，采用 SSE 遍历策略。

**渲染特性**:
- **SSE (Screen Space Error)** 遍历：根据屏幕投影大小决定渲染精细度
- **视锥剔除** (Frustum Culling)：只渲染可见节点
- **动态阈值**：交互时降低质量，静止时提高质量
- **Actor 对象池**：复用 VTK Actor（最多 500 个），减少 GPU 资源创建开销
- **增量脏标记清理**：`invalidateDirtyActors()` 仅重建脏节点，保留干净节点

## FileIO (libs/io/)

流式文件加载与保存。按数据类型拆分为多个编译单元，FileIO 类作为调度器。

**文件结构**:
```
libs/io/
├── fileio.h              # FileIO 类声明（信号、槽、公共接口）
├── fileio.cpp            # 调度器 + 辅助函数 + saveMeshFile + parseOBJMaterialTexture
├── fileio_pointcloud.cpp # 点云格式加载/保存（LAS, PLY, PCD, TXT, E57）
├── fileio_mesh.cpp       # 模型格式加载（OBJ, STL, VTK, IFS）
├── textured_mesh.h       # (已移至 libs/core/)
└── projectfile.h/cpp     # 项目文件保存/加载
```

> `textured_mesh.h` 已从 `libs/io/` 移至 `libs/core/`，因为 TexturedMesh 是核心数据类型。
> 注意：`textured_mesh.h` 仍依赖 `pcl/PolygonMesh.h`，这是 `pw_core` 中唯一的 PCL
> 类型依赖。短期内保持现状，长期可通过 `shared_ptr<void>` 类型擦除移除。

**支持格式**:
- 点云: LAS/LAZ（LASlib）、PLY、PCD、TXT/XYZ/ASC、E57（E57Format）
- 模型: OBJ（带纹理检测）、STL、VTK、IFS

**流式加载流程**:
```cpp
bool FileIO::loadLAS(const QString& filename, Cloud::Ptr& cloud) {
    // 1. 读取 Header 获取包围盒
    // 2. 初始化八叉树
    // 3. 流式读取（每批 50 万点）
    // 4. 生成 LOD
    cloud->generateLOD();
}
```

## CloudView (libs/viz/cloudview.h)

基于 VTK 的三维可视化控件。通过 `view_params.h`（pw_core）获取相机参数，不再直接依赖 `projectfile.h`。

**功能**:
- 交互式点拾取（单点和多边形区域选择）
- 相机控制和视口管理
- 支持添加形状、箭头、标签、对应关系
- 多点云渲染管理
- 增量脏标记渲染（`invalidateCloudRenderDirty`）

## CloudType (libs/core/cloudtype.h)

点类型定义与数据结构。依赖 PCL 点类型和 Eigen（项目基石类型，短期内无法移除）。

```cpp
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointXYZRGBNormal PointXYZRGBN;
typedef pcl::Normal PointNormal;

struct ColorRGB { uint8_t r, g, b; };
struct CompressedNormal { uint16_t data; }; // 球面坐标编码，2 bytes
```

## Colormap (libs/core/colormap.h)

色图定义。使用 `std::string` / `std::vector<std::string>`，**无 Qt 依赖**。UI 层使用 `QString::fromStdString()` 转换。

## FieldTypes (libs/core/field_types.h)

标量场、距离计算和配准参数的类型定义。定义了距离方法枚举、配准参数结构体等共享类型。

```cpp
// 距离方法枚举
enum class DistanceMethod { C2C, C2M, C2P, CPS };

// 配准参数
struct RegistrationParams { ... };
```

## Statistics (libs/core/statistics.h)

统计计算工具，提供点云基本统计信息计算（点数、包围盒、分辨率等）。

## TexturedMesh (libs/core/textured_mesh.h)

纹理网格数据结构，从 `libs/io/` 移入（核心数据类型）。

```cpp
struct TexturedMesh {
    pcl::PolygonMesh mesh;        // 基础网格
    std::string texture_path;     // 纹理图片路径
    std::string material_name;    // 材质名称
    // OBJ MTL 材质参数
};
```

> `textured_mesh.h` 仍依赖 `pcl/PolygonMesh.h`，这是 `pw_core` 中唯一的 PCL 类型依赖。长期可通过 `shared_ptr<void>` 类型擦除移除。

## ViewParams (libs/core/view_params.h)

相机参数和视图选项，从 `projectfile.h` 移出，使 `pw_viz` 不再依赖 `pw_io`。

## SurfaceVizHelper (libs/viz/surface_viz_helper.h/cpp)

曲面重建的 VTK 渲染预处理，从 `libs/algorithm/surface` 移入 `libs/viz/`，实现算法层与可视化层彻底解耦。

```cpp
class SurfaceVizHelper {
public:
    static vtkSmartPointer<vtkPolyData> prepareForRendering(
        const pcl::PolygonMesh& mesh);
};
```

## ViewCube (libs/viz/viewcube.h)

三维视图方向指示器（ViewCube），提供直观的视角导航。

**功能**:
- 可拖拽的立方体方向指示器，显示当前相机朝向
- 点击面/边/角快速切换到标准视角（Top/Front/Right/Back/Bottom/Left）
- 支持鼠标拖拽旋转相机
- 坐标轴颜色标注（X=红, Y=绿, Z=蓝）

## ScalarBarWidget (libs/viz/scalar_bar_widget.h)

标量场色度条控件，显示标量场的颜色映射范围和图例。

**功能**:
- 显示标量场的最小值/最大值范围
- 渲染当前使用的色图（Jet、Rainbow 等）
- 支持标量场切换时动态更新
- 与 CloudView 的标量场显示联动

## 大点云处理策略

### 1. 八叉树分区
- 空间递归划分为 8 个子空间
- 叶子节点最多存储 6 万个点（可配置）

### 2. 流式 I/O
- 批量加载（每批 50 万点）
- 实时进度反馈

### 3. 全局坐标偏移
- 渲染前减去质心，保存时加回，解决 GPU 精度问题

### 4. 动态 LOD
- 交互态：降低阈值，渲染粗糙 LOD
- 静止态：提高阈值，渲染精细 Block
- SSE 策略：根据屏幕投影大小动态选择

### 5. 视锥剔除
- 6 个裁剪面检测 8 个顶点
