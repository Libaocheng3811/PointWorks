# 核心组件详解

PointWorks 的核心数据结构、文件 I/O 与渲染系统。分布在 `libs/core/`、`libs/io/`、`libs/viz/` 目录。

## Cloud (libs/core/cloud.h)

主要点云数据结构，采用 **AOS (Array of Structures)** 格式以节省内存。

**关键特性**:
- 基于八叉树的空间索引，`CloudBlock` 叶子节点最多存储 6 万个点
- 支持多种点类型：XYZ、XYZRGB、XYZNormal、XYZRGBNormal
- 标量字段管理 (`m_scalar_cache`)，支持自定义属性
- PCL 互操作性：`toPCL_XYZRGB()`、`fromPCL_XYZRGB()` 等方法

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
    std::shared_ptr<void> m_vtk_polydata;    // VTK 缓存
    bool m_is_dirty = true;                  // 脏标记
};
```

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

**渲染流程**:
```cpp
void OctreeRenderer::update() {
    // 1. 检查相机是否移动
    if (!camChanged && !m_force_update) return;

    // 2. 优先队列遍历（SSE 策略）
    std::priority_queue<PriorityNode> pq;
    pq.push({root, projectSize(root->m_box)});

    while (!pq.empty()) {
        auto node = pq.top().node;
        float screenSize = projectSize(node->m_box);

        // 判断是否需要细分
        if (screenSize > m_base_threshold && node->hasChildren()) {
            // 继续细分子节点
        } else {
            // 渲染此节点（LOD 或 Block）
            vtkActor* actor = getOrCreateActor(node, isLOD);
        }
    }
}
```

## FileIO (libs/io/)

流式文件加载与保存。按数据类型拆分为多个编译单元，FileIO 类作为调度器。

**文件结构**:
```
libs/io/
├── fileio.h              # FileIO 类声明（信号、槽、公共接口）
├── fileio.cpp            # 调度器 + 辅助函数 + saveMeshFile + parseOBJMaterialTexture (~280行)
├── fileio_pointcloud.cpp # 点云格式加载/保存（LAS, PLY, PCD, TXT, E57）(~1580行)
├── fileio_mesh.cpp       # 模型格式加载（OBJ, STL, VTK, IFS）(~170行)
├── textured_mesh.h       # 纹理网格数据结构
└── projectfile.h/cpp     # 项目文件保存/加载
```

**支持格式**:
- 点云: LAS/LAZ（LASlib）、PLY、PCD、TXT/XYZ/ASC、E57（E57Format）
- 模型: OBJ（带纹理检测）、STL、VTK、IFS

**方法拆分策略**:
方法保持为 `FileIO` 成员函数（声明在 fileio.h），实现分布在各 .cpp 中。调度器只负责格式分发和公共逻辑。

**流式加载流程**:
```cpp
bool FileIO::loadLAS(const QString& filename, Cloud::Ptr& cloud) {
    // 1. 读取 Header 获取包围盒
    LASreader* lasreader = lasreadopener.open(filename);

    // 2. 初始化八叉树
    cloud->initOctree(globalBox);

    // 3. 流式读取（每批 50 万点）
    CloudBatch batch;
    batch.reserve(BATCH_SIZE);

    while (lasreader->read_point()) {
        batch.points.push_back(pt);
        if (batch.points.size() >= BATCH_SIZE) {
            batch.flushTo(cloud);
            emit progress(percent);
        }
    }

    // 4. 生成 LOD
    cloud->generateLOD();
}
```

**全局坐标偏移**:
```cpp
// 问题：UTM 坐标 (x=500000) 导致 GPU 精度丢失
// 解决：渲染前减去质心，保存时加回
Eigen::Vector3d shift = calculateCentroid(cloud);
cloud->setGlobalShift(shift);
```

## ProjectFile (libs/io/projectfile.h)

项目文件保存/加载，管理多点云工作区状态。

## CloudView (libs/viz/cloudview.h)

基于 VTK 的三维可视化控件。

**功能**:
- 交互式点拾取（单点和多边形区域选择）
- 相机控制和视口管理
- 支持添加形状、箭头、标签、对应关系
- 多点云渲染管理

## Console (libs/viz/console.h)

日志输出控件，提供 GUI 控制台输出功能。

## CloudType (libs/core/cloudtype.h)

点类型定义与数据结构。

**类型定义**:
```cpp
using RGB = pcl::RGB;
struct PointXYZ { float x, y, z; };

// 压缩法线（节省内存）
struct CompressedNormal {
    int nx : 11;  // [-1, 1] -> [0, 2047]
    int ny : 11;
    int nz : 10;
};
```

## 大点云处理策略

### 1. 八叉树分区
- 空间递归划分为 8 个子空间
- 叶子节点最多存储 6 万个点（可配置）
- 非连续内存分配

### 2. 流式 I/O
- 批量加载（每批 50 万点）
- 实时进度反馈
- 内存峰值控制

### 3. 全局坐标偏移
```cpp
// 问题：UTM 坐标 (x=500000) 导致 GPU 精度丢失
// 解决：渲染前减去质心
pt.x -= shift.x();
pt.y -= shift.y();
pt.z -= shift.z();

// 保存时加回
pt.x += shift.x();
```

### 4. 动态 LOD
- 交互态：降低阈值，渲染粗糙 LOD
- 静止态：提高阈值，渲染精细 Block
- SSE 策略：根据屏幕投影大小动态选择

### 5. 视锥剔除
```cpp
bool isBoxInFrustum(const Box& box, const double* planes) {
    for (int i = 0; i < 6; ++i) {  // 6 个裁剪面
        // 检查 8 个顶点是否都在平面外侧
    }
}
```

## 重要实现说明

### 全局坐标偏移
**问题**: 大坐标（如 UTM）导致 GPU 精度问题（抖动/卡顿）
**解决方案**: 渲染前减去质心，保存时加回
**配置**: `GlobalShiftDialog` 允许用户配置/记忆偏移值

### PCL 集成
- 内部使用 PCL 点云进行算法处理
- 完整的 PCL 转换在大点云下可能导致内存问题，需选择性使用
- 自定义 `Cloud` 类封装 PCL，添加八叉树索引

### AOS vs SOA
- **旧版本 (SOA)**: `std::vector<float> x, y, z;` - 多次内存跳转
- **新版本 (AOS)**: `std::vector<PointXYZ> points;` - 连续内存访问
