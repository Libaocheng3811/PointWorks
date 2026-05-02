# UI 组件、工具与插件系统

## 架构概述

CloudTree 上帝类已拆分为 4 个专注组件：

```
CloudTree（精简）
├── CloudRegistry      — 云/网格/形状注册表
├── CloudIOController  — 文件 I/O 编排（PRIVATE ct_io）
└── ProgressManager    — 进度/取消/异步执行统一接口
```

## DialogRegistry (libs/ui/base/dialog_registry.h)

单例类，替代原有的 `static` 全局变量，管理所有对话框和停靠栏。

**API**:
```cpp
DialogRegistry& reg = DialogRegistry::instance();
reg.registerDialog("Filters", dlg);
reg.getDialog("Filters");
reg.hasOpenDialog();       // 互斥检查
reg.unregisterDialog("Filters");  // destroyed 信号自动调用
```

## ProgressManager (libs/ui/base/progress_manager.h)

统一进度管理，替代 20+ 处重复的并发样板代码。

**统一异步执行**:
```cpp
// 旧方式：15 行样板代码（QAtomicInt + QtConcurrent + QFutureWatcher + closeProgress）
// 新方式：
m_progress->runAsync("CSF 地面分割",
    [=]() -> CSFResult {
        return CSFFilter::apply(cloud, params, progress);
    },
    [=](CSFResult result) {
        // 完成回调（主线程）
    }
);
```

**获取方式**: 通过 `CustomDialog` 基类的 `m_progress` 成员，`createDialog<T>` 自动注入。

## CloudRegistry (libs/ui/base/cloudregistry.h)

云/网格/形状注册表，从 CloudTree 提取的数据管理组件。

**管理内容**:
- `m_cloud_map`: QTreeWidgetItem* → Cloud::Ptr
- `m_mesh_map`: cloudId → PolygonMesh::Ptr
- `m_textured_mesh_map`: cloudId → TexturedMeshPtr
- `m_shape_map`: shapeId → PolygonMesh::Ptr
- `m_cloud_polyline_map`: shapeId → Cloud::Ptr
- `m_clouds_in_use`: 脚本删除保护

**获取方式**: 通过 `CloudTree::registry()` 或 `m_cloudtree->registry()`。

## CloudIOController (libs/ui/base/cloud_io_controller.h)

文件 I/O 编排，从 CloudTree 提取。头文件仅使用 `ct_core` 类型和前向声明 `class FileIO`，**不依赖 ct_io 头文件**。`ct_io` 仅在 `.cpp` 中链接（PRIVATE）。

**获取方式**: 通过 `CloudTree::ioController()` 或 `m_cloudtree->ioController()`。

## CloudTree (libs/ui/base/cloudtree.h)

精简后的点云树 UI 控件，仅负责：
- 树节点管理（插入、删除、克隆、合并）
- 右键菜单
- 复选框/拖放
- 通过组合持有 `CloudRegistry`、`CloudIOController`、`ProgressManager`

```cpp
class CloudTree : public CustomTree {
    CloudRegistry* m_registry;
    CloudIOController* m_io;
    ProgressManager* m_progress;
    QMenu* m_tree_menu;
};
```

## CustomDialog (libs/ui/base/customdialog.h)

对话框基类。

**特性**:
- 依赖注入（CloudView、CloudTree、Console、ProgressManager）
- 自动位置管理（工具浮窗跟随 CloudView）
- `DialogRegistry` 单例管理
- `Qt::WA_DeleteOnClose` 自动清理

**使用模式**:
```cpp
// 工具浮窗（无边框、跟随视图）
ct::createDialog<Filters>(parent, "Filters", cloudview, cloudtree, console,
                           true, false);

// 模态对话框（阻塞、居中）
ct::createDialog<DisplaySettingsDialog>(parent, "Display Settings",
    cloudview, cloudtree, console, false, true);
```

## 通用对话框 (libs/ui/dialog/)

| 对话框 | 文件 | 功能 |
|--------|------|------|
| ProcessingDialog | processingdialog.h | 进度条 + 取消按钮 |
| GlobalShiftDialog | globalshiftdialog.h/cpp/ui | 全局偏移设置 + 记忆 |
| FieldMappingDialog | fieldmappingdialog.h | TXT 字段映射 |
| TxtImportDialog | txtimportdialog.h | TXT 导入配置 |
| TxtExportDialog | txtexportdialog.h | TXT 导出配置 |

## 工具列表 (src/tool/)

| 工具 | 文件 | 功能 |
|------|------|------|
| Filters | src/tool/filters.h | 滤波器类型选择、参数配置、预览/应用 |
| Cutting | src/tool/cutting.h | 包围盒裁剪、多边形裁剪 |
| PickPoints | src/tool/pickpoints.h | 单点拾取、多边形区域选择 |
| RangeImage | src/tool/rangeimage.h | 深度图、边界提取、法线估计 |
| Sampling | src/tool/sampling.h | 点云采样 |
| Measure | src/tool/measure.h | 距离测量 |

### 配准工具 (src/tool/align/)

| 工具 | 文件 | 功能 |
|------|------|------|
| AlignByCenters | src/tool/align/align_by_centers.h | 中心对齐 |
| GlobalRegistration | src/tool/align/global_registration.h | 全局配准 |
| FineRegistration | src/tool/align/fine_registration.h | 精配准 |
| PointPairsAlignment | src/tool/align/point_pairs_alignment.h | 点对配准 |

### 距离工具 (src/tool/distance/)

| 工具 | 文件 | 功能 |
|------|------|------|
| CloudCloudDist | src/tool/distance/cloud_cloud_dist_dialog.h | 云对云距离 |
| CloudMeshDist | src/tool/distance/cloud_mesh_dist_dialog.h | 云对网格距离 |
| CloudPrimitiveDist | src/tool/distance/cloud_primitive_dist_dialog.h | 云对基元距离 |
| ClosestPointSet | src/tool/distance/closest_point_set_dialog.h | 最近点集 |

### 网格工具 (src/tool/mesh/)

| 工具 | 文件 | 功能 |
|------|------|------|
| ReconstructSurface | src/tool/mesh/reconstruct_surface_dialog.h | 曲面重建（Poisson/Greedy/MC） |
| ComputeHull | src/tool/mesh/compute_hull_dialog.h | 凸包/凹包计算 |
| ExtractBoundary | src/tool/mesh/extract_boundary_dialog.h | 边界提取 |

### 分割工具 (src/tool/segmentation/)

| 工具 | 文件 | 功能 |
|------|------|------|
| ShapeDetection | src/tool/segmentation/shape_detection_dialog.h | 形状检测（RANSAC） |
| MorphologicalFilter | src/tool/segmentation/morphological_filter_dialog.h | 形态学滤波 |
| RegionGrowing | src/tool/segmentation/region_growing_dialog.h | 区域生长 |
| Clustering | src/tool/segmentation/clustering_dialog.h | 欧式聚类/DBSCAN |
| Supervoxel | src/tool/segmentation/supervoxel_dialog.h | 超体素分割 |

## 编辑工具列表 (src/edit/)

| 工具 | 文件 | 功能 |
|------|------|------|
| Color | src/edit/color.h | 颜色设置（点云 Points/Normals + 模型） |
| BoundingBox | src/edit/boundingbox.h | 包围盒绘制 |
| Transformation | src/edit/transformation.h | 点云变换（平移/旋转/矩阵） |
| Normals | src/edit/normals.h | 法线编辑 |
| Scale | src/edit/scale.h | 尺度缩放 |
| Coordinate | src/edit/coordinate.h | 坐标系显示与对比（支持自定义位置、缩放、变换矩阵） |

## 选项设置 (src/options/)

采用 QListWidget 侧栏 + QStackedWidget 的模块化设置页面架构。

## 插件系统

### 插件模板

所有插件遵循统一模板：

```cpp
class Plugin : public ct::CustomDialog {
    void init() override;       // 设置 UI 连接
    void onApply();             // 使用 m_progress->runAsync() 执行
    void onDone(Result);        // 完成回调
};
```

### 已有插件 (src/plugins/)

| 插件 | 文件 | 功能 |
|------|------|------|
| CSFPlugin | src/plugins/csfplugin.h | 地面点分割（布料模拟） |
| VegPlugin | src/plugins/vegplugin.h | 植被分割（4 种植被指数 + Otsu 自动阈值） |
| ChangeDetectPlugin | src/plugins/changedetectplugin.h | 变化检测（C2C/C2M 距离 + Jet 色带） |
| M3C2Plugin | src/plugins/m3c2plugin.h | M3C2 多尺度模型对比距离计算 |

> 注意：`changedetectplugin.cpp` 仍直接访问 `CloudBlock` 内部（`->m_points` 等），
> 因为该文件使用 block 批量遍历模式，`forEachPoint` 无法直接替代，需后续迭代处理。

## ViewCube (libs/viz/viewcube.h)

视图方向指示器，作为可视化辅助组件嵌入在 CloudView 中。

**功能**:
- 可拖拽立方体显示当前相机朝向
- 点击面/边/角切换标准视角（Top/Front/Right 等）
- 坐标轴颜色标注（X=红, Y=绿, Z=蓝）
- 鼠标拖拽旋转相机

## 多视窗 (MainWindow)

主窗口支持多视窗显示，每个视窗独立的 CloudView 实例，支持独立相机控制和点云渲染。

## 国际化 (i18n)

支持中文/英文切换，翻译文件位于 `src/resources/trans/zh_CN.ts`，通过 `LanguageManager` 管理。
