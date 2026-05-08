# 项目架构地图

PointWorks 是一个基于 Qt5、VTK 和 PCL 构建的三维点云处理应用程序（原名 CloudTool2）。采用 libs/ + src/ 两层架构。

## 两层架构

```
┌─────────────────────────────────────────────┐
│ 应用层 (src/)                               │
│ pointworks 可执行文件                        │
│ app/, tool/, edit/, options/, plugins/, python/ │
│ 业务逻辑与 UI 结合                           │
└──────────────────┬──────────────────────────┘
                   │ depends on
┌──────────────────┴──────────────────────────┐
│ 库层 (libs/)                                │
│ pw_core, pw_viz, pw_io, pw_algorithm,      │
│ pw_ui_base, pw_ui_dialog, pw_python        │
│ 可复用组件与算法                              │
└─────────────────────────────────────────────┘
```

## 依赖关系

```
pointworks (executable)
    ↓ depends on
pw_ui_base (STATIC) ←→ pw_ui_dialog (STATIC)
    ↓                    ↓ depends on
    ↓ depends on         pw_core (SHARED)
    ↓
pw_viz (SHARED) → pw_io (SHARED) → pw_core (SHARED)
    ↓
pw_python (OBJECT)   pw_algorithm (STATIC) → CSF_Lib
    ↓                    ↓
pybind11::embed      PCL, OpenMP
```

**重要**: `pw_io` 对 `pw_ui_base` 是 PRIVATE 依赖，不向下游传播。`pw_viz` 仍 PUBLIC 依赖 `pw_io`。

**外部依赖**: Qt5, VTK, PCL, OpenMP, pybind11, Python3, LASlib, E57Format, CSF

## 目录结构

```
PointWorks/
├── libs/                        # 库层（可复用组件）
│   ├── core/                    # pw_core (SHARED) — 核心数据结构
│   │   ├── cloud.h/cpp         # 点云数据结构（AOS 格式）
│   │   ├── cloudtype.h         # 点类型定义与压缩法线
│   │   ├── octree.h            # 八叉树空间索引 + LOD
│   │   ├── common.h/cpp        # 通用工具函数
│   │   ├── exports.h           # DLL 导出宏
│   │   ├── colormap.h          # 色图（纯 std::string，无 Qt 依赖）
│   │   ├── field_types.h       # 标量场/距离/配准参数类型定义
│   │   ├── view_params.h       # 相机参数与视图选项（从 projectfile.h 移出）
│   │   ├── statistics.h        # 统计计算工具（点数、包围盒、分辨率等）
│   │   └── textured_mesh.h     # 纹理网格数据结构（含材质/纹理路径）
│   ├── viz/                     # pw_viz (SHARED) — 可视化
│   │   ├── cloudview.h/cpp     # VTK 三维可视化控件
│   │   ├── octreerenderer.h/cpp # 高级渲染器（SSE 遍历）
│   │   ├── surface_viz_helper.h/cpp # 曲面重建 VTK 渲染辅助（从 algorithm 移入）
│   │   ├── viewcube.h/cpp      # 视图方向指示器（ViewCube 导航）
│   │   ├── scalar_bar_widget.h/cpp # 标量场色度条控件
│   │   └── console.h/cpp       # 日志输出
│   ├── io/                      # pw_io (SHARED) — 文件 I/O
│   │   ├── fileio.h/cpp        # FileIO 调度器（加载/保存入口）
│   │   ├── fileio_pointcloud.cpp # 点云格式读写（LAS, PLY, PCD, TXT, E57）
│   │   ├── fileio_mesh.cpp     # 模型格式读写（OBJ, STL, VTK, IFS）
│   │   └── projectfile.h/cpp   # 项目文件保存/加载
│   ├── algorithm/               # pw_algorithm (STATIC) — 算法模块
│   │   ├── filters.h/cpp       # 滤波算法
│   │   ├── features.h/cpp      # 特征提取
│   │   ├── keypoints.h/cpp     # 关键点检测
│   │   ├── registration.h/cpp  # 点云配准
│   │   ├── csffilter.h/cpp     # CSF 地面分割
│   │   ├── vegfilter.h/cpp     # 植被分割
│   │   ├── distancecalculator.h/cpp # 距离计算（变化检测）
│   │   ├── normals.h/cpp       # 法线估计
│   │   ├── segmentation.h/cpp  # 点云分割
│   │   ├── surface.h/cpp       # 曲面重建（纯 PCL，零 VTK）
│   │   └── utils.h             # 算法工具函数（头文件）
│   ├── ui/                      # UI 组件库
│   │   ├── base/               # pw_ui_base (STATIC) — 基础控件
│   │   │   ├── cloudtree.h/cpp # 点云树 UI（精简，通过组合持有子系统）
│   │   │   ├── cloudregistry.h/cpp # 云/网格/形状注册表
│   │   │   ├── cloud_io_controller.h/cpp # 文件 I/O 编排（ PRIVATE ct_io）
│   │   │   ├── progress_manager.h/cpp # 进度/取消/异步执行统一接口
│   │   │   ├── dialog_registry.h/cpp # DialogRegistry 单例
│   │   │   ├── customdialog.h  # 对话框基类 + createDialog<T> 模板
│   │   │   ├── customdock.h    # Dock 基类 + createDock<T> 模板
│   │   │   ├── customtree.h/cpp # 树控件基类
│   │   │   ├── paramsnapshot.h # 参数快照
│   │   │   └── scenenodetype.h # 场景节点类型定义
│   │   ├── dialog/             # pw_ui_dialog (STATIC) — 通用对话框
│   │   │   ├── processingdialog.h     # 进度条 + 取消按钮
│   │   │   ├── globalshiftdialog.h/cpp/ui # 全局偏移设置
│   │   │   ├── fieldmappingdialog.h    # TXT 字段映射
│   │   │   ├── txtimportdialog.h       # TXT 导入配置
│   │   │   └── txtexportdialog.h       # TXT 导出配置
│   │   └── widgets/            # pw_ui_base 子组件
│   │       ├── sf_histogram_chart.h/cpp # 标量场直方图
│   │       └── sf_display_panel.h/cpp   # 标量场显示面板
│   └── python/                   # pw_python (OBJECT) — 嵌入式 Python
│       ├── python_manager.h/cpp # Python 解释器生命周期管理（单例，Qt-free 头文件）
│       ├── python_worker.h/cpp  # QThread 脚本执行引擎（GIL + 异步取消）
│       ├── python_bridge.h/cpp  # 信号桥接（内部持有 CloudRegistry）
│       ├── cloud_registry.h/cpp # Qt-free 线程安全云注册表
│       ├── python_bindings.cpp  # pybind11 嵌入模块 `ct`
│       └── bindings/            # 16 个绑定模块
│           ├── bind_common.h    # 公共 include
│           ├── bind_core.h/cpp  # ct.Cloud 类 + 基础函数
│           ├── bind_cloud_mgmt.h/cpp # 云管理（加载/保存/克隆/合并）
│           ├── bind_view.h/cpp  # 视图控制
│           ├── bind_appearance.h/cpp # 外观控制
│           ├── bind_overlay.h/cpp # 叠加物
│           ├── bind_progress.h/cpp # 进度管理
│           ├── bind_transform.h/cpp # 变换/缩放/裁剪
│           ├── bind_filters.h/cpp # 滤波算法
│           ├── bind_normals.h/cpp # 法线估计
│           ├── bind_features.h/cpp # 特征描述子
│           ├── bind_keypoints.h/cpp # 关键点检测
│           ├── bind_registration.h/cpp # 配准算法
│           ├── bind_segmentation.h/cpp # 分割算法
│           ├── bind_surface.h/cpp # 曲面重建
│           ├── bind_csf_veg.h/cpp # CSF + 植被分割
│           ├── bind_distance.h/cpp # 距离计算
│           └── mesh_utils.h/cpp # ct.Mesh 类
├── src/                        # 应用层（pointworks 可执行文件）
│   ├── app/                    # 主程序
│   │   ├── main.cpp            # 程序入口（Python 初始化 + 语言加载）
│   │   ├── mainwindow.h/cpp/ui # 主窗口（多视窗管理、菜单、工具栏）
│   │   ├── projectmanager.h/cpp # 项目管理器
│   │   └── recentprojects.h    # 最近项目列表
│   ├── tool/                   # 处理工具
│   │   ├── cutting.h/cpp/ui    # 裁剪
│   │   ├── pickpoints.h/cpp/ui # 点拾取
│   │   ├── filters.h/cpp/ui    # 滤波器
│   │   ├── rangeimage.h/cpp    # 深度图
│   │   ├── sampling.h/cpp/ui   # 采样
│   │   ├── measure.h/cpp/ui    # 测量
│   │   ├── align/              # 配准工具（4 个）
│   │   │   ├── align_by_centers.h/cpp # 中心对齐
│   │   │   ├── global_registration.h/cpp # 全局配准
│   │   │   ├── fine_registration.h/cpp # 精配准
│   │   │   └── point_pairs_alignment.h/cpp # 点对配准
│   │   ├── segmentation/       # 分割工具（5 个）
│   │   │   ├── shape_detection_dialog.h/cpp # 形状检测
│   │   │   ├── morphological_filter_dialog.h/cpp # 形态学滤波
│   │   │   ├── region_growing_dialog.h/cpp # 区域生长
│   │   │   ├── clustering_dialog.h/cpp # 欧式/DBSCAN 聚类
│   │   │   └── supervoxel_dialog.h/cpp # 超体素
│   │   ├── mesh/               # 网格工具（3 个）
│   │   │   ├── reconstruct_surface_dialog.h/cpp # 曲面重建
│   │   │   ├── compute_hull_dialog.h/cpp # 凸包/凹包
│   │   │   └── extract_boundary_dialog.h/cpp # 边界提取
│   │   └── distance/           # 距离工具（4 个）
│   │       ├── cloud_cloud_dist_dialog.h/cpp # 云对云距离
│   │       ├── cloud_mesh_dist_dialog.h/cpp # 云对网格距离
│   │       ├── cloud_primitive_dist_dialog.h/cpp # 云对基元距离
│   │       └── closest_point_set_dialog.h/cpp # 最近点集
│   ├── edit/                   # 编辑工具
│   │   ├── color.h/cpp/ui      # 颜色设置
│   │   ├── boundingbox.h/cpp/ui # 包围盒
│   │   ├── transformation.h/cpp/ui # 变换
│   │   ├── normals.h/cpp/ui    # 法线编辑
│   │   ├── scale.h/cpp/ui      # 尺度缩放
│   │   └── coordinate.h/cpp/ui # 坐标系
│   ├── options/                # 选项设置（模块化页面架构）
│   │   └── displaysettingsdialog.h/cpp # 显示设置对话框
│   ├── plugins/                # 插件系统
│   │   ├── csfplugin.h/cpp/ui  # CSF 地面分割插件
│   │   ├── vegplugin.h/cpp/ui  # 植被分割插件
│   │   ├── changedetectplugin.h/cpp/ui # 变化检测插件
│   │   └── m3c2plugin.h/cpp/ui # M3C2 距离计算插件
│   ├── python/                 # Python UI 组件
│   │   ├── python_console.h/cpp # Python 交互式控制台
│   │   └── python_editor.h/cpp  # Python 脚本编辑器
│   ├── help/                   # 帮助启动器
│   ├── resources/              # 资源文件
│   │   ├── res.qrc             # Qt 资源文件
│   │   ├── icon/               # SVG 图标（483 个）
│   │   ├── images/             # 工具截图
│   │   ├── logo/               # Logo 文件
│   │   ├── theme/              # QSS 主题（darkstyle + lightstyle）
│   │   └── trans/              # 国际化翻译（zh_CN）
│   └── device/                 # 设备支持（预留，空目录）
├── tests/                      # 单元测试（Google Test）
│   ├── common/                 # test_common 静态库
│   ├── core/                   # 核心测试
│   ├── algorithm/              # 算法测试
│   └── io/                     # I/O 测试
├── scripts/                    # 构建 & 部署脚本
│   ├── deploy_collect.bat      # 依赖收集打包
│   ├── deploy_portable.bat     # 绿色版 ZIP 打包
│   └── examples/               # Python 示例脚本（5 个）
├── installer/                  # 安装包
│   └── pointworks.iss          # Inno Setup 安装脚本（v1.0.0）
├── 3rdparty/                   # 第三方库
│   ├── CSF/                    # 地面滤波算法（布料模拟）
│   ├── LAStools/               # LAS/LAZ 格式支持
│   ├── libE57/                 # E57 格式（预编译静态库，gitignored）
│   ├── libE57-Format/          # E57 源码 + CMake（gitignored）
│   ├── Xerces-C/               # XML 解析器，E57 依赖（预编译，gitignored）
│   ├── googletest/             # Google Test 单元测试框架
│   ├── python-embed/           # 嵌入式 Python 3.9 发行版（gitignored）
│   └── pybind11/               # Python C++ 绑定库（git submodule）
├── data/                       # 测试数据
├── planDocs/                   # 重构方案文档（53 份）
└── CMakeLists.txt              # 主构建配置
```

## 设计模式

| 模式 | 应用位置 |
|------|----------|
| 工厂模式 | 对话框创建 (`createDialog<T>`, `createModalDialog<T>`) |
| 观察者模式 | 信号/槽机制 |
| 策略模式 | 配准算法选择 |
| 单例模式 | `DialogRegistry`（对话框/Dock 管理）、`PythonManager` |
| 模板方法 | 插件架构 (`init()` + `onApply()`) |
| 组合模式 | `CloudTree` 通过组合持有 `CloudRegistry` + `CloudIOController` + `ProgressManager` |
| 模块化设置页 | QListWidget 侧栏 + QStackedWidget（Display Settings） |

## 渲染管线

```
1. 数据存储在 CloudBlock 结构中（AOS 格式）
2. OctreeRenderer 管理 VTK actor 对象池
3. 视锥剔除确定可见节点
4. SSE 遍历选择合适的 LOD 级别
5. Actor 池复用 GPU 资源（最多 500 个）
```

## 架构约束

| 约束 | 说明 |
|------|------|
| `libs/` → `src/` 禁止 | libs 层严禁引用 src 层，零违规 |
| `pw_ui_base` → `pw_io` PRIVATE | pw_io 不向链接 pw_ui_base 的下游目标传播 |
| `libs/algorithm/` 零 VTK | SurfaceResult 无 vtkPolyData，渲染预处理由 `SurfaceVizHelper` 负责 |
| `libs/core/` 无 Qt 依赖 | colormap.h 使用 `std::string`，UI 层负责 Qt 转换 |

## 库类型策略

| 库 | 类型 | 理由 |
|----|------|------|
| pw_core | SHARED | 核心基础设施，多模块共享 |
| pw_viz | SHARED | 可视化基础设施 |
| pw_io | SHARED | 文件 I/O 基础设施 |
| pw_algorithm | STATIC | 算法模块，无跨模块共享需求，零 VTK |
| pw_ui_base | STATIC | UI 组件，仅应用层使用 |
| pw_ui_dialog | STATIC | 对话框，仅应用层使用 |
| pw_python | OBJECT | 直接链接到可执行文件 |
