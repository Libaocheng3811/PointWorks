# PointWorks

> 基于 Qt5、VTK 和 PCL 构建的专业三维点云处理应用程序

## 简介

PointWorks 是一个功能丰富的点云处理软件，提供点云可视化、滤波、地面/植被分割、变化检测、点云配准、曲面重建等功能。项目采用 libs/ + src/ 两层架构设计，支持大规模点云数据的流式加载和高效 LOD 渲染。

## 功能特性

### 核心功能
- **点云可视化** - 基于 VTK 的三维交互式可视化，支持 LOD 自适应渲染
- **多格式数据导入/导出** - 支持 LAS/LAZ、PLY、PCD、TXT、E57、OBJ、STL、VTK 等
- **滤波处理** - 直通滤波、体素降采样、统计离群点移除等
- **点云配准** - ICP、IA-RANSAC、NDT 等多种配准算法
- **特征提取** - PFH、FPFH、VFH、SHOT 等特征描述符
- **地面分割** - CSF 布料模拟滤波
- **植被分割** - 多种植被指数 (ExG、NGRDI、CIVE)
- **变化检测** - 点云距离计算与分析（C2C、C2M、M3C2）
- **曲面重建** - 点云到网格转换、凸包计算、边界提取
- **分割** - 形状检测、形态学滤波、区域生长、聚类、超体素
- **嵌入式 Python** - 通过 pybind11 嵌入 Python 脚本引擎，支持自定义数据处理

### 大点云处理
- **八叉树空间索引** - 高效的空间分割与查询
- **流式加载** - 批量加载（每批 50 万点）避免内存峰值
- **LOD 渲染** - 基于屏幕空间误差（SSE）的动态细节层次
- **全局坐标偏移** - 解决 UTM 大坐标的 GPU 精度问题

## 环境要求

| 依赖项 | 版本 |
|--------|------|
| CMake | 3.28+ |
| C++ 标准 | C++17 |
| Qt5 | 5.15.2 |
| VTK | 9.1.0 |
| PCL | 1.12.1 |
| OpenMP | 最新版 |
| Python | 3.9 (EXACT) |
| pybind11 | git submodule |
| LAStools | 3rdparty 内置 |
| CSF | 3rdparty 内置 |
| E57Format | 3rdparty 内置 |
| 编译器 | MSVC 2019+ (推荐 MSVC 2022) |

## 编译构建

```bash
# 配置
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build build --config Release

# 输出: build/bin/pointworks.exe
```

## 项目架构

```
PointWorks/
├── libs/                        # 库层（可复用组件）
│   ├── core/                    # ct_core — 核心数据结构（点云、八叉树）
│   ├── viz/                     # ct_viz — 可视化（VTK 渲染、日志）
│   ├── io/                      # ct_io — 文件 I/O（点云/模型格式读写）
│   ├── algorithm/               # ct_algorithm — 算法模块（滤波、配准、分割等）
│   ├── ui/base/                 # ct_ui_base — 基础 UI 控件
│   ├── ui/dialog/               # ct_ui_dialog — 通用对话框
│   └── python/                  # ct_python — 嵌入式 Python
├── src/                        # 应用层
│   ├── app/                    # 主程序
│   ├── edit/                   # 编辑工具（颜色、变换、法线等）
│   ├── tool/                   # 处理工具（滤波、配准、分割、距离等）
│   ├── options/                # 选项设置
│   ├── plugins/                # 插件（地面分割、植被、变化检测）
│   └── python/                 # Python 控制台与编辑器
├── 3rdparty/                   # 第三方库（LASlib, CSF, pybind11, E57）
└── CMakeLists.txt
```

### 两层架构

```
┌─────────────────────────────────────────────┐
│ 应用层 (src/)                               │
│ pointworks.exe — 业务逻辑与 UI              │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────┴──────────────────────────┐
│ 库层 (libs/)                                │
│ ct_core, ct_viz, ct_io, ct_algorithm,      │
│ ct_ui_base, ct_ui_dialog, ct_python        │
└─────────────────────────────────────────────┘
```

**分层原则**: `libs/` 层严禁依赖 `src/` 层，各子库按依赖方向单向引用。

## 支持的文件格式

### 点云格式

| 格式 | 扩展名 | 读取 | 写入 | 说明 |
|------|--------|------|------|------|
| LAS | .las | Yes | Yes | ASPRS LAS 格式 |
| LAZ | .laz | Yes | Yes | 压缩 LAS 格式 |
| E57 | .e57 | Yes | Yes | ASTM E2807 工业扫描格式 |
| PLY | .ply | Yes | Yes | Stanford PLY（支持自定义字段映射） |
| PCD | .pcd | Yes | Yes | PCL 原生格式（二进制/ASCII） |
| TXT | .txt/.xyz/.asc | Yes | Yes | 文本格式（交互式字段映射配置） |

### 模型格式

| 格式 | 扩展名 | 读取 | 写入 | 说明 |
|------|--------|------|------|------|
| OBJ | .obj | Yes | Yes | Wavefront OBJ（自动检测纹理材质） |
| STL | .stl | Yes | Yes | STL 网格格式 |
| VTK | .vtk | Yes | Yes | VTK PolyData 格式 |
| IFS | .ifs | Yes | - | IFS 网格格式 |

## 主要功能模块

### 滤波
- PassThrough、VoxelGrid、StatisticalOutlierRemoval、RadiusOutlierRemoval、ConditionalRemoval、GridMinimum、LocalMaximum

### 配准
- ICP、ICPWithNormals、IA-RANSAC、SCPR、NDT

### 分割
- 形状检测、形态学滤波、区域生长、欧式聚类、超体素

### 曲面重建与网格
- 曲面重建、凸包计算、边界提取

### 距离与变化检测
- 云对云距离、云对网格距离、云对基元距离、最近点集距离

### 插件
- **CSF Plugin** - 地面点分割（布料模拟）
- **VegPlugin** - 植被分割（4 种植被指数 + Otsu 自动阈值）
- **ChangeDetectPlugin** - 变化检测（多种距离方法 + Jet 色带）

## 技术亮点

### AOS 内存布局
采用 Array of Structures 格式存储点云数据，连续内存访问，可处理更大规模的点云文件。

### 八叉树 + LOD
- 空间分割递归划分 3D 空间，叶子节点最多存储 6 万个点
- 蓄水池采样 (Reservoir Sampling) 实时生成 LOD
- SSE (Screen Space Error) 策略动态选择渲染精细度

### 流式 I/O
- 批量加载（每批 50 万点）使用 `CloudBatch` 缓冲区
- 实时进度反馈，协作式取消
- 文件 I/O 在独立 QThread 中执行

### 全局坐标偏移
渲染前减去偏移，保存时加回，解决 UTM 大坐标导致的 GPU 浮点精度问题。支持用户交互式配置和跳过。

### 模块化设置架构
Display Settings 采用 QListWidget 侧栏 + QStackedWidget 页面模式，便于扩展新的设置类别。

## 许可证

本项目使用的第三方库请遵循各自的许可证协议。
