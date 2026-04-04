# 项目架构地图

CloudTool2 是一个基于 Qt5、VTK 和 PCL 构建的三维点云处理应用程序。严格遵循三层架构。

## 三层架构

```
┌─────────────────────────────────────────────┐
│ Level 3: 应用层 (cloudtool/)                 │
│ MainWindow, Tools, Plugins                  │
│ 业务逻辑与 UI 结合                           │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────┴──────────────────────────┐
│ Level 2: UI 组件层 (widgets/)               │
│ CustomDialog, CustomDock, CloudTree         │
│ 通用对话框                                   │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────┴──────────────────────────┐
│ Level 1: 核心与算法层                        │
│ ct_core: Cloud, Octree, FileIO, CloudView   │
│ ct_modules: Filters, Features, Registration │
└─────────────────────────────────────────────┘
```

## 依赖关系

```
cloudtool
    ↓ depends on
ct_widget ←→ ct_common_ui
    ↓ depends on
ct_python (OBJECT) ←→ ct_modules
    ↓               ↓ depends on
    ↓ depends on    ct_core
ct_core              ↓ depends on
    ↓ depends on    Qt5, PCL
Qt5, VTK, PCL, OpenMP, pybind11, Python3
```

## 目录结构

```
CloudTool2/
├── 3rdparty/                    # 第三方库
│   ├── CSF/                    # 地面滤波算法（布料模拟）
│   ├── LAStools/               # LAS/LAZ 格式支持
│   │   ├── LASlib/             # LAS 读写库
│   │   └── LASzip/             # LAZ 压缩库
│   └── pybind11/               # Python C++ 绑定库（git submodule）
├── core/                        # 核心数据结构 (ct_core)
│   ├── cloud.h/cpp             # 点云数据结构（AOS 格式）
│   ├── cloudtype.h             # 点类型定义与压缩法线
│   ├── octree.h/cpp            # 八叉树空间索引 + LOD
│   ├── octreerenderer.h/cpp    # 高级渲染器（SSE 遍历）
│   ├── fileio.h/cpp            # 文件 I/O（流式加载）
│   ├── cloudview.h/cpp         # VTK 三维可视化控件
│   ├── console.h/cpp           # 日志输出
│   └── common.h                # 通用工具函数
├── python/                      # 嵌入式 Python 模块 (ct_python)
│   ├── python_manager.h/cpp    # Python 解释器生命周期管理（单例）
│   ├── python_worker.h/cpp     # QThread 脚本执行引擎（GIL + 异步取消）
│   ├── python_bridge.h/cpp     # 信号桥接 + 线程安全云注册表
│   ├── python_bindings.cpp     # pybind11 嵌入模块 `ct`（PyCloud + 工厂函数）
│   └── CMakeLists.txt          # OBJECT 库构建配置
├── modules/                     # 算法模块 (ct_modules)
│   ├── filters.h/cpp           # 滤波算法
│   ├── features.h/cpp          # 特征提取
│   ├── registration.h/cpp      # 点云配准
│   ├── keypoints.h/cpp         # 关键点检测
│   ├── csffilter.h/cpp         # CSF 地面分割
│   ├── vegfilter.h/cpp         # 植被分割
│   ├── distancecalculator.h/cpp # 距离计算（变化检测）
│   └── utils.h                 # 算法工具函数
├── widgets/                     # UI 组件
│   ├── customdialog.h          # 对话框基类
│   ├── customdock.h            # Dock 基类
│   ├── customtree.h            # 树控件基类
│   ├── cloudtree.h/cpp         # 点云树管理
│   └── common_ui/              # 通用对话框 (ct_common_ui)
├── cloudtool/                   # 应用层
│   ├── main.cpp                # 程序入口
│   ├── mainwindow.h/cpp        # 主窗口
│   ├── edit/                   # 编辑工具
│   ├── tool/                   # 处理工具
│   ├── plugins/                # 插件系统
│   └── resources/              # 资源文件
├── camera/                     # 相机 SDK（可选）
├── data/                       # 测试数据
└── CMakeLists.txt              # 主构建配置
```

## 设计模式

| 模式 | 应用位置 |
|------|----------|
| 工厂模式 | 对话框创建 (`createDialog<T>`) |
| 观察者模式 | 信号/槽机制 |
| 策略模式 | 配准算法选择 |
| 单例模式 | 对话框管理 (`registed_dialogs`) |
| 模板方法 | 插件架构 (`init()` + `onApply()`) |

## 渲染管线

```
1. 数据存储在 CloudBlock 结构中（AOS 格式）
2. OctreeRenderer 管理 VTK actor 对象池
3. 视锥剔除确定可见节点
4. SSE 遍历选择合适的 LOD 级别
5. Actor 池复用 GPU 资源（最多 500 个）
```
