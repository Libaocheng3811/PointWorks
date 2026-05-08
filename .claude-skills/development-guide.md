# 开发指南

## Git 提交规范

遵循约定式提交格式：

```
<type>(<scope>): <description>
```

**类型**: feat, fix, docs, style, refactor, perf, test, chore

**示例**:
```
feat(ui): 添加导出按钮
fix(core): 修复空指针崩溃
refactor(render): 优化八叉树渲染性能
```

## 添加新的滤波器

1. 在 `libs/algorithm/filters.h` 中添加滤波器枚举
2. 在 `Filters::apply()` 中实现滤波逻辑
3. 在 `src/tool/filters.h` 中添加 UI 配置

## 添加新的算法模块

1. 在 `libs/algorithm/` 下创建新的 `.h` 和 `.cpp` 文件
2. **禁止引入 VTK 头文件**，保持 `pw_algorithm` 零 VTK 依赖
3. 如需 VTK 渲染预处理，在 `libs/viz/` 中创建 `xxx_viz_helper.h/cpp`
4. 在 `libs/algorithm/CMakeLists.txt` 中注册源文件（如使用 `file(GLOB)` 则自动收集）
5. 如需新工具 UI，在 `src/tool/` 下创建对应文件并在 `src/CMakeLists.txt` 中注册

## 添加新的插件

1. 在 `src/plugins/` 下创建插件文件（继承 `pw::CustomDialog`）
2. 实现 `init()`, `onApply()`, `onDone()` 方法
3. **使用 `m_progress->runAsync()` 执行异步任务**，不要手动创建 QThread
4. 在 `src/CMakeLists.txt` 中注册源文件
5. 在 `MainWindow` 中注册插件

## 添加新的编辑工具

1. 在 `src/edit/` 下创建工具文件
2. 在 `src/CMakeLists.txt` 中注册源文件和 UI 文件
3. 在 `MainWindow` 中注册菜单项

## 添加新的文件格式

**点云格式**:
1. 在 `libs/io/fileio.h` 中添加 `loadXXX()` / `saveXXX()` 私有方法声明
2. 在 `libs/io/fileio_pointcloud.cpp` 中实现
3. 在 `fileio.cpp` 的 `loadPointCloud()` / `savePointCloud()` 调度器中添加分发分支
4. 在 `libs/ui/base/cloud_io_controller.cpp` 的 `addCloud()` 中添加格式过滤器
5. 测试流式加载和坐标偏移

**模型格式**:
1. 在 `libs/io/fileio.h` 中添加 `loadXXX()` 私有方法声明（含 mesh 参数）
2. 在 `libs/io/fileio_mesh.cpp` 中实现
3. 在 `fileio.cpp` 的 `loadPointCloud()` 调度器的 else 分支中处理
4. 在 `fileio.cpp` 的 `saveMeshFile()` 中添加保存分发
5. 测试网格加载和纹理检测

> pw_io 使用 `file(GLOB *.cpp)`，新增 .cpp 文件无需手动注册。

## 扩展 Python API

1. 在 `libs/python/python_bindings.cpp` 的 `PYBIND11_EMBEDDED_MODULE(ct, m)` 中添加新函数或类
2. 零拷贝访问使用 `py::capsule` 绑定 `Cloud::Ptr` 生命周期
3. UI 操作只通过 `PythonBridge` 发射信号，禁止 Python 线程直接操作 UI
4. 需要新的 UI 信号时，在 `PythonBridge` 中添加信号定义 + `MainWindow` 中 connect

## 架构注意事项

### 依赖隔离

- `libs/ui/` 的公共头文件中**禁止** `#include "io/..."` — pw_io 对 pw_ui_base 是 PRIVATE
- `libs/algorithm/` 中**禁止** `#include <vtk...>` — 零 VTK 依赖
- `libs/core/` 中**禁止** `#include <Q...>` — 核心层无 Qt 依赖

### 进度管理

- **推荐**: 使用 `m_progress->runAsync(title, work, onDone)`
- **禁止**: 直接访问 `ProcessingDialog` 或创建 `QAtomicInt` + `QFutureWatcher` 样板代码
- ProgressManager 通过 `CustomDialog::m_progress` 获取，`createDialog<T>` 自动注入

### 对话框生命周期

- 对话框设置 `Qt::WA_DeleteOnClose`
- `DialogRegistry::unregisterDialog` 在 `destroyed` 信号中自动调用
- lambda 捕获注意：**必须使用 `[=]` 值捕获局部变量**，禁止 `[&]` 引用捕获（避免悬空引用）

### pw_core 第三方依赖现状

`pw_core` 中存在以下第三方库依赖（短期内无法移除）：

| 头文件 | 依赖 | 原因 |
|--------|------|------|
| `cloudtype.h` | PCL + Eigen | 项目基石点类型，全项目数百处引用 |
| `common.h` | Eigen | 通用几何类型 |
| `view_params.h` | Eigen | 相机参数含 Eigen 向量/矩阵 |
| `textured_mesh.h` | PCL PolygonMesh | 可通过 `shared_ptr<void>` 类型擦除移除 |

## 常见问题

### Q: 编译时找不到 Qt5/VTK/PCL？
确保已正确设置 `CMAKE_PREFIX_PATH` 或环境变量。

### Q: 大点云文件打开缓慢？
已启用流式加载和 LOD 渲染，可调整 `CloudConfig` 参数。

### Q: UTM 坐标渲染时抖动？
使用全局坐标偏移功能。

### Q: Python 初始化失败？
Python 路径通过 QSettings（注册表 `PointWorks/PointWorks` → `python_home`）动态配置，
也可通过 Options → Display Settings → Python 页面设置。默认使用嵌入式 Python 3.9。

### Q: `#undef slots` 编译错误？
pybind11 头文件必须在 `#undef slots` 之后包含。

### Q: 项目重命名？
项目已从 CloudTool2 重命名为 **PointWorks**，可执行文件为 `pointworks.exe`。
