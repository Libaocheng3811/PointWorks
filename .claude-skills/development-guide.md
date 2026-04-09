# 开发指南

## Git 提交规范

遵循约定式提交格式：

```
<type>(<scope>): <description>
```

**类型 (type)**:
- `feat` - 新功能
- `fix` - 缺陷修复
- `docs` - 文档更新
- `style` - 代码格式
- `refactor` - 代码重构
- `perf` - 性能优化
- `test` - 测试相关
- `chore` - 构建/工具

**示例**:
```
feat(ui): 添加导出按钮
fix(core): 修复空指针崩溃
refactor(render): 优化八叉树渲染性能
chore(build): 更新 cmake 版本
```

## 添加新的滤波器

1. 在 `libs/algorithm/filters.h` 中添加滤波器枚举
2. 在 `Filters::apply()` 中实现滤波逻辑
3. 在 `src/tool/filters.h` 中添加 UI 配置

## 添加新的算法模块

1. 在 `libs/algorithm/` 下创建新的 `.h` 和 `.cpp` 文件
2. 在 `libs/algorithm/CMakeLists.txt` 中注册源文件
3. 如需新工具 UI，在 `src/tool/` 下创建对应文件并在 `src/CMakeLists.txt` 中注册

## 添加新的插件

1. 在 `src/plugins/` 下创建插件文件（继承 `ct::CustomDialog`）
2. 实现 `init()`, `onApply()`, `onDone()` 方法
3. 在 `src/CMakeLists.txt` 中注册源文件
4. 在 `MainWindow` 中注册插件

## 添加新的编辑工具

1. 在 `src/edit/` 下创建工具文件
2. 在 `src/CMakeLists.txt` 中注册源文件和 UI 文件
3. 在 `MainWindow` 中注册菜单项

## 添加新的文件格式

1. 在 `libs/io/fileio.h` 中添加加载/保存函数
2. 在 `libs/ui/base/cloudtree.h` 中添加格式过滤器
3. 测试流式加载和坐标偏移

## 扩展 Python API

1. 在 `libs/python/python_bindings.cpp` 的 `PYBIND11_EMBEDDED_MODULE(ct, m)` 中添加新函数或类
2. 零拷贝访问使用 `py::capsule` 绑定 `Cloud::Ptr` 生命周期
3. UI 操作只通过 `PythonBridge` 发射信号，禁止 Python 线程直接操作 UI
4. 需要新的 UI 信号时，在 `PythonBridge` 中添加信号定义 + `MainWindow` 中 connect

## 常见问题

### Q: 编译时找不到 Qt5/VTK/PCL？
请确保已正确设置 `CMAKE_PREFIX_PATH` 或环境变量。

### Q: 大点云文件打开缓慢？
已启用流式加载和 LOD 渲染，可尝试调整 `CloudConfig` 中的参数。

### Q: UTM 坐标渲染时抖动？
使用全局坐标偏移功能，在导入时设置合适的偏移值。

### Q: 内存占用过高？
检查是否启用了八叉树模式，调整 `maxPointsPerBlock` 参数。

### Q: Python 初始化失败？
检查 `MY_NATIVE_PYTHON_DIR` 路径是否正确指向 Python 3.9 安装目录。pybind11 要求版本精确匹配。

### Q: `#undef slots` 编译错误？
pybind11 头文件必须在 `#undef slots` 之后包含（见 `libs/python/python_bindings.cpp`），因为 Qt 的 `slots` 宏与 Python `object.h` 冲突。

### Q: 项目重命名说明？
项目已从 CloudTool2 重命名为 **PointWorks**，可执行文件为 `pointworks.exe`。旧的 `cloudtool/`、`core/`、`modules/`、`widgets/`、`python/` 目录已重组为 `libs/` + `src/` 两层结构。
