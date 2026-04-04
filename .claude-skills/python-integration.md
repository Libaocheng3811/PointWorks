# 嵌入式 Python 集成

CloudTool2 通过 pybind11 embed 模式嵌入 Python 3.9 解释器，允许用户通过 Python 脚本访问和操作点云数据。

## 解释器生命周期

```cpp
// cloudtool/main.cpp
int main() {
    QApplication app;
    ct::PythonManager::instance().initialize();  // Py_Initialize + 注册模块
    MainWindow w;
    w.show();
    int ret = app.exec();
    ct::PythonManager::instance().finalize();     // 清理（不调用 Py_Finalize）
    return ret;
}
```

**关键约束**: 不调用 `Py_Finalize()`，避免 pybind11 全局析构顺序问题。

## 架构三要素

| 组件 | 职责 |
|------|------|
| `PythonManager` | 单例，管理解释器初始化/销毁、stdio 重定向、DLL 搜索路径 |
| `PythonWorker` | QThread，GIL 管理的脚本执行（`PyGILState_Ensure/Release`），支持异步取消 |
| `PythonBridge` | 信号桥接 + 线程安全云注册表。Python 侧只发信号，不直接操作 UI |

## 线程安全规则

1. **GIL**: Python 代码只在 `PythonWorker::run()` 中持有 GIL 执行；主线程释放 GIL
2. **Capsule 生命周期**: 零拷贝 NumPy view 通过 `py::capsule` 持有 `Cloud::Ptr`，防止点云被提前销毁
3. **信号-only UI**: Python 侧所有 UI 操作通过 `PythonBridge` 发射信号，由 Qt 主线程处理

## Python API (`import ct`)

```python
import ct

# 日志输出到 GUI Console
ct.printI("info message")
ct.printW("warning")
ct.printE("error")

# 获取点云（线程安全，自动 hold + mark in-use）
cloud = ct.get_cloud("my_cloud")  # 返回 ct.Cloud 或 None

# 按 Block 零拷贝访问（高性能，适用于大点云）
for i in range(cloud.num_blocks()):
    xyz = cloud.block_to_numpy(i)       # shape (M, 3), float32 零拷贝
    colors = cloud.block_get_colors(i)  # shape (M, 3), uint8 零拷贝

    # 修改后写回
    cloud.block_set_colors(i, new_colors)  # 拷贝写入
    cloud.block_set_numpy(i, new_xyz)      # 拷贝写入
    cloud.block_mark_dirty(i)              # 重算包围盒
    cloud.refresh()                        # 触发 VTK 重绘

# 全量拷贝（合并所有 Block）
xyz_all = cloud.to_numpy()        # shape (N, 3), float32
colors_all = cloud.get_colors()   # shape (N, 3), uint8

# 属性查询
cloud.size()          # 总点数
cloud.has_colors()    # 是否有颜色
cloud.has_normals()   # 是否有法线
```

## In-use 保护机制

```cpp
// Python 获取云时自动标记 in-use，防止 CloudTree 删除正在使用的云
bridge->holdCloud(cloud);           // 持有 shared_ptr 引用
bridge->markCloudInUse(cloud->id()); // 标记 UI 侧禁止删除

// Python 脚本执行完成后自动释放
bridge->releaseAllHeld();           // 释放引用
bridge->releaseAllInUse();          // 取消删除保护
```

## 构建配置

- **ct_python** 为 OBJECT 库（非 STATIC/SHARED），编译产物直接链接到 cloudtool 可执行文件
- pybind11 通过 `3rdparty/pybind11` git submodule 引入
- Python 路径硬编码为 `MY_NATIVE_PYTHON_DIR`，需根据本地环境修改
- `#undef slots` 必须在包含 pybind11 头文件之前，解决 Qt `slots` 宏与 Python `object.h` 的冲突
