# 嵌入式 Python 集成

PointWorks 通过 pybind11 embed 模式嵌入 Python 3.9 解释器，允许用户通过 Python 脚本访问和操作点云数据。

## API 设计策略：两层 API

PointWorks 采用 **两层 API + 管道模式**，兼顾便捷性和灵活性：

```
┌──────────────────────────────────────────────┐
│  cloud.xxx()  —  面向对象便捷 API（Layer 1） │
│  日常使用，10 行以内搞定                      │
│  参数有合理默认值，入口简单                   │
├──────────────────────────────────────────────┤
│  ct.xxx()  —  模块级精细 API（Layer 2）      │
│  高级场景，完全控制每个参数                   │
│  与 C++ 算法 1:1 对应，功能无损失            │
└──────────────────────────────────────────────┘
```

- **Layer 1**：`ct.Cloud` 上的便捷方法，是对 Layer 2 的薄封装
- **Layer 2**：模块级函数，保留完整参数控制
- **两者共存**，用户自由选择，不隐藏低层 API

## 解释器生命周期

```cpp
// src/app/main.cpp
int main() {
    QApplication app;

    // 从 QSettings 读取用户自定义 Python 路径（优先级高于嵌入式 Python）
    QSettings settings("PointWorks", "PointWorks");
    QString customPy = settings.value("python_home").toString();
    if (!customPy.isEmpty()) {
        pw::PythonManager::instance().setCustomPythonHome(customPy.toStdString());
    }

    pw::PythonManager::instance().initialize();  // Py_Initialize + 注册模块
    MainWindow w;
    w.show();
    int ret = app.exec();
    pw::PythonManager::instance().finalize();     // 清理（不调用 Py_Finalize）
    return ret;
}
```

**关键约束**: 不调用 `Py_Finalize()`，避免 pybind11 全局析构顺序问题。

## 架构三要素

| 组件 | 文件 | 职责 |
|------|------|------|
| `PythonManager` | `libs/python/python_manager.h` | 单例，管理解释器初始化/销毁、stdio 重定向、DLL 搜索路径 |
| `PythonWorker` | `libs/python/python_worker.h` | QThread，GIL 管理的脚本执行（`PyGILState_Ensure/Release`），支持异步取消 |
| `PythonBridge` | `libs/python/python_bridge.h` | 信号桥接 + 线程安全云注册表。Python 侧只发信号，不直接操作 UI |

## Python UI 组件

| 组件 | 文件 | 职责 |
|------|------|------|
| PythonConsole | `src/python/python_console.h` | Python 交互式控制台 UI |
| PythonEditor | `src/python/python_editor.h` | Python 脚本编辑器 UI |

## 线程安全规则

1. **GIL**: Python 代码只在 `PythonWorker::run()` 中持有 GIL 执行；主线程释放 GIL
2. **Capsule 生命周期**: 零拷贝 NumPy view 通过 `py::capsule` 持有 `Cloud::Ptr`，防止点云被提前销毁
3. **信号-only UI**: Python 侧所有 UI 操作通过 `PythonBridge` 发射信号，由 Qt 主线程处理

## Python API (`import ct`)

### 日志输出

```python
ct.printI("info message")
ct.printW("warning")
ct.printE("error")
```

### 点云管理（Layer 2 — 模块级函数）

```python
cloud = ct.get_cloud("my_cloud")              # 获取点云 → ct.Cloud 或 None
ct.add_cloud("name", xyz, colors=None)         # 从 numpy 创建
ct.insert_cloud(cloud)                          # 插入到场景
ct.remove_selected_clouds()
ct.remove_cloud("id")
ct.remove_all_clouds()
ct.clone_cloud("id")                            # 克隆
ct.merge_clouds(["id1", "id2"])                 # 合并
ct.select_cloud("id")
ct.load_cloud("path.las")                       # 异步加载
ct.save_cloud("id", "out.ply", binary=True)     # 异步保存
ct.update_cloud("id", xyz, colors=None)         # 就地更新
ct.get_all_cloud_names() -> list[str]
```

### ct.Cloud 类（Layer 1 — 便捷方法）

```python
cloud = ct.get_cloud("my_cloud")

# 属性
cloud.size()                  # 总点数
cloud.name()                  # 名称
cloud.set_name("new_name")    # 设置名称
cloud.center()                # [x, y, z]
cloud.bounding_box()          # {cx, cy, cz, width, height, depth}
cloud.resolution()            # 点分辨率
cloud.volume()                # 包围盒体积
cloud.filepath()              # 源文件路径
cloud.has_colors()            # 是否有颜色
cloud.has_normals()           # 是否有法线
cloud.num_blocks()            # 数据块数量
cloud.block_size(i)           # 第 i 块点数

# 数据访问
xyz = cloud.to_numpy()        # shape (N, 3), float32 全量拷贝
colors = cloud.get_colors()   # shape (N, 3), uint8 全量拷贝
for i in range(cloud.num_blocks()):
    xyz = cloud.block_to_numpy(i)       # 零拷贝
    colors = cloud.block_get_colors(i)  # 零拷贝
    cloud.block_set_colors(i, new_colors)
    cloud.block_set_numpy(i, new_xyz)
    cloud.block_mark_dirty(i)
cloud.refresh()                        # 触发重绘

# 标量场
cloud.add_scalar_field("heights", data)
cloud.get_scalar_field("heights")
cloud.update_color_by_field("heights")
cloud.get_scalar_field_names()
cloud.remove_scalar_field("heights")
cloud.has_scalar_field("heights")   # 是否存在
cloud.clear_scalar_fields()          # 清除所有标量场

# ===== 变换（Layer 1 便捷方法） =====
new_cloud = cloud.translate(1.0, 2.0, 3.0)
new_cloud = cloud.rotate(0, 0, 45)         # 欧拉角，度
new_cloud = cloud.rotate_axis(45, 0, 0, 1) # 轴角
new_cloud = cloud.scale(2.0, 2.0, 2.0)
new_cloud = cloud.apply_matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

# ===== 滤波（Layer 1 便捷方法） =====
new_cloud = cloud.voxel_down_sample(0.5, 0.5, 0.5)
new_cloud = cloud.remove_outliers(30, 2.0)
new_cloud = cloud.crop_by_box(min_x, min_y, min_z, max_x, max_y, max_z)

# ===== 法线 =====
new_cloud = cloud.estimate_normals(k=30, radius=0.0)

# ===== 特征描述子 =====
result = cloud.fpfh(k=30, radius=0.05)      # {'descriptor': ndarray, 'time_ms': float}
result = cloud.shot(radius=0.05)
new_cloud = cloud.boundary_estimation(k=30, radius=0.05)

# ===== 曲面重建 =====
mesh = cloud.convex_hull()
mesh = cloud.concave_hull(alpha=0.1)
mesh = cloud.poisson(depth=8)

# ===== 关键点 =====
new_cloud = cloud.iss_keypoints(resolution=0.1)

# ===== 克隆 =====
new_cloud = cloud.clone()

# ===== 显式显示 =====
cloud.show("name")              # 将此点云添加到文件树和视图（可选设置名称）
ct.add_to_scene(cloud, "name")  # 等价的模块级函数
```

### 变换（Layer 2 — 精细控制）

```python
ct.translate("name", tx, ty, tz) -> ct.Cloud
ct.rotate("name", rx, ry, rz)              # 欧拉角，度
ct.rotate_axis("name", angle, ax, ay, az,  # 轴角 + 可选平移
               tx=0, ty=0, tz=0) -> ct.Cloud
ct.scale("name", sx, sy, sz, use_center=False) -> ct.Cloud
ct.apply_matrix("name", matrix)            # 4x4 list[list[float]]
ct.crop_by_box("name", min_x, min_y, min_z, max_x, max_y, max_z, negative=False)
```

### 滤波（Layer 2）

```python
ct.voxel_grid(name, lx, ly, lz, negative=False) -> ct.Cloud
ct.approx_voxel_grid(name, lx, ly, lz, negative=False)
ct.statistical_outlier_removal(name, nr_k=30, stddev_mult=2.0, negative=False)
ct.radius_outlier_removal(name, radius=1.0, min_pts=2, negative=False)
ct.pass_through(name, field_name, limit_min, limit_max, negative=False)
ct.grid_minimum(name, resolution=1.0, negative=False)
ct.local_maximum(name, radius=1.0, negative=False)
ct.shadow_points(name, threshold=0.1, negative=False)
ct.down_sampling(name, radius=1.0, negative=False)
ct.uniform_sampling(name, radius=1.0, negative=False)
ct.random_sampling(name, sample=1000, seed=42, negative=False)
ct.resampling(name, radius=1.0, polynomial_order=2, negative=False)
ct.normal_space_sampling(name, sample=1000, seed=42, bin=10, negative=False)
ct.sampling_surface_normal(name, sample=10000, seed=42, ratio=0.5, negative=False)
```

### CSF / 植被分割

```python
ct.csf_filter(name, smooth=True, ...) -> {'ground': ct.Cloud, 'off_ground': ct.Cloud, 'time_ms': float}
ct.veg_filter(name, index_type=0, threshold=0.35) -> {'vegetation': ct.Cloud, 'non_vegetation': ct.Cloud, 'time_ms': float}
```

### 距离计算

```python
ct.cloud_cloud_distance(ref, comp, method=0, k_knn=6, radius=0.5) -> dict
ct.closest_point_set(source, target, max_distance=1.0) -> ct.Cloud
```

### 特征描述子（Layer 2 — 完整列表）

```python
# 局部描述子
ct.fpfh(name, k=30, radius=0.05, surface=False)
ct.pfh(name, k=30, radius=0.05, surface=False)
ct.shot(name, radius=0.05, surface=False)
ct.shot_color(name, radius=0.05, surface=False)

# 全局描述子
ct.vfh(name, dir_x=0, dir_y=0, dir_z=0, surface=False)
ct.esf(name)
ct.gasd(name, dir_x=0, dir_y=0, dir_z=0, shgs=5, shs=3, interp=0)
ct.gasd_color(name, dir_x=0, dir_y=0, dir_z=0, shgs=5, shs=3, interp=0, chgs=5, chs=3, cinterp=0)
ct.rsd(name, nr_subdiv=5, plane_radius=0.1)
ct.grsd(name)
ct.crh(name, dir_x=0, dir_y=0, dir_z=0)
ct.cvfh(name, dir_x=0, dir_y=0, dir_z=0, radius_normals=0.05, d1=0.02, d2=0.04, d3=0.06, min_points=50, normalize=True)
ct.shape_context_3d(name, min_radius=0.005, radius=0.05)
ct.unique_shape_context(name, lrf_radius=0.015, radius=0.025, loc_radius=0.075)

# 局部参考帧
ct.shot_lrf(name, radius=0.05)
ct.board_lrf(name, radius=0.03, find_holes=True, margin_thresh=0.001, size=0, prob_thresh=0.001, steep_thresh=0.5)
ct.flare_lrf(name, radius=0.03, margin_thresh=0.02, min_neighbors_normal=5, min_neighbors_tangent=5)

# 其他
ct.boundary_estimation(name, k=30, radius=0.05, angle=30.0) -> ct.Cloud
ct.bounding_box_aabb(name) -> dict
ct.bounding_box_obb(name) -> dict
```

### 配准

```python
ct.icp(source, target, max_iterations=50, correspondence_distance=1.0, use_reciprocal=False)
ct.icp_with_normals(source, target, ...)
ct.icp_nonlinear(source, target, ...)
ct.gicp(source, target, max_iterations=200, k=30, ...)
ct.ndt(source, target, resolution=1.0, step_size=0.1, outlier_ratio=0.05)
ct.fpcs(source, target, delta=1.0, ...)
ct.kfpcs(source, target, delta=1.0, ...)
# 返回 dict: {'aligned': ct.Cloud, 'score': float, 'matrix': list[list[float]], 'time_ms': float}
```

### 分割

```python
ct.sac_segmentation(name, model=0, method=0, threshold=1.0, ...) -> dict
ct.sac_segmentation_from_normals(name, ...) -> dict
ct.euclidean_cluster(name, tolerance=1.0, ...) -> list[ct.Cloud]
ct.dbscan_cluster(name, eps=1.0, min_pts=2, ...) -> list[ct.Cloud]
ct.kmeans_cluster(name, k=8, ...) -> list[ct.Cloud]
ct.region_growing(name, ...) -> list[ct.Cloud]
ct.region_growing_from_seed(name, ...) -> list[ct.Cloud]
ct.region_growing_rgb(name, ...) -> list[ct.Cloud]
ct.supervoxel(name, ...) -> list[ct.Cloud]
ct.don_segmentation(name, ...) -> list[ct.Cloud]
ct.min_cut_segmentation(name, ...) -> list[ct.Cloud]
ct.morphological_filter(name, ...) -> list[ct.Cloud]
ct.seeded_hue_segmentation(name, ...) -> list[ct.Cloud]
ct.segment_differences(name, target, ...) -> list[ct.Cloud]
ct.extract_polygonal_prism_data(name, hull, ...) -> list[ct.Cloud]
```

### 曲面重建

```python
ct.poisson(name, depth=8, ...) -> ct.Mesh
ct.greedy_triangulation(name, ...) -> ct.Mesh
ct.marching_cubes_hoppe(name, iso_level=0.0, ...) -> ct.Mesh
ct.marching_cubes_rbf(name, iso_level=0.0, ...) -> ct.Mesh
ct.grid_projection(name, resolution=0.001, ...) -> ct.Mesh
ct.convex_hull(name, compute_area_volume=False, dimension=3) -> ct.Mesh
ct.concave_hull(name, alpha=0.1, keep_information=False, dimension=3) -> ct.Mesh
```

### 关键点检测

```python
ct.iss_keypoints(name, resolution=0.1, ...) -> ct.Cloud
ct.harris_keypoints(name, response_method=0, ...) -> ct.Cloud
ct.sift_keypoints(name, min_scale=0.01, ...) -> ct.Cloud
ct.trajkovic_keypoints(name, compute_method=0, ...) -> ct.Cloud
```

### 法线估计

```python
ct.estimate_normals(name, k_search=30, radius_search=0.0, ...) -> ct.Cloud
```

### 视图控制

```python
ct.refresh_view()
ct.reset_camera()
ct.zoom_to_bounds()
ct.zoom_to_selected()
ct.set_top_view()
ct.set_front_view()
ct.set_back_view()
ct.set_left_side_view()
ct.set_right_side_view()
ct.set_bottom_view()
ct.set_auto_render(enable)
```

### 外观控制

```python
ct.set_point_size(id, size)
ct.set_opacity(id, value)
ct.set_cloud_color(id, r, g, b)
ct.set_color_by_axis(id, axis)
ct.reset_cloud_color(id)
ct.set_cloud_visibility(id, visible)
ct.set_background_color(r, g, b)
ct.reset_background_color()
ct.show_id(show)
ct.show_axes(show)
ct.show_fps(show)
ct.show_info(text)
ct.clear_info()
```

### 叠加物

```python
ct.add_cube(cx, cy, cz, size, id="cube")
ct.add_3d_label(text, x, y, z, id="label")
ct.add_arrow(x1, y1, z1, x2, y2, z2, r=1, g=1, b=1, id="arrow")
ct.add_polygon(cloud_id, r=1, g=1, b=1, id="polygon")
ct.remove_shape(id)
ct.remove_all_shapes()
ct.set_shape_color(id, r, g, b)
ct.set_shape_size(id, size)
ct.set_shape_opacity(id, value)
ct.set_shape_line_width(id, value)
ct.set_shape_font_size(id, value)
ct.set_shape_representation(id, type)
ct.zoom_to_bounds_xyz(min_x, min_y, min_z, max_x, max_y, max_z)
ct.invalidate_cloud_render(id)
ct.set_interactor_enable(enable)
```

### 进度管理

```python
ct.show_progress(title)
ct.set_progress(percent)
ct.close_progress()
```

### 网格显示

```python
# 显示网格到视图 + 文件树
mesh = ct.poisson("chef")
mesh.show("poisson-mesh")

# 模块级函数
ct.show_mesh(mesh, "my_mesh")
ct.remove_mesh("my_mesh")
```

### 脚本模式与数据清理

```python
# 默认模式：算法结果自动插入文件树 + 显示在视图（向后兼容）
cloud = ct.voxel_grid("my_cloud", 0.5, 0.5, 0.5)  # 自动显示并加入文件树

# 显式控制：任何 ct.Cloud 都可以手动 show 到场景
cloud2 = ct.voxel_grid("my_cloud", 0.5, 0.5, 0.5)
cloud2.show("my_voxel")          # 手动添加到文件树和视图
ct.add_to_scene(cloud2, "name")  # 等价的模块级函数

# 脚本/静默模式：结果只在内存中，不自动显示
ct.set_script_mode(True)
cloud3 = ct.voxel_grid("my_cloud", 0.5, 0.5, 0.5)  # 数据存在但不可见
# ... 自定义处理、导出等 ...
cloud3.show("filtered")  # 显式添加到场景（即使在脚本模式也可用）
cloud3.show()            # 也可以不加名称（使用自动生成的 ID）

# 清理接口
ct.clear_script_data()    # 仅清理脚本生成但未 show() 的数据
ct.clear_all()            # 清理所有 Python 生成的数据（点云+网格）
```

| 模式 | 算法结果行为 | `.show()` / `add_to_scene()` | 关闭 Python Editor |
|------|------------|---------------------------|-------------------|
| 默认 (`set_script_mode(False)`) | 自动插入文件树 + 显示 | 仍可使用（通常不需要） | 结果保留在树中 |
| 脚本模式 (`set_script_mode(True)`) | 数据在内存中，不显示 | **显式调用可显示** | 自动清理未 show 的数据 |

```python
ct.set_script_mode(enabled)   # True 开启静默模式
ct.clear_script_data()        # 仅清理未挂载到场景的脚本数据
ct.clear_all()                # 清理所有 Python 生成的数据（点云+网格）
```

**所有权规则**：
- 脚本变量 → 随脚本生命周期销毁
- 调用 `.show()` 或 `add_to_scene()` 后 → 所有权移交主程序，关闭 Python Editor 后仍保留

## In-use 保护机制

```cpp
// Python 获取云时自动标记 in-use，防止 CloudTree 删除正在使用的云
bridge->holdCloud(cloud);           // 持有 shared_ptr 引用
bridge->markCloudInUse(cloud->id()); // 标记 UI 侧禁止删除

// Python 脚本执行完成后自动释放
bridge->releaseAllHeld();           // 释放引用
bridge->releaseAllInUse();          // 取消删除保护
```

## 架构要点

Python 绑定层分为两层：
- **Qt-free 数据层**：`CloudRegistry`（`cloud_registry.h`）— 纯 `std::mutex` + `std::unordered_map`，不依赖 Qt。被算法绑定直接使用。
- **Qt 信号桥接层**：`PythonBridge`（`python_bridge.h`）— 保留 QObject 继承，内部持有 `CloudRegistry` 实例。UI 绑定通过它发射信号。

纯算法绑定（filters, normals, features, keypoints, transform, registration, segmentation, distance, csf_veg, surface）通过 `bind_common.h` 提供的 `getRegistry()` 访问 `CloudRegistry`，无需 include `python_bridge.h`。

## 绑定文件结构

```
libs/python/
├── python_bindings.cpp         # PYBIND11_EMBEDDED_MODULE(ct) 入口
├── python_manager.h/cpp        # 解释器生命周期（Qt-free 头文件）
├── python_worker.h/cpp         # QThread 脚本执行
├── python_bridge.h/cpp         # 信号桥接（内部持有 CloudRegistry）
├── cloud_registry.h/cpp        # Qt-free 线程安全云注册表
└── bindings/
    ├── bind_common.h            # 公共 include
    ├── bind_core.h/cpp          # ct.Cloud 类 + 基础函数 + Layer 1 便捷方法
    ├── bind_cloud_mgmt.h/cpp    # 云管理（加载/保存/克隆/合并）
    ├── bind_view.h/cpp          # 视图控制
    ├── bind_appearance.h/cpp    # 外观控制
    ├── bind_overlay.h/cpp       # 叠加物
    ├── bind_progress.h/cpp      # 进度管理
    ├── bind_transform.h/cpp     # 变换/缩放/裁剪（Layer 2）
    ├── bind_filters.h/cpp       # 滤波算法
    ├── bind_normals.h/cpp       # 法线估计
    ├── bind_features.h/cpp      # 特征描述子
    ├── bind_keypoints.h/cpp     # 关键点检测
    ├── bind_registration.h/cpp  # 配准算法
    ├── bind_segmentation.h/cpp  # 分割算法
    ├── bind_surface.h/cpp       # 曲面重建
    ├── bind_csf_veg.h/cpp       # CSF 地面分割 + 植被分割
    ├── bind_distance.h/cpp      # 距离计算
    ├── mesh_utils.h/cpp         # ct.Mesh 类 + 曲面重建封装
```

## 构建配置

- **pw_python** 为 OBJECT 库（非 STATIC/SHARED），编译产物直接链接到 pointworks 可执行文件
- pybind11 通过 `3rdparty/pybind11` git submodule 引入
- Python 路径通过 QSettings 动态配置（注册表键 `PointWorks/PointWorks` → `python_home`），
  也可通过 Options → Display Settings → Python 页面交互式设置
- `#undef slots` 必须在包含 pybind11 头文件之前，解决 Qt `slots` 宏与 Python `object.h` 的冲突
