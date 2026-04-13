# Python 接口补全 + mainwindow.cpp 瘦身 + bindings 拆分方案

## 需求重述

1. **补充缺失的 Python 接口** — 所有算法模块完整绑定
2. **将 Python 相关代码从 mainwindow.cpp 抽离** — 341 行 Bridge 信号连接拆到独立文件
3. **python_bindings.cpp 按功能分类拆分** — 当前 1452 行，拆分为多个文件

---

## 关键决策（已确认）

| 决策 | 结论 |
|------|------|
| Surface Mesh 返回类型 | 返回 `ct.Mesh` 对象，支持后续操作（不可见，纯数据） |
| Segmentation 绑定范围 | 绑定全部 15 个 API |
| python_bindings.cpp 拆分 | 按功能分类拆分为多个文件 |

---

## Phase 1：mainwindow.cpp 瘦身

### 新增文件
```
libs/python/python_connections.h    # 声明连接函数
libs/python/python_connections.cpp  # 实现所有 Bridge → UI 信号连接（~341 行）
```

### 接口设计
```cpp
// python_connections.h
namespace ct {
void connectPythonSignals(
    PythonBridge* bridge,
    CloudView* cloudview,
    CloudTree* cloudtree,
    Console* console,
    QTabWidget* consoleTabWidget,
    QAction* actionShowConsole,
    QDockWidget* consoleDock);
}
```

### mainwindow.cpp 改动
- 删除 lines 320-660 全部 Bridge 信号连接
- 替换为：`ct::connectPythonSignals(bridge, ui->cloudview, ui->cloudtree, ...);`
- Python Console/Editor 的 UI 信号（lines 260-296）保留在 mainwindow.cpp（属于 UI 菜单逻辑）

### CMake 改动
- `libs/python/CMakeLists.txt` 的 OBJECT 库源文件列表新增 `python_connections.cpp`

---

## Phase 2：python_bindings.cpp 按功能拆分

### 当前文件结构
```
libs/python/python_bindings.cpp   # 1452 行，全部绑在一起
```

### 拆分后文件结构
```
libs/python/
├── bindings/
│   ├── bind_core.h            # PyCloud 类定义 + 模块入口（CT 模块注册）
│   ├── bind_core.cpp          # Cloud 数据操作、标量场、add_cloud/get_cloud (~400 行)
│   ├── bind_view.h/.cpp       # 视图控制、相机、显示开关 (~200 行)
│   ├── bind_appearance.h/.cpp # 颜色/透明度/背景/点大小/可见性 (~150 行)
│   ├── bind_overlay.h/.cpp    # 叠加物（cube/arrow/polygon/shape 操作）(~200 行)
│   ├── bind_cloud_mgmt.h/.cpp # 点云管理（load/save/remove/clone/merge/select）(~250 行)
│   ├── bind_progress.h/.cpp   # 进度条、脚本模式、日志 (~80 行)
│   ├── bind_filters.h/.cpp    # 滤波算法 (~150 行)
│   ├── bind_segmentation.h/.cpp # 分割算法（15 个 API）(~300 行)
│   ├── bind_surface.h/.cpp    # 曲面重建 + ct.Mesh 类型定义 (~250 行)
│   ├── bind_normals.h/.cpp    # 法线估计 (~50 行)
│   ├── bind_keypoints.h/.cpp  # 关键点检测 (~100 行)
│   ├── bind_features.h/.cpp   # 特征提取 + LRF (~200 行)
│   ├── bind_registration.h/.cpp # 配准算法（已有 7 个 + 新增）(~250 行)
│   ├── bind_distance.h/.cpp   # 距离计算（新版）(~100 行)
│   └── bind_csf_veg.h/.cpp    # CSF + 植被过滤 (~100 行)
└── python_bindings.cpp        # 仅保留 PYBIND11_EMBEDDED_MODULE 入口，include 各 bind_*.h
```

### 模块入口设计
```cpp
// python_bindings.cpp（精简后）
#include "python_manager.h"
#include "python_bridge.h"
#undef slots
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// 注册所有子模块
#include "bindings/bind_core.h"
#include "bindings/bind_view.h"
#include "bindings/bind_appearance.h"
// ... 其余 bind_*.h

PYBIND11_EMBEDDED_MODULE(ct, m) {
    registerCoreBindings(m);
    registerViewBindings(m);
    registerAppearanceBindings(m);
    registerOverlayBindings(m);
    registerCloudMgmtBindings(m);
    registerProgressBindings(m);
    registerFilterBindings(m);
    registerSegmentationBindings(m);
    registerSurfaceBindings(m);
    registerNormalBindings(m);
    registerKeypointBindings(m);
    registerFeatureBindings(m);
    registerRegistrationBindings(m);
    registerDistanceBindings(m);
    registerCsfVegBindings(m);
}
```

### 每个 bind_*.h 的模式
```cpp
// bind_segmentation.h
#pragma once
#include <pybind11/pybind11.h>

namespace py = pybind11;
namespace ct {

// 在 CT 模块上注册所有分割相关函数
void registerSegmentationBindings(py::module_& m);

} // namespace ct

// bind_segmentation.cpp 中实现
```

### 共享依赖
- `bind_core.h` 中定义 `PyCloud` 类，其他模块如需返回 Cloud 相关对象则依赖 core
- `bind_surface.h` 中定义 `PyMesh` 类
- Bridge 指针获取：各模块通过 `PythonManager::instance().bridge()` 获取

---

## Phase 3：补充 Python 算法绑定

### 3.1 Normals（1 个 API）
```python
ct.estimate_normals(name, k_search=30, radius_search=0.0,
                    vpx=0, vpy=0, vpz=0, reverse=False) -> ct.Cloud
```

### 3.2 Segmentation（15 个 API）
```python
ct.sac_segmentation(name, model="plane", method="RANSAC", threshold=0.01,
                    max_iterations=1000, probability=0.99, optimize=True,
                    negative=False, min_radius=0, max_radius=0)
ct.sac_segmentation_from_normals(name, model="plane", method="RANSAC",
                                  threshold=0.01, distance_weight=0.1, ...)
ct.euclidean_cluster(name, tolerance=0.02, min_size=100, max_size=100000, negative=False)
ct.dbscan_cluster(name, eps=0.03, min_pts=10, min_size=100, max_size=100000,
                  normal_weight=0.0, color_weight=0.0, negative=False)
ct.kmeans_cluster(name, k=5, max_iterations=100, normal_weight=0.0, color_weight=0.0)
ct.region_growing(name, min_size=100, max_size=100000, smooth_mode=True,
                  curvature_test=True, residual_test=False,
                  smoothness_threshold=7.0, curvature_threshold=1.0,
                  residual_threshold=0.05, neighbours=30, negative=False)
ct.region_growing_from_seed(name, seed_index=0, min_size=100, max_size=100000, ...)
ct.region_growing_rgb(name, pt_thresh=6.0, re_thresh=5.0, dis_thresh=3000.0,
                      nghbr_number=30, min_size=100, max_size=100000, negative=False)
ct.supervoxel(name, voxel_resolution=0.01, seed_resolution=0.1,
              color_importance=0.1, spatial_importance=0.4, normal_importance=1.0)
ct.don_segmentation(name, mean_radius=0.01, scale1=0.01, scale2=0.1,
                    threshold=0.05, segradius=0.05,
                    min_cluster_size=100, max_cluster_size=100000, negative=False)
ct.min_cut_segmentation(name, sigma=0.25, radius=0.25, weight=0.8, neighbour_number=14)
ct.morphological_filter(name, max_window_size=33, slope=1.0, max_distance=0.3,
                        initial_distance=0.3, cell_size=1.0, base=2.0, negative=False)
ct.seeded_hue_segmentation(name, tolerance=0.01, delta_hue=20.0, negative=False)
ct.segment_differences(name, target_name, sqr_threshold=0.001, negative=False)
ct.extract_polygonal_prism_data(name, hull_min=0, hull_max=100, height_min=0,
                                 height_max=10, vpx=0, vpy=0, vpz=0, negative=False)
```

### 3.3 Surface（5 个 API）+ ct.Mesh 类型
```python
# 新增 ct.Mesh 类型（不可见数据对象）
mesh = ct.poisson(name, depth=8, min_depth=5, point_weight=4.0)
mesh = ct.greedy_triangulation(name, mu=2.5, nnn=12, radius=0.025)
mesh = ct.marching_cubes_hoppe(name, iso_level=0.0, res_x=50, res_y=50, res_z=50)
mesh = ct.convex_hull(name)
mesh = ct.concave_hull(name, alpha=0.1)

# ct.Mesh 对象方法
mesh.vertices()      -> np.ndarray (N, 3) float32
mesh.faces()         -> np.ndarray (M, 3) int32
mesh.num_vertices()  -> int
mesh.num_faces()     -> int
mesh.save(filepath, binary=True)  # 保存为 PLY/OBJ
```

### 3.4 Distance（3 个 API，替代废弃 API）
```python
ct.cloud_cloud_distance(ref_name, comp_name, method="hausdorff",
                        k_knn=1, max_distance=1.0, flip_normals=False)
ct.cloud_mesh_distance(cloud_name, mesh, method="hausdorff")
ct.closest_point_set(source_name, target_name, max_distance=1.0, num_closest=1)
```

### 3.5 Keypoints（3 个高频 API）
```python
ct.iss_keypoints(name, resolution=0.1, gamma_21=0.975, gamma_32=0.975)
ct.harris_keypoints(name, threshold=0.001, radius=0.01, k=10)
ct.sift_keypoints(name, min_scale=0.01, nr_octaves=6, min_contrast=0.01)
```

### 3.6 Features（精选 5 个高频 + 3 个 LRF）
```python
ct.fpfh(name, k=30, radius=0.05)
ct.shot(name, radius=0.05)
ct.shot_color(name, radius=0.05)
ct.boundary_estimation(name, k=30, radius=0.05, angle=30.0)
ct.shot_lrf(name, radius=0.05)
```

---

## 实施顺序

| 步骤 | 内容 | 涉及文件 |
|------|------|---------|
| 1 | Phase 1: 新建 python_connections.h/cpp + mainwindow 瘦身 | +2 new, mainwindow.cpp, CMakeLists.txt |
| 2 | Phase 2: python_bindings.cpp 拆分为 bindings/ 目录 | +16 new, python_bindings.cpp(精简) |
| 3 | Phase 3.1: Normals 绑定 | bind_normals.h/cpp |
| 4 | Phase 3.2: Segmentation 绑定（15 个） | bind_segmentation.h/cpp |
| 5 | Phase 3.3: Surface 绑定 + ct.Mesh | bind_surface.h/cpp |
| 6 | Phase 3.4: Distance 绑定 | bind_distance.h/cpp |
| 7 | Phase 3.5: Keypoints 绑定 | bind_keypoints.h/cpp |
| 8 | Phase 3.6: Features 绑定 | bind_features.h/cpp |

---

## 风险评估

| 风险 | 级别 | 缓解措施 |
|------|------|---------|
| Lambda 捕获 `this` → 控件指针 | LOW | 行为等价，逐行迁移 |
| ct.Mesh 类型设计 | MEDIUM | 参考 PyCloud 模式，capsule 持有 PolygonMesh::Ptr |
| python_bindings.cpp 拆分后编译顺序 | LOW | OBJECT 库内部链接，无跨文件依赖问题 |
| Segment API 参数过多 | LOW | Python 侧提供默认值，高频参数前置 |
| Distance C2M 需要 mesh 输入 | LOW | 使用 Phase 3.3 的 ct.Mesh 对象 |
