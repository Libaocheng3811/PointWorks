# 算法模块与多线程规范

所有算法位于 `libs/algorithm/` 目录，编译为 `ct_algorithm` 静态库。核心原则：**耗时算法严禁在主线程执行**，必须通过 `QThread` 或 `QtConcurrent::run` 放入后台。

## 重要约束

- **零 VTK 依赖**: `libs/algorithm/` 中无任何 `#include <vtk...>`，CMakeLists 中无 `${VTK_LIBRARIES}` 链接
- **渲染解耦**: 曲面重建的 VTK 渲染预处理由 `libs/viz/surface_viz_helper.cpp` 负责，算法层仅返回纯 PCL 数据

## 异步执行规范

工具/插件应使用 `ProgressManager::runAsync()` 统一执行异步任务：

```cpp
m_progress->runAsync("任务名称",
    [=]() -> ResultType {
        // 后台线程：执行耗时算法
        return Algorithm::compute(params);
    },
    [=](ResultType result) {
        // 主线程：处理结果、更新 UI
        m_cloudtree->addResultGroup(...);
    }
);
```

**旧方式（已废弃）**: 手动创建 `QAtomicInt` + `QtConcurrent::run` + `QFutureWatcher` + `ProcessingDialog`。

## PCL 开发规范

1. **智能指针**: 严禁裸指针。所有 PCL 对象必须使用其自带的 `Ptr` 类型
2. **内存释放**: 算法执行完毕后，确保临时点云变量的指针被正确重置或随作用域销毁
3. **PCL 转换限制**: 完整的 `toPCL()`/`fromPCL()` 在大点云下可能导致内存问题，需选择性使用

## Filters (libs/algorithm/filters.h)

支持的滤波器：

| 滤波器 | 说明 |
|--------|------|
| PassThrough | 直通滤波（阈值裁剪） |
| VoxelGrid | 体素降采样 |
| StatisticalOutlierRemoval | 统计离群点移除 |
| RadiusOutlierRemoval | 半径离群点移除 |
| ConditionalRemoval | 条件滤波 |
| GridMinimum | 2D 网格最小值投影 |
| LocalMaximum | 局部最大值移除 |

## Features (libs/algorithm/features.h)

支持的描述符：PFH, FPFH, VFH, SHOT, ESF, GASD

## Registration (libs/algorithm/registration.h)

配准算法：ICP, ICPWithNormals, ICPNonLinear, IA-RANSAC, SCPR, FPCS, KFPCS, NDT

## Keypoints (libs/algorithm/keypoints.h)

关键点检测：ISS, Harris3D, SIFT3D, NARF

## Normals (libs/algorithm/normals.h)

法线估计模块。

## Segmentation (libs/algorithm/segmentation.h)

点云分割：区域生长、平面分割、欧式聚类、RANSAC

## Surface (libs/algorithm/surface.h)

曲面重建模块。**仅含纯 PCL 数据，不含 VTK 类型**。

```cpp
struct SurfaceResult {
    pcl::PolygonMesh mesh;
    // 渲染预处理由 SurfaceVizHelper（libs/viz/）负责
};
```

## CSFFilter (libs/algorithm/csffilter.h)

布料模拟地面滤波器。参数：bSloopSmooth, time_step, class_threshold, cloth_resolution, rigidness, iterations

## VegFilter (libs/algorithm/vegfilter.h)

植被分割滤波器。支持 ExG_ExR, ExG, NGRDI, CIVE 四种植被指数。

## DistanceCalculator (libs/algorithm/distancecalculator.h)

点云距离计算（变化检测）。支持 C2C、C2M、C2P、CPS 等距离方法。参数类型定义在 `libs/core/field_types.h`。

## utils.h (libs/algorithm/utils.h)

算法工具函数（头文件），提供通用辅助功能。
