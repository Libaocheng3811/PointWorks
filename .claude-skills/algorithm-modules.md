# 算法模块与多线程规范

所有算法位于 `modules/` 目录。核心原则：**耗时算法严禁在主线程执行**，必须通过 `QThread` 或 `QtConcurrent::run` 放入后台。

## PCL 开发规范

1. **智能指针**: 严禁裸指针。所有 PCL 对象必须使用其自带的 `Ptr` 类型
2. **内存释放**: 算法执行完毕后，确保临时点云变量的指针被正确重置或随作用域销毁
3. **PCL 转换限制**: 完整的 `toPCL()`/`fromPCL()` 在大点云下可能导致内存问题，需选择性使用

## Filters (modules/filters.h)

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

## Features (modules/features.h)

支持的特征描述符：

| 描述符 | 全称 |
|--------|------|
| PFH | Point Feature Histogram |
| FPFH | Fast Point Feature Histograms |
| VFH | Viewpoint Feature Histogram |
| SHOT | Signature of Histograms of OrienTations |
| ESF | Ensemble of Shape Functions |
| GASD | Globally Aligned Spatial Distribution |

## Registration (modules/registration.h)

配准算法：

| 算法 | 说明 |
|------|------|
| ICP | 迭代最近点算法 |
| ICPWithNormals | 带法线的 ICP |
| ICPNonLinear | 非线性 ICP |
| IA-RANSAC | SampleConsensusInitialAlignment |
| SCPR | SampleConsensusPrerejective |
| FPCS | FPCSInitialAlignment |
| KFPCS | KFPCSInitialAlignment |
| NDT | Normal Distributions Transform |

## Keypoints (modules/keypoints.h)

关键点检测算法：

| 算法 | 说明 |
|------|------|
| ISS | Intrinsic Shape Signatures |
| Harris3D | Harris 3D 关键点 |
| SIFT3D | SIFT 3D |
| NARF | Normal Aligned Radial Features |

## CSFFilter (modules/csffilter.h)

布料模拟地面滤波器。

**参数**:
- `bSloopSmooth` - 坡度平滑
- `time_step` - 时间步长
- `class_threshold` - 分类阈值
- `cloth_resolution` - 布料分辨率
- `rigidness` - 布料硬度 (1-3)
- `iterations` - 迭代次数

## VegFilter (modules/vegfilter.h)

植被分割滤波器。

**植被指数类型**:
```cpp
enum class VegIndexType {
    ExG_ExR,  // Excess Green - Excess Red
    ExG,      // Excess Green: 2g - r - b
    NGRDI,    // (g - r) / (g + r)
    CIVE      // 0.441r - 0.811g + 0.385b + 18.787
};
```

**阈值策略**:
- 用户手动设置
- Otsu 自动阈值（大津法）

## DistanceCalculator (modules/distancecalculator.h)

点云距离计算（变化检测）。

**距离计算方法**:
```cpp
enum Method {
    C2C_NEAREST = 0,      // 最近邻距离
    C2C_KNN_MEAN = 1,     // K 近邻平均距离
    C2C_RADIUS_MEAN = 2,  // 半径内平均距离
    C2M_SIGNED = 3,       // 点到网格有符号距离（预留）
    M3C2 = 4              // M3C2 算法（预留）
};
```
