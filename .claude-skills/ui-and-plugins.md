# UI 组件、工具与插件系统

## CustomDialog (libs/ui/base/customdialog.h)

对话框基类。

**特性**:
- 依赖注入模式（通过构造函数注入 CloudView、CloudTree、Console）
- 自动位置管理（工具浮窗跟随 CloudView）
- 单例管理（`registed_dialogs` 全局注册表）

**使用模式**:
```cpp
// 创建工具浮窗（无边框、跟随视图）
createDialog<Filters>(parent, "Filters", cloudview, cloudtree, console,
                      true, false);

// 创建模态对话框（阻塞、居中）
createModalDialog<GlobalShiftDialog>(parent, "Global Shift", ...);
```

## CloudTree (libs/ui/base/cloudtree.h)

点云树管理控件。

**核心功能**:
- 点云添加/删除/保存
- 节点克隆/合并
- 右键菜单管理
- 进度条绑定

**数据结构**:
```cpp
class CloudTree {
    QMap<QTreeWidgetItem*, Cloud::Ptr> m_cloud_map;  // Item -> Cloud 映射
    QThread m_thread;                                 // 文件 I/O 线程
    FileIO* m_fileio;                                 // 文件 I/O 实例
    ProcessingDialog* m_processing_dialog;            // 进度对话框
};
```

## 通用对话框 (libs/ui/dialog/)

| 对话框 | 文件 | 功能 |
|--------|------|------|
| ProcessingDialog | processingdialog.h | 进度条 + 取消按钮 |
| GlobalShiftDialog | globalshiftdialog.h/cpp/ui | 全局偏移设置 + 记忆 |
| FieldMappingDialog | fieldmappingdialog.h | TXT 字段映射 |
| TxtImportDialog | txtimportdialog.h | TXT 导入配置 |
| TxtExportDialog | txtexportdialog.h | TXT 导出配置 |

## 工具列表 (src/tool/)

| 工具 | 文件 | 功能 |
|------|------|------|
| Filters | src/tool/filters.h | 滤波器类型选择、参数配置、预览/应用 |
| Cutting | src/tool/cutting.h | 包围盒裁剪、多边形裁剪 |
| PickPoints | src/tool/pickpoints.h | 单点拾取、多边形区域选择 |
| RangeImage | src/tool/rangeimage.h | 深度图、边界提取、法线估计 |
| Segmentation | src/tool/segmentation.h | 点云分割 |
| Surface | src/tool/surface.h | 曲面重建 |
| Boundary | src/tool/boundary.h | 边界提取 |
| Sampling | src/tool/sampling.h | 点云采样 |
| AlignByCenters | src/tool/align_by_centers.h | 中心对齐 |
| GlobalRegistration | src/tool/global_registration.h | 全局配准 |
| FineRegistration | src/tool/fine_registration.h | 精配准 |
| PointPairsAlignment | src/tool/point_pairs_alignment.h | 点对配准 |

## 编辑工具列表 (src/edit/)

| 工具 | 文件 | 功能 |
|------|------|------|
| Color | src/edit/color.h | 颜色设置 |
| BoundingBox | src/edit/boundingbox.h | 包围盒绘制 |
| Transformation | src/edit/transformation.h | 点云变换（平移/旋转/矩阵） |
| Normals | src/edit/normals.h | 法线编辑 |
| Scale | src/edit/scale.h | 尺度缩放 |
| Coordinate | src/edit/coordinate.h | 坐标系操作 |

## 插件系统

### 插件模板

所有插件遵循统一模板：

```cpp
class Plugin : public ct::CustomDialog {
    QThread m_thread;           // 工作线程
    Worker* m_worker;           // 算法实例
    ct::Cloud::Ptr m_cloud;     // 输入点云

    void init() override;       // 设置 UI 连接
    void onApply();             // 开始处理
    void onDone(Result);        // 处理完成
};
```

**工作线程模式**: 耗时操作在 `QThread` 中运行：
- `progress(int percent)` 信号用于进度跟踪
- 原子标志 `m_is_canceled` 用于协作式取消
- 模态对话框 `ProcessingDialog` 提供用户反馈

### 已有插件 (src/plugins/)

| 插件 | 文件 | 功能 |
|------|------|------|
| CSFPlugin | src/plugins/csfplugin.h | 地面点分割 |
| VegPlugin | src/plugins/vegplugin.h | 植被分割（4 种植被指数 + Otsu） |
| ChangeDetectPlugin | src/plugins/changedetectplugin.h | 变化检测（多种距离方法 + Jet 色带） |
