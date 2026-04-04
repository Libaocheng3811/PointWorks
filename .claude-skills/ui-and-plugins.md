# UI 组件、工具与插件系统

## CustomDialog (widgets/customdialog.h)

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

## CloudTree (widgets/cloudtree.h)

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

## 通用对话框

| 对话框 | 文件 | 功能 |
|--------|------|------|
| ProcessingDialog | common_ui/processingdialog.h | 进度条 + 取消按钮 |
| GlobalShiftDialog | common_ui/globalshiftdialog.h | 全局偏移设置 + 记忆 |
| FieldMappingDialog | common_ui/fieldmappingdialog.h | TXT 字段映射 |
| TxtImportDialog | common_ui/txtimportdialog.h | TXT 导入配置 |
| TxtExportDialog | common_ui/txtexportdialog.h | TXT 导出配置 |

## 工具列表

| 工具 | 文件 | 功能 |
|------|------|------|
| Filters | cloudtool/tool/filters.h | 滤波器类型选择、参数配置、预览/应用 |
| Registration | cloudtool/tool/registration.h | 配准参数配置、结果显示、对应关系可视化 |
| Keypoints | cloudtool/tool/keypoints.h | 关键点检测 |
| Cutting | cloudtool/tool/cutting.h | 包围盒裁剪、多边形裁剪 |
| PickPoints | cloudtool/tool/pickpoints.h | 单点拾取、多边形区域选择 |
| RangeImage | cloudtool/tool/rangeimage.h | 深度图、边界提取、法线估计 |
| Descriptor | cloudtool/tool/descriptor.h | 描述符工具 |
| Color | cloudtool/edit/color.h | 颜色设置 |
| BoundingBox | cloudtool/edit/boundingbox.h | 包围盒绘制 |

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

### 已有插件

| 插件 | 文件 | 功能 |
|------|------|------|
| CSFPlugin | cloudtool/plugins/csfplugin.h | 地面点分割 |
| VegPlugin | cloudtool/plugins/vegplugin.h | 植被分割（4 种植被指数 + Otsu） |
| ChangeDetectPlugin | cloudtool/plugins/changedetectplugin.h | 变化检测（多种距离方法 + Jet 色带） |

## 相机 SDK 支持

| SDK | 目录 | 说明 |
|-----|------|------|
| Azure Kinect | camera/AzureKinect/ | 深度相机，头文件 `include/k4a/k4a.h` |
| Photoneo | camera/Photoneo/ | 3D 扫描仪，头文件 `include/` |
