# RangeImage 重构实施方案

## 1. 现状分析

### 当前实现

- **UI**: `src/tool/rangeimage.ui` — 极简布局，仅一个 QVTKOpenGLWidget + 一个 CheckBox
- **逻辑**: `src/tool/rangeimage.cpp` (107行) — 监听 `CloudView::viewerPose` 信号，实时生成深度图
- **算法**: `ct::RangeImage` 即 `pcl::RangeImage`，`createFromPointCloud` 投影点云
- **显示**: VTK `vtkImageActor` 渲染在对话框内嵌 VTK 窗口中
- **创建方式**: `createDialog<RangeImage>` — 非模态 ToolDialog（停靠在 DataDock 下方）

### 当前问题

| # | 问题 | 严重程度 | 说明 |
|---|------|---------|------|
| 1 | **无关闭按钮** | 高 | 窗口没有 Close/Cancel 按钮，只能通过面板 tab 的关闭按钮关闭 |
| 2 | **窗口太小，无法缩放** | 高 | 固定 224x244，深度图看不清，不支持放大缩小 |
| 3 | **无导出功能** | 中 | 无法保存深度图为图片文件 |
| 4 | **参数硬编码** | 中 | 角分辨率 0.5°、水平 FOV 360°、垂直 FOV 180° 全部硬编码，用户无法修改 |
| 5 | **实时计算卡顿** | 高 | 每次相机移动都重新计算，大数据量(>100万点)时明显卡顿 |
| 6 | **CheckBox 文本无意义** | 低 | 文本显示 "CheckBox"，应改为 "Show Point Cloud" |
| 7 | **代码风格不统一** | 低 | 使用 .ui 文件 + Ui 命名空间，与重构后的其他对话框（纯代码 setupUi）风格不一致 |

## 2. 重构目标

1. 增加关闭按钮
2. 支持鼠标滚轮/按钮缩放查看深度图
3. 支持导出深度图为 PNG 图片
4. 参数可调（角分辨率、FOV）
5. 加防抖/降采样优化性能
6. 统一代码风格（纯代码 setupUi，移除 .ui 文件）
7. 修正 UI 文案

## 3. 架构设计

### 3.1 重构方式

**保留 ToolDialog 模式**（非模态停靠面板），因为 RangeImage 需要实时跟随主视图相机角度变化，必须与主视图共存。

### 3.2 缩放方案

**移除 QVTKOpenGLWidget，改用 QLabel + QPixmap 显示深度图。**

原因：
- QVTKOpenGLWidget 内嵌 VTK 渲染器，用于显示 2D 图像过于重量级
- QLabel + QPixmap 原生支持 `scaled()`、鼠标滚轮缩放、拖拽平移
- 实现简单，性能好，天然支持高 DPI

### 3.3 防抖方案

相机移动时 **不立即计算**，而是启动一个 200ms 的 QTimer，如果 200ms 内相机再次移动则重置计时器，只有相机静止 200ms 后才触发深度图计算。

### 3.4 文件变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `src/tool/rangeimage.h` | 重写 | 纯代码 setupUi，新成员变量 |
| `src/tool/rangeimage.cpp` | 重写 | 完整重构 |
| `src/tool/rangeimage.ui` | 删除 | 不再使用 .ui 文件 |
| `src/CMakeLists.txt` | 修改 | 移除 `tool/rangeimage.ui` |

## 4. 详细设计

### 4.1 rangeimage.h

```cpp
class RangeImage : public ct::CustomDialog {
    Q_OBJECT
public:
    explicit RangeImage(QWidget* parent = nullptr);
    ~RangeImage() override;
    void init() override;
    void reset() override;

private slots:
    void onViewerPoseChanged(Eigen::Affine3f pose);
    void onParamChanged();
    void onExportImage();
    void onTogglePointCloud(bool checked);

protected:
    void wheelEvent(QWheelEvent* event) override;   // 滚轮缩放
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void setupUi();
    void updateDepthImage();
    void fitToView();
    QPixmap generateDepthPixmap();

    // --- UI 控件 ---
    QLabel* image_label_;                          // 深度图显示
    QDoubleSpinBox* dspin_angular_res_;            // 角分辨率
    QDoubleSpinBox* dspin_h_fov_;                  // 水平FOV
    QDoubleSpinBox* dspin_v_fov_;                  // 垂直FOV
    QPushButton* btn_export_;
    QPushButton* btn_close_;
    QCheckBox* check_show_cloud_;

    // --- 数据 ---
    ct::RangeImage::Ptr m_range_image;
    ct::Cloud::Ptr m_cloud;
    Eigen::Affine3f m_viewer_pose;
    QTimer* m_debounce_timer_;                     // 防抖定时器

    // --- 缩放/平移 ---
    QPixmap m_pixmap;
    qreal m_scale_factor_ = 1.0;
    QPoint m_last_mouse_pos_;
    bool m_dragging_ = false;
};
```

### 4.2 UI 布局

```
+------------------------------------------+
| RangeImage                               |
+------------------------------------------+
| Parameters:                               |
|   Angular Res: [0.5] deg                  |
|   H-FOV: [360.0] deg  V-FOV: [180.0] deg |
+------------------------------------------+
|                                          |
|           深度图显示区域                   |
|          (QLabel + QPixmap)              |
|         支持滚轮缩放/拖拽平移              |
|                                          |
+------------------------------------------+
| [x] Show Point Cloud                     |
+------------------------------------------+
| [弹簧] [Export] [弹簧] [Close] [弹簧]    |
+------------------------------------------+
```

### 4.3 深度图生成流程

```
viewerPose 信号触发
    ↓
重置 m_debounce_timer (200ms)
    ↓
200ms 无新信号 → updateDepthImage()
    ↓
读取当前参数 (角分辨率, FOV)
    ↓
pcl::RangeImage::createFromPointCloud(...)
    ↓
FloatImageUtils::getVisualImage(...) → RGB 数组
    ↓
QImage(rgba_data, width, height, ...) → QPixmap
    ↓
按当前 m_scale_factor_ 缩放 → 设置到 image_label_
```

### 4.4 缩放实现

```cpp
void RangeImage::wheelEvent(QWheelEvent* event) {
    // 滚轮上 = 放大，下 = 缩小
    qreal factor = (event->angleDelta().y() > 0) ? 1.15 : 1.0 / 1.15;
    m_scale_factor_ *= factor;
    m_scale_factor_ = qBound(0.1, m_scale_factor_, 20.0); // 限制范围

    image_label_->setPixmap(m_pixmap.scaled(
        m_pixmap.size() * m_scale_factor_,
        Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

// 拖拽平移: 在 scrollArea 内嵌 image_label_，或手动调整 label 位置
```

### 4.5 导出实现

```cpp
void RangeImage::onExportImage() {
    if (m_pixmap.isNull()) {
        printW("No range image to export.");
        return;
    }
    QString path = QFileDialog::getSaveFileName(
        this, "Export Range Image",
        m_cloud ? QString::fromStdString(m_cloud->id()) + "_range.png" : "range.png",
        "PNG (*.png);;JPEG (*.jpg);;BMP (*.bmp)");

    if (!path.isEmpty()) {
        m_pixmap.save(path);
        printI(QString("Range image exported to: %1").arg(path));
    }
}
```

## 5. 实施步骤

### 步骤 1：重写 rangeimage.h

- 移除 `Ui::RangeImage* ui` 和 VTK 相关成员
- 添加新的 UI 控件成员和缩放状态成员
- 添加 wheelEvent / mouseEvent 声明

### 步骤 2：重写 rangeimage.cpp

**setupUi():**
- 顶部：参数面板（角分辨率 spinbox、H-FOV spinbox、V-FOV spinbox）
- 中间：QScrollArea > QLabel 显示深度图
- 底部：ShowPointCloud 复选框 + Export/Close 按钮

**init():**
- 连接 `CloudView::viewerPose` → `onViewerPoseChanged`
- 创建防抖定时器 m_debounce_timer (200ms, 单次触发)

**updateDepthImage():**
- 使用当前参数调用 `pcl::RangeImage::createFromPointCloud`
- `FloatImageUtils::getVisualImage` 生成 RGB
- 构建 QImage → QPixmap，按 scale_factor 缩放显示

**缩放/平移:**
- wheelEvent: 调整 scale_factor，重新缩放 pixmap
- mousePressEvent/Move/Release: 拖拽平移（通过 QScrollArea 的 scrollBars）

**onExportImage():**
- QFileDialog 选择保存路径
- m_pixmap.save(path)

**reset():**
- 断开 viewerPose 信号
- 清空深度图显示
- 移除主视图中的点云叠加

### 步骤 3：删除 rangeimage.ui，更新 CMakeLists.txt

- 删除 `src/tool/rangeimage.ui`
- 从 `src/CMakeLists.txt` 的 `set(UIS ...)` 中移除 `tool/rangeimage.ui`

### 步骤 4：编译验证

## 6. 注意事项

- **QImage 数据格式**: `FloatImageUtils::getVisualImage` 返回的是 RGB（3通道），构建 QImage 时需用 `QImage::Format_RGB888`，且需要手动深拷贝数据（因为原数据会被下一帧覆盖）
- **防抖定时器**: 必须是单次触发（`setSingleShot(true)`），每次新信号到来先 `stop()` 再 `start()`
- **VTK 头文件依赖**: 移除 `vtkImageData.h`、`vtkImageActor.h`、`vtkRenderer.h`、`vtkSmartPointer.h` 等不再需要的包含
- **Close 按钮行为**: 调用 `this->close()` 关闭停靠面板（不是 `reject()`，因为这是非模态对话框）
- **mainwindow.cpp 无需修改**: `createDialog<RangeImage>` 调用方式不变
