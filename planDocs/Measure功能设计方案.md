# Measure 测量功能设计方案

## 需求重述

为 PointWorks (CloudTool2) 的 Tool 菜单添加 **Measure（测量）** 功能，用于在 3D 点云视图中交互式测量距离。需要参考 CloudCompare、MeshLab 等主流点云软件的测量工具设计。

**已有基础**:
- `mainwindow.ui` 中已有 `actionMeasure`（Tool 菜单项）和图标 `:/res/icon/measure.svg`
- `mainwindow.cpp` 中尚未连接该 action
- PickPoints 工具的 PICK_PAIR 模式已实现了基础的两点距离测量，但功能有限（仅单次测量、无结果管理）
- CloudView 已提供 `singlePick()`、`addArrow()`、`add3DBadge()`、`showInfo()` 等交互 API

---

## 功能设计

### 支持的测量类型

| 类型 | ID | 描述 | 优先级 |
|------|----|------|--------|
| **Point-to-Point** | `MEASURE_P2P` | 两点间欧氏距离，显示距离 + XYZ 分量 | P0 |
| **Point-to-Point (Multi)** | `MEASURE_MULTI` | 多段连续测距（折线），显示总长 + 各段长 | P1 |
| **Angle** | `MEASURE_ANGLE` | 三点定角（三点形成两条射线），显示角度值 | P1 |

> 后续可扩展：Point-to-Surface（需 mesh）、Radius（需密度计算）等

### 交互流程（Point-to-Point 模式）

```
1. 用户在点云树选中一个点云
2. 打开 Measure 工具浮窗 → 选择测量类型（默认 P2P）
3. 点击 "Start" → 进入测量模式（状态栏提示 "Pick Start Point..."）
4. 左键点击 → singlePick 获取 3D 坐标 → 红色球标记起点
5. 鼠标移动 → 实时预览绿色箭头 + 距离数值（addArrow + showInfo）
6. 左键再次点击 → 确认终点 → 显示最终箭头 + 3D 标签 + 结果记录到列表
7. 自动进入下一次测量（无需重新点 Start）
8. 右键 → 取消当前测量
9. 点击 "Stop" → 退出测量模式（保留所有已完成的测量标注）
10. 点击 "Clear All" → 删除所有测量结果
```

### UI 布局（浮动工具面板）

```
┌──────────────────────────┐
│  Measure           [X]   │
├──────────────────────────┤
│  Type: [Point-to-Point ▼] │
├──────────────────────────┤
│  [Start/Stop]  [Clear All]│
├──────────────────────────┤
│  ┌────────────────────┐  │
│  │ #  Distance  ΔX/ΔY/ΔZ│  │
│  │ 1  12.345   5/3/11  │  │
│  │ 2  8.912    2/7/4   │  │
│  │ ...                 │  │
│  └────────────────────┘  │
│  [Delete Selected] [Export]│
├──────────────────────────┤
│  ☐ Show Labels            │
│  ☐ Show Arrows            │
│  Precision: [3]           │
└──────────────────────────┘
```

### 3D 标注显示

- **端点**: 红色小球（复用 addPointCloud 或 vtkSphereSource）
- **连线**: 绿色箭头（`addArrow`，带距离标签）
- **距离数值**: 3D Billboard 标签（`add3DBadge`，始终面向相机，不被点云遮挡）
- **预览**: 鼠标悬停时显示虚线/半透明箭头 + 实时距离

### 结果管理

- **列表显示**: TableWidget 显示所有测量记录（序号、距离、XYZ分量、起点坐标、终点坐标）
- **单条删除**: 选中列表项后点击 Delete，同时移除对应的 3D 标注
- **全部清除**: Clear All 按钮
- **导出**: 导出为 CSV 文件（序号、类型、距离、ΔX、ΔY、ΔZ、起点坐标、终点坐标）

---

## 实施方案

### 文件结构

```
src/tool/measure.h       — Measure 类声明
src/tool/measure.cpp     — Measure 类实现
src/tool/measure.ui      — Qt Designer 界面文件
```

### 类设计

```cpp
class Measure : public ct::CustomDialog {
    Q_OBJECT

public:
    explicit Measure(QWidget *parent = nullptr);
    ~Measure() override;

    void init() override;
    void reset() override;
    void deinit() override;

private slots:
    void onStartStop();                          // 开始/停止测量
    void onClearAll();                           // 清除所有测量
    void onDeleteSelected();                     // 删除选中测量
    void onExport();                             // 导出 CSV
    void onTypeChanged(int index);              // 切换测量类型
    void onPrecisionChanged(int decimals);       // 精度变化时刷新显示
    void onShowLabelsChanged(int state);         // 标签显示开关
    void onShowArrowsChanged(int state);         // 箭头显示开关

    // CloudView 鼠标事件
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

    // 列表选中变化
    void onMeasurementSelected(int row, int col);

private:
    Ui::Measure *ui;
    bool m_measuring = false;
    bool m_first_point_set = false;

    // 测量类型枚举
    enum MeasureType {
        TYPE_P2P = 0,       // 两点距离
        TYPE_MULTI = 1,     // 多段折线
        TYPE_ANGLE = 2      // 角度
    };

    // 单条测量结果
    struct Measurement {
        int id;                              // 序号
        MeasureType type;                    // 测量类型
        ct::PointXYZRGBN start_point;       // 起点
        ct::PointXYZRGBN end_point;         // 终点（P2P）
        QVector<ct::PointXYZRGBN> points;   // 多段点集（Multi/Angle）
        float distance;                      // 距离/总长
        float angle;                         // 角度（仅 Angle 类型）
    };

    QVector<Measurement> m_measurements;
    int m_next_id = 1;
    ct::Cloud::Ptr m_selected_cloud;
    ct::Cloud::Ptr m_marker_cloud;           // 临时端点标记点云
    int m_precision = 3;                     // 显示精度

    // ID 生成
    QString arrowId(int measId) const { return QString("meas_arrow_%1").arg(measId); }
    QString labelId(int measId) const { return QString("meas_label_%1").arg(measId); }
    QString markerId(int measId) const { return QString("meas_marker_%1").arg(measId); }

    // 内部方法
    void pickFirstPoint(const ct::PointXY& screenPt);
    void pickSecondPoint(const ct::PointXY& screenPt);
    void updatePreview(const ct::PointXY& screenPt);
    void addMeasurementToTable(const Measurement& m);
    void removeMeasurementFromScene(int measId);
    void refreshAllDisplay();
    void updateInfoText();
};
```

### 实施步骤

#### Phase 1: 基础框架 + Point-to-Point 测量

1. **创建 `measure.ui`** — 浮动面板布局
   - 类型选择 QComboBox（Point-to-Point / Multi-segment / Angle）
   - Start/Stop 按钮 + Clear All 按钮
   - QTableWidget 测量结果列表
   - Delete Selected + Export CSV 按钮
   - 显示选项：Show Labels / Show Arrows 复选框、Precision SpinBox

2. **创建 `measure.h` + `measure.cpp`** — 核心逻辑
   - 继承 `ct::CustomDialog`，遵循 PickPoints 的交互模式
   - Start/Stop 切换测量模式（connect/disconnect 鼠标信号）
   - P2P 模式：左键选第一点 → 预览箭头 → 左键选第二点 → 确认测量
   - 右键取消当前测量
   - 使用唯一 ID 管理 3D 标注（箭头、标签）
   - 测量完成后自动进入下一次测量

3. **3D 标注渲染**
   - 端点：使用临时 Cloud 对象 + addPointCloud（红色小球）
   - 箭头：`addArrow(end, start, arrowId, true, Color::Green)`
   - 距离标签：`add3DBadge(midpoint, distance_text, labelId, ...)`
   - 预览箭头用半透明/不同颜色区分

4. **结果列表管理**
   - QTableWidget 列：#、Distance、ΔX、ΔY、ΔZ
   - 单条删除：从 QVector 移除 + 移除对应 3D 标注
   - 全部清除

5. **导出 CSV**
   - QFileDialog 选择保存路径
   - 写入 CSV：序号、距离、ΔX、ΔY、ΔZ、起点坐标、终点坐标

6. **连接 MainWindow**
   - `mainwindow.cpp` 添加 `#include "tool/measure.h"`
   - 连接 `ui->actionMeasure` → `createDialog<Measure>("Measure")`
   - `src/CMakeLists.txt` 添加 `measure.h/cpp/ui`

#### Phase 2: Multi-segment 多段测距

- 连续点击多个点形成折线
- 每段显示距离，汇总显示总长
- 右键结束当前折线测量
- 使用独立的 arrowId：`meas_arrow_1_seg0`, `meas_arrow_1_seg1` 等

#### Phase 3: Angle 角度测量

- 三个点定义角度：顶点 + 两个方向点
- 计算两向量夹角：`acos(dot(v1, v2) / (|v1| * |v2|))`
- 在顶点处绘制弧线 + 角度标签

---

## 关键依赖

| 依赖 | 来源 | 状态 |
|------|------|------|
| `ct::CustomDialog` | `libs/ui/base/customdialog.h` | 已有 |
| `CloudView::singlePick()` | `libs/viz/cloudview.h` | 已有 |
| `CloudView::addArrow()` | `libs/viz/cloudview.h` | 已有 |
| `CloudView::add3DBadge()` | `libs/viz/cloudview.h` | 已有 |
| `CloudView::removeShape()` | `libs/viz/cloudview.h` | 已有 |
| `CloudView::showInfo()` | `libs/viz/cloudview.h` | 已有 |
| `CloudTree::getSelectedClouds()` | `libs/ui/base/cloudtree.h` | 已有 |
| `actionMeasure` | `mainwindow.ui` | 已有（未连接） |
| `measure.svg` 图标 | `src/resources/icon/measure.svg` | 已有 |

## 风险评估

| 风险 | 级别 | 缓解方案 |
|------|------|----------|
| `addArrow` 的 display_length 与 `add3DBadge` 标签重叠 | 低 | Arrow 仅画线段，用 Badge 显示距离数值 |
| PickPoints 的 PICK_PAIR 模式与 Measure 功能冲突 | 低 | 使用 `createDialog` 的单例机制，同时只能打开一个 |
| 大量测量标注影响渲染性能 | 低 | 限制最大测量数量（如 100），或使用 LOD 标签 |
| Marker 点云 ID 唯一性 | 低 | 使用 `measure_marker_{measId}` 命名规则 |

## 预计复杂度

- **Phase 1（P2P + 结果管理）**: 中等 — 核心功能，参考 PickPoints 实现模式
- **Phase 2（Multi-segment）**: 低 — P2P 的扩展
- **Phase 3（Angle）**: 低 — 独立的计算逻辑

---

**等待确认**: 是否按照此方案实施？如有修改意见请告知。
