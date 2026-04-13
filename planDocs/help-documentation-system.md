# PointWorks 帮助文档系统方案

## 需求概述

为 PointWorks 软件构建一套完整的帮助文档系统，包括：

1. **软件使用教程** — 覆盖所有菜单功能的图文教程
2. **Python API 参考手册** — 完整的 `ct` 模块接口文档（~108 个函数 + 2 个类）
3. **About 对话框** — 软件版本与版权信息
4. **Help 菜单入口** — 点击即在默认浏览器中打开对应网页

## 当前状态

| 项目 | 状态 |
|------|------|
| Help 菜单 (`menuHelp`) | UI 存在但无信号连接（`action_Help`、`actionAbout` 均未实现） |
| 帮助图标 | 已有 `help-contents-symbolic.svg`、`help-info-symbolic.svg` |
| docs/ 目录 | 仅有内部设计文档，无用户文档 |
| Python API 文档 | 无任何文档 |

## 方案设计

### 架构：静态 HTML + Qt 资源嵌入 + 系统浏览器打开

```
Help 菜单
 ├── 使用教程  → 打开 docs/tutorial/index.html
 ├── Python API  → 打开 docs/api/index.html
 ├── 快捷键     → 打开 docs/shortcuts.html
 ├── About      → 弹出 QDialog
```

**为什么选择静态 HTML 而非 QWebEngineView 内嵌？**
- QWebEngineView 依赖 Chromium，增加 ~50MB 体积
- 静态 HTML 零额外依赖，打包体积小
- 用户可在浏览器中自由搜索、收藏、多标签页
- 方便后续独立部署到线上

### Help 菜单重构

修改 `mainwindow.ui` 中的 Help 菜单结构：

```
Help (menuHelp)
 ├── User Guide          (actionHelpTutorial)     icon: help-contents-symbolic.svg
 ├── Python API Reference (actionHelpApi)          icon: help-contents-symbolic.svg
 ├── Keyboard Shortcuts   (actionHelpShortcuts)     icon: help-whatsthis.svg
 ├── ---
 ├── About PointWorks     (actionAbout)            icon: help-info-symbolic.svg
```

### 文档结构

```
docs/
├── help/                          # 帮助文档根目录
│   ├── css/
│   │   └── style.css              # 统一样式（暗色主题 + 响应式）
│   ├── tutorial/
│   │   ├── index.html             # 使用教程首页
│   │   ├── getting-started.html   # 快速入门
│   │   ├── file-operations.html   # 文件操作（打开/保存/项目）
│   │   ├── view-control.html      # 视图控制（相机/主题/显示开关）
│   │   ├── edit-tools.html        # 编辑工具（颜色/变换/法线/缩放/坐标系）
│   │   ├── filters.html           # 滤波工具
│   │   ├── registration.html      # 点云配准
│   │   ├── segmentation.html      # 点云分割
│   │   ├── surface.html           # 曲面重建
│   │   ├── distance.html          # 距离计算与变化检测
│   │   ├── plugins.html           # 插件（CSF/植被/变化检测）
│   │   ├── python-scripting.html  # Python 脚本入门
│   │   └── shortcuts.html         # 快捷键参考
│   ├── api/
│   │   ├── index.html             # API 首页（模块总览）
│   │   ├── cloud.html             # ct.Cloud 类
│   │   ├── mesh.html              # ct.Mesh 类
│   │   ├── core.html              # 核心函数（日志/点云生命周期）
│   │   ├── view.html              # 视图控制
│   │   ├── appearance.html        # 外观设置
│   │   ├── overlay.html           # 叠加物
│   │   ├── cloud-mgmt.html        # 点云管理
│   │   ├── progress.html          # 进度/脚本模式
│   │   ├── filters.html           # 滤波算法
│   │   ├── segmentation.html      # 分割算法
│   │   ├── surface.html           # 曲面重建
│   │   ├── registration.html      # 配准算法
│   │   ├── features.html          # 特征/描述子
│   │   ├── distance.html          # 距离计算
│   │   ├── normals.html           # 法线估计
│   │   ├── keypoints.html         # 关键点检测
│   │   ├── csf-veg.html           # 地面/植被分割
│   │   └── examples.html          # Python 脚本示例合集
│   └── about.html                 # 关于页面（浏览器版备份）
└── ... (现有设计文档)
```

### 实现步骤

#### Phase 1: C++ 基础设施（HelpLauncher）

**1.1 创建 `src/app/help_launcher.h/cpp`**

```cpp
namespace ct {

class HelpLauncher : public QObject {
    Q_OBJECT
public:
    // 从 Qt 资源中提取 HTML 到临时目录，再用系统浏览器打开
    static void openPage(const QString& relativePath);
    static void showAboutDialog(QWidget* parent);
};

}
```

核心逻辑：
- `openPage()`: 从 `:/res/help/` 资源提取到 `%TEMP%/PointWorks/help/` 缓存目录，然后 `QDesktopServices::openUrl(QUrl::fromLocalFile(path))`
- 首次提取后缓存到临时目录，后续直接打开（通过文件时间戳判断是否需要更新）
- `showAboutDialog()`: 弹出简洁的 About 对话框，显示版本号、Logo、版权信息

**1.2 修改 `mainwindow.cpp`**

- 连接 `action_Help` → `HelpLauncher::openPage("tutorial/index.html")`
- 新增 `actionHelpApi` → `HelpLauncher::openPage("api/index.html")`
- 新增 `actionHelpShortcuts` → `HelpLauncher::openPage("tutorial/shortcuts.html")`
- 连接 `actionAbout` → `HelpLauncher::showAboutDialog(this)`

**1.3 修改 `mainwindow.ui`**

在 Help 菜单中增加：
- `User Guide` action（使用现有 `action_Help`，改文本）
- `Python API Reference` action（新增 `actionHelpApi`）
- `Keyboard Shortcuts` action（复用已有 `actionShortCutKey`，放入 Help 菜单）
- Separator
- `About` action（使用现有 `actionAbout`，改文本为 "About PointWorks"）

**1.4 修改 `src/resources/res.qrc`**

将 `docs/help/` 目录下的所有 HTML/CSS 文件加入 Qt 资源系统。

#### Phase 2: 样式系统

**2.1 创建 `docs/help/css/style.css`**

设计一套简洁、专业的文档样式：
- 配色与 PointWorks 暗色主题呼应
- 响应式布局（桌面 + 平板）
- 代码块语法高亮（纯 CSS，无需 JS）
- 侧边导航栏
- 搜索框（纯前端，使用 list.js 或无 JS 的 Ctrl+F）

#### Phase 3: 使用教程文档（~12 个 HTML 文件）

按照 Help 菜单的功能分区，每个功能区一个页面：

| 文件 | 内容 |
|------|------|
| `getting-started.html` | 安装、界面总览、基本工作流 |
| `file-operations.html` | 打开/保存点云、项目管理、最近文件 |
| `view-control.html` | 相机控制（预设视角/重置/缩放）、主题切换、显示开关 |
| `edit-tools.html` | 颜色编辑、包围盒、变换（平移/旋转/矩阵）、法线、缩放、坐标系 |
| `filters.html` | 各种滤波器说明与参数 |
| `registration.html` | 配准流程（粗配准 → 精配准）、各算法说明 |
| `segmentation.html` | 聚类、区域生长、超体素、形态学滤波等 |
| `surface.html` | 曲面重建算法、包围盒提取 |
| `distance.html` | 距离计算、变化检测 |
| `plugins.html` | CSF 地面分割、植被分割、变化检测插件 |
| `python-scripting.html` | Python 控制台/编辑器使用、脚本入门 |
| `shortcuts.html` | 完整快捷键列表 |

#### Phase 4: Python API 参考文档（~17 个 HTML 文件）

每个绑定模块一个页面，采用统一的 API 文档模板：

```
函数签名
──────────
简要描述

参数表
──────────
| 参数 | 类型 | 默认值 | 说明 |

返回值
──────────
类型 + 说明

示例代码
──────────
python 代码块
```

需覆盖的 API（共 ~108 个函数 + 2 个类）：

| 文件 | API 数量 |
|------|----------|
| `cloud.html` | ct.Cloud 类（22 个方法） |
| `mesh.html` | ct.Mesh 类（4 个方法） |
| `core.html` | printI/W/E, get_cloud, add_cloud, insert_cloud, remove_selected_clouds |
| `view.html` | refresh_view, reset_camera, zoom_to_*, set_*_view (11个) |
| `appearance.html` | set_point_size, set_opacity, set_cloud_color, 等 (12个) |
| `overlay.html` | add_cube, add_arrow, add_polygon, set_shape_* (14个) |
| `cloud-mgmt.html` | update_cloud, load_cloud, save_cloud, merge_clouds 等 (9个) |
| `progress.html` | show/set/close_progress, set_script_mode (4个) |
| `filters.html` | 13 个滤波函数 |
| `segmentation.html` | 15 个分割函数 |
| `surface.html` | 5 个曲面重建函数 |
| `registration.html` | 7 个配准函数 |
| `features.html` | bounding_box, fpfh, shot, boundary_estimation 等 (7个) |
| `distance.html` | cloud_cloud_distance, closest_point_set 等 (4个) |
| `normals.html` | estimate_normals (1个) |
| `keypoints.html` | iss/harris/sift_keypoints (3个) |
| `csf-veg.html` | csf_filter, veg_filter (2个) |
| `examples.html` | 综合示例脚本合集 |

#### Phase 5: About 对话框

创建简洁的 About 对话框：
- 左侧：Logo（使用已有 `:/res/logo/PointWorks.svg`）
- 右侧：软件名称、版本号、版权、联系方式
- 底部：Qt/PCL/VTK 开源库致谢

## 风险评估

| 风险 | 等级 | 应对 |
|------|------|------|
| Qt 资源文件过大导致编译慢 | 中 | HTML 文件总量预计 < 500KB，影响可忽略 |
| 临时目录提取权限问题 | 低 | 使用 `QStandardPaths::writableLocation(QStandardPaths::TempLocation)` |
| 文档与代码不同步 | 中 | API 文档从 pybind11 docstring 自动生成（未来可考虑） |
| 中文编码问题 | 低 | HTML 统一使用 UTF-8 编码 |

## 复杂度估算

| 阶段 | 工作量 |
|------|--------|
| Phase 1: C++ 基础设施 | 小（~3 个文件） |
| Phase 2: 样式系统 | 中 |
| Phase 3: 使用教程（12 页） | 大（内容量大） |
| Phase 4: Python API（17 页） | 大（108 个函数文档） |
| Phase 5: About 对话框 | 小 |

## 实施建议

1. **优先级**：Phase 1 → Phase 5 → Phase 2 → Phase 4 → Phase 3
   - 先让菜单能打开网页，再做内容
   - About 对话框最简单，可以快速完成
   - API 文档结构化强，可批量生成
   - 使用教程需要截图和详细说明，工作量最大

2. **可并行**：Phase 2 和 Phase 4 可以同时进行（样式和内容分离）

3. **后续扩展**：
   - 可增加"检查更新"功能
   - 可将文档部署到 GitHub Pages 实现在线版本
   - 可考虑从 pybind11 docstring 自动提取 API 签名
