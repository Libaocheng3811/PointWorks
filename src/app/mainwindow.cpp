// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include "mainwindow.h"
#include "ui_MainWindow.h"
#include "projectmanager.h"

#include "edit/boundingbox.h"
#include "edit/color.h"

#include "tool/cutting.h"
#include "tool/pickpoints.h"
#include "tool/filters.h"
#include "tool/sampling.h"
#include "tool/rangeimage.h"
#include "tool/segmentation.h"
#include "tool/surface.h"
#include "tool/boundary.h"
#include "edit/transformation.h"
#include "edit/normals.h"
#include "edit/scale.h"
#include "edit/coordinate.h"
#include "tool/global_registration.h"
#include "tool/fine_registration.h"
#include "tool/point_pairs_alignment.h"
#include "tool/align_by_centers.h"

#include "plugins/csfplugin.h"
#include "plugins/vegplugin.h"
#include "plugins/changedetectplugin.h"

#include "python/python_manager.h"
#include "python/python_bridge.h"

#include "python/python_console.h"
#include "python/python_editor.h"

#include <algorithm>

#include <QDesktopWidget>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QComboBox>
#include <QHeaderView>

#define  CLOUD_ICON_PATH    ":/res/icon/view-calendar.svg"
#define  GROUP_ICON_PATH    ":/res/icon/view-group.svg"

// setupUi()是Ui::MainWindow类的方法，负责初始化界面中的控件。
MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // resize
    this->setBaseSize(1320, 845);
    QList<QDockWidget*> docks;
    docks.push_back(ui->DataDock);
    docks.push_back(ui->PropertiesDock);
    docks.push_back(ui->ConsoleDock);
    QList<int> size;
    size.push_back(300);
    size.push_back(320);
    size.push_back(140);
    resizeDocks(docks, size, Qt::Orientation::Vertical);

    // === Python Console（按需创建，默认不添加到 tab）===
    auto* python_console = new ct::PythonConsole(nullptr);

    // 处理 tab 关闭请求：关闭单个 tab
    connect(ui->consoleTabWidget, &QTabWidget::tabCloseRequested, this, [this, python_console](int index) {
        QString tabText = ui->consoleTabWidget->tabText(index);
        ui->consoleTabWidget->removeTab(index);
        if (tabText == "Python Console") {
            ui->actionPythonConsole->setChecked(false);
        }
    });

    // connect pointer
    ui->cloudtree->setCloudView(ui->cloudview);
    ui->cloudtree->setConsole(ui->console);
    ui->cloudtree->setPropertiesTable(ui->cloudtable);
    ui->cloudtree->setFileIcon(style()->standardIcon(QStyle::SP_DirClosedIcon));
    ui->cloudtree->setCloudIcon(QIcon(CLOUD_ICON_PATH));
    ui->cloudtree->setGroupIcon(style()->standardIcon(QStyle::SP_DirOpenIcon));

    // 设置属性表格列宽 - 第一列固定，第二列拉伸
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->cloudtable->setColumnWidth(0, 120);

    // 设置属性表格列宽自适应
    // 第一列（属性名）固定宽度，第二列（值）可拉伸填充
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->cloudtable->setColumnWidth(0, 140);  // 第一列固定宽度，确保属性名显示完整

    // file
    connect(ui->actionOpen, &QAction::triggered, ui->cloudtree, &ct::CloudTree::addCloud);
    connect(ui->actionSave, &QAction::triggered, ui->cloudtree, &ct::CloudTree::saveSelectedClouds);
    connect(ui->actionClose, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeSelectedClouds);
    connect(ui->actionClose_All, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeAllClouds);
    connect(ui->actionMerge, &QAction::triggered, ui->cloudtree, &ct::CloudTree::mergeSelectedClouds);
    connect(ui->actionClone, &QAction::triggered, ui->cloudtree, &ct::CloudTree::cloneSelectedClouds);
    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);

    // edit
    connect(ui->actionBoundingBox, &QAction::triggered, [=] {this->createToolDialog<BoundingBox>("BoundingBox"); });
    connect(ui->actionColors, &QAction::triggered, [=] {this->createToolDialog<Color>("Color"); });
    connect(ui->actionTransformation, &QAction::triggered, [=] {this->createToolDialog<Transformation>("Transformation"); });
    connect(ui->actionNormals, &QAction::triggered, [=] {this->createToolDialog<Normals>("Normals"); });
    connect(ui->actionScale, &QAction::triggered, [=] {this->createDialog<Scale>("Scale"); });
    connect(ui->actionCoords, &QAction::triggered, [=] {this->createDialog<Coordinate>("Coordinate"); });

    // view
    connect(ui->actionResetcamera, &QAction::triggered, ui->cloudtree, &ct::CloudTree::zoomToSelected);
    connect(ui->actionTopView, &QAction::triggered, ui->cloudview, &ct::CloudView::setTopView);
    connect(ui->actionFrontView, &QAction::triggered, ui->cloudview, &ct::CloudView::setFrontView);
    connect(ui->actionLeftSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setLeftSideView);
    connect(ui->actionBackView, &QAction::triggered, ui->cloudview, &ct::CloudView::setBackView);
    connect(ui->actionRightSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setRightSideView);
    connect(ui->actionBottomView, &QAction::triggered, ui->cloudview, &ct::CloudView::setBottomView);
    connect(ui->actionShowID, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowId);
    connect(ui->actionShowAxes, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowAxes);
    connect(ui->actionShowFPS, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowFPS);
    ui->actionShowFPS->setChecked(true);
    ui->actionShowAxes->setChecked(true);
    ui->actionShowID->setChecked(true);
    connect(ui->actionShowDataTree, &QAction::toggled, [=](bool checked){
        ui->DataDock->setVisible(checked); });
    connect(ui->DataDock, &QDockWidget::visibilityChanged, [=](bool visible){
        if (!this->isMinimized()) ui->actionShowDataTree->setChecked(visible);
    });
    connect(ui->actionShowProperties, &QAction::toggled, [=](bool checked){
        ui->PropertiesDock->setVisible(checked); });
    connect(ui->PropertiesDock, &QDockWidget::visibilityChanged, [=](bool visible){
        if (!this->isMinimized()) ui->actionShowProperties->setChecked(visible);
    });
    connect(ui->actionShowConsole, &QAction::toggled, [=](bool checked){
        if (checked) {
            // 确保 Console tab 存在
            int idx = ui->consoleTabWidget->indexOf(ui->console);
            if (idx < 0) {
                ui->consoleTabWidget->insertTab(0, ui->console, "Report Console");
            }
            ui->ConsoleDock->show();
            ui->consoleTabWidget->setCurrentIndex(0);
        } else {
            ui->ConsoleDock->hide();
        }
    });
    connect(ui->ConsoleDock, &QDockWidget::visibilityChanged, [=](bool visible){
        if (!this->isMinimized()) ui->actionShowConsole->setChecked(visible);
    });

    // tools
    connect(ui->actionCutting, &QAction::triggered, [=] { this->createDialog<Cutting>("Cutting"); });
    connect(ui->actionPickPoints, &QAction::triggered, [=] {this->createDialog<PickPoints>("PickPoints"); });
    connect(ui->actionFilters, &QAction::triggered, [=] {this->createToolDialog<Filters>("Filters"); });
    connect(ui->actionSampling, &QAction::triggered, [=] {
        this->createModalDialog<Sampling>("Point Cloud Sampling");
    });
    connect(ui->actionRangeImage, &QAction::triggered, [=]{this->createDialog<RangeImage>("RangeImage");});
    connect(ui->actionSegmentation, &QAction::triggered, [=]
    {
        this->createToolDialog<Segmentation>("Segmentation");
    });
    connect(ui->actionSurface, &QAction::triggered, [=]
    {
        this->createToolDialog<Surface>("Surface");
    });
    connect(ui->actionBoundary, &QAction::triggered, [=]
    {
        this->createDialog<Boundary>("Boundary");
    });

    // registration submenu
    connect(ui->actionAlignByCenters, &QAction::triggered, [=] {
        this->createModalDialog<AlignByCentersDialog>("AlignByCenters");
    });
    connect(ui->actionPointPairsAlignment, &QAction::triggered, [=] {
        auto* dlg = ct::createDialog<PointPairsAlignment>(
            this, "PointPairsAlignment", ui->cloudview, ui->cloudtree, ui->console);
        if (dlg) {
            // 禁用菜单栏、工具栏、文件树和属性栏，但保留 ViewBar 用于视角调整
            menuBar()->setEnabled(false);
            for (auto* tb : findChildren<QToolBar*>())
                tb->setEnabled(false);
            ui->ViewBar->setEnabled(true);
            ui->cloudtree->setEnabled(false);
            ui->PropertiesDock->setEnabled(false);
            ui->DataDock->setEnabled(false);

            connect(dlg, &QDialog::destroyed, this, [=] {
                menuBar()->setEnabled(true);
                for (auto* tb : findChildren<QToolBar*>())
                    tb->setEnabled(true);
                ui->cloudtree->setEnabled(true);
                ui->PropertiesDock->setEnabled(true);
                ui->DataDock->setEnabled(true);
            });
        }
    });
    connect(ui->actionGlobalRegistration, &QAction::triggered, [=] {
        this->createModalDialog<GlobalRegistrationDialog>("GlobalRegistration");
    });
    connect(ui->actionFineRegistration, &QAction::triggered, [=] {
        this->createModalDialog<FineRegistrationDialog>("FineRegistration");
    });

    // plugins
    connect(ui->actionCSF, &QAction::triggered, [=] {
        this->createModalDialog<CSFPlugin>("Cloth Simulation Filter");});
    connect(ui->actionVegetation_Filters, &QAction::triggered, [=] {
        this->createModalDialog<VegPlugin>("Vegetation Filters");});
    connect(ui->actionChange_Detection, &QAction::triggered, [=] {
        this->createModalDialog<ChangeDetectPlugin>("Change Detection");});

    // === Python Console（View 菜单，可勾选，默认不打开）===
    connect(ui->actionPythonConsole, &QAction::toggled, this, [this, python_console](bool checked) {
        if (checked) {
            // 添加 Python Console tab（如果尚未添加）
            int idx = ui->consoleTabWidget->indexOf(python_console);
            if (idx < 0) {
                idx = ui->consoleTabWidget->addTab(python_console, "Python Console");
            }
            ui->ConsoleDock->show();
            ui->actionShowConsole->setChecked(true);
            ui->consoleTabWidget->setCurrentIndex(idx);
        } else {
            // 移除 Python Console tab
            int idx = ui->consoleTabWidget->indexOf(python_console);
            if (idx >= 0) {
                ui->consoleTabWidget->removeTab(idx);
            }
        }
    });

    connect(ui->actionPythonEditor, &QAction::toggled, this, [this](bool checked) {
        static ct::PythonEditor* editor = nullptr;
        if (!editor) {
            editor = new ct::PythonEditor(this);
        }
        if (checked) {
            // 基于 CloudView 全局坐标定位：宽=CloudView宽，高=CloudView一半，遮住右半侧
            QPoint tl = ui->cloudview->mapToGlobal(QPoint(0, 0));
            QSize cvSize = ui->cloudview->size();
            editor->setGeometry(tl.x() + cvSize.width() / 2, tl.y(),
                                cvSize.width() / 2, cvSize.height());
            editor->show();
            editor->activateWindow();
        } else {
            editor->hide();
        }
    });

    connect(ui->actionDark, &QAction::triggered, [=]{
        QFile styleFile(":/res/theme/darkstyle.qss");
        if (styleFile.open(QIODevice::ReadOnly)) {
            QTextStream ts(&styleFile);
            this->setStyleSheet(ts.readAll());
            styleFile.close();
        }
    });

    connect(ui->actionLight, &QAction::triggered, [=]{
        QFile styleFile(":/res/theme/lightstyle.qss");
        if (styleFile.open(QIODevice::ReadOnly)) {
            QTextStream ts(&styleFile);
            this->setStyleSheet(ts.readAll());
            styleFile.close();
        }
    });

    connect(ui->actionOrigin, &QAction::triggered, [=]{
        this->setStyleSheet("");  // 清空样式表，恢复默认
    });

    // === Python Bridge 信号连接 ===
    auto* bridge = ct::PythonManager::instance().bridge();
    if (bridge) {
        // ---- 渲染缓存 ----
        connect(bridge, &ct::PythonBridge::signalCloudChanged,
                ui->cloudview, &ct::CloudView::invalidateCloudRender,
                Qt::QueuedConnection);

        // ---- 注册表同步 ----
        connect(ui->cloudtree, &ct::CloudTree::cloudInserted,
                bridge, &ct::PythonBridge::registerCloud, Qt::QueuedConnection);
        connect(ui->cloudtree, &ct::CloudTree::removedCloudId,
                bridge, &ct::PythonBridge::unregisterCloud, Qt::QueuedConnection);

        // ---- In-use 删除保护 ----
        connect(bridge, &ct::PythonBridge::signalMarkCloudInUse,
                ui->cloudtree, &ct::CloudTree::markCloudInUse, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalUnmarkCloudInUse,
                ui->cloudtree, &ct::CloudTree::unmarkCloudInUse, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalReleaseAllInUse,
                ui->cloudtree, &ct::CloudTree::releaseAllInUse, Qt::QueuedConnection);

        // 脚本执行完毕后重置脚本模式
        connect(bridge, &ct::PythonBridge::signalReleaseAllInUse,
                ui->cloudtree, [this]() { ui->cloudtree->setScriptMode(false); },
                Qt::QueuedConnection);
    }

    // ---- 日志 → Console ----
    connect(bridge, &ct::PythonBridge::signalLog,
            ui->console, [this](int level, const QString &msg) {
                ui->console->print(static_cast<ct::log_level>(level), msg);
            }, Qt::QueuedConnection);

    // ---- 进度 → CloudTree ----
    connect(bridge, &ct::PythonBridge::signalShowProgress,
            ui->cloudtree, &ct::CloudTree::showProgress, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetProgress,
            ui->cloudtree, &ct::CloudTree::setProgress, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalCloseProgress,
            ui->cloudtree, &ct::CloudTree::closeProgress, Qt::QueuedConnection);

    // ---- 视图控制 → CloudView ----
    connect(bridge, &ct::PythonBridge::signalRefreshView,
            ui->cloudview, [this]() { ui->cloudview->refresh(); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalResetCamera,
            ui->cloudview, [this]() { ui->cloudview->resetCamera(); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalZoomToBounds,
            ui->cloudtree, &ct::CloudTree::zoomToSelected, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetAutoRender,
            ui->cloudview, [this](bool en) { ui->cloudview->setAutoRender(en); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalZoomToSelected,
            ui->cloudtree, &ct::CloudTree::zoomToSelected, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetTopView,
            ui->cloudview, [this]() { ui->cloudview->setTopView(); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetFrontView,
            ui->cloudview, [this]() { ui->cloudview->setFrontView(); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetBackView,
            ui->cloudview, [this]() { ui->cloudview->setBackView(); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetLeftSideView,
            ui->cloudview, [this]() { ui->cloudview->setLeftSideView(); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetRightSideView,
            ui->cloudview, [this]() { ui->cloudview->setRightSideView(); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetBottomView,
            ui->cloudview, [this]() { ui->cloudview->setBottomView(); },
            Qt::QueuedConnection);

    // ---- 点云外观 → CloudView ----
    connect(bridge, &ct::PythonBridge::signalSetPointSize,
            ui->cloudview, [this](const QString &id, float s) {
                ui->cloudview->setPointCloudSize(id, s);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetOpacity,
            ui->cloudview, [this](const QString &id, float v) {
                ui->cloudview->setPointCloudOpacity(id, v);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetCloudColorRGB,
            ui->cloudview, [this](const QString &id, float r, float g, float b) {
                ui->cloudview->setPointCloudColor(id, ct::RGB{
                        static_cast<uint8_t>(std::min(std::max(r * 255.f, 0.f), 255.f)),
                        static_cast<uint8_t>(std::min(std::max(g * 255.f, 0.f), 255.f)),
                        static_cast<uint8_t>(std::min(std::max(b * 255.f, 0.f), 255.f))});
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetCloudColorByAxis,
            this, [this, bridge](const QString &id, const QString &axis) {
                auto cloud = bridge->getCloud(id);
                if (cloud) ui->cloudview->setPointCloudColor(cloud, axis);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalResetCloudColor,
            this, [this, bridge](const QString &id) {
                auto cloud = bridge->getCloud(id);
                if (cloud) ui->cloudview->resetPointCloudColor(cloud);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetCloudVisibility,
            ui->cloudview, [this](const QString &id, bool v) {
                ui->cloudview->setPointCloudVisibility(id, v);
            }, Qt::QueuedConnection);

    // ---- 场景外观 → CloudView ----
    connect(bridge, &ct::PythonBridge::signalSetBackgroundColor,
            ui->cloudview, [this](float r, float g, float b) {
                ui->cloudview->setBackgroundColor(ct::RGB{
                        static_cast<uint8_t>(std::min(std::max(r * 255.f, 0.f), 255.f)),
                        static_cast<uint8_t>(std::min(std::max(g * 255.f, 0.f), 255.f)),
                        static_cast<uint8_t>(std::min(std::max(b * 255.f, 0.f), 255.f))});
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalResetBackgroundColor,
            ui->cloudview, [this]() { ui->cloudview->resetBackgroundColor(); },
            Qt::QueuedConnection);

    // ---- 显示开关 → CloudView ----
    connect(bridge, &ct::PythonBridge::signalShowId,
            ui->cloudview, [this](bool en) { ui->cloudview->setShowId(en); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalShowAxes,
            ui->cloudview, [this](bool en) { ui->cloudview->setShowAxes(en); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalShowFPS,
            ui->cloudview, [this](bool en) { ui->cloudview->setShowFPS(en); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalShowInfo,
            ui->cloudview, [this](const QString &text) {
                ui->cloudview->showInfo(text, 1);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalClearInfo,
            ui->cloudview, [this]() { ui->cloudview->clearInfo(); },
            Qt::QueuedConnection);

    // ---- 叠加物 → CloudView ----
    connect(bridge, &ct::PythonBridge::signalAddCube,
            ui->cloudview, [this](float cx, float cy, float cz, float size, const QString &id) {
                ct::Box box;
                box.translation = Eigen::Vector3f(cx, cy, cz);
                box.width = box.height = box.depth = size;
                ui->cloudview->addCube(box, id);
            }, Qt::QueuedConnection);
    // TODO: CloudView::add3DLabel 未实现，暂时注释
    // connect(bridge, &ct::PythonBridge::signalAdd3DLabel,
    //         ui->cloudview, [this](const QString& text, float x, float y, float z, const QString& id) {
    //     ct::PointXYZRGBN pos;
    //     pos.x = x; pos.y = y; pos.z = z;
    //     ui->cloudview->add3DLabel(pos, text, id);
    // }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalRemoveShape,
            ui->cloudview, [this](const QString &id) { ui->cloudview->removeShape(id); },
            Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalRemoveAllShapes,
            ui->cloudview, [this]() { ui->cloudview->removeAllShapes(); },
            Qt::QueuedConnection);

    // ---- Phase 3: 高级叠加物 + 视图控制 ----
    connect(bridge, &ct::PythonBridge::signalAddArrow,
            ui->cloudview, [this](float x1, float y1, float z1, float x2, float y2, float z2,
                                  const QString &id, float r, float g, float b) {
                ct::PointXYZRGBN pt1, pt2;
                pt1.x = x1;
                pt1.y = y1;
                pt1.z = z1;
                pt2.x = x2;
                pt2.y = y2;
                pt2.z = z2;
                ct::RGB rgb;
                rgb.r = static_cast<uint8_t>(r * 255);
                rgb.g = static_cast<uint8_t>(g * 255);
                rgb.b = static_cast<uint8_t>(b * 255);
                ui->cloudview->addArrow(pt1, pt2, id, false, rgb);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalAddPolygon,
            ui->cloudview, [this](const QString &cloud_id, const QString &id, float r, float g, float b) {
                auto *item = ui->cloudtree->getItemById(cloud_id);
                if (item) {
                    auto cloud = ui->cloudtree->getCloud(item);
                    if (cloud) {
                        ct::RGB rgb;
                        rgb.r = static_cast<uint8_t>(r * 255);
                        rgb.g = static_cast<uint8_t>(g * 255);
                        rgb.b = static_cast<uint8_t>(b * 255);
                        ui->cloudview->addPolygon(cloud, id, rgb);
                    }
                }
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetShapeColor,
            ui->cloudview, [this](const QString &id, float r, float g, float b) {
                ct::RGB rgb;
                rgb.r = static_cast<uint8_t>(r * 255);
                rgb.g = static_cast<uint8_t>(g * 255);
                rgb.b = static_cast<uint8_t>(b * 255);
                ui->cloudview->setShapeColor(id, rgb);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetShapeSize,
            ui->cloudview, [this](const QString &id, float size) {
                ui->cloudview->setShapeSize(id, size);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetShapeOpacity,
            ui->cloudview, [this](const QString &id, float value) {
                ui->cloudview->setShapeOpacity(id, value);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetShapeLineWidth,
            ui->cloudview, [this](const QString &id, float value) {
                ui->cloudview->setShapeLineWidth(id, value);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetShapeFontSize,
            ui->cloudview, [this](const QString &id, float value) {
                ui->cloudview->setShapeFontSize(id, value);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetShapeRepresentation,
            ui->cloudview, [this](const QString &id, int type) {
                ui->cloudview->setShapeRepersentation(id, type);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalZoomToBoundsXYZ,
            ui->cloudview, [this](float min_x, float min_y, float min_z,
                                  float max_x, float max_y, float max_z) {
                ui->cloudview->zoomToBounds(
                        Eigen::Vector3f(min_x, min_y, min_z),
                        Eigen::Vector3f(max_x, max_y, max_z));
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalInvalidateCloudRender,
            ui->cloudview, [this](const QString &id) {
                ui->cloudview->invalidateCloudRender(id);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSetInteractorEnable,
            ui->cloudview, [this](bool enable) {
                ui->cloudview->setInteractorEnable(enable);
            }, Qt::QueuedConnection);

    // ---- 点云管理 → CloudTree ----
    connect(bridge, &ct::PythonBridge::signalInsertCloud,
            ui->cloudtree, [this](ct::Cloud::Ptr cloud) {
                ui->cloudtree->insertCloud(cloud);
                ui->cloudview->addPointCloud(cloud);
                ui->cloudview->refresh();
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalRemoveSelectedClouds,
            ui->cloudtree, &ct::CloudTree::removeSelectedClouds, Qt::QueuedConnection);

    // ---- Phase 1: 核心 Python API 扩展 ----
    connect(bridge, &ct::PythonBridge::signalRemoveCloud,
            ui->cloudtree, [this](const QString &id) {
                auto *item = ui->cloudtree->getItemById(id);
                if (item) {
                    // Deselect all, select target, then remove
                    for (auto *sel: ui->cloudtree->selectedItems())
                        sel->setSelected(false);
                    ui->cloudtree->setCloudSelected(ui->cloudtree->getCloud(item), true);
                    ui->cloudtree->removeSelectedClouds();
                }
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalRemoveAllClouds,
            ui->cloudtree, &ct::CloudTree::removeAllClouds, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalCloneCloud,
            ui->cloudtree, [this](const QString &id) {
                auto *item = ui->cloudtree->getItemById(id);
                if (item) {
                    // Deselect all, select target, then clone
                    for (auto *sel: ui->cloudtree->selectedItems())
                        sel->setSelected(false);
                    ui->cloudtree->setCloudSelected(ui->cloudtree->getCloud(item), true);
                    ui->cloudtree->cloneSelectedClouds();
                }
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSelectCloud,
            ui->cloudtree, [this](const QString &id) {
                auto *item = ui->cloudtree->getItemById(id);
                if (item) {
                    // Deselect all currently selected items
                    for (auto *sel: ui->cloudtree->selectedItems())
                        sel->setSelected(false);
                    ui->cloudtree->setCloudSelected(ui->cloudtree->getCloud(item), true);
                }
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalLoadCloud,
            ui->cloudtree, [this](const QString &filepath) {
                ui->cloudtree->loadCloudFile(filepath);
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalSaveCloud,
            ui->cloudtree, [this](const QString &id, const QString &filepath, bool binary) {
                auto *item = ui->cloudtree->getItemById(id);
                if (item) {
                    auto cloud = ui->cloudtree->getCloud(item);
                    if (cloud) {
                        ui->cloudtree->saveCloudFile(cloud, filepath, binary);
                    }
                }
            }, Qt::QueuedConnection);
    connect(bridge, &ct::PythonBridge::signalMergeClouds,
            ui->cloudtree, [this](const QStringList &ids) {
                // Deselect all first
                for (auto *sel: ui->cloudtree->selectedItems())
                    sel->setSelected(false);
                for (const auto &id: ids) {
                    auto *item = ui->cloudtree->getItemById(id);
                    if (item) {
                        ui->cloudtree->setCloudSelected(ui->cloudtree->getCloud(item), true);
                    }
                }
                ui->cloudtree->mergeSelectedClouds();
            }, Qt::QueuedConnection);

    // ---- Phase 2: 算法结果组 ----
    connect(bridge, &ct::PythonBridge::signalAddResultGroup,
            ui->cloudtree,
            [this](const QString &origin_id, const std::vector<ct::Cloud::Ptr> &results, const QString &group_name) {
                auto *origin_item = ui->cloudtree->getItemById(origin_id);
                ct::Cloud::Ptr origin_cloud = origin_item ? ui->cloudtree->getCloud(origin_item) : nullptr;
                if (origin_cloud && !results.empty()) {
                    ui->cloudtree->addResultGroup(origin_cloud, results, group_name);
                    for (const auto &r: results)
                        ui->cloudview->addPointCloud(r);
                    ui->cloudview->refresh();
                }
            }, Qt::QueuedConnection);

    // ---- Phase 3: 就地更新点云 ----
    connect(bridge, &ct::PythonBridge::signalUpdateCloud,
            ui->cloudtree, [this](const QString &id, const ct::Cloud::Ptr &new_cloud) {
                auto *item = ui->cloudtree->getItemById(id);
                if (item) {
                    auto old_cloud = ui->cloudtree->getCloud(item);
                    if (old_cloud) {
                        ui->cloudtree->updateCloud(old_cloud, new_cloud, false);
                        ui->cloudview->refresh();
                    }
                }
            }, Qt::QueuedConnection);

    // ---- 脚本模式：跳过弹窗 ----
    connect(bridge, &ct::PythonBridge::signalSetScriptMode,
            ui->cloudtree, &ct::CloudTree::setScriptMode, Qt::QueuedConnection);

    // 脚本执行完毕后自动重置脚本模式
    connect(bridge, &ct::PythonBridge::signalReleaseAllInUse,
            ui->cloudtree, [this]() { ui->cloudtree->setScriptMode(false); },
            Qt::QueuedConnection);

    // === 项目管理（自包含控制器） ===
    m_project_manager = new ProjectManager(ui->cloudtree, ui->cloudview, ui->menuOpenRecent, this);
    connect(m_project_manager, &ProjectManager::windowTitleChanged, this, &MainWindow::setWindowTitle);
    setWindowTitle(m_project_manager->windowTitle()); // 初始标题（信号在连接前已发）
    connect(ui->actionNewProject, &QAction::triggered, m_project_manager, &ProjectManager::onNewProject);
    connect(ui->actionOpenProject, &QAction::triggered, m_project_manager, &ProjectManager::onOpenProject);
    connect(ui->actionSaveProject, &QAction::triggered, m_project_manager, &ProjectManager::onSaveProject);
    connect(ui->actionSaveProjectAs, &QAction::triggered, m_project_manager, &ProjectManager::onSaveProjectAs);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    if (!m_project_manager->confirmClose()) {
        event->ignore();
        return;
    }
    event->accept();
}

void MainWindow::moveEvent(QMoveEvent *event)
{
    QPoint pos = ui->cloudview->mapToGlobal(QPoint(0, 0));
    emit ui->cloudview->posChanged(pos);
    return QMainWindow::moveEvent(event);
}
