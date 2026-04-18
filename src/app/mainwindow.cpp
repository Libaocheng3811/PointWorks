// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include "mainwindow.h"
#include "ui_MainWindow.h"
#include "projectmanager.h"

#include "edit/boundingbox.h"
#include "edit/color.h"
#include "options/displaysettings.h"

#include "tool/cutting.h"
#include "tool/pickpoints.h"
#include "tool/filters.h"
#include "tool/sampling.h"
#include "tool/measure.h"
#include "tool/rangeimage.h"
#include "tool/segmentation/shape_detection_dialog.h"
#include "tool/segmentation/morphological_filter_dialog.h"
#include "tool/segmentation/region_growing_dialog.h"
#include "tool/segmentation/clustering_dialog.h"
#include "tool/segmentation/supervoxel_dialog.h"
#include "tool/mesh/compute_hull_dialog.h"
#include "tool/mesh/extract_boundary_dialog.h"
#include "tool/mesh/reconstruct_surface_dialog.h"
#include "edit/transformation.h"
#include "edit/normals.h"
#include "edit/scale.h"
#include "edit/coordinate.h"
#include "tool/align/global_registration.h"
#include "tool/align/fine_registration.h"
#include "tool/align/point_pairs_alignment.h"
#include "tool/align/align_by_centers.h"

#include "plugins/csfplugin.h"
#include "plugins/vegplugin.h"
#include "plugins/changedetectplugin.h"

#include "tool/distance/cloud_cloud_dist_dialog.h"
#include "tool/distance/cloud_mesh_dist_dialog.h"
#include "tool/distance/cloud_primitive_dist_dialog.h"
#include "tool/distance/closest_point_set_dialog.h"

#include "python/python_manager.h"
#include "python/python_bridge.h"
#include "python_connections.h"
#include "help_launcher.h"

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

#define  CLOUD_ICON_PATH    ":/res/icon2/cloud.svg"
#define  MESH_ICON_PATH     ":/res/icon2/mesh.svg"
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
    size.push_back(450);
    size.push_back(450);
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
    ui->cloudtree->setMeshIcon(QIcon(MESH_ICON_PATH));
    ui->cloudtree->setGroupIcon(style()->standardIcon(QStyle::SP_DirOpenIcon));

    // 设置属性表格列宽 - 第一列固定，第二列拉伸
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->cloudtable->setColumnWidth(0, 140);

    // 整行选中 + 交替行颜色
    ui->cloudtable->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->cloudtable->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->cloudtable->setAlternatingRowColors(true);

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
    connect(ui->actionDisplaySettings, &QAction::triggered, [=] {this->createModalDialog<DisplaySettingsDialog>("Display Settings"); });
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
    connect(ui->actionMeasure, &QAction::triggered, [=] {
        this->createDialog<Measure>("Measure");
    });
    connect(ui->actionRangeImage, &QAction::triggered, [=]{this->createDialog<RangeImage>("RangeImage");});
    connect(ui->actionClustering, &QAction::triggered, [=] {
        this->createModalDialog<ClusteringDialog>("Clustering");
    });
    connect(ui->actionRegionGrowing, &QAction::triggered, [=] {
        this->createModalDialog<RegionGrowingDialog>("Region Growing");
    });
    connect(ui->actionSupervoxel, &QAction::triggered, [=] {
        this->createModalDialog<SupervoxelDialog>("Supervoxel");
    });
    connect(ui->actionShapeDetection, &QAction::triggered, [=] {
        this->createModalDialog<ShapeDetectionDialog>("Shape Detection");
    });
    connect(ui->actionMorphologicalFilter, &QAction::triggered, [=] {
        this->createToolDialog<MorphologicalFilterDialog>("Morphological Filter");
    });
    connect(ui->actionMeshComputeHull, &QAction::triggered, [=] {
        this->createModalDialog<ComputeHullDialog>("Compute Hull");
    });
    connect(ui->actionMeshReconstructSurface, &QAction::triggered, [=] {
        this->createModalDialog<ReconstructSurfaceDialog>("Reconstruct Surface");
    });
    connect(ui->actionMeshExtractBoundary, &QAction::triggered, [=] {
        this->createModalDialog<ExtractBoundaryDialog>("Extract Boundary");
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

    // distance submenu
    connect(ui->actionDistanceC2C, &QAction::triggered, [=] {
        this->createModalDialog<CloudCloudDistDialog>("Cloud / Cloud Distance");
    });
    connect(ui->actionDistanceC2M, &QAction::triggered, [=] {
        this->createModalDialog<CloudMeshDistDialog>("Cloud / Mesh Distance");
    });
    connect(ui->actionDistanceC2P, &QAction::triggered, [=] {
        this->createModalDialog<CloudPrimitiveDistDialog>("Cloud / Primitive Distance");
    });
    connect(ui->actionDistanceCPS, &QAction::triggered, [=] {
        this->createModalDialog<ClosestPointSetDialog>("Closest Point Set");
    });

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
        ct::connectPythonSignals(bridge, ui->cloudview, ui->cloudtree, ui->console);
    }

    // === About ===
    connect(ui->actionAbout, &QAction::triggered, this, [this] { ct::HelpLauncher::showAbout(this); });

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
