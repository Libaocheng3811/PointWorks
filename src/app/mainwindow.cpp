// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include <QDesktopServices>
#include <QUrl>
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
#include "plugins/m3c2plugin.h"

#include "tool/distance/cloud_cloud_dist_dialog.h"
#include "tool/distance/cloud_mesh_dist_dialog.h"
#include "tool/distance/cloud_primitive_dist_dialog.h"
#include "tool/distance/closest_point_set_dialog.h"

#include "python/python_manager.h"
#include "python/python_bridge.h"
#include "python_connections.h"
#include "help/help_launcher.h"

#include "ui/base/language_manager.h"

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
#include <QSettings>
#include <QEvent>
#include <QActionGroup>

#define  OFFICIAL_WEBSITE   "https://libaocheng3811.github.io/PointWorks-docs/"
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

    // === ViewportManager（接管 centralwidget 中的 viewport_container）===
    m_viewport_mgr = new ct::ViewportManager(ui->viewport_container);

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
    ui->cloudtree->setCloudView(m_viewport_mgr->activeView());
    ui->cloudtree->setViewportManager(m_viewport_mgr);
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
    connect(ui->actionSave, &QAction::triggered, ui->cloudtree, &ct::CloudTree::smartSave);
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

    // view — 标准视图操作作用于活跃视窗
    auto activeView = [this]() { return m_viewport_mgr->activeView(); };
    connect(ui->actionResetcamera, &QAction::triggered, ui->cloudtree, &ct::CloudTree::zoomToSelected);
    connect(ui->actionTopView, &QAction::triggered, this, [=]{ activeView()->setTopView(); });
    connect(ui->actionFrontView, &QAction::triggered, this, [=]{ activeView()->setFrontView(); });
    connect(ui->actionLeftSideView, &QAction::triggered, this, [=]{ activeView()->setLeftSideView(); });
    connect(ui->actionBackView, &QAction::triggered, this, [=]{ activeView()->setBackView(); });
    connect(ui->actionRightSideView, &QAction::triggered, this, [=]{ activeView()->setRightSideView(); });
    connect(ui->actionBottomView, &QAction::triggered, this, [=]{ activeView()->setBottomView(); });
    connect(ui->actionShowID, &QAction::toggled, this, [=](bool checked){
        for (auto* v : m_viewport_mgr->allViews()) v->setShowId(checked);
    });
    connect(ui->actionShowAxes, &QAction::toggled, this, [=](bool checked){
        for (auto* v : m_viewport_mgr->allViews()) v->setShowAxes(checked);
    });
    connect(ui->actionShowFPS, &QAction::toggled, this, [=](bool checked){
        for (auto* v : m_viewport_mgr->allViews()) v->setShowFPS(checked);
    });
    ui->actionShowFPS->setChecked(true);
    ui->actionShowAxes->setChecked(true);
    ui->actionShowID->setChecked(true);

    // view — 布局切换
    auto* layoutGroup = new QActionGroup(this);
    layoutGroup->addAction(ui->actionViewportSingle);
    layoutGroup->addAction(ui->actionViewportHSplit);
    layoutGroup->addAction(ui->actionViewportVSplit);
    layoutGroup->addAction(ui->actionViewportTriple);
    layoutGroup->addAction(ui->actionViewportQuad);
    layoutGroup->setExclusive(true);

    connect(ui->actionViewportSingle, &QAction::triggered, this, [=]{
        m_viewport_mgr->setLayout(ct::ViewportManager::Single);
    });
    connect(ui->actionViewportHSplit, &QAction::triggered, this, [=]{
        m_viewport_mgr->setLayout(ct::ViewportManager::HorizontalSplit);
    });
    connect(ui->actionViewportVSplit, &QAction::triggered, this, [=]{
        m_viewport_mgr->setLayout(ct::ViewportManager::VerticalSplit);
    });
    connect(ui->actionViewportTriple, &QAction::triggered, this, [=]{
        m_viewport_mgr->setLayout(ct::ViewportManager::TripleSplit);
    });
    connect(ui->actionViewportQuad, &QAction::triggered, this, [=]{
        m_viewport_mgr->setLayout(ct::ViewportManager::QuadSplit);
    });

    // 活跃视窗切换时更新 CloudTree 的 CloudView 引用
    connect(m_viewport_mgr, &ct::ViewportManager::activeViewChanged, this, [this](ct::CloudView* view) {
        ui->cloudtree->setCloudView(view);
    });

    // 布局切换后重建视窗，需要重新添加所有点云并刷新属性栏
    connect(m_viewport_mgr, &ct::ViewportManager::viewsRecreated, this, [this]() {
        ui->cloudtree->setCloudView(m_viewport_mgr->activeView());
        ui->cloudtree->repopulateAllViews();
        emit ui->cloudtree->itemSelectionChanged();
        installDragFilterOnViews();
    });

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
            this, "PointPairsAlignment", m_viewport_mgr->activeView(), ui->cloudtree, ui->console);
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
    connect(ui->actionM3C2, &QAction::triggered, [=] {
        this->createModalDialog<M3C2Plugin>("M3C2");});

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
            ct::CloudView* view = m_viewport_mgr->activeView();
            QPoint tl = view->mapToGlobal(QPoint(0, 0));
            QSize cvSize = view->size();
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
        this->setStyleSheet("");  // clear stylesheet, restore default
    });

    // === Language switching ===
    connect(ui->actionEnglish, &QAction::triggered, this, [=] {
        ct::LanguageManager::instance().switchLanguage(ct::LanguageManager::English);
        ui->actionEnglish->setChecked(true);
        ui->actionChinese->setChecked(false);
        QSettings settings("PointWorks", "PointWorks");
        settings.setValue("language", static_cast<int>(ct::LanguageManager::English));
    });
    connect(ui->actionChinese, &QAction::triggered, this, [=] {
        ct::LanguageManager::instance().switchLanguage(ct::LanguageManager::Chinese);
        ui->actionChinese->setChecked(true);
        ui->actionEnglish->setChecked(false);
        QSettings settings("PointWorks", "PointWorks");
        settings.setValue("language", static_cast<int>(ct::LanguageManager::Chinese));
    });

    // Set initial checked state based on current language
    auto& langMgr = ct::LanguageManager::instance();
    ui->actionEnglish->setChecked(langMgr.currentLanguage() == ct::LanguageManager::English);
    ui->actionChinese->setChecked(langMgr.currentLanguage() == ct::LanguageManager::Chinese);

    // === Python Bridge 信号连接 ===
    auto* bridge = ct::PythonManager::instance().bridge();
    if (bridge) {
        ct::connectPythonSignals(bridge, m_viewport_mgr->activeView(), ui->cloudtree, ui->console);
    }

    // === Help ===
    connect(ui->action_Help, &QAction::triggered, this, [] {
        QDesktopServices::openUrl(QUrl(OFFICIAL_WEBSITE));
    });

    // === About ===
    connect(ui->actionAbout, &QAction::triggered, this, [this] { ct::HelpLauncher::showAbout(this); });

    // === 项目管理（自包含控制器） ===
    m_project_manager = new ProjectManager(ui->cloudtree, m_viewport_mgr->activeView(), ui->menuOpenRecent, this);
    connect(m_project_manager, &ProjectManager::windowTitleChanged, this, &MainWindow::setWindowTitle);
    setWindowTitle(m_project_manager->windowTitle()); // 初始标题（信号在连接前已发）
    connect(ui->actionNewProject, &QAction::triggered, m_project_manager, &ProjectManager::onNewProject);
    connect(ui->actionOpenProject, &QAction::triggered, m_project_manager, &ProjectManager::onOpenProject);
    connect(ui->cloudtree, &ct::CloudTree::requestSaveProject, this, [this](){
        if (m_project_manager->currentProjectPath().isEmpty())
            m_project_manager->onSaveProjectAs();
        else
            m_project_manager->onSaveProject();
    });
    connect(ui->actionSaveProjectAs, &QAction::triggered, m_project_manager, &ProjectManager::onSaveProjectAs);

    // === 功能操作边界：基于选择类型启用/禁用 Action ===
    connect(ui->cloudtree, &QTreeWidget::itemSelectionChanged,
            this, &MainWindow::onTreeSelectionChanged);

    // 初始化：无选中时禁用
    updateActionEnableState(ct::SelectionInfo{});

    // 为 CloudView 安装拖拽事件过滤器（VTK 控件默认不转发拖拽事件）
    installDragFilterOnViews();
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
    ct::CloudView* view = m_viewport_mgr->activeView();
    if (view) {
        QPoint pos = view->mapToGlobal(QPoint(0, 0));
        emit view->posChanged(pos);
    }
    return QMainWindow::moveEvent(event);
}

void MainWindow::changeEvent(QEvent* event)
{
    if (event->type() == QEvent::LanguageChange)
        ui->retranslateUi(this);
    QMainWindow::changeEvent(event);
}

void MainWindow::onTreeSelectionChanged()
{
    ct::SelectionInfo info = ui->cloudtree->getSelectionInfo();
    updateActionEnableState(info);
}

void MainWindow::updateActionEnableState(const ct::SelectionInfo& info)
{
    const bool any      = info.hasAny();
    const bool hasData  = info.hasAnySelection();
    const bool cloud    = info.hasCloud();
    const bool onlyCloud = info.hasOnlyCloud();
    const bool cloudOrMesh = info.hasCloudOrMesh();
    const bool singleCloud = info.isSingleCloud();

    const int totalClouds = ui->cloudtree->getTotalCloudCount();
    const int totalMeshes = ui->cloudtree->getTotalMeshCount();
    const bool enoughClouds = totalClouds >= 2;
    const bool hasContent = (totalClouds + totalMeshes) > 0;

    // --- File ---
    ui->actionClose->setEnabled(any);
    ui->actionClose_All->setEnabled(any);
    ui->actionMerge->setEnabled(any);
    ui->actionClone->setEnabled(any);
    ui->actionSave->setEnabled(any || hasContent);

    // --- Edit ---
    ui->actionColors->setEnabled(cloudOrMesh);
    ui->actionBoundingBox->setEnabled(onlyCloud);
    ui->actionTransformation->setEnabled(onlyCloud);
    ui->actionNormals->setEnabled(cloud);
    ui->actionScale->setEnabled(onlyCloud);
    ui->actionCoords->setEnabled(any);

    // --- Tools ---
    ui->actionPickPoints->setEnabled(singleCloud);
    ui->actionMeasure->setEnabled(onlyCloud);
    ui->actionCutting->setEnabled(onlyCloud);
    ui->actionFilters->setEnabled(singleCloud);
    ui->actionSampling->setEnabled(singleCloud);
    ui->actionRangeImage->setEnabled(singleCloud);

    // --- Segmentation ---
    ui->actionClustering->setEnabled(singleCloud);
    ui->actionRegionGrowing->setEnabled(singleCloud);
    ui->actionSupervoxel->setEnabled(singleCloud);
    ui->actionShapeDetection->setEnabled(singleCloud);
    ui->actionMorphologicalFilter->setEnabled(singleCloud);

    // --- Mesh (点云 → 网格) ---
    ui->actionMeshComputeHull->setEnabled(singleCloud);
    ui->actionMeshReconstructSurface->setEnabled(singleCloud);
    ui->actionMeshExtractBoundary->setEnabled(singleCloud);

    // --- Registration ---
    ui->actionAlignByCenters->setEnabled(onlyCloud && enoughClouds);
    ui->actionGlobalRegistration->setEnabled(onlyCloud && enoughClouds);
    ui->actionFineRegistration->setEnabled(onlyCloud && enoughClouds);
    ui->actionPointPairsAlignment->setEnabled(onlyCloud && enoughClouds);

    // --- Distance ---
    ui->actionDistanceC2C->setEnabled(onlyCloud && enoughClouds);
    ui->actionDistanceC2M->setEnabled(totalClouds > 0 && totalMeshes > 0);
    ui->actionDistanceC2P->setEnabled(singleCloud);
    ui->actionDistanceCPS->setEnabled(onlyCloud && enoughClouds);

    // --- Plugins ---
    ui->actionCSF->setEnabled(singleCloud);
    ui->actionVegetation_Filters->setEnabled(singleCloud);
    ui->actionChange_Detection->setEnabled(totalClouds >= 2);
    ui->actionM3C2->setEnabled(totalClouds >= 2);
}

// ================================================================
// 拖拽文件打开
// ================================================================

void MainWindow::installDragFilterOnViews()
{
    for (auto* view : m_viewport_mgr->allViews())
        view->installEventFilter(this);
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event)
{
    if (!event->mimeData()->hasUrls()) return;
    for (const auto& url : event->mimeData()->urls()) {
        if (url.isLocalFile() && isSupportedFile(QFileInfo(url.toLocalFile()).suffix().toLower())) {
            event->acceptProposedAction();
            return;
        }
    }
}

void MainWindow::dragMoveEvent(QDragMoveEvent* event)
{
    if (event->mimeData()->hasUrls())
        event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent* event)
{
    if (!event->mimeData()->hasUrls()) return;
    handleDroppedFiles(event->mimeData()->urls());
    event->acceptProposedAction();
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event)
{
    switch (event->type()) {
    case QEvent::DragEnter:
        dragEnterEvent(static_cast<QDragEnterEvent*>(event));
        return true;
    case QEvent::DragMove:
        dragMoveEvent(static_cast<QDragMoveEvent*>(event));
        return true;
    case QEvent::Drop:
        dropEvent(static_cast<QDropEvent*>(event));
        return true;
    default:
        break;
    }
    return QMainWindow::eventFilter(watched, event);
}

bool MainWindow::isSupportedFile(const QString& suffix) const
{
    static const QSet<QString> supported = {
        "ply", "pcd", "las", "laz", "e57", "txt", "asc", "xyz",
        "obj", "stl", "vtk", "ifs",
        "pwproj"
    };
    return supported.contains(suffix);
}

void MainWindow::handleDroppedFiles(const QList<QUrl>& urls)
{
    QStringList cloudFiles;
    QStringList unsupportedFiles;

    for (const auto& url : urls) {
        if (!url.isLocalFile()) continue;
        QString filepath = url.toLocalFile();
        QString suffix = QFileInfo(filepath).suffix().toLower();

        if (suffix == "pwproj") {
            m_project_manager->openProject(filepath);
        } else if (isSupportedFile(suffix)) {
            cloudFiles.append(filepath);
        } else {
            unsupportedFiles.append(QFileInfo(filepath).fileName());
        }
    }

    for (const auto& filepath : cloudFiles)
        ui->cloudtree->loadCloudFile(filepath);

    if (!unsupportedFiles.isEmpty()) {
        QMessageBox::warning(this, tr("Unsupported Files"),
            tr("The following files are not supported:\n%1").arg(unsupportedFiles.join("\n")));
    }
}
