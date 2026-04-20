#include "reconstruct_surface_dialog.h"

#include "core/cloud.h"
#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QCoreApplication>
#include <QTimer>

#include <QtConcurrent/QtConcurrent>

#include <pcl/filters/random_sample.h>
#include <pcl/conversions.h>

// ======================== Constructor ========================

ReconstructSurfaceDialog::ReconstructSurfaceDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_canceled(false)
{
    setupUi();
    this->setWindowTitle("Reconstruct Surface");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(380, 440);
    QTimer::singleShot(0, this, [this]() { this->adjustSize(); });
}

ReconstructSurfaceDialog::~ReconstructSurfaceDialog() = default;

// ======================== setupUi ========================

void ReconstructSurfaceDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- 法线警告标签 (默认隐藏) ---
    label_warning_ = new QLabel(this);
    label_warning_->setWordWrap(true);
    label_warning_->setStyleSheet(
        "QLabel { background-color: #FFF3CD; color: #856404; "
        "padding: 6px; border-radius: 4px; }");
    label_warning_->setText(
        "⚠ Selected point cloud has no normals.\n"
        "Surface reconstruction requires normals. Please estimate normals first.");
    label_warning_->setVisible(false);
    main_layout->addWidget(label_warning_);

    // --- Algorithm Selection ---
    auto* algo_row = new QHBoxLayout();
    algo_row->addWidget(new QLabel("Method:", this));
    cbox_algorithm_ = new QComboBox(this);
    cbox_algorithm_->addItems({
        "Poisson (Recommended)",
        "Greedy Projection",
        "Marching Cubes (Hoppe)",
        "Grid Projection"
    });
    algo_row->addWidget(cbox_algorithm_, 1);
    main_layout->addLayout(algo_row);

    // --- Parameters (Stacked) ---
    param_pages_ = new QStackedWidget(this);
    param_pages_->addWidget(createPoissonParamPage());     // 0
    param_pages_->addWidget(createGreedyParamPage());      // 1
    param_pages_->addWidget(createMarchingCubesParamPage());// 2
    param_pages_->addWidget(createGridProjectionParamPage());// 3
    main_layout->addWidget(param_pages_);

    // --- Post-processing ---
    auto* post_group = new QGroupBox("Post-processing", this);
    auto* post_layout = new QVBoxLayout(post_group);
    post_layout->setContentsMargins(6, 10, 6, 6);
    post_layout->setSpacing(4);

    check_extract_boundary_ = new QCheckBox("Extract Boundary Polylines", this);
    post_layout->addWidget(check_extract_boundary_);
    main_layout->addWidget(post_group);

    // --- Preview Downsample ---
    auto* preview_row = new QHBoxLayout();
    preview_row->addWidget(new QLabel("Preview Downsample:", this));
    dspin_downsample_rate_ = new QDoubleSpinBox(this);
    dspin_downsample_rate_->setRange(0.01, 1.0);
    dspin_downsample_rate_->setDecimals(2);
    dspin_downsample_rate_->setValue(0.10);
    dspin_downsample_rate_->setSingleStep(0.05);
    preview_row->addWidget(dspin_downsample_rate_, 1);
    preview_row->addWidget(new QLabel("(10% of points)", this));
    main_layout->addLayout(preview_row);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_preview_ = new QPushButton("Preview", this);
    btn_apply_ = new QPushButton("Apply", this);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_close_ = new QPushButton("Close", this);
    btn_row->addWidget(btn_preview_);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel_);
    btn_row->addWidget(btn_close_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(cbox_algorithm_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ReconstructSurfaceDialog::onAlgorithmChanged);
    connect(btn_preview_, &QPushButton::clicked, this, &ReconstructSurfaceDialog::onPreview);
    connect(btn_apply_, &QPushButton::clicked, this, &ReconstructSurfaceDialog::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &ReconstructSurfaceDialog::onCancel);
    connect(btn_close_, &QPushButton::clicked, this, &ReconstructSurfaceDialog::onClose);
}

// ======================== Param Pages ========================

QWidget* ReconstructSurfaceDialog::createPoissonParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);

    // --- 核心参数 ---
    auto* core_form = new QFormLayout();
    core_form->setContentsMargins(6, 4, 6, 4);
    core_form->setSpacing(4);

    spin_depth_ = new QSpinBox(page);
    spin_depth_->setRange(5, 12);
    spin_depth_->setValue(8);
    core_form->addRow("Depth (Octree):", spin_depth_);

    dspin_point_weight_ = new QDoubleSpinBox(page);
    dspin_point_weight_->setRange(0.0, 100.0);
    dspin_point_weight_->setDecimals(1);
    dspin_point_weight_->setValue(2.0);
    dspin_point_weight_->setSingleStep(0.5);
    core_form->addRow("Point Weight:", dspin_point_weight_);

    layout->addLayout(core_form);

    // --- Advanced 折叠区 ---
    auto* adv_group = new QGroupBox("Advanced", page);
    adv_group->setCheckable(true);
    adv_group->setChecked(false);
    auto* adv_form = new QFormLayout(adv_group);
    adv_form->setContentsMargins(6, 10, 6, 6);
    adv_form->setSpacing(4);

    spin_min_depth_ = new QSpinBox(adv_group);
    spin_min_depth_->setRange(0, 12);
    spin_min_depth_->setValue(5);
    adv_form->addRow("Min Depth:", spin_min_depth_);

    dspin_scale_ = new QDoubleSpinBox(adv_group);
    dspin_scale_->setRange(0.0, 10.0);
    dspin_scale_->setDecimals(1);
    dspin_scale_->setValue(1.1);
    dspin_scale_->setSingleStep(0.1);
    adv_form->addRow("Scale:", dspin_scale_);

    spin_solver_divide_ = new QSpinBox(adv_group);
    spin_solver_divide_->setRange(1, 12);
    spin_solver_divide_->setValue(8);
    adv_form->addRow("Solver Divide:", spin_solver_divide_);

    spin_iso_divide_ = new QSpinBox(adv_group);
    spin_iso_divide_->setRange(1, 12);
    spin_iso_divide_->setValue(8);
    adv_form->addRow("ISO Divide:", spin_iso_divide_);

    dspin_samples_per_node_ = new QDoubleSpinBox(adv_group);
    dspin_samples_per_node_->setRange(0.0, 100.0);
    dspin_samples_per_node_->setDecimals(1);
    dspin_samples_per_node_->setValue(1.5);
    dspin_samples_per_node_->setSingleStep(0.5);
    adv_form->addRow("Samples/Node:", dspin_samples_per_node_);

    check_confidence_ = new QCheckBox("Confidence", adv_group);
    adv_form->addRow(check_confidence_);

    check_manifold_ = new QCheckBox("Manifold", adv_group);
    check_manifold_->setChecked(true);
    adv_form->addRow(check_manifold_);

    layout->addWidget(adv_group);
    return page;
}

QWidget* ReconstructSurfaceDialog::createGreedyParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);

    // --- 核心参数 ---
    auto* core_form = new QFormLayout();
    core_form->setContentsMargins(6, 4, 6, 4);
    core_form->setSpacing(4);

    dspin_search_radius_ = new QDoubleSpinBox(page);
    dspin_search_radius_->setRange(0.001, 100.0);
    dspin_search_radius_->setDecimals(4);
    dspin_search_radius_->setValue(0.025);
    dspin_search_radius_->setSingleStep(0.005);
    core_form->addRow("Search Radius:", dspin_search_radius_);

    dspin_mu_ = new QDoubleSpinBox(page);
    dspin_mu_->setRange(0.0, 100.0);
    dspin_mu_->setDecimals(1);
    dspin_mu_->setValue(2.5);
    dspin_mu_->setSingleStep(0.5);
    core_form->addRow("Multiplier (Mu):", dspin_mu_);

    spin_max_neighbors_ = new QSpinBox(page);
    spin_max_neighbors_->setRange(1, 500);
    spin_max_neighbors_->setValue(100);
    core_form->addRow("Max Neighbors:", spin_max_neighbors_);

    layout->addLayout(core_form);

    // --- Advanced 折叠区 ---
    auto* adv_group = new QGroupBox("Advanced", page);
    adv_group->setCheckable(true);
    adv_group->setChecked(false);
    auto* adv_form = new QFormLayout(adv_group);
    adv_form->setContentsMargins(6, 10, 6, 6);
    adv_form->setSpacing(4);

    dspin_min_angle_ = new QDoubleSpinBox(adv_group);
    dspin_min_angle_->setRange(0.0, 180.0);
    dspin_min_angle_->setDecimals(1);
    dspin_min_angle_->setValue(5.0);
    adv_form->addRow("Min Angle (°):", dspin_min_angle_);

    dspin_max_angle_ = new QDoubleSpinBox(adv_group);
    dspin_max_angle_->setRange(0.0, 180.0);
    dspin_max_angle_->setDecimals(1);
    dspin_max_angle_->setValue(150.0);
    adv_form->addRow("Max Angle (°):", dspin_max_angle_);

    dspin_eps_angle_ = new QDoubleSpinBox(adv_group);
    dspin_eps_angle_->setRange(0.0, 180.0);
    dspin_eps_angle_->setDecimals(1);
    dspin_eps_angle_->setValue(15.0);
    adv_form->addRow("EPS Angle (°):", dspin_eps_angle_);

    check_consistent_ = new QCheckBox("Consistent Normals", adv_group);
    check_consistent_->setChecked(true);
    adv_form->addRow(check_consistent_);

    layout->addWidget(adv_group);
    return page;
}

QWidget* ReconstructSurfaceDialog::createMarchingCubesParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    dspin_iso_level_ = new QDoubleSpinBox(page);
    dspin_iso_level_->setRange(0.0, 1.0);
    dspin_iso_level_->setDecimals(2);
    dspin_iso_level_->setValue(0.0);
    dspin_iso_level_->setSingleStep(0.05);
    layout->addRow("ISO Level:", dspin_iso_level_);

    spin_grid_res_ = new QSpinBox(page);
    spin_grid_res_->setRange(10, 200);
    spin_grid_res_->setValue(50);
    spin_grid_res_->setSingleStep(10);
    layout->addRow("Grid Resolution:", spin_grid_res_);

    dspin_percentage_ = new QDoubleSpinBox(page);
    dspin_percentage_->setRange(0.0, 1.0);
    dspin_percentage_->setDecimals(2);
    dspin_percentage_->setValue(0.1);
    dspin_percentage_->setSingleStep(0.05);
    layout->addRow("Percentage:", dspin_percentage_);

    dspin_epsilon_ = new QDoubleSpinBox(page);
    dspin_epsilon_->setRange(0.0, 1.0);
    dspin_epsilon_->setDecimals(3);
    dspin_epsilon_->setValue(0.01);
    dspin_epsilon_->setSingleStep(0.005);
    layout->addRow("Epsilon:", dspin_epsilon_);

    return page;
}

QWidget* ReconstructSurfaceDialog::createGridProjectionParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    dspin_resolution_ = new QDoubleSpinBox(page);
    dspin_resolution_->setRange(0.001, 1.0);
    dspin_resolution_->setDecimals(4);
    dspin_resolution_->setValue(0.01);
    dspin_resolution_->setSingleStep(0.005);
    layout->addRow("Resolution:", dspin_resolution_);

    spin_padding_size_ = new QSpinBox(page);
    spin_padding_size_->setRange(1, 20);
    spin_padding_size_->setValue(3);
    layout->addRow("Padding Size:", spin_padding_size_);

    spin_k_ = new QSpinBox(page);
    spin_k_->setRange(1, 100);
    spin_k_->setValue(20);
    layout->addRow("K Neighbors:", spin_k_);

    return page;
}

// ======================== Init / Reset ========================

void ReconstructSurfaceDialog::init()
{
    auto selection = m_cloudtree->getSelectedClouds();
    if (!selection.empty()) {
        m_cloud = selection.front();
    }
    checkNormalsWarning();
}

void ReconstructSurfaceDialog::reset()
{
    m_progress->closeProgress();
    m_canceled.store(true);
    m_cloud.reset();

    // 清理 preview 产生的 mesh 和 boundary
    for (const auto& id : m_preview_ids) {
        m_cloudview->removePolygonMesh(id);
        m_cloudview->removeShape(id);
    }
    m_preview_ids.clear();
    m_cloudview->clearInfo();
}

// ======================== Helpers ========================

void ReconstructSurfaceDialog::checkNormalsWarning()
{
    bool has_warning = m_cloud && !m_cloud->hasNormals();
    label_warning_->setVisible(has_warning);
    QTimer::singleShot(0, this, [this]() { this->adjustSize(); });
}

// ======================== Slots ========================

void ReconstructSurfaceDialog::onAlgorithmChanged(int index)
{
    if (index >= 0 && index < param_pages_->count())
        param_pages_->setCurrentIndex(index);
    QTimer::singleShot(0, this, [this]() { this->adjustSize(); });
}

void ReconstructSurfaceDialog::runReconstruct(bool is_preview)
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    if (!m_cloud->hasNormals()) {
        printW("Selected cloud has no normals. Please estimate normals first.");
        return;
    }

    int algorithm = cbox_algorithm_->currentIndex();

    // 读取各算法参数
    // Poisson
    int depth = spin_depth_->value();
    int min_depth = spin_min_depth_->value();
    float point_weight = static_cast<float>(dspin_point_weight_->value());
    float scale = static_cast<float>(dspin_scale_->value());
    int solver_divide = spin_solver_divide_->value();
    int iso_divide = spin_iso_divide_->value();
    float samples_per_node = static_cast<float>(dspin_samples_per_node_->value());
    bool confidence = check_confidence_->isChecked();
    bool manifold = check_manifold_->isChecked();
    // Greedy
    double search_radius = dspin_search_radius_->value();
    double mu = dspin_mu_->value();
    int max_neighbors = spin_max_neighbors_->value();
    double min_angle = dspin_min_angle_->value();
    double max_angle = dspin_max_angle_->value();
    double eps_angle = dspin_eps_angle_->value();
    bool consistent = check_consistent_->isChecked();
    // Marching Cubes
    float iso_level = static_cast<float>(dspin_iso_level_->value());
    int grid_res = spin_grid_res_->value();
    float percentage = static_cast<float>(dspin_percentage_->value());
    float epsilon = static_cast<float>(dspin_epsilon_->value());
    // Grid Projection
    double resolution = dspin_resolution_->value();
    int padding_size = spin_padding_size_->value();
    int k = spin_k_->value();

    // Preview 下采样
    double downsample_rate = is_preview ? dspin_downsample_rate_->value() : 1.0;

    // ========== Step 1: 隐藏对话框 ==========
    this->hide();
    QCoreApplication::processEvents();

    // ========== Step 2: 显示进度条 ==========
    QString algo_names[] = {"Poisson", "Greedy Projection", "Marching Cubes (Hoppe)", "Grid Projection"};
    QString prefix = is_preview ? "Preview: " : "";
    m_progress->showProgress(prefix + algo_names[algorithm] + "...");

    // ========== Step 3: 设置取消标志 ==========
    auto* cancel = new std::atomic<bool>(false);
    auto* progress_closed = new std::atomic<bool>(false);
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [=]() {
                    *cancel = true;
                    m_canceled.store(true);
                    m_progress->closeProgress();
                    progress_closed->store(true);
                    printW(prefix + algo_names[algorithm] + " canceled.");
                });
    }

    // ========== Step 4: 进度回调 ==========
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ========== Step 5: 准备点云 (Preview 时下采样) ==========
    auto cloud = m_cloud;
    ct::Cloud::Ptr work_cloud = cloud;
    if (is_preview && downsample_rate < 1.0) {
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        size_t total = pcl_cloud->size();
        size_t target = static_cast<size_t>(total * downsample_rate);
        if (target < 100) target = 100;

        pcl::RandomSample<ct::PointXYZRGBN> rs;
        rs.setInputCloud(pcl_cloud);
        rs.setSample(static_cast<int>(target));
        auto filtered = pcl::PointCloud<ct::PointXYZRGBN>::Ptr(new pcl::PointCloud<ct::PointXYZRGBN>);
        rs.filter(*filtered);
        work_cloud = ct::Cloud::fromPCL_XYZRGBN(*filtered);
    }

    // 确保 work_cloud 有 ID
    if (work_cloud->id().empty())
        work_cloud->setId(cloud->id() + "_preview");

    m_canceled.store(false);

    auto viz = std::make_shared<ct::SurfaceResultViz>();

    // ========== Step 6: 提交算法到工作线程 ==========
    auto future = QtConcurrent::run(
        [work_cloud, algorithm, depth, min_depth, point_weight, scale,
         solver_divide, iso_divide, samples_per_node, confidence, manifold,
         search_radius, mu, max_neighbors, min_angle, max_angle, eps_angle, consistent,
         iso_level, grid_res, percentage, epsilon,
         resolution, padding_size, k,
         cancel, on_progress, viz]() -> ct::SurfaceResult {
            ct::SurfaceResult result;
            switch (algorithm) {
            case 0: // Poisson
                result = ct::Surface::Poisson(work_cloud, depth, min_depth,
                    point_weight, scale, solver_divide, iso_divide,
                    samples_per_node, confidence, false, manifold,
                    cancel, on_progress);
                break;
            case 1: // Greedy Projection
                result = ct::Surface::GreedyProjectionTriangulation(work_cloud,
                    mu, max_neighbors, search_radius,
                    min_angle, max_angle, eps_angle,
                    consistent, true,
                    cancel, on_progress);
                break;
            case 2: // Marching Cubes Hoppe
                result = ct::Surface::MarchingCubesHoppe(work_cloud,
                    iso_level, grid_res, grid_res, grid_res,
                    percentage, epsilon,
                    cancel, on_progress);
                break;
            case 3: // Grid Projection
                result = ct::Surface::GridProjection(work_cloud,
                    resolution, padding_size, k, 8,
                    cancel, on_progress);
                break;
            default:
                return ct::SurfaceResult{};
            }

            // 在工作线程中完成所有耗时的数据准备
            ct::prepareSurfaceForRendering(result.mesh, *viz);
            return result;
        });

    // ========== Step 7: 监听完成信号 ==========
    auto* watcher = new QFutureWatcher<ct::SurfaceResult>(this);
    connect(watcher, &QFutureWatcher<ct::SurfaceResult>::finished, this,
        [=]() {
            auto result = watcher->result();
            watcher->deleteLater();

            if (m_canceled.load() || !result.mesh) {
                m_progress->closeProgress();
                delete cancel;
                delete progress_closed;
                if (result.mesh == nullptr && !m_canceled.load()) {
                    printW(QString("%1 failed: %2")
                               .arg(algo_names[algorithm])
                               .arg(QString::fromStdString(result.error_msg)));
                }
                if (is_preview) {
                    QTimer::singleShot(0, this, [this]() { this->show(); });
                }
                return;
            }

            // 更新进度条提示
            if (!is_preview && m_progress->dialog()) {
                m_progress->setMessage("Loading result...");
            }
            QCoreApplication::processEvents();

            QString suffix = is_preview ? "_preview" : "_surface";
            QString algo_suffix[] = {"_poisson", "_greedy", "_marching_cubes", "_grid_projection"};
            QString result_id = QString::fromStdString(cloud->id()) + algo_suffix[algorithm] + suffix;

            // 确保 result_id 唯一
            if (!is_preview) {
                QString base_id = result_id;
                int counter = 1;
                while (m_cloudtree->getItemById(result_id) != nullptr) {
                    result_id = base_id + "(" + QString::number(counter++) + ")";
                }
            }

            if (is_preview) {
                for (const auto& pid : m_preview_ids) {
                    m_cloudview->removePolygonMesh(pid);
                    m_cloudview->removeShape(pid);
                }
                m_preview_ids.clear();
            }

            if (is_preview) {
                m_cloudview->addPolygonMesh(result.mesh, result_id);
                m_preview_ids.append(result_id);
            }

            printI(QString("%1%2 done in %3 ms, result: [%4]")
                       .arg(prefix)
                       .arg(algo_names[algorithm])
                       .arg(result.time_ms)
                       .arg(result_id));

            if (check_extract_boundary_->isChecked()) {
                QString boundary_id = result_id + "_boundary";
                m_cloudview->addPolylineFromPolygonMesh(result.mesh, boundary_id);
                if (is_preview) {
                    m_preview_ids.append(boundary_id);
                }
                printI(QString("Boundary polylines extracted: [%1]").arg(boundary_id));
            }

            if (is_preview) {
                m_progress->closeProgress();
                delete cancel;
                delete progress_closed;
                QTimer::singleShot(0, this, [this]() { this->show(); });
            } else {
                if (viz->prepared_cloud) {
                    viz->prepared_cloud->setId(result_id.toStdString());

                    QTreeWidgetItem* origin_item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
                    m_cloudtree->insertCloud(viz->prepared_cloud, origin_item, true, ct::MountStrategy::Sibling,
                                             ct::NodeMesh);

                    if (viz->prepared_polydata) {
                        m_cloudtree->registerMeshPrebuilt(result_id, result.mesh, viz->prepared_polydata);
                    } else {
                        m_cloudtree->registerMesh(result_id, result.mesh);
                    }

                    if (check_extract_boundary_->isChecked()) {
                        QString boundary_id = result_id + "_boundary";
                        m_cloudtree->registerShape(result_id, boundary_id, "Boundary", result.mesh);
                    }
                }
                m_progress->closeProgress();
                delete cancel;
                delete progress_closed;
                this->accept();
            }
        });

    watcher->setFuture(future);
}

void ReconstructSurfaceDialog::onPreview()
{
    runReconstruct(true);
}

void ReconstructSurfaceDialog::onApply()
{
    // 先清除可能存在的 preview 结果
    for (const auto& id : m_preview_ids) {
        m_cloudview->removePolygonMesh(id);
        m_cloudview->removeShape(id);
    }
    m_preview_ids.clear();
    runReconstruct(false);
}

void ReconstructSurfaceDialog::onCancel()
{
    // Cancel: 清除 preview 结果，恢复初始状态，但不关闭对话框
    for (const auto& id : m_preview_ids) {
        m_cloudview->removePolygonMesh(id);
        m_cloudview->removeShape(id);
    }
    m_preview_ids.clear();
    m_cloudview->clearInfo();
    m_canceled.store(true);
}

void ReconstructSurfaceDialog::onClose()
{
    // Close: 关闭对话框，reset() 会被 CustomDialog::closeEvent 自动调用
    this->reject();
}
