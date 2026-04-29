#include "fine_registration.h"

#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "ui/dialog/processingdialog.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QFormLayout>

#include <QtConcurrent/QtConcurrent>
#include <QTimer>

// ======================== Constructor ========================

FineRegistrationDialog::FineRegistrationDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_canceled(false)
{
    setupUi();

    this->setWindowTitle("ICP (Fine Registration)");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(360, 520);

    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

FineRegistrationDialog::~FineRegistrationDialog() = default;

// ======================== setupUi ========================

void FineRegistrationDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- Source / Target ---
    auto* cloud_row = new QHBoxLayout();
    cloud_row->addWidget(new QLabel("Source:", this));
    cbox_source_ = new QComboBox(this);
    cbox_source_->setMinimumWidth(120);
    cloud_row->addWidget(cbox_source_, 1);
    cloud_row->addSpacing(10);
    cloud_row->addWidget(new QLabel("Target:", this));
    cbox_target_ = new QComboBox(this);
    cbox_target_->setMinimumWidth(120);
    cloud_row->addWidget(cbox_target_, 1);
    main_layout->addLayout(cloud_row);

    // --- Algorithm + Params ---
    auto* algo_group = new QGroupBox("Algorithm", this);
    auto* algo_layout = new QVBoxLayout(algo_group);
    algo_layout->setContentsMargins(6, 10, 6, 6);
    algo_layout->setSpacing(4);

    auto* algo_row = new QHBoxLayout();
    algo_row->addWidget(new QLabel("Method:", this));
    cbox_algorithm_ = new QComboBox(this);
    cbox_algorithm_->addItems({"ICP", "ICP with Normals", "ICP NonLinear", "GICP", "NDT"});
    algo_row->addWidget(cbox_algorithm_, 1);
    algo_layout->addLayout(algo_row);

    param_pages_ = new QStackedWidget(this);
    for (int i = 0; i < 5; i++)
        param_pages_->addWidget(createParamPage(i));
    algo_layout->addWidget(param_pages_);
    main_layout->addWidget(algo_group);

    // --- Result ---
    txt_result_ = new QTextEdit(this);
    txt_result_->setReadOnly(true);
    txt_result_->setMaximumHeight(160);
    txt_result_->setPlaceholderText("Click Compute to start fine registration...");
    main_layout->addWidget(txt_result_);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_compute_ = new QPushButton("Compute", this);
    btn_reset_ = new QPushButton("Reset", this);
    btn_apply_ = new QPushButton("Apply", this);
    btn_apply_->setEnabled(false);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_row->addWidget(btn_compute_);
    btn_row->addWidget(btn_reset_);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(cbox_algorithm_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &FineRegistrationDialog::onAlgorithmChanged);
    connect(btn_compute_, &QPushButton::clicked, this, &FineRegistrationDialog::onCompute);
    connect(btn_reset_, &QPushButton::clicked, this, &FineRegistrationDialog::onReset);
    connect(btn_apply_, &QPushButton::clicked, this, &FineRegistrationDialog::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &FineRegistrationDialog::onCancel);
}

// ======================== Param Pages ========================

QWidget* FineRegistrationDialog::createParamPage(int type)
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    switch (type) {
    case 0: { // ICP
        spin_icp_max_iter_ = new QSpinBox(page);
        spin_icp_max_iter_->setRange(1, 10000);
        spin_icp_max_iter_->setValue(50);
        spin_icp_max_iter_->setSingleStep(10);
        layout->addRow("Max Iterations:", spin_icp_max_iter_);

        spin_icp_corr_dist_ = new QDoubleSpinBox(page);
        spin_icp_corr_dist_->setRange(0.0, 1000.0);
        spin_icp_corr_dist_->setDecimals(4);
        spin_icp_corr_dist_->setValue(0.05);
        spin_icp_corr_dist_->setSingleStep(0.01);
        layout->addRow("Corr. Distance:", spin_icp_corr_dist_);

        check_icp_reciprocal_ = new QCheckBox("Use reciprocal correspondences", page);
        layout->addRow(check_icp_reciprocal_);
        break;
    }
    case 1: { // ICP with Normals
        spin_icpn_max_iter_ = new QSpinBox(page);
        spin_icpn_max_iter_->setRange(1, 10000);
        spin_icpn_max_iter_->setValue(50);
        spin_icpn_max_iter_->setSingleStep(10);
        layout->addRow("Max Iterations:", spin_icpn_max_iter_);

        check_icpn_reciprocal_ = new QCheckBox("Use reciprocal", page);
        layout->addRow(check_icpn_reciprocal_);

        check_icpn_symmetric_ = new QCheckBox("Symmetric objective", page);
        layout->addRow(check_icpn_symmetric_);

        check_icpn_enforce_normals_ = new QCheckBox("Enforce same direction normals", page);
        layout->addRow(check_icpn_enforce_normals_);
        break;
    }
    case 2: { // ICP NonLinear
        spin_icpnl_max_iter_ = new QSpinBox(page);
        spin_icpnl_max_iter_->setRange(1, 10000);
        spin_icpnl_max_iter_->setValue(50);
        spin_icpnl_max_iter_->setSingleStep(10);
        layout->addRow("Max Iterations:", spin_icpnl_max_iter_);

        spin_icpnl_corr_dist_ = new QDoubleSpinBox(page);
        spin_icpnl_corr_dist_->setRange(0.0, 1000.0);
        spin_icpnl_corr_dist_->setDecimals(4);
        spin_icpnl_corr_dist_->setValue(0.05);
        spin_icpnl_corr_dist_->setSingleStep(0.01);
        layout->addRow("Corr. Distance:", spin_icpnl_corr_dist_);

        check_icpnl_reciprocal_ = new QCheckBox("Use reciprocal", page);
        layout->addRow(check_icpnl_reciprocal_);
        break;
    }
    case 3: { // GICP
        spin_gicp_max_iter_ = new QSpinBox(page);
        spin_gicp_max_iter_->setRange(1, 10000);
        spin_gicp_max_iter_->setValue(50);
        spin_gicp_max_iter_->setSingleStep(10);
        layout->addRow("Max Iterations:", spin_gicp_max_iter_);

        spin_gicp_k_ = new QSpinBox(page);
        spin_gicp_k_->setRange(1, 500);
        spin_gicp_k_->setValue(30);
        layout->addRow("K:", spin_gicp_k_);

        spin_gicp_rol_tolerance_ = new QDoubleSpinBox(page);
        spin_gicp_rol_tolerance_->setRange(1e-9, 1.0);
        spin_gicp_rol_tolerance_->setDecimals(8);
        spin_gicp_rol_tolerance_->setValue(1e-5);
        spin_gicp_rol_tolerance_->setSingleStep(1e-5);
        layout->addRow("Rot. Epsilon:", spin_gicp_rol_tolerance_);
        break;
    }
    case 4: { // NDT
        spin_ndt_resolution_ = new QDoubleSpinBox(page);
        spin_ndt_resolution_->setRange(0.01, 100.0);
        spin_ndt_resolution_->setDecimals(2);
        spin_ndt_resolution_->setValue(1.0);
        spin_ndt_resolution_->setSingleStep(0.1);
        layout->addRow("Resolution:", spin_ndt_resolution_);

        spin_ndt_step_size_ = new QDoubleSpinBox(page);
        spin_ndt_step_size_->setRange(0.001, 10.0);
        spin_ndt_step_size_->setDecimals(4);
        spin_ndt_step_size_->setValue(0.05);
        spin_ndt_step_size_->setSingleStep(0.01);
        layout->addRow("Step Size:", spin_ndt_step_size_);

        spin_ndt_outlier_ratio_ = new QDoubleSpinBox(page);
        spin_ndt_outlier_ratio_->setRange(0.0, 1.0);
        spin_ndt_outlier_ratio_->setDecimals(3);
        spin_ndt_outlier_ratio_->setValue(0.35);
        spin_ndt_outlier_ratio_->setSingleStep(0.05);
        layout->addRow("Outlier Ratio:", spin_ndt_outlier_ratio_);
        break;
    }
    }
    return page;
}

// ======================== Init / Reset ========================

void FineRegistrationDialog::init()
{
    refreshCloudList();
}

void FineRegistrationDialog::reset()
{
    m_aligned_cloud.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    m_last_compute_snapshot.clear();
    txt_result_->clear();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    btn_compute_->setEnabled(true);
}

void FineRegistrationDialog::deinit()
{
    m_progress->closeProgress();
    // 防御性清理预览云
    m_cloudview->removePointCloud(PREVIEW_ID);
    if (!m_source_id.isEmpty())
        m_cloudview->setPointCloudVisibility(m_source_id, true);
    reset();
}

// ======================== Slots ========================

void FineRegistrationDialog::onAlgorithmChanged(int index)
{
    if (index >= 0 && index < param_pages_->count())
        param_pages_->setCurrentIndex(index);
}

void FineRegistrationDialog::onCompute()
{
    // 1. 验证
    if (cbox_source_->currentIndex() < 0 || cbox_target_->currentIndex() < 0) {
        printW("Please select source and target clouds.");
        return;
    }
    if (cbox_source_->currentIndex() == cbox_target_->currentIndex()) {
        printW("Source and target must be different clouds.");
        return;
    }

    auto clouds = m_cloudtree->getAllClouds();
    int si = cbox_source_->currentIndex();
    int ti = cbox_target_->currentIndex();
    if (si >= (int)clouds.size() || ti >= (int)clouds.size()) return;

    m_source = clouds[si];
    m_target = clouds[ti];
    m_source_id = QString::fromStdString(m_source->id());

    if (!m_source || !m_target || m_source->empty() || m_target->empty()) {
        printE("Source or target cloud is empty.");
        return;
    }

    // 参数快照对比：非首次且参数一致则跳过
    {
        ct::ParamSnapshot snap;
        snap.set("source_id", cbox_source_->currentText());
        snap.set("target_id", cbox_target_->currentText());
        snap.set("algorithm", cbox_algorithm_->currentIndex());
        snap.set("icp_max_iter", spin_icp_max_iter_->value());
        snap.set("icp_corr_dist", spin_icp_corr_dist_->value());
        snap.set("icp_reciprocal", check_icp_reciprocal_->isChecked());
        snap.set("icpn_max_iter", spin_icpn_max_iter_->value());
        snap.set("icpn_reciprocal", check_icpn_reciprocal_->isChecked());
        snap.set("icpn_symmetric", check_icpn_symmetric_->isChecked());
        snap.set("icpn_enforce_normals", check_icpn_enforce_normals_->isChecked());
        snap.set("icpnl_max_iter", spin_icpnl_max_iter_->value());
        snap.set("icpnl_corr_dist", spin_icpnl_corr_dist_->value());
        snap.set("icpnl_reciprocal", check_icpnl_reciprocal_->isChecked());
        snap.set("gicp_max_iter", spin_gicp_max_iter_->value());
        snap.set("gicp_k", spin_gicp_k_->value());
        snap.set("gicp_rol_tolerance", spin_gicp_rol_tolerance_->value());
        snap.set("ndt_resolution", spin_ndt_resolution_->value());
        snap.set("ndt_step_size", spin_ndt_step_size_->value());
        snap.set("ndt_outlier_ratio", spin_ndt_outlier_ratio_->value());
        if (snap == m_last_compute_snapshot && m_aligned_cloud) {
            printI("Parameters unchanged, skipping recomputation.");
            return;
        }
        m_last_compute_snapshot = snap;
    }

    // 2. 清理上次结果
    m_aligned_cloud.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);

    // 3. 捕获参数
    int algo = cbox_algorithm_->currentIndex();

    // 通用参数
    int max_iter = 50;
    double corr_dist = 0.05;
    bool reciprocal = false;
    double distance_threshold = std::sqrt(std::numeric_limits<double>::max());

    switch (algo) {
    case 0: // ICP
        max_iter = spin_icp_max_iter_->value();
        corr_dist = spin_icp_corr_dist_->value();
        reciprocal = check_icp_reciprocal_->isChecked();
        break;
    case 1: // ICP with Normals
        max_iter = spin_icpn_max_iter_->value();
        reciprocal = check_icpn_reciprocal_->isChecked();
        break;
    case 2: // ICP NonLinear
        max_iter = spin_icpnl_max_iter_->value();
        corr_dist = spin_icpnl_corr_dist_->value();
        reciprocal = check_icpnl_reciprocal_->isChecked();
        break;
    case 3: // GICP
        max_iter = spin_gicp_max_iter_->value();
        break;
    case 4: // NDT
        // NDT 使用独立参数，不设 max_iter
        break;
    }

    int gicp_k = spin_gicp_k_->value();
    double gicp_rol = spin_gicp_rol_tolerance_->value();
    double ndt_res = spin_ndt_resolution_->value();
    double ndt_step = spin_ndt_step_size_->value();
    double ndt_outlier = spin_ndt_outlier_ratio_->value();

    // ICP with Normals 专用
    bool icpn_symmetric = check_icpn_symmetric_->isChecked();
    bool icpn_enforce = check_icpn_enforce_normals_->isChecked();

    // 4. UI 状态
    btn_compute_->setEnabled(false);
    txt_result_->setPlainText("Computing...");
    m_canceled.store(false);

    // 显示模态进度条
    m_progress->showProgress("Fine Registration...");
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [this]() {
                    m_canceled.store(true);
                    if (m_progress->dialog())
                        m_progress->setMessage("Canceling...");
                });
    }

    // 跨线程进度更新回调
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // 5. 异步执行
    auto source = m_source;
    auto target = m_target;
    auto* cancel = &m_canceled;

    auto future = QtConcurrent::run([=]() -> ct::RegistrationResult {
        ct::RegistrationContext ctx;
        ctx.source_cloud = source;
        ctx.target_cloud = target;
        ctx.params.max_iterations = max_iter;
        ctx.params.distance_threshold = distance_threshold;
        ctx.params.inlier_threshold = corr_dist;

        switch (algo) {
        case 0:
            return ct::Registration::IterativeClosestPoint(ctx, reciprocal, cancel, on_progress);
        case 1:
            return ct::Registration::IterativeClosestPointWithNormals(
                ctx, reciprocal, icpn_symmetric, icpn_enforce, cancel, on_progress);
        case 2:
            return ct::Registration::IterativeClosestPointNonLinear(ctx, reciprocal, cancel, on_progress);
        case 3:
            return ct::Registration::GeneralizedIterativeClosestPoint(
                ctx, gicp_k, max_iter, 0.0, gicp_rol, false, cancel, on_progress);
        case 4:
            return ct::Registration::NormalDistributionsTransform(
                ctx, ndt_res, ndt_step, ndt_outlier, cancel, on_progress);
        }
        return {false, nullptr, 0, Eigen::Matrix4f::Identity(), 0};
    });

    auto* watcher = new QFutureWatcher<ct::RegistrationResult>(this);
    watcher->setFuture(future);
    connect(watcher, &QFutureWatcher<ct::RegistrationResult>::finished, this, [=]() {
        m_progress->closeProgress();
        auto result = watcher->result();
        watcher->deleteLater();

        btn_compute_->setEnabled(true);

        if (m_canceled.load()) {
            txt_result_->setPlainText("Canceled.");
            return;
        }

        if (!result.success || !result.aligned_cloud) {
            printE("Fine registration failed. Try adjusting parameters.");
            txt_result_->setPlainText("Registration failed.");
            return;
        }

        // 保存结果
        m_result_matrix = result.matrix;
        m_aligned_cloud = result.aligned_cloud;

        // 显示结果
        QString text;
        text += QString("Fitness Score: %1\n").arg(result.score, 0, 'f', 6);
        text += QString("Time: %1 ms\n\n").arg(result.time_ms, 0, 'f', 1);
        text += "Transformation Matrix:\n";
        for (int r = 0; r < 4; r++) {
            QStringList row;
            for (int c = 0; c < 4; c++)
                row << QString("%1").arg(result.matrix(r, c), 9, 'f', 4);
            text += row.join("  ") + "\n";
        }
        txt_result_->setPlainText(text);

        // 预览：只在视图中显示，不插入文件树
        m_aligned_cloud->setId(PREVIEW_ID);
        m_cloudview->removePointCloud(PREVIEW_ID);
        m_cloudview->setPointCloudVisibility(m_source_id, false);
        m_cloudview->addPointCloud(m_aligned_cloud);
        m_cloudview->refresh();

        btn_apply_->setEnabled(true);
        btn_cancel_->setEnabled(true);

        printI(QString("Fine registration done. Score: %1, Time: %2 ms")
               .arg(result.score, 0, 'f', 6).arg(result.time_ms, 0, 'f', 1));
    }, Qt::UniqueConnection);
}

void FineRegistrationDialog::onApply()
{
    if (!m_aligned_cloud || !m_source) return;

    // 用变换矩阵变换原始源点云（深拷贝避免缓存污染）
    auto pcl_src = m_source->toPCL_XYZRGBN();
    auto pcl_copy = std::make_shared<pcl::PointCloud<ct::PointXYZRGBN>>(*pcl_src);
    pcl::transformPointCloud(*pcl_copy, *pcl_copy, m_result_matrix);
    auto transformed = ct::Cloud::fromPCL_XYZRGBN(*pcl_copy, m_source->getGlobalShift());
    transformed->setId(m_source_id.toStdString());

    m_cloudtree->updateCloud(m_source, transformed);

    // 移除预览云，恢复源点云可见
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->invalidateCloudRender(m_source_id);
    m_cloudview->refresh();

    printI("Fine registration applied successfully.");

    m_aligned_cloud.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    txt_result_->clear();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
}

void FineRegistrationDialog::onReset()
{
    m_progress->closeProgress();
    m_canceled.store(true);

    m_cloudview->removePointCloud(PREVIEW_ID);
    if (!m_source_id.isEmpty())
        m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->refresh();

    m_aligned_cloud.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    m_last_compute_snapshot.clear();
    txt_result_->clear();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    btn_compute_->setEnabled(true);

    printI("Fine registration reset.");
}

void FineRegistrationDialog::onCancel()
{
    m_progress->closeProgress();
    m_canceled.store(true);

    // 移除预览云，恢复源点云可见（源点云从未被修改）
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->refresh();

    m_aligned_cloud.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    txt_result_->clear();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    btn_compute_->setEnabled(true);

    printI("Fine registration canceled.");
    this->close();
}

// ======================== Helpers ========================

void FineRegistrationDialog::refreshCloudList()
{
    cbox_source_->clear();
    cbox_target_->clear();
    if (!m_cloudtree) return;

    auto clouds = m_cloudtree->getAllClouds();
    for (const auto& c : clouds) {
        QString name = QString::fromStdString(c->id());
        cbox_source_->addItem(name);
        cbox_target_->addItem(name);
    }
    if (cbox_source_->count() >= 2) {
        cbox_source_->setCurrentIndex(0);
        cbox_target_->setCurrentIndex(1);
    }
}
