#include "global_registration.h"

#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "ui/dialog/processingdialog.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QFormLayout>
#include <QMessageBox>
#include <QTimer>
#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>

#include <pcl/common/transforms.h>
#include <cmath>

// ======================== Constructor ========================

GlobalRegistrationDialog::GlobalRegistrationDialog(QWidget* parent)
    : ct::CustomDialog(parent)
{
    setupUi();

    this->setWindowTitle("Global Registration");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(360, 680);

    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

GlobalRegistrationDialog::~GlobalRegistrationDialog() = default;

// ======================== setupUi ========================

void GlobalRegistrationDialog::setupUi()
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

    // --- Keypoint ---
    auto* kp_group = new QGroupBox("Keypoint Extraction", this);
    auto* kp_layout = new QVBoxLayout(kp_group);
    kp_layout->setContentsMargins(6, 10, 6, 6);
    kp_layout->setSpacing(4);

    auto* kp_type_row = new QHBoxLayout();
    kp_type_row->addWidget(new QLabel("Algorithm:", this));
    cbox_keypoint_ = new QComboBox(this);
    cbox_keypoint_->addItems({"ISS", "Harris3D", "SIFT", "Trajkovic"});
    kp_type_row->addWidget(cbox_keypoint_, 1);
    kp_layout->addLayout(kp_type_row);

    keypoint_pages_ = new QStackedWidget(this);
    for (int i = 0; i < 4; i++)
        keypoint_pages_->addWidget(createKeypointPage(i));
    kp_layout->addWidget(keypoint_pages_);
    main_layout->addWidget(kp_group);

    // --- Descriptor ---
    auto* desc_group = new QGroupBox("Descriptor", this);
    auto* desc_layout = new QVBoxLayout(desc_group);
    desc_layout->setContentsMargins(6, 10, 6, 6);
    desc_layout->setSpacing(4);

    auto* desc_row = new QHBoxLayout();
    desc_row->addWidget(new QLabel("Type:", this));
    cbox_descriptor_ = new QComboBox(this);
    cbox_descriptor_->addItems({"FPFH"});
    desc_row->addWidget(cbox_descriptor_, 1);
    desc_layout->addLayout(desc_row);

    // Descriptor 参数 (FPFH/PFH 共享 k 和 radius)
    auto* desc_param_row = new QHBoxLayout();
    desc_param_row->addWidget(new QLabel("k:", this));
    spin_desc_k_ = new QSpinBox(this);
    spin_desc_k_->setRange(1, 500);
    spin_desc_k_->setValue(10);
    desc_param_row->addWidget(spin_desc_k_);
    desc_param_row->addSpacing(10);
    desc_param_row->addWidget(new QLabel("radius:", this));
    spin_desc_radius_ = new QDoubleSpinBox(this);
    spin_desc_radius_->setRange(0.001, 100.0);
    spin_desc_radius_->setDecimals(3);
    spin_desc_radius_->setValue(0.05);
    spin_desc_radius_->setSingleStep(0.01);
    desc_param_row->addWidget(spin_desc_radius_);
    desc_param_row->addStretch();
    desc_layout->addLayout(desc_param_row);
    main_layout->addWidget(desc_group);

    // --- Alignment ---
    auto* align_group = new QGroupBox("Alignment", this);
    auto* align_layout = new QVBoxLayout(align_group);
    align_layout->setContentsMargins(6, 10, 6, 6);
    align_layout->setSpacing(4);

    auto* align_type_row = new QHBoxLayout();
    align_type_row->addWidget(new QLabel("Method:", this));
    cbox_alignment_ = new QComboBox(this);
    cbox_alignment_->addItems({"SAC-IA", "SAC-Prerejective"});
    align_type_row->addWidget(cbox_alignment_, 1);
    align_layout->addLayout(align_type_row);

    alignment_pages_ = new QStackedWidget(this);
    for (int i = 0; i < 2; i++)
        alignment_pages_->addWidget(createAlignmentPage(i));
    align_layout->addWidget(alignment_pages_);
    main_layout->addWidget(align_group);

    // --- 对应连线选项 ---
    auto* lines_row = new QHBoxLayout();
    check_show_lines_ = new QCheckBox("Show correspondence lines", this);
    lines_row->addWidget(check_show_lines_);
    lines_row->addSpacing(10);
    lines_row->addWidget(new QLabel("Top-N:", this));
    spin_topn_ = new QSpinBox(this);
    spin_topn_->setRange(1, 10000);
    spin_topn_->setValue(100);
    spin_topn_->setSingleStep(50);
    lines_row->addWidget(spin_topn_);
    lines_row->addStretch();
    main_layout->addLayout(lines_row);

    // --- Result ---
    txt_result_ = new QTextEdit(this);
    txt_result_->setReadOnly(true);
    txt_result_->setMaximumHeight(140);
    txt_result_->setPlaceholderText("Click Compute to start global registration...");
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
    connect(cbox_keypoint_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GlobalRegistrationDialog::onKeypointChanged);
    connect(cbox_alignment_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GlobalRegistrationDialog::onAlignmentChanged);
    connect(check_show_lines_, &QCheckBox::toggled,
            this, &GlobalRegistrationDialog::onToggleLines);
    connect(btn_compute_, &QPushButton::clicked, this, &GlobalRegistrationDialog::onCompute);
    connect(btn_reset_, &QPushButton::clicked, this, &GlobalRegistrationDialog::onReset);
    connect(btn_apply_, &QPushButton::clicked, this, &GlobalRegistrationDialog::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &GlobalRegistrationDialog::onCancel);
}

// ======================== Keypoint Parameter Pages ========================

QWidget* GlobalRegistrationDialog::createKeypointPage(int type)
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    switch (type) {
    case 0: { // ISS
        spin_iss_gamma21_ = new QDoubleSpinBox(page);
        spin_iss_gamma21_->setRange(0.0, 1.0);
        spin_iss_gamma21_->setDecimals(3);
        spin_iss_gamma21_->setValue(0.975);
        layout->addRow("Gamma21:", spin_iss_gamma21_);

        spin_iss_gamma32_ = new QDoubleSpinBox(page);
        spin_iss_gamma32_->setRange(0.0, 1.0);
        spin_iss_gamma32_->setDecimals(3);
        spin_iss_gamma32_->setValue(0.975);
        layout->addRow("Gamma32:", spin_iss_gamma32_);

        spin_iss_min_neighbors_ = new QSpinBox(page);
        spin_iss_min_neighbors_->setRange(1, 100);
        spin_iss_min_neighbors_->setValue(5);
        layout->addRow("Min Neighbors:", spin_iss_min_neighbors_);

        spin_iss_angle_ = new QDoubleSpinBox(page);
        spin_iss_angle_->setRange(0.0, 180.0);
        spin_iss_angle_->setDecimals(1);
        spin_iss_angle_->setValue(30.0);
        layout->addRow("Angle (deg):", spin_iss_angle_);
        break;
    }
    case 1: { // Harris3D
        cbox_harris_method_ = new QComboBox(page);
        cbox_harris_method_->addItems({"HARRIS", "NOBLE", "LOWE", "TOMASI", "CURVATURE"});
        layout->addRow("Response:", cbox_harris_method_);

        spin_harris_threshold_ = new QDoubleSpinBox(page);
        spin_harris_threshold_->setRange(1e-9, 1e6);
        spin_harris_threshold_->setDecimals(6);
        spin_harris_threshold_->setValue(0.001);
        spin_harris_threshold_->setSingleStep(0.001);
        layout->addRow("Threshold:", spin_harris_threshold_);
        break;
    }
    case 2: { // SIFT
        spin_sift_min_scale_ = new QDoubleSpinBox(page);
        spin_sift_min_scale_->setRange(0.001, 100.0);
        spin_sift_min_scale_->setDecimals(4);
        spin_sift_min_scale_->setValue(0.01);
        spin_sift_min_scale_->setSingleStep(0.01);
        layout->addRow("Min Scale:", spin_sift_min_scale_);

        spin_sift_octaves_ = new QSpinBox(page);
        spin_sift_octaves_->setRange(1, 20);
        spin_sift_octaves_->setValue(6);
        layout->addRow("Octaves:", spin_sift_octaves_);

        spin_sift_scales_per_octave_ = new QSpinBox(page);
        spin_sift_scales_per_octave_->setRange(1, 20);
        spin_sift_scales_per_octave_->setValue(4);
        layout->addRow("Scales/Octave:", spin_sift_scales_per_octave_);

        spin_sift_min_contrast_ = new QDoubleSpinBox(page);
        spin_sift_min_contrast_->setRange(0.0, 1e6);
        spin_sift_min_contrast_->setDecimals(4);
        spin_sift_min_contrast_->setValue(0.05);
        spin_sift_min_contrast_->setSingleStep(0.01);
        layout->addRow("Min Contrast:", spin_sift_min_contrast_);
        break;
    }
    case 3: { // Trajkovic
        cbox_traj_method_ = new QComboBox(page);
        cbox_traj_method_->addItems({"Default", "Adaptive"});
        layout->addRow("Method:", cbox_traj_method_);

        spin_traj_window_ = new QSpinBox(page);
        spin_traj_window_->setRange(1, 20);
        spin_traj_window_->setValue(3);
        layout->addRow("Window:", spin_traj_window_);

        spin_traj_thr1_ = new QDoubleSpinBox(page);
        spin_traj_thr1_->setRange(0.0, 1e6);
        spin_traj_thr1_->setDecimals(4);
        spin_traj_thr1_->setValue(0.3);
        spin_traj_thr1_->setSingleStep(0.1);
        layout->addRow("Threshold1:", spin_traj_thr1_);

        spin_traj_thr2_ = new QDoubleSpinBox(page);
        spin_traj_thr2_->setRange(0.0, 1e6);
        spin_traj_thr2_->setDecimals(4);
        spin_traj_thr2_->setValue(0.1);
        spin_traj_thr2_->setSingleStep(0.1);
        layout->addRow("Threshold2:", spin_traj_thr2_);
        break;
    }
    }
    return page;
}

// ======================== Descriptor Page ========================

QWidget* GlobalRegistrationDialog::createDescriptorPage()
{
    // Descriptor 参数已在 setupUi 中直接添加到 desc_group
    // 此函数保留用于未来扩展独立页面
    return new QWidget(this);
}

// ======================== Alignment Parameter Pages ========================

QWidget* GlobalRegistrationDialog::createAlignmentPage(int type)
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    switch (type) {
    case 0: { // SAC-IA
        spin_sacia_min_dist_ = new QDoubleSpinBox(page);
        spin_sacia_min_dist_->setRange(0.0, 100.0);
        spin_sacia_min_dist_->setDecimals(4);
        spin_sacia_min_dist_->setValue(0.05);
        spin_sacia_min_dist_->setSingleStep(0.01);
        layout->addRow("Min Sample Dist:", spin_sacia_min_dist_);

        spin_sacia_samples_ = new QSpinBox(page);
        spin_sacia_samples_->setRange(1, 100000);
        spin_sacia_samples_->setValue(3);
        spin_sacia_samples_->setSingleStep(1);
        layout->addRow("Samples:", spin_sacia_samples_);

        spin_sacia_k_ = new QSpinBox(page);
        spin_sacia_k_->setRange(1, 500);
        spin_sacia_k_->setValue(10);
        layout->addRow("K:", spin_sacia_k_);
        break;
    }
    case 1: { // SAC-Prerejective
        spin_sacp_samples_ = new QSpinBox(page);
        spin_sacp_samples_->setRange(1, 100000);
        spin_sacp_samples_->setValue(3);
        spin_sacp_samples_->setSingleStep(1);
        layout->addRow("Samples:", spin_sacp_samples_);

        spin_sacp_k_ = new QSpinBox(page);
        spin_sacp_k_->setRange(1, 500);
        spin_sacp_k_->setValue(10);
        layout->addRow("K:", spin_sacp_k_);

        spin_sacp_similarity_ = new QDoubleSpinBox(page);
        spin_sacp_similarity_->setRange(0.0, 1.0);
        spin_sacp_similarity_->setDecimals(3);
        spin_sacp_similarity_->setValue(0.9);
        spin_sacp_similarity_->setSingleStep(0.05);
        layout->addRow("Similarity:", spin_sacp_similarity_);

        spin_sacp_inlier_frac_ = new QDoubleSpinBox(page);
        spin_sacp_inlier_frac_->setRange(0.0, 1.0);
        spin_sacp_inlier_frac_->setDecimals(3);
        spin_sacp_inlier_frac_->setValue(0.1);
        spin_sacp_inlier_frac_->setSingleStep(0.05);
        layout->addRow("Inlier Fraction:", spin_sacp_inlier_frac_);
        break;
    }
    }
    return page;
}

// ======================== Init / Reset ========================

void GlobalRegistrationDialog::init()
{
    refreshCloudList();
}

void GlobalRegistrationDialog::reset()
{
    clearCorrespondenceLines();
    m_aligned_cloud.reset();
    m_correspondences.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    m_last_compute_snapshot.clear();
    txt_result_->clear();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    btn_compute_->setEnabled(true);
}

void GlobalRegistrationDialog::deinit()
{
    m_progress->closeProgress();
    // 防御性清理预览云（Cancel/closeEvent 可能已处理）
    m_cloudview->removePointCloud(PREVIEW_ID);
    if (!m_source_id.isEmpty())
        m_cloudview->setPointCloudVisibility(m_source_id, true);
    reset();
}

// ======================== Slots ========================

void GlobalRegistrationDialog::onKeypointChanged(int index)
{
    if (index >= 0 && index < keypoint_pages_->count())
        keypoint_pages_->setCurrentIndex(index);
}

void GlobalRegistrationDialog::onDescriptorChanged(int index)
{
    Q_UNUSED(index);
}

void GlobalRegistrationDialog::onAlignmentChanged(int index)
{
    if (index >= 0 && index < alignment_pages_->count())
        alignment_pages_->setCurrentIndex(index);
}

void GlobalRegistrationDialog::onCompute()
{
    // 1. 验证 source / target
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
        snap.set("keypoint", cbox_keypoint_->currentIndex());
        snap.set("descriptor", cbox_descriptor_->currentIndex());
        snap.set("alignment", cbox_alignment_->currentIndex());
        snap.set("desc_k", spin_desc_k_->value());
        snap.set("desc_radius", spin_desc_radius_->value());
        snap.set("iss_gamma21", spin_iss_gamma21_->value());
        snap.set("iss_gamma32", spin_iss_gamma32_->value());
        snap.set("iss_min_neighbors", spin_iss_min_neighbors_->value());
        snap.set("iss_angle", spin_iss_angle_->value());
        snap.set("harris_method", cbox_harris_method_->currentIndex());
        snap.set("harris_threshold", spin_harris_threshold_->value());
        snap.set("sift_min_scale", spin_sift_min_scale_->value());
        snap.set("sift_octaves", spin_sift_octaves_->value());
        snap.set("sift_scales", spin_sift_scales_per_octave_->value());
        snap.set("sift_contrast", spin_sift_min_contrast_->value());
        snap.set("traj_method", cbox_traj_method_->currentIndex());
        snap.set("traj_window", spin_traj_window_->value());
        snap.set("traj_thr1", spin_traj_thr1_->value());
        snap.set("traj_thr2", spin_traj_thr2_->value());
        snap.set("sacia_min_dist", spin_sacia_min_dist_->value());
        snap.set("sacia_samples", spin_sacia_samples_->value());
        snap.set("sacia_k", spin_sacia_k_->value());
        snap.set("sacp_samples", spin_sacp_samples_->value());
        snap.set("sacp_k", spin_sacp_k_->value());
        snap.set("sacp_similarity", spin_sacp_similarity_->value());
        snap.set("sacp_inlier_frac", spin_sacp_inlier_frac_->value());
        if (snap == m_last_compute_snapshot && m_aligned_cloud) {
            printI("Parameters unchanged, skipping recomputation.");
            return;
        }
        m_last_compute_snapshot = snap;
    }

    // 2. 清理上次结果
    clearCorrespondenceLines();
    m_aligned_cloud.reset();
    m_correspondences.reset();
    m_kp_source.reset();
    m_kp_target.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);

    // 3. 计算点云分辨率
    auto calcResolution = [](const ct::Cloud::Ptr& cloud) -> double {
        auto box = cloud->box();
        float diag = std::sqrt(
            (box.width) * (box.width) +
            (box.height) * (box.height) +
            (box.depth) * (box.depth));
        size_t n = cloud->size();
        return n > 0 ? diag / std::sqrt((double)n) : 0.01;
    };
    double src_resolution = calcResolution(m_source);
    double tgt_resolution = calcResolution(m_target);

    // 4. 捕获控件参数
    int kp_type = cbox_keypoint_->currentIndex();
    int desc_type = cbox_descriptor_->currentIndex();
    int align_type = cbox_alignment_->currentIndex();
    int desc_k = spin_desc_k_->value();
    double desc_radius = spin_desc_radius_->value();

    double iss_gamma21 = spin_iss_gamma21_->value();
    double iss_gamma32 = spin_iss_gamma32_->value();
    int iss_min_neighbors = spin_iss_min_neighbors_->value();
    float iss_angle = spin_iss_angle_->value();
    int harris_method = cbox_harris_method_->currentIndex() + 1;
    double harris_threshold = spin_harris_threshold_->value();
    double sift_min_scale = spin_sift_min_scale_->value();
    int sift_octaves = spin_sift_octaves_->value();
    int sift_scales = spin_sift_scales_per_octave_->value();
    double sift_contrast = spin_sift_min_contrast_->value();
    int traj_method = cbox_traj_method_->currentIndex();
    int traj_window = spin_traj_window_->value();
    float traj_thr1 = spin_traj_thr1_->value();
    float traj_thr2 = spin_traj_thr2_->value();

    double sacia_min_dist = spin_sacia_min_dist_->value();
    int sacia_samples = spin_sacia_samples_->value();
    int sacia_k = spin_sacia_k_->value();
    int sacp_samples = spin_sacp_samples_->value();
    int sacp_k = spin_sacp_k_->value();
    double sacp_similarity = spin_sacp_similarity_->value();
    double sacp_inlier = spin_sacp_inlier_frac_->value();
    int topn = spin_topn_->value();

    // 5. UI 状态
    btn_compute_->setEnabled(false);
    txt_result_->setPlainText("Computing...");

    // 取消之前的计算，并为本次计算创建独立的取消标志
    if (m_cancel_flag) m_cancel_flag->store(true);
    auto cancel_flag = std::make_shared<std::atomic<bool>>(false);
    m_cancel_flag = cancel_flag;

    // 显示模态进度条
    m_progress->showProgress("Global Registration...");
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [this]() {
                    if (m_cancel_flag) m_cancel_flag->store(true);
                    if (m_progress->dialog())
                        m_progress->setMessage("Canceling...");
                });
    }

    // 跨线程进度更新回调
    auto setProgress = [this](int pct) {
        QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // 6. 异步执行
    auto source = m_source;
    auto target = m_target;
    auto* cancel = cancel_flag.get();

    // 结果结构体：同时携带配准结果和关键点
    struct PipelineResult {
        ct::RegistrationResult reg;
        ct::Cloud::Ptr kp_source;
        ct::Cloud::Ptr kp_target;
        QString error_msg;  // 失败原因
    };

    auto future = QtConcurrent::run([=]() -> PipelineResult {
        std::srand(42); // 固定随机种子，确保整个流水线可重复
        PipelineResult out;
        out.reg = {false, nullptr, 0, Eigen::Matrix4f::Identity(), 0};

        // --- a. 提取关键点 ---
        auto extractKp = [&](const ct::Cloud::Ptr& cloud, double res,
                              std::function<void(int)> on_prog) -> ct::Cloud::Ptr {
            ct::Cloud::Ptr kp;
            switch (kp_type) {
            case 0:
                kp = ct::Keypoints::ISSKeypoint3D(cloud, res,
                    iss_gamma21, iss_gamma32, iss_min_neighbors, iss_angle,
                    desc_k, desc_radius, cancel, on_prog).cloud;
                break;
            case 1:
                kp = ct::Keypoints::HarrisKeypoint3D(cloud,
                    harris_method, harris_threshold, true, false,
                    desc_k, desc_radius, cancel, on_prog).cloud;
                break;
            case 2:
                kp = ct::Keypoints::SIFTKeypoint(cloud,
                    sift_min_scale, sift_octaves, sift_scales, sift_contrast,
                    desc_k, desc_radius, cancel, on_prog).cloud;
                break;
            case 3:
                kp = ct::Keypoints::TrajkovicKeypoint3D(cloud,
                    traj_method, traj_window, traj_thr1, traj_thr2,
                    desc_k, desc_radius, cancel, on_prog).cloud;
                break;
            }
            return kp;
        };

        // Stage 1: 源关键点 (0-20%)
        out.kp_source = extractKp(source, src_resolution,
            [setProgress](int p) { setProgress(p / 5); });
        if (cancel->load()) return out;
        if (!out.kp_source || out.kp_source->empty()) {
            out.error_msg = "No keypoints extracted from source cloud.\n"
                            "Try adjusting keypoint parameters (e.g. lower ISS thresholds).";
            return out;
        }

        // Stage 2: 目标关键点 (20-40%)
        out.kp_target = extractKp(target, tgt_resolution,
            [setProgress](int p) { setProgress(20 + p / 5); });
        if (cancel->load()) return out;
        if (!out.kp_target || out.kp_target->empty()) {
            out.error_msg = "No keypoints extracted from target cloud.\n"
                            "Try adjusting keypoint parameters (e.g. lower ISS thresholds).";
            return out;
        }

        // --- b. 计算描述子 ---
        auto computeDesc = [&](const ct::Cloud::Ptr& cloud,
                                std::function<void(int)> on_prog) -> ct::FeatureType::Ptr {
            ct::FeatureType::Ptr feat(new ct::FeatureType);
            ct::FeatureResult fr;
            fr = ct::Features::FPFHEstimation(cloud, desc_k, desc_radius,
                nullptr, cancel, on_prog);
            if (fr.feature) feat->fpfh = fr.feature->fpfh;
            return feat;
        };

        // Stage 3: 源 FPFH (40-60%)
        auto src_feat = computeDesc(out.kp_source,
            [setProgress](int p) { setProgress(40 + p / 5); });
        if (cancel->load()) return out;
        if (!src_feat) return out;

        // Stage 4: 目标 FPFH (60-80%)
        auto tgt_feat = computeDesc(out.kp_target,
            [setProgress](int p) { setProgress(60 + p / 5); });
        if (cancel->load()) return out;
        if (!tgt_feat) return out;

        // Stage 5: SAC 配准 (80-100%)
        setProgress(80);

        // SAC-IA/SAC-Prerejective 要求 setInputSource 的点数与特征点数一一对应，
        // 因此必须传入关键点云而非原始点云
        ct::RegistrationContext ctx;
        ctx.source_cloud = out.kp_source;
        ctx.target_cloud = out.kp_target;
        ctx.params.max_iterations = 10;

        switch (align_type) {
        case 0: // SAC-IA
            if (src_feat->fpfh && tgt_feat->fpfh) {
                out.reg = ct::Registration::SampleConsensusInitialAlignment<pcl::FPFHSignature33>(
                    ctx, src_feat->fpfh, tgt_feat->fpfh,
                    sacia_min_dist, sacia_samples, sacia_k);
            } else {
                out.error_msg = "SAC-IA requires FPFH descriptor.\nPlease switch descriptor to FPFH.";
            }
            break;
        case 1: // SAC-Prerejective
            if (src_feat->fpfh && tgt_feat->fpfh) {
                out.reg = ct::Registration::SampleConsensusPrerejective<pcl::FPFHSignature33>(
                    ctx, src_feat->fpfh, tgt_feat->fpfh,
                    sacp_samples, sacp_k, sacp_similarity, sacp_inlier);
            } else {
                out.error_msg = "SAC-Prerejective requires FPFH descriptor.\nPlease switch descriptor to FPFH.";
            }
            break;
        }

        return out;
    });

    auto* watcher = new QFutureWatcher<PipelineResult>(this);
    watcher->setFuture(future);
    connect(watcher, &QFutureWatcher<PipelineResult>::finished, this, [=]() {
        m_progress->closeProgress();

        // 忽略过期计算的结果（例如用户 Reset 后重新 Compute）
        if (cancel_flag != m_cancel_flag) {
            watcher->deleteLater();
            return;
        }

        auto result = watcher->result();
        watcher->deleteLater();

        btn_compute_->setEnabled(true);

        if (cancel_flag->load()) {
            txt_result_->setPlainText("Canceled.");
            return;
        }

        if (!result.reg.success || !result.reg.aligned_cloud) {
            QString msg = !result.error_msg.isEmpty()
                ? result.error_msg
                : "Registration failed. The algorithm did not converge.\nTry adjusting alignment parameters.";
            printE("Global registration failed: " + msg.split('\n').first());
            txt_result_->setPlainText(msg);
            return;
        }

        // 保存结果
        m_result_matrix = result.reg.matrix;
        m_kp_source = result.kp_source;
        m_kp_target = result.kp_target;

        // 将源关键点也做变换，用于连线显示（源点云已隐藏，连线应连到变换后的关键点）
        if (m_kp_source) {
            auto kp_pcl = m_kp_source->toPCL_XYZRGBN();
            auto kp_copy = std::make_shared<pcl::PointCloud<ct::PointXYZRGBN>>(*kp_pcl);
            pcl::transformPointCloud(*kp_copy, *kp_copy, m_result_matrix);
            m_kp_source = ct::Cloud::fromPCL_XYZRGBN(*kp_copy);
        }

        // 用变换矩阵对原始源点云做变换，生成完整预览点云
        // （算法返回的 aligned_cloud 只是变换后的关键点，不能直接用于预览）
        {
            auto pcl_src = m_source->toPCL_XYZRGBN();
            auto pcl_copy = std::make_shared<pcl::PointCloud<ct::PointXYZRGBN>>(*pcl_src);
            pcl::transformPointCloud(*pcl_copy, *pcl_copy, m_result_matrix);
            m_aligned_cloud = ct::Cloud::fromPCL_XYZRGBN(*pcl_copy);
            m_aligned_cloud->setId(PREVIEW_ID);
        }

        // 构建对应关系（按索引配对，Top-N）
        if (m_kp_source && m_kp_target) {
            m_correspondences.reset(new pcl::Correspondences);
            int n = std::min(m_kp_source->size(), m_kp_target->size());
            n = std::min(n, topn);
            for (int i = 0; i < n; i++)
                m_correspondences->push_back(pcl::Correspondence(i, i, 0));
        }

        // 显示结果
        QString text;
        text += QString("Fitness Score: %1\n").arg(result.reg.score, 0, 'f', 6);
        text += QString("Time: %1 ms\n\n").arg(result.reg.time_ms, 0, 'f', 1);
        text += "Transformation Matrix:\n";
        for (int r = 0; r < 4; r++) {
            QStringList row;
            for (int c = 0; c < 4; c++)
                row << QString("%1").arg(result.reg.matrix(r, c), 9, 'f', 4);
            text += row.join("  ") + "\n";
        }
        txt_result_->setPlainText(text);

        // 预览：只在视图中显示，不插入文件树
        m_aligned_cloud->setId(PREVIEW_ID);
        m_cloudview->removePointCloud(PREVIEW_ID);
        m_cloudview->setPointCloudVisibility(m_source_id, false);
        m_cloudview->addPointCloud(m_aligned_cloud);
        m_cloudview->refresh();

        // 渲染连线
        if (check_show_lines_->isChecked() && m_correspondences)
            updateCorrespondenceLines();

        btn_apply_->setEnabled(true);
        btn_cancel_->setEnabled(true);

        printI(QString("Global registration done. Score: %1, Time: %2 ms")
               .arg(result.reg.score, 0, 'f', 6).arg(result.reg.time_ms, 0, 'f', 1));
    }, Qt::UniqueConnection);
}

void GlobalRegistrationDialog::onComputeDone()
{
    // 由 QFutureWatcher<PipelineResult>::finished lambda 直接处理
    // 此 slot 不再单独使用
}

void GlobalRegistrationDialog::onToggleLines(bool checked)
{
    if (checked && !m_correspondences) return;
    if (checked)
        updateCorrespondenceLines();
    else
        clearCorrespondenceLines();
}

void GlobalRegistrationDialog::onApply()
{
    if (!m_aligned_cloud || !m_source) return;

    // 用变换矩阵直接变换原始源点云（避免 toPCL 缓存被污染，需深拷贝）
    auto pcl_src = m_source->toPCL_XYZRGBN();
    auto pcl_copy = std::make_shared<pcl::PointCloud<ct::PointXYZRGBN>>(*pcl_src);
    pcl::transformPointCloud(*pcl_copy, *pcl_copy, m_result_matrix);
    auto transformed = ct::Cloud::fromPCL_XYZRGBN(*pcl_copy);
    transformed->setId(m_source_id.toStdString());

    m_cloudtree->updateCloud(m_source, transformed);

    // 移除预览云，恢复源点云可见
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->invalidateCloudRender(m_source_id);
    m_cloudview->refresh();

    clearCorrespondenceLines();
    printI("Global registration applied successfully.");

    m_aligned_cloud.reset();
    m_kp_source.reset();
    m_kp_target.reset();
    m_correspondences.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    txt_result_->clear();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
}

void GlobalRegistrationDialog::onReset()
{
    m_progress->closeProgress();
    if (m_cancel_flag) m_cancel_flag->store(true);

    // 移除预览云，恢复源点云可见
    m_cloudview->removePointCloud(PREVIEW_ID);
    if (!m_source_id.isEmpty())
        m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->refresh();

    clearCorrespondenceLines();
    m_aligned_cloud.reset();
    m_kp_source.reset();
    m_kp_target.reset();
    m_correspondences.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    m_last_compute_snapshot.clear();
    txt_result_->clear();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    btn_compute_->setEnabled(true);

    printI("Global registration reset.");
}

void GlobalRegistrationDialog::onCancel()
{
    m_progress->closeProgress();
    if (m_cancel_flag) m_cancel_flag->store(true);

    // 移除预览云，恢复源点云可见（源点云从未被修改）
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->refresh();

    clearCorrespondenceLines();
    m_aligned_cloud.reset();
    m_kp_source.reset();
    m_kp_target.reset();
    m_correspondences.reset();
    m_result_matrix = Eigen::Matrix4f::Identity();
    txt_result_->clear();
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    btn_compute_->setEnabled(true);

    printI("Global registration canceled.");
    this->close();
}

// ======================== Helpers ========================

void GlobalRegistrationDialog::refreshCloudList()
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

void GlobalRegistrationDialog::updateCorrespondenceLines()
{
    if (!m_cloudview || !m_correspondences || !m_kp_source || !m_kp_target) return;
    m_cloudview->addCorrespondences(m_kp_source, m_kp_target, m_correspondences, "correspondences", 10);
}

void GlobalRegistrationDialog::clearCorrespondenceLines()
{
    if (m_cloudview) {
        m_cloudview->removeShape("correspondences");
    }
}
