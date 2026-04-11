#include "shape_detection_dialog.h"

#include "algorithm/segmentation.h"
#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QCoreApplication>

#include <QtConcurrent/QtConcurrent>
#include <QTimer>

// PCL model type constants
#define SAC_MODEL_PLANE       0
#define SAC_MODEL_SPHERE      2
#define SAC_MODEL_CYLINDER    3
#define SAC_MODEL_CONE        4

// ======================== Constructor ========================

ShapeDetectionDialog::ShapeDetectionDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_canceled(false)
{
    setupUi();
    this->setWindowTitle("Shape Detection");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(380, 480);
    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

ShapeDetectionDialog::~ShapeDetectionDialog() = default;

// ======================== setupUi ========================

void ShapeDetectionDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- Target Shape ---
    auto* shape_group = new QGroupBox("Target Shape", this);
    auto* shape_layout = new QHBoxLayout(shape_group);
    shape_layout->setContentsMargins(6, 10, 6, 6);

    bg_model_ = new QButtonGroup(this);
    radio_plane_ = new QRadioButton("Plane", this);
    radio_sphere_ = new QRadioButton("Sphere", this);
    radio_cylinder_ = new QRadioButton("Cylinder", this);
    radio_cone_ = new QRadioButton("Cone", this);
    bg_model_->addButton(radio_plane_, SAC_MODEL_PLANE);
    bg_model_->addButton(radio_sphere_, SAC_MODEL_SPHERE);
    bg_model_->addButton(radio_cylinder_, SAC_MODEL_CYLINDER);
    bg_model_->addButton(radio_cone_, SAC_MODEL_CONE);
    radio_plane_->setChecked(true);
    shape_layout->addWidget(radio_plane_);
    shape_layout->addWidget(radio_sphere_);
    shape_layout->addWidget(radio_cylinder_);
    shape_layout->addWidget(radio_cone_);
    main_layout->addWidget(shape_group);

    // --- Fitting Parameters ---
    auto* fit_group = new QGroupBox("Fitting Parameters", this);
    auto* fit_layout = new QFormLayout(fit_group);
    fit_layout->setContentsMargins(6, 10, 6, 6);
    fit_layout->setSpacing(4);

    dspin_threshold_ = new QDoubleSpinBox(this);
    dspin_threshold_->setRange(0.0, 1000.0);
    dspin_threshold_->setDecimals(4);
    dspin_threshold_->setValue(0.01);
    dspin_threshold_->setSingleStep(0.001);
    fit_layout->addRow("Distance Threshold:", dspin_threshold_);

    dspin_probability_ = new QDoubleSpinBox(this);
    dspin_probability_->setRange(0.0, 1.0);
    dspin_probability_->setDecimals(3);
    dspin_probability_->setValue(0.99);
    dspin_probability_->setSingleStep(0.01);
    fit_layout->addRow("Probability:", dspin_probability_);

    spin_iterations_ = new QSpinBox(this);
    spin_iterations_->setRange(1, 100000);
    spin_iterations_->setValue(1000);
    spin_iterations_->setSingleStep(100);
    fit_layout->addRow("Max Iterations:", spin_iterations_);

    cbox_method_ = new QComboBox(this);
    cbox_method_->addItems({"RANSAC", "LMedS", "MSAC", "RRANSAC", "RMSAC", "MLESAC", "PROSAC"});
    fit_layout->addRow("Method:", cbox_method_);

    main_layout->addWidget(fit_group);

    // --- Advanced ---
    auto* adv_group = new QGroupBox("Advanced", this);
    auto* adv_layout = new QFormLayout(adv_group);
    adv_layout->setContentsMargins(6, 10, 6, 6);
    adv_layout->setSpacing(4);

    check_use_normals_ = new QCheckBox("Use Normals", this);
    adv_layout->addRow(check_use_normals_);

    dspin_distance_weight_ = new QDoubleSpinBox(this);
    dspin_distance_weight_->setRange(0.0, 1.0);
    dspin_distance_weight_->setDecimals(3);
    dspin_distance_weight_->setValue(0.1);
    dspin_distance_weight_->setSingleStep(0.01);
    dspin_distance_weight_->setEnabled(false);
    adv_layout->addRow("Normal Distance Weight:", dspin_distance_weight_);

    dspin_distance_origin_ = new QDoubleSpinBox(this);
    dspin_distance_origin_->setRange(0.0, 1000.0);
    dspin_distance_origin_->setDecimals(3);
    dspin_distance_origin_->setValue(0.0);
    dspin_distance_origin_->setSingleStep(0.1);
    dspin_distance_origin_->setEnabled(false);
    adv_layout->addRow("Distance From Origin:", dspin_distance_origin_);

    check_optimize_ = new QCheckBox("Optimize Coefficients", this);
    check_optimize_->setChecked(true);
    adv_layout->addRow(check_optimize_);

    main_layout->addWidget(adv_group);

    // --- Radius Range ---
    group_radius_ = new QGroupBox("Radius Range", this);
    auto* radius_layout = new QFormLayout(group_radius_);
    radius_layout->setContentsMargins(6, 10, 6, 6);
    radius_layout->setSpacing(4);

    dspin_min_radius_ = new QDoubleSpinBox(this);
    dspin_min_radius_->setRange(0.0, 1000.0);
    dspin_min_radius_->setDecimals(4);
    dspin_min_radius_->setValue(0.0);
    radius_layout->addRow("Min Radius:", dspin_min_radius_);

    dspin_max_radius_ = new QDoubleSpinBox(this);
    dspin_max_radius_->setRange(0.0, 1000.0);
    dspin_max_radius_->setDecimals(4);
    dspin_max_radius_->setValue(10.0);
    dspin_max_radius_->setSingleStep(0.1);
    radius_layout->addRow("Max Radius:", dspin_max_radius_);

    group_radius_->setVisible(false);
    main_layout->addWidget(group_radius_);

    // --- Output ---
    auto* output_group = new QGroupBox("Output", this);
    auto* output_layout = new QVBoxLayout(output_group);
    output_layout->setContentsMargins(6, 10, 6, 6);
    output_layout->setSpacing(4);

    check_colorize_ = new QCheckBox("Colorize Inliers", this);
    check_colorize_->setChecked(true);
    output_layout->addWidget(check_colorize_);

    check_split_inlier_ = new QCheckBox("Split Inlier / Outlier", this);
    check_split_inlier_->setChecked(true);
    output_layout->addWidget(check_split_inlier_);

    check_create_mesh_ = new QCheckBox("Create Geometric Mesh", this);
    check_create_mesh_->setEnabled(false);  // 二期灰显
    output_layout->addWidget(check_create_mesh_);

    main_layout->addWidget(output_group);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_apply_ = new QPushButton("Apply", this);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(bg_model_, QOverload<int>::of(&QButtonGroup::buttonClicked),
            this, &ShapeDetectionDialog::onModelChanged);
    connect(check_use_normals_, &QCheckBox::toggled, [=](bool checked) {
        dspin_distance_weight_->setEnabled(checked);
        dspin_distance_origin_->setEnabled(checked);
    });
    connect(btn_apply_, &QPushButton::clicked, this, &ShapeDetectionDialog::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &ShapeDetectionDialog::onCancel);
}

// ======================== Init / Reset ========================

void ShapeDetectionDialog::init()
{
    auto selection = m_cloudtree->getSelectedClouds();
    if (!selection.empty()) {
        m_cloud = selection.front();
    }
}

void ShapeDetectionDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_cloud.reset();
}

// ======================== Slots ========================

void ShapeDetectionDialog::onModelChanged(int index)
{
    // Show radius range only for sphere, cylinder, cone
    group_radius_->setVisible(index == SAC_MODEL_SPHERE ||
                              index == SAC_MODEL_CYLINDER ||
                              index == SAC_MODEL_CONE);
}

void ShapeDetectionDialog::onApply()
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    // ========== Step 1: 读取 UI 参数 ==========
    int model = bg_model_->checkedId();
    int method = cbox_method_->currentIndex();
    double threshold = dspin_threshold_->value();
    int max_iterations = spin_iterations_->value();
    double probability = dspin_probability_->value();
    bool optimize = check_optimize_->isChecked();
    double min_radius = dspin_min_radius_->value();
    double max_radius = dspin_max_radius_->value();
    bool useNormals = check_use_normals_->isChecked();
    double distance_weight = dspin_distance_weight_->value();
    double distance_origin = dspin_distance_origin_->value();
    bool splitInOutlier = check_split_inlier_->isChecked();
    bool colorizeInliers = check_colorize_->isChecked();

    // ========== Step 2: 隐藏对话框 ==========
    this->hide();
    QCoreApplication::processEvents();

    // ========== Step 3: 显示进度对话框 ==========
    m_cloudtree->showProgress("Shape Detection...");

    // ========== Step 4: 设置取消标志 ==========
    auto* cancel = new std::atomic<bool>(false);
    auto* progress_closed = new std::atomic<bool>(false);
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [=]() {
                    *cancel = true;
                    m_canceled.store(true);
                    m_cloudtree->closeProgress();
                    progress_closed->store(true);
                    printW("Shape detection canceled.");
                });
    }

    // ========== Step 5: 进度回调 ==========
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ========== Step 6: 提交算法 ==========
    auto cloud = m_cloud;
    m_canceled.store(false);

    auto future = QtConcurrent::run([cloud, useNormals, model, method,
                                     threshold, max_iterations, probability, optimize,
                                     min_radius, max_radius, distance_weight, distance_origin,
                                     splitInOutlier, cancel, on_progress]() {
        ct::SegmentationResult result;
        result.time_ms = 0;

        // 第一次调用：提取 inlier（negative=false）
        ct::SegmentationResult inlierResult;
        if (useNormals) {
            inlierResult = ct::Segmentation::SACSegmentationFromNormals(
                cloud, false, model, method, threshold, max_iterations, probability,
                optimize, min_radius, max_radius, distance_weight, distance_origin,
                cancel, on_progress);
        } else {
            inlierResult = ct::Segmentation::SACSegmentation(
                cloud, false, model, method, threshold, max_iterations, probability,
                optimize, min_radius, max_radius, cancel, on_progress);
        }

        if (inlierResult.clouds.empty()) return result;
        result.clouds.push_back(inlierResult.clouds[0]);
        result.coefficients = inlierResult.coefficients;
        result.time_ms += inlierResult.time_ms;

        // 检查取消
        if (cancel && cancel->load()) return result;

        // 第二次调用：提取 outlier（negative=true）
        if (splitInOutlier) {
            ct::SegmentationResult outlierResult;
            if (useNormals) {
                outlierResult = ct::Segmentation::SACSegmentationFromNormals(
                    cloud, true, model, method, threshold, max_iterations, probability,
                    optimize, min_radius, max_radius, distance_weight, distance_origin,
                    cancel, on_progress);
            } else {
                outlierResult = ct::Segmentation::SACSegmentation(
                    cloud, true, model, method, threshold, max_iterations, probability,
                    optimize, min_radius, max_radius, cancel, on_progress);
            }
            if (!outlierResult.clouds.empty()) {
                result.clouds.push_back(outlierResult.clouds[0]);
            }
            result.time_ms += outlierResult.time_ms;
        }

        return result;
    });

    // ========== Step 7: 监听完成信号 ==========
    auto* watcher = new QFutureWatcher<ct::SegmentationResult>(this);
    connect(watcher, &QFutureWatcher<ct::SegmentationResult>::finished, this,
        [=]() {
            if (!progress_closed->load()) {
                m_cloudtree->closeProgress();
            }
            delete cancel;
            delete progress_closed;

            auto result = watcher->result();
            watcher->deleteLater();

            if (m_canceled.load() || result.clouds.empty()) {
                this->reject();
                return;
            }

            printI(QString("Shape detection done in %1 ms, %2 cloud(s) generated.")
                       .arg(result.time_ms)
                       .arg(result.clouds.size()));

            // 添加结果到点云树
            std::vector<ct::Cloud::Ptr> results;
            for (size_t i = 0; i < result.clouds.size(); i++) {
                auto& c = result.clouds[i];
                c->setId(m_cloud->id() + (i == 0 ? "-inlier" : "-outlier"));
                c->makeAdaptive();
                if (i == 0 && colorizeInliers) {
                    c->setCloudColor(ct::RGB{0, 255, 0});
                } else if (i == 1) {
                    c->setCloudColor(ct::RGB{255, 0, 0});
                }
                results.push_back(c);
            }

            QString groupName = QString::fromStdString(m_cloud->id()) + "_ShapeDetection";
            m_cloudtree->addResultGroup(m_cloud, results, groupName);

            this->accept();
        });

    watcher->setFuture(future);
}

void ShapeDetectionDialog::onCancel()
{
    this->close();
}
