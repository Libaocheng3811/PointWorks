#include "supervoxel_dialog.h"

#include "algorithm/segmentation.h"
#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QCoreApplication>

#include <QtConcurrent/QtConcurrent>
#include <QTimer>

// ======================== Constructor ========================

SupervoxelDialog::SupervoxelDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_canceled(false)
{
    setupUi();
    this->setWindowTitle("Supervoxel Clustering");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(320, 300);
    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

SupervoxelDialog::~SupervoxelDialog() = default;

// ======================== setupUi ========================

void SupervoxelDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- Voxel Parameters ---
    auto* voxel_group = new QGroupBox("Voxel Parameters", this);
    auto* voxel_layout = new QFormLayout(voxel_group);
    voxel_layout->setContentsMargins(6, 10, 6, 6);
    voxel_layout->setSpacing(4);

    dspin_voxel_resolution_ = new QDoubleSpinBox(this);
    dspin_voxel_resolution_->setRange(0.001, 100.0);
    dspin_voxel_resolution_->setDecimals(4);
    dspin_voxel_resolution_->setValue(0.008);
    dspin_voxel_resolution_->setSingleStep(0.001);
    voxel_layout->addRow("Voxel Resolution:", dspin_voxel_resolution_);

    dspin_seed_resolution_ = new QDoubleSpinBox(this);
    dspin_seed_resolution_->setRange(0.001, 100.0);
    dspin_seed_resolution_->setDecimals(3);
    dspin_seed_resolution_->setValue(0.1);
    dspin_seed_resolution_->setSingleStep(0.01);
    voxel_layout->addRow("Seed Resolution:", dspin_seed_resolution_);

    main_layout->addWidget(voxel_group);

    // --- Importance Weights ---
    auto* weight_group = new QGroupBox("Importance Weights", this);
    auto* weight_layout = new QFormLayout(weight_group);
    weight_layout->setContentsMargins(6, 10, 6, 6);
    weight_layout->setSpacing(4);

    dspin_color_importance_ = new QDoubleSpinBox(this);
    dspin_color_importance_->setRange(0.0, 1.0);
    dspin_color_importance_->setDecimals(2);
    dspin_color_importance_->setValue(0.2);
    dspin_color_importance_->setSingleStep(0.05);
    weight_layout->addRow("Color Importance:", dspin_color_importance_);

    dspin_spatial_importance_ = new QDoubleSpinBox(this);
    dspin_spatial_importance_->setRange(0.0, 1.0);
    dspin_spatial_importance_->setDecimals(2);
    dspin_spatial_importance_->setValue(0.4);
    dspin_spatial_importance_->setSingleStep(0.05);
    weight_layout->addRow("Spatial Importance:", dspin_spatial_importance_);

    dspin_normal_importance_ = new QDoubleSpinBox(this);
    dspin_normal_importance_->setRange(0.0, 1.0);
    dspin_normal_importance_->setDecimals(2);
    dspin_normal_importance_->setValue(1.0);
    dspin_normal_importance_->setSingleStep(0.05);
    weight_layout->addRow("Normal Importance:", dspin_normal_importance_);

    main_layout->addWidget(weight_group);

    // --- Transform ---
    check_camera_transform_ = new QCheckBox("Camera Transform", this);
    main_layout->addWidget(check_camera_transform_);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_apply_ = new QPushButton("Apply", this);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(btn_apply_, &QPushButton::clicked, this, &SupervoxelDialog::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &SupervoxelDialog::onCancel);
}

// ======================== Init / Reset ========================

void SupervoxelDialog::init()
{
    auto selection = m_cloudtree->getSelectedClouds();
    if (!selection.empty()) {
        m_cloud = selection.front();
    }
}

void SupervoxelDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_cloud.reset();
}

// ======================== Slots ========================

void SupervoxelDialog::onApply()
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    // ========== Step 1: 读取 UI 参数 ==========
    float voxel_resolution = dspin_voxel_resolution_->value();
    float seed_resolution = dspin_seed_resolution_->value();
    float color_importance = dspin_color_importance_->value();
    float spatial_importance = dspin_spatial_importance_->value();
    float normal_importance = dspin_normal_importance_->value();
    bool camera_transform = check_camera_transform_->isChecked();

    // ========== Step 2: 隐藏对话框 ==========
    this->hide();
    QCoreApplication::processEvents();

    // ========== Step 3: 显示进度对话框 ==========
    m_cloudtree->showProgress("Supervoxel Clustering...");

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
                    printW("Supervoxel clustering canceled.");
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

    auto future = QtConcurrent::run([cloud, voxel_resolution, seed_resolution,
                                     color_importance, spatial_importance,
                                     normal_importance, camera_transform,
                                     cancel, on_progress]() {
        return ct::Segmentation::SupervoxelClustering(cloud,
            voxel_resolution, seed_resolution,
            color_importance, spatial_importance,
            normal_importance, camera_transform,
            cancel, on_progress);
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

            printI(QString("Supervoxel clustering done in %1 ms, %2 supervoxel(s) generated.")
                       .arg(result.time_ms)
                       .arg(result.clouds.size()));

            // 添加结果到点云树
            std::vector<ct::Cloud::Ptr> results;
            for (size_t i = 0; i < result.clouds.size(); i++) {
                auto& c = result.clouds[i];
                c->setId(cloud->id() + "-supervoxel" + std::to_string(i));
                c->makeAdaptive();
                results.push_back(c);
            }

            QString groupName = QString::fromStdString(cloud->id()) + "_Supervoxel";
            m_cloudtree->addResultGroup(cloud, results, groupName);

            this->accept();
        });

    watcher->setFuture(future);
}

void SupervoxelDialog::onCancel()
{
    this->close();
}
