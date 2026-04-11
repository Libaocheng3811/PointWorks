#include "extract_boundary_dialog.h"

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

// ======================== Constructor ========================

ExtractBoundaryDialog::ExtractBoundaryDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_canceled(false)
{
    setupUi();
    this->setWindowTitle("Extract Boundary");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(320, 260);
    QTimer::singleShot(0, this, [this]() { this->adjustSize(); });
}

ExtractBoundaryDialog::~ExtractBoundaryDialog() = default;

// ======================== setupUi ========================

void ExtractBoundaryDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- Parameters ---
    auto* param_group = new QGroupBox("Parameters", this);
    auto* param_layout = new QFormLayout(param_group);
    param_layout->setContentsMargins(6, 10, 6, 6);
    param_layout->setSpacing(4);

    spin_k_ = new QSpinBox(this);
    spin_k_->setRange(1, 100);
    spin_k_->setValue(30);
    param_layout->addRow("K Neighbors:", spin_k_);

    dspin_search_radius_ = new QDoubleSpinBox(this);
    dspin_search_radius_->setRange(0.0, 1000.0);
    dspin_search_radius_->setDecimals(4);
    dspin_search_radius_->setValue(0.0);
    dspin_search_radius_->setSingleStep(0.01);
    param_layout->addRow("Search Radius (0=auto):", dspin_search_radius_);

    dspin_angle_threshold_ = new QDoubleSpinBox(this);
    dspin_angle_threshold_->setRange(0.0, 180.0);
    dspin_angle_threshold_->setDecimals(1);
    dspin_angle_threshold_->setValue(45.0);
    dspin_angle_threshold_->setSingleStep(5.0);
    param_layout->addRow("Angle Threshold (°):", dspin_angle_threshold_);

    main_layout->addWidget(param_group);

    // --- Output Mode ---
    auto* output_group = new QGroupBox("Output Mode", this);
    auto* output_layout = new QVBoxLayout(output_group);
    output_layout->setContentsMargins(6, 10, 6, 6);
    output_layout->setSpacing(4);

    auto* btn_group = new QButtonGroup(this);
    radio_output_polyline_ = new QRadioButton("Create Polyline (recommended)", this);
    radio_output_select_ = new QRadioButton("Select Boundary Points only", this);
    btn_group->addButton(radio_output_polyline_, 0);
    btn_group->addButton(radio_output_select_, 1);
    radio_output_polyline_->setChecked(true);
    output_layout->addWidget(radio_output_polyline_);
    output_layout->addWidget(radio_output_select_);

    main_layout->addWidget(output_group);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_extract_ = new QPushButton("Extract", this);
    btn_close_ = new QPushButton("Close", this);
    btn_row->addWidget(btn_extract_);
    btn_row->addWidget(btn_close_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(btn_extract_, &QPushButton::clicked, this, &ExtractBoundaryDialog::onExtract);
    connect(btn_close_, &QPushButton::clicked, this, &ExtractBoundaryDialog::onCancel);
}

// ======================== Init / Reset ========================

void ExtractBoundaryDialog::init()
{
    auto selection = m_cloudtree->getSelectedClouds();
    if (!selection.empty()) {
        m_cloud = selection.front();
    }
}

void ExtractBoundaryDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_cloud.reset();
}

// ======================== Slots ========================

void ExtractBoundaryDialog::onExtract()
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    if (!m_cloud->hasNormals()) {
        printW("Selected cloud has no normals. Please estimate normals first.");
        return;
    }

    int k = spin_k_->value();
    double radius = dspin_search_radius_->value();
    double angle = dspin_angle_threshold_->value();

    if (k == 0 && radius == 0) {
        printW("Please set K or Search Radius.");
        return;
    }

    bool output_polyline = radio_output_polyline_->isChecked();

    // ========== Step 1: 隐藏对话框 ==========
    this->hide();
    QCoreApplication::processEvents();

    // ========== Step 2: 显示进度条 ==========
    m_cloudtree->showProgress("Extract Boundary...");

    // ========== Step 3: 设置取消标志 ==========
    auto* cancel = new std::atomic<bool>(false);
    auto* progress_closed = new std::atomic<bool>(false);
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [=]() {
                    *cancel = true;
                    m_canceled.store(true);
                    m_cloudtree->closeProgress();
                    progress_closed->store(true);
                    printW("Extract Boundary canceled.");
                });
    }

    // ========== Step 4: 进度回调 ==========
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ========== Step 5: 提交算法到工作线程 ==========
    auto cloud = m_cloud;
    m_canceled.store(false);

    auto future = QtConcurrent::run([cloud, k, radius, angle, cancel, on_progress]() -> ct::Cloud::Ptr {
        return ct::Features::BoundaryEstimation(cloud, k, radius, angle, cancel, on_progress);
    });

    // ========== Step 6: 监听完成信号 ==========
    auto* watcher = new QFutureWatcher<ct::Cloud::Ptr>(this);
    connect(watcher, &QFutureWatcher<ct::Cloud::Ptr>::finished, this,
        [=]() {
            if (!progress_closed->load()) {
                m_cloudtree->closeProgress();
            }
            delete cancel;
            delete progress_closed;

            auto result = watcher->result();
            watcher->deleteLater();

            if (m_canceled.load() || !result) {
                if (!result && !m_canceled.load()) {
                    printW("Extract Boundary failed.");
                }
                this->reject();
                return;
            }

            if (output_polyline) {
                // 创建折线模式：作为新点云加载到对象树
                result->setId(cloud->id() + "_boundary");
                result->makeAdaptive();
                m_cloudview->addPointCloud(result);
                m_cloudview->setPointCloudColor(QString::fromStdString(result->id()), ct::Color::Green);

                printI(QString("Extract Boundary done, boundary cloud [%1] added.")
                           .arg(QString::fromStdString(result->id())));
            } else {
                // 仅选中边界点模式：高亮显示边界点
                result->setId(cloud->id() + "_boundary");
                result->makeAdaptive();
                m_cloudview->addPointCloud(result);
                m_cloudview->setPointCloudColor(QString::fromStdString(result->id()), ct::Color::Red);

                printI(QString("Extract Boundary done, %1 boundary points selected.")
                           .arg(result->size()));
            }

            this->accept();
        });

    watcher->setFuture(future);
}

void ExtractBoundaryDialog::onCancel()
{
    this->close();
}
