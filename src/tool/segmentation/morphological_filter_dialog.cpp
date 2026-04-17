#include "morphological_filter_dialog.h"

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

// ======================== Constructor ========================

MorphologicalFilterDialog::MorphologicalFilterDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_canceled(false)
{
    setupUi();
    this->setWindowTitle("Morphological Filter");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(300, 320);
    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

MorphologicalFilterDialog::~MorphologicalFilterDialog()
{
    m_canceled.store(true);
    if (m_watcher && !m_watcher->isFinished()) {
        m_watcher->waitForFinished();
    }
}

// ======================== setupUi ========================

void MorphologicalFilterDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    auto* form = new QFormLayout();
    form->setContentsMargins(0, 0, 0, 0);
    form->setSpacing(4);

    spin_max_window_size_ = new QSpinBox(this);
    spin_max_window_size_->setRange(1, 200);
    spin_max_window_size_->setValue(33);
    form->addRow("Max Window Size:", spin_max_window_size_);

    dspin_slope_ = new QDoubleSpinBox(this);
    dspin_slope_->setRange(0.0, 100.0);
    dspin_slope_->setDecimals(2);
    dspin_slope_->setValue(1.0);
    dspin_slope_->setSingleStep(0.1);
    form->addRow("Slope:", dspin_slope_);

    dspin_max_distance_ = new QDoubleSpinBox(this);
    dspin_max_distance_->setRange(0.0, 1000.0);
    dspin_max_distance_->setDecimals(2);
    dspin_max_distance_->setValue(2.5);
    dspin_max_distance_->setSingleStep(0.1);
    form->addRow("Max Distance:", dspin_max_distance_);

    dspin_initial_distance_ = new QDoubleSpinBox(this);
    dspin_initial_distance_->setRange(0.0, 1000.0);
    dspin_initial_distance_->setDecimals(3);
    dspin_initial_distance_->setValue(0.15);
    dspin_initial_distance_->setSingleStep(0.01);
    form->addRow("Initial Distance:", dspin_initial_distance_);

    dspin_cell_size_ = new QDoubleSpinBox(this);
    dspin_cell_size_->setRange(0.01, 1000.0);
    dspin_cell_size_->setDecimals(2);
    dspin_cell_size_->setValue(1.0);
    dspin_cell_size_->setSingleStep(0.1);
    form->addRow("Cell Size:", dspin_cell_size_);

    dspin_base_ = new QDoubleSpinBox(this);
    dspin_base_->setRange(0.01, 100.0);
    dspin_base_->setDecimals(2);
    dspin_base_->setValue(2.0);
    dspin_base_->setSingleStep(0.1);
    form->addRow("Base:", dspin_base_);

    check_negative_ = new QCheckBox("Negative (invert)", this);
    form->addRow(check_negative_);

    main_layout->addLayout(form);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_preview_ = new QPushButton("Preview", this);
    btn_apply_ = new QPushButton("Add", this);
    btn_reset_ = new QPushButton("Reset", this);
    btn_row->addWidget(btn_preview_);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_reset_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(btn_preview_, &QPushButton::clicked, this, &MorphologicalFilterDialog::onPreview);
    connect(btn_apply_, &QPushButton::clicked, this, &MorphologicalFilterDialog::onApply);
    connect(btn_reset_, &QPushButton::clicked, this, &MorphologicalFilterDialog::onReset);
}

// ======================== Init / Reset ========================

void MorphologicalFilterDialog::init()
{
    auto selection = m_cloudtree->getSelectedClouds();
    if (!selection.empty()) {
        m_cloud = selection.front();
    }
}

void MorphologicalFilterDialog::reset()
{
    m_canceled.store(true);
    m_cloudtree->closeProgress();

    // 清除预览结果
    for (auto& c : m_preview_clouds) {
        m_cloudview->removePointCloud(QString::fromStdString(c->id()));
    }
    m_preview_clouds.clear();
    m_cloudview->refresh();
}

void MorphologicalFilterDialog::deinit()
{
    reset();
}

// ======================== Slots ========================

void MorphologicalFilterDialog::runFilter(bool preview)
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    // 清除上一次预览
    for (auto& c : m_preview_clouds) {
        m_cloudview->removePointCloud(QString::fromStdString(c->id()));
    }
    m_preview_clouds.clear();

    // 读取参数
    int max_window_size = spin_max_window_size_->value();
    float slope = dspin_slope_->value();
    float max_distance = dspin_max_distance_->value();
    float initial_distance = dspin_initial_distance_->value();
    float cell_size = dspin_cell_size_->value();
    float base = dspin_base_->value();
    bool negative = check_negative_->isChecked();

    m_cloudtree->showProgress("Morphological Filter...");

    auto* cancel = new std::atomic<bool>(false);
    auto* progress_closed = new std::atomic<bool>(false);
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [=]() {
                    *cancel = true;
                    m_canceled.store(true);
                    m_cloudtree->closeProgress();
                    progress_closed->store(true);
                    printW("Morphological filter canceled.");
                });
    }

    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    auto cloud = m_cloud;
    m_canceled.store(false);
    bool is_preview = preview;

    auto future = QtConcurrent::run([cloud, negative, max_window_size, slope,
                                     max_distance, initial_distance, cell_size, base,
                                     cancel, on_progress]() {
        return ct::Segmentation::MorphologicalFilter(cloud, negative,
            max_window_size, slope, max_distance, initial_distance,
            cell_size, base, cancel, on_progress);
    });

    if (m_watcher && !m_watcher->isFinished()) {
        m_canceled.store(true);
        m_watcher->waitForFinished();
    }

    m_watcher = new QFutureWatcher<ct::SegmentationResult>(this);
    connect(m_watcher, &QFutureWatcher<ct::SegmentationResult>::finished, this,
        [=]() {
            if (!progress_closed->load()) {
                m_cloudtree->closeProgress();
            }
            delete cancel;
            delete progress_closed;

            auto result = m_watcher->result();

            if (m_canceled.load() || result.clouds.empty()) {
                return;
            }

            printI(QString("Morphological filter done in %1 ms.")
                       .arg(result.time_ms));

            for (size_t i = 0; i < result.clouds.size(); i++) {
                auto& c = result.clouds[i];
                if (is_preview) {
                    c->setId(PREVIEW_PREFIX + std::to_string(i) + "-" + cloud->id());
                    m_preview_clouds.push_back(c);
                    m_cloudview->addPointCloud(c);
                } else {
                    c->setId("mf-" + cloud->id());
                    c->makeAdaptive();
                    m_cloudtree->addResultGroup(cloud, {c},
                        QString::fromStdString(cloud->id()) + "_MorphologicalFilter");
                }
            }
            m_cloudview->refresh();
        });

    m_watcher->setFuture(future);
}

void MorphologicalFilterDialog::onPreview()
{
    runFilter(true);
}

void MorphologicalFilterDialog::onApply()
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    // 清除预览
    for (auto& c : m_preview_clouds) {
        m_cloudview->removePointCloud(QString::fromStdString(c->id()));
    }
    m_preview_clouds.clear();

    runFilter(false);
}

void MorphologicalFilterDialog::onReset()
{
    reset();
    m_cloudview->refresh();
    printI("Morphological filter reset.");
}
