#include "morphological_filter_dialog.h"

#include "algorithm/segmentation.h"
#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "viz/console.h"
#include "base/progress_manager.h"

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
    btn_apply_ = new QPushButton("Apply", this);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(btn_apply_, &QPushButton::clicked, this, &MorphologicalFilterDialog::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &MorphologicalFilterDialog::onCancel);
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
    m_progress->closeProgress();
}

void MorphologicalFilterDialog::deinit()
{
    reset();
}

// ======================== Slots ========================

void MorphologicalFilterDialog::onApply()
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    int max_window_size = spin_max_window_size_->value();
    float slope = dspin_slope_->value();
    float max_distance = dspin_max_distance_->value();
    float initial_distance = dspin_initial_distance_->value();
    float cell_size = dspin_cell_size_->value();
    float base = dspin_base_->value();
    bool negative = check_negative_->isChecked();

    this->hide();
    QCoreApplication::processEvents();

    m_progress->showProgress("Morphological Filter...");

    auto* cancel = new std::atomic<bool>(false);
    auto* progress_closed = new std::atomic<bool>(false);
    connect(m_progress, &ct::ProgressManager::cancelRequested,
            this, [=]() {
                *cancel = true;
                m_canceled.store(true);
                m_progress->closeProgress();
                progress_closed->store(true);
                printW("Morphological filter canceled.");
                this->reject();
            });

    QPointer<ct::ProcessingDialog> dialog = m_progress->dialog();
    auto on_progress = [dialog](int pct) {
        if (dialog)
            QMetaObject::invokeMethod(dialog.data(), "setProgress",
                                      Qt::QueuedConnection, Q_ARG(int, pct));
    };

    auto cloud = m_cloud;
    m_canceled.store(false);

    auto future = QtConcurrent::run([cloud, negative, max_window_size, slope,
                                     max_distance, initial_distance, cell_size, base,
                                     cancel, on_progress]() {
        return ct::Segmentation::MorphologicalFilter(cloud, negative,
            max_window_size, slope, max_distance, initial_distance,
            cell_size, base, cancel, on_progress);
    });

    auto* watcher = new QFutureWatcher<ct::SegmentationResult>(this);
    connect(watcher, &QFutureWatcher<ct::SegmentationResult>::finished, this,
        [=]() {
            if (!progress_closed->load()) {
                m_progress->closeProgress();
            }
            delete cancel;
            delete progress_closed;

            auto result = watcher->result();
            watcher->deleteLater();

            if (m_canceled.load() || result.clouds.empty()) {
                this->reject();
                return;
            }

            printI(QString("Morphological filter done in %1 ms.")
                       .arg(result.time_ms));

            for (size_t i = 0; i < result.clouds.size(); i++) {
                auto& c = result.clouds[i];
                c->setId("mf-" + cloud->id());
                c->makeAdaptive();
                m_cloudtree->addResultGroup(cloud, {c},
                    QString::fromStdString(cloud->id()) + "_MorphologicalFilter");
            }

            this->accept();
        });

    watcher->setFuture(future);
}

void MorphologicalFilterDialog::onCancel()
{
    this->close();
}
