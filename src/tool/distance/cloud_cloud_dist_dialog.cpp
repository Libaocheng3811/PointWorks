#include "cloud_cloud_dist_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QCoreApplication>
#include <QtConcurrent>
#include <cmath>
#include <algorithm>

// ======================== Constructor ========================

CloudCloudDistDialog::CloudCloudDistDialog(QWidget* parent)
    : ct::CustomDialog(parent)
{
    setupUi();
}

// ======================== setupUi ========================

void CloudCloudDistDialog::setupUi()
{
    setWindowTitle("Cloud / Cloud Distance");
    resize(380, 480);

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(10, 10, 10, 10);
    main_layout->setSpacing(8);

    // --- Cloud Selection ---
    auto* sel_group = new QGroupBox("Cloud Selection", this);
    auto* sel_layout = new QGridLayout(sel_group);
    sel_layout->setContentsMargins(6, 10, 6, 6);
    sel_layout->setSpacing(4);

    sel_layout->addWidget(new QLabel("Compare Cloud:", this), 0, 0);
    cbox_source_ = new QComboBox(this);
    sel_layout->addWidget(cbox_source_, 0, 1);

    sel_layout->addWidget(new QLabel("Reference Cloud:", this), 1, 0);
    cbox_target_ = new QComboBox(this);
    sel_layout->addWidget(cbox_target_, 1, 1);

    main_layout->addWidget(sel_group);

    // --- Method Selection ---
    auto* method_group = new QGroupBox("Distance Method", this);
    auto* method_layout = new QVBoxLayout(method_group);
    method_layout->setContentsMargins(6, 10, 6, 6);
    method_layout->setSpacing(4);

    method_group_ = new QButtonGroup(this);
    radio_nearest_ = new QRadioButton("Nearest Neighbor", this);
    radio_knn_ = new QRadioButton("K-Nearest Neighbors Mean", this);
    radio_radius_ = new QRadioButton("Radius Mean", this);
    method_group_->addButton(radio_nearest_, 0);
    method_group_->addButton(radio_knn_, 1);
    method_group_->addButton(radio_radius_, 2);
    radio_nearest_->setChecked(true);

    method_layout->addWidget(radio_nearest_);
    method_layout->addWidget(radio_knn_);
    method_layout->addWidget(radio_radius_);

    // Max distance (shared)
    check_limit_dist_ = new QCheckBox("Limit Search Distance", this);
    method_layout->addWidget(check_limit_dist_);

    auto* max_dist_layout = new QHBoxLayout();
    max_dist_layout->setContentsMargins(20, 0, 0, 0);
    dspin_max_dist_ = new QDoubleSpinBox(this);
    dspin_max_dist_->setRange(0.001, 1e9);
    dspin_max_dist_->setDecimals(3);
    dspin_max_dist_->setValue(1.0);
    max_dist_layout->addWidget(dspin_max_dist_);
    max_dist_layout->addStretch();
    method_layout->addLayout(max_dist_layout);
    dspin_max_dist_->setEnabled(false);

    main_layout->addWidget(method_group);

    // --- Method-specific parameters (StackedWidget) ---
    param_pages_ = new QStackedWidget(this);

    // Page 0: Nearest (no extra params)
    page_nearest_ = new QWidget(this);
    param_pages_->addWidget(page_nearest_);

    // Page 1: KNN params
    page_knn_ = new QWidget(this);
    auto* knn_layout = new QHBoxLayout(page_knn_);
    knn_layout->setContentsMargins(0, 0, 0, 0);
    knn_layout->addWidget(new QLabel("K Neighbors:", this));
    spin_k_ = new QSpinBox(this);
    spin_k_->setRange(1, 100);
    spin_k_->setValue(6);
    knn_layout->addWidget(spin_k_);
    knn_layout->addStretch();
    param_pages_->addWidget(page_knn_);

    // Page 2: Radius params
    page_radius_ = new QWidget(this);
    auto* radius_layout = new QHBoxLayout(page_radius_);
    radius_layout->setContentsMargins(0, 0, 0, 0);
    radius_layout->addWidget(new QLabel("Search Radius:", this));
    dspin_radius_ = new QDoubleSpinBox(this);
    dspin_radius_->setRange(0.001, 1000.0);
    dspin_radius_->setDecimals(4);
    dspin_radius_->setValue(0.5);
    radius_layout->addWidget(dspin_radius_);
    radius_layout->addStretch();
    param_pages_->addWidget(page_radius_);

    main_layout->addWidget(param_pages_);

    // --- Output Options ---
    auto* out_group = new QGroupBox("Output", this);
    auto* out_layout = new QGridLayout(out_group);
    out_layout->setContentsMargins(6, 10, 6, 6);
    out_layout->setSpacing(4);

    check_color_map_ = new QCheckBox("Apply Jet Color Map", this);
    check_color_map_->setChecked(true);
    out_layout->addWidget(check_color_map_, 0, 0, 1, 2);

    out_layout->addWidget(new QLabel("Scalar Field Name:", this), 1, 0);
    edit_field_name_ = new QLineEdit(this);
    edit_field_name_->setText("C2C_Distance");
    out_layout->addWidget(edit_field_name_, 1, 1);

    main_layout->addWidget(out_group);

    // --- Buttons ---
    auto* btn_layout = new QHBoxLayout();
    btn_compute_ = new QPushButton("Compute", this);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_layout->addStretch();
    btn_layout->addWidget(btn_compute_);
    btn_layout->addStretch();
    btn_layout->addWidget(btn_cancel_);
    btn_layout->addStretch();
    main_layout->addLayout(btn_layout);

    // --- Connections ---
    connect(method_group_, QOverload<int>::of(&QButtonGroup::idClicked),
            this, &CloudCloudDistDialog::onMethodChanged);
    connect(check_limit_dist_, &QCheckBox::toggled, dspin_max_dist_, &QDoubleSpinBox::setEnabled);
    connect(btn_compute_, &QPushButton::clicked, this, &CloudCloudDistDialog::onCompute);
    connect(btn_cancel_, &QPushButton::clicked, this, &QDialog::reject);
}

// ======================== init ========================

void CloudCloudDistDialog::init()
{
    populateComboBoxes();
    autoSelectClouds();
}

// ======================== reset ========================

void CloudCloudDistDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_source_cloud.reset();
    m_target_cloud.reset();
}

// ======================== Slots ========================

void CloudCloudDistDialog::onMethodChanged(int id)
{
    param_pages_->setCurrentIndex(id);
}

void CloudCloudDistDialog::onCompute()
{
    // Compare = source combo, Reference = target combo
    m_source_cloud = cbox_source_->currentData().value<ct::Cloud::Ptr>();
    m_target_cloud = cbox_target_->currentData().value<ct::Cloud::Ptr>();

    if (!m_source_cloud || !m_target_cloud) {
        printW("Please select both compare and reference clouds.");
        return;
    }
    if (m_source_cloud == m_target_cloud) {
        printW("Compare and reference must be different clouds.");
        return;
    }

    // Build params
    ct::C2CParams params;
    params.max_distance = check_limit_dist_->isChecked() ? dspin_max_dist_->value() : 0.0;

    switch (method_group_->checkedId()) {
        case 0:
            params.method = ct::C2CParams::C2C_NEAREST;
            break;
        case 1:
            params.method = ct::C2CParams::C2C_KNN_MEAN;
            params.k_knn = spin_k_->value();
            break;
        case 2:
            params.method = ct::C2CParams::C2C_RADIUS_MEAN;
            params.radius = dspin_radius_->value();
            break;
    }

    QString field_name = edit_field_name_->text().trimmed();
    if (field_name.isEmpty()) field_name = "C2C_Distance";
    bool apply_color = check_color_map_->isChecked();

    // ---- Step 1: Hide dialog ----
    this->hide();
    QCoreApplication::processEvents();

    // ---- Step 2: Show progress ----
    m_cloudtree->showProgress("Computing C2C distance...");

    // ---- Step 3: Setup cancel ----
    auto* cancel = new std::atomic<bool>(false);
    auto* progress_closed = new std::atomic<bool>(false);
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [=]() {
                    *cancel = true;
                    m_canceled.store(true);
                    m_cloudtree->closeProgress();
                    progress_closed->store(true);
                    printW("C2C distance computation canceled.");
                });
    }

    // ---- Step 4: Progress callback ----
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ---- Step 5: Run in background ----
    // compare cloud = source, reference cloud = target
    // Algorithm: for each point in compare, find nearest in reference
    auto compare = m_source_cloud;
    auto reference = m_target_cloud;
    m_canceled.store(false);

    auto future = QtConcurrent::run([reference, compare, params, cancel, on_progress]() {
        return ct::DistanceCalculator::calculateC2C(reference, compare, params, cancel, on_progress);
    });

    // ---- Step 6: Handle result ----
    auto* watcher = new QFutureWatcher<ct::DistanceResult>(this);
    connect(watcher, &QFutureWatcher<ct::DistanceResult>::finished, this,
        [=]() {
            if (!progress_closed->load()) {
                m_cloudtree->closeProgress();
            }
            delete cancel;
            delete progress_closed;

            auto result = watcher->result();
            watcher->deleteLater();

            if (m_canceled.load() || !result.success) {
                if (!result.error_msg.empty()) {
                    printE(QString::fromStdString(result.error_msg));
                }
                this->reject();
                return;
            }

            // Attach scalar field to COMPARE cloud
            m_source_cloud->addScalarField(field_name.toStdString(), result.distances);

            if (apply_color) {
                m_source_cloud->updateColorByField(field_name.toStdString());
                m_source_cloud->setHasColors(true);
            }

            // Refresh rendering
            QString cloudId = QString::fromStdString(m_source_cloud->id());
            m_cloudview->invalidateCloudRender(cloudId);
            m_cloudview->refresh();

            // Compute statistics
            float min_d = std::numeric_limits<float>::max();
            float max_d = -std::numeric_limits<float>::max();
            double sum = 0;
            int count = 0;
            for (float d : result.distances) {
                if (!std::isnan(d)) {
                    min_d = std::min(min_d, d);
                    max_d = std::max(max_d, d);
                    sum += d;
                    ++count;
                }
            }
            double mean_d = (count > 0) ? (sum / count) : 0;

            printI(QString("C2C distance computed in %1 ms. Points: %2, Valid: %3, Min: %4, Max: %5, Mean: %6")
                       .arg(result.time_ms, 0, 'f', 1)
                       .arg(m_source_cloud->size())
                       .arg(count)
                       .arg(min_d, 0, 'f', 4)
                       .arg(max_d, 0, 'f', 4)
                       .arg(mean_d, 0, 'f', 4));

            this->accept();
        });

    watcher->setFuture(future);
}

// ======================== Helpers ========================

void CloudCloudDistDialog::populateComboBoxes()
{
    cbox_source_->clear();
    cbox_target_->clear();

    auto allClouds = m_cloudtree->getAllClouds();
    for (const auto& cloud : allClouds) {
        QString id = QString::fromStdString(cloud->id());
        cbox_source_->addItem(id, QVariant::fromValue(cloud));
        cbox_target_->addItem(id, QVariant::fromValue(cloud));
    }
}

void CloudCloudDistDialog::autoSelectClouds()
{
    auto selected = m_cloudtree->getSelectedClouds();
    if (selected.size() >= 2) {
        int idx0 = cbox_source_->findText(QString::fromStdString(selected[0]->id()));
        int idx1 = cbox_target_->findText(QString::fromStdString(selected[1]->id()));
        if (idx0 >= 0) cbox_source_->setCurrentIndex(idx0);
        if (idx1 >= 0) cbox_target_->setCurrentIndex(idx1);
    } else if (selected.size() == 1 && cbox_source_->count() >= 2) {
        int idx0 = cbox_source_->findText(QString::fromStdString(selected[0]->id()));
        if (idx0 >= 0) cbox_source_->setCurrentIndex(idx0);
        for (int i = 0; i < cbox_target_->count(); ++i) {
            if (i != idx0) { cbox_target_->setCurrentIndex(i); break; }
        }
    }
}
