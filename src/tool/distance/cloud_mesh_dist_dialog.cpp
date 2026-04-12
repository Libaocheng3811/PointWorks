#include "cloud_mesh_dist_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QCoreApplication>
#include <QtConcurrent>
#include <cmath>
#include <algorithm>

// ======================== Constructor ========================

CloudMeshDistDialog::CloudMeshDistDialog(QWidget* parent)
    : ct::CustomDialog(parent)
{
    setupUi();
}

// ======================== setupUi ========================

void CloudMeshDistDialog::setupUi()
{
    setWindowTitle("Cloud / Mesh Distance");
    resize(380, 400);

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(10, 10, 10, 10);
    main_layout->setSpacing(8);

    // --- Selection ---
    auto* sel_group = new QGroupBox("Selection", this);
    auto* sel_layout = new QGridLayout(sel_group);
    sel_layout->setContentsMargins(6, 10, 6, 6);
    sel_layout->setSpacing(4);

    sel_layout->addWidget(new QLabel("Compare Cloud:", this), 0, 0);
    cbox_source_ = new QComboBox(this);
    sel_layout->addWidget(cbox_source_, 0, 1);

    sel_layout->addWidget(new QLabel("Reference Mesh:", this), 1, 0);
    cbox_mesh_ = new QComboBox(this);
    sel_layout->addWidget(cbox_mesh_, 1, 1);

    main_layout->addWidget(sel_group);

    // --- Parameters ---
    auto* param_group = new QGroupBox("Parameters", this);
    auto* param_layout = new QVBoxLayout(param_group);
    param_layout->setContentsMargins(6, 10, 6, 6);
    param_layout->setSpacing(4);

    check_signed_ = new QCheckBox("Signed Distance", this);
    check_signed_->setChecked(true);
    param_layout->addWidget(check_signed_);

    auto* sign_layout = new QHBoxLayout();
    auto* sign_group = new QButtonGroup(this);
    radio_outside_ = new QRadioButton("Positive = Outside", this);
    radio_inside_ = new QRadioButton("Positive = Inside", this);
    sign_group->addButton(radio_outside_, 0);
    sign_group->addButton(radio_inside_, 1);
    radio_outside_->setChecked(true);
    sign_layout->addWidget(radio_outside_);
    sign_layout->addWidget(radio_inside_);
    sign_layout->addStretch();
    param_layout->addLayout(sign_layout);

    check_limit_dist_ = new QCheckBox("Limit Search Distance", this);
    param_layout->addWidget(check_limit_dist_);

    auto* max_layout = new QHBoxLayout();
    max_layout->setContentsMargins(20, 0, 0, 0);
    dspin_max_dist_ = new QDoubleSpinBox(this);
    dspin_max_dist_->setRange(0.001, 1e9);
    dspin_max_dist_->setDecimals(3);
    dspin_max_dist_->setValue(1.0);
    max_layout->addWidget(dspin_max_dist_);
    max_layout->addStretch();
    param_layout->addLayout(max_layout);
    dspin_max_dist_->setEnabled(false);

    main_layout->addWidget(param_group);

    // --- Output ---
    auto* out_group = new QGroupBox("Output", this);
    auto* out_layout = new QGridLayout(out_group);
    out_layout->setContentsMargins(6, 10, 6, 6);
    out_layout->setSpacing(4);

    check_color_map_ = new QCheckBox("Apply Jet Color Map", this);
    check_color_map_->setChecked(true);
    out_layout->addWidget(check_color_map_, 0, 0, 1, 2);

    out_layout->addWidget(new QLabel("Scalar Field Name:", this), 1, 0);
    edit_field_name_ = new QLineEdit(this);
    edit_field_name_->setText("C2M_Distance");
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
    connect(check_signed_, &QCheckBox::toggled, this, &CloudMeshDistDialog::onSignedToggled);
    connect(check_limit_dist_, &QCheckBox::toggled, dspin_max_dist_, &QDoubleSpinBox::setEnabled);
    connect(btn_compute_, &QPushButton::clicked, this, &CloudMeshDistDialog::onCompute);
    connect(btn_cancel_, &QPushButton::clicked, this, &QDialog::reject);

    // Initial state
    onSignedToggled(true);
}

// ======================== init ========================

void CloudMeshDistDialog::init()
{
    populateComboBoxes();
}

// ======================== reset ========================

void CloudMeshDistDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_source_cloud.reset();
    m_target_mesh.reset();
}

// ======================== Slots ========================

void CloudMeshDistDialog::onSignedToggled(bool checked)
{
    radio_outside_->setEnabled(checked);
    radio_inside_->setEnabled(checked);
}

void CloudMeshDistDialog::onCompute()
{
    m_source_cloud = cbox_source_->currentData().value<ct::Cloud::Ptr>();

    // Look up target mesh by ID from cloudtree
    QString meshId = cbox_mesh_->currentData().toString();
    m_target_mesh.reset();
    auto meshes = m_cloudtree->getLoadedMeshes();
    for (const auto& [id, mesh] : meshes) {
        if (id == meshId) { m_target_mesh = mesh; break; }
    }

    if (!m_source_cloud) {
        printW("Please select a compare cloud.");
        return;
    }
    if (!m_target_mesh || m_target_mesh->polygons.empty()) {
        printW("Please select a valid reference mesh.");
        return;
    }

    ct::C2MParams params;
    params.max_distance = check_limit_dist_->isChecked() ? dspin_max_dist_->value() : 0.0;
    params.signed_distance = check_signed_->isChecked();
    params.flip_normals = radio_inside_->isChecked();

    QString field_name = edit_field_name_->text().trimmed();
    if (field_name.isEmpty()) field_name = "C2M_Distance";
    bool apply_color = check_color_map_->isChecked();

    // ---- Step 1: Hide dialog ----
    this->hide();
    QCoreApplication::processEvents();

    // ---- Step 2: Show progress ----
    m_cloudtree->showProgress("Computing C2M distance...");

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
                    printW("C2M distance computation canceled.");
                });
    }

    // ---- Step 4: Progress callback ----
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ---- Step 5: Run in background ----
    auto source = m_source_cloud;
    auto mesh = m_target_mesh;
    m_canceled.store(false);

    auto future = QtConcurrent::run([source, mesh, params, cancel, on_progress]() {
        return ct::DistanceCalculator::calculateC2M(source, mesh, params, cancel, on_progress);
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

            // Attach scalar field to compare cloud
            m_source_cloud->addScalarField(field_name.toStdString(), result.distances);

            if (apply_color) {
                m_source_cloud->updateColorByField(field_name.toStdString());
                m_source_cloud->setHasColors(true);
            }

            // Refresh rendering
            QString cloudId = QString::fromStdString(m_source_cloud->id());
            m_cloudview->invalidateCloudRender(cloudId);
            m_cloudview->refresh();

            // Statistics
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

            printI(QString("C2M distance computed in %1 ms. Points: %2, Valid: %3, Min: %4, Max: %5, Mean: %6")
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

void CloudMeshDistDialog::populateComboBoxes()
{
    cbox_source_->clear();
    cbox_mesh_->clear();

    // Populate source clouds
    auto allClouds = m_cloudtree->getAllClouds();
    for (const auto& cloud : allClouds) {
        QString id = QString::fromStdString(cloud->id());
        cbox_source_->addItem(id, QVariant::fromValue(cloud));
    }

    // Populate target meshes (store ID as string, look up mesh at compute time)
    auto meshes = m_cloudtree->getLoadedMeshes();
    for (const auto& [id, mesh] : meshes) {
        cbox_mesh_->addItem(id, id);  // store mesh ID, not pointer
    }

    if (meshes.isEmpty()) {
        cbox_mesh_->addItem("(No mesh loaded)", QVariant());
    }
}
