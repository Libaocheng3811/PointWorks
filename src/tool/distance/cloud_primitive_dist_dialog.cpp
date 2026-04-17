#include "cloud_primitive_dist_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QCoreApplication>
#include <QtConcurrent>
#include <cmath>
#include <algorithm>

// ======================== Constructor ========================

CloudPrimitiveDistDialog::CloudPrimitiveDistDialog(QWidget* parent)
    : ct::CustomDialog(parent)
{
    setupUi();
}

// ======================== setupUi ========================

void CloudPrimitiveDistDialog::setupUi()
{
    setWindowTitle("Cloud / Primitive Distance");
    resize(380, 440);

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

    sel_layout->addWidget(new QLabel("Primitive Type:", this), 1, 0);
    cbox_primitive_ = new QComboBox(this);
    cbox_primitive_->addItem("Plane");
    cbox_primitive_->addItem("Sphere");
    sel_layout->addWidget(cbox_primitive_, 1, 1);

    main_layout->addWidget(sel_group);

    // --- Primitive Parameters ---
    param_pages_ = new QStackedWidget(this);
    param_pages_->addWidget(createPlaneParamPage());
    param_pages_->addWidget(createSphereParamPage());
    main_layout->addWidget(param_pages_);

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
    edit_field_name_->setText("C2P_Distance");
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
    connect(cbox_primitive_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &CloudPrimitiveDistDialog::onPrimitiveChanged);
    connect(btn_compute_, &QPushButton::clicked, this, &CloudPrimitiveDistDialog::onCompute);
    connect(btn_cancel_, &QPushButton::clicked, this, &QDialog::reject);
}

// ======================== Param Pages ========================

QWidget* CloudPrimitiveDistDialog::createPlaneParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QGridLayout(page);
    layout->setContentsMargins(0, 6, 0, 6);
    layout->setSpacing(4);

    layout->addWidget(new QLabel("Normal (a, b, c):", this), 0, 0);
    dspin_plane_a_ = new QDoubleSpinBox(this);
    dspin_plane_b_ = new QDoubleSpinBox(this);
    dspin_plane_c_ = new QDoubleSpinBox(this);
    for (auto* spin : {dspin_plane_a_, dspin_plane_b_, dspin_plane_c_}) {
        spin->setRange(-1e6, 1e6);
        spin->setDecimals(4);
        spin->setSingleStep(0.1);
    }
    dspin_plane_a_->setValue(0.0);
    dspin_plane_b_->setValue(0.0);
    dspin_plane_c_->setValue(1.0);
    layout->addWidget(dspin_plane_a_, 0, 1);
    layout->addWidget(dspin_plane_b_, 0, 2);
    layout->addWidget(dspin_plane_c_, 0, 3);

    layout->addWidget(new QLabel("Offset d:", this), 1, 0);
    dspin_plane_d_ = new QDoubleSpinBox(this);
    dspin_plane_d_->setRange(-1e9, 1e9);
    dspin_plane_d_->setDecimals(4);
    dspin_plane_d_->setValue(0.0);
    layout->addWidget(dspin_plane_d_, 1, 1);

    return page;
}

QWidget* CloudPrimitiveDistDialog::createSphereParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QGridLayout(page);
    layout->setContentsMargins(0, 6, 0, 6);
    layout->setSpacing(4);

    layout->addWidget(new QLabel("Center (x, y, z):", this), 0, 0);
    dspin_sphere_cx_ = new QDoubleSpinBox(this);
    dspin_sphere_cy_ = new QDoubleSpinBox(this);
    dspin_sphere_cz_ = new QDoubleSpinBox(this);
    for (auto* spin : {dspin_sphere_cx_, dspin_sphere_cy_, dspin_sphere_cz_}) {
        spin->setRange(-1e9, 1e9);
        spin->setDecimals(4);
        spin->setSingleStep(0.1);
        spin->setValue(0.0);
    }
    layout->addWidget(dspin_sphere_cx_, 0, 1);
    layout->addWidget(dspin_sphere_cy_, 0, 2);
    layout->addWidget(dspin_sphere_cz_, 0, 3);

    layout->addWidget(new QLabel("Radius:", this), 1, 0);
    dspin_sphere_r_ = new QDoubleSpinBox(this);
    dspin_sphere_r_->setRange(0.0, 1e9);
    dspin_sphere_r_->setDecimals(4);
    dspin_sphere_r_->setValue(1.0);
    layout->addWidget(dspin_sphere_r_, 1, 1);

    return page;
}

// ======================== init ========================

void CloudPrimitiveDistDialog::init()
{
    cbox_source_->clear();
    auto allClouds = m_cloudtree->getAllClouds();
    for (const auto& cloud : allClouds) {
        QString id = QString::fromStdString(cloud->id());
        cbox_source_->addItem(id, QVariant::fromValue(cloud));
    }

    // Smart select
    auto selected = m_cloudtree->getSelectedClouds();
    if (!selected.empty()) {
        int idx = cbox_source_->findText(QString::fromStdString(selected[0]->id()));
        if (idx >= 0) cbox_source_->setCurrentIndex(idx);
    }
}

// ======================== reset ========================

void CloudPrimitiveDistDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_source_cloud.reset();
}

// ======================== Slots ========================

void CloudPrimitiveDistDialog::onPrimitiveChanged(int index)
{
    param_pages_->setCurrentIndex(index);

    // Update default field name
    QString name;
    switch (index) {
        case 0: name = "C2P_Plane_Distance"; break;
        case 1: name = "C2P_Sphere_Distance"; break;
        default: name = "C2P_Distance"; break;
    }
    edit_field_name_->setText(name);
}

void CloudPrimitiveDistDialog::onCompute()
{
    m_source_cloud = cbox_source_->currentData().value<ct::Cloud::Ptr>();

    if (!m_source_cloud) {
        printW("Please select a compare cloud.");
        return;
    }

    ct::C2PParams params;
    // No max_distance spinbox for C2P — analytical computation is fast

    switch (cbox_primitive_->currentIndex()) {
        case 0: // Plane
            params.primitive_type = ct::PrimitiveType::PLANE;
            params.plane_params.a = dspin_plane_a_->value();
            params.plane_params.b = dspin_plane_b_->value();
            params.plane_params.c = dspin_plane_c_->value();
            params.plane_params.d = dspin_plane_d_->value();
            break;
        case 1: // Sphere
            params.primitive_type = ct::PrimitiveType::SPHERE;
            params.sphere_params.cx = dspin_sphere_cx_->value();
            params.sphere_params.cy = dspin_sphere_cy_->value();
            params.sphere_params.cz = dspin_sphere_cz_->value();
            params.sphere_params.radius = dspin_sphere_r_->value();
            break;
    }

    QString field_name = edit_field_name_->text().trimmed();
    if (field_name.isEmpty()) field_name = "C2P_Distance";
    bool apply_color = check_color_map_->isChecked();

    // ---- Step 1: Hide dialog ----
    this->hide();
    QCoreApplication::processEvents();

    // ---- Step 2: Show progress ----
    m_cloudtree->showProgress("Computing C2P distance...");

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
                    printW("C2P distance computation canceled.");
                });
    }

    // ---- Step 4: Progress callback ----
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ---- Step 5: Run in background ----
    auto source = m_source_cloud;
    m_canceled.store(false);

    auto future = QtConcurrent::run([source, params, cancel, on_progress]() {
        return ct::DistanceCalculator::calculateC2P(source, params, cancel, on_progress);
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

            printI(QString("C2P distance computed in %1 ms. Points: %2, Valid: %3, Min: %4, Max: %5, Mean: %6")
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

