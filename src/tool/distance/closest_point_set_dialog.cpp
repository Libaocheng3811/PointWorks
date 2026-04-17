#include "closest_point_set_dialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QCoreApplication>
#include <QtConcurrent>

// ======================== Constructor ========================

ClosestPointSetDialog::ClosestPointSetDialog(QWidget* parent)
    : ct::CustomDialog(parent)
{
    setupUi();
}

// ======================== setupUi ========================

void ClosestPointSetDialog::setupUi()
{
    setWindowTitle("Closest Point Set");
    resize(380, 360);

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(10, 10, 10, 10);
    main_layout->setSpacing(8);

    // --- Cloud Selection ---
    auto* sel_group = new QGroupBox("Cloud Selection", this);
    auto* sel_layout = new QGridLayout(sel_group);
    sel_layout->setContentsMargins(6, 10, 6, 6);
    sel_layout->setSpacing(4);

    sel_layout->addWidget(new QLabel("Source Cloud:", this), 0, 0);
    cbox_source_ = new QComboBox(this);
    sel_layout->addWidget(cbox_source_, 0, 1);

    sel_layout->addWidget(new QLabel("Target Cloud/Mesh:", this), 1, 0);
    cbox_target_ = new QComboBox(this);
    sel_layout->addWidget(cbox_target_, 1, 1);

    main_layout->addWidget(sel_group);

    // --- Attributes ---
    auto* attr_group = new QGroupBox("Attributes to Keep", this);
    auto* attr_layout = new QVBoxLayout(attr_group);
    attr_layout->setContentsMargins(6, 10, 6, 6);
    attr_layout->setSpacing(4);

    check_keep_colors_ = new QCheckBox("Keep Original Colors", this);
    check_keep_colors_->setChecked(true);
    attr_layout->addWidget(check_keep_colors_);

    check_keep_intensity_ = new QCheckBox("Keep Original Intensity", this);
    attr_layout->addWidget(check_keep_intensity_);

    check_keep_scalars_ = new QCheckBox("Keep Scalar Fields", this);
    attr_layout->addWidget(check_keep_scalars_);

    check_limit_dist_ = new QCheckBox("Limit Search Distance", this);
    attr_layout->addWidget(check_limit_dist_);

    auto* max_layout = new QHBoxLayout();
    max_layout->setContentsMargins(20, 0, 0, 0);
    dspin_max_dist_ = new QDoubleSpinBox(this);
    dspin_max_dist_->setRange(0.001, 1e9);
    dspin_max_dist_->setDecimals(3);
    dspin_max_dist_->setValue(1.0);
    max_layout->addWidget(dspin_max_dist_);
    max_layout->addStretch();
    attr_layout->addLayout(max_layout);
    dspin_max_dist_->setEnabled(false);

    main_layout->addWidget(attr_group);

    // --- Output ---
    auto* out_group = new QGroupBox("Output", this);
    auto* out_layout = new QGridLayout(out_group);
    out_layout->setContentsMargins(6, 10, 6, 6);
    out_layout->setSpacing(4);

    out_layout->addWidget(new QLabel("Output Name:", this), 0, 0);
    edit_output_name_ = new QLineEdit(this);
    out_layout->addWidget(edit_output_name_, 0, 1);

    main_layout->addWidget(out_group);

    // --- Buttons ---
    auto* btn_layout = new QHBoxLayout();
    btn_extract_ = new QPushButton("Extract", this);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_layout->addStretch();
    btn_layout->addWidget(btn_extract_);
    btn_layout->addStretch();
    btn_layout->addWidget(btn_cancel_);
    btn_layout->addStretch();
    main_layout->addLayout(btn_layout);

    // --- Connections ---
    connect(cbox_source_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ClosestPointSetDialog::onTargetChanged);
    connect(cbox_target_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ClosestPointSetDialog::onTargetChanged);
    connect(check_limit_dist_, &QCheckBox::toggled, dspin_max_dist_, &QDoubleSpinBox::setEnabled);
    connect(btn_extract_, &QPushButton::clicked, this, &ClosestPointSetDialog::onExtract);
    connect(btn_cancel_, &QPushButton::clicked, this, &QDialog::reject);
}

// ======================== init ========================

void ClosestPointSetDialog::init()
{
    populateComboBoxes();
    updateOutputName();
}

// ======================== reset ========================

void ClosestPointSetDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_source_cloud.reset();
    m_target_cloud.reset();
}

// ======================== Slots ========================

void ClosestPointSetDialog::onTargetChanged(int /*index*/)
{
    updateOutputName();
}

void ClosestPointSetDialog::onExtract()
{
    m_source_cloud = cbox_source_->currentData().value<ct::Cloud::Ptr>();
    m_target_cloud = cbox_target_->currentData().value<ct::Cloud::Ptr>();

    if (!m_source_cloud || !m_target_cloud) {
        printW("Please select both source and target clouds.");
        return;
    }
    if (m_source_cloud == m_target_cloud) {
        printW("Source and target must be different clouds.");
        return;
    }

    ct::CPSParams params;
    params.max_distance = check_limit_dist_->isChecked() ? dspin_max_dist_->value() : 0.0;
    params.keep_colors = check_keep_colors_->isChecked();
    params.keep_intensity = check_keep_intensity_->isChecked();
    params.keep_scalar_fields = check_keep_scalars_->isChecked();

    QString output_name = edit_output_name_->text().trimmed();
    if (output_name.isEmpty()) output_name = "CPS_Result";

    // ---- Step 1: Hide dialog ----
    this->hide();
    QCoreApplication::processEvents();

    // ---- Step 2: Show progress ----
    m_cloudtree->showProgress("Extracting closest point set...");

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
                    printW("Closest point set extraction canceled.");
                });
    }

    // ---- Step 4: Progress callback ----
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ---- Step 5: Run in background ----
    auto source = m_source_cloud;
    auto target = m_target_cloud;
    m_canceled.store(false);

    auto future = QtConcurrent::run([source, target, params, cancel, on_progress]() {
        return ct::DistanceCalculator::extractClosestPoints(source, target, params, cancel, on_progress);
    });

    // ---- Step 6: Handle result ----
    auto* watcher = new QFutureWatcher<ct::CPSResult>(this);
    connect(watcher, &QFutureWatcher<ct::CPSResult>::finished, this,
        [=]() {
            if (!progress_closed->load()) {
                m_cloudtree->closeProgress();
            }
            delete cancel;
            delete progress_closed;

            auto result = watcher->result();
            watcher->deleteLater();

            if (m_canceled.load() || !result.success || !result.projected_cloud) {
                if (!result.error_msg.empty()) {
                    printE(QString::fromStdString(result.error_msg));
                }
                this->reject();
                return;
            }

            // Set the output name
            result.projected_cloud->setId(output_name.toStdString());

            // Insert into tree
            m_cloudtree->insertCloud(result.projected_cloud);

            printI(QString("Closest point set extracted in %1 ms. Points: %2")
                       .arg(result.time_ms, 0, 'f', 1)
                       .arg(result.projected_cloud->size()));

            this->accept();
        });

    watcher->setFuture(future);
}

// ======================== Helpers ========================

void ClosestPointSetDialog::populateComboBoxes()
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

void ClosestPointSetDialog::updateOutputName()
{
    QString src = cbox_source_->currentText();
    QString tgt = cbox_target_->currentText();
    if (src.isEmpty() || tgt.isEmpty()) {
        edit_output_name_->setText("CPS_Result");
    } else {
        edit_output_name_->setText(QString("[CPS] %1 -> %2").arg(src).arg(tgt));
    }
}
