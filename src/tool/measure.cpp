//
// Created by LBC on 2026/04/12.
//

#include "measure.h"
#include "ui_Measure.h"
#include "base/cloudtree.h"

#include <QFileDialog>
#include <QTextStream>
#include <QHeaderView>

Measure::Measure(QWidget *parent)
    : CustomDialog(parent), ui(new Ui::Measure),
      m_marker_cloud(new ct::Cloud),
      m_first_screen_pos(-1, -1)
{
    ui->setupUi(this);
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);

    // Table setup
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->setColumnCount(5);
    ui->tableWidget->setHorizontalHeaderLabels({"#", "Distance", "dX", "dY", "dZ"});
    ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);

    // Button connections
    connect(ui->btnStartStop, &QPushButton::clicked, this, &Measure::onStartStop);
    connect(ui->btnClearAll, &QPushButton::clicked, this, &Measure::onClearAll);
    connect(ui->btnDelete, &QPushButton::clicked, this, &Measure::onDeleteSelected);
    connect(ui->btnExport, &QPushButton::clicked, this, &Measure::onExport);
    connect(ui->btnClose, &QPushButton::clicked, this, &Measure::close);

    // Display options
    connect(ui->chkShowLabels, &QCheckBox::stateChanged, this, &Measure::onShowLabelsChanged);
    connect(ui->chkShowArrows, &QCheckBox::stateChanged, this, &Measure::onShowArrowsChanged);
    connect(ui->spinPrecision, QOverload<int>::of(&QSpinBox::valueChanged), this, &Measure::onPrecisionChanged);

    // Table selection
    connect(ui->tableWidget, &QTableWidget::cellClicked, this, &Measure::onMeasurementSelected);
}

Measure::~Measure()
{
    delete ui;
}

void Measure::init()
{
    updateInfoText();
}

void Measure::reset()
{
    if (m_measuring) {
        onStartStop();
    }
    m_selected_cloud = nullptr;
    onClearAll();
}

void Measure::deinit()
{
    if (m_measuring) {
        onStartStop();
    }
    // Remove all scene annotations
    for (const auto& m : m_measurements) {
        removeMeasurementFromScene(m);
    }
    m_cloudview->remove3DBadge(previewLabelId());
    m_cloudview->removeShape(previewArrowId());
    clearMarkerCloud();
    m_cloudview->clearInfo();
}

// ============================================================
// UI Slots
// ============================================================

void Measure::onStartStop()
{
    if (!m_measuring) {
        // Start measuring
        std::vector<ct::Cloud::Ptr> selected = m_cloudtree->getSelectedClouds();
        if (selected.empty()) {
            printW("Please select a cloud first!");
            return;
        }
        m_selected_cloud = selected.front();
        m_measuring = true;
        m_first_point_set = false;

        ui->btnStartStop->setText("Stop");

        connect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &Measure::mouseLeftPressed);
        connect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &Measure::mouseLeftReleased);
        connect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Measure::mouseRightReleased);
        connect(m_cloudview, &ct::CloudView::mouseMoved, this, &Measure::mouseMoved);

        updateInfoText();
    } else {
        // Stop measuring
        m_measuring = false;
        cancelCurrentPick();

        ui->btnStartStop->setText("Start");

        disconnect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &Measure::mouseLeftPressed);
        disconnect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &Measure::mouseLeftReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Measure::mouseRightReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseMoved, this, &Measure::mouseMoved);

        clearMarkerCloud();
        m_cloudview->removeShape(previewArrowId());
        m_cloudview->remove3DBadge(previewLabelId());
        m_cloudview->clearInfo();

        updateInfoText();
    }
}

void Measure::onClearAll()
{
    for (const auto& m : m_measurements) {
        removeMeasurementFromScene(m);
    }
    m_measurements.clear();
    m_next_id = 1;

    ui->tableWidget->setRowCount(0);
    printI("All measurements cleared.");
}

void Measure::onDeleteSelected()
{
    int row = ui->tableWidget->currentRow();
    if (row < 0 || row >= m_measurements.size()) {
        printW("Please select a measurement to delete.");
        return;
    }

    removeMeasurementFromScene(m_measurements[row]);
    m_measurements.removeAt(row);
    removeMeasurementFromTable(row);

    printI(QString("Measurement #%1 deleted.").arg(row + 1));
}

void Measure::onExport()
{
    if (m_measurements.isEmpty()) {
        printW("No measurements to export.");
        return;
    }

    QString filepath = QFileDialog::getSaveFileName(this, "Export Measurements",
        "measurements.csv", "CSV Files (*.csv)");
    if (filepath.isEmpty()) return;

    QFile file(filepath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        printE(QString("Failed to open file: %1").arg(filepath));
        return;
    }

    QTextStream out(&file);
    out << "#,Distance,dX,dY,dZ,StartX,StartY,StartZ,EndX,EndY,EndZ\n";

    for (const auto& m : m_measurements) {
        out << m.id << ","
            << QString::number(m.distance, 'f', m_precision) << ","
            << QString::number(m.dx, 'f', m_precision) << ","
            << QString::number(m.dy, 'f', m_precision) << ","
            << QString::number(m.dz, 'f', m_precision) << ","
            << QString::number(m.start_pt.x, 'f', m_precision) << ","
            << QString::number(m.start_pt.y, 'f', m_precision) << ","
            << QString::number(m.start_pt.z, 'f', m_precision) << ","
            << QString::number(m.end_pt.x, 'f', m_precision) << ","
            << QString::number(m.end_pt.y, 'f', m_precision) << ","
            << QString::number(m.end_pt.z, 'f', m_precision) << "\n";
    }

    file.close();
    printI(QString("Exported %1 measurements to %2").arg(m_measurements.size()).arg(filepath));
}

void Measure::onShowLabelsChanged(int state)
{
    bool show = (state == Qt::Checked);
    for (const auto& m : m_measurements) {
        if (show) {
            ct::PointXYZRGBN mid;
            mid.x = (m.start_pt.x + m.end_pt.x) / 2.0f;
            mid.y = (m.start_pt.y + m.end_pt.y) / 2.0f;
            mid.z = (m.start_pt.z + m.end_pt.z) / 2.0f;
            QString text = QString::number(m.distance, 'f', m_precision);
            m_cloudview->add3DBadge(mid, text, labelId(m.id));
        } else {
            m_cloudview->remove3DBadge(labelId(m.id));
        }
    }
}

void Measure::onShowArrowsChanged(int state)
{
    bool show = (state == Qt::Checked);
    for (const auto& m : m_measurements) {
        if (show) {
            m_cloudview->addArrow(m.end_pt, m.start_pt, arrowId(m.id), false, ct::Color::Green);
        } else {
            m_cloudview->removeShape(arrowId(m.id));
        }
    }
}

void Measure::onPrecisionChanged(int value)
{
    m_precision = value;
    // Refresh table values
    for (int i = 0; i < m_measurements.size(); ++i) {
        const auto& m = m_measurements[i];
        if (auto* item = ui->tableWidget->item(i, 1))
            item->setText(QString::number(m.distance, 'f', m_precision));
        if (auto* item = ui->tableWidget->item(i, 2))
            item->setText(QString::number(m.dx, 'f', m_precision));
        if (auto* item = ui->tableWidget->item(i, 3))
            item->setText(QString::number(m.dy, 'f', m_precision));
        if (auto* item = ui->tableWidget->item(i, 4))
            item->setText(QString::number(m.dz, 'f', m_precision));
    }
    // Refresh labels
    onShowLabelsChanged(ui->chkShowLabels->checkState());
}

void Measure::onMeasurementSelected(int row, int col)
{
    if (row < 0 || row >= m_measurements.size()) return;

    const auto& m = m_measurements[row];
    QString info = QString("Meas #%1: dist=%2, dX=%3, dY=%4, dZ=%5")
        .arg(m.id)
        .arg(m.distance, 0, 'f', m_precision)
        .arg(m.dx, 0, 'f', m_precision)
        .arg(m.dy, 0, 'f', m_precision)
        .arg(m.dz, 0, 'f', m_precision);

    m_cloudview->showInfo(info, 5, ct::Color::Yellow);
}

// ============================================================
// CloudView Mouse Events
// ============================================================

void Measure::mouseLeftPressed(const ct::PointXY& pt)
{
    if (!m_measuring) return;

    ct::PickResult res = m_cloudview->singlePick(pt, QString::fromStdString(m_selected_cloud->id()));
    if (!res.valid) return;

    if (!m_first_point_set) {
        pickFirstPoint(res.point, pt);
    }
    // Second point is handled in mouseLeftReleased
}

void Measure::mouseLeftReleased(const ct::PointXY& pt)
{
    if (!m_measuring || !m_first_point_set) return;

    // Ignore if released at the same position as press (avoid re-triggering)
    if (pt.x == m_first_screen_pos.x && pt.y == m_first_screen_pos.y) return;

    ct::PickResult res = m_cloudview->singlePick(pt, QString::fromStdString(m_selected_cloud->id()));
    if (!res.valid) return;

    pickSecondPoint(res.point);
}

void Measure::mouseRightReleased(const ct::PointXY&)
{
    if (!m_measuring) return;
    cancelCurrentPick();
}

void Measure::mouseMoved(const ct::PointXY& pt)
{
    if (!m_measuring || !m_first_point_set) return;

    ct::PickResult res = m_cloudview->singlePick(pt, QString::fromStdString(m_selected_cloud->id()));
    if (!res.valid) {
        m_cloudview->removeShape(previewArrowId());
        m_cloudview->remove3DBadge(previewLabelId());
        return;
    }

    updatePreview(res.point);
}

// ============================================================
// Internal: Pick Logic
// ============================================================

void Measure::pickFirstPoint(const ct::PointXYZRGBN& pt3d, const ct::PointXY& screenPt)
{
    clearMarkerCloud();
    updateMarkerCloud(pt3d);

    m_first_point_set = true;
    m_first_screen_pos = screenPt;

    m_cloudview->showInfo("Pick End Point...", 3);
}

void Measure::pickSecondPoint(const ct::PointXYZRGBN& pt3d)
{
    const auto& blocks = m_marker_cloud->getBlocks();
    if (blocks.empty() || blocks.front()->empty()) return;

    const auto& p = blocks.front()->m_points[0];
    ct::PointXYZRGBN start_pt;
    start_pt.x = p.x; start_pt.y = p.y; start_pt.z = p.z;

    // Remove preview
    m_cloudview->removeShape(previewArrowId());
    m_cloudview->remove3DBadge(previewLabelId());

    // Compute distance
    Eigen::Vector3f diff = start_pt.getVector3fMap() - pt3d.getVector3fMap();
    float dist = diff.norm();

    // Create measurement
    Measurement m;
    m.id = m_next_id++;
    m.start_pt = start_pt;
    m.end_pt = pt3d;
    m.distance = dist;
    m.dx = pt3d.x - start_pt.x;
    m.dy = pt3d.y - start_pt.y;
    m.dz = pt3d.z - start_pt.z;

    m_measurements.push_back(m);

    // Add to scene
    addMeasurementToScene(m);

    // Add to table
    addMeasurementToTable(m);

    // Reset first point state - stay in measuring mode for next measurement
    m_first_point_set = false;
    clearMarkerCloud();

    printI(QString("Measurement #%1: Distance = %2").arg(m.id).arg(dist, 0, 'f', m_precision));

    // Prompt for next
    m_cloudview->showInfo("Pick Start Point...", 3);
}

void Measure::updatePreview(const ct::PointXYZRGBN& hover_pt)
{
    const auto& blocks = m_marker_cloud->getBlocks();
    if (blocks.empty() || blocks.front()->empty()) return;

    const auto& p = blocks.front()->m_points[0];
    ct::PointXYZRGBN start_pt;
    start_pt.x = p.x; start_pt.y = p.y; start_pt.z = p.z;

    float dist = (start_pt.getVector3fMap() - hover_pt.getVector3fMap()).norm();

    // Preview arrow (yellow)
    m_cloudview->addArrow(hover_pt, start_pt, previewArrowId(), false, ct::Color::Yellow);

    // Preview label at midpoint
    ct::PointXYZRGBN mid;
    mid.x = (start_pt.x + hover_pt.x) / 2.0f;
    mid.y = (start_pt.y + hover_pt.y) / 2.0f;
    mid.z = (start_pt.z + hover_pt.z) / 2.0f;
    m_cloudview->add3DBadge(mid, QString::number(dist, 'f', m_precision),
                            previewLabelId(), 20, 1.0, 1.0, 0.0, 0.2, 0.2, 0.2, 0.7);
}

void Measure::cancelCurrentPick()
{
    if (m_first_point_set) {
        clearMarkerCloud();
        m_cloudview->removeShape(previewArrowId());
        m_cloudview->remove3DBadge(previewLabelId());
        m_first_point_set = false;

        if (m_measuring) {
            m_cloudview->showInfo("Pick Start Point...", 3);
        }
    }
}

// ============================================================
// Internal: Scene Management
// ============================================================

void Measure::addMeasurementToScene(const Measurement& m)
{
    // Arrow (green)
    if (ui->chkShowArrows->isChecked()) {
        m_cloudview->addArrow(m.end_pt, m.start_pt, arrowId(m.id), false, ct::Color::Green);
    }

    // Label at midpoint
    if (ui->chkShowLabels->isChecked()) {
        ct::PointXYZRGBN mid;
        mid.x = (m.start_pt.x + m.end_pt.x) / 2.0f;
        mid.y = (m.start_pt.y + m.end_pt.y) / 2.0f;
        mid.z = (m.start_pt.z + m.end_pt.z) / 2.0f;
        m_cloudview->add3DBadge(mid, QString::number(m.distance, 'f', m_precision),
                                labelId(m.id));
    }
}

void Measure::removeMeasurementFromScene(const Measurement& m)
{
    m_cloudview->removeShape(arrowId(m.id));
    m_cloudview->remove3DBadge(labelId(m.id));
}

void Measure::removeMeasurementFromTable(int index)
{
    ui->tableWidget->removeRow(index);

    // Renumber rows after removal
    for (int i = 0; i < ui->tableWidget->rowCount(); ++i) {
        ui->tableWidget->item(i, 0)->setText(QString::number(m_measurements[i].id));
    }
}

void Measure::refreshAllSceneDisplay()
{
    onShowArrowsChanged(ui->chkShowArrows->checkState());
    onShowLabelsChanged(ui->chkShowLabels->checkState());
}

// ============================================================
// Internal: Marker Cloud
// ============================================================

void Measure::updateMarkerCloud(const ct::PointXYZRGBN& pt)
{
    m_marker_cloud->clear();
    m_marker_cloud->addPoint(pt);
    m_marker_cloud->setId(markerCloudId().toStdString());
    m_marker_cloud->setPointSize(15);

    m_cloudview->removePointCloud(markerCloudId());
    m_cloudview->addPointCloud(m_marker_cloud);
    m_cloudview->setPointCloudColor(m_marker_cloud, ct::Color::Red);
}

void Measure::clearMarkerCloud()
{
    if (!m_marker_cloud->empty()) {
        m_cloudview->removePointCloud(markerCloudId());
        m_marker_cloud->clear();
    }
}

// ============================================================
// Internal: Table Management
// ============================================================

void Measure::addMeasurementToTable(const Measurement& m)
{
    int row = ui->tableWidget->rowCount();
    ui->tableWidget->insertRow(row);

    ui->tableWidget->setItem(row, 0, new QTableWidgetItem(QString::number(m.id)));
    ui->tableWidget->setItem(row, 1, new QTableWidgetItem(QString::number(m.distance, 'f', m_precision)));
    ui->tableWidget->setItem(row, 2, new QTableWidgetItem(QString::number(m.dx, 'f', m_precision)));
    ui->tableWidget->setItem(row, 3, new QTableWidgetItem(QString::number(m.dy, 'f', m_precision)));
    ui->tableWidget->setItem(row, 4, new QTableWidgetItem(QString::number(m.dz, 'f', m_precision)));

    // Center-align all columns
    for (int col = 0; col < 5; ++col) {
        ui->tableWidget->item(row, col)->setTextAlignment(Qt::AlignCenter);
    }

    // Scroll to bottom
    ui->tableWidget->scrollToBottom();
}

// ============================================================
// Internal: Info Text
// ============================================================

void Measure::updateInfoText()
{
    QString status = m_measuring ? "[ON]" : "[OFF]";
    QString prompt = m_first_point_set ? "(Pick End Point...)" : "(Pick Start Point...)";
    if (!m_measuring) prompt.clear();

    m_cloudview->showInfo(QString("Measure %1 %2").arg(status).arg(prompt), 1);
}
