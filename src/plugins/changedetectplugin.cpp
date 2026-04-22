//
// Created by LBC on 2026/1/18.
//

#include "changedetectplugin.h"
#include "ui_changedetectplugin.h"

#include <cmath>

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>

ChangeDetectPlugin::ChangeDetectPlugin(QWidget *parent) :
        ct::CustomDialog(parent), ui(new Ui::ChangeDetectPlugin) {
    ui->setupUi(this);

    connect(ui->btn_ok, &QPushButton::clicked, this, &ChangeDetectPlugin::onApply);
    connect(ui->btn_cancel, &QPushButton::clicked, this, &ChangeDetectPlugin::onCancel);
}

ChangeDetectPlugin::~ChangeDetectPlugin() {
    delete ui;
}

void ChangeDetectPlugin::init() {
    ui->combo_phase1->clear();
    ui->combo_phase2->clear();

    std::vector<ct::Cloud::Ptr> allClouds = m_cloudtree->getAllClouds();
    if (allClouds.empty()){
        printW(QString("No clouds available"));
        ui->btn_ok->setEnabled(false);
        return;
    }

    for (const auto& cloud : allClouds){
        QString cloudId = QString::fromStdString(cloud->id());
        ui->combo_phase1->addItem(cloudId, QVariant::fromValue(cloud));
        ui->combo_phase2->addItem(cloudId, QVariant::fromValue(cloud));
    }

    std::vector<ct::Cloud::Ptr> selectedClouds = m_cloudtree->getSelectedClouds();
    if (selectedClouds.size() >= 2){
        int idx1 = ui->combo_phase1->findText(QString::fromStdString(selectedClouds[0]->id()));
        int idx2 = ui->combo_phase2->findText(QString::fromStdString(selectedClouds[1]->id()));

        if (idx1 >= 0) ui->combo_phase1->setCurrentIndex(idx1);
        if (idx2 >= 0) ui->combo_phase2->setCurrentIndex(idx2);
    }
    else if (selectedClouds.size() == 1 && allClouds.size() >= 2){
        int idx1 = ui->combo_phase1->findText(QString::fromStdString(selectedClouds[0]->id()));
        ui->combo_phase1->setCurrentIndex(idx1);

        for (int i = 0; i < ui->combo_phase2->count(); ++i){
            if (i != idx1){
                ui->combo_phase2->setCurrentIndex(i);
                break;
            }
        }
    }

    ui->combo_method->clear();
    ui->combo_method->addItem("C2C - Nearest Neighbor", ct::DistanceParams::C2C_NEAREST);
    ui->combo_method->addItem("C2C - K-Mean", ct::DistanceParams::C2C_KNN_MEAN);
    ui->combo_method->addItem("C2C - Radius Mean", ct::DistanceParams::C2C_RADIUS_MEAN);

    connect(ui->combo_method, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ChangeDetectPlugin::onMethodChanged);
    ui->stackedWidget->setCurrentIndex(0);
    ui->btn_ok->setEnabled(true);
}

void ChangeDetectPlugin::onApply() {
    m_phase1Cloud = ui->combo_phase1->currentData().value<ct::Cloud::Ptr>();
    m_phase2Cloud = ui->combo_phase2->currentData().value<ct::Cloud::Ptr>();

    if (!m_phase1Cloud || !m_phase2Cloud){
        printE(QString("Invalid clouds selected"));
        return;
    }

    if (m_phase1Cloud == m_phase2Cloud){
        printW(QString("Phase 1 and Phase 2 clouds cannot be the same"));
        return;
    }

    ct::DistanceParams params;
    params.method = static_cast<ct::DistanceParams::Method>(ui->combo_method->currentData().toInt());

    if (params.method == ct::DistanceParams::C2C_NEAREST) {
        m_threshold = ui->dsb_nearestThreshold->value();
    }
    else {
        m_threshold = ui->dsb_meanThreshold->value();
        if (params.method == ct::DistanceParams::C2C_KNN_MEAN) {
            params.k_knn = ui->sb_Knn->value();
        }
        else if (params.method == ct::DistanceParams::C2C_RADIUS_MEAN) {
            params.radius = ui->dsb_radius->value();
        }
    }

    this->hide();
    m_progress->showProgress("Change Detection: Calculating Distance...");

    auto* cancel = new std::atomic<bool>(false);
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [cancel]() { *cancel = true; });
    }

    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    auto phase1 = m_phase1Cloud;
    auto phase2 = m_phase2Cloud;
    auto threshold = m_threshold;
    auto params_copy = params;
    auto phase1_has_colors = phase1->hasColors();
    auto phase2_has_colors = phase2->hasColors();
    auto phase1_has_normals = phase1->hasNormals();
    auto phase2_has_normals = phase2->hasNormals();
    auto phase1_id = phase1->id();
    auto phase2_id = phase2->id();

    auto future = QtConcurrent::run([=]() -> std::pair<ct::DistanceResult, ct::DistanceResult> {
        // Direction 1: phase2 → phase1 (added points)
        ct::DistanceResult resultAdded = ct::DistanceCalculator::calculate(phase1, phase2, params_copy, cancel,
            [on_progress](int pct) { on_progress(pct / 2); });

        if (!resultAdded.success || *cancel)
            return {};

        // Direction 2: phase1 → phase2 (removed points)
        ct::DistanceResult resultRemoved = ct::DistanceCalculator::calculate(phase2, phase1, params_copy, cancel,
            [on_progress](int pct) { on_progress(50 + pct / 2); });

        return {resultAdded, resultRemoved};
    });

    auto* watcher = new QFutureWatcher<std::pair<ct::DistanceResult, ct::DistanceResult>>(this);
    connect(watcher, &QFutureWatcher<std::pair<ct::DistanceResult, ct::DistanceResult>>::finished, this,
            [=]() {
        auto [resultAdded, resultRemoved] = watcher->result();
        m_progress->closeProgress();
        delete cancel;

        if (!resultAdded.success) {
            printE(QString("Distance calculation failed: %1").arg(QString::fromStdString(resultAdded.error_msg)));
            watcher->deleteLater();
            return;
        }
        if (!resultRemoved.success) {
            printE(QString("Distance calculation failed: %1").arg(QString::fromStdString(resultRemoved.error_msg)));
            watcher->deleteLater();
            return;
        }

        printI(QString("Change detection completed (Added: %1 ms, Removed: %2 ms)")
               .arg(resultAdded.time_ms).arg(resultRemoved.time_ms));

        // Extract added points from phase2 (phase2→phase1 distance exceeds threshold)
        ct::Cloud::Ptr addedCloud(new ct::Cloud);
        addedCloud->initOctree(phase2->box());
        if (phase2_has_colors) addedCloud->enableColors();
        if (phase2_has_normals) addedCloud->enableNormals();

        const auto& distAdded = resultAdded.distances;
        size_t globalIdx = 0;

        struct BatchBuf {
            std::vector<ct::PointXYZ> pts;
            std::vector<ct::ColorRGB> colors;
            std::vector<ct::CompressedNormal> normals;
            void clear() { pts.clear(); colors.clear(); normals.clear(); }
        };
        BatchBuf buf;
        size_t batch_size = 50000;
        buf.pts.reserve(batch_size);

        for (const auto& block : phase2->getBlocks()) {
            if (block->empty()) continue;
            for (size_t i = 0; i < block->size(); ++i) {
                float d = distAdded[globalIdx++];
                if (std::isnan(d) || d <= threshold) continue;

                buf.pts.push_back(block->m_points[i]);
                if (phase2_has_colors && block->m_colors)
                    buf.colors.push_back((*block->m_colors)[i]);
                if (phase2_has_normals && block->m_normals)
                    buf.normals.push_back((*block->m_normals)[i]);

                if (buf.pts.size() >= batch_size) {
                    addedCloud->addPoints(buf.pts,
                        buf.colors.empty() ? nullptr : &buf.colors,
                        buf.normals.empty() ? nullptr : &buf.normals);
                    buf.clear();
                }
            }
        }
        if (!buf.pts.empty()) {
            addedCloud->addPoints(buf.pts,
                buf.colors.empty() ? nullptr : &buf.colors,
                buf.normals.empty() ? nullptr : &buf.normals);
        }
        addedCloud->update();

        // Extract removed points from phase1 (phase1→phase2 distance exceeds threshold)
        ct::Cloud::Ptr removedCloud(new ct::Cloud);
        removedCloud->initOctree(phase1->box());
        if (phase1_has_colors) removedCloud->enableColors();
        if (phase1_has_normals) removedCloud->enableNormals();

        const auto& distRemoved = resultRemoved.distances;
        globalIdx = 0;
        buf.clear();
        buf.pts.reserve(batch_size);

        for (const auto& block : phase1->getBlocks()) {
            if (block->empty()) continue;
            for (size_t i = 0; i < block->size(); ++i) {
                float d = distRemoved[globalIdx++];
                if (std::isnan(d) || d <= threshold) continue;

                buf.pts.push_back(block->m_points[i]);
                if (phase1_has_colors && block->m_colors)
                    buf.colors.push_back((*block->m_colors)[i]);
                if (phase1_has_normals && block->m_normals)
                    buf.normals.push_back((*block->m_normals)[i]);

                if (buf.pts.size() >= batch_size) {
                    removedCloud->addPoints(buf.pts,
                        buf.colors.empty() ? nullptr : &buf.colors,
                        buf.normals.empty() ? nullptr : &buf.normals);
                    buf.clear();
                }
            }
        }
        if (!buf.pts.empty()) {
            removedCloud->addPoints(buf.pts,
                buf.colors.empty() ? nullptr : &buf.colors,
                buf.normals.empty() ? nullptr : &buf.normals);
        }
        removedCloud->update();

        std::vector<ct::Cloud::Ptr> results;

        if (!addedCloud->empty()) {
            addedCloud->makeAdaptive();
            addedCloud->setId(phase2_id + "_Added");
            addedCloud->setHasColors(phase2_has_colors);
            results.push_back(addedCloud);
            printI(QString("Added cloud: %1 points").arg(addedCloud->size()));
        } else {
            printI("No added points detected");
        }

        if (!removedCloud->empty()) {
            removedCloud->makeAdaptive();
            removedCloud->setId(phase1_id + "_Removed");
            removedCloud->setHasColors(phase1_has_colors);
            results.push_back(removedCloud);
            printI(QString("Removed cloud: %1 points").arg(removedCloud->size()));
        } else {
            printI("No removed points detected");
        }

        if (!results.empty()) {
            // Use phase1 as origin cloud
            QString groupName = QString::fromStdString(phase1_id + "_ChangeDetect");
            m_cloudtree->addResultGroup(phase1, results, groupName);
        }

        this->accept();
        watcher->deleteLater();
    });
    watcher->setFuture(future);
}

void ChangeDetectPlugin::onCancel() {
    this->close();
}

void ChangeDetectPlugin::onMethodChanged(int index) {
    auto method = static_cast<ct::DistanceParams::Method>(ui->combo_method->itemData(index).toInt());

    if (method == ct::DistanceParams::C2C_NEAREST) {
        ui->stackedWidget->setCurrentIndex(0);
    }
    else {
        ui->stackedWidget->setCurrentIndex(1);

        if (method == ct::DistanceParams::C2C_KNN_MEAN) {
            ui->lblParamName->setText("Neighbors (K):");
            ui->stackInput->setCurrentIndex(0);
        } else {
            ui->lblParamName->setText("Search Radius (m):");
            ui->stackInput->setCurrentIndex(1);
        }
    }
}
