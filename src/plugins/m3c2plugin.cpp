#include "m3c2plugin.h"
#include "ui_m3c2plugin.h"

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <limits>
#include "core/cloud.h"
#include <numeric>

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

M3C2Plugin::M3C2Plugin(QWidget *parent) :
        ct::CustomDialog(parent), ui(new Ui::M3C2Plugin) {
    ui->setupUi(this);

    connect(ui->btn_ok, &QPushButton::clicked, this, &M3C2Plugin::onApply);
    connect(ui->btn_cancel, &QPushButton::clicked, this, &M3C2Plugin::onCancel);

    // Disable normal estimation params when using existing normals
    connect(ui->chk_useNormals, &QCheckBox::toggled, this, [=](bool checked) {
        ui->dsb_maxScale->setEnabled(!checked);
    });
}

M3C2Plugin::~M3C2Plugin() {
    delete ui;
}

void M3C2Plugin::init() {
    ui->combo_ref->clear();
    ui->combo_comp->clear();

    std::vector<ct::Cloud::Ptr> allClouds = m_cloudtree->getAllClouds();
    if (allClouds.empty()) {
        printW(QString("No clouds available"));
        ui->btn_ok->setEnabled(false);
        return;
    }

    for (const auto& cloud : allClouds) {
        QString cloudId = QString::fromStdString(cloud->id());
        ui->combo_ref->addItem(cloudId, QVariant::fromValue(cloud));
        ui->combo_comp->addItem(cloudId, QVariant::fromValue(cloud));
    }

    std::vector<ct::Cloud::Ptr> selectedClouds = m_cloudtree->getSelectedClouds();
    if (selectedClouds.size() >= 2) {
        int idx1 = ui->combo_ref->findText(QString::fromStdString(selectedClouds[0]->id()));
        int idx2 = ui->combo_comp->findText(QString::fromStdString(selectedClouds[1]->id()));
        if (idx1 >= 0) ui->combo_ref->setCurrentIndex(idx1);
        if (idx2 >= 0) ui->combo_comp->setCurrentIndex(idx2);
    } else if (selectedClouds.size() == 1 && allClouds.size() >= 2) {
        int idx1 = ui->combo_ref->findText(QString::fromStdString(selectedClouds[0]->id()));
        ui->combo_ref->setCurrentIndex(idx1);
        for (int i = 0; i < ui->combo_comp->count(); ++i) {
            if (i != idx1) {
                ui->combo_comp->setCurrentIndex(i);
                break;
            }
        }
    }

    // Initial state for normal params
    ui->dsb_maxScale->setEnabled(false);

    ui->btn_ok->setEnabled(true);
}

void M3C2Plugin::onApply() {
    FILE* dbg = fopen("m3c2_debug.log", "a");
    fprintf(dbg, "=== onApply ENTER ===\n"); fflush(dbg);

    m_refCloud = ui->combo_ref->currentData().value<ct::Cloud::Ptr>();
    m_compCloud = ui->combo_comp->currentData().value<ct::Cloud::Ptr>();
    fprintf(dbg, "clouds: ref=%p comp=%p\n", (void*)m_refCloud.get(), (void*)m_compCloud.get()); fflush(dbg);

    if (!m_refCloud || !m_compCloud) {
        fprintf(dbg, "EXIT: invalid clouds\n"); fflush(dbg); fclose(dbg);
        printE(QString("Invalid clouds selected"));
        return;
    }

    if (m_refCloud == m_compCloud) {
        fprintf(dbg, "EXIT: same cloud\n"); fflush(dbg); fclose(dbg);
        printW(QString("Reference and Compared clouds cannot be the same"));
        return;
    }

    ct::M3C2Params params;
    params.use_existing_normals = ui->chk_useNormals->isChecked();
    params.normal_max_scale = ui->dsb_maxScale->value();
    params.proj_radius = ui->dsb_projRadius->value();
    params.compute_lod = ui->chk_computeLOD->isChecked();
    params.max_distance = ui->dsb_maxDepth->value();
    fprintf(dbg, "params: use_normals=%d scale=%.3f proj=%.3f lod=%d maxd=%.3f\n",
            params.use_existing_normals, params.normal_max_scale, params.proj_radius,
            params.compute_lod, params.max_distance); fflush(dbg);

    // === Extract all data on the MAIN thread for thread safety ===
    fprintf(dbg, "toPCL_XYZ ref...\n"); fflush(dbg);
    auto refPCL = m_refCloud->toPCL_XYZ();
    fprintf(dbg, "toPCL_XYZ comp...\n"); fflush(dbg);
    auto compPCL = m_compCloud->toPCL_XYZ();
    fprintf(dbg, "refPCL=%zu compPCL=%zu\n", refPCL->size(), compPCL->size()); fflush(dbg);

    pcl::PointCloud<pcl::Normal>::Ptr existingNormals;

    if (params.use_existing_normals && m_refCloud->hasNormals()) {
        fprintf(dbg, "extracting normals...\n"); fflush(dbg);
        existingNormals.reset(new pcl::PointCloud<pcl::Normal>);
        size_t n_ref = refPCL->size();
        existingNormals->width = static_cast<uint32_t>(n_ref);
        existingNormals->height = 1;
        existingNormals->points.resize(n_ref, pcl::Normal());

        size_t gi = 0;
        for (const auto& block : m_refCloud->getBlocks()) {
            if (block->empty()) continue;
            if (!block->m_normals || block->m_normals->size() < block->size()) {
                gi += block->size();
                continue;
            }
            for (size_t i = 0; i < block->size(); ++i) {
                if (gi >= n_ref) break;
                auto n = block->m_normals->at(i).get();
                existingNormals->points[gi].normal_x = n.x();
                existingNormals->points[gi].normal_y = n.y();
                existingNormals->points[gi].normal_z = n.z();
                gi++;
            }
        }
        fprintf(dbg, "normals extracted ok\n"); fflush(dbg);
    }

    this->hide();
    m_progress->showProgress("M3C2: Computing distances...");

    auto* cancel = new std::atomic<bool>(false);
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [cancel]() { *cancel = true; });
    }

    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    auto ref = m_refCloud;
    auto comp = m_compCloud;
    auto params_copy = params;
    auto ref_id = ref->id();

    auto future = QtConcurrent::run([=]() -> ct::M3C2Result {
        FILE* dbg2 = fopen("m3c2_debug.log", "a");
        fprintf(dbg2, "worker thread START\n"); fflush(dbg2); fclose(dbg2);
        auto r = ct::DistanceCalculator::calculateM3C2(refPCL, compPCL, existingNormals,
                                                      params_copy, cancel, on_progress);
        FILE* dbg3 = fopen("m3c2_debug.log", "a");
        fprintf(dbg3, "worker thread END success=%d\n", r.success); fflush(dbg3); fclose(dbg3);
        return r;
    });

    auto* watcher = new QFutureWatcher<ct::M3C2Result>(this);
    connect(watcher, &QFutureWatcher<ct::M3C2Result>::finished, this,
            [=]() {
        FILE* dbg4 = fopen("m3c2_debug.log", "a");
        fprintf(dbg4, "finished handler ENTER\n"); fflush(dbg4);
        ct::M3C2Result result = watcher->result();
        fprintf(dbg4, "got result success=%d, closing progress\n", result.success); fflush(dbg4);
        m_progress->closeProgress();
        delete cancel;

        if (!result.success) {
            printE(QString("M3C2 failed: %1").arg(QString::fromStdString(result.error_msg)));
            fprintf(dbg4, "EXIT: failed\n"); fflush(dbg4); fclose(dbg4);
            watcher->deleteLater();
            return;
        }

        printI(QString("M3C2 completed in %1 ms").arg(result.time_ms));

        fprintf(dbg4, "addScalarField dist_size=%zu\n", result.signed_distances.size()); fflush(dbg4);
        ref->addScalarField("M3C2_Distance", result.signed_distances);
        fprintf(dbg4, "addScalarField DONE\n"); fflush(dbg4);

        if (params_copy.compute_lod && !result.lod_values.empty()) {
            ref->addScalarField("M3C2_LOD", result.lod_values);
        }

        fprintf(dbg4, "updateColorByField\n"); fflush(dbg4);
        ref->updateColorByField("M3C2_Distance",
                                 std::numeric_limits<float>::lowest(),
                                 std::numeric_limits<float>::max(),
                                 ct::ColormapType::JET,
                                 true);
        ref->setHasColors(true);

        fprintf(dbg4, "invalidateCloudRender + refresh\n"); fflush(dbg4);
        QString refId = QString::fromStdString(ref->id());
        m_cloudview->invalidateCloudRender(refId);
        m_cloudview->refresh();
        fprintf(dbg4, "refresh DONE\n"); fflush(dbg4);

        // Print statistics
        const auto& dists = result.signed_distances;
        std::vector<float> valid_dists;
        for (float d : dists) {
            if (!std::isnan(d)) valid_dists.push_back(d);
        }

        if (!valid_dists.empty()) {
            float sum = std::accumulate(valid_dists.begin(), valid_dists.end(), 0.0f);
            float mean = sum / valid_dists.size();

            float min_d = *std::min_element(valid_dists.begin(), valid_dists.end());
            float max_d = *std::max_element(valid_dists.begin(), valid_dists.end());

            float sq_sum = 0;
            for (float d : valid_dists) sq_sum += (d - mean) * (d - mean);
            float std_dev = std::sqrt(sq_sum / valid_dists.size());

            printI(QString("M3C2 Statistics [%1 points]:\n"
                           "  Mean: %2 m\n"
                           "  Std:  %3 m\n"
                           "  Min:  %4 m\n"
                           "  Max:  %5 m")
                   .arg(valid_dists.size())
                   .arg(mean, 0, 'f', 4)
                   .arg(std_dev, 0, 'f', 4)
                   .arg(min_d, 0, 'f', 4)
                   .arg(max_d, 0, 'f', 4));

            if (params_copy.compute_lod && !result.lod_values.empty()) {
                std::vector<float> valid_lod;
                for (float l : result.lod_values) {
                    if (!std::isnan(l)) valid_lod.push_back(l);
                }
                if (!valid_lod.empty()) {
                    float lod_mean = std::accumulate(valid_lod.begin(), valid_lod.end(), 0.0f) / valid_lod.size();
                    printI(QString("  LOD (mean): %1 m").arg(lod_mean, 0, 'f', 4));
                }
            }
        } else {
            printW("No valid distances computed. Check projection radius and point cloud overlap.");
        }

        this->accept();
        watcher->deleteLater();
    });
    watcher->setFuture(future);
}

void M3C2Plugin::onCancel() {
    this->close();
}
