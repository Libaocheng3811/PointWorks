#include "m3c2plugin.h"
#include "ui_m3c2plugin.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>
#include <random>
#include "core/cloud.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

M3C2Plugin::M3C2Plugin(QWidget *parent) :
        pw::CustomDialog(parent), ui(new Ui::M3C2Plugin) {
    ui->setupUi(this);

    connect(ui->btn_ok, &QPushButton::clicked, this, &M3C2Plugin::onApply);
    connect(ui->btn_cancel, &QPushButton::clicked, this, &M3C2Plugin::onCancel);
    connect(ui->btn_guessParams, &QPushButton::clicked, this, &M3C2Plugin::onGuessParams);

    // Normal mode: enable/disable diameter spinbox
    connect(ui->radio_computeNormals, &QRadioButton::toggled, this, [=](bool checked) {
        ui->dsb_normalDiameter->setEnabled(checked);
    });

    // Core points mode: enable/disable subsample factor
    connect(ui->radio_subsampleRef, &QRadioButton::toggled, this, [=](bool checked) {
        ui->dsb_subsampleFactor->setEnabled(checked);
        ui->lbl_subsampleFactor->setEnabled(checked);
    });

    // Registration error: enable/disable value
    connect(ui->chk_regError, &QCheckBox::toggled, this, [=](bool checked) {
        ui->dsb_regError->setEnabled(checked);
        ui->lbl_regError->setEnabled(checked);
    });

    // Initial state: use existing normals → diameter disabled
    ui->dsb_normalDiameter->setEnabled(false);
}

M3C2Plugin::~M3C2Plugin() {
    delete ui;
}

void M3C2Plugin::init() {
    ui->combo_ref->clear();
    ui->combo_comp->clear();

    std::vector<pw::Cloud::Ptr> allClouds = m_cloudtree->getAllClouds();
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

    std::vector<pw::Cloud::Ptr> selectedClouds = m_cloudtree->getSelectedClouds();
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

    ui->btn_ok->setEnabled(true);
}

void M3C2Plugin::onGuessParams() {
    auto ref = ui->combo_ref->currentData().value<pw::Cloud::Ptr>();
    auto comp = ui->combo_comp->currentData().value<pw::Cloud::Ptr>();
    if (!ref || !comp) {
        printW(QString("Select both clouds first"));
        return;
    }

    // Estimate mean point spacing from reference cloud
    auto refPCL = ref->toPCL_XYZ();
    if (refPCL->empty()) return;

    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(refPCL);

    size_t n = std::min(refPCL->size(), static_cast<size_t>(10000));
    std::vector<size_t> indices(refPCL->size());
    std::iota(indices.begin(), indices.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
    indices.resize(n);

    double total_dist = 0.0;
    size_t valid = 0;
    for (size_t idx : indices) {
        std::vector<int> ki(2);
        std::vector<float> sd(2);
        if (tree.nearestKSearch(refPCL->points[idx], 2, ki, sd) >= 2) {
            total_dist += std::sqrt(sd[1]);
            valid++;
        }
    }
    double mean_spacing = (valid > 0) ? (total_dist / valid) : 1.0;

    // CloudCompare-like defaults:
    // Normal Diameter = mean_spacing * 25 (diameter, not radius)
    double normal_diameter = mean_spacing * 25.0;
    // Projection Scale = normal_diameter (cylinder diameter, same as normal diameter)
    double proj_scale = normal_diameter;
    // Max Depth = proj_scale * 10
    double max_depth = proj_scale * 10.0;

    ui->dsb_normalDiameter->setValue(normal_diameter);
    ui->dsb_projScale->setValue(proj_scale);
    ui->dsb_maxDepth->setValue(max_depth);

    printI(QString("Guessed params: mean_spacing=%.6f, normal_diam=%.3f, proj_scale=%.3f, max_depth=%.3f")
           .arg(mean_spacing, 0, 'f', 6)
           .arg(normal_diameter, 0, 'f', 3)
           .arg(proj_scale, 0, 'f', 3)
           .arg(max_depth, 0, 'f', 3));
}

void M3C2Plugin::onApply() {
    m_refCloud = ui->combo_ref->currentData().value<pw::Cloud::Ptr>();
    m_compCloud = ui->combo_comp->currentData().value<pw::Cloud::Ptr>();

    if (!m_refCloud || !m_compCloud) {
        printE(QString("Invalid clouds selected"));
        return;
    }

    if (m_refCloud == m_compCloud) {
        printW(QString("Reference and Compared clouds cannot be the same"));
        return;
    }

    pw::M3C2Params params;
    params.use_existing_normals = ui->radio_useExistingNormals->isChecked();
    params.normal_diameter = ui->dsb_normalDiameter->value();
    params.proj_diameter = ui->dsb_projScale->value();
    params.max_depth = ui->dsb_maxDepth->value();
    params.compute_lod = true;

    // Core points mode
    if (ui->radio_useRefCloud->isChecked()) {
        params.core_points_mode = pw::M3C2Params::CorePointsMode::USE_REF_CLOUD;
    } else if (ui->radio_subsampleRef->isChecked()) {
        params.core_points_mode = pw::M3C2Params::CorePointsMode::SUBSAMPLE_REF;
        params.subsample_factor = ui->dsb_subsampleFactor->value();
    } else {
        params.core_points_mode = pw::M3C2Params::CorePointsMode::USE_COMP_CLOUD;
    }

    // Registration error
    params.use_registration_error = ui->chk_regError->isChecked();
    params.registration_error = ui->dsb_regError->value();

    // === Extract all data on the MAIN thread ===
    auto refPCL = m_refCloud->toPCL_XYZ();
    auto compPCL = m_compCloud->toPCL_XYZ();

    pcl::PointCloud<pcl::Normal>::Ptr existingNormals;
    if (params.use_existing_normals && m_refCloud->hasNormals()) {
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
    }

    this->hide();

    auto ref = m_refCloud;
    auto params_copy = params;
    auto* cloudview = m_cloudview;
    auto* cloudtree = m_cloudtree;

    m_progress->runAsync<pw::M3C2Result>(
        "M3C2: Computing distances...",
        [=](std::atomic<bool>& cancel, pw::ProgressCallback progress) {
            return pw::DistanceCalculator::calculateM3C2(refPCL, compPCL, existingNormals,
                                                         params_copy, &cancel, progress);
        },
        [=](const pw::M3C2Result& result) {
        if (!result.success) {
            printE(QString("M3C2 failed: %1").arg(QString::fromStdString(result.error_msg)));
            return;
        }

        printI(QString("M3C2 completed in %1 ms").arg(result.time_ms));

        // Map core point results back to full reference cloud
        size_t ref_size = static_cast<size_t>(ref->size());
        bool is_use_comp = params_copy.core_points_mode == pw::M3C2Params::CorePointsMode::USE_COMP_CLOUD;

        if (is_use_comp) {
            // USE_COMP_CLOUD: apply results to comp cloud instead
            auto comp = m_compCloud;
            if (comp && result.signed_distances.size() == comp->size()) {
                comp->addScalarField("M3C2_Distance", result.signed_distances);
                if (params_copy.compute_lod && !result.lod_values.empty()) {
                    comp->addScalarField("M3C2_LOD", result.lod_values);
                }
                comp->updateColorByField("M3C2_Distance");
                comp->setHasColors(true);
                QString compId = QString::fromStdString(comp->id());
                cloudview->invalidateCloudRender(compId);
            }
        } else {
            // USE_REF_CLOUD or SUBSAMPLE_REF: expand results to full ref cloud
            std::vector<float> full_dists(ref_size, std::numeric_limits<float>::quiet_NaN());
            std::vector<float> full_lod(ref_size, std::numeric_limits<float>::quiet_NaN());

            for (size_t i = 0; i < result.core_indices.size() && i < result.signed_distances.size(); ++i) {
                size_t idx = result.core_indices[i];
                if (idx < ref_size) {
                    full_dists[idx] = result.signed_distances[i];
                    if (i < result.lod_values.size()) {
                        full_lod[idx] = result.lod_values[i];
                    }
                }
            }

            ref->addScalarField("M3C2_Distance", full_dists);
            if (params_copy.compute_lod && !result.lod_values.empty()) {
                ref->addScalarField("M3C2_LOD", full_lod);
            }

            ref->updateColorByField("M3C2_Distance");
            ref->setHasColors(true);

            QString refId = QString::fromStdString(ref->id());
            cloudview->invalidateCloudRender(refId);
        }

        cloudview->refresh();

        // Rebuild property panel so the new scalar field appears in the Color dropdown
        // and SFDisplayPanel shows the correct histogram + scalar bar
        cloudtree->refreshSelectedProperties();

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
    }
    );
}

void M3C2Plugin::onCancel() {
    this->close();
}
