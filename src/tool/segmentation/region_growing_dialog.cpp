#include "region_growing_dialog.h"

#include "algorithm/segmentation.h"
#include "core/cloud.h"
#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QCoreApplication>

#include <QtConcurrent/QtConcurrent>
#include <QTimer>

#include <cmath>
#include <limits>

// ======================== Constructor ========================

RegionGrowingDialog::RegionGrowingDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_canceled(false), m_has_seed(false)
{
    setupUi();
    this->setWindowTitle("Region Growing");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(380, 540);
    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

RegionGrowingDialog::~RegionGrowingDialog() = default;

// ======================== setupUi ========================

void RegionGrowingDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- Search Settings ---
    auto* search_group = new QGroupBox("Search Settings", this);
    auto* search_layout = new QFormLayout(search_group);
    search_layout->setContentsMargins(6, 10, 6, 6);
    search_layout->setSpacing(4);

    spin_neighbours_ = new QSpinBox(this);
    spin_neighbours_->setRange(1, 200);
    spin_neighbours_->setValue(30);
    search_layout->addRow("Neighbors:", spin_neighbours_);

    main_layout->addWidget(search_group);

    // --- Growth Criteria ---
    auto* growth_group = new QGroupBox("Growth Criteria", this);
    auto* growth_layout = new QVBoxLayout(growth_group);
    growth_layout->setContentsMargins(6, 10, 6, 6);
    growth_layout->setSpacing(4);

    // Smoothness
    check_smooth_ = new QCheckBox("Smoothness Threshold", this);
    check_smooth_->setChecked(true);
    growth_layout->addWidget(check_smooth_);

    auto* smooth_row = new QHBoxLayout();
    slider_smooth_ = new QSlider(Qt::Horizontal, this);
    slider_smooth_->setRange(0, 900);
    slider_smooth_->setValue(250);  // 25.0°
    dspin_smooth_ = new QDoubleSpinBox(this);
    dspin_smooth_->setRange(0.0, 90.0);
    dspin_smooth_->setDecimals(1);
    dspin_smooth_->setValue(25.0);
    dspin_smooth_->setSingleStep(1.0);
    dspin_smooth_->setSuffix("°");
    smooth_row->addWidget(slider_smooth_, 1);
    smooth_row->addWidget(dspin_smooth_);
    growth_layout->addLayout(smooth_row);

    // Curvature
    check_curvature_ = new QCheckBox("Curvature Threshold", this);
    growth_layout->addWidget(check_curvature_);

    auto* curv_layout = new QHBoxLayout();
    dspin_curvature_ = new QDoubleSpinBox(this);
    dspin_curvature_->setRange(0.0, 10.0);
    dspin_curvature_->setDecimals(3);
    dspin_curvature_->setValue(0.05);
    dspin_curvature_->setSingleStep(0.01);
    dspin_curvature_->setEnabled(false);
    curv_layout->addWidget(new QLabel("Threshold:", this));
    curv_layout->addWidget(dspin_curvature_, 1);
    growth_layout->addLayout(curv_layout);

    // Residual
    check_residual_ = new QCheckBox("Residual Threshold", this);
    growth_layout->addWidget(check_residual_);

    auto* resid_layout = new QHBoxLayout();
    dspin_residual_ = new QDoubleSpinBox(this);
    dspin_residual_->setRange(0.0, 1000.0);
    dspin_residual_->setDecimals(3);
    dspin_residual_->setValue(0.05);
    dspin_residual_->setSingleStep(0.01);
    dspin_residual_->setEnabled(false);
    resid_layout->addWidget(new QLabel("Threshold:", this));
    resid_layout->addWidget(dspin_residual_, 1);
    growth_layout->addLayout(resid_layout);

    // Color
    check_color_ = new QCheckBox("Use Color (RGB)", this);
    growth_layout->addWidget(check_color_);

    group_color_ = new QGroupBox("Color Settings", this);
    auto* color_layout = new QFormLayout(group_color_);
    color_layout->setContentsMargins(6, 6, 6, 6);
    color_layout->setSpacing(4);

    dspin_point_color_ = new QDoubleSpinBox(this);
    dspin_point_color_->setRange(0.0, 100.0);
    dspin_point_color_->setDecimals(1);
    dspin_point_color_->setValue(5.0);
    dspin_point_color_->setSingleStep(0.5);
    color_layout->addRow("Point Color:", dspin_point_color_);

    dspin_region_color_ = new QDoubleSpinBox(this);
    dspin_region_color_->setRange(0.0, 100.0);
    dspin_region_color_->setDecimals(1);
    dspin_region_color_->setValue(10.0);
    dspin_region_color_->setSingleStep(0.5);
    color_layout->addRow("Region Color:", dspin_region_color_);

    dspin_distance_ = new QDoubleSpinBox(this);
    dspin_distance_->setRange(0.0, 1000.0);
    dspin_distance_->setDecimals(1);
    dspin_distance_->setValue(0.0);
    dspin_distance_->setSingleStep(0.1);
    color_layout->addRow("Distance:", dspin_distance_);

    spin_color_neighbors_ = new QSpinBox(this);
    spin_color_neighbors_->setRange(1, 200);
    spin_color_neighbors_->setValue(5);
    color_layout->addRow("Color Neighbors:", spin_color_neighbors_);

    group_color_->setVisible(false);
    growth_layout->addWidget(group_color_);

    main_layout->addWidget(growth_group);

    // --- Seed Strategy ---
    auto* seed_group = new QGroupBox("Seed Strategy", this);
    auto* seed_layout = new QVBoxLayout(seed_group);
    seed_layout->setContentsMargins(6, 10, 6, 6);
    seed_layout->setSpacing(4);

    bg_seed_ = new QButtonGroup(this);
    radio_auto_ = new QRadioButton("Automatic", this);
    radio_manual_ = new QRadioButton("Manual Pick", this);
    bg_seed_->addButton(radio_auto_, 0);
    bg_seed_->addButton(radio_manual_, 1);
    radio_auto_->setChecked(true);
    seed_layout->addWidget(radio_auto_);
    seed_layout->addWidget(radio_manual_);

    auto* pick_row = new QHBoxLayout();
    btn_pick_seed_ = new QPushButton("Pick Seed Point", this);
    btn_pick_seed_->setEnabled(false);
    label_seed_info_ = new QLabel("(no seed selected)", this);
    pick_row->addWidget(btn_pick_seed_);
    pick_row->addWidget(label_seed_info_, 1);
    seed_layout->addLayout(pick_row);

    main_layout->addWidget(seed_group);

    // --- Output ---
    auto* output_group = new QGroupBox("Output", this);
    auto* output_layout = new QFormLayout(output_group);
    output_layout->setContentsMargins(6, 10, 6, 6);
    output_layout->setSpacing(4);

    spin_min_cluster_ = new QSpinBox(this);
    spin_min_cluster_->setRange(1, 1000000);
    spin_min_cluster_->setValue(100);
    output_layout->addRow("Min Cluster Size:", spin_min_cluster_);

    spin_max_cluster_ = new QSpinBox(this);
    spin_max_cluster_->setRange(1, 100000000);
    spin_max_cluster_->setValue(1000000);
    output_layout->addRow("Max Cluster Size:", spin_max_cluster_);

    check_split_ = new QCheckBox("Split into individual clouds", this);
    output_layout->addRow(check_split_);

    main_layout->addWidget(output_group);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_apply_ = new QPushButton("Apply", this);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(check_smooth_, &QCheckBox::toggled, this, &RegionGrowingDialog::onSmoothToggled);
    connect(check_curvature_, &QCheckBox::toggled, this, &RegionGrowingDialog::onCurvatureToggled);
    connect(check_residual_, &QCheckBox::toggled, [=](bool checked) {
        dspin_residual_->setEnabled(checked);
    });
    connect(check_color_, &QCheckBox::toggled, this, &RegionGrowingDialog::onColorToggled);
    connect(bg_seed_, QOverload<int>::of(&QButtonGroup::buttonClicked),
            this, &RegionGrowingDialog::onSeedModeChanged);
    connect(btn_pick_seed_, &QPushButton::clicked, this, &RegionGrowingDialog::onPickSeed);
    connect(btn_apply_, &QPushButton::clicked, this, &RegionGrowingDialog::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &RegionGrowingDialog::onCancel);

    // Slider-SpinBox 同步
    connect(slider_smooth_, &QSlider::valueChanged, [=](int val) {
        double deg = val / 10.0;
        dspin_smooth_->blockSignals(true);
        dspin_smooth_->setValue(deg);
        dspin_smooth_->blockSignals(false);
    });
    connect(dspin_smooth_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double val) {
        slider_smooth_->blockSignals(true);
        slider_smooth_->setValue(static_cast<int>(val * 10.0));
        slider_smooth_->blockSignals(false);
    });
}

// ======================== Init / Reset ========================

void RegionGrowingDialog::init()
{
    auto selection = m_cloudtree->getSelectedClouds();
    if (!selection.empty()) {
        m_cloud = selection.front();
    }
    m_picking = false;
    m_has_seed = false;
    label_seed_info_->setText("(no seed selected)");
}

void RegionGrowingDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_cloud.reset();
    m_picking = false;
    m_has_seed = false;
    label_seed_info_->setText("(no seed selected)");
}

// ======================== Slots ========================

void RegionGrowingDialog::onSmoothToggled(bool checked)
{
    slider_smooth_->setEnabled(checked);
    dspin_smooth_->setEnabled(checked);
}

void RegionGrowingDialog::onCurvatureToggled(bool checked)
{
    dspin_curvature_->setEnabled(checked);
}

void RegionGrowingDialog::onColorToggled(bool checked)
{
    group_color_->setVisible(checked);
}

void RegionGrowingDialog::onSeedModeChanged()
{
    bool manual = bg_seed_->checkedId() == 1;
    btn_pick_seed_->setEnabled(manual);
    if (!manual) {
        m_picking = false;
        m_has_seed = false;
        label_seed_info_->setText("(no seed selected)");
    }
}

void RegionGrowingDialog::onPickSeed()
{
    if (!m_cloud) {
        printW("Please select a cloud first.");
        return;
    }
    m_picking = true;
    printI("Click on the point cloud to select a seed point...");
    // 隐藏模态对话框，允许用户与点云交互
    this->hide();
    QCoreApplication::processEvents();
    // 禁用 VTK 旋转/平移，避免选点时点云跟随鼠标
    m_cloudview->setInteractorEnable(false);
    // 连接鼠标信号
    connect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &RegionGrowingDialog::mouseLeftPressed);
}

void RegionGrowingDialog::mouseLeftPressed(const ct::PointXY& pt)
{
    if (!m_picking) return;

    // 断开信号，避免重复触发
    disconnect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &RegionGrowingDialog::mouseLeftPressed);

    ct::PickResult res = m_cloudview->singlePick(pt, QString::fromStdString(m_cloud->id()));
    // 恢复 VTK 交互和 picking 状态
    m_cloudview->setInteractorEnable(true);
    m_picking = false;

    if (!res.valid) {
        printW("Failed to pick a seed point.");
        label_seed_info_->setText("(pick failed)");
        this->show();
        return;
    }

    m_seed_point = res.point;
    m_has_seed = true;
    label_seed_info_->setText(QString("(%.2f, %.2f, %.2f)")
                                   .arg(static_cast<double>(m_seed_point.x))
                                   .arg(static_cast<double>(m_seed_point.y))
                                   .arg(static_cast<double>(m_seed_point.z)));
    printI(QString("Seed point selected: (%1, %2, %3)")
               .arg(static_cast<double>(m_seed_point.x))
               .arg(static_cast<double>(m_seed_point.y))
               .arg(static_cast<double>(m_seed_point.z)));
    // 选点完成，恢复对话框
    this->show();
}

void RegionGrowingDialog::onApply()
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    bool manual_seed = bg_seed_->checkedId() == 1;
    if (manual_seed && !m_has_seed) {
        printW("Please pick a seed point first.");
        return;
    }

    if (!m_cloud->hasNormals()) {
        printW("Please estimate normals first.");
        return;
    }

    // ========== Step 1: 读取 UI 参数 ==========
    int neighbours = spin_neighbours_->value();
    bool smoothMode = check_smooth_->isChecked();
    bool curvatureTest = check_curvature_->isChecked();
    bool residualTest = check_residual_->isChecked();
    float smoothThreshold = dspin_smooth_->value();
    float curvatureThreshold = dspin_curvature_->value();
    float residualThreshold = dspin_residual_->value();
    int minClusterSize = spin_min_cluster_->value();
    int maxClusterSize = spin_max_cluster_->value();
    bool useColor = check_color_->isChecked();
    float ptThresh = dspin_point_color_->value();
    float reThresh = dspin_region_color_->value();
    float disThresh = dspin_distance_->value();
    int nghbrNumber = spin_color_neighbors_->value();

    // ========== Step 2: 隐藏对话框 ==========
    this->hide();
    QCoreApplication::processEvents();

    // ========== Step 3: 显示进度对话框 ==========
    m_cloudtree->showProgress("Region Growing...");

    // ========== Step 4: 设置取消标志 ==========
    auto* cancel = new std::atomic<bool>(false);
    auto* progress_closed = new std::atomic<bool>(false);
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [=]() {
                    *cancel = true;
                    m_canceled.store(true);
                    m_cloudtree->closeProgress();
                    progress_closed->store(true);
                    printW("Region growing canceled.");
                });
    }

    // ========== Step 5: 进度回调 ==========
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ========== Step 6: 提交算法 ==========
    auto cloud = m_cloud;
    m_canceled.store(false);

    bool is_manual = manual_seed;
    ct::PointXYZRGBN seed_pt = m_seed_point;
    bool has_seed = m_has_seed;
    bool split = check_split_->isChecked();

    auto future = QtConcurrent::run([cloud, is_manual, seed_pt, has_seed, minClusterSize, maxClusterSize,
                                     smoothMode, curvatureTest, residualTest,
                                     smoothThreshold, residualThreshold, curvatureThreshold,
                                     neighbours, useColor, ptThresh, reThresh, disThresh,
                                     nghbrNumber, cancel, on_progress]() {
        // 在后台线程中查找种子点索引
        int seed_idx = -1;
        if (is_manual && has_seed) {
            auto pcl_cloud = cloud->toPCL_XYZRGBN();
            double min_dist = std::numeric_limits<double>::max();
            for (size_t i = 0; i < pcl_cloud->size(); i++) {
                double dx = pcl_cloud->at(i).x - seed_pt.x;
                double dy = pcl_cloud->at(i).y - seed_pt.y;
                double dz = pcl_cloud->at(i).z - seed_pt.z;
                double dist = dx * dx + dy * dy + dz * dz;
                if (dist < min_dist) {
                    min_dist = dist;
                    seed_idx = static_cast<int>(i);
                }
            }
        }

        if (is_manual && seed_idx >= 0) {
            return ct::Segmentation::RegionGrowingFromSeed(cloud, false, seed_idx,
                minClusterSize, maxClusterSize, smoothMode, curvatureTest, residualTest,
                smoothThreshold, residualThreshold, curvatureThreshold, neighbours,
                cancel, on_progress);
        } else if (useColor) {
            return ct::Segmentation::RegionGrowingRGB(cloud, false,
                minClusterSize, maxClusterSize, smoothMode, curvatureTest, residualTest,
                smoothThreshold, residualThreshold, curvatureThreshold, neighbours,
                ptThresh, reThresh, disThresh, nghbrNumber, cancel, on_progress);
        } else {
            return ct::Segmentation::RegionGrowing(cloud, false,
                minClusterSize, maxClusterSize, smoothMode, curvatureTest, residualTest,
                smoothThreshold, residualThreshold, curvatureThreshold, neighbours,
                cancel, on_progress);
        }
    });

    // ========== Step 7: 监听完成信号 ==========
    auto* watcher = new QFutureWatcher<ct::SegmentationResult>(this);
    connect(watcher, &QFutureWatcher<ct::SegmentationResult>::finished, this,
        [=]() {
            if (!progress_closed->load()) {
                m_cloudtree->closeProgress();
            }
            delete cancel;
            delete progress_closed;

            auto result = watcher->result();
            watcher->deleteLater();

            if (m_canceled.load() || result.clouds.empty()) {
                this->reject();
                return;
            }

            printI(QString("Region growing done in %1 ms, %2 cluster(s) generated.")
                       .arg(result.time_ms)
                       .arg(result.clouds.size()));

            if (split) {
                // Split 模式：每个簇作为单独点云
                std::vector<ct::Cloud::Ptr> results;
                for (size_t i = 0; i < result.clouds.size(); i++) {
                    auto& c = result.clouds[i];
                    c->setId(cloud->id() + "-rg" + std::to_string(i));
                    c->makeAdaptive();
                    results.push_back(c);
                }
                QString groupName = QString::fromStdString(cloud->id()) + "_RegionGrowing";
                m_cloudtree->addResultGroup(cloud, results, groupName);
            } else {
                // 合并模式：复制原始点云，添加标量字段存储分割标签
                auto labeled = cloud->clone();
                labeled->setId(cloud->id() + "_region_growing");
                if (!result.labels.empty())
                    labeled->addScalarField("segment_label", result.labels);
                labeled->makeAdaptive();
                std::vector<ct::Cloud::Ptr> results = {labeled};
                m_cloudtree->addResultGroup(cloud, results, QString::fromStdString(cloud->id()) + "_RegionGrowing");
            }

            this->accept();
        });

    watcher->setFuture(future);
}

void RegionGrowingDialog::onCancel()
{
    this->close();
}
