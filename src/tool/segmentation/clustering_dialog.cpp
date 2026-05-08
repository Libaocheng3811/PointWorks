#include "clustering_dialog.h"

#include "algorithm/segmentation.h"
#include "core/cloud.h"
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

ClusteringDialog::ClusteringDialog(QWidget* parent)
    : pw::CustomDialog(parent), m_canceled(false)
{
    setupUi();
    this->setWindowTitle("Clustering");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(380, 520);
    // 默认选中 Euclidean，隐藏维度面板
    dim_group_->setVisible(false);
    weight_panel_->setVisible(false);
    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

ClusteringDialog::~ClusteringDialog() = default;

// ======================== setupUi ========================

void ClusteringDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- Algorithm ---
    auto* algo_row = new QHBoxLayout();
    algo_row->addWidget(new QLabel("Algorithm:", this));
    cbox_algorithm_ = new QComboBox(this);
    cbox_algorithm_->addItems({"Euclidean", "DBSCAN", "K-Means"});
    algo_row->addWidget(cbox_algorithm_, 1);
    main_layout->addLayout(algo_row);

    // --- Input Dimensions ---
    dim_group_ = new QGroupBox("Input Dimensions", this);
    auto* dim_layout = new QVBoxLayout(dim_group_);
    dim_layout->setContentsMargins(6, 10, 6, 6);
    dim_layout->setSpacing(4);

    check_normal_ = new QCheckBox("Surface (Normal)", this);
    check_color_ = new QCheckBox("Appearance (Color)", this);
    dim_layout->addWidget(check_normal_);
    dim_layout->addWidget(check_color_);

    main_layout->addWidget(dim_group_);

    // --- Weight Panel ---
    weight_panel_ = createWeightPanel();
    weight_panel_->setVisible(false);
    main_layout->addWidget(weight_panel_);

    // --- Parameters ---
    param_pages_ = new QStackedWidget(this);
    param_pages_->addWidget(createEuclideanParamPage());
    param_pages_->addWidget(createDBSCANParamPage());
    param_pages_->addWidget(createKMeansParamPage());
    main_layout->addWidget(param_pages_);

    // --- Output ---
    auto* output_group = new QGroupBox("Output", this);
    auto* output_layout = new QVBoxLayout(output_group);
    output_layout->setContentsMargins(6, 10, 6, 6);
    output_layout->setSpacing(4);

    auto* size_row = new QHBoxLayout();
    size_row->addWidget(new QLabel("Min Cluster Size:", this));
    spin_min_cluster_ = new QSpinBox(this);
    spin_min_cluster_->setRange(1, 1000000);
    spin_min_cluster_->setValue(100);
    size_row->addWidget(spin_min_cluster_, 1);
    size_row->addSpacing(10);
    size_row->addWidget(new QLabel("Max Cluster Size:", this));
    spin_max_cluster_ = new QSpinBox(this);
    spin_max_cluster_->setRange(1, 100000000);
    spin_max_cluster_->setValue(1000000);
    size_row->addWidget(spin_max_cluster_, 1);
    output_layout->addLayout(size_row);

    check_split_ = new QCheckBox("Split into individual clouds", this);
    output_layout->addWidget(check_split_);

    main_layout->addWidget(output_group);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_apply_ = new QPushButton("Apply", this);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(cbox_algorithm_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ClusteringDialog::onAlgorithmChanged);
    connect(check_normal_, &QCheckBox::toggled, this, &ClusteringDialog::onDimensionToggled);
    connect(check_color_, &QCheckBox::toggled, this, &ClusteringDialog::onDimensionToggled);
    connect(btn_apply_, &QPushButton::clicked, this, &ClusteringDialog::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &ClusteringDialog::onCancel);
}

// ======================== Param Pages ========================

QWidget* ClusteringDialog::createEuclideanParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    dspin_tolerance_ = new QDoubleSpinBox(page);
    dspin_tolerance_->setRange(0.0, 1000.0);
    dspin_tolerance_->setDecimals(4);
    dspin_tolerance_->setValue(0.02);
    dspin_tolerance_->setSingleStep(0.005);
    layout->addRow("Tolerance:", dspin_tolerance_);

    return page;
}

QWidget* ClusteringDialog::createDBSCANParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    dspin_eps_ = new QDoubleSpinBox(page);
    dspin_eps_->setRange(0.0, 1000.0);
    dspin_eps_->setDecimals(4);
    dspin_eps_->setValue(0.02);
    dspin_eps_->setSingleStep(0.005);
    layout->addRow("Eps (Radius):", dspin_eps_);

    spin_min_pts_ = new QSpinBox(page);
    spin_min_pts_->setRange(1, 100000);
    spin_min_pts_->setValue(10);
    layout->addRow("Min Points:", spin_min_pts_);

    return page;
}

QWidget* ClusteringDialog::createKMeansParamPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(6, 4, 6, 4);
    layout->setSpacing(4);

    spin_k_ = new QSpinBox(page);
    spin_k_->setRange(2, 1000);
    spin_k_->setValue(5);
    layout->addRow("K (Clusters):", spin_k_);

    spin_max_iter_ = new QSpinBox(page);
    spin_max_iter_->setRange(1, 10000);
    spin_max_iter_->setValue(100);
    spin_max_iter_->setSingleStep(10);
    layout->addRow("Max Iterations:", spin_max_iter_);

    return page;
}

// ======================== Weight Panel ========================

QWidget* ClusteringDialog::createWeightPanel()
{
    auto* panel = new QGroupBox("Weights (Normalized)", this);
    auto* layout = new QFormLayout(panel);
    layout->setContentsMargins(6, 10, 6, 6);
    layout->setSpacing(4);

    // Position
    auto* pos_row = new QHBoxLayout();
    slider_pos_ = new QSlider(Qt::Horizontal, this);
    slider_pos_->setRange(0, 100);
    slider_pos_->setValue(60);
    dspin_pos_ = new QDoubleSpinBox(this);
    dspin_pos_->setRange(0.0, 1.0);
    dspin_pos_->setDecimals(2);
    dspin_pos_->setValue(0.60);
    dspin_pos_->setSingleStep(0.05);
    pos_row->addWidget(slider_pos_, 1);
    pos_row->addWidget(dspin_pos_);
    layout->addRow("Position:", pos_row);

    // Normal (用 QWidget 包装以便整体隐藏)
    row_normal_ = new QWidget(this);
    auto* normal_inner = new QHBoxLayout(row_normal_);
    normal_inner->setContentsMargins(0, 0, 0, 0);
    normal_inner->setSpacing(4);
    slider_normal_ = new QSlider(Qt::Horizontal, this);
    slider_normal_->setRange(0, 100);
    slider_normal_->setValue(25);
    dspin_normal_ = new QDoubleSpinBox(this);
    dspin_normal_->setRange(0.0, 1.0);
    dspin_normal_->setDecimals(2);
    dspin_normal_->setValue(0.25);
    dspin_normal_->setSingleStep(0.05);
    normal_inner->addWidget(slider_normal_, 1);
    normal_inner->addWidget(dspin_normal_);
    layout->addRow("Normal:", row_normal_);

    // Color (用 QWidget 包装以便整体隐藏)
    row_color_ = new QWidget(this);
    auto* color_inner = new QHBoxLayout(row_color_);
    color_inner->setContentsMargins(0, 0, 0, 0);
    color_inner->setSpacing(4);
    slider_color_ = new QSlider(Qt::Horizontal, this);
    slider_color_->setRange(0, 100);
    slider_color_->setValue(15);
    dspin_color_ = new QDoubleSpinBox(this);
    dspin_color_->setRange(0.0, 1.0);
    dspin_color_->setDecimals(2);
    dspin_color_->setValue(0.15);
    dspin_color_->setSingleStep(0.05);
    color_inner->addWidget(slider_color_, 1);
    color_inner->addWidget(dspin_color_);
    layout->addRow("Color:", row_color_);

    label_weight_hint_ = new QLabel("Position 60% | Normal 25% | Color 15%", this);
    layout->addRow(label_weight_hint_);

    // Signals
    connect(slider_pos_, &QSlider::valueChanged, [=](int val) {
        dspin_pos_->blockSignals(true);
        dspin_pos_->setValue(val / 100.0);
        dspin_pos_->blockSignals(false);
        onWeightChanged();
    });
    connect(slider_normal_, &QSlider::valueChanged, [=](int val) {
        dspin_normal_->blockSignals(true);
        dspin_normal_->setValue(val / 100.0);
        dspin_normal_->blockSignals(false);
        onWeightChanged();
    });
    connect(slider_color_, &QSlider::valueChanged, [=](int val) {
        dspin_color_->blockSignals(true);
        dspin_color_->setValue(val / 100.0);
        dspin_color_->blockSignals(false);
        onWeightChanged();
    });
    connect(dspin_pos_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double val) {
        slider_pos_->blockSignals(true);
        slider_pos_->setValue(static_cast<int>(val * 100));
        slider_pos_->blockSignals(false);
        onWeightChanged();
    });
    connect(dspin_normal_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double val) {
        slider_normal_->blockSignals(true);
        slider_normal_->setValue(static_cast<int>(val * 100));
        slider_normal_->blockSignals(false);
        onWeightChanged();
    });
    connect(dspin_color_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double val) {
        slider_color_->blockSignals(true);
        slider_color_->setValue(static_cast<int>(val * 100));
        slider_color_->blockSignals(false);
        onWeightChanged();
    });

    return panel;
}

// ======================== Init / Reset ========================

void ClusteringDialog::init()
{
    auto selection = m_cloudtree->getSelectedClouds();
    if (!selection.empty()) {
        m_cloud = selection.front();
    }
}

void ClusteringDialog::reset()
{
    m_progress->closeProgress();
    m_canceled.store(true);
    m_cloud.reset();
}

// ======================== Slots ========================

void ClusteringDialog::onAlgorithmChanged(int index)
{
    if (index >= 0 && index < param_pages_->count())
        param_pages_->setCurrentIndex(index);

    // 只有 K-Means 支持法线/颜色维度
    bool show_dims = (index == 2); // K-Means
    dim_group_->setVisible(show_dims);
    weight_panel_->setVisible(show_dims && (check_normal_->isChecked() || check_color_->isChecked()));
    this->adjustSize();
}

void ClusteringDialog::onDimensionToggled()
{
    bool show_weights = check_normal_->isChecked() || check_color_->isChecked();
    weight_panel_->setVisible(show_weights);

    if (show_weights) {
        // 控制 Normal/Color 行的显隐
        row_normal_->setVisible(check_normal_->isChecked());
        row_color_->setVisible(check_color_->isChecked());

        // 隐藏对应的 QFormLayout label
        auto* form = qobject_cast<QFormLayout*>(weight_panel_->layout());
        if (form) {
            if (auto* label = form->labelForField(row_normal_))
                label->setVisible(check_normal_->isChecked());
            if (auto* label = form->labelForField(row_color_))
                label->setVisible(check_color_->isChecked());
        }

        // 未勾选维度权重置 0，然后重新归一化
        if (!check_normal_->isChecked()) {
            slider_normal_->blockSignals(true);
            dspin_normal_->blockSignals(true);
            slider_normal_->setValue(0);
            dspin_normal_->setValue(0.0);
            slider_normal_->blockSignals(false);
            dspin_normal_->blockSignals(false);
        }
        if (!check_color_->isChecked()) {
            slider_color_->blockSignals(true);
            dspin_color_->blockSignals(true);
            slider_color_->setValue(0);
            dspin_color_->setValue(0.0);
            slider_color_->blockSignals(false);
            dspin_color_->blockSignals(false);
        }
        onWeightChanged();
    }
}

int ClusteringDialog::findChangedIndex(double* old_values, double* new_values)
{
    for (int i = 0; i < 3; i++) {
        if (std::abs(old_values[i] - new_values[i]) > 1e-9)
            return i;
    }
    return -1;
}

void ClusteringDialog::onWeightChanged()
{
    double old_values[3] = {
        dspin_pos_->value(),
        dspin_normal_->value(),
        dspin_color_->value()
    };

    double total = old_values[0] + old_values[1] + old_values[2];

    if (total > 0.0 && std::abs(total - 1.0) > 1e-9) {
        // 归一化到总和 = 1.0
        double new_values[3] = {
            old_values[0] / total,
            old_values[1] / total,
            old_values[2] / total
        };

        // 找到触发变化的索引
        // 通过 sender() 判断是哪个控件触发的
        QObject* sender_obj = sender();
        int changed = -1;
        if (sender_obj == slider_pos_ || sender_obj == dspin_pos_) changed = 0;
        else if (sender_obj == slider_normal_ || sender_obj == dspin_normal_) changed = 1;
        else if (sender_obj == slider_color_ || sender_obj == dspin_color_) changed = 2;

        if (changed >= 0) {
            // 保持触发控件不变，其余按比例缩放
            double remaining = 1.0 - new_values[changed];
            if (remaining < 0.0) remaining = 0.0;

            double other_total = 0.0;
            for (int i = 0; i < 3; i++) {
                if (i != changed) other_total += new_values[i];
            }

            if (other_total > 1e-9) {
                for (int i = 0; i < 3; i++) {
                    if (i != changed) {
                        new_values[i] = (new_values[i] / other_total) * remaining;
                    }
                }
            } else {
                // 均分
                int other_count = 0;
                for (int i = 0; i < 3; i++) {
                    if (i != changed) other_count++;
                }
                if (other_count > 0) {
                    for (int i = 0; i < 3; i++) {
                        if (i != changed) new_values[i] = remaining / other_count;
                    }
                }
            }
        }

        updateWeightUI(new_values);
    }

    // 更新提示文本
    double p = dspin_pos_->value();
    double n = dspin_normal_->value();
    double c = dspin_color_->value();
    QStringList parts;
    if (p > 0.001) parts << QString("Position %1%").arg(static_cast<int>(p * 100));
    if (n > 0.001) parts << QString("Normal %1%").arg(static_cast<int>(n * 100));
    if (c > 0.001) parts << QString("Color %1%").arg(static_cast<int>(c * 100));
    label_weight_hint_->setText(parts.join(" | "));
}

void ClusteringDialog::updateWeightUI(double values[3])
{
    slider_pos_->blockSignals(true);
    dspin_pos_->blockSignals(true);
    slider_pos_->setValue(static_cast<int>(values[0] * 100));
    dspin_pos_->setValue(values[0]);
    slider_pos_->blockSignals(false);
    dspin_pos_->blockSignals(false);

    slider_normal_->blockSignals(true);
    dspin_normal_->blockSignals(true);
    slider_normal_->setValue(static_cast<int>(values[1] * 100));
    dspin_normal_->setValue(values[1]);
    slider_normal_->blockSignals(false);
    dspin_normal_->blockSignals(false);

    slider_color_->blockSignals(true);
    dspin_color_->blockSignals(true);
    slider_color_->setValue(static_cast<int>(values[2] * 100));
    dspin_color_->setValue(values[2]);
    slider_color_->blockSignals(false);
    dspin_color_->blockSignals(false);
}

void ClusteringDialog::onApply()
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    // ========== Step 1: 读取 UI 参数 ==========
    int algorithm = cbox_algorithm_->currentIndex();
    double pos_w = dspin_pos_->value();
    double normal_w = check_normal_->isChecked() ? dspin_normal_->value() : 0.0;
    double color_w = check_color_->isChecked() ? dspin_color_->value() : 0.0;

    // ========== Step 2: 隐藏对话框 ==========
    this->hide();
    QCoreApplication::processEvents();

    // ========== Step 3: 显示进度对话框 ==========
    m_progress->showProgress("Clustering...");

    // ========== Step 4: 设置取消标志 ==========
    auto* cancel = new std::atomic<bool>(false);
    auto* progress_closed = new std::atomic<bool>(false);
    connect(m_progress, &pw::ProgressManager::cancelRequested,
            this, [=]() {
                *cancel = true;
                m_canceled.store(true);
                m_progress->closeProgress();
                progress_closed->store(true);
                printW("Clustering canceled.");
            });

    // ========== Step 5: 进度回调 ==========
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ========== Step 6: 提交算法 ==========
    auto cloud = m_cloud;
    m_canceled.store(false);

    // 各算法参数
    double tolerance = dspin_tolerance_->value();
    int minClusterSize = spin_min_cluster_->value();
    int maxClusterSize = spin_max_cluster_->value();
    double eps = dspin_eps_->value();
    int minPts = spin_min_pts_->value();
    int k = spin_k_->value();
    int maxIter = spin_max_iter_->value();
    bool split = check_split_->isChecked();

    auto future = QtConcurrent::run([cloud, algorithm, normal_w, color_w,
                                     tolerance, minClusterSize, maxClusterSize,
                                     eps, minPts, k, maxIter, cancel, on_progress]() {
        switch (algorithm) {
        case 0: // Euclidean
            return pw::Segmentation::EuclideanClusterExtraction(cloud, false,
                tolerance, minClusterSize, maxClusterSize, cancel, on_progress);
        case 1: // DBSCAN
            return pw::Segmentation::DBSCANClusterExtraction(cloud, false,
                eps, minPts, minClusterSize, maxClusterSize,
                normal_w, color_w, cancel, on_progress);
        case 2: // K-Means
            return pw::Segmentation::KMeansClusterExtraction(cloud,
                k, maxIter, normal_w, color_w, cancel, on_progress);
        default:
            return pw::SegmentationResult{};
        }
    });

    // ========== Step 7: 监听完成信号 ==========
    auto* watcher = new QFutureWatcher<pw::SegmentationResult>(this);
    connect(watcher, &QFutureWatcher<pw::SegmentationResult>::finished, this,
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

            printI(QString("Clustering done in %1 ms, %2 cluster(s) generated.")
                       .arg(result.time_ms)
                       .arg(result.clouds.size()));

            if (split) {
                // Split 模式：每个聚类作为单独点云添加到文件树
                std::vector<pw::Cloud::Ptr> results;
                for (size_t i = 0; i < result.clouds.size(); i++) {
                    auto& c = result.clouds[i];
                    c->setId(cloud->id() + "-cluster" + std::to_string(i));
                    c->makeAdaptive();
                    results.push_back(c);
                }
                QString groupName = QString::fromStdString(cloud->id()) + "_Clustering";
                m_cloudtree->addResultGroup(cloud, results, groupName);
            } else {
                // 合并模式：复制原始点云，添加标量字段存储聚类标签
                auto labeled = cloud->clone();
                labeled->setId(cloud->id() + "_clustering");
                if (!result.labels.empty())
                    labeled->addScalarField("cluster_label", result.labels);
                labeled->makeAdaptive();
                std::vector<pw::Cloud::Ptr> results = {labeled};
                m_cloudtree->addResultGroup(cloud, results, QString::fromStdString(cloud->id()) + "_Clustering");
            }

            this->accept();
        });

    watcher->setFuture(future);
}

void ClusteringDialog::onCancel()
{
    this->close();
}
