#include "compute_hull_dialog.h"

#include "core/cloud.h"
#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QCoreApplication>
#include <QTimer>

#include <QtConcurrent/QtConcurrent>

// ======================== Constructor ========================

ComputeHullDialog::ComputeHullDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_canceled(false)
{
    setupUi();
    this->setWindowTitle("Compute Hull");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(340, 280);
    QTimer::singleShot(0, this, [this]() { this->adjustSize(); });
}

ComputeHullDialog::~ComputeHullDialog() = default;

// ======================== setupUi ========================

void ComputeHullDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- Method Selection ---
    auto* method_group = new QGroupBox("Hull Type", this);
    auto* method_layout = new QVBoxLayout(method_group);
    method_layout->setContentsMargins(6, 10, 6, 6);
    method_layout->setSpacing(4);

    auto* btn_group = new QButtonGroup(this);
    radio_convex_ = new QRadioButton("Convex Hull", this);
    radio_concave_ = new QRadioButton("Concave Hull", this);
    btn_group->addButton(radio_convex_, 0);
    btn_group->addButton(radio_concave_, 1);
    radio_convex_->setChecked(true);
    method_layout->addWidget(radio_convex_);
    method_layout->addWidget(radio_concave_);
    main_layout->addWidget(method_group);

    // --- Concave Parameters (only visible for concave) ---
    auto* concave_group = new QGroupBox("Concave Parameters", this);
    auto* concave_layout = new QFormLayout(concave_group);
    concave_layout->setContentsMargins(6, 10, 6, 6);
    concave_layout->setSpacing(4);

    dspin_alpha_ = new QDoubleSpinBox(this);
    dspin_alpha_->setRange(0.001, 1000.0);
    dspin_alpha_->setDecimals(3);
    dspin_alpha_->setValue(0.5);
    dspin_alpha_->setSingleStep(0.1);
    concave_layout->addRow("Alpha Value:", dspin_alpha_);
    main_layout->addWidget(concave_group);
    setConcaveParamsEnabled(false);

    // --- Dimension ---
    auto* dim_group = new QGroupBox("Dimension", this);
    auto* dim_layout = new QVBoxLayout(dim_group);
    dim_layout->setContentsMargins(6, 10, 6, 6);
    dim_layout->setSpacing(4);

    auto* dim_btn_group = new QButtonGroup(this);
    radio_3d_ = new QRadioButton("3D Envelope", this);
    radio_2d_ = new QRadioButton("Project to 2D Planar", this);
    dim_btn_group->addButton(radio_3d_, 0);
    dim_btn_group->addButton(radio_2d_, 1);
    radio_3d_->setChecked(true);
    dim_layout->addWidget(radio_3d_);
    dim_layout->addWidget(radio_2d_);
    main_layout->addWidget(dim_group);

    // --- Options ---
    check_keep_info_ = new QCheckBox("Keep RGB and Normal information", this);
    main_layout->addWidget(check_keep_info_);

    // --- Buttons ---
    auto* btn_row = new QHBoxLayout();
    btn_compute_ = new QPushButton("Compute", this);
    btn_close_ = new QPushButton("Close", this);
    btn_row->addWidget(btn_compute_);
    btn_row->addWidget(btn_close_);
    main_layout->addLayout(btn_row);

    // --- Signals ---
    connect(radio_convex_, &QRadioButton::toggled, this, &ComputeHullDialog::onHullTypeChanged);
    connect(radio_concave_, &QRadioButton::toggled, this, &ComputeHullDialog::onHullTypeChanged);
    connect(btn_compute_, &QPushButton::clicked, this, &ComputeHullDialog::onCompute);
    connect(btn_close_, &QPushButton::clicked, this, &ComputeHullDialog::onCancel);
}

// ======================== Helpers ========================

void ComputeHullDialog::setConcaveParamsEnabled(bool enabled)
{
    // Find the concave parameters group and enable/disable
    for (int i = 0; i < this->layout()->count(); ++i) {
        auto* item = this->layout()->itemAt(i);
        if (auto* group = qobject_cast<QGroupBox*>(item->widget())) {
            if (group->title() == "Concave Parameters") {
                group->setVisible(enabled);
                break;
            }
        }
    }
}

// ======================== Init / Reset ========================

void ComputeHullDialog::init()
{
    auto selection = m_cloudtree->getSelectedClouds();
    if (!selection.empty()) {
        m_cloud = selection.front();
    }
}

void ComputeHullDialog::reset()
{
    m_cloudtree->closeProgress();
    m_canceled.store(true);
    m_cloud.reset();
}

// ======================== Slots ========================

void ComputeHullDialog::onHullTypeChanged()
{
    setConcaveParamsEnabled(radio_concave_->isChecked());
    QTimer::singleShot(0, this, [this]() { this->adjustSize(); });
}

void ComputeHullDialog::onCompute()
{
    if (!m_cloud) {
        printW("Please select a cloud.");
        return;
    }

    // ========== Step 1: 读取 UI 参数 ==========
    bool is_convex = radio_convex_->isChecked();
    double alpha = dspin_alpha_->value();
    int dimension = radio_2d_->isChecked() ? 2 : 3;
    bool keep_info = check_keep_info_->isChecked();

    // ========== Step 2: 隐藏对话框 ==========
    this->hide();
    QCoreApplication::processEvents();

    // ========== Step 3: 显示进度条 ==========
    QString algo_name = is_convex ? "Convex Hull" : "Concave Hull";
    m_cloudtree->showProgress(algo_name + "...");

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
                    printW(algo_name + " canceled.");
                });
    }

    // ========== Step 5: 进度回调 ==========
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    // ========== Step 6: 提交算法到工作线程 ==========
    auto cloud = m_cloud;
    m_canceled.store(false);

    auto future = QtConcurrent::run([cloud, is_convex, alpha, keep_info, dimension,
                                     cancel, on_progress]() -> ct::SurfaceResult {
        if (is_convex) {
            return ct::Surface::ConvexHull(cloud, keep_info, dimension, cancel, on_progress);
        } else {
            return ct::Surface::ConcaveHull(cloud, alpha, keep_info, dimension, cancel, on_progress);
        }
    });

    // ========== Step 7: 监听完成信号 ==========
    auto* watcher = new QFutureWatcher<ct::SurfaceResult>(this);
    connect(watcher, &QFutureWatcher<ct::SurfaceResult>::finished, this,
        [=]() {
            if (!progress_closed->load()) {
                m_cloudtree->closeProgress();
            }
            delete cancel;
            delete progress_closed;

            auto result = watcher->result();
            watcher->deleteLater();

            if (m_canceled.load() || !result.mesh) {
                if (result.mesh == nullptr && !m_canceled.load()) {
                    printW(QString("Compute Hull failed: %1")
                               .arg(QString::fromStdString(result.error_msg)));
                }
                this->reject();
                return;
            }

            QString suffix = is_convex ? "_convex_hull" : "_concave_hull";
            QString result_id = QString::fromStdString(cloud->id()) + suffix;

            // 提取 mesh 顶点作为 Cloud 插入树中，并注册完整 mesh 以支持保存
            pcl::PointCloud<ct::PointXYZRGBN> mesh_points;
            pcl::fromPCLPointCloud2(result.mesh->cloud, mesh_points);
            if (mesh_points.size() > 0) {
                auto mesh_cloud = ct::Cloud::fromPCL_XYZRGBN(mesh_points);
                mesh_cloud->setId(result_id.toStdString());
                mesh_cloud->makeAdaptive();

                QTreeWidgetItem* origin_item = m_cloudtree->getItemById(
                    QString::fromStdString(cloud->id()));
                m_cloudtree->insertCloud(mesh_cloud, origin_item, true,
                                         ct::MountStrategy::Sibling,
                                         ct::NodeMesh);

                m_cloudtree->registerMesh(result_id, result.mesh);
            }

            printI(QString("%1 done in %2 ms, result: [%3]")
                       .arg(algo_name)
                       .arg(result.time_ms)
                       .arg(result_id));

            this->accept();
        });

    watcher->setFuture(future);
}

void ComputeHullDialog::onCancel()
{
    this->close();
}
