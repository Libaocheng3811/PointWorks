#include "point_pairs_alignment.h"

#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "ui/dialog/processingdialog.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QMenu>
#include <QHeaderView>
#include <QDialog>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QFileDialog>
#include <QTextStream>
#include <QGroupBox>
#include <QtConcurrent/QtConcurrent>
#include <cmath>

#include <pcl/common/transforms.h>

// ======================== Constructor ========================

PointPairsAlignment::PointPairsAlignment(QWidget* parent)
    : ct::CustomDialog(parent), m_is_picking(false), m_has_preview(false), m_is_computing(false), m_canceled(false)
{
    setupUi();

    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

PointPairsAlignment::~PointPairsAlignment() = default;

// ======================== setupUi ========================

static void setupTable(QTableWidget* table, QWidget* parent)
{
    table->setColumnCount(9);
    table->setHorizontalHeaderLabels({"#", "X", "Y", "Z", "dX", "dY", "dZ", "Error", ""});
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(8, QHeaderView::Fixed);
    for (int i = 1; i < 8; i++)
        table->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Stretch);
    table->setColumnWidth(0, 24);
    table->setColumnWidth(8, 26);
    table->verticalHeader()->setVisible(false);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setAlternatingRowColors(true);
    table->setShowGrid(false);
}

void PointPairsAlignment::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // ===== Row 1: Source / Target cloud selection + Swap =====
    auto* cloud_row = new QHBoxLayout();
    cloud_row->addWidget(new QLabel("Source:", this));
    cbox_source_ = new QComboBox(this);
    cbox_source_->setMinimumWidth(120);
    cloud_row->addWidget(cbox_source_, 1);
    cloud_row->addSpacing(6);
    btn_swap_ = new QPushButton("\u21C4", this);
    btn_swap_->setFixedSize(28, 28);
    btn_swap_->setToolTip("Swap source and target");
    cloud_row->addWidget(btn_swap_);
    cloud_row->addSpacing(6);
    cloud_row->addWidget(new QLabel("Target:", this));
    cbox_target_ = new QComboBox(this);
    cbox_target_->setMinimumWidth(120);
    cloud_row->addWidget(cbox_target_, 1);
    main_layout->addLayout(cloud_row);

    // ===== Row 2: Source Points GroupBox =====
    group_source_ = new QGroupBox("Source Points", this);
    group_source_->setStyleSheet(
        "QGroupBox { color: #e74c3c; font-weight: bold; border: 1px solid #e74c3c; "
        "border-radius: 3px; margin-top: 8px; padding-top: 14px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }");
    auto* src_layout = new QVBoxLayout(group_source_);
    src_layout->setContentsMargins(4, 4, 4, 4);
    src_layout->setSpacing(4);

    auto* src_toolbar = new QHBoxLayout();
    check_show_source_ = new QCheckBox("Show", this);
    check_show_source_->setChecked(true);
    btn_clear_src_ = new QPushButton(QIcon(":/res/icon/clear.svg"), "", this);
    btn_clear_src_->setFixedSize(24, 24);
    btn_clear_src_->setToolTip("Clear");
    btn_manual_src_ = new QPushButton(QIcon(":/res/icon2/pencil.svg"), "", this);
    btn_manual_src_->setFixedSize(24, 24);
    btn_manual_src_->setToolTip("Manual Input");
    src_toolbar->addWidget(check_show_source_);
    src_toolbar->addStretch();
    src_toolbar->addWidget(btn_clear_src_);
    src_toolbar->addWidget(btn_manual_src_);
    btn_import_src_ = new QPushButton("Import...", this);
    btn_import_src_->setFixedHeight(24);
    src_toolbar->addWidget(btn_import_src_);
    src_layout->addLayout(src_toolbar);

    table_source_ = new QTableWidget(this);
    setupTable(table_source_, this);
    table_source_->setMaximumHeight(130);
    table_source_->setContextMenuPolicy(Qt::CustomContextMenu);
    src_layout->addWidget(table_source_);
    main_layout->addWidget(group_source_);

    // ===== Row 3: Target Points GroupBox =====
    group_target_ = new QGroupBox("Target Points", this);
    group_target_->setStyleSheet(
        "QGroupBox { color: #3498db; font-weight: bold; border: 1px solid #3498db; "
        "border-radius: 3px; margin-top: 8px; padding-top: 14px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }");
    auto* tgt_layout = new QVBoxLayout(group_target_);
    tgt_layout->setContentsMargins(4, 4, 4, 4);
    tgt_layout->setSpacing(4);

    auto* tgt_toolbar = new QHBoxLayout();
    check_show_target_ = new QCheckBox("Show", this);
    check_show_target_->setChecked(true);
    btn_clear_tgt_ = new QPushButton(QIcon(":/res/icon/clear.svg"), "", this);
    btn_clear_tgt_->setFixedSize(24, 24);
    btn_clear_tgt_->setToolTip("Clear");
    btn_manual_tgt_ = new QPushButton(QIcon(":/res/icon2/pencil.svg"), "", this);
    btn_manual_tgt_->setFixedSize(24, 24);
    btn_manual_tgt_->setToolTip("Manual Input");
    btn_import_tgt_ = new QPushButton("Import...", this);
    btn_import_tgt_->setFixedHeight(24);
    tgt_toolbar->addWidget(check_show_target_);
    tgt_toolbar->addStretch();
    tgt_toolbar->addWidget(btn_clear_tgt_);
    tgt_toolbar->addWidget(btn_manual_tgt_);
    tgt_toolbar->addWidget(btn_import_tgt_);
    tgt_layout->addLayout(tgt_toolbar);

    table_target_ = new QTableWidget(this);
    setupTable(table_target_, this);
    table_target_->setMaximumHeight(130);
    table_target_->setContextMenuPolicy(Qt::CustomContextMenu);
    tgt_layout->addWidget(table_target_);
    main_layout->addWidget(group_target_);

    // ===== Row 4: Info (Pairs + RMS) =====
    auto* info_row = new QHBoxLayout();
    label_pair_count_ = new QLabel("Pairs: 0", this);
    label_pair_count_->setStyleSheet("font-weight: bold;");
    info_row->addWidget(label_pair_count_);
    info_row->addStretch();
    label_rms_ = new QLabel("RMS: --", this);
    label_rms_->setStyleSheet("font-weight: bold; color: #27ae60;");
    info_row->addWidget(label_rms_);
    main_layout->addLayout(info_row);

    // ===== Row 5: Constraints =====
    auto* constraint_group = new QGroupBox("Constraints", this);
    constraint_group->setStyleSheet(
        "QGroupBox { border: 1px solid #ccc; border-radius: 3px; "
        "margin-top: 8px; padding-top: 14px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }");
    auto* constraint_layout = new QVBoxLayout(constraint_group);
    constraint_layout->setContentsMargins(4, 4, 4, 4);
    constraint_layout->setSpacing(4);

    check_adjust_scale_ = new QCheckBox("Adjust Scale (7 DOF)", this);

    auto* rot_trans_row = new QHBoxLayout();
    rot_trans_row->addWidget(new QLabel("Rotation:", this));
    cbox_rotation_ = new QComboBox(this);
    cbox_rotation_->addItems({"None", "X", "Y", "Z", "XY", "XZ", "YZ", "XYZ"});
    cbox_rotation_->setCurrentIndex(7);
    cbox_rotation_->setFixedWidth(60);
    rot_trans_row->addWidget(cbox_rotation_);
    rot_trans_row->addSpacing(12);
    rot_trans_row->addWidget(new QLabel("T:", this));
    check_tx_ = new QCheckBox("X", this);
    check_ty_ = new QCheckBox("Y", this);
    check_tz_ = new QCheckBox("Z", this);
    check_tx_->setChecked(true);
    check_ty_->setChecked(true);
    check_tz_->setChecked(true);
    rot_trans_row->addWidget(check_tx_);
    rot_trans_row->addWidget(check_ty_);
    rot_trans_row->addWidget(check_tz_);
    rot_trans_row->addStretch();
    constraint_layout->addLayout(rot_trans_row);
    constraint_layout->addWidget(check_adjust_scale_);

    auto* filter_row = new QHBoxLayout();
    check_rms_filter_ = new QCheckBox("RMS Filter", this);
    filter_row->addWidget(check_rms_filter_);
    spin_rms_threshold_ = new QDoubleSpinBox(this);
    spin_rms_threshold_->setRange(0, 1e9);
    spin_rms_threshold_->setDecimals(4);
    spin_rms_threshold_->setSingleStep(0.001);
    spin_rms_threshold_->setValue(0.1);
    spin_rms_threshold_->setEnabled(false);
    filter_row->addWidget(spin_rms_threshold_);
    filter_row->addWidget(new QLabel("(remove pairs with error > threshold after align)", this));
    filter_row->addStretch();
    constraint_layout->addLayout(filter_row);

    main_layout->addWidget(constraint_group);

    // ===== Row 6: Buttons =====
    auto* btn_row = new QHBoxLayout();
    btn_start_ = new QPushButton("Start", this);
    btn_align_ = new QPushButton("Align", this);
    btn_align_->setEnabled(false);
    btn_reset_ = new QPushButton("Reset", this);
    btn_apply_ = new QPushButton("Apply", this);
    btn_apply_->setEnabled(false);
    btn_cancel_ = new QPushButton("Cancel", this);
    btn_cancel_->setEnabled(false);
    btn_close_ = new QPushButton("Close", this);
    btn_row->addWidget(btn_start_);
    btn_row->addWidget(btn_align_);
    btn_row->addWidget(btn_reset_);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel_);
    btn_row->addWidget(btn_close_);
    main_layout->addLayout(btn_row);

    // ===== Row 7: Status =====
    label_status_ = new QLabel("Select source and target clouds, then click Start.", this);
    label_status_->setWordWrap(true);
    label_status_->setStyleSheet("color: #888; font-size: 11px;");
    main_layout->addWidget(label_status_);

    // ===== Signals =====
    connect(btn_start_, &QPushButton::clicked, this, &PointPairsAlignment::onStartStop);
    connect(btn_reset_, &QPushButton::clicked, this, &PointPairsAlignment::onReset);
    connect(btn_align_, &QPushButton::clicked, this, &PointPairsAlignment::onAlign);
    connect(btn_apply_, &QPushButton::clicked, this, &PointPairsAlignment::onApply);
    connect(btn_cancel_, &QPushButton::clicked, this, &PointPairsAlignment::onCancel);
    connect(btn_close_, &QPushButton::clicked, this, &PointPairsAlignment::close);
    connect(btn_swap_, &QPushButton::clicked, this, &PointPairsAlignment::onSwapSourceTarget);
    connect(btn_clear_src_, &QPushButton::clicked, this, &PointPairsAlignment::onClearSourcePoints);
    connect(btn_clear_tgt_, &QPushButton::clicked, this, &PointPairsAlignment::onClearTargetPoints);
    connect(btn_manual_src_, &QPushButton::clicked, this, &PointPairsAlignment::onAddSourcePoint);
    connect(btn_manual_tgt_, &QPushButton::clicked, this, &PointPairsAlignment::onAddTargetPoint);
    connect(btn_import_src_, &QPushButton::clicked, this, &PointPairsAlignment::onImportSourcePoints);
    connect(btn_import_tgt_, &QPushButton::clicked, this, &PointPairsAlignment::onImportTargetPoints);
    connect(check_rms_filter_, &QCheckBox::toggled, this, &PointPairsAlignment::onRmsFilterToggled);
    connect(check_show_source_, &QCheckBox::toggled, this, &PointPairsAlignment::onToggleSourceVisibility);
    connect(check_show_target_, &QCheckBox::toggled, this, &PointPairsAlignment::onToggleTargetVisibility);

    connect(table_source_, &QTableWidget::customContextMenuRequested, this, [=](const QPoint& pos) {
        int row = table_source_->rowAt(pos.y());
        if (row < 0 || row >= m_source_points.size()) return;
        QMenu menu(this);
        menu.addAction("Delete this point", this, [=]() { deleteSourcePoint(row); });
        menu.exec(table_source_->viewport()->mapToGlobal(pos));
    });
    connect(table_target_, &QTableWidget::customContextMenuRequested, this, [=](const QPoint& pos) {
        int row = table_target_->rowAt(pos.y());
        if (row < 0 || row >= m_target_points.size()) return;
        QMenu menu(this);
        menu.addAction("Delete this point", this, [=]() { deleteTargetPoint(row); });
        menu.exec(table_target_->viewport()->mapToGlobal(pos));
    });

    this->setWindowTitle("Point-Pairs Alignment");
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(480, 700);
}

// ======================== Init / Reset ========================

void PointPairsAlignment::init()
{
    refreshCloudList();
}

void PointPairsAlignment::reset()
{
    stopPicking();

    // 保存 source_id 用于恢复可见性（必须在 clear 之前）
    QString saved_source_id = m_source_id;

    // 清除 3D 号码牌（必须在 points 清空之前）
    clearLabels();

    m_source_points.clear();
    m_target_points.clear();
    m_last_result = {};
    m_has_preview = false;
    m_original_source_cloud.reset();
    m_source_id.clear();
    m_last_align_snapshot.clear();

    clearAllLines();
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->removePointCloud(MARKER_SRC_ID);
    m_cloudview->removePointCloud(MARKER_TGT_ID);

    // 恢复源点云可见
    if (!saved_source_id.isEmpty())
        m_cloudview->setPointCloudVisibility(saved_source_id, true);

    m_cloudview->refresh();
    updateTables();
    label_status_->setText("Select source and target clouds, then click Start.");
    label_rms_->setText("RMS: --");
    btn_align_->setEnabled(false);
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    btn_start_->setEnabled(true);
}

void PointPairsAlignment::deinit()
{
    // 清理预览云（closeEvent 已处理还原，这里做防御性清理）
    m_cloudview->removePointCloud(PREVIEW_ID);

    // 恢复所有点云可见性
    restoreAllCloudVisibility();

    // 恢复 source/target checkbox
    if (check_show_source_ && !check_show_source_->isChecked())
        check_show_source_->setChecked(true);
    if (check_show_target_ && !check_show_target_->isChecked())
        check_show_target_->setChecked(true);

    // 清理 3D 号码牌
    clearLabels();

    reset();
}

// ======================== showEvent ========================

void PointPairsAlignment::showEvent(QShowEvent* event)
{
    ct::CustomDialog::showEvent(event);
    if (m_cloudview) {
        QPoint pos = m_cloudview->mapToGlobal(QPoint(0, 0));
        move(pos.x() + m_cloudview->width() - width() - 9, pos.y() + 9);
    }
}

void PointPairsAlignment::closeEvent(QCloseEvent* event)
{
    // 移除预览云，恢复源点云可见（源点云从未被修改，无需还原）
    if (m_has_preview) {
        m_cloudview->removePointCloud(PREVIEW_ID);
        m_cloudview->setPointCloudVisibility(m_source_id, true);
        m_cloudview->refresh();
        printI("Point-pairs alignment reverted (dialog closed without apply).");
    }

    ct::CustomDialog::closeEvent(event);
}

// ======================== Slots ========================

void PointPairsAlignment::onStartStop()
{
    if (!m_is_picking) {
        if (cbox_source_->currentIndex() < 0 || cbox_target_->currentIndex() < 0) {
            printW("Please select source and target clouds.");
            return;
        }
        if (cbox_source_->currentIndex() == cbox_target_->currentIndex()) {
            printW("Source and target must be different clouds.");
            return;
        }

        m_is_picking = true;
        btn_start_->setText("Stop");
        cbox_source_->setEnabled(false);
        cbox_target_->setEnabled(false);

        hideNonSelectedClouds();

        connect(m_cloudview, &ct::CloudView::mouseLeftPressed,
                this, &PointPairsAlignment::onMouseLeftPressed);

        m_cloudview->showInfo("Click on Source or Target cloud to pick points.", 1);
        label_status_->setText("Click on any cloud to pick points.");
    } else {
        stopPicking();
    }
}

void PointPairsAlignment::stopPicking()
{
    if (!m_is_picking) return;
    m_is_picking = false;
    disconnect(m_cloudview, &ct::CloudView::mouseLeftPressed,
               this, &PointPairsAlignment::onMouseLeftPressed);
    m_cloudview->clearInfo();
    btn_start_->setText("Start");
    cbox_source_->setEnabled(true);
    cbox_target_->setEnabled(true);
}

void PointPairsAlignment::onReset()
{
    m_cloudtree->closeProgress();
    reset();
}

void PointPairsAlignment::onAlign()
{
    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    if (pair_count < 3) {
        printW(QString("Need at least 3 matched pairs (have %1 source, %2 target).")
               .arg(m_source_points.size()).arg(m_target_points.size()));
        return;
    }

    // 参数快照对比：非首次且参数一致则跳过
    {
        ct::ParamSnapshot snap;
        snap.set("source_id", cbox_source_->currentText());
        snap.set("target_id", cbox_target_->currentText());
        snap.set("rotation", cbox_rotation_->currentIndex());
        snap.set("tx", check_tx_->isChecked());
        snap.set("ty", check_ty_->isChecked());
        snap.set("tz", check_tz_->isChecked());
        snap.set("adjust_scale", check_adjust_scale_->isChecked());
        snap.set("rms_filter", check_rms_filter_->isChecked());
        snap.set("rms_threshold", spin_rms_threshold_->value());
        snap.set("source_points", ct::ParamSnapshot::pointsToString(m_source_points));
        snap.set("target_points", ct::ParamSnapshot::pointsToString(m_target_points));
        if (snap == m_last_align_snapshot && m_has_preview) {
            printI("Parameters unchanged, skipping recomputation.");
            return;
        }
        m_last_align_snapshot = snap;
    }

    // m_source_points 始终保持原始坐标（用户拾取位置），不随变换修改
    std::vector<Eigen::Vector3d> src_pts, tgt_pts;
    src_pts.reserve(pair_count);
    tgt_pts.reserve(pair_count);
    for (int i = 0; i < pair_count; i++) {
        src_pts.push_back(m_source_points[i]);
        tgt_pts.push_back(m_target_points[i]);
    }

    // UI 状态
    btn_align_->setEnabled(false);
    btn_start_->setEnabled(false);
    m_is_computing = true;
    m_canceled.store(false);
    label_status_->setText("Computing...");

    // 显示模态进度条
    m_cloudtree->showProgress("Computing...");
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [this]() {
                    m_canceled.store(true);
                    if (m_cloudtree->m_processing_dialog)
                        m_cloudtree->m_processing_dialog->setMessage("Canceling...");
                });
    }

    auto params = currentConstraintParams();
    auto result_ptr = std::make_shared<ct::PointPairErrorResult>();

    // 首次时备份原始源点云并记录 ID
    bool first_time = !m_original_source_cloud;
    if (first_time) {
        auto clouds = m_cloudtree->getAllClouds();
        int si = cbox_source_->currentIndex();
        ct::Cloud::Ptr source = (si < (int)clouds.size()) ? clouds[si] : nullptr;
        if (!source) {
            m_is_computing = false;
            btn_align_->setEnabled(true);
            btn_start_->setEnabled(true);
            return;
        }
        m_source_id = QString::fromStdString(source->id());
        // 深拷贝原始点云作为备份
        auto pcl_src = source->toPCL_XYZRGBN();
        m_original_source_cloud = ct::Cloud::fromPCL_XYZRGBN(*pcl_src);
        m_original_source_cloud->setId(source->id());
    }

    // 捕获原始备份用于工作线程变换
    ct::Cloud::Ptr source_snap = m_original_source_cloud;

    auto future = QtConcurrent::run([this, source_snap, result_ptr, params, src_pts, tgt_pts]() -> ct::Cloud::Ptr {
        if (m_canceled.load()) return nullptr;

        *result_ptr = ct::Registration::ConstrainedPointPairsRegistration(src_pts, tgt_pts, params);
        if (m_canceled.load()) return nullptr;

        // 始终从原始备份变换，不会叠加
        // 注意：toPCL_XYZRGBN() 返回内部缓存 shared_ptr，必须深拷贝后再变换，否则会污染备份
        auto pcl_src = source_snap->toPCL_XYZRGBN();
        auto pcl_copy = std::make_shared<pcl::PointCloud<ct::PointXYZRGBN>>(*pcl_src);
        pcl::transformPointCloud(*pcl_copy, *pcl_copy, result_ptr->matrix.cast<float>());
        auto transformed = ct::Cloud::fromPCL_XYZRGBN(*pcl_copy);
        transformed->setId(PREVIEW_ID);
        return transformed;
    });

    auto* watcher = new QFutureWatcher<ct::Cloud::Ptr>(this);
    watcher->setFuture(future);
    connect(watcher, &QFutureWatcher<ct::Cloud::Ptr>::finished, this,
        [this, watcher, result_ptr]() mutable {
        m_cloudtree->closeProgress();
        auto transformed = watcher->result();
        watcher->deleteLater();

        m_is_computing = false;
        btn_start_->setEnabled(true);

        if (m_canceled.load() || !transformed) {
            label_status_->setText("Canceled.");
            btn_align_->setEnabled(true);
            return;
        }

        m_last_result = *result_ptr;
        label_rms_->setText(QString("RMS: %1").arg(m_last_result.rms, 0, 'f', 6));
        updateTableErrors(m_last_result);

        // 如果启用了 RMS Filter，自动剔除粗差点
        if (check_rms_filter_->isChecked()) {
            double threshold = spin_rms_threshold_->value();
            int pc = std::min({m_source_points.size(), m_target_points.size(),
                              (int)m_last_result.deltas.size(), (int)m_last_result.errors.size()});
            if (pc >= 4) {
                QVector<int> to_remove;
                for (int i = 0; i < pc; i++) {
                    if (m_last_result.errors[i] > threshold)
                        to_remove.append(i);
                }
                if (!to_remove.isEmpty() && pc - to_remove.size() >= 3) {
                    for (int i = to_remove.size() - 1; i >= 0; i--) {
                        m_source_points.removeAt(to_remove[i]);
                        m_target_points.removeAt(to_remove[i]);
                    }
                    printI(QString("RMS filter: removed %1 pair(s) (error > %2), %3 remaining.")
                           .arg(to_remove.size()).arg(threshold, 0, 'f', 4)
                           .arg(std::min(m_source_points.size(), m_target_points.size())));
                }
            }
        }

        // 移除旧预览云（如果有），隐藏源点云，显示新的预览云
        m_cloudview->removePointCloud(PREVIEW_ID);
        m_cloudview->setPointCloudVisibility(m_source_id, false);
        m_cloudview->addPointCloud(transformed);
        m_cloudview->refresh();

        rebuildMarkers();
        rebuildLabels();
        updateAllLines();
        updateTables();

        m_has_preview = true;
        btn_align_->setEnabled(true);
        btn_apply_->setEnabled(true);
        btn_cancel_->setEnabled(true);
        label_status_->setText("Preview. Click Apply to confirm, Cancel to revert, or re-click Align.");

        int pair_count = std::min(m_source_points.size(), m_target_points.size());
        printI(QString("Align preview: %1 pairs, RMS: %2, Scale: %3")
               .arg(pair_count).arg(m_last_result.rms, 0, 'f', 4).arg(m_last_result.scale, 0, 'f', 6));
    });
}

void PointPairsAlignment::onApply()
{
    if (!m_has_preview) return;

    int pair_count = std::min(m_source_points.size(), m_target_points.size());

    // 从原始备份变换，写入树中源点云
    auto pcl_src = m_original_source_cloud->toPCL_XYZRGBN();
    auto pcl_copy = std::make_shared<pcl::PointCloud<ct::PointXYZRGBN>>(*pcl_src);
    pcl::transformPointCloud(*pcl_copy, *pcl_copy, m_last_result.matrix.cast<float>());
    auto transformed = ct::Cloud::fromPCL_XYZRGBN(*pcl_copy);
    transformed->setId(m_source_id.toStdString());

    {
        auto all_clouds = m_cloudtree->getAllClouds();
        for (auto& c : all_clouds) {
            if (QString::fromStdString(c->id()) == m_source_id) {
                m_cloudtree->updateCloud(c, transformed);
                break;
            }
        }
    }

    // 移除预览云，恢复源点云可见
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->invalidateCloudRender(m_source_id);
    m_cloudview->refresh();

    m_original_source_cloud.reset();
    clearAllLines();
    m_has_preview = false;
    printI(QString("Point-pairs alignment applied (%1 pairs, RMS: %2).")
           .arg(pair_count).arg(m_last_result.rms, 0, 'f', 4));
    reset();
}

void PointPairsAlignment::onCancel()
{
    m_cloudtree->closeProgress();
    if (!m_has_preview) return;

    // 移除预览云，恢复源点云可见（源点云从未被修改）
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->refresh();

    clearAllLines();
    m_has_preview = false;
    label_rms_->setText("RMS: --");

    for (int i = 0; i < table_source_->rowCount(); i++)
        for (int c = 4; c < 8; c++)
            table_source_->item(i, c)->setText("");
    for (int i = 0; i < table_target_->rowCount(); i++)
        for (int c = 4; c < 8; c++)
            table_target_->item(i, c)->setText("");

    printI("Point-pairs alignment canceled.");

    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);
    label_status_->setText(QString("Canceled. %1 pairs ready. Click Align to retry.").arg(pair_count));
}

void PointPairsAlignment::onMouseLeftPressed(const ct::PointXY& pt)
{
    if (!m_is_picking) return;

    // 不指定目标点云，让 singlePick 自动检测点击的是哪个点云
    ct::PickResult res = doPick(pt);
    if (!res.valid || !res.cloud) {
        printW("Failed to pick point. Click on source or target cloud.");
        return;
    }

    // 判断点击的是 source 还是 target
    QString source_id = cbox_source_->currentText();
    QString target_id = cbox_target_->currentText();
    QString clicked_id = QString::fromStdString(res.cloud->id());

    Eigen::Vector3d picked_pt(res.point.x, res.point.y, res.point.z);

    if (clicked_id == source_id) {
        m_source_points.append(picked_pt);
    } else if (clicked_id == target_id) {
        m_target_points.append(picked_pt);
    } else {
        return; // 点击了无关点云，忽略
    }

    // 更新可视化
    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();

    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);

    bool is_src = (clicked_id == source_id);
    QString which = is_src ? "Source" : "Target";
    m_cloudview->showInfo(
        QString("%1 #%2 picked. Pairs: %3")
        .arg(which)
        .arg(is_src ? m_source_points.size() : m_target_points.size())
        .arg(pair_count), 1);
    label_status_->setText(
        QString("%1 #%2 picked. Source: %3, Target: %4, Pairs: %5")
        .arg(which)
        .arg(is_src ? m_source_points.size() : m_target_points.size())
        .arg(m_source_points.size())
        .arg(m_target_points.size())
        .arg(pair_count));
}

// ======================== Table Management ========================

void PointPairsAlignment::updateTables()
{
    table_source_->setRowCount(m_source_points.size());
    for (int i = 0; i < m_source_points.size(); i++) {
        const auto& p = m_source_points[i];
        table_source_->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));
        table_source_->setItem(i, 1, new QTableWidgetItem(QString::number(p[0], 'f', 6)));
        table_source_->setItem(i, 2, new QTableWidgetItem(QString::number(p[1], 'f', 6)));
        table_source_->setItem(i, 3, new QTableWidgetItem(QString::number(p[2], 'f', 6)));
        for (int c = 4; c < 8; c++)
            table_source_->setItem(i, c, new QTableWidgetItem(""));

        auto* btn_del = new QPushButton("x", table_source_);
        btn_del->setFixedSize(20, 20);
        btn_del->setStyleSheet("QPushButton { color: #e74c3c; font-weight: bold; border: none; background: transparent; }"
                               "QPushButton:hover { background: #fdd; }");
        table_source_->setCellWidget(i, 8, btn_del);
        connect(btn_del, &QPushButton::clicked, this, [=]() { deleteSourcePoint(i); });
    }
    if (!m_source_points.isEmpty()) table_source_->scrollToBottom();

    table_target_->setRowCount(m_target_points.size());
    for (int i = 0; i < m_target_points.size(); i++) {
        const auto& p = m_target_points[i];
        table_target_->setItem(i, 0, new QTableWidgetItem(QString::number(i + 1)));
        table_target_->setItem(i, 1, new QTableWidgetItem(QString::number(p[0], 'f', 6)));
        table_target_->setItem(i, 2, new QTableWidgetItem(QString::number(p[1], 'f', 6)));
        table_target_->setItem(i, 3, new QTableWidgetItem(QString::number(p[2], 'f', 6)));
        for (int c = 4; c < 8; c++)
            table_target_->setItem(i, c, new QTableWidgetItem(""));

        auto* btn_del = new QPushButton("x", table_target_);
        btn_del->setFixedSize(20, 20);
        btn_del->setStyleSheet("QPushButton { color: #3498db; font-weight: bold; border: none; background: transparent; }"
                               "QPushButton:hover { background: #def; }");
        table_target_->setCellWidget(i, 8, btn_del);
        connect(btn_del, &QPushButton::clicked, this, [=]() { deleteTargetPoint(i); });
    }
    if (!m_target_points.isEmpty()) table_target_->scrollToBottom();

    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    label_pair_count_->setText(QString("Pairs: %1").arg(pair_count));
}

void PointPairsAlignment::deleteSourcePoint(int index)
{
    if (index < 0 || index >= m_source_points.size()) return;
    if (m_has_preview) onCancel();
    m_source_points.removeAt(index);
    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();
    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    label_rms_->setText("RMS: --");
    printI(QString("Deleted source point #%1.").arg(index + 1));
}

void PointPairsAlignment::deleteTargetPoint(int index)
{
    if (index < 0 || index >= m_target_points.size()) return;
    if (m_has_preview) onCancel();
    m_target_points.removeAt(index);
    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();
    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    label_rms_->setText("RMS: --");
    printI(QString("Deleted target point #%1.").arg(index + 1));
}

void PointPairsAlignment::rebuildMarkers()
{
    double scale = calcLabelScale();
    int marker_size = std::max(8, static_cast<int>(scale * 4000));

    // --- Source markers (red) ---
    m_cloudview->removePointCloud(MARKER_SRC_ID);
    auto src_display = displaySourcePoints();
    if (check_show_source_->isChecked() && !src_display.isEmpty()) {
        ct::Cloud::Ptr src_markers(new ct::Cloud);
        src_markers->setId(MARKER_SRC_ID);
        src_markers->setPointSize(marker_size);
        for (const auto& p : src_display) {
            ct::PointXYZRGBN pt;
            pt.x = static_cast<float>(p[0]); pt.y = static_cast<float>(p[1]); pt.z = static_cast<float>(p[2]);
            pt.r = 255; pt.g = 60; pt.b = 60;
            src_markers->addPoint(pt);
        }
        m_cloudview->addPointCloud(src_markers);
    }

    // --- Target markers (blue) ---
    m_cloudview->removePointCloud(MARKER_TGT_ID);
    if (check_show_target_->isChecked() && !m_target_points.isEmpty()) {
        ct::Cloud::Ptr tgt_markers(new ct::Cloud);
        tgt_markers->setId(MARKER_TGT_ID);
        tgt_markers->setPointSize(marker_size);
        for (const auto& p : m_target_points) {
            ct::PointXYZRGBN pt;
            pt.x = static_cast<float>(p[0]); pt.y = static_cast<float>(p[1]); pt.z = static_cast<float>(p[2]);
            pt.r = 60; pt.g = 120; pt.b = 255;
            tgt_markers->addPoint(pt);
        }
        m_cloudview->addPointCloud(tgt_markers);
    }
}

void PointPairsAlignment::clearLabels()
{
    int count = qMax(m_source_points.size(), m_target_points.size());
    for (int i = 0; i < count; i++) {
        m_cloudview->remove3DBadge(QString("ppa_badge_s%1").arg(i));
        m_cloudview->remove3DBadge(QString("ppa_badge_t%1").arg(i));
    }
}

void PointPairsAlignment::rebuildLabels()
{
    clearLabels();

    // Source 标签（红色号码牌）
    auto src_disp = displaySourcePoints();
    if (check_show_source_->isChecked()) {
        for (int i = 0; i < src_disp.size(); i++) {
            ct::PointXYZRGBN pos;
            pos.x = static_cast<float>(src_disp[i][0]);
            pos.y = static_cast<float>(src_disp[i][1]);
            pos.z = static_cast<float>(src_disp[i][2]);
            m_cloudview->add3DBadge(pos, QString::number(i + 1),
                                    QString("ppa_badge_s%1").arg(i), 28,
                                    1.0, 0.15, 0.15,   // 红色文字
                                    1.0, 1.0, 1.0,     // 白色背景
                                    0.9);
        }
    }

    // Target 标签（蓝色号码牌）
    if (check_show_target_->isChecked()) {
        for (int i = 0; i < m_target_points.size(); i++) {
            ct::PointXYZRGBN pos;
            pos.x = static_cast<float>(m_target_points[i][0]);
            pos.y = static_cast<float>(m_target_points[i][1]);
            pos.z = static_cast<float>(m_target_points[i][2]);
            m_cloudview->add3DBadge(pos, QString::number(i + 1),
                                    QString("ppa_badge_t%1").arg(i), 28,
                                    0.15, 0.35, 1.0,    // 蓝色文字
                                    1.0, 1.0, 1.0,     // 白色背景
                                    0.9);
        }
    }
}

QVector<Eigen::Vector3d> PointPairsAlignment::displaySourcePoints() const
{
    if (!m_has_preview) return m_source_points;
    QVector<Eigen::Vector3d> result;
    result.reserve(m_source_points.size());
    const Eigen::Matrix4d& T = m_last_result.matrix;
    for (const auto& p : m_source_points) {
        Eigen::Vector4d hp(p[0], p[1], p[2], 1.0);
        Eigen::Vector4d tp = T * hp;
        result.append(tp.head<3>());
    }
    return result;
}

double PointPairsAlignment::calcLabelScale() const
{
    auto clouds = m_cloudtree->getAllClouds();
    int si = cbox_source_->currentIndex();
    int ti = cbox_target_->currentIndex();

    double max_diag = 0.01;
    auto calcDiag = [&](int idx) {
        if (idx < 0 || idx >= clouds.size()) return;
        auto c = clouds[idx];
        if (!c || c->empty()) return;
        auto box = c->box();
        double d = std::sqrt(box.width * box.width + box.height * box.height + box.depth * box.depth);
        if (d > max_diag) max_diag = d;
    };
    calcDiag(si);
    calcDiag(ti);

    return max_diag * 0.012;
}

// ======================== Helpers ========================

void PointPairsAlignment::onToggleSourceVisibility(bool visible)
{
    if (!m_cloudtree || cbox_source_->currentIndex() < 0) return;
    if (m_has_preview) {
        // 预览模式下控制预览云可见性
        m_cloudview->setPointCloudVisibility(PREVIEW_ID, visible);
    } else {
        auto clouds = m_cloudtree->getAllClouds();
        int si = cbox_source_->currentIndex();
        if (si < clouds.size() && clouds[si]) {
            m_cloudview->setPointCloudVisibility(QString::fromStdString(clouds[si]->id()), visible);
        }
    }
    rebuildMarkers();
    rebuildLabels();
    m_cloudview->refresh();
}

void PointPairsAlignment::onToggleTargetVisibility(bool visible)
{
    if (!m_cloudtree || cbox_target_->currentIndex() < 0) return;
    auto clouds = m_cloudtree->getAllClouds();
    int ti = cbox_target_->currentIndex();
    if (ti < clouds.size() && clouds[ti]) {
        m_cloudview->setPointCloudVisibility(QString::fromStdString(clouds[ti]->id()), visible);
    }
    rebuildMarkers();
    rebuildLabels();
    m_cloudview->refresh();
}

void PointPairsAlignment::updateTableErrors(const ct::PointPairErrorResult& result)
{
    int pair_count = std::min({m_source_points.size(), m_target_points.size(),
                               (int)result.deltas.size(), (int)result.errors.size()});

    for (int i = 0; i < pair_count; i++) {
        const auto& d = result.deltas[i];
        double err = result.errors[i];

        if (i < table_source_->rowCount()) {
            table_source_->item(i, 4)->setText(QString::number(d.x(), 'f', 6));
            table_source_->item(i, 5)->setText(QString::number(d.y(), 'f', 6));
            table_source_->item(i, 6)->setText(QString::number(d.z(), 'f', 6));
            auto* err_item = table_source_->item(i, 7);
            err_item->setText(QString::number(err, 'f', 6));
            err_item->setForeground(err < result.rms ? QColor("#27ae60") : QColor("#e74c3c"));
        }

        if (i < table_target_->rowCount()) {
            table_target_->item(i, 4)->setText(QString::number(d.x(), 'f', 6));
            table_target_->item(i, 5)->setText(QString::number(d.y(), 'f', 6));
            table_target_->item(i, 6)->setText(QString::number(d.z(), 'f', 6));
            auto* err_item = table_target_->item(i, 7);
            err_item->setText(QString::number(err, 'f', 6));
            err_item->setForeground(err < result.rms ? QColor("#27ae60") : QColor("#e74c3c"));
        }
    }
}

ct::ConstrainedTransformParams PointPairsAlignment::currentConstraintParams() const
{
    ct::ConstrainedTransformParams params;
    params.rotation = static_cast<ct::RotationConstraint>(cbox_rotation_->currentIndex());
    params.tx_enabled = check_tx_->isChecked();
    params.ty_enabled = check_ty_->isChecked();
    params.tz_enabled = check_tz_->isChecked();
    params.adjust_scale = check_adjust_scale_->isChecked();
    return params;
}

void PointPairsAlignment::updateAllLines()
{
    clearAllLines();
    if (!m_cloudview) return;

    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    if (pair_count == 0) return;

    auto src_disp = displaySourcePoints();

    ct::Cloud::Ptr src_cloud(new ct::Cloud);
    ct::Cloud::Ptr tgt_cloud(new ct::Cloud);

    for (int i = 0; i < pair_count; i++) {
        ct::PointXYZRGBN sp, tp;
        sp.x = static_cast<float>(src_disp[i][0]); sp.y = static_cast<float>(src_disp[i][1]); sp.z = static_cast<float>(src_disp[i][2]);
        tp.x = static_cast<float>(m_target_points[i][0]); tp.y = static_cast<float>(m_target_points[i][1]); tp.z = static_cast<float>(m_target_points[i][2]);
        src_cloud->addPoint(sp);
        tgt_cloud->addPoint(tp);
    }

    pcl::CorrespondencesPtr corrs(new pcl::Correspondences);
    for (int i = 0; i < pair_count; i++)
        corrs->push_back(pcl::Correspondence(i, i, 0));

    m_cloudview->addCorrespondences(src_cloud, tgt_cloud, corrs, LINES_ID, 10);
    m_cloudview->refresh();
}

void PointPairsAlignment::clearAllLines()
{
    if (m_cloudview) {
        m_cloudview->removeShape(LINES_ID);
    }
}

bool PointPairsAlignment::addPointManual(bool is_source, double x, double y, double z)
{
    Eigen::Vector3d pt(x, y, z);
    if (is_source)
        m_source_points.append(pt);
    else
        m_target_points.append(pt);

    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();

    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);

    QString which = is_source ? "Source" : "Target";
    int count = is_source ? m_source_points.size() : m_target_points.size();
    printI(QString("Manual %1 point #%2 added: (%3, %4, %5). Pairs: %6")
           .arg(which).arg(count).arg(x, 0, 'f', 4).arg(y, 0, 'f', 4).arg(z, 0, 'f', 4)
           .arg(pair_count));
    return true;
}

void PointPairsAlignment::onAddSourcePoint()
{
    QDialog dlg(this);
    dlg.setWindowTitle("Add Source Point");
    dlg.setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);

    auto* form = new QFormLayout(&dlg);
    auto* spin_x = new QDoubleSpinBox(&dlg);
    auto* spin_y = new QDoubleSpinBox(&dlg);
    auto* spin_z = new QDoubleSpinBox(&dlg);
    for (auto* spin : {spin_x, spin_y, spin_z}) {
        spin->setRange(-1e9, 1e9);
        spin->setDecimals(4);
        spin->setSingleStep(0.1);
    }
    form->addRow("X:", spin_x);
    form->addRow("Y:", spin_y);
    form->addRow("Z:", spin_z);

    auto* btn_row = new QHBoxLayout();
    auto* btn_ok = new QPushButton("OK", &dlg);
    auto* btn_cancel = new QPushButton("Cancel", &dlg);
    btn_row->addStretch();
    btn_row->addWidget(btn_ok);
    btn_row->addWidget(btn_cancel);
    form->addRow(btn_row);

    QObject::connect(btn_ok, &QPushButton::clicked, &dlg, &QDialog::accept);
    QObject::connect(btn_cancel, &QPushButton::clicked, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted)
        addPointManual(true, spin_x->value(), spin_y->value(), spin_z->value());
}

void PointPairsAlignment::onAddTargetPoint()
{
    QDialog dlg(this);
    dlg.setWindowTitle("Add Target Point");
    dlg.setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);

    auto* form = new QFormLayout(&dlg);
    auto* spin_x = new QDoubleSpinBox(&dlg);
    auto* spin_y = new QDoubleSpinBox(&dlg);
    auto* spin_z = new QDoubleSpinBox(&dlg);
    for (auto* spin : {spin_x, spin_y, spin_z}) {
        spin->setRange(-1e9, 1e9);
        spin->setDecimals(4);
        spin->setSingleStep(0.1);
    }
    form->addRow("X:", spin_x);
    form->addRow("Y:", spin_y);
    form->addRow("Z:", spin_z);

    auto* btn_row = new QHBoxLayout();
    auto* btn_ok = new QPushButton("OK", &dlg);
    auto* btn_cancel = new QPushButton("Cancel", &dlg);
    btn_row->addStretch();
    btn_row->addWidget(btn_ok);
    btn_row->addWidget(btn_cancel);
    form->addRow(btn_row);

    QObject::connect(btn_ok, &QPushButton::clicked, &dlg, &QDialog::accept);
    QObject::connect(btn_cancel, &QPushButton::clicked, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted)
        addPointManual(false, spin_x->value(), spin_y->value(), spin_z->value());
}

void PointPairsAlignment::refreshCloudList()
{
    cbox_source_->clear();
    cbox_target_->clear();
    if (!m_cloudtree) return;

    auto clouds = m_cloudtree->getAllClouds();
    for (const auto& c : clouds) {
        QString name = QString::fromStdString(c->id());
        cbox_source_->addItem(name);
        cbox_target_->addItem(name);
    }
    if (cbox_source_->count() >= 2) {
        cbox_source_->setCurrentIndex(0);
        cbox_target_->setCurrentIndex(1);
    }
}

ct::PickResult PointPairsAlignment::doPick(const ct::PointXY& pt)
{
    return m_cloudview->singlePick(pt, "");
}

void PointPairsAlignment::hideNonSelectedClouds()
{
    if (!m_cloudtree) return;
    QString source_id = cbox_source_->currentText();
    QString target_id = cbox_target_->currentText();

    auto clouds = m_cloudtree->getAllClouds();
    for (const auto& c : clouds) {
        QString id = QString::fromStdString(c->id());
        if (id != source_id && id != target_id) {
            m_cloudview->setPointCloudVisibility(id, false);
        }
    }
    m_cloudview->refresh();
}

void PointPairsAlignment::restoreAllCloudVisibility()
{
    if (!m_cloudtree) return;
    auto clouds = m_cloudtree->getAllClouds();
    for (const auto& c : clouds) {
        m_cloudview->setPointCloudVisibility(QString::fromStdString(c->id()), true);
    }
    m_cloudview->refresh();
}

// ======================== New Slot Functions ========================

void PointPairsAlignment::onSwapSourceTarget()
{
    if (m_is_picking) return;

    int si = cbox_source_->currentIndex();
    int ti = cbox_target_->currentIndex();
    cbox_source_->setCurrentIndex(ti);
    cbox_target_->setCurrentIndex(si);

    std::swap(m_source_points, m_target_points);
    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();

    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);
    printI("Source and target swapped.");
}

void PointPairsAlignment::onClearSourcePoints()
{
    if (m_source_points.isEmpty()) return;
    if (m_has_preview) onCancel();
    m_source_points.clear();
    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();
    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    label_rms_->setText("RMS: --");
    printI("Source points cleared.");
}

void PointPairsAlignment::onClearTargetPoints()
{
    if (m_target_points.isEmpty()) return;
    if (m_has_preview) onCancel();
    m_target_points.clear();
    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();
    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);
    btn_apply_->setEnabled(false);
    btn_cancel_->setEnabled(false);
    label_rms_->setText("RMS: --");
    printI("Target points cleared.");
}

void PointPairsAlignment::onImportTargetPoints()
{
    QString filepath = QFileDialog::getOpenFileName(this, "Import Target Points",
        QString(), "Text Files (*.txt *.csv);;All Files (*)");
    if (filepath.isEmpty()) return;

    QFile file(filepath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        printE("Failed to open file: " + filepath);
        return;
    }

    if (m_has_preview) onCancel();

    QTextStream in(&file);
    int count = 0;
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        QStringList parts = line.split(QRegExp("[,;\\t\\s]+"), Qt::SkipEmptyParts);
        if (parts.size() < 3) continue;

        bool ok;
        double x = parts[0].toDouble(&ok); if (!ok) continue;
        double y = parts[1].toDouble(&ok); if (!ok) continue;
        double z = parts[2].toDouble(&ok); if (!ok) continue;

        addPointManual(false, x, y, z);
        count++;
    }

    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();

    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);
    printI(QString("Imported %1 target points from %2.").arg(count).arg(filepath));
}

void PointPairsAlignment::onImportSourcePoints()
{
    QString filepath = QFileDialog::getOpenFileName(this, "Import Source Points",
        QString(), "Text Files (*.txt *.csv);;All Files (*)");
    if (filepath.isEmpty()) return;

    QFile file(filepath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        printE("Failed to open file: " + filepath);
        return;
    }

    if (m_has_preview) onCancel();

    QTextStream in(&file);
    int count = 0;
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        QStringList parts = line.split(QRegExp("[,;\\t\\s]+"), Qt::SkipEmptyParts);
        if (parts.size() < 3) continue;

        bool ok;
        double x = parts[0].toDouble(&ok); if (!ok) continue;
        double y = parts[1].toDouble(&ok); if (!ok) continue;
        double z = parts[2].toDouble(&ok); if (!ok) continue;

        addPointManual(true, x, y, z);
        count++;
    }

    rebuildMarkers();
    rebuildLabels();
    updateAllLines();
    updateTables();

    int pair_count = std::min(m_source_points.size(), m_target_points.size());
    btn_align_->setEnabled(pair_count >= 3);
    printI(QString("Imported %1 source points from %2.").arg(count).arg(filepath));
}

void PointPairsAlignment::onRmsFilterToggled(bool checked)
{
    spin_rms_threshold_->setEnabled(checked);
}
