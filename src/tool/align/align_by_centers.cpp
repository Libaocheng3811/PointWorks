#include "align_by_centers.h"

#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "ui/dialog/processingdialog.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QtConcurrent/QtConcurrent>

#include <pcl/common/transforms.h>

// ======================== Constructor ========================

AlignByCentersDialog::AlignByCentersDialog(QWidget* parent)
    : ct::CustomDialog(parent), m_has_preview(false)
{
    setupUi();

    this->setWindowTitle("Align by Centers");
    this->setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(320, 160);

    QTimer::singleShot(0, this, [this](){ this->adjustSize(); });
}

AlignByCentersDialog::~AlignByCentersDialog() = default;

// ======================== setupUi ========================

void AlignByCentersDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(12, 12, 12, 12);
    main_layout->setSpacing(8);

    auto* src_row = new QHBoxLayout();
    src_row->addWidget(new QLabel("Source:", this));
    cbox_source_ = new QComboBox(this);
    cbox_source_->setMinimumWidth(160);
    src_row->addWidget(cbox_source_, 1);
    main_layout->addLayout(src_row);

    auto* tgt_row = new QHBoxLayout();
    tgt_row->addWidget(new QLabel("Target:", this));
    cbox_target_ = new QComboBox(this);
    cbox_target_->setMinimumWidth(160);
    tgt_row->addWidget(cbox_target_, 1);
    main_layout->addLayout(tgt_row);

    auto* info = new QLabel("Click Align to preview, then Apply or Cancel.", this);
    info->setWordWrap(true);
    info->setStyleSheet("color: #888; font-size: 11px;");
    main_layout->addWidget(info);

    auto* btn_row = new QHBoxLayout();
    btn_row->addStretch();
    btn_align_ = new QPushButton("Align", this);
    btn_apply_ = new QPushButton("Apply", this);
    btn_apply_->setEnabled(false);
    auto* btn_cancel = new QPushButton("Cancel", this);
    btn_row->addWidget(btn_align_);
    btn_row->addWidget(btn_apply_);
    btn_row->addWidget(btn_cancel);
    main_layout->addLayout(btn_row);

    connect(btn_align_, &QPushButton::clicked, this, &AlignByCentersDialog::onAlign);
    connect(btn_apply_, &QPushButton::clicked, this, &AlignByCentersDialog::onApply);
    connect(btn_cancel, &QPushButton::clicked, this, &AlignByCentersDialog::onCancel);
}

// ======================== Init / Reset ========================

void AlignByCentersDialog::init()
{
    refreshCloudList();
}

void AlignByCentersDialog::reset()
{
    m_aligned_cloud.reset();
    m_has_preview = false;
    m_matrix = Eigen::Matrix4f::Identity();
    m_source_id.clear();
    m_last_align_snapshot.clear();
    refreshCloudList();
}

void AlignByCentersDialog::deinit()
{
    m_progress->closeProgress();
    // 防御性清理预览云
    m_cloudview->removePointCloud(PREVIEW_ID);
    if (!m_source_id.isEmpty())
        m_cloudview->setPointCloudVisibility(m_source_id, true);
    reset();
}

// ======================== Slots ========================

void AlignByCentersDialog::onAlign()
{
    if (cbox_source_->currentIndex() < 0 || cbox_target_->currentIndex() < 0) {
        printW("Please select source and target clouds.");
        return;
    }
    if (cbox_source_->currentIndex() == cbox_target_->currentIndex()) {
        printW("Source and target must be different clouds.");
        return;
    }

    auto clouds = m_cloudtree->getAllClouds();
    int si = cbox_source_->currentIndex();
    int ti = cbox_target_->currentIndex();
    if (si >= (int)clouds.size() || ti >= (int)clouds.size()) return;

    auto source = clouds[si];
    auto target = clouds[ti];
    if (!source || !target || source->empty() || target->empty()) {
        printE("Source or target cloud is empty.");
        return;
    }

    // 参数快照对比：非首次且参数一致则跳过
    {
        ct::ParamSnapshot snap;
        snap.set("source_id", cbox_source_->currentText());
        snap.set("target_id", cbox_target_->currentText());
        if (snap == m_last_align_snapshot && m_has_preview) {
            printI("Parameters unchanged, skipping recomputation.");
            return;
        }
        m_last_align_snapshot = snap;
    }

    btn_align_->setEnabled(false);
    m_canceled.store(false);
    m_source_id = QString::fromStdString(source->id());

    // 显示模态进度条
    m_progress->showProgress("Aligning...");
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [this]() {
                    m_canceled.store(true);
                    if (m_progress->dialog())
                        m_progress->setMessage("Canceling...");
                });
    }

    // 计算平移矩阵
    Eigen::Vector3f src_center = source->center();
    Eigen::Vector3f tgt_center = target->center();
    Eigen::Vector3f offset = tgt_center - src_center;

    m_matrix = Eigen::Matrix4f::Identity();
    m_matrix(0, 3) = offset.x();
    m_matrix(1, 3) = offset.y();
    m_matrix(2, 3) = offset.z();

    // 深拷贝后变换，避免污染源点云缓存
    auto future = QtConcurrent::run([=]() -> ct::Cloud::Ptr {
        if (m_canceled.load()) return nullptr;
        auto pcl_cloud = source->toPCL_XYZRGBN();
        auto pcl_copy = std::make_shared<pcl::PointCloud<ct::PointXYZRGBN>>(*pcl_cloud);
        if (m_canceled.load()) return nullptr;
        pcl::transformPointCloud(*pcl_copy, *pcl_copy, m_matrix);
        auto transformed = ct::Cloud::fromPCL_XYZRGBN(*pcl_copy, source->getGlobalShift());
        return transformed;
    });

    auto* watcher = new QFutureWatcher<ct::Cloud::Ptr>(this);
    watcher->setFuture(future);
    connect(watcher, &QFutureWatcher<ct::Cloud::Ptr>::finished, this, &AlignByCentersDialog::onAlignFinished);
}

void AlignByCentersDialog::onAlignFinished()
{
    auto* watcher = dynamic_cast<QFutureWatcher<ct::Cloud::Ptr>*>(sender());
    m_progress->closeProgress();
    auto result = watcher->result();
    watcher->deleteLater();

    btn_align_->setEnabled(true);

    if (m_canceled.load() || !result) {
        printI("Align by centers canceled.");
        return;
    }

    m_aligned_cloud = result;

    // 预览：只在视图中显示，不插入文件树
    m_aligned_cloud->setId(PREVIEW_ID);
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, false);
    m_cloudview->addPointCloud(m_aligned_cloud);
    m_cloudview->refresh();

    m_has_preview = true;
    btn_apply_->setEnabled(true);

    printI(QString("Align by centers preview. Offset: (%1, %2, %3)")
           .arg(m_matrix(0, 3), 0, 'f', 3)
           .arg(m_matrix(1, 3), 0, 'f', 3)
           .arg(m_matrix(2, 3), 0, 'f', 3));
}

void AlignByCentersDialog::onApply()
{
    if (!m_aligned_cloud || m_source_id.isEmpty()) return;

    auto clouds = m_cloudtree->getAllClouds();
    ct::Cloud::Ptr source;
    for (const auto& c : clouds) {
        if (QString::fromStdString(c->id()) == m_source_id) { source = c; break; }
    }
    if (!source) return;

    // 用变换矩阵变换原始源点云
    auto pcl_src = source->toPCL_XYZRGBN();
    auto pcl_copy = std::make_shared<pcl::PointCloud<ct::PointXYZRGBN>>(*pcl_src);
    pcl::transformPointCloud(*pcl_copy, *pcl_copy, m_matrix);
    auto transformed = ct::Cloud::fromPCL_XYZRGBN(*pcl_copy, source->getGlobalShift());
    transformed->setId(m_source_id.toStdString());

    m_cloudtree->updateCloud(source, transformed);

    // 移除预览云，恢复源点云可见
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->invalidateCloudRender(m_source_id);
    m_cloudview->refresh();

    printI(QString("Align by centers applied to [%1].")
           .arg(m_source_id));

    m_aligned_cloud.reset();
    m_has_preview = false;
    this->close();
}

void AlignByCentersDialog::onCancel()
{
    m_progress->closeProgress();
    m_canceled.store(true);

    // 移除预览云，恢复源点云可见（源点云从未被修改）
    m_cloudview->removePointCloud(PREVIEW_ID);
    m_cloudview->setPointCloudVisibility(m_source_id, true);
    m_cloudview->refresh();

    m_aligned_cloud.reset();
    m_has_preview = false;

    printI("Align by centers canceled.");
    this->close();
}

// ======================== Helpers ========================

void AlignByCentersDialog::refreshCloudList()
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
