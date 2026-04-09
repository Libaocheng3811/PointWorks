#include "align_by_centers.h"

#include "viz/cloudview.h"
#include "base/cloudtree.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QtConcurrent/QtConcurrent>

#include <pcl/common/transforms.h>

// ======================== Constructor ========================

AlignByCentersDialog::AlignByCentersDialog(QWidget* parent)
    : ct::CustomDialog(parent)
{
    setupUi();

    this->setWindowTitle("Align by Centers");
    this->setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);
    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->resize(320, 140);

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

    auto* btn_row = new QHBoxLayout();
    btn_row->addStretch();
    btn_align_ = new QPushButton("Align", this);
    auto* btn_cancel = new QPushButton("Cancel", this);
    btn_row->addWidget(btn_align_);
    btn_row->addWidget(btn_cancel);
    main_layout->addLayout(btn_row);

    connect(btn_align_, &QPushButton::clicked, this, &AlignByCentersDialog::onAlign);
    connect(btn_cancel, &QPushButton::clicked, this, &AlignByCentersDialog::close);
}

// ======================== Init / Reset ========================

void AlignByCentersDialog::init()
{
    refreshCloudList();
}

void AlignByCentersDialog::reset()
{
    refreshCloudList();
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

    // 禁用按钮
    btn_align_->setEnabled(false);
    m_canceled.store(false);
    m_source_id = QString::fromStdString(source->id());

    // 计算平移矩阵（很快，可在主线程）
    Eigen::Vector3f src_center = source->center();
    Eigen::Vector3f tgt_center = target->center();
    Eigen::Vector3f offset = tgt_center - src_center;

    m_matrix = Eigen::Matrix4f::Identity();
    m_matrix(0, 3) = offset.x();
    m_matrix(1, 3) = offset.y();
    m_matrix(2, 3) = offset.z();

    // 异步执行点云变换（耗时与点数成正比）
    auto future = QtConcurrent::run([=]() -> ct::Cloud::Ptr {
        if (m_canceled.load()) return nullptr;
        auto pcl_cloud = source->toPCL_XYZRGBN();
        if (m_canceled.load()) return nullptr;
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, m_matrix);
        auto transformed = ct::Cloud::fromPCL_XYZRGBN(*pcl_cloud);
        return transformed;
    });

    auto* watcher = new QFutureWatcher<ct::Cloud::Ptr>(this);
    watcher->setFuture(future);
    connect(watcher, &QFutureWatcher<ct::Cloud::Ptr>::finished, this, &AlignByCentersDialog::onAlignFinished);
}

void AlignByCentersDialog::onAlignFinished()
{
    auto* watcher = dynamic_cast<QFutureWatcher<ct::Cloud::Ptr>*>(sender());
    auto result = watcher->result();
    watcher->deleteLater();

    btn_align_->setEnabled(true);

    if (m_canceled.load() || !result) {
        printI("Align by centers canceled.");
        return;
    }

    auto clouds = m_cloudtree->getAllClouds();
    ct::Cloud::Ptr source;
    for (const auto& c : clouds) {
        if (QString::fromStdString(c->id()) == m_source_id) { source = c; break; }
    }
    if (!source) return;

    result->setId(source->id());
    m_cloudtree->updateCloud(source, result);
    m_cloudview->invalidateCloudRender(m_source_id);
    m_cloudview->refresh();

    printI(QString("Aligned [%1]. Offset: (%2, %3, %4)")
           .arg(m_source_id)
           .arg(m_matrix(0, 3), 0, 'f', 3)
           .arg(m_matrix(1, 3), 0, 'f', 3)
           .arg(m_matrix(2, 3), 0, 'f', 3));

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
