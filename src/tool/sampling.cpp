//
// Created by LBC on 2026/3/24.
//

#include "sampling.h"
#include "base/cloudtree.h"
#include "ui_sampling.h"

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>

Sampling::Sampling(QWidget* parent)
        : CustomDialog(parent), ui(new Ui::Sampling)
{
    ui->setupUi(this);

    // 连接按钮
    connect(ui->btn_ok, &QPushButton::clicked, this, &Sampling::onOkClicked);
    connect(ui->btn_cancel, &QPushButton::clicked, this, &Sampling::onCancelClicked);

    // 设置默认选择
    ui->cbox_method->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

Sampling::~Sampling()
{
    delete ui;
}

void Sampling::init()
{
    // 获取当前选中的点云
    if (!m_cloudtree) return;

    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud first!");
        reject();
        return;
    }

    // 只支持单个点云采样
    if (selected_clouds.size() > 1)
    {
        printW("Sampling only supports single cloud. Will process the first selected cloud.");
    }

    m_current_cloud = selected_clouds[0];
}

void Sampling::onOkClicked()
{
    if (!m_current_cloud || !m_cloudtree || !m_cloudview)
    {
        reject();
        return;
    }

    m_progress->showProgress("Sampling PointCloud...");

    // 取消标志
    auto* cancel = new std::atomic<bool>(false);
    m_cancel = false;
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [cancel]() { *cancel = true; }, Qt::UniqueConnection);
    }

    // 进度回调
    auto on_progress = [this](int pct) {
        if (m_progress->dialog()) {
            QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                      Qt::QueuedConnection, Q_ARG(int, pct));
        }
    };

    auto cloud = m_current_cloud;

    // 根据选择的方法执行采样
    QFuture<ct::FilterResult> future;
    switch (ui->cbox_method->currentIndex())
    {
        case METHOD_DOWNSAMPLING:
            future = QtConcurrent::run([cloud, cancel, on_progress, radius = ui->dspin_radius1->value()]() {
                return ct::Filters::DownSampling(cloud, radius, false, cancel, on_progress);
            });
            break;

        case METHOD_UNIFORMSAMPLING:
            future = QtConcurrent::run([cloud, cancel, on_progress, radius = ui->dspin_radius2->value()]() {
                return ct::Filters::UniformSampling(cloud, radius, false, cancel, on_progress);
            });
            break;

        case METHOD_RANDOMSAMPLING:
            future = QtConcurrent::run([cloud, cancel, on_progress, sample = ui->spin_sample1->value(), seed = ui->spin_seed1->value()]() {
                return ct::Filters::RandomSampling(cloud, sample, seed, false, cancel, on_progress);
            });
            break;

        case METHOD_RESAMPLING:
            future = QtConcurrent::run([cloud, cancel, on_progress, radius = ui->dspin_radius3->value(), order = ui->spin_order->value()]() {
                return ct::Filters::ReSampling(cloud, radius, order, false, cancel, on_progress);
            });
            break;

        case METHOD_SAMPLINGSURFACENORMAL:
            future = QtConcurrent::run([cloud, cancel, on_progress, sample = ui->spin_sample2->value(), seed = ui->spin_seed2->value(), ratio = ui->dspin_ratio->value()]() {
                return ct::Filters::SamplingSurfaceNormal(cloud, sample, seed, ratio, false, cancel, on_progress);
            });
            break;

        case METHOD_NORMALSPACESAMPLING:
            future = QtConcurrent::run([cloud, cancel, on_progress, sample = ui->spin_sample3->value(), seed = ui->spin_seed3->value(), bin = ui->spin_bin->value()]() {
                return ct::Filters::NormalSpaceSampling(cloud, sample, seed, bin, false, cancel, on_progress);
            });
            break;
    }

    auto* watcher = new QFutureWatcher<ct::FilterResult>(this);
    connect(watcher, &QFutureWatcher<ct::FilterResult>::finished, this, [=]() {
        auto result = watcher->result();
        m_progress->closeProgress();
        delete cancel;
        handleSamplingResult(result);
        watcher->deleteLater();
    });
    watcher->setFuture(future);
}

void Sampling::onCancelClicked()
{
    reject();
}

void Sampling::handleSamplingResult(const ct::FilterResult& result)
{
    auto cloud = result.result_cloud;

    if (!cloud || !m_current_cloud || !m_cloudtree || !m_cloudview)
    {
        m_progress->closeProgress();
        reject();
        return;
    }

    // 打印完成信息
    printI(QString("Sampling completed in %1 ms. Original: %2 points -> Sampled: %3 points")
          .arg(result.time_ms)
          .arg(m_current_cloud->size())
          .arg(cloud->size()));

    // 设置新点云的名称：原名称 + "-sampling"
    // 策略一：采样结果作为兄弟节点挂载
    m_cloudtree->addSiblingCloud(m_current_cloud, cloud, "-sampling");

    // 在视图中显示新点云（保留原始颜色）
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudSize(QString::fromStdString(cloud->id()), cloud->pointSize() + 2);

    // 取消选中源点云，选中新生成的采样点云
    m_cloudtree->setCloudChecked(m_current_cloud, false);
    m_cloudtree->setCloudChecked(cloud, true);

    // 关闭进度条
    m_progress->closeProgress();

    // 关闭对话框
    accept();
}
