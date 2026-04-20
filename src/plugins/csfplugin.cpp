//
// Created by LBC on 2026/1/4.
//

// You may need to build the project (run Qt uic code generator) to get "ui_csfplugin.h" resolved

#include "csfplugin.h"
#include "ui_csfplugin.h"

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>

CSFPlugin::CSFPlugin(QWidget *parent) :
        ct::CustomDialog(parent), ui(new Ui::CSFPlugin) {
    ui->setupUi(this);

    // ui按钮
    connect(ui->btnOk, &QPushButton::clicked, this, &CSFPlugin::onApply);
    connect(ui->btnCancel, &QPushButton::clicked, this, &CSFPlugin::onCancel);
}

CSFPlugin::~CSFPlugin() {
    delete ui;
}

void CSFPlugin::init(){
    auto selection = m_cloudtree->getSelectedClouds();
    if (selection.empty()){
        printW("Please select at least one cloud.");
        ui->btnOk->setEnabled(false);
        return;
    }

    m_cloud = selection.front();
    ui->btnOk->setEnabled(true);
}

void CSFPlugin::onApply() {
    if (!m_cloud) return;

    //获取参数
    bool smooth = ui->m_chkSlopeProcessing->isChecked();
    double res = ui->m_dsResolution->value();
    double thresh = ui->m_dsThreshold->value();
    int iter = ui->m_spIterations->value();

    int rigidness = 2; // 默认为relief
    if (ui->m_rbFlat->isChecked()) rigidness = 3;
    else if (ui->m_rbRelief->isChecked()) rigidness = 2;
    else rigidness = 1;

    this->hide();
    QCoreApplication::processEvents();

    m_progress->showProgress("Running Cloth Simulation Filter...");

    // 通过 cancelRequested 信号设置取消标志
    auto* cancel = new std::atomic<bool>(false);
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [cancel]() { *cancel = true; });
    }

    // 进度回调：跨线程安全地更新进度条
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    auto cloud = m_cloud;
    auto future = QtConcurrent::run([cloud, smooth, thresh, res, rigidness, iter, cancel, on_progress]() {
        return ct::CSFFilter::apply(cloud, smooth, 0.65f, thresh, res, rigidness, iter, cancel, on_progress);
    });

    auto* watcher = new QFutureWatcher<ct::CSFResult>(this);
    connect(watcher, &QFutureWatcher<ct::CSFResult>::finished, this, [=]() {
        auto result = watcher->result();
        m_progress->closeProgress();
        delete cancel;
        printI(QString("CSF Finished in %1 s").arg(result.time_ms));

        auto ground_cloud = result.ground_cloud;
        auto off_ground_cloud = result.off_ground_cloud;

        ground_cloud->setId(m_cloud->id() + "_ground");
        off_ground_cloud->setId(m_cloud->id() + "_off_ground");

        if (!m_cloud->hasColors()){
            // 如果没有RGB信息，手动赋色
            ground_cloud->setCloudColor(ct::ColorRGB{0, 255, 0}); // Green
            off_ground_cloud->setCloudColor(ct::ColorRGB{255, 0, 0}); // Red
        }

        // 处理地面点云
        if (ground_cloud && !ground_cloud->empty()) {
            ground_cloud->makeAdaptive();
        }

        // 处理非地面点云
        if (off_ground_cloud && !off_ground_cloud->empty()) {
            off_ground_cloud->makeAdaptive();
        }

        std::vector<ct::Cloud::Ptr> results;
        results.push_back(ground_cloud);
        results.push_back(off_ground_cloud);

        QString groupName = QString::fromStdString(m_cloud->id()) + "_CSF";
        m_cloudtree->addResultGroup(m_cloud, results, groupName);

        this->accept();
        watcher->deleteLater();
    });
    watcher->setFuture(future);
}

void CSFPlugin::onCancel() {
    this->close();
}
