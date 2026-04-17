//
// Created by LBC on 2026/1/6.
//

// You may need to build the project (run Qt uic code generator) to get "ui_vegplugin.h" resolved

#include "vegplugin.h"
#include "ui_vegplugin.h"

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>


VegPlugin::VegPlugin(QWidget *parent) :
        ct::CustomDialog(parent), ui(new Ui::VegPlugin) {
    ui->setupUi(this);

    connect(ui->m_comboType, SIGNAL(currentIndexChanged(int)), this, SLOT(onIndexChanged(int)));
    connect(ui->m_btnOk, &QPushButton::clicked, this, &VegPlugin::onApply);
    connect(ui->m_btnCancel, &QPushButton::clicked, this, &QDialog::reject);

    onIndexChanged(0);
}

VegPlugin::~VegPlugin() {
    delete ui;
}

void VegPlugin::init() {
    auto selection = m_cloudtree->getSelectedClouds();
    if (selection.empty() || !selection.front()->hasColors()){
        printW("Please select a cloud with RGB data first!");
        ui->m_btnOk->setEnabled(false);
        return;
    }
    m_cloud = selection.front();
    ui->m_btnOk->setEnabled(true);
}

void VegPlugin::onIndexChanged(int index) {
    QString info;
    switch (index) {
        case 0:
            info = "<b>ExG-ExR（过绿过红差异指数）</b><br>"
                   "<b>公式：</b>3g - 2.4r - b<br>"
                   "<b>值阈：</b>[-0.5, 0.5]<br>"
                   "<b>说明：</b>经典的 RGB 植被指数，通过增强绿色分量并抑制红、蓝分量来突出植被特征，"
                   "在自然场景中能够有效区分植被与土壤，鲁棒性较好，但在城市场景中可能对红色屋顶产生误判。";
            ui->m_dsThreshold->setEnabled(true);
            ui->m_dsThreshold->setRange(-0.5, 0.5);
            ui->m_dsThreshold->setValue(-0.05);
            break;
        case 1:
            info = "<b>ExG (过绿指数):</b><br>"
                   "<b>公式: </b>2g - r - b<br>"
                   "<b>值阈：</b>[-0.5, 0.5]<br>"
                   "<b>说明: </b>经典的植被提取算法，能有效区分植物与土壤。";
            ui->m_dsThreshold->setEnabled(true);
            ui->m_dsThreshold->setRange(-0.5, 0.5);
            ui->m_dsThreshold->setValue(0.0);
            break;
        case 2:
            info = "<b>NGRDI (归一化绿红差异指数):</b><br>"
                   "<b>公式: </b>(g-r)/(g+r)<br>"
                   "<b>值阈：</b>[-0.2, 0.4]<br>"
                   "<b>说明: </b>类似于卫星遥感中的 NDVI，但用 Green 替代了 NIR（近红外）";
            ui->m_dsThreshold->setEnabled(true);
            ui->m_dsThreshold->setRange(-0.2, 0.4);
            ui->m_dsThreshold->setValue(0.0);
            break;
        case 3:
            info = "<b>CIVE(植被颜色提取指数): </b><br>"
                   "<b>公式: </b>0.441r-0.811g+0.385b+18.78745<br>"
                   "<b>值阈：</b>自适应阈值,多波段线性加权<br>"
                   "<b>说明：</b>综合利用RGB分量，对不同光照条件有较好鲁棒性。";
            ui->m_dsThreshold->setEnabled(false);
            break;
        default:
            break;
    }
    ui->m_textBrowser->setHtml(info);
}

void VegPlugin::onApply() {
    this->hide();

    m_cloudtree->showProgress("Vegetation Filter...");

    // 通过 cancelRequested 信号设置取消标志
    auto* cancel = new std::atomic<bool>(false);
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [cancel]() { *cancel = true; });
    }

    // 进度回调：跨线程安全地更新进度条
    auto on_progress = [this](int pct) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, pct));
    };

    int type = ui->m_comboType->currentIndex();
    double threshold = ui->m_dsThreshold->value();
    auto cloud = m_cloud;

    auto future = QtConcurrent::run([cloud, type, threshold, cancel, on_progress]() {
        return ct::VegetationFilter::apply(cloud, type, threshold, cancel, on_progress);
    });

    auto* watcher = new QFutureWatcher<ct::VegResult>(this);
    connect(watcher, &QFutureWatcher<ct::VegResult>::finished, this, [=]() {
        auto result = watcher->result();
        m_cloudtree->closeProgress();
        delete cancel;
        printI(QString("Vegetation Filter Finished in %1 s").arg(result.time_ms));

        auto veg_cloud = result.veg_cloud;
        auto non_veg_cloud = result.non_veg_cloud;

        if (veg_cloud) {
            veg_cloud->setId(m_cloud->id() + "_vegetation");
            if (!veg_cloud->empty()) veg_cloud->makeAdaptive();
        }

        if (non_veg_cloud) {
            non_veg_cloud->setId(m_cloud->id() + "_non_vegetation");
            if (!non_veg_cloud->empty()) non_veg_cloud->makeAdaptive();
        }

        std::vector<ct::Cloud::Ptr> results;
        results.push_back(veg_cloud);
        results.push_back(non_veg_cloud);

        QString groupName = QString::fromStdString(m_cloud->id()) + "_Vegetation";
        m_cloudtree->addResultGroup(m_cloud, results, groupName);

        this->accept();
        watcher->deleteLater();
    });
    watcher->setFuture(future);
}
