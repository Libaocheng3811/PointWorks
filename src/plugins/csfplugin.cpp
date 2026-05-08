//
// Created by LBC on 2026/1/4.
//

// You may need to build the project (run Qt uic code generator) to get "ui_csfplugin.h" resolved

#include "csfplugin.h"
#include "ui_csfplugin.h"

CSFPlugin::CSFPlugin(QWidget *parent) :
        pw::CustomDialog(parent), ui(new Ui::CSFPlugin) {
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

    bool smooth = ui->m_chkSlopeProcessing->isChecked();
    double res = ui->m_dsResolution->value();
    double thresh = ui->m_dsThreshold->value();
    int iter = ui->m_spIterations->value();

    int rigidness = 2;
    if (ui->m_rbFlat->isChecked()) rigidness = 3;
    else if (ui->m_rbRelief->isChecked()) rigidness = 2;
    else rigidness = 1;

    this->hide();
    QCoreApplication::processEvents();

    auto cloud = m_cloud;
    auto* cloudtree = m_cloudtree;

    m_progress->runAsync<pw::CSFResult>(
        "Running Cloth Simulation Filter...",
        [cloud, smooth, thresh, res, rigidness, iter](
            std::atomic<bool>& cancel, pw::ProgressCallback progress) {
            return pw::CSFFilter::apply(cloud, smooth, 0.65f, thresh, res,
                                         rigidness, iter, &cancel, progress);
        },
        [=](const pw::CSFResult& result) {
            printI(QString("CSF Finished in %1 s").arg(result.time_ms));

            auto ground_cloud = result.ground_cloud;
            auto off_ground_cloud = result.off_ground_cloud;

            ground_cloud->setId(cloud->id() + "_ground");
            off_ground_cloud->setId(cloud->id() + "_off_ground");

            if (!cloud->hasColors()){
                ground_cloud->setCloudColor(pw::ColorRGB{0, 255, 0});
                off_ground_cloud->setCloudColor(pw::ColorRGB{255, 0, 0});
            }

            if (ground_cloud && !ground_cloud->empty()) ground_cloud->makeAdaptive();
            if (off_ground_cloud && !off_ground_cloud->empty()) off_ground_cloud->makeAdaptive();

            std::vector<pw::Cloud::Ptr> results;
            results.push_back(ground_cloud);
            results.push_back(off_ground_cloud);

            QString groupName = QString::fromStdString(cloud->id()) + "_CSF";
            cloudtree->addResultGroup(cloud, results, groupName);

            QMetaObject::invokeMethod(qApp, [this]() { this->accept(); }, Qt::QueuedConnection);
        }
    );
}

void CSFPlugin::onCancel() {
    this->close();
}
