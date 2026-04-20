//
// Created by LBC on 2026/1/18.
//

// You may need to build the project (run Qt uic code generator) to get "ui_changedetectdialog.h" resolved

#include "changedetectplugin.h"
#include "ui_changedetectplugin.h"

#include <cmath>

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>

// 辅助函数，配置点云
void setupResultCloud(ct::Cloud::Ptr cloud, const QString& idSuffix){
    if (!cloud || cloud->empty()) return;

    // 基于标量场进行伪彩色渲染 (Color Ramp)
    cloud->updateColorByField("C2C_Distance");

    // 设置其他属性
    cloud->setId(cloud->id() + idSuffix.toStdString());
    cloud->setHasColors(true); // 确保渲染器使用 RGB
}

ChangeDetectPlugin::ChangeDetectPlugin(QWidget *parent) :
        ct::CustomDialog(parent), ui(new Ui::ChangeDetectPlugin) {
    ui->setupUi(this);

    connect(ui->btn_ok, &QPushButton::clicked, this, &ChangeDetectPlugin::onApply);
    connect(ui->btn_cancel, &QPushButton::clicked, this, &ChangeDetectPlugin::onCancel);
}

ChangeDetectPlugin::~ChangeDetectPlugin() {
    delete ui;
}

void ChangeDetectPlugin::init() {
    ui->combo_reference->clear();
    ui->combo_compare->clear();

    // 获取可用点云
    std::vector<ct::Cloud::Ptr> allClouds = m_cloudtree->getAllClouds();
    if (allClouds.empty()){
        printW(QString("No clouds available"));
        ui->btn_ok->setEnabled(false);
        return;
    }

    for (const auto& cloud : allClouds){
        QString cloudId = QString::fromStdString(cloud->id());
        ui->combo_reference->addItem(cloudId, QVariant::fromValue(cloud));
        ui->combo_compare->addItem(cloudId, QVariant::fromValue(cloud));
    }

    // 智能默认选项
    std::vector<ct::Cloud::Ptr> selectedClouds = m_cloudtree->getSelectedClouds();
    if (selectedClouds.size() >= 2){
        // 认为第一个选择的是参考点云，第二个选择的是待比较点云
        int idxRef = ui->combo_reference->findText(QString::fromStdString(selectedClouds[0]->id()));
        int idxComp = ui->combo_compare->findText(QString::fromStdString(selectedClouds[1]->id()));

        if (idxRef >= 0) ui->combo_reference->setCurrentIndex(idxRef);
        if (idxComp >= 0) ui->combo_compare->setCurrentIndex(idxComp);
    }
    else if (selectedClouds.size() == 1 && allClouds.size() >= 2){
        // 如果仅选择一个点云，则默认选择该点云为参考点云
        int idxRef = ui->combo_reference->findText(QString::fromStdString(selectedClouds[0]->id()));
        ui->combo_reference->setCurrentIndex(idxRef);

        // 选择剩余点云为待比较点云
        for (int i = 0; i < ui->combo_compare->count(); ++i){
            if (i != idxRef){
                ui->combo_compare->setCurrentIndex(i);
                break;
            }
        }
    }

    ui->combo_method->clear();
    ui->combo_method->addItem("C2C - Nearest Neighbor", ct::DistanceParams::C2C_NEAREST);
    ui->combo_method->addItem("C2C - K-Mean", ct::DistanceParams::C2C_KNN_MEAN);
    ui->combo_method->addItem("C2C - Radius Mean", ct::DistanceParams::C2C_RADIUS_MEAN);

    // 连接信号
    connect(ui->combo_method, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ChangeDetectPlugin::onMethodChanged);
    // 默认选中第0页
    ui->stackedWidget->setCurrentIndex(0);
    ui->btn_ok->setEnabled(true);
}

void ChangeDetectPlugin::onApply() {
    m_refCloud = ui->combo_reference->currentData().value<ct::Cloud::Ptr>();
    m_compCloud = ui->combo_compare->currentData().value<ct::Cloud::Ptr>();

    if (!m_refCloud || !m_compCloud){
        printE(QString("Invalid clouds selected"));
        return;
    }

    if (m_refCloud == m_compCloud){
        printW(QString("Reference cloud and compare cloud are the same"));
        return;
    }
    ct::DistanceParams params;
    params.method = static_cast<ct::DistanceParams::Method>(ui->combo_method->currentData().toInt());

    // 根据选择的方法，只读取对应页面的控件值
    if (params.method == ct::DistanceParams::C2C_NEAREST) {
        // Nearest
        m_threshold = ui->dsb_nearestThreshold->value();
    }
    else {
        m_threshold = ui->dsb_meanThreshold->value();
        if (params.method == ct::DistanceParams::C2C_KNN_MEAN) {
            // 读取子栈 Page 0 的 K 值
            params.k_knn = ui->sb_Knn->value();
        }
        else if (params.method == ct::DistanceParams::C2C_RADIUS_MEAN) {
            // 读取子栈 Page 1 的 Radius 值
            params.radius = ui->dsb_radius->value();
        }
    }

    this->hide();
    m_progress->showProgress("Calculating Distance...");

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

    auto refCloud = m_refCloud;
    auto compCloud = m_compCloud;
    auto threshold = m_threshold;
    auto export_unchanged = ui->chk_export->isChecked();
    auto comp_has_colors = m_compCloud->hasColors();
    auto comp_has_normals = m_compCloud->hasNormals();
    auto comp_id = m_compCloud->id();

    auto future = QtConcurrent::run([refCloud, compCloud, params, cancel, on_progress]() {
        return ct::DistanceCalculator::calculate(refCloud, compCloud, params, cancel, on_progress);
    });

    auto* watcher = new QFutureWatcher<ct::DistanceResult>(this);
    connect(watcher, &QFutureWatcher<ct::DistanceResult>::finished, this, [=]() {
        auto result = watcher->result();
        m_progress->closeProgress();
        delete cancel;

        if (!result.success) {
            printE(QString("Distance calculation failed: %1").arg(QString::fromStdString(result.error_msg)));
            watcher->deleteLater();
            return;
        }

        const auto& distances = result.distances;

        if (distances.size() != compCloud->size()){
            printW(QString("Calculation error: Result size mismatch."));
            watcher->deleteLater();
            return;
        }

        printI(QString("Distance calculation finished in %1 s").arg(result.time_ms));

        ct::Cloud::Ptr changed(new ct::Cloud);
        ct::Cloud::Ptr unchanged(new ct::Cloud);

        // 初始化八叉树空间 (使用原始点云的 Box)
        changed->initOctree(compCloud->box());
        if (export_unchanged) {
            unchanged->initOctree(compCloud->box());
        }

        // 启用属性
        if (comp_has_colors) { changed->enableColors(); unchanged->enableColors(); }
        if (comp_has_normals) { changed->enableNormals(); unchanged->enableNormals(); }

        // 准备批量缓冲区
        struct CloudBuffer {
            std::vector<ct::PointXYZ> pts;
            std::vector<ct::ColorRGB> colors;
            std::vector<ct::CompressedNormal> normals;
            std::vector<float> dists; // 距离标量值

            void clear() { pts.clear(); colors.clear(); normals.clear(); dists.clear(); }
        };

        CloudBuffer buf_changed, buf_unchanged;
        size_t batch_size = 50000;

        buf_changed.pts.reserve(batch_size);
        buf_unchanged.pts.reserve(batch_size);

        // 全局索引，用于访问 distances
        size_t global_idx = 0;

        // 遍历 Block
        const auto& blocks = compCloud->getBlocks();

        for (const auto& block : blocks) {
            if (block->empty()) continue;

            size_t n = block->size();

            for (size_t i = 0; i < n; ++i) {
                float d = distances[global_idx++];

                if (std::isnan(d)) continue; // 跳过无效点

                // 决定归属
                bool is_changed = (d > threshold);

                // 如果是不变点且不需要导出，跳过
                if (!is_changed && !export_unchanged) continue;

                CloudBuffer& target = is_changed ? buf_changed : buf_unchanged;

                // 收集数据
                target.pts.push_back(block->m_points[i]);

                if (comp_has_colors && block->m_colors) {
                    target.colors.push_back((*block->m_colors)[i]);
                }

                if (comp_has_normals && block->m_normals) {
                    target.normals.push_back((*block->m_normals)[i]);
                }

                target.dists.push_back(d);

                // 批满提交
                if (target.pts.size() >= batch_size) {
                    ct::Cloud::Ptr targetCloud = is_changed ? changed : unchanged;

                    // 构造标量 map
                    std::unordered_map<std::string, std::vector<float>> scalarMap;
                    scalarMap["C2C_Distance"] = target.dists;

                    targetCloud->addPoints(target.pts,
                                           target.colors.empty() ? nullptr : &target.colors,
                                           target.normals.empty() ? nullptr : &target.normals,
                                           &scalarMap);

                    target.clear();
                }
            }
        }

        // 提交剩余数据
        auto flushBuffer = [&](CloudBuffer& buf, ct::Cloud::Ptr cloud) {
            if (!buf.pts.empty()) {
                std::unordered_map<std::string, std::vector<float>> scalarMap;
                scalarMap["C2C_Distance"] = buf.dists;
                cloud->addPoints(buf.pts,
                                 buf.colors.empty() ? nullptr : &buf.colors,
                                 buf.normals.empty() ? nullptr : &buf.normals,
                                 &scalarMap);
            }
        };

        flushBuffer(buf_changed, changed);
        if (export_unchanged) flushBuffer(buf_unchanged, unchanged);

        // 更新统计
        changed->update();
        unchanged->update();

        // 1. 处理变化点云
        if (!changed->empty()) {
            changed->makeAdaptive();
        }

        // 2. 处理未变化点云
        if (!unchanged->empty()) {
            unchanged->makeAdaptive();
        }

        // 结果处理
        std::vector<ct::Cloud::Ptr> results;

        if (!changed->empty()){
            changed->setId(comp_id);
            setupResultCloud(changed, "_Changed");
            results.push_back(changed);
        } else {
            printI("No changed points found (all within threshold).");
        }

        if (!unchanged->empty()) {
            unchanged->setId(comp_id);
            setupResultCloud(unchanged, "_Unchanged");
            results.push_back(unchanged);
        }

        if (!results.empty()) {
            QString groupName = QString::fromStdString(comp_id + "_ChangeDetect");
            m_cloudtree->addResultGroup(compCloud, results, groupName);
        }
        this->accept();
        watcher->deleteLater();
    });
    watcher->setFuture(future);
}

void ChangeDetectPlugin::onCancel() {
    this->close();
}

void ChangeDetectPlugin::onMethodChanged(int index) {
    auto method = static_cast<ct::DistanceParams::Method>(ui->combo_method->itemData(index).toInt());

    if (method == ct::DistanceParams::C2C_NEAREST) {
        ui->stackedWidget->setCurrentIndex(0);
    }
    else {
        ui->stackedWidget->setCurrentIndex(1); // 切换到 Local Mean 页

        if (method == ct::DistanceParams::C2C_KNN_MEAN) {
            ui->lblParamName->setText("Neighbors (K):");
            ui->stackInput->setCurrentIndex(0); // 显示 SpinBox
        } else {
            ui->lblParamName->setText("Radius (m):");
            ui->stackInput->setCurrentIndex(1); // 显示 DoubleSpinBox
        }
    }
}
