//
// Created by LBC on 2025/1/10.
//

#include "descriptor.h"
#include "ui_descriptor.h"

#define DESCRIPTOR_TYPE_PFHEstimation                               (0)
#define DESCRIPTOR_TYPE_FPFHEstimation                              (1)
#define DESCRIPTOR_TYPE_VFHEstimation                               (2)
#define DESCRIPTOR_TYPE_ESFEstimation                               (3)
#define DESCRIPTOR_TYPE_GASDEstimation                              (4)
#define DESCRIPTOR_TYPE_GASDColorEstimation                         (5)
#define DESCRIPTOR_TYPE_RSDEstimation                               (6)
#define DESCRIPTOR_TYPE_GRSDEstimation                              (7)
#define DESCRIPTOR_TYPE_CRHEstimation                               (8)
#define DESCRIPTOR_TYPE_CVFHEstimation                              (9)
#define DESCRIPTOR_TYPE_ShapeContext3DEstimation                    (10)
#define DESCRIPTOR_TYPE_SHOTEstimation                              (11)
#define DESCRIPTOR_TYPE_SHOTColorEstimation                         (12)
#define DESCRIPTOR_TYPE_UniqueShapeContext                          (13)
#define DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation          (14)
#define DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation          (15)
#define DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation           (16)


Descriptor::Descriptor(QWidget *parent) :
        CustomDialog(parent), ui(new Ui::Descriptor),
        m_plotter(new pcl::visualization::PCLPlotter) {
    ui->setupUi(this);

    connect(ui->btn_apply, &QPushButton::clicked, this, &Descriptor::preview);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Descriptor::reset);

    connect(ui->cbox_feature, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index) {
        // 如果index = 0，意味着选择了特征类型，则显示特征类型下拉框
        if (index == 0) {
            ui->cbox_type->show();
            ui->cbox_lrf->hide();
            // 显示特征类型对应的停靠面板
            ui->stackedWidget->setCurrentIndex(ui->cbox_type->currentIndex());
        } else {
            ui->cbox_type->hide();
            ui->cbox_lrf->show();
            // 显示特征类型对应的停靠面板
            ui->stackedWidget->setCurrentIndex(ui->cbox_lrf->currentIndex() + 14);
        }
    });

    connect(ui->cbox_lrf, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index) {
        ui->stackedWidget->setCurrentIndex(index + 14);
    });

    // 初始情况下，显示特征类型下拉框，隐藏local reference frame下拉框
    ui->cbox_feature->setCurrentIndex(0);
    ui->cbox_type->setCurrentIndex(0);
    ui->cbox_lrf->hide();

}

Descriptor::~Descriptor() {
    delete ui;
}

void Descriptor::preview() {
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) {
        printW("Please select at least one cloud!");
        return;
    }

    // clearPlots() 方法的作用是清除当前 PCLPlotter 窗口中所有的图表或绘图内容
    m_plotter->clearPlots();
    if (ui->check_show_histogram->isChecked())
        m_plotter.reset(new pcl::visualization::PCLPlotter);

    for (auto &cloud: selected_clouds) {
        if (ui->spin_k->value() == 0 && ui->dspin_r->value() == 0) {
            printW("Parameter set error!");
            return;
        }

        int k = ui->spin_k->value();
        double radius = ui->dspin_r->value();
        m_cloudtree->showProgress("Computing Feature Descriptor...");

        switch (ui->cbox_type->currentIndex()) {
            case DESCRIPTOR_TYPE_PFHEstimation: {
                m_cloudview->showInfo("PFHEstimation", 1);
                auto future = QtConcurrent::run([cloud, k, radius]() {
                    return ct::Features::PFHEstimation(cloud, k, radius);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_FPFHEstimation: {
                m_cloudview->showInfo("FPFHEstimation", 1);
                auto future = QtConcurrent::run([cloud, k, radius]() {
                    return ct::Features::FPFHEstimation(cloud, k, radius);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_VFHEstimation: {
                m_cloudview->showInfo("VFHEstimation", 1);
                Eigen::Vector3f dir(ui->dspin_vpx1->value(), ui->dspin_vpy1->value(), ui->dspin_vpz1->value());
                auto future = QtConcurrent::run([cloud, dir]() {
                    return ct::Features::VFHEstimation(cloud, dir);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_ESFEstimation: {
                m_cloudview->showInfo("ESFEstimation", 1);
                auto future = QtConcurrent::run([cloud]() {
                    return ct::Features::ESFEstimation(cloud);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_GASDEstimation: {
                m_cloudview->showInfo("GASDEstimation", 1);
                Eigen::Vector3f dir(ui->dspin_vx1->value(), ui->dspin_vy1->value(), ui->dspin_vz1->value());
                int shgs = ui->spin_shgs1->value();
                int shs = ui->spin_shs1->value();
                int interp = ui->cbox_interp1->currentIndex();
                auto future = QtConcurrent::run([cloud, dir, shgs, shs, interp]() {
                    return ct::Features::GASDEstimation(cloud, dir, shgs, shs, interp);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_GASDColorEstimation: {
                m_cloudview->showInfo("GASDColorEstimation", 1);
                Eigen::Vector3f dir(ui->dspin_vx2->value(), ui->dspin_vy2->value(), ui->dspin_vz2->value());
                int shgs = ui->spin_shgs2->value();
                int shs = ui->spin_shs2->value();
                int interp = ui->cbox_interp2->currentIndex();
                int chgs = ui->spin_chgs->value();
                int chs = ui->spin_chs->value();
                int cinterp = ui->cbox_cinterp->currentIndex();
                auto future = QtConcurrent::run([cloud, dir, shgs, shs, interp, chgs, chs, cinterp]() {
                    return ct::Features::GASDColorEstimation(cloud, dir, shgs, shs, interp, chgs, chs, cinterp);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_RSDEstimation: {
                m_cloudview->showInfo("RSDEstimation", 1);
                int nr_subdiv = ui->spin_nr_subdiv->value();
                double plane_radius = ui->dspin_plane_radius->value();
                auto future = QtConcurrent::run([cloud, nr_subdiv, plane_radius]() {
                    return ct::Features::RSDEstimation(cloud, nr_subdiv, plane_radius);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_GRSDEstimation: {
                m_cloudview->showInfo("GRSDEstimation", 1);
                auto future = QtConcurrent::run([cloud]() {
                    return ct::Features::GRSDEstimation(cloud);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_CRHEstimation: {
                m_cloudview->showInfo("CRHEstimation", 1);
                Eigen::Vector3f dir(ui->dspin_vpx2->value(), ui->dspin_vpy2->value(), ui->dspin_vpz2->value());
                auto future = QtConcurrent::run([cloud, dir]() {
                    return ct::Features::CRHEstimation(cloud, dir);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_CVFHEstimation: {
                m_cloudview->showInfo("CVFHEstimation", 1);
                Eigen::Vector3f dir(ui->dspin_vpx2->value(), ui->dspin_vpy2->value(), ui->dspin_vpz2->value());
                float radius_normals = ui->dspin_rn->value();
                float d1 = ui->dspin_d1->value();
                float d2 = ui->dspin_d2->value();
                float d3 = ui->dspin_d3->value();
                int min_neighbors = ui->spin_min->value();
                bool normalize = ui->check_normalize->isChecked();
                auto future = QtConcurrent::run([cloud, dir, radius_normals, d1, d2, d3, min_neighbors, normalize]() {
                    return ct::Features::CVFHEstimation(cloud, dir, radius_normals, d1, d2, d3, min_neighbors, normalize);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_ShapeContext3DEstimation: {
                m_cloudview->showInfo("ShapeContext3DEstimation", 1);
                double min_radius = ui->dspin_min_r->value();
                double r2 = ui->dspin_r2->value();
                auto future = QtConcurrent::run([cloud, min_radius, r2]() {
                    return ct::Features::ShapeContext3DEstimation(cloud, min_radius, r2);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_SHOTEstimation: {
                if (m_lrf_map.find(cloud->id()) == m_lrf_map.end()) {
                    printW("Please Estimation LocalReferenceFrame First!");
                    m_cloudtree->closeProgress();
                    return;
                }
                m_cloudview->showInfo("SHOTEstimation", 1);
                auto lrf = m_lrf_map.find(cloud->id())->second;
                float lrf_radius = ui->dspin_lrf1->value();
                auto future = QtConcurrent::run([cloud, lrf, lrf_radius, k, radius]() {
                    return ct::Features::SHOTEstimation(cloud, lrf, lrf_radius);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_SHOTColorEstimation: {
                if (m_lrf_map.find(cloud->id()) == m_lrf_map.end()) {
                    printW("Please Estimation LocalReferenceFrame First!");
                    m_cloudtree->closeProgress();
                    return;
                }
                m_cloudview->showInfo("SHOTColorEstimation", 1);
                auto lrf = m_lrf_map.find(cloud->id())->second;
                float lrf_radius = ui->dspin_lrf2->value();
                auto future = QtConcurrent::run([cloud, lrf, lrf_radius, k, radius]() {
                    return ct::Features::SHOTColorEstimation(cloud, lrf, lrf_radius);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_UniqueShapeContext: {
                if (m_lrf_map.find(cloud->id()) == m_lrf_map.end()) {
                    printW("Please Estimation LocalReferenceFrame first!");
                    m_cloudtree->closeProgress();
                    return;
                }
                m_cloudview->showInfo("UniqueShapeContext", 1);
                auto lrf = m_lrf_map.find(cloud->id())->second;
                double min_r2 = ui->dspin_min_r2->value();
                double pt_r = ui->dspin_p_r->value();
                double loc_r = ui->dspin_l_r->value();
                auto future = QtConcurrent::run([cloud, lrf, min_r2, pt_r, loc_r]() {
                    return ct::Features::UniqueShapeContext(cloud, lrf, min_r2, pt_r, loc_r);
                });
                auto* watcher = new QFutureWatcher<ct::FeatureResult>(this);
                connect(watcher, &QFutureWatcher<ct::FeatureResult>::finished, this, [=]() {
                    handleFeatureResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation: {
                m_cloudview->showInfo("BOARDLocalReferenceFrameEstimation", 1);
                float t_r = ui->dspin_t_r1->value();
                bool find_holes = ui->check_find_holes->isChecked();
                float margin_thresh = ui->dspin_m_t1->value();
                int size = ui->spin_size->value();
                float prob_thresh = ui->dspin_h_t1->value();
                float steep_thresh = ui->dspin_s_t1->value();
                auto future = QtConcurrent::run([cloud, t_r, find_holes, margin_thresh, size, prob_thresh, steep_thresh]() {
                    return ct::Features::BOARDLocalReferenceFrameEstimation(cloud, t_r, find_holes, margin_thresh, size, prob_thresh, steep_thresh);
                });
                auto* watcher = new QFutureWatcher<ct::LRFResult>(this);
                connect(watcher, &QFutureWatcher<ct::LRFResult>::finished, this, [=]() {
                    handleLrfResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation: {
                m_cloudview->showInfo("FLARELocalReferenceFrameEstimation", 1);
                float t_r = ui->dspin_t_r2->value();
                float margin_thresh = ui->dspin_m_t2->value();
                int min_n = ui->spin_m_n2->value();
                int min_t = ui->spin_m_t2->value();
                auto future = QtConcurrent::run([cloud, t_r, margin_thresh, min_n, min_t]() {
                    return ct::Features::FLARELocalReferenceFrameEstimation(cloud, t_r, margin_thresh, min_n, min_t);
                });
                auto* watcher = new QFutureWatcher<ct::LRFResult>(this);
                connect(watcher, &QFutureWatcher<ct::LRFResult>::finished, this, [=]() {
                    handleLrfResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
            case DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation: {
                m_cloudview->showInfo("SHOTLocalReferenceFrameEstimation", 1);
                auto future = QtConcurrent::run([cloud]() {
                    return ct::Features::SHOTLocalReferenceFrameEstimation(cloud);
                });
                auto* watcher = new QFutureWatcher<ct::LRFResult>(this);
                connect(watcher, &QFutureWatcher<ct::LRFResult>::finished, this, [=]() {
                    handleLrfResult(watcher->result());
                    watcher->deleteLater();
                });
                watcher->setFuture(future);
                break;
            }
        }
    }
}

void Descriptor::reset() {

    /**
     * @breif 这里涉及一个重要的知识点，关于m_plotter.reset，m_plotter->close()中的.运算符和->运算符的区别。
     * 首先，智能指针 std::shared_ptr<T> 本质上是一个对象，不是普通指针。它是一个封装了指针的类
     * .(点运算符)用于对象本身（普通变量）； ->(箭头运算符)用于指针指向的对象（即(*ptr).method()）。
     * ptr.reset() 调用的是 shared_ptr 本身的 reset() 方法。
     * m_plotter->close();  调用 PCLPlotter 的 close() 方法。
     * 但是，但是，但是，在 C++ 中，普通指针（如 int* p）一般只能使用 -> 运算符来访问指针指向的对象的成员，
     * 几乎所有情况下，你不能直接对 p 本身使用 . 来调用 int 的成员，因为 int 不是类，没有成员函数。
     */
    // 重置智能指针m_plotter，释放它所管理的旧对象（如果有的话），并分配一个新的 PCLPlotter 对象。
    m_plotter.reset(new pcl::visualization::PCLPlotter);
    // close 方法用于关闭 PCLPlotter 对象所显示的窗口或界面
    m_plotter->close();
    m_plotter = nullptr;
    m_descriptor_map.clear();
    m_lrf_map.clear();
    m_cloudview->clearInfo();
}

void Descriptor::handleFeatureResult(const ct::FeatureResult &result) {
    m_cloudtree->closeProgress();
    if (!result.feature) return;

    m_descriptor_map[result.id] = result.feature;
    QString id = QString::fromStdString(result.id);
    float time = result.time_ms;

    switch (ui->cbox_type->currentIndex()) {
        case DESCRIPTOR_TYPE_PFHEstimation:
            printI(QString("Estimate cloud[id:%1] PFHFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("PFHEstimation");
            // 显示特征直方图
            m_plotter->addFeatureHistogram(*result.feature->pfh, 1000);
            break;
        case DESCRIPTOR_TYPE_FPFHEstimation:
            printI(QString("Estimate cloud[id:%1] FPFHFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("FPFHEstimation");
            m_plotter->addFeatureHistogram(*result.feature->fpfh, 1000);
            break;
        case DESCRIPTOR_TYPE_VFHEstimation:
            printI(QString("Estimate cloud[id:%1] VFHFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("VFHEstimation");
            m_plotter->addFeatureHistogram(*result.feature->vfh, 1000);
            break;
        case DESCRIPTOR_TYPE_ESFEstimation:
            printI(QString("Estimate cloud[id:%1] ESFFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("ESFEstimation");
            m_plotter->addFeatureHistogram(*result.feature->esf, 1000);
            break;
        case DESCRIPTOR_TYPE_GASDEstimation:
            printI(QString("Estimate cloud[id:%1] GASDFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("GASDEstimation");
            m_plotter->addFeatureHistogram(*result.feature->gasd, 1000);
            break;
        case DESCRIPTOR_TYPE_GASDColorEstimation:
            printI(QString("Estimate cloud[id:%1] GASDColorFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("GASDColorEstimation");
            m_plotter->addFeatureHistogram(*result.feature->gasdc, 1000);
            break;
        case DESCRIPTOR_TYPE_RSDEstimation:
            printI(QString("Estimate cloud[id:%1] RSDEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("RSDEstimation");
            // m_plotter->addFeatureHistogram(*result.feature->rsd, 1000);
            break;
        case DESCRIPTOR_TYPE_GRSDEstimation:
            printI(QString("Estimate cloud[id:%1] GRSDEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("GRSDEstimation");
            m_plotter->addFeatureHistogram(*result.feature->grsd, 1000);
            break;
        case DESCRIPTOR_TYPE_CRHEstimation:
            printI(QString("Estimate cloud[id:%1] CRHEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("CRHEstimation");
            m_plotter->addFeatureHistogram(*result.feature->crh, 1000);
            break;
        case DESCRIPTOR_TYPE_CVFHEstimation:
            printI(QString("Estimate cloud[id:%1] CVFHEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("CVFHEstimation");
            m_plotter->addFeatureHistogram(*result.feature->vfh, 1000);
            break;
        case DESCRIPTOR_TYPE_ShapeContext3DEstimation:
            printI(QString("Estimate cloud[id:%1] ShapeContext3DEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("ShapeContext3DEstimation");
            // m_plotter->addFeatureHistogram(*result.feature->sc3d, 1000);
            break;
        case DESCRIPTOR_TYPE_SHOTEstimation:
            printI(QString("Estimate cloud[id:%1] SHOTFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("SHOTEstimation");
            // m_plotter->addFeatureHistogram(*result.feature->shot, 1000);
            break;
        case DESCRIPTOR_TYPE_SHOTColorEstimation:
            printI(QString("Estimate cloud[id:%1] SHOTColorFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("SHOTColorEstimation");
            // m_plotter->addFeatureHistogram(*result.feature->shotc, 1000);
            break;
        case DESCRIPTOR_TYPE_UniqueShapeContext:
            printI(QString("Estimate cloud[id:%1] UniqueShapeContext done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("UniqueShapeContext");
            // m_plotter->addFeatureHistogram(*result.feature->usc, 1000);
            break;
    }
    if (ui->check_show_histogram->isChecked())
    {
        // 在窗口中计算出一个点的位置，并设置为直方图窗口的位置
        QPoint pos = m_cloudview->mapToGlobal(QPoint((m_cloudview->width() - 640) / 2, (m_cloudview->height() - 200) / 2));
        m_plotter->setWindowPosition(pos.x(), pos.y());
        // 绘制显示直方图
        m_plotter->plot();
    }
}

void Descriptor::handleLrfResult(const ct::LRFResult &result)
{
    m_cloudtree->closeProgress();
    QString id = QString::fromStdString(result.id);
    float time = result.time_ms;

    switch(ui->cbox_type->currentIndex())
    {
        case DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation:
            printI(QString("Estimate cloud[id:%1] BOARDLocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
            break;
        case DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation:
            printI(QString("Estimate cloud[id:%1] FLARELocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
            break;
        case DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation:
            printI(QString("Estimate cloud[id:%1] SHOTLocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
            break;
    }
    m_lrf_map[result.id] = result.lrf;
}