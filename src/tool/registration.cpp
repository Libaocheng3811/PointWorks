//
// Created by LBC on 2025/1/10.
//

#include "registration.h"
#include "base/cloudtree.h"
#include "ui_registration.h"
#include "core/cloud.h"
#include "core/common.h"

#define REG_TYPE_CorrespondenceEstimation               (0)
#define REG_TYPE_CorrespondenceRejector                 (1)
#define REG_TYPE_TransformationEstimation               (2)
#define REG_TYPE_Registration                           (3)

#define REG_TYPE_CE_Base                                (0)
#define REG_TYPE_CE_BackProjection                      (1)
#define REG_TYPE_CE_NormalShooting                      (2)

#define REG_TYPE_CR_Distance                            (0)
#define REG_TYPE_CR_MedianDistance                      (1)
#define REG_TYPE_CR_OneToOne                            (2)
#define REG_TYPE_CR_OrganizedBoundary                   (3)
#define REG_TYPE_CR_Poly                                (4)
#define REG_TYPE_CR_SampleConsensus                     (5)
#define REG_TYPE_CR_SurfaceNormal                       (6)
#define REG_TYPE_CR_Trimmed                             (7)
#define REG_TYPE_CR_VarTrimmed                          (8)

#define REG_TYPE_TE_2D                                  (0)
#define REG_TYPE_TE_3Point                              (1)
#define REG_TYPE_TE_DualQuaternion                      (2)
#define REG_TYPE_TE_LM                                  (3)
#define REG_TYPE_TE_PointToPlane                        (4)
#define REG_TYPE_TE_PointToPlaneLLS                     (5)
#define REG_TYPE_TE_SVD                                 (6)
#define REG_TYPE_TE_SymmetricPointToPlaneLLS            (7)

#define REG_TYPE_PointCloud                             (0)
#define REG_TYPE_PFHFeature                             (1)
#define REG_TYPE_FPFHFeature                            (2)

#define REG_IterativeClosestPoint                       (0)
#define REG_IterativeClosestPointWithNormals            (1)
#define REG_IterativeClosestPointNonLinear              (2)
#define REG_GeneralizedIterativeClosestPoint            (3)
#define REG_SampleConsensusInitialAlignment             (4)
#define REG_SampleConsensusPrerejective                 (5)
#define REG_NormalDistributionsTransform                (6)
#define REG_FPCSInitialAlignment                        (7)
#define REG_KFPCSInitialAlignment                       (8)

#define REG_CORRE_PRE_FLAG                              "-corre"
#define REG_TRANS_PRE_FLAG                              "-trans"
#define REG_ALIGN_PRE_FLAG                              "-align"
#define REG_ALIGN_ADD_FLAG                              "align-"

Registration::Registration(QWidget *parent) :
        CustomDock(parent), ui(new Ui::Registration),
        m_target_cloud(nullptr),
        m_source_cloud(nullptr),
        m_corr(nullptr),
        m_ce(nullptr)
{
    ui->setupUi(this);

    connect(ui->btn_setTarget, &QPushButton::clicked, this, &Registration::setTarget);
    connect(ui->btn_setSource, &QPushButton::clicked, this, &Registration::setSource);
    connect(ui->btn_setCE, &QPushButton::clicked, this, &Registration::setCorrespondenceEstimation);
    connect(ui->btn_addCR, &QPushButton::clicked, this, &Registration::addCorrespondenceRejector);
    connect(ui->btn_removeCR, &QPushButton::clicked, this, &Registration::removeCorrespondenceRejector);
    connect(ui->btn_setTE, &QPushButton::clicked, this, &Registration::setTransformationEstimation);
    connect(ui->btn_preview, &QPushButton::clicked, this, &Registration::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Registration::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Registration::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Registration::reset);

    // Setup future watchers
    m_ce_watcher = new QFutureWatcher<ct::CorrespondenceResult>(this);
    connect(m_ce_watcher, &QFutureWatcher<ct::CorrespondenceResult>::finished, this, [=]() {
        handleCorrespondenceResult(m_ce_watcher->result());
    });

    m_cr_watcher = new QFutureWatcher<ct::RejectorResult>(this);
    connect(m_cr_watcher, &QFutureWatcher<ct::RejectorResult>::finished, this, [=]() {
        handleRejectorResult(m_cr_watcher->result());
    });

    m_te_watcher = new QFutureWatcher<ct::TransformationResult>(this);
    connect(m_te_watcher, &QFutureWatcher<ct::TransformationResult>::finished, this, [=]() {
        handleTransformationResult(m_te_watcher->result());
    });

    m_reg_watcher = new QFutureWatcher<ct::RegistrationResult>(this);
    connect(m_reg_watcher, &QFutureWatcher<ct::RegistrationResult>::finished, this, [=]() {
        handleRegistrationResult(m_reg_watcher->result());
    });

    // 为什么需要 static_cast？
    // Qt 信号的重载函数（如currentIndexChanged）在编译时无法直接确定具体版本，需通过强制类型转换指定目标信号。
    // [=] 代表 以值捕获所有外部变量，即 lambda 内部可以访问 ui，但 index 是通过参数传入的，不是捕获的。
    // 捕获列表 [=] 只会捕获外部作用域的变量，但 index 不是外部变量，而是 currentIndexChanged(int) 信号传进来的参数
    // index 是 Qt 信号槽机制传递给 lambda 的，不是 lambda 外部作用域已有的变量，所以它必须通过参数 (int index) 显式声明。
    connect(ui->cbox_TransEst, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index)
    {
        if (index != REG_TYPE_TE_SymmetricPointToPlaneLLS)
            ui->stackedWidget_5->setCurrentIndex(0);
        else
            ui->stackedWidget_5->setCurrentIndex(1);
    });

    ui->cbox_type->setCurrentIndex(0);
    ui->cbox_feature->setCurrentIndex(0);
    ui->cbox_correEst->setCurrentIndex(0);
    ui->cbox_correRej->setCurrentIndex(0);
    ui->cbox_TransEst->setCurrentIndex(0);

    ui->stackedWidget->setCurrentIndex(0);
    ui->stackedWidget_2->setCurrentIndex(0);
    ui->stackedWidget_3->setCurrentIndex(0);
    ui->stackedWidget_4->setCurrentIndex(0);
    ui->stackedWidget_5->setCurrentIndex(0);
}

Registration::~Registration() {
    delete ui;
}

ct::RegistrationContext Registration::buildContext() const
{
    ct::RegistrationContext ctx;
    ctx.source_cloud = m_source_cloud;
    ctx.target_cloud = m_target_cloud;
    ctx.correspondences = m_corr;
    ctx.correspondence_estimation = m_ce;
    ctx.transformation_estimation = m_te;
    ctx.correspondence_rejectors = m_cr_map;

    ctx.params.max_iterations = ui->spin_nr_iterations->value();
    ctx.params.ransac_iterations = ui->spin_ransac_iterations->value();
    ctx.params.inlier_threshold = ui->dspin_inlier_threshold->value();
    ctx.params.distance_threshold = ui->dspin_distance_threshold->value();
    ctx.params.transformation_epsilon = ui->dspin_transformation_epsilon->value();
    ctx.params.transformation_rotation_epsilon = ui->dspin_transformation_rotation_epsilon->value();
    ctx.params.euclidean_fitness_epsilon = ui->dspin_euclidean_fitness_epsilon->value();

    return ctx;
}

void Registration::setTarget()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW(tr("Please select a cloud."));
        return;
    }
    m_target_cloud = selected_clouds.front();
    if (m_source_cloud && m_source_cloud->id() == m_target_cloud->id())
    {
        printW(tr("Please choose another cloud as target cloud!"));
        return;
    }
    m_cloudview->removeShape(QString::fromStdString(m_target_cloud->boxId()));
    m_cloudview->setPointCloudColor(m_target_cloud, ct::Color::Red);
    m_cloudview->showInfo("Target cloud(Red): " + QString::fromStdString(m_target_cloud->id()), 1);
    if (m_source_cloud)
    {
        ui->btn_setTarget->setEnabled(false);
        ui->btn_setSource->setEnabled(false);
        m_cloudtree->setEnabled(false);
    }

}

void Registration::setSource()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW(tr("Please select a cloud."));
        return;
    }
    m_source_cloud = selected_clouds.front();
    if (m_target_cloud && m_source_cloud->id() == m_target_cloud->id())
    {
        printW(tr("Please choose another cloud as source cloud!"));
        return;
    }
    m_cloudview->removeShape(QString::fromStdString(m_source_cloud->boxId()));
    m_cloudview->setPointCloudColor(m_source_cloud, ct::Color::Green);
    m_cloudview->showInfo("Source cloud(Green): " + QString::fromStdString(m_source_cloud->id()), 1);
    if (m_target_cloud)
    {
        ui->btn_setTarget->setEnabled(false);
        ui->btn_setSource->setEnabled(false);
        m_cloudtree->setEnabled(false);
    }
}

void Registration::setCorrespondenceEstimation()
{
    if (m_ce == nullptr)
    {
        printW("Please estimate correspondences first!");
        return;
    }
    printI("Set CorrespondenceEstimation done!");
}

void Registration::addCorrespondenceRejector()
{
    if(m_cr_map.find(ui->cbox_correRej->currentIndex()) == m_cr_map.end())
    {
        printW("Please select correspondenceRejector first!");
        return;
    }
    printI("Add CorrespondenceRejector done!");
}

// 移除配准拒绝器
void Registration::removeCorrespondenceRejector()
{
    m_cr_map.erase(ui->cbox_correRej->currentIndex());
    printI("Remove CorrespondenceRejector done!");
}

void Registration::setTransformationEstimation()
{
    if(m_te == nullptr)
    {
        printW("Please estimate transformation first!");
        return;
    }
    printI("Set TransformationEstimation done!");
}

void Registration::preview()
{
    if (!m_source_cloud || !m_target_cloud)
    {
        printW("Please select a source cloud and a target cloud first!");
        return;
    }

    m_cloudtree->showProgress("Processing Registration...");

    switch (ui->cbox_type->currentIndex())
    {
        case REG_TYPE_CorrespondenceEstimation:
            m_cloudview->removeCorrespondences(QString::fromStdString(m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG));
            switch (ui->cbox_correEst->currentIndex())
            {
                case REG_TYPE_CE_Base:
                    m_cloudview->showInfo("CorrespondenceEstimation: Base", 3);
                    if (ui->cbox_ce_type->currentIndex() > 0 && m_descriptor == nullptr)
                    {
                        printW("Please estimate target and source cloud descriptor first!");
                        m_cloudtree->closeProgress();
                        return;
                    }
                    switch (ui->cbox_ce_type->currentIndex())
                    {
                        case REG_TYPE_PointCloud:
                            m_cloudview->showInfo("Estimation Type: PointCloud", 4);
                        {
                            auto result = ct::Registration::CorrespondenceEstimation<ct::PointXYZRGBN>(
                                m_source_cloud->toPCL_XYZRGBN(),
                                m_target_cloud->toPCL_XYZRGBN());
                            handleCorrespondenceResult(result);
                        }
                            break;
                        case REG_TYPE_PFHFeature:
                            m_cloudview->showInfo("Estimation Type: PFHFeature", 4);
                            if (m_descriptor->getDescriptor(m_source_cloud->id())->pfh && m_descriptor->getDescriptor(m_target_cloud->id())->pfh)
                            {
                                auto result = ct::Registration::CorrespondenceEstimation<pcl::PFHSignature125>(
                                    m_descriptor->getDescriptor(m_source_cloud->id())->pfh,
                                    m_descriptor->getDescriptor(m_target_cloud->id())->pfh);
                                handleCorrespondenceResult(result);
                            }
                            else
                            {
                                printW("Please estimate target and source cloud PFHFeature first!");
                                m_cloudtree->closeProgress();
                                return;
                            }

                            break;
                        case REG_TYPE_FPFHFeature:
                            m_cloudview->showInfo("Estimation Type: FPFHFeature", 4);
                            if (m_descriptor->getDescriptor(m_source_cloud->id())->fpfh && m_descriptor->getDescriptor(m_target_cloud->id())->fpfh)
                            {
                                auto result = ct::Registration::CorrespondenceEstimation<pcl::FPFHSignature33>(
                                    m_descriptor->getDescriptor(m_source_cloud->id())->fpfh,
                                    m_descriptor->getDescriptor(m_target_cloud->id())->fpfh);
                                handleCorrespondenceResult(result);
                            }
                            else
                            {
                                printW("Please estimate target and source cloud FPFHFeature first!");
                                m_cloudtree->closeProgress();
                                return;
                            }
                            break;
                            //TODO: other feature
                    }
                    break;
                case REG_TYPE_CE_BackProjection:
                    m_cloudview->showInfo("CorrespondenceEstimation: BackProjection", 3);
                    if (!m_source_cloud->hasNormals() || !m_target_cloud->hasNormals())
                    {
                        printW("Please estimate target and source cloud normals first!");
                        m_cloudtree->closeProgress();
                        return;
                    }
                {
                    auto ctx = buildContext();
                    int k = ui->spin_k1->value();
                    auto future = QtConcurrent::run([ctx, k]() {
                        return ct::Registration::CorrespondenceEstimationBackProjection(ctx, k);
                    });
                    m_ce_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CE_NormalShooting:
                    m_cloudview->showInfo("CorrespondenceEstimation: NormalShooting", 3);
                    if (!m_target_cloud->hasNormals())
                    {
                        printW("Please estimate target cloud normals first!");
                        m_cloudtree->closeProgress();
                        return;
                    }
                {
                    auto ctx = buildContext();
                    int k = ui->spin_k2_2->value();
                    auto future = QtConcurrent::run([ctx, k]() {
                        return ct::Registration::CorrespondenceEstimationNormalShooting(ctx, k);
                    });
                    m_ce_watcher->setFuture(future);
                }
                    break;
            }
            break;
        case REG_TYPE_CorrespondenceRejector:
            m_cloudview->removeCorrespondences(QString::fromStdString(m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG));
            if (m_corr == nullptr)
            {
                printW("please estimate target and source cloud correspondence first!");
                m_cloudtree->closeProgress();
                return;
            }
            switch (ui->cbox_correRej->currentIndex())
            {
                case REG_TYPE_CR_Distance:
                    m_cloudview->showInfo("CorrespondenceRejector: Distance", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    float distance = ui->dspin_distance->value();
                    auto future = QtConcurrent::run([ctx, input_corr, distance]() {
                        return ct::Registration::CorrespondenceRejectorDistance(ctx, input_corr, distance);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CR_MedianDistance:
                    m_cloudview->showInfo("CorrespondenceRejector: MedianDistance", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    double factor = ui->dspin_factor->value();
                    auto future = QtConcurrent::run([ctx, input_corr, factor]() {
                        return ct::Registration::CorrespondenceRejectorMedianDistance(ctx, input_corr, factor);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CR_OneToOne:
                    m_cloudview->showInfo("CorrespondenceRejector: OneToOne", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    auto future = QtConcurrent::run([ctx, input_corr]() {
                        return ct::Registration::CorrespondenceRejectorOneToOne(ctx, input_corr);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CR_OrganizedBoundary:
                    m_cloudview->showInfo("CorrespondenceRejector: OrganizedBoundary", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    int val = ui->spin_val->value();
                    auto future = QtConcurrent::run([ctx, input_corr, val]() {
                        return ct::Registration::CorrespondenceRejectionOrganizedBoundary(ctx, input_corr, val);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CR_Poly:
                    m_cloudview->showInfo("CorrespondenceRejector: Poly", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    int cardinality = ui->spin_cardinality->value();
                    float similarity_threshold = ui->dspin_similarity_threshold->value();
                    int iterations = ui->spin_iterations->value();
                    auto future = QtConcurrent::run([ctx, input_corr, cardinality, similarity_threshold, iterations]() {
                        return ct::Registration::CorrespondenceRejectorPoly(ctx, input_corr, cardinality, similarity_threshold, iterations);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CR_SampleConsensus:
                    m_cloudview->showInfo("CorrespondenceRejector: SampleConsensus", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    double threshold = ui->dspin_threshold->value();
                    int max_iterations = ui->spin_max_iterations->value();
                    bool refine = ui->check_refine->isChecked();
                    auto future = QtConcurrent::run([ctx, input_corr, threshold, max_iterations, refine]() {
                        return ct::Registration::CorrespondenceRejectorSampleConsensus(ctx, input_corr, threshold, max_iterations, refine);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CR_SurfaceNormal:
                    m_cloudview->showInfo("CorrespondenceRejector: SurfaceNormal", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    double threshold = ui->dspin_threshold_2->value();
                    auto future = QtConcurrent::run([ctx, input_corr, threshold]() {
                        return ct::Registration::CorrespondenceRejectorSurfaceNormal(ctx, input_corr, threshold);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CR_Trimmed:
                    m_cloudview->showInfo("CorrespondenceRejector: Trimmed", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    float ratio = ui->dspin_ratio->value();
                    int min_corre = ui->spin_min_corre->value();
                    auto future = QtConcurrent::run([ctx, input_corr, ratio, min_corre]() {
                        return ct::Registration::CorrespondenceRejectorTrimmed(ctx, input_corr, ratio, min_corre);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_CR_VarTrimmed:
                    m_cloudview->showInfo("CorrespondenceRejector: VarTrimmed", 3);
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    double min_ratio = ui->dspin_min_ratio->value();
                    double max_ratio = ui->dspin_max_ratio->value();
                    auto future = QtConcurrent::run([ctx, input_corr, min_ratio, max_ratio]() {
                        return ct::Registration::CorrespondenceRejectorVarTrimmed(ctx, input_corr, min_ratio, max_ratio);
                    });
                    m_cr_watcher->setFuture(future);
                }
                    break;
            }
            break;
        case REG_TYPE_TransformationEstimation:
            m_cloudview->removePointCloud(QString::fromStdString(m_source_cloud->id() + REG_TRANS_PRE_FLAG));
            switch (ui->cbox_TransEst->currentIndex())
            {
                case REG_TYPE_TE_2D:
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    auto future = QtConcurrent::run([ctx, input_corr]() {
                        return ct::Registration::TransformationEstimation2D(ctx, input_corr);
                    });
                    m_te_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_TE_3Point:
                    if ((m_target_cloud->size() != 3) || (m_source_cloud->size() != 3))
                    {
                        printW(QString("Number of points in source (%1) and target (%2) must be 3!").arg(m_source_cloud->size()).arg(m_target_cloud->size()));
                        m_cloudtree->closeProgress();
                        return;
                    }
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    auto future = QtConcurrent::run([ctx, input_corr]() {
                        return ct::Registration::TransformationEstimation3Point(ctx, input_corr);
                    });
                    m_te_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_TE_DualQuaternion:
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    auto future = QtConcurrent::run([ctx, input_corr]() {
                        return ct::Registration::TransformationEstimationDualQuaternion(ctx, input_corr);
                    });
                    m_te_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_TE_LM:
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    auto future = QtConcurrent::run([ctx, input_corr]() {
                        return ct::Registration::TransformationEstimationLM(ctx, input_corr);
                    });
                    m_te_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_TE_PointToPlane:
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    auto future = QtConcurrent::run([ctx, input_corr]() {
                        return ct::Registration::TransformationEstimationPointToPlane(ctx, input_corr);
                    });
                    m_te_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_TE_PointToPlaneLLS:
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    auto future = QtConcurrent::run([ctx, input_corr]() {
                        return ct::Registration::TransformationEstimationPointToPlaneLLS(ctx, input_corr);
                    });
                    m_te_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_TE_SVD:
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    auto future = QtConcurrent::run([ctx, input_corr]() {
                        return ct::Registration::TransformationEstimationSVD(ctx, input_corr);
                    });
                    m_te_watcher->setFuture(future);
                }
                    break;
                case REG_TYPE_TE_SymmetricPointToPlaneLLS:
                {
                    auto ctx = buildContext();
                    auto input_corr = m_corr;
                    bool enforce = ui->check_enforce_same_direction_normals->isChecked();
                    auto future = QtConcurrent::run([ctx, input_corr, enforce]() {
                        return ct::Registration::TransformationEstimationSymmetricPointToPlaneLLS(ctx, input_corr, enforce);
                    });
                    m_te_watcher->setFuture(future);
                }
                    break;
            }
            break;
        case REG_TYPE_Registration:
            m_cloudview->removePointCloud(QString::fromStdString(m_source_cloud->id() + REG_ALIGN_PRE_FLAG));
        {
            auto ctx = buildContext();
            // If continuous mode and previous result exists, use it as source
            if (ui->check_continus->isChecked() && m_reg_map.find(m_source_cloud->id() + m_target_cloud->id()) != m_reg_map.end())
            {
                ct::Cloud::Ptr new_source_cloud = m_reg_map.find(m_source_cloud->id() + m_target_cloud->id())->second;
                ctx.source_cloud = new_source_cloud;
            }
            switch (ui->cbox_registration->currentIndex())
            {
                case REG_IterativeClosestPoint:
                    m_cloudview->showInfo("Registration: IterativeClosestPoint", 3);
                {
                    bool use_recip = ui->check_use_reciprocal->isChecked();
                    auto future = QtConcurrent::run([ctx, use_recip]() {
                        return ct::Registration::IterativeClosestPoint(ctx, use_recip);
                    });
                    m_reg_watcher->setFuture(future);
                }
                    break;
                case REG_IterativeClosestPointWithNormals:
                    m_cloudview->showInfo("Registration: IterativeClosestPointWithNormals", 3);
                {
                    bool use_recip = ui->check_use_reciprocal_2->isChecked();
                    bool use_sym = ui->check_use_symmetric_2->isChecked();
                    bool enforce = ui->check_enforce_samedirection_2->isChecked();
                    auto future = QtConcurrent::run([ctx, use_recip, use_sym, enforce]() {
                        return ct::Registration::IterativeClosestPointWithNormals(ctx, use_recip, use_sym, enforce);
                    });
                    m_reg_watcher->setFuture(future);
                }
                    break;
                case REG_IterativeClosestPointNonLinear:
                    m_cloudview->showInfo("Registration: IterativeClosestPointNonLinear", 3);
                {
                    bool use_recip = ui->check_use_reciprocal_3->isChecked();
                    auto future = QtConcurrent::run([ctx, use_recip]() {
                        return ct::Registration::IterativeClosestPointNonLinear(ctx, use_recip);
                    });
                    m_reg_watcher->setFuture(future);
                }
                    break;
                case REG_GeneralizedIterativeClosestPoint:
                    m_cloudview->showInfo("Registration: GeneralizedIterativeClosestPoint", 3);
                {
                    int k = ui->spin_k_2->value();
                    int max = ui->spin_max->value();
                    double tra_tolerance = ui->dspin_tra_tolerance->value();
                    double rol_tolerance = ui->dspin_rol_tolerance->value();
                    bool use_recip = ui->check_use_recip_corre->isChecked();
                    auto future = QtConcurrent::run([ctx, k, max, tra_tolerance, rol_tolerance, use_recip]() {
                        return ct::Registration::GeneralizedIterativeClosestPoint(ctx, k, max, tra_tolerance, rol_tolerance, use_recip);
                    });
                    m_reg_watcher->setFuture(future);
                }
                    break;
                case REG_SampleConsensusInitialAlignment:
                    if (ui->cbox_feature2->currentIndex() > 0 && m_descriptor == nullptr)
                    {
                        printW("Please estimate target and source cloud descriptor first!");
                        m_cloudtree->closeProgress();
                        return;
                    }
                    m_cloudview->showInfo("Registration: SampleConsensusInitialAlignment", 3);
                    switch (ui->cbox_feature2->currentIndex())
                    {
                        case REG_TYPE_PointCloud:
                            printW("Please estimate target and source cloud Feature first!");
                            m_cloudtree->closeProgress();
                            return;
                        case REG_TYPE_PFHFeature:
                            m_cloudview->showInfo("Estimation Type: PFHFeature", 4);
                            if (m_descriptor->getDescriptor(m_source_cloud->id())->pfh && m_descriptor->getDescriptor(m_target_cloud->id())->pfh)
                            {
                                auto ctx_copy = ctx;
                                auto src_pfh = m_descriptor->getDescriptor(m_source_cloud->id())->pfh;
                                auto tgt_pfh = m_descriptor->getDescriptor(m_target_cloud->id())->pfh;
                                float min_sample_distance = ui->dspin_min_sample_distance->value();
                                int nr_samples = ui->spin_nr_samples->value();
                                int k = ui->spin_k2->value();
                                auto future = QtConcurrent::run([ctx_copy, src_pfh, tgt_pfh, min_sample_distance, nr_samples, k]() {
                                    return ct::Registration::SampleConsensusInitialAlignment<pcl::PFHSignature125>(
                                        ctx_copy, src_pfh, tgt_pfh, min_sample_distance, nr_samples, k);
                                });
                                m_reg_watcher->setFuture(future);
                            }
                            else
                            {
                                printW("Please estimate target and source cloud PFHFeature first!");
                                m_cloudtree->closeProgress();
                                return;
                            }

                            break;
                        case REG_TYPE_FPFHFeature:
                            m_cloudview->showInfo("Estimation Type: FPFHFeature", 4);
                            if (m_descriptor->getDescriptor(m_source_cloud->id())->fpfh && m_descriptor->getDescriptor(m_target_cloud->id())->fpfh)
                            {
                                auto ctx_copy = ctx;
                                auto src_fpfh = m_descriptor->getDescriptor(m_source_cloud->id())->fpfh;
                                auto tgt_fpfh = m_descriptor->getDescriptor(m_target_cloud->id())->fpfh;
                                float min_sample_distance = ui->dspin_min_sample_distance->value();
                                int nr_samples = ui->spin_nr_samples->value();
                                int k = ui->spin_k2->value();
                                auto future = QtConcurrent::run([ctx_copy, src_fpfh, tgt_fpfh, min_sample_distance, nr_samples, k]() {
                                    return ct::Registration::SampleConsensusInitialAlignment<pcl::FPFHSignature33>(
                                        ctx_copy, src_fpfh, tgt_fpfh, min_sample_distance, nr_samples, k);
                                });
                                m_reg_watcher->setFuture(future);
                            }
                            else
                            {
                                printW("Please estimate target and source cloud FPFHFeature first!");
                                m_cloudtree->closeProgress();
                                return;
                            }
                            break;
                            //TODO: other feature
                    }
                    break;
                case REG_SampleConsensusPrerejective:
                    if (ui->cbox_feature2->currentIndex() > 0 && m_descriptor == nullptr)
                    {
                        printW("Please estimate target and source cloud descriptor first!");
                        m_cloudtree->closeProgress();
                        return;
                    }
                    m_cloudview->showInfo("Registration: SampleConsensusPrerejective", 3);
                    switch (ui->cbox_feature2->currentIndex())
                    {
                        case REG_TYPE_PointCloud:
                            printW("Please estimate target and source cloud Feature first!");
                            m_cloudtree->closeProgress();
                            return;
                        case REG_TYPE_PFHFeature:
                            m_cloudview->showInfo("Estimation Type: PFHFeature", 4);
                            if (m_descriptor->getDescriptor(m_source_cloud->id())->pfh && m_descriptor->getDescriptor(m_target_cloud->id())->pfh)
                            {
                                auto ctx_copy = ctx;
                                auto src_pfh = m_descriptor->getDescriptor(m_source_cloud->id())->pfh;
                                auto tgt_pfh = m_descriptor->getDescriptor(m_target_cloud->id())->pfh;
                                int nr_samples = ui->spin_nr_samples_2->value();
                                int k = ui->spin_k->value();
                                float similarity_threshold = ui->dspin_similarity_threshold->value();
                                float inlier_fraction = ui->dspin_inlier_fraction->value();
                                auto future = QtConcurrent::run([ctx_copy, src_pfh, tgt_pfh, nr_samples, k, similarity_threshold, inlier_fraction]() {
                                    return ct::Registration::SampleConsensusPrerejective<pcl::PFHSignature125>(
                                        ctx_copy, src_pfh, tgt_pfh, nr_samples, k, similarity_threshold, inlier_fraction);
                                });
                                m_reg_watcher->setFuture(future);
                            }
                            else
                            {
                                printW("Please estimate target and source cloud PFHFeature first!");
                                m_cloudtree->closeProgress();
                                return;
                            }

                            break;
                        case REG_TYPE_FPFHFeature:
                            m_cloudview->showInfo("Estimation Type: FPFHFeature", 4);
                            if (m_descriptor->getDescriptor(m_source_cloud->id())->fpfh && m_descriptor->getDescriptor(m_target_cloud->id())->fpfh)
                            {
                                auto ctx_copy = ctx;
                                auto src_fpfh = m_descriptor->getDescriptor(m_source_cloud->id())->fpfh;
                                auto tgt_fpfh = m_descriptor->getDescriptor(m_target_cloud->id())->fpfh;
                                int nr_samples = ui->spin_nr_samples_2->value();
                                int k = ui->spin_k->value();
                                float similarity_threshold = ui->dspin_similarity_threshold->value();
                                float inlier_fraction = ui->dspin_inlier_fraction->value();
                                auto future = QtConcurrent::run([ctx_copy, src_fpfh, tgt_fpfh, nr_samples, k, similarity_threshold, inlier_fraction]() {
                                    return ct::Registration::SampleConsensusPrerejective<pcl::FPFHSignature33>(
                                        ctx_copy, src_fpfh, tgt_fpfh, nr_samples, k, similarity_threshold, inlier_fraction);
                                });
                                m_reg_watcher->setFuture(future);
                            }
                            else
                            {
                                printW("Please estimate target and source cloud FPFHFeature first!");
                                m_cloudtree->closeProgress();
                                return;
                            }
                            break;
                            //TODO: other feature
                    }
                    break;
                case REG_NormalDistributionsTransform:
                    m_cloudview->showInfo("Registration: NormalDistributionsTransform", 3);
                {
                    float resolution = ui->dspin_resolution->value();
                    double step_size = ui->dspin_step_size->value();
                    double outlier_ratio = ui->dspin_outlier_ratio->value();
                    auto future = QtConcurrent::run([ctx, resolution, step_size, outlier_ratio]() {
                        return ct::Registration::NormalDistributionsTransform(ctx, resolution, step_size, outlier_ratio);
                    });
                    m_reg_watcher->setFuture(future);
                }
                    break;
                case REG_FPCSInitialAlignment:
                    m_cloudview->showInfo("Registration: FPCSInitialAlignment", 3);
                {
                    float delta = ui->dspin_delta->value();
                    bool normalize = ui->check_normalize->isChecked();
                    float approx_overlap = ui->dspin_approx_overlap->value();
                    float score_threshold = ui->dspin_score_threshold->value();
                    int nr_samples = ui->spin_nr_samples_3->value();
                    float max_norm_diff = ui->dspin_max_norm_diff->value();
                    int max_runtime = ui->dspin_max_runtime->value();
                    auto future = QtConcurrent::run([ctx, delta, normalize, approx_overlap, score_threshold, nr_samples, max_norm_diff, max_runtime]() {
                        return ct::Registration::FPCSInitialAlignment(ctx, delta, normalize, approx_overlap, score_threshold, nr_samples, max_norm_diff, max_runtime);
                    });
                    m_reg_watcher->setFuture(future);
                }
                    break;
                case REG_KFPCSInitialAlignment:
                    m_cloudview->showInfo("Registration: KFPCSInitialAlignment", 3);
                {
                    float delta = ui->dspin_delta->value();
                    bool normalize = ui->check_normalize_2->isChecked();
                    float approx_overlap = ui->dspin_approx_overlap_2->value();
                    float score_threshold = ui->dspin_score_threshold_2->value();
                    int nr_samples = ui->spin_nr_samples_4->value();
                    float max_norm_diff = ui->dspin_max_norm_diff->value();
                    int max_runtime = ui->dspin_max_runtime->value();
                    float upper_trl_boundary = ui->dspin_upper_trl_boundary->value();
                    float lower_trl_boundary = ui->dspin_lower_trl_boundary->value();
                    float lambda = ui->dspin_lambda->value();
                    auto future = QtConcurrent::run([ctx, delta, normalize, approx_overlap, score_threshold, nr_samples, max_norm_diff, max_runtime, upper_trl_boundary, lower_trl_boundary, lambda]() {
                        return ct::Registration::KFPCSInitialAlignment(ctx, delta, normalize, approx_overlap, score_threshold, nr_samples, max_norm_diff, max_runtime, upper_trl_boundary, lower_trl_boundary, lambda);
                    });
                    m_reg_watcher->setFuture(future);
                }
                    break;
            }
        }
            break;
    }
}

void Registration::add()
{
    if (!m_source_cloud || !m_target_cloud)
    {
        printW("Please select source and target cloud first!");
        return;
    }
    if (m_reg_map.find(m_source_cloud->id() + m_target_cloud->id()) == m_reg_map.end())
    {
        printW(QString("Please registrate target cloud[id:%1] and source cloud[id:%2] first!").arg(QString::fromStdString(m_target_cloud->id())).arg(QString::fromStdString(m_source_cloud->id())));
        return;
    }
    // 将源点云和目标点云的 ID 连接起来，形成一个新的字符串。
    ct::Cloud::Ptr new_cloud = m_reg_map.find(m_source_cloud->id() + m_target_cloud->id())->second;
    m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
    new_cloud->setId(REG_ALIGN_ADD_FLAG + m_source_cloud->id());
    QTreeWidgetItem *item = m_cloudtree->getItemById(QString::fromStdString(m_source_cloud->id()));
    // 策略一：配准结果作为兄弟节点挂载
    m_cloudtree->insertCloud(new_cloud, item, true, ct::MountStrategy::Sibling);

    m_reg_map.erase(m_source_cloud->id() + m_target_cloud->id());
    printI(QString("Add registrated cloud[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    m_cloudview->clearInfo();
}

void Registration::apply()
{
    if (!m_source_cloud || !m_target_cloud)
    {
        printW("Please select source and target cloud first!");
        return;
    }
    if (m_reg_map.find(m_source_cloud->id() + m_target_cloud->id()) == m_reg_map.end())
    {
        printW(QString("Please registrate target cloud[id:%1] and source cloud[id:%2] first!").arg(QString::fromStdString(m_target_cloud->id())).arg(QString::fromStdString(m_source_cloud->id())));
        return;
    }
    ct::Cloud::Ptr new_cloud = m_reg_map.find(m_source_cloud->id() + m_target_cloud->id())->second;
    m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
    m_cloudtree->updateCloud(m_source_cloud, new_cloud);
    m_reg_map.erase(m_source_cloud->id() + m_target_cloud->id());
    printI(QString("Apply registrated cloud[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    m_cloudview->clearInfo();
}

void Registration::reset()
{
    m_cloudtree->setEnabled(true);
    ui->btn_setTarget->setEnabled(true);
    ui->btn_setSource->setEnabled(true);
    m_cloudview->clearInfo();
    if (m_target_cloud) m_cloudview->resetPointCloudColor(m_target_cloud);
    if (m_source_cloud) m_cloudview->resetPointCloudColor(m_source_cloud);
    if (m_source_cloud && m_target_cloud)
    {
        m_cloudview->removeCorrespondences(QString::fromStdString(m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG));
        m_cloudview->removePointCloud(QString::fromStdString(m_source_cloud->id() + REG_TRANS_PRE_FLAG));
        m_cloudview->removePointCloud(QString::fromStdString(m_source_cloud->id() + REG_ALIGN_PRE_FLAG));
    }
    m_target_cloud = nullptr;
    m_source_cloud = nullptr;
    m_corr = nullptr;
    m_ce = nullptr;
    m_te = nullptr;
    for (auto& cloud : m_reg_map)
        m_cloudview->removePointCloud(QString::fromStdString(cloud.second->id()));
    m_reg_map.clear();
}

void Registration::handleCorrespondenceResult(const ct::CorrespondenceResult& result)
{
    m_cloudtree->closeProgress();
    printI(QString("Estimate target cloud[id:%1] and source cloud[id:%2] correspondences done, take time %3 ms.").arg(QString::fromStdString(m_target_cloud->id())).arg(QString::fromStdString(m_source_cloud->id())).arg(result.time_ms));
    QString id = QString::fromStdString(m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG);
    m_cloudview->addCorrespondences(m_source_cloud, m_target_cloud, result.correspondences, id);
    m_corr = result.correspondences;
    m_ce = result.ce;
}

void Registration::handleRejectorResult(const ct::RejectorResult& result)
{
    m_cloudtree->closeProgress();
    printI(QString("Reject target cloud[id:%1] and source cloud[id:%2] correspondences done, take time %3 ms.").arg(QString::fromStdString(m_target_cloud->id())).arg(QString::fromStdString(m_source_cloud->id())).arg(result.time_ms));
    QString id = QString::fromStdString(m_target_cloud->id() + m_source_cloud->id() + REG_CORRE_PRE_FLAG);
    m_cloudview->addCorrespondences(m_source_cloud, m_target_cloud, result.correspondences, id);
    m_cr_map[ui->cbox_correRej->currentIndex()] = result.cr;
}

void Registration::handleTransformationResult(const ct::TransformationResult& result)
{
    m_cloudtree->closeProgress();
    printI(QString("Estimate target cloud[id:%1] and source cloud[id:%2] transformation done, take time %3 ms.").arg(QString::fromStdString(m_target_cloud->id())).arg(QString::fromStdString(m_source_cloud->id())).arg(result.time_ms));

    // 使用PCL transformPointCloud进行变换
    pcl::PointCloud<ct::PointXYZRGBN>::Ptr pcl_transformed(new pcl::PointCloud<ct::PointXYZRGBN>);
    pcl::transformPointCloud(*m_source_cloud->toPCL_XYZRGBN(), *pcl_transformed, result.matrix);

    ct::Cloud::Ptr cloud = ct::Cloud::fromPCL_XYZRGBN(*pcl_transformed);
    cloud->setId(m_source_cloud->id() + REG_TRANS_PRE_FLAG);
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudSize(QString::fromStdString(cloud->id()), cloud->pointSize() + 2);
    m_cloudview->setPointCloudColor(QString::fromStdString(cloud->id()), ct::Color::Blue);
    m_te = result.te;
}

void Registration::handleRegistrationResult(const ct::RegistrationResult& result)
{
    m_cloudtree->closeProgress();
    if (!result.success)
    {
        printE(QString("Registrate target cloud[id:%1] and source cloud[id:%2] failed!").arg(QString::fromStdString(m_target_cloud->id())).arg(QString::fromStdString(m_source_cloud->id())));
        return;
    }
    printI(QString("Registrate target cloud[id:%1] and source cloud[id:%2] done, take time %3 ms.").arg(QString::fromStdString(m_target_cloud->id())).arg(QString::fromStdString(m_source_cloud->id())).arg(result.time_ms));
    result.aligned_cloud->setId(m_source_cloud->id() + REG_ALIGN_PRE_FLAG);
    m_cloudview->addPointCloud(result.aligned_cloud);
    m_cloudview->setPointCloudSize(QString::fromStdString(result.aligned_cloud->id()), result.aligned_cloud->pointSize() + 2);
    m_cloudview->setPointCloudColor(QString::fromStdString(result.aligned_cloud->id()), ct::Color::Blue);
    ui->txt_matrix->clear();
    ui->txt_matrix->append(tr("Fitness Score: %1 ").arg(result.score));
    ui->txt_matrix->append(tr("Transformation Matrix:"));
    ui->txt_matrix->append(QString::fromStdString(ct::getTransformationString(result.matrix, 6)));
    m_reg_map[m_source_cloud->id() + m_target_cloud->id()] = result.aligned_cloud;
}
