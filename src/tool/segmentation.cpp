#include "segmentation.h"
#include "ui_segmentation.h"

#include <QMessageBox>

#define SEG_TYPE_SACSegmentation                (0)
#define SEG_TYPE_EuclideanClusterExtraction     (1)
#define SEG_TYPE_RegionGrowing                  (2)
#define SEG_TYPE_SupervoxelClustering           (3)
#define SEG_TYPE_MinCutSegmentation             (4)
#define SEG_TYPE_DonSegmentation                (5)
#define SEG_TYPE_MorphologicalFilter            (6)

Segmentation::Segmentation(QWidget* parent)
    : CustomDialog(parent), ui(new Ui::Segmentation)
{
    ui->setupUi(this);

    connect(ui->btn_preview, &QPushButton::clicked, this, &Segmentation::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Segmentation::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Segmentation::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Segmentation::reset);

    // SACSegmentation
    connect(ui->check_fromNormals, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    ui->label_5->show(), ui->dspin_distanceWeight->show();
                    ui->label_37->show(), ui->dspin_distanceFromOrigin->show();
                }
                else
                {
                    ui->label_5->hide(), ui->dspin_distanceWeight->hide();
                    ui->label_37->hide(), ui->dspin_distanceFromOrigin->hide();
                }
            });
    connect(ui->cbox_modelType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int i)
            {
                if (i == 2 || i == 3 || i == 4 || i == 5 || i == 6 || i == 7 || i == 12)
                    ui->label_7->show(), ui->dspin_minRadius->show(), ui->dspin_maxRadius->show();
                else
                    ui->label_7->hide(), ui->dspin_minRadius->hide(), ui->dspin_maxRadius->hide();
            });

    ui->check_fromNormals->setChecked(false);
    ui->label_5->hide(), ui->dspin_distanceWeight->hide();
    ui->label_37->hide(), ui->dspin_distanceFromOrigin->hide();
    ui->label_7->hide(), ui->dspin_minRadius->hide(), ui->dspin_maxRadius->hide();

    // RegionGrowing
    connect(ui->check_smoothmode, &QCheckBox::clicked, [=](bool state)
            {
                ui->dspin_smooth->setEnabled(state);
            });
    connect(ui->check_curvaturetest, &QCheckBox::clicked, [=](bool state)
            {
                ui->dspin_curvature->setEnabled(state);
            });
    connect(ui->check_residualtest, &QCheckBox::clicked, [=](bool state)
            {
                ui->dspin_residual->setEnabled(state);
            });
    connect(ui->check_fromRGB, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    ui->label_14->show(), ui->dspin_pointcolor->show();
                    ui->label_17->show(), ui->dspin_regioncolor->show();
                    ui->label_15->show(), ui->dspin_distance->show();
                    ui->label_16->show(), ui->spin_nghbr_number->show();
                }
                else
                {
                    ui->label_14->hide(), ui->dspin_pointcolor->hide();
                    ui->label_17->hide(), ui->dspin_regioncolor->hide();
                    ui->label_15->hide(), ui->dspin_distance->hide();
                    ui->label_16->hide(), ui->spin_nghbr_number->hide();
                }
            });

    ui->check_fromRGB->setChecked(false);
    ui->label_14->hide(), ui->dspin_pointcolor->hide();
    ui->label_17->hide(), ui->dspin_regioncolor->hide();
    ui->label_15->hide(), ui->dspin_distance->hide();
    ui->label_16->hide(), ui->spin_nghbr_number->hide();
    ui->check_residualtest->setChecked(false);
    ui->dspin_residual->setEnabled(false);

    ui->cbox_segmentations->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

Segmentation::~Segmentation()
{
    m_cancel = true;
    if (m_watcher && !m_watcher->isFinished()) {
        m_watcher->waitForFinished();
    }
    delete ui;
}

void Segmentation::runSegmentation(const std::string& source_id, std::function<ct::SegmentationResult()> fn)
{
    m_cloudtree->showProgress("Segmenting...");

    m_cancel = false;
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [this]() { m_cancel = true; }, Qt::UniqueConnection);
    }

    auto future = QtConcurrent::run([fn = std::move(fn), this]() -> ct::SegmentationResult {
        return fn();
    });

    if (!m_watcher) {
        m_watcher = new QFutureWatcher<ct::SegmentationResult>(this);
    }

    connect(m_watcher, &QFutureWatcher<ct::SegmentationResult>::finished, this,
        [this, source_id]() {
            m_cloudtree->closeProgress();
            auto result = m_watcher->result();
            handleSegmentationResult(source_id, result);
        }, Qt::UniqueConnection);

    m_watcher->setFuture(future);
}

void Segmentation::handleSegmentationResult(const std::string& source_id, const ct::SegmentationResult& result)
{
    if (result.clouds.empty()) return;

    printI(QString("Segmented cloud[id:%1] to %2 cloud(s) done, take time %3 ms.")
               .arg(QString::fromStdString(source_id))
               .arg(result.clouds.size())
               .arg(result.time_ms));

    for (size_t i = 0; i < result.clouds.size(); i++)
    {
        if (result.clouds[i]->empty()) continue;
        result.clouds[i]->setId(source_id + SEGMENTATION_PRE_FLAG + std::to_string(i));
        m_cloudview->addPointCloud(result.clouds[i]);
        m_cloudview->setPointCloudColor(QString::fromStdString(result.clouds[i]->id()),
                                         {static_cast<uint8_t>(rand() % 256),
                                          static_cast<uint8_t>(rand() % 256),
                                          static_cast<uint8_t>(rand() % 256)});
        m_cloudview->setPointCloudSize(QString::fromStdString(result.clouds[i]->id()),
                                        result.clouds[i]->pointSize() + 2);
    }
    m_segmentation_map[source_id] = result.clouds;
}

void Segmentation::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }

    // 先清除之前该云的预览结果
    for (auto& cloud : selected_clouds)
    {
        auto it = m_segmentation_map.find(cloud->id());
        if (it != m_segmentation_map.end())
        {
            for (auto& c : it->second)
                m_cloudview->removePointCloud(QString::fromStdString(c->id()));
            m_segmentation_map.erase(it);
        }
    }

    for (auto& cloud : selected_clouds)
    {
        bool negative = ui->check_negative->isChecked();
        std::string cloud_id = cloud->id();

        switch (ui->cbox_segmentations->currentIndex())
        {
        case SEG_TYPE_SACSegmentation:
        {
            m_cloudview->showInfo("SACSegmentation", 1);
            if (ui->check_fromNormals->isChecked())
            {
                if (!cloud->hasNormals())
                {
                    printW("Please estimate normals first!");
                    return;
                }
                int model = ui->cbox_modelType->currentIndex();
                int method = ui->cbox_methodType->currentIndex();
                double threshold = ui->dspin_Threshold->value();
                int iterations = ui->spin_Iterations->value();
                double probability = ui->dspin_probability->value();
                bool optimize = ui->check_optimize->isChecked();
                double minRadius = ui->dspin_minRadius->value();
                double maxRadius = ui->dspin_maxRadius->value();
                double distanceWeight = ui->dspin_distanceWeight->value();
                double distanceFromOrigin = ui->dspin_distanceFromOrigin->value();
                runSegmentation(cloud_id, [cloud, negative, model, method, threshold, iterations,
                                 probability, optimize, minRadius, maxRadius,
                                 distanceWeight, distanceFromOrigin, this]() {
                    auto result = ct::Segmentation::SACSegmentationFromNormals(cloud, negative,
                        model, method, threshold, iterations, probability, optimize,
                        minRadius, maxRadius, distanceWeight, distanceFromOrigin, &m_cancel);
                    return result;
                });
            }
            else
            {
                int model = ui->cbox_modelType->currentIndex();
                int method = ui->cbox_methodType->currentIndex();
                double threshold = ui->dspin_Threshold->value();
                int iterations = ui->spin_Iterations->value();
                double probability = ui->dspin_probability->value();
                bool optimize = ui->check_optimize->isChecked();
                double minRadius = ui->dspin_minRadius->value();
                double maxRadius = ui->dspin_maxRadius->value();
                runSegmentation(cloud_id, [cloud, negative, model, method, threshold, iterations,
                                 probability, optimize, minRadius, maxRadius, this]() {
                    auto result = ct::Segmentation::SACSegmentation(cloud, negative,
                        model, method, threshold, iterations, probability, optimize,
                        minRadius, maxRadius, &m_cancel);
                    return result;
                });
            }
            break;
        }
        case SEG_TYPE_EuclideanClusterExtraction:
        {
            m_cloudview->showInfo("EuclideanClusterExtraction", 1);
            double tolerance = ui->dspin_tolerance->value();
            int minClusterSize = ui->spin_min_cluster_size->value();
            int maxClusterSize = ui->spin_max_cluster_size->value();
            runSegmentation(cloud_id, [cloud, negative, tolerance, minClusterSize, maxClusterSize, this]() {
                return ct::Segmentation::EuclideanClusterExtraction(cloud, negative,
                    tolerance, minClusterSize, maxClusterSize, &m_cancel);
            });
            break;
        }
        case SEG_TYPE_RegionGrowing:
        {
            m_cloudview->showInfo("RegionGrowing", 1);
            if (!cloud->hasNormals())
            {
                printW("Please estimate normals first!");
                return;
            }
            int minClusterSize = ui->spin_minclustersize->value();
            int maxClusterSize = ui->spin_maxclustersize->value();
            bool smoothMode = ui->check_smoothmode->isChecked();
            bool curvatureTest = ui->check_curvaturetest->isChecked();
            bool residualTest = ui->check_residualtest->isChecked();
            float smooth = ui->dspin_smooth->value();
            float residual = ui->dspin_residual->value();
            float curvature = ui->dspin_curvature->value();
            int neighbours = ui->spin_numofnei->value();

            if (ui->check_fromRGB->isChecked())
            {
                if (cloud->type() == CLOUD_TYPE_XYZ || cloud->type() == CLOUD_TYPE_XYZN)
                {
                    printW("The cloud type does not support !");
                    return;
                }
                float ptThresh = ui->dspin_pointcolor->value();
                float reThresh = ui->dspin_regioncolor->value();
                float disThresh = ui->dspin_distance->value();
                int nghbrNumber = ui->spin_nghbr_number->value();
                runSegmentation(cloud_id, [cloud, negative, minClusterSize, maxClusterSize, smoothMode,
                                 curvatureTest, residualTest, smooth, residual, curvature,
                                 neighbours, ptThresh, reThresh, disThresh, nghbrNumber, this]() {
                    return ct::Segmentation::RegionGrowingRGB(cloud, negative,
                        minClusterSize, maxClusterSize, smoothMode, curvatureTest, residualTest,
                        smooth, residual, curvature, neighbours,
                        ptThresh, reThresh, disThresh, nghbrNumber, &m_cancel);
                });
            }
            else
            {
                runSegmentation(cloud_id, [cloud, negative, minClusterSize, maxClusterSize, smoothMode,
                                 curvatureTest, residualTest, smooth, residual, curvature,
                                 neighbours, this]() {
                    return ct::Segmentation::RegionGrowing(cloud, negative,
                        minClusterSize, maxClusterSize, smoothMode, curvatureTest, residualTest,
                        smooth, residual, curvature, neighbours, &m_cancel);
                });
            }
            break;
        }
        case SEG_TYPE_SupervoxelClustering:
        {
            m_cloudview->showInfo("SupervoxelClustering", 1);
            float voxelRes = ui->dspin_voxelresolution->value();
            float seedRes = ui->dspin_seedresolution->value();
            float colorImp = ui->dspin_colorimportance->value();
            float spatialImp = ui->dspin_spatialmportance->value();
            float normalImp = ui->dspin_normallmportance->value();
            bool cameraTransform = ui->check_transform->isChecked();
            runSegmentation(cloud_id, [cloud, voxelRes, seedRes, colorImp, spatialImp, normalImp,
                             cameraTransform, this]() {
                return ct::Segmentation::SupervoxelClustering(cloud,
                    voxelRes, seedRes, colorImp, spatialImp, normalImp, cameraTransform, &m_cancel);
            });
            break;
        }
        case SEG_TYPE_MinCutSegmentation:
        {
            m_cloudview->showInfo("MinCutSegmentation", 1);
            double sigma = ui->dspin_sigma->value();
            double radius = ui->dspin_radius->value();
            double weight = ui->dspin_weight->value();
            int neighbourNumber = ui->spin_neighbour_number->value();
            runSegmentation(cloud_id, [cloud, sigma, radius, weight, neighbourNumber, this]() {
                return ct::Segmentation::MinCutSegmentation(cloud,
                    sigma, radius, weight, neighbourNumber, &m_cancel);
            });
            break;
        }
        case SEG_TYPE_DonSegmentation:
        {
            m_cloudview->showInfo("DonSegmentation", 1);
            double meanRadius = cloud->resolution();
            double scale1 = ui->dspin_smallscale->value();
            double scale2 = ui->dspin_largescale->value();
            double threshold = ui->dspin_threshold->value();
            double segRadius = ui->dspin_segradius->value();
            int minClusterSize = ui->spin_mincluster->value();
            int maxClusterSize = ui->spin_maxcluster->value();
            runSegmentation(cloud_id, [cloud, negative, meanRadius, scale1, scale2, threshold,
                             segRadius, minClusterSize, maxClusterSize, this]() {
                return ct::Segmentation::DonSegmentation(cloud, negative,
                    meanRadius, scale1, scale2, threshold, segRadius,
                    minClusterSize, maxClusterSize, &m_cancel);
            });
            break;
        }
        case SEG_TYPE_MorphologicalFilter:
        {
            m_cloudview->showInfo("MorphologicalFilter", 1);
            int maxWinSize = ui->spin_maxwinsize->value();
            float slope = ui->dspin_slope->value();
            float maxDist = ui->dspin_maxdistance->value();
            float initDist = ui->dspin_initialdistance->value();
            float cellSize = ui->dspin_cellsize->value();
            float base = ui->dspin_base->value();
            runSegmentation(cloud_id, [cloud, negative, maxWinSize, slope, maxDist, initDist,
                             cellSize, base, this]() {
                return ct::Segmentation::MorphologicalFilter(cloud, negative,
                    maxWinSize, slope, maxDist, initDist, cellSize, base, &m_cancel);
            });
            break;
        }
        }
    }
}

void Segmentation::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_segmentation_map.find(cloud->id()) == m_segmentation_map.end())
        {
            printW(QString("The cloud[id:%1] has no segmented clouds!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        std::vector<ct::Cloud::Ptr> new_clouds = m_segmentation_map.find(cloud->id())->second;
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        for (size_t i = 0; i < new_clouds.size(); i++)
        {
            m_cloudview->removePointCloud(QString::fromStdString(new_clouds[i]->id()));
            new_clouds[i]->setId(SEGMENTATION_ADD_FLAG + std::to_string(i) + "-" + cloud->id());
            m_cloudtree->insertCloud(new_clouds[i], item, true, ct::MountStrategy::Sibling);
            printI(QString("Add segmented cloud[id:%1] done.").arg(QString::fromStdString(new_clouds[i]->id())));
        }
        m_segmentation_map.erase(cloud->id());
    }
    m_cloudview->clearInfo();
}

void Segmentation::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_segmentation_map.find(cloud->id()) == m_segmentation_map.end())
        {
            printW(QString("The cloud[id:%1] has no segmented cloud!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        std::vector<ct::Cloud::Ptr> new_clouds = m_segmentation_map.find(cloud->id())->second;
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        for (size_t i = 0; i < new_clouds.size(); i++)
        {
            m_cloudview->removePointCloud(QString::fromStdString(new_clouds[i]->id()));
            new_clouds[i]->setId(SEGMENTATION_ADD_FLAG + std::to_string(i) + "-" + cloud->id());
            if (i == 0)
            {
                m_cloudtree->updateCloud(cloud, new_clouds[i], true);
                printI(QString("Apply segmented cloud[id:%1] done.").arg(QString::fromStdString(new_clouds[i]->id())));
            }
            else
            {
                m_cloudtree->insertCloud(new_clouds[i], item, true, ct::MountStrategy::Sibling);
                printI(QString("Add segmented cloud[id:%1] done.").arg(QString::fromStdString(new_clouds[i]->id())));
            }
        }
        m_segmentation_map.erase(cloud->id());
    }
    m_cloudview->clearInfo();
}

void Segmentation::reset()
{
    for (auto& clouds : m_segmentation_map)
        for (auto& cloud : clouds.second)
            m_cloudview->removePointCloud(QString::fromStdString(cloud->id()));
    m_segmentation_map.clear();
    m_cloudview->clearInfo();
}
