#include "keypoints.h"
#include "ui_keypoints.h"

#include <QMessageBox>

#define KEYPOINTS_TYPE_NarfKeypoint         (0)
#define KEYPOINTS_TYPE_HarrisKeypoint3D     (1)
#define KEYPOINTS_TYPE_ISSKeypoint3D        (2)
#define KEYPOINTS_TYPE_SIFTKeypoint         (3)
#define KEYPOINTS_TYPE_TrajkovicKeypoint3D  (4)

#define KEYPOINTS_PRE_FLAG                 "-keypoints"
#define KEYPOINTS_ADD_FLAG                 "keypoints-"

KeyPoints::KeyPoints(QWidget* parent)
    : CustomDialog(parent), ui(new Ui::KeyPoints),
    m_rangeimage(nullptr)
{
    ui->setupUi(this);

    connect(ui->btn_preview, &QPushButton::clicked, this, &KeyPoints::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &KeyPoints::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &KeyPoints::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &KeyPoints::reset);

    ui->cbox_type->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

KeyPoints::~KeyPoints()
{
    m_cancel = true;
    if (m_watcher && !m_watcher->isFinished()) {
        m_watcher->waitForFinished();
    }
    delete ui;
}

void KeyPoints::runKeypoint(std::function<ct::KeypointResult()> fn)
{
    m_cloudtree->showProgress("Estimating keypoints...");

    m_cancel = false;
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [this]() { m_cancel = true; }, Qt::UniqueConnection);
    }

    auto future = QtConcurrent::run([fn = std::move(fn), this]() -> ct::KeypointResult {
        return fn();
    });

    if (!m_watcher) {
        m_watcher = new QFutureWatcher<ct::KeypointResult>(this);
    }

    connect(m_watcher, &QFutureWatcher<ct::KeypointResult>::finished, this, [this]() {
        m_cloudtree->closeProgress();
        auto result = m_watcher->result();
        handleKeypointResult(result);
    }, Qt::UniqueConnection);

    m_watcher->setFuture(future);
}

void KeyPoints::handleKeypointResult(const ct::KeypointResult& result)
{
    if (!result.cloud) return;

    auto cloud = result.cloud;

    printI(QString("Estimate cloud[id:%1] keypoints done, take time %2 ms.")
               .arg(QString::fromStdString(cloud->id()))
               .arg(result.time_ms));

    std::string id = cloud->id();
    cloud->setId(id + KEYPOINTS_PRE_FLAG);

    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudColor(QString::fromStdString(cloud->id()), ct::Color::Green);
    m_cloudview->setPointCloudSize(QString::fromStdString(cloud->id()), cloud->pointSize() + 2);
    m_keypoints_map[id] = cloud;
}

void KeyPoints::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        int k = ui->spin_k->value();
        double r = ui->dspin_r->value();
        if (k == 0 && r == 0)
        {
            printW("Parameters set error!");
            return;
        }

        switch (ui->cbox_type->currentIndex())
        {
        case KEYPOINTS_TYPE_NarfKeypoint:
        {
            if (m_rangeimage == nullptr)
            {
                printW("Please get a range image first!");
                return;
            }
            m_cloudview->showInfo("NarfKeypoint", 1);
            float support_size = ui->dspin_support_size->value();
            auto range_img = m_rangeimage->getRangeImage();
            runKeypoint([cloud, range_img, support_size, this]() {
                return ct::Keypoints::NarfKeypoint(cloud, range_img, support_size, &m_cancel);
            });
            break;
        }
        case KEYPOINTS_TYPE_HarrisKeypoint3D:
        {
            m_cloudview->showInfo("HarrisKeypoint3D", 1);
            int response_method = ui->cbox_response_method->currentIndex();
            float threshold = ui->dspin_threshold->value();
            bool non_maxima = ui->check_non_maxima->isChecked();
            bool do_refine = ui->check_do_refine->isChecked();
            runKeypoint([=]() {
                return ct::Keypoints::HarrisKeypoint3D(cloud, response_method, threshold,
                                                    non_maxima, do_refine, k, r, &m_cancel);
            });
            break;
        }
        case KEYPOINTS_TYPE_ISSKeypoint3D:
        {
            m_cloudview->showInfo("ISSKeypoint3D", 1);
            double resolution = cloud->resolution();
            double gamma_21 = ui->dspin_gamma_21->value();
            double gamma_32 = ui->dspin_gamma_32->value();
            int min_neighbors = ui->spin_min_neighbors->value();
            float angle = ui->dspin_angle->value();
            runKeypoint([=]() {
                return ct::Keypoints::ISSKeypoint3D(cloud, resolution, gamma_21, gamma_32,
                                                     min_neighbors, angle, k, r, &m_cancel);
            });
            break;
        }
        case KEYPOINTS_TYPE_SIFTKeypoint:
        {
            m_cloudview->showInfo("SIFTKeypoint", 1);
            float min_scale = ui->dspin_min_scale->value();
            int nr_octaves = ui->spin_nr_octaves->value();
            int nr_scales_per_octave = ui->spin_nr_scales_per_octave->value();
            float min_contrast = ui->dspin_min_contrast->value();
            runKeypoint([=]() {
                return ct::Keypoints::SIFTKeypoint(cloud, min_scale, nr_octaves,
                                                    nr_scales_per_octave, min_contrast, k, r, &m_cancel);
            });
            break;
        }
        case KEYPOINTS_TYPE_TrajkovicKeypoint3D:
        {
            m_cloudview->showInfo("TrajkovicKeypoint3D", 1);
            int compute_method = ui->cbox_compute_method->currentIndex();
            int window_size = ui->spin_window_size->value();
            float first_threshold = ui->dspin_first_threshold->value();
            float second_threshold = ui->dspin_second_threshold->value();
            runKeypoint([=]() {
                return ct::Keypoints::TrajkovicKeypoint3D(cloud, compute_method, window_size,
                                                           first_threshold, second_threshold, k, r, &m_cancel);
            });
            break;
        }
        }
    }
}

void KeyPoints::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_keypoints_map.find(cloud->id()) == m_keypoints_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched keypoints!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_keypoints_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
        new_cloud->setId(KEYPOINTS_ADD_FLAG + cloud->id());
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        m_cloudtree->insertCloud(new_cloud, item, true, ct::MountStrategy::Sibling);
        m_keypoints_map.erase(cloud->id());
        printI(QString("Add cloud keypoints[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void KeyPoints::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_keypoints_map.find(cloud->id()) == m_keypoints_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched keypoints!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_keypoints_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_keypoints_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply cloud keypoints[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void KeyPoints::reset()
{
    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);
    for (auto& cloud : m_keypoints_map)
        m_cloudview->removePointCloud(QString::fromStdString(cloud.second->id()));
    m_keypoints_map.clear();
    m_cloudview->clearInfo();
}
