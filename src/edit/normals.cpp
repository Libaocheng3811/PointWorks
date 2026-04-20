#include "normals.h"
#include "ui_normals.h"

Normals::Normals(QWidget* parent)
    : CustomDialog(parent), ui(new Ui::Normals)
{
    ui->setupUi(this);

    connect(ui->btn_preview, &QPushButton::clicked, this, &Normals::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Normals::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Normals::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Normals::reset);

    // ViewPoint 互斥
    connect(ui->check_max, &QCheckBox::clicked, [=](bool state)
    {
        if (state) { ui->check_center->setChecked(false); ui->check_origin->setChecked(false); }
    });
    connect(ui->check_center, &QCheckBox::clicked, [=](bool state)
    {
        if (state) { ui->check_max->setChecked(false); ui->check_origin->setChecked(false); }
    });
    connect(ui->check_origin, &QCheckBox::clicked, [=](bool state)
    {
        if (state) { ui->check_max->setChecked(false); ui->check_center->setChecked(false); }
    });

    // 实时刷新
    connect(ui->spin_level, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=](int)
    {
        if (ui->check_refresh->isChecked()) updateNormals();
    });
    connect(ui->dspin_scale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
    {
        if (ui->check_refresh->isChecked()) updateNormals();
    });

    connect(ui->check_reverse, &QCheckBox::stateChanged, this, &Normals::reverseNormals);
}

Normals::~Normals()
{
    m_cancel = true;
    if (m_watcher && !m_watcher->isFinished()) {
        m_watcher->waitForFinished();
    }
    delete ui;
}

void Normals::runNormals(const std::string& source_id, const ct::Cloud::Ptr& cloud,
                         float vpx, float vpy, float vpz)
{
    m_progress->showProgress("Normals Estimation...");

    m_cancel = false;
    if (m_progress->dialog()) {
        connect(m_progress, &ct::ProgressManager::cancelRequested,
                this, [this]() { m_cancel = true; }, Qt::UniqueConnection);
    }

    auto progress_cb = [this](int value) {
        QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, value));
    };

    int k = ui->spin_k->value();
    double r = ui->dspin_r->value();
    bool reverse = ui->check_reverse->isChecked();

    auto future = QtConcurrent::run([this, cloud, k, r, vpx, vpy, vpz, reverse, progress_cb]() -> ct::NormalsResult {
        return ct::Normals::estimate(cloud, k, r, vpx, vpy, vpz, reverse, &m_cancel, progress_cb);
    });

    if (!m_watcher) {
        m_watcher = new QFutureWatcher<ct::NormalsResult>(this);
    }

    connect(m_watcher, &QFutureWatcher<ct::NormalsResult>::finished, this,
        [this, source_id]() {
            m_progress->closeProgress();
            handleNormalsResult(source_id);
        }, Qt::UniqueConnection);

    m_watcher->setFuture(future);
}

void Normals::handleNormalsResult(const std::string& source_id)
{
    auto result = m_watcher->result();

    if (!result.cloud)
    {
        printW(QString("Normals cloud[id:%1] failed: %2")
                   .arg(QString::fromStdString(source_id))
                   .arg(QString::fromStdString(result.error_msg)));
        return;
    }

    printI(QString("Estimate cloud[id:%1] normals done, take time %2 ms.")
               .arg(QString::fromStdString(source_id)).arg(result.time_ms));

    m_cloudview->addPointCloudNormals(result.cloud, ui->spin_level->value(), ui->dspin_scale->value());
    m_normals_map[source_id] = result.cloud;
    m_cloudview->clearInfo();
}

void Normals::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (ui->spin_k->value() == 0 && ui->dspin_r->value() == 0)
    {
        printW("Parameters set error!");
        return;
    }

    for (auto& cloud : selected_clouds)
    {
        Eigen::Vector3f viewpoint;
        if (ui->check_center->isChecked())
            viewpoint = cloud->center();
        else if (ui->check_max->isChecked())
            viewpoint << std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max();
        else
            viewpoint << 0.f, 0.f, 0.f;

        runNormals(cloud->id(), cloud, viewpoint[0], viewpoint[1], viewpoint[2]);
    }
}

void Normals::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_normals_map.find(cloud->id()) == m_normals_map.end())
        {
            printW(QString("The cloud[id:%1] has no estimated normals !").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_normals_map[cloud->id()];
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->normalId()));
        m_cloudtree->addSiblingCloud(cloud, new_cloud, NORMALS_ADD_FLAG);
        m_normals_map.erase(cloud->id());
        printI(QString("Add cloud[id:%1] with estimated normals done.").arg(QString::fromStdString(cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Normals::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_normals_map.find(cloud->id()) == m_normals_map.end())
        {
            printW(QString("The cloud[id:%1] has no estimated normals !").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_normals_map[cloud->id()];
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->normalId()));
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_normals_map.erase(cloud->id());
        printI(QString("Apply cloud[id:%1] estimated normals done.").arg(QString::fromStdString(cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Normals::reset()
{
    for (auto& [id, cloud] : m_normals_map)
        m_cloudview->removePointCloud(QString::fromStdString(cloud->normalId()));
    m_normals_map.clear();
    m_cloudview->clearInfo();
}

void Normals::reverseNormals()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) return;
    for (auto& cloud : selected_clouds)
    {
        auto it = m_normals_map.find(cloud->id());
        if (it == m_normals_map.end()) continue;

        // 通过 PCL 转换翻转法线
        auto pcl_cloud = it->second->toPCL_XYZRGBN();
        for (auto& pt : pcl_cloud->points)
        {
            pt.normal_x = -pt.normal_x;
            pt.normal_y = -pt.normal_y;
            pt.normal_z = -pt.normal_z;
        }
        it->second = ct::Cloud::fromPCL_XYZRGBN(*pcl_cloud);
        it->second->setId(cloud->id());
        it->second->setFilepath(cloud->filepath());
        it->second->setBox(cloud->box());

        m_cloudview->addPointCloudNormals(it->second, ui->spin_level->value(), ui->dspin_scale->value());
    }
}

void Normals::updateNormals()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) return;
    for (auto& cloud : selected_clouds)
    {
        auto it = m_normals_map.find(cloud->id());
        if (it == m_normals_map.end()) continue;
        m_cloudview->addPointCloudNormals(it->second, ui->spin_level->value(), ui->dspin_scale->value());
    }
}
