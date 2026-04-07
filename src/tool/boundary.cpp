#include "boundary.h"
#include "ui_boundary.h"

Boundary::Boundary(QWidget* parent)
    : CustomDialog(parent), ui(new Ui::Boundary)
{
    ui->setupUi(this);

    connect(ui->btn_preview, &QPushButton::clicked, this, &Boundary::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Boundary::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Boundary::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Boundary::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Boundary::close);
}

Boundary::~Boundary()
{
    m_cancel = true;
    if (m_watcher && !m_watcher->isFinished()) {
        m_watcher->waitForFinished();
    }
    delete ui;
}

void Boundary::runBoundary(const std::string& source_id, std::function<ct::Cloud::Ptr(std::function<void(int)>)> fn)
{
    m_cloudtree->showProgress("Boundary...");

    m_cancel = false;
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [this]() { m_cancel = true; }, Qt::UniqueConnection);
    }

    auto progress_cb = [this](int value) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, value));
    };

    auto start_time = std::chrono::high_resolution_clock::now();

    auto future = QtConcurrent::run([fn = std::move(fn), progress_cb = std::move(progress_cb)]() -> ct::Cloud::Ptr {
        try {
            return fn(progress_cb);
        } catch (const std::exception& e) {
            return nullptr;
        } catch (...) {
            return nullptr;
        }
    });

    if (!m_watcher) {
        m_watcher = new QFutureWatcher<ct::Cloud::Ptr>(this);
    }

    connect(m_watcher, &QFutureWatcher<ct::Cloud::Ptr>::finished, this,
        [this, source_id, start_time]() {
            m_cloudtree->closeProgress();
            auto result = m_watcher->result();
            auto end_time = std::chrono::high_resolution_clock::now();
            float time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
            handleBoundaryResult(source_id, result, time_ms);
        }, Qt::UniqueConnection);

    m_watcher->setFuture(future);
}

void Boundary::handleBoundaryResult(const std::string& source_id, const ct::Cloud::Ptr& boundary_cloud, float time_ms)
{
    if (!boundary_cloud)
    {
        printW(QString("Boundary cloud[id:%1] failed.").arg(QString::fromStdString(source_id)));
        return;
    }

    printI(QString("Estimate cloud[id:%1] boundary done, take time %2 ms.")
               .arg(QString::fromStdString(source_id)).arg(time_ms));

    QString sid = QString::fromStdString(source_id) + BOUNDARY_PRE_FLAG;
    boundary_cloud->setId(sid.toStdString());

    m_cloudview->addPointCloud(boundary_cloud);
    m_cloudview->setPointCloudColor(sid, ct::Color::Green);
    m_cloudview->setPointCloudSize(sid, boundary_cloud->pointSize());
    m_boundary_map[source_id] = boundary_cloud;
}

void Boundary::preview()
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
        if (!cloud->hasNormals())
        {
            printW("Please estimate normals first!");
            return;
        }

        // 清除之前的预览结果
        auto it = m_boundary_map.find(cloud->id());
        if (it != m_boundary_map.end())
        {
            QString sid = QString::fromStdString(cloud->id()) + BOUNDARY_PRE_FLAG;
            m_cloudview->removePointCloud(sid);
            m_boundary_map.erase(it);
        }

        std::string cloud_id = cloud->id();
        double angle = ui->dspin_angle->value();

        m_cloudview->showInfo("BoundaryEstimation", 1);
        runBoundary(cloud_id, [cloud, k, r, angle, this](std::function<void(int)> on_progress) {
            return ct::Features::BoundaryEstimation(cloud, k, r, angle, &m_cancel, on_progress);
        });
    }
}

void Boundary::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        auto it = m_boundary_map.find(cloud->id());
        if (it == m_boundary_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched boundary!")
                       .arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr new_cloud = it->second;
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
        new_cloud->setId(BOUNDARY_ADD_FLAG + cloud->id());
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        m_cloudtree->insertCloud(new_cloud, item, true, ct::MountStrategy::Sibling);
        m_boundary_map.erase(cloud->id());
        printI(QString("Add cloud boundary[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Boundary::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        auto it = m_boundary_map.find(cloud->id());
        if (it == m_boundary_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched boundary!")
                       .arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr new_cloud = it->second;
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_boundary_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply cloud boundary[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Boundary::reset()
{
    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);
    for (auto& [sid, cloud] : m_boundary_map)
        m_cloudview->removePointCloud(QString::fromStdString(cloud->id()));
    m_boundary_map.clear();
    m_cloudview->clearInfo();
}
