#include "surface.h"
#include "ui_surface.h"

#define SURFACE_TYPE_GreedyProjectionTriangulation  (0)
#define SURFACE_TYPE_GridProjection                 (1)
#define SURFACE_TYPE_Poisson                        (2)
#define SURFACE_TYPE_MarchingCubesRBF               (3)
#define SURFACE_TYPE_MarchingCubesHoppe             (4)
#define SURFACE_TYPE_ConvexHull                     (5)
#define SURFACE_TYPE_ConcaveHull                    (6)

Surface::Surface(QWidget* parent)
    : CustomDialog(parent), ui(new Ui::Surface)
{
    ui->setupUi(this);

    connect(ui->btn_preview, &QPushButton::clicked, this, &Surface::preview);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Surface::reset);

    ui->cbox_surface->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);

    connect(ui->check_polygonline, &QCheckBox::clicked, [=](bool state)
    {
        std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
        for (auto& cloud : selected_clouds)
        {
            auto it = m_surface_map.find(cloud->id());
            if (it == m_surface_map.end()) continue;

            QString sid = QString::fromStdString(cloud->id()) + SURFACE_PRE_FLAG;
            m_cloudview->removePolygonMesh(sid);
            m_cloudview->removeShape(sid);

            if (state)
                m_cloudview->addPolylineFromPolygonMesh(it->second, sid);
            else
                m_cloudview->addPolygonMesh(it->second, sid);
        }
    });
}

Surface::~Surface()
{
    m_cancel = true;
    if (m_watcher && !m_watcher->isFinished()) {
        m_watcher->waitForFinished();
    }
    delete ui;
}

void Surface::runSurface(const std::string& source_id, std::function<ct::SurfaceResult(std::function<void(int)>)> fn)
{
    m_cloudtree->showProgress("Surface...");

    m_cancel = false;
    if (m_cloudtree->m_processing_dialog) {
        connect(m_cloudtree->m_processing_dialog, &ct::ProcessingDialog::cancelRequested,
                this, [this]() { m_cancel = true; }, Qt::UniqueConnection);
    }

    auto progress_cb = [this](int value) {
        QMetaObject::invokeMethod(m_cloudtree->m_processing_dialog, "setProgress",
                                  Qt::QueuedConnection, Q_ARG(int, value));
    };

    auto future = QtConcurrent::run([fn = std::move(fn), progress_cb = std::move(progress_cb)]() -> ct::SurfaceResult {
        try {
            return fn(progress_cb);
        } catch (const std::exception& e) {
            ct::SurfaceResult result;
            result.mesh = nullptr;
            result.error_msg = std::string("Exception: ") + e.what();
            return result;
        } catch (...) {
            ct::SurfaceResult result;
            result.mesh = nullptr;
            result.error_msg = "Unknown exception occurred.";
            return result;
        }
    });

    if (!m_watcher) {
        m_watcher = new QFutureWatcher<ct::SurfaceResult>(this);
    }

    connect(m_watcher, &QFutureWatcher<ct::SurfaceResult>::finished, this,
        [this, source_id]() {
            m_cloudtree->closeProgress();
            auto result = m_watcher->result();
            handleSurfaceResult(source_id, result);
        }, Qt::UniqueConnection);

    m_watcher->setFuture(future);
}

void Surface::handleSurfaceResult(const std::string& source_id, const ct::SurfaceResult& result)
{
    if (!result.mesh)
    {
        printW(QString("Surface cloud[id:%1] failed: %2")
                   .arg(QString::fromStdString(source_id))
                   .arg(QString::fromStdString(result.error_msg)));
        return;
    }

    printI(QString("Surface cloud[id:%1] done, take time %2 ms.")
               .arg(QString::fromStdString(source_id)).arg(result.time_ms));

    m_surface_map[source_id] = result.mesh;
    QString sid = QString::fromStdString(source_id) + SURFACE_PRE_FLAG;

    if (ui->check_polygonline->isChecked())
        m_cloudview->addPolylineFromPolygonMesh(result.mesh, sid);
    else
        m_cloudview->addPolygonMesh(result.mesh, sid);
}

void Surface::preview()
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
        auto it = m_surface_map.find(cloud->id());
        if (it != m_surface_map.end())
        {
            QString sid = QString::fromStdString(cloud->id()) + SURFACE_PRE_FLAG;
            m_cloudview->removePolygonMesh(sid);
            m_cloudview->removeShape(sid);
            m_surface_map.erase(it);
        }
    }

    for (auto& cloud : selected_clouds)
    {
        std::string cloud_id = cloud->id();

        switch (ui->cbox_surface->currentIndex())
        {
        case SURFACE_TYPE_GreedyProjectionTriangulation:
        {
            m_cloudview->showInfo("Greedy Projection Triangulation", 1);
            double mu = ui->dspin_mu->value();
            int nnn = ui->spin_nnn->value();
            double radius = ui->dspin_radius->value();
            double min = ui->spin_minangle->value();
            double max = ui->spin_maxangle->value();
            double ep = ui->spin_maxsurface->value();
            bool consistent = ui->check_consistent->isChecked();
            bool consistent_ordering = ui->check_consistent_order->isChecked();
            runSurface(cloud_id, [cloud, mu, nnn, radius, min, max, ep, consistent, consistent_ordering, this](std::function<void(int)> on_progress) {
                return ct::Surface::GreedyProjectionTriangulation(cloud, mu, nnn, radius, min, max, ep,
                    consistent, consistent_ordering, &m_cancel, on_progress);
            });
            break;
        }
        case SURFACE_TYPE_GridProjection:
        {
            m_cloudview->showInfo("GridProjection", 1);
            double resolution = ui->dspin_leaf_size->value();
            int padding_size = ui->spin_padding_size->value();
            int k = ui->spin_k->value();
            int max_binary_search_level = ui->spin_max_binary_search_level->value();
            runSurface(cloud_id, [cloud, resolution, padding_size, k, max_binary_search_level, this](std::function<void(int)> on_progress) {
                return ct::Surface::GridProjection(cloud, resolution, padding_size, k,
                    max_binary_search_level, &m_cancel, on_progress);
            });
            break;
        }
        case SURFACE_TYPE_Poisson:
        {
            m_cloudview->showInfo("Poisson", 1);
            int depth = ui->spin_depth->value();
            int min_depth = ui->spin_min_depth->value();
            float point_weight = ui->dspin_point_weight->value();
            float scale = ui->dspin_scale->value();
            int solver_divide = ui->spin_solver_divide->value();
            int iso_divide = ui->spin_iso_divide->value();
            float samples_per_node = ui->dspin_samples_per_node->value();
            bool confidence = ui->check_confidence->isChecked();
            bool output_polygons = ui->check_out_polygons->isChecked();
            bool manifold = ui->check_manifold->isChecked();
            runSurface(cloud_id, [cloud, depth, min_depth, point_weight, scale,
                             solver_divide, iso_divide, samples_per_node,
                             confidence, output_polygons, manifold, this](std::function<void(int)> on_progress) {
                return ct::Surface::Poisson(cloud, depth, min_depth, point_weight, scale,
                    solver_divide, iso_divide, samples_per_node,
                    confidence, output_polygons, manifold, &m_cancel, on_progress);
            });
            break;
        }
        case SURFACE_TYPE_MarchingCubesRBF:
        {
            m_cloudview->showInfo("MarchingCubesRBF", 1);
            float iso_level = ui->dspin_iso_level->value();
            int res_x = ui->spin_res_x->value();
            int res_y = ui->spin_res_y->value();
            int res_z = ui->spin_res_z->value();
            float percentage = ui->dspin_percentage->value();
            float epsilon = ui->dspin_epslion->value();
            runSurface(cloud_id, [cloud, iso_level, res_x, res_y, res_z, percentage, epsilon, this](std::function<void(int)> on_progress) {
                return ct::Surface::MarchingCubesRBF(cloud, iso_level, res_x, res_y, res_z,
                    percentage, epsilon, &m_cancel, on_progress);
            });
            break;
        }
        case SURFACE_TYPE_MarchingCubesHoppe:
        {
            m_cloudview->showInfo("MarchingCubesHoppe", 1);
            float iso_level = ui->dspin_iso_level_2->value();
            int res_x = ui->spin_res_x_2->value();
            int res_y = ui->spin_res_y_2->value();
            int res_z = ui->spin_res_z_2->value();
            float percentage = ui->dspin_percentage_2->value();
            float dist_ignore = ui->dspin_dist_ignore->value();
            runSurface(cloud_id, [cloud, iso_level, res_x, res_y, res_z, percentage, dist_ignore, this](std::function<void(int)> on_progress) {
                return ct::Surface::MarchingCubesHoppe(cloud, iso_level, res_x, res_y, res_z,
                    percentage, dist_ignore, &m_cancel, on_progress);
            });
            break;
        }
        case SURFACE_TYPE_ConvexHull:
        {
            m_cloudview->showInfo("ConvexHull", 1);
            bool value = ui->check_value->isChecked();
            int dimensio = ui->spin_dimensio->value();
            runSurface(cloud_id, [cloud, value, dimensio, this](std::function<void(int)> on_progress) {
                return ct::Surface::ConvexHull(cloud, value, dimensio, &m_cancel, on_progress);
            });
            break;
        }
        case SURFACE_TYPE_ConcaveHull:
        {
            m_cloudview->showInfo("ConcaveHull", 1);
            double alpha = ui->dspin_alpha->value();
            bool value = ui->check_value_2->isChecked();
            int dimensio = ui->spin_dimensio_2->value();
            runSurface(cloud_id, [cloud, alpha, value, dimensio, this](std::function<void(int)> on_progress) {
                return ct::Surface::ConcaveHull(cloud, alpha, value, dimensio, &m_cancel, on_progress);
            });
            break;
        }
        }
    }
}

void Surface::reset()
{
    for (auto& [sid, mesh] : m_surface_map)
    {
        QString qid = QString::fromStdString(sid) + SURFACE_PRE_FLAG;
        m_cloudview->removePolygonMesh(qid);
        m_cloudview->removeShape(qid);
    }
    m_surface_map.clear();
    m_cloudview->clearInfo();
}
