#include "cloudview.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType)

#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCommand.h>
#include <vtkNew.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkJPEGWriter.h>
#include <vtkBMPWriter.h>
#include <vtkTIFFWriter.h>

#include <QDropEvent>
#include <QFileInfo>
#include <QUrl>

#include <cmath>

namespace pw
{
    namespace
    {
        class PCLDisableInteractorStyle : public vtkInteractorStyleTrackballCamera
        {
        public:
            static PCLDisableInteractorStyle* New();

            vtkTypeMacro(PCLDisableInteractorStyle, vtkInteractorStyleTrackballCamera);

            virtual void OnLeftButtonDown() override {}
            virtual void OnMiddleButtonDown() override {}
            virtual void OnRightButtonDown() override {}
            virtual void OnMouseWheelForward() override {}
            virtual void OnMouseWheelBackward() override {}
        };
        vtkStandardNewMacro(PCLDisableInteractorStyle);
    } // namespace

    CloudView::CloudView(QWidget *parent)
        : QVTKOpenGLNativeWidget(parent),
        m_show_id(true),
        m_info_level(0),
        m_last_id(""),
        m_render(vtkSmartPointer<vtkRenderer>::New()),
        m_renderwindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New())
    {
        this->setMinimumWidth(400);

        m_renderwindow->AddRenderer(m_render);
        m_render->GetActiveCamera()->SetClippingRange(0.01, 10000.0);
        m_viewer.reset(new pcl::visualization::PCLVisualizer(m_render, m_renderwindow, "viewer", false));
        this->setRenderWindow(m_renderwindow);

        m_viewer->setupInteractor(this->interactor(), this->renderWindow());
        if (this->interactor()) {
            vtkNew<vtkCallbackCommand> renderCallback;
            renderCallback->SetCallback(CloudView::OnRenderEvent);
            renderCallback->SetClientData(this);
            m_observer_tag = this->interactor()->AddObserver(vtkCommand::RenderEvent, renderCallback);

            vtkNew<vtkCallbackCommand> interactionCallback;
            interactionCallback->SetCallback(CloudView::OnInteractionEvent);
            interactionCallback->SetClientData(this);

            this->interactor()->AddObserver(vtkCommand::StartInteractionEvent, interactionCallback);
            this->interactor()->AddObserver(vtkCommand::InteractionEvent, interactionCallback);
            this->interactor()->AddObserver(vtkCommand::EndInteractionEvent, interactionCallback);
        }

        m_render->GradientBackgroundOn();
        m_render->SetBackground2(0.05, 0.4, 0.6);
        m_render->SetBackground(0.00, 0.05, 0.08);

        m_scalar_bar_widget = new ScalarBarWidget(this);
        m_scalar_bar_widget->lower();
        m_scalar_bar_widget->setAttribute(Qt::WA_TransparentForMouseEvents);
        m_scalar_bar_widget->show();
        m_scalar_bar_widget->raise();

        connect(this, &CloudView::sizeChanged, [this](QSize size){
            if (m_show_id && !m_current_id.isEmpty()) {
                m_viewer->updateText(m_current_id.toStdString(),
                                     10,
                                     size.height() - 25,
                                     12, 1, 1, 1, INFO_CLOUD_ID);
            }

            QMap<int, InfoData>::iterator i;
            for (i = m_active_infos.begin(); i != m_active_infos.end(); ++i) {
                int level = i.key();
                const InfoData& data = i.value();
                std::string id = INFO_TEXT + std::to_string(level);
                int y_pos = size.height() - 25 * (level + 1);
                m_viewer->updateText(data.text.toStdString(), 10, y_pos, 12,
                                     data.rgb.rf(), data.rgb.gf(), data.rgb.bf(), id);
            }

            if (m_view_cube) {
                int side = std::min(size.width(), size.height());
                int cubeSize = side / 5;
                cubeSize = std::max(cubeSize, 120);
                cubeSize = std::min(cubeSize, 280);
                m_view_cube->setFixedSize(cubeSize, cubeSize);
                m_view_cube->move(size.width() - cubeSize - 8,
                                  size.height() - cubeSize - 8);

                m_scalar_bar_widget->setBottomMargin(cubeSize + 16);
            }

            m_scalar_bar_widget->relayout();
            m_scalar_bar_widget->update();
        });

        m_view_cube = new ViewCube(this);
        m_view_cube->setCameraCallback(
            [this](const Eigen::Vector3f& dir, const Eigen::Vector3f& up, bool smooth) {
                if (smooth) {
                    vtkCamera* cam = m_render->GetActiveCamera();
                    double fp[3], pos[3];
                    cam->GetFocalPoint(fp);
                    cam->GetPosition(pos);
                    double dist = std::sqrt(std::pow(pos[0]-fp[0], 2) +
                                            std::pow(pos[1]-fp[1], 2) +
                                            std::pow(pos[2]-fp[2], 2));
                    m_viewer->setCameraPosition(
                        fp[0] + dir.x() * dist, fp[1] + dir.y() * dist, fp[2] + dir.z() * dist,
                        fp[0], fp[1], fp[2],
                        up.x(), up.y(), up.z());
                    if (m_auto_render) m_viewer->getRenderWindow()->Render();
                } else {
                    setView(dir, up);
                }
            });
        m_view_cube->lower();
        m_view_cube->setAttribute(Qt::WA_TransparentForMouseEvents, false);
        m_view_cube->show();
        m_view_cube->raise();

        {
            int side = std::min(width(), height());
            int cubeSize = side / 5;
            cubeSize = std::max(cubeSize, 120);
            cubeSize = std::min(cubeSize, 280);
            m_view_cube->setFixedSize(cubeSize, cubeSize);
            m_view_cube->move(width() - cubeSize - 8, height() - cubeSize - 8);

            m_scalar_bar_widget->setBottomMargin(cubeSize + 16);
        }

        m_viewer->getRenderWindow()->Render();
    }

    CloudView::~CloudView()
    {
        if (this->interactor()) {
            this->interactor()->RemoveObserver(m_observer_tag);
            if (m_interaction_tag)
                this->interactor()->RemoveObserver(m_interaction_tag);
        }

        for (auto it = m_textured_mesh_actors.begin(); it != m_textured_mesh_actors.end(); ++it) {
            for (const auto& actor : it.value())
                m_render->RemoveActor(actor);
        }
        m_textured_mesh_actors.clear();

        for (auto it = m_badge_actors.begin(); it != m_badge_actors.end(); ++it)
            m_render->RemoveActor(it.value());
        m_badge_actors.clear();

        m_OctreeRenders.clear();
    }

    void CloudView::OnRenderEvent(vtkObject* caller, unsigned long eventId, void* clientData, void* callData)
    {
        CloudView* self = static_cast<CloudView*>(clientData);
        if (self) {
            self->updateRenderers();
        }
    }

    void CloudView::OnInteractionEvent(vtkObject* caller, unsigned long eventId, void* clientData, void* callData)
    {
        CloudView* self = static_cast<CloudView*>(clientData);
        if (!self) return;

        if (eventId == vtkCommand::StartInteractionEvent) {
            self->onInteraction(true);
        } else if (eventId == vtkCommand::InteractionEvent) {
            emit self->viewerPose(self->m_viewer->getViewerPose());
        } else if (eventId == vtkCommand::EndInteractionEvent) {
            self->onInteraction(false);
        }
    }

    void CloudView::updateRenderers() {
        syncViewCubeOrientation();

        vtkCamera* cam = m_render->GetActiveCamera();
        double* pos = cam->GetPosition();
        double* focal = cam->GetFocalPoint();

        bool cameraChanged = (m_last_render_cam != cam ||
                              std::abs(pos[0] - m_last_render_pos.x()) > 0.01 ||
                              std::abs(pos[1] - m_last_render_pos.y()) > 0.01 ||
                              std::abs(pos[2] - m_last_render_pos.z()) > 0.01 ||
                              std::abs(focal[0] - m_last_render_focal[0]) > 0.01 ||
                              std::abs(focal[1] - m_last_render_focal[1]) > 0.01 ||
                              std::abs(focal[2] - m_last_render_focal[2]) > 0.01);

        if (cameraChanged) {
            m_last_render_cam = cam;
            m_last_render_pos = Eigen::Vector3f(pos[0], pos[1], pos[2]);
            m_last_render_focal[0] = focal[0];
            m_last_render_focal[1] = focal[1];
            m_last_render_focal[2] = focal[2];
        }

        for (auto& renderer : m_OctreeRenders) {
            renderer->update();
        }
    }

    void CloudView::onInteraction(bool is_interacting)
    {
        for (auto& renderer : m_OctreeRenders) {
            if (renderer) {
                renderer->setInteractionState(is_interacting);
            }
        }

        if (!is_interacting && m_auto_render) {
            m_viewer->getRenderWindow()->Render();
        }
    }

    ///////////////////////////////////////////////////////////////////
    // properties
    void CloudView::setPointCloudColor(const Cloud::Ptr &cloud, const ColorRGB& rgb)
    {
        cloud->setCloudColor(rgb);

        QString qid = QString::fromStdString(cloud->id());
        if (m_OctreeRenders.contains(qid)) {
            m_OctreeRenders[qid]->invalidateCache();
            m_OctreeRenders[qid]->update();
        }

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const QString &id, const ColorRGB &rgb)
    {
        std::string sid = id.toStdString();
        for(auto& weak_c : m_visible_clouds) {
            if (auto c = weak_c.lock()) {
                if (c->id() == sid) {
                    setPointCloudColor(c, rgb);
                    return;
                }
            }
        }
    }

    void CloudView::setPointCloudColor(const Cloud::Ptr &cloud, const QString& axis)
    {
        cloud->setCloudColor(axis.toStdString());

        QString qid = QString::fromStdString(cloud->id());
        if (m_OctreeRenders.contains(qid)) {
            m_OctreeRenders[qid]->invalidateCache();
            m_OctreeRenders[qid]->update();
        }

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetPointCloudColor(const Cloud::Ptr &cloud)
    {
        cloud->restoreColors();

        QString qid = QString::fromStdString(cloud->id());
        if (m_OctreeRenders.contains(qid)) {
            m_OctreeRenders[qid]->invalidateCache();
            m_OctreeRenders[qid]->update();
        }

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudSize(const QString &id, float size)
    {
        std::string sid = id.toStdString();
        for(auto& weak_c : m_visible_clouds) {
            if (auto c = weak_c.lock()) {
                if (c->id() == sid) {
                    c->setPointSize(size);

                    if (m_OctreeRenders.contains(id)) m_OctreeRenders[id]->update();
                    break;
                }
            }
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudOpacity(const QString &id, float value)
    {
        std::string sid = id.toStdString();
        for(auto& weak_c : m_visible_clouds) {
            if (auto c = weak_c.lock()) {
                if (c->id() == sid) {
                    c->setOpacity(value);
                    if (m_OctreeRenders.contains(id)) m_OctreeRenders[id]->update();
                    break;
                }
            }
        }

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setBackgroundColor(const pw::ColorRGB &rgb)
    {
        m_render->GradientBackgroundOff();
        m_viewer->setBackgroundColor(rgb.rf(), rgb.gf(), rgb.bf());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetBackgroundColor()
    {
        m_render->GradientBackgroundOn();
        m_render->SetBackground2(0.05, 0.4, 0.6);
        m_render->SetBackground(0.0, 0.05, 0.08);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeColor(const QString &shapeid, const ColorRGB &rgb)
    {
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                              rgb.rf(), rgb.gf(), rgb.bf(), shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeSize(const QString& shapeid, float size)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeOpacity(const QString& shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, value, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeLineWidth(const QString &shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeFontSize(const QString &shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_FONT_SIZE, value, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeRepersentation(const QString &shapeid, int type)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION, type, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeVisibility(const QString& id, bool visible)
    {
        auto it = m_textured_mesh_actors.find(id);
        if (it != m_textured_mesh_actors.end()) {
            for (const auto& actor : it.value()) {
                if (actor) actor->SetVisibility(visible ? 1 : 0);
            }
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
            return;
        }
        std::string std_id = id.toStdString();
        if (m_viewer->contains(std_id)) {
            m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                  visible ? 1.0 : 0.0, std_id);
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::setPointCloudVisibility(const QString &id, bool visible) {
        if (m_OctreeRenders.contains(id)) {
            m_OctreeRenders[id]->setVisibility(visible);
            m_OctreeRenders[id]->update();

            std::string std_id = id.toStdString();
            std::string normal_id = id.toStdString() + "-normals";
            std::string box_id = id.toStdString() + "-box";

            if (m_viewer->contains(normal_id)) m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, visible?1.0:0.0, normal_id);

            auto shape_map = m_viewer->getShapeActorMap();
            auto it_box = shape_map->find(box_id);
            if (it_box != shape_map->end()){
                it_box->second->SetVisibility(visible ? 1 : 0);
            }
        }

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::showInfo(const QString &text, int level, const ColorRGB &rgb)
    {
        m_info_level = std::max(m_info_level, level);

        ColorRGB color = rgb;
        bool isDefaultWhite = (color.r == Color::White.r &&
                               color.g == Color::White.g &&
                               color.b == Color::White.b);
        if (isDefaultWhite) {
            switch (level) {
            case 1:  color = Color::Cyan;   break;
            case 2:  color = Color::Green;  break;
            case 3:  color = Color::Yellow; break;
            case 4:  color = Color::Purple; break;
            default: color = Color::White;  break;
            }
        }

        m_active_infos[level] = {text, color};

        int y_row = level + 1;
        std::string id = INFO_TEXT + std::to_string(level);
        if (!m_viewer->contains(id))
            m_viewer->addText(text.toStdString(), 10, this->height() - 25 * y_row, 12, color.rf(), color.gf(), color.bf(), id);
        else
            m_viewer->updateText(text.toStdString(), 10, this->height() - 25 * y_row, 12, color.rf(), color.gf(), color.bf(), id);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::clearInfo()
    {
        for (int i = 0; i < m_info_level; i++)
        {
            std::string id = INFO_TEXT + std::to_string(i + 1);
            if (m_viewer->contains(id))
            {
                m_viewer->removeShape(id);
            }
        }
        m_active_infos.clear();
        m_info_level = 0;
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::showCloudId (const QString& id)
    {
        m_last_id = id;
        m_current_id = id;
        if (!m_show_id)
        {
            return;
        }
        if (!m_viewer->contains(INFO_CLOUD_ID))
            m_viewer->addText(id.toStdString(), 10, this->height() - 25, 12, 1, 1, 1, INFO_CLOUD_ID);
        else
            m_viewer->updateText(id.toStdString(), 10, this->height() - 25, 12, 1, 1, 1, INFO_CLOUD_ID);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShowId(const bool &enable)
    {
        m_show_id = enable;
        if (enable)
            showCloudId(m_last_id);
        else
            m_viewer->removeShape(INFO_CLOUD_ID);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setInteractorEnable(const bool &enable)
    {
        if (!enable)
        {
            vtkNew<PCLDisableInteractorStyle> style;
            m_renderwindow->GetInteractor()->SetInteractorStyle(style);
        }
        else
        {
            vtkNew<pcl::visualization::PCLVisualizerInteractorStyle> style;
            m_renderwindow->GetInteractor()->SetInteractorStyle(style);
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    ///////////////////////////////////////////////////////////////////
    // viewport
    void CloudView::setView(const Eigen::Vector3f& direction, const Eigen::Vector3f& up) {
        m_viewer->resetCamera();

        m_viewer->getRenderWindow()->Render();
        vtkCamera* cam = m_render->GetActiveCamera();

        double* fp = cam->GetFocalPoint();

        double* pos = cam->GetPosition();
        double dist = std::sqrt(std::pow(pos[0] - fp[0], 2) +
                                std::pow(pos[1] - fp[1], 2) +
                                std::pow(pos[2] - fp[2], 2));

        double new_x = fp[0] + direction.x () * dist;
        double new_y = fp[1] + direction.y () * dist;
        double new_z = fp[2] + direction.z () * dist;

        m_viewer->setCameraPosition(
                new_x, new_y, new_z,
                fp[0], fp[1], fp[2],
                up.x(), up.y(), up.z()
                );

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
        syncViewCubeOrientation();
    }

    void CloudView::setTopView() {
        setView(Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 1, 0));
    }

    void CloudView::setBottomView() {
        setView(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f(0, 1, 0));
    }

    void CloudView::setFrontView() {
        setView(Eigen::Vector3f(0, -1, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setBackView() {
        setView(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setLeftSideView() {
        setView(Eigen::Vector3f(-1, 0, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setRightSideView() {
        setView(Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::zoomToBounds(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt){
        double bounds[6] = {
                (double)min_pt.x(), (double)max_pt.x(),
                (double)min_pt.y(), (double)max_pt.y(),
                (double)min_pt.z(), (double)max_pt.z()
        };
        m_render->ResetCamera(bounds);
        m_render->ResetCameraClippingRange();
        updateRenderers();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::refresh()
    {
        updateRenderers();
        m_render->ResetCameraClippingRange();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::invalidateCloudRender(const QString& cloud_id)
    {
        auto it = m_OctreeRenders.find(cloud_id);
        if (it != m_OctreeRenders.end() && it.value()) {
            it.value()->invalidateCache();
            it.value()->update();
        }
        m_render->ResetCameraClippingRange();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::invalidateCloudRenderDirty(const QString& cloud_id)
    {
        auto it = m_OctreeRenders.find(cloud_id);
        if (it != m_OctreeRenders.end() && it.value()) {
            it.value()->invalidateDirtyActors();
            it.value()->update();
        }
        m_render->ResetCameraClippingRange();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resizeEvent(QResizeEvent* event)
    {
        QVTKOpenGLNativeWidget::resizeEvent(event);
        emit sizeChanged(event->size());
    }

    void CloudView::mousePressEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            emit mouseLeftPressed(PointXY(this->interactor()->GetEventPosition()[0],
                                          this->interactor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightPressed(PointXY(this->interactor()->GetEventPosition()[0],
                                           this->interactor()->GetEventPosition()[1]));
        }
    }

    void CloudView::mouseReleaseEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            emit viewerPose(m_viewer->getViewerPose());
            emit mouseLeftReleased(PointXY(this->interactor()->GetEventPosition()[0],
                                           this->interactor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightReleased(PointXY(this->interactor()->GetEventPosition()[0],
                                            this->interactor()->GetEventPosition()[1]));
        }
    }

    void CloudView::mouseMoveEvent(QMouseEvent *event)
    {
        emit mouseMoved(PointXY(this->interactor()->GetEventPosition()[0],
                                this->interactor()->GetEventPosition()[1]));
    }

    ///////////////////////////////////////////////////////////////////
    // camera & view state serialization
    CameraParams CloudView::getCameraParams() const
    {
        CameraParams p;
        vtkCamera* cam = m_render->GetActiveCamera();
        if (!cam) return p;

        cam->GetPosition(p.position);
        cam->GetFocalPoint(p.focal_point);
        cam->GetViewUp(p.view_up);
        p.clip_near = cam->GetClippingRange()[0];
        p.clip_far = cam->GetClippingRange()[1];
        return p;
    }

    Eigen::Affine3f CloudView::getViewerPose() const
    {
        if (!m_viewer) return Eigen::Affine3f::Identity();
        return m_viewer->getViewerPose();
    }

    void CloudView::setCameraParams(const CameraParams& params)
    {
        vtkCamera* cam = m_render->GetActiveCamera();
        if (!cam) return;

        cam->SetPosition(params.position);
        cam->SetFocalPoint(params.focal_point);
        cam->SetViewUp(params.view_up);
        cam->SetClippingRange(params.clip_near, params.clip_far);
        m_render->ResetCameraClippingRange();
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShowAxes(const bool& enable)
    {
        if (m_view_cube) m_view_cube->setViewCubeEnabled(enable);
    }

    void CloudView::syncViewCubeOrientation()
    {
        if (!m_view_cube || !m_view_cube->isViewCubeEnabled()) return;
        vtkCamera* cam = m_render->GetActiveCamera();
        double pos[3], fp[3], up[3];
        cam->GetPosition(pos);
        cam->GetFocalPoint(fp);
        cam->GetViewUp(up);
        Eigen::Vector3f dir(pos[0]-fp[0], pos[1]-fp[1], pos[2]-fp[2]);
        dir.normalize();
        Eigen::Vector3f cameraUp(up[0], up[1], up[2]);
        m_view_cube->updateCameraOrientation(dir, cameraUp);
    }

    ViewOptions CloudView::getViewOptions() const
    {
        ViewOptions opts;
        opts.show_fps = m_show_fps;
        opts.show_axes = m_view_cube && m_view_cube->isViewCubeEnabled();
        opts.show_id = m_show_id;

        double bg[3], bg2[3];
        m_render->GetBackground(bg);
        m_render->GetBackground2(bg2);
        opts.use_gradient_bg = m_render->GetGradientBackground() != 0;
        for (int i = 0; i < 3; ++i) {
            opts.bg_color[i] = bg[i];
            opts.bg_color2[i] = bg2[i];
        }

        return opts;
    }

    void CloudView::setViewOptions(const ViewOptions& opts)
    {
        setShowFPS(opts.show_fps);
        setShowAxes(opts.show_axes);
        setShowId(opts.show_id);

        if (opts.use_gradient_bg) {
            m_render->GradientBackgroundOn();
            m_render->SetBackground2(opts.bg_color2[0], opts.bg_color2[1], opts.bg_color2[2]);
        } else {
            m_render->GradientBackgroundOff();
        }
        m_render->SetBackground(opts.bg_color[0], opts.bg_color[1], opts.bg_color[2]);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    ///////////////////////////////////////////////////////////////////
    // scalar bar
    void CloudView::showScalarBar(double min_val, double max_val,
                                  const QString& title, ColormapType colormap)
    {
        m_scalar_bar_widget->updateData(min_val, max_val, title, colormap);
        m_scalar_bar_widget->setBarVisible(true);
    }

    void CloudView::setScalarBarDisplayRange(double disp_min, double disp_max)
    {
        m_scalar_bar_widget->setDisplayRange(disp_min, disp_max);
    }

    void CloudView::setScalarBarHistogram(const std::vector<int>& bin_counts,
                                          double data_min, double data_max,
                                          bool show_grey)
    {
        m_scalar_bar_widget->setHistogramData(bin_counts, data_min, data_max, show_grey);
    }

    void CloudView::setScalarBarShowCurve(bool show)
    {
        m_scalar_bar_widget->setShowCurve(show);
    }

    void CloudView::hideScalarBar()
    {
        m_scalar_bar_widget->setBarVisible(false);
    }

    void CloudView::setScalarBarVisible(bool visible)
    {
        m_scalar_bar_widget->setBarVisible(visible);
    }

    bool CloudView::isScalarBarVisible() const
    {
        return m_scalar_bar_widget->isBarVisible();
    }

    void CloudView::setScalarBarShowZero(bool show)
    {
        m_scalar_bar_widget->setShowZeroLine(show);
    }

    bool CloudView::captureScreenshot(const QString& filePath)
    {
        if (!m_renderwindow) return false;

        m_renderwindow->Render();

        vtkNew<vtkWindowToImageFilter> windowToImage;
        windowToImage->SetInput(m_renderwindow);
        windowToImage->SetInputBufferTypeToRGB();
        windowToImage->SetScale(1);
        windowToImage->ReadFrontBufferOn();
        windowToImage->ShouldRerenderOff();
        windowToImage->Update();

        m_renderwindow->Render();

        vtkImageData* imgData = windowToImage->GetOutput();
        int dims[3];
        imgData->GetDimensions(dims);
        int w = dims[0], h = dims[1];
        auto* pixels = static_cast<unsigned char*>(imgData->GetScalarPointer());

        QImage qimg(w, h, QImage::Format_RGB888);
        for (int y = 0; y < h; y++) {
            memcpy(qimg.scanLine(y), pixels + (h - 1 - y) * w * 3, w * 3);
        }

        return qimg.save(filePath);
    }

} // namespace pw
