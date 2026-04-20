#include "cloudview.h"

#include <vtkAutoInit.h>
// VTK_MODULE_INIT 宏用于初始化 VTK 模块
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType)

#include <vtkAxesActor.h>
#include <vtkPointPicker.h>
#include <vtkCamera.h>
#include <vtkProperty2D.h>
#include <vtkTextActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>

#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkTexture.h>
#include <vtkImageData.h>
#include <vtkPNGReader.h>
#include <vtkJPEGReader.h>
#include <vtkBMPReader.h>
#include <vtkTIFFReader.h>
#include <vtkActorCollection.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/search/kdtree.h>


#include <QDropEvent>
#include <QMimeData>
#include <QUrl>
#include <QFileInfo>
#include <QDir>
#include <QFile>
#include <QTextStream>


#include <cmath>
#include <map>

#define INFO_CLOUD_ID  "info_cloud_id"
#define INFO_TEXT      "info_text"


namespace ct
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
        // 一个VTK宏，自动定义new函数的实现，用于简化对象创建
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
            // 1. 渲染回调 (已存在)
            vtkNew<vtkCallbackCommand> renderCallback;
            renderCallback->SetCallback(CloudView::OnRenderEvent);
            renderCallback->SetClientData(this);
            m_observer_tag = this->interactor()->AddObserver(vtkCommand::RenderEvent, renderCallback);

            // 2. 【新增】交互回调 (监听开始和结束)
            vtkNew<vtkCallbackCommand> interactionCallback;
            interactionCallback->SetCallback(CloudView::OnInteractionEvent);
            interactionCallback->SetClientData(this);

            // 监听 StartInteractionEvent (鼠标按下准备移动)
            this->interactor()->AddObserver(vtkCommand::StartInteractionEvent, interactionCallback);
            // 监听 EndInteractionEvent (鼠标松开停止移动)
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
                int y_pos = size.height() - 25 * level;
                m_viewer->updateText(data.text.toStdString(), 10, y_pos, 12,
                                     data.rgb.rf(), data.rgb.gf(), data.rgb.bf(), id);
            }

            // ViewCube 定位到右下角，并按窗口大小缩放
            if (m_view_cube) {
                int side = std::min(size.width(), size.height());
                int cubeSize = side / 5;
                cubeSize = std::max(cubeSize, 120);
                cubeSize = std::min(cubeSize, 280);
                m_view_cube->setFixedSize(cubeSize, cubeSize);
                m_view_cube->move(size.width() - cubeSize - 8,
                                  size.height() - cubeSize - 8);

                // 色度条底部边距 = cube 区域 + 间距
                m_scalar_bar_widget->setBottomMargin(cubeSize + 16);
            }

            // 色度条：Qt 子控件，固定像素宽度，仅高度跟随窗口
            m_scalar_bar_widget->relayout();
            m_scalar_bar_widget->update();
        });

        // ViewCube: 导航立方体（坐标系 + 立方体）
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

        // 初始尺寸
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
        }

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
        } else if (eventId == vtkCommand::EndInteractionEvent) {
            self->onInteraction(false);
        }
    }

    void CloudView::updateRenderers() {
        // 只在相机移动时更新
        static vtkCamera* lastCam = nullptr;
        static Eigen::Vector3f lastPos(0, 0, 0);
        static double lastFocal[3] = {0, 0, 0};

        // 同步 ViewCube 朝向
        syncViewCubeOrientation();

        vtkCamera* cam = m_render->GetActiveCamera();
        double* pos = cam->GetPosition();
        double* focal = cam->GetFocalPoint();

        // 检查相机是否真的移动了
        bool cameraChanged = (lastCam != cam ||
                              std::abs(pos[0] - lastPos.x()) > 0.01 ||
                              std::abs(pos[1] - lastPos.y()) > 0.01 ||
                              std::abs(pos[2] - lastPos.z()) > 0.01 ||
                              std::abs(focal[0] - lastFocal[0]) > 0.01 ||
                              std::abs(focal[1] - lastFocal[1]) > 0.01 ||
                              std::abs(focal[2] - lastFocal[2]) > 0.01);

        if (!cameraChanged) return; // 相机没动，不更新

        lastCam = cam;
        lastPos = Eigen::Vector3f(pos[0], pos[1], pos[2]);
        lastFocal[0] = focal[0];
        lastFocal[1] = focal[1];
        lastFocal[2] = focal[2];

        for (auto& renderer : m_OctreeRenders) {
            renderer->update();
        }
    }

    void CloudView::onInteraction(bool is_interacting)
    {
        // 遍历所有八叉树渲染器，通知它们改变状态
        for (auto& renderer : m_OctreeRenders) {
            if (renderer) {
                renderer->setInteractionState(is_interacting);
            }
        }

        // 如果是停止交互（松手），强制刷新一帧以加载高精度细节
        // (开始交互时不需要强制刷新，因为 VTK 交互本身就会触发连续渲染)
        if (!is_interacting && m_auto_render) {
            m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::addPointCloud(const Cloud::Ptr &cloud)
    {
        bool found = false;
        for (auto& c : m_visible_clouds){
            if (c->id() == cloud->id()) {
                found = true;
                break;
            }
        }
        if (!found) m_visible_clouds.push_back(cloud);

        QString qid = QString::fromStdString(cloud->id());
        if (m_OctreeRenders.contains(qid)) {
            m_OctreeRenders.remove(qid);
        }

        auto renderer = std::make_shared<OctreeRenderer>(cloud, m_viewer->getRendererCollection()->GetFirstRenderer());
        m_OctreeRenders.insert(qid, renderer);

        renderer->invalidateCache();
        renderer->update();

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudFromRangeImage(const pcl::RangeImage::Ptr &image, const QString &id, const ct::ColorRGB &rgb)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color(image, rgb.r, rgb.g, rgb.b);
        // 判断是否添加了该点云，如果未添加就将点云数据和颜色添加到视图器中，否则就更新视图器
        if (!m_viewer->contains(id.toStdString()))
        {
            m_viewer->addPointCloud(image, range_image_color, id.toStdString());
        }
        else
            m_viewer->updatePointCloud(image, range_image_color, id.toStdString());
        // 刷新窗口
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addBox(const Cloud::Ptr& cloud)
    {
        if (cloud->volume() <= 0.0f || cloud->box().width <= 0.0f){
            return;
        }
        std::string id = cloud->boxId();
        if (!m_viewer->contains(id))
        {
            m_viewer->addCube(cloud->box().translation, cloud->box().rotation,
                              cloud->box().width, cloud->box().height,
                              cloud->box().depth, id);

            m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                  pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                  id);
        }

        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud->boxColor().rf(),
                                              cloud->boxColor().gf(), cloud->boxColor().bf(), id);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudNormals(const Cloud::Ptr &cloud, int level, float scale)
    {
        // TODO 只显示一定数量的法线，不全部显示
        std::string id = cloud->normalId();

        if (!cloud->hasNormals()) return;

        // [性能优化]：不要转换全量点云！只采样一部分用于显示法线。
        // 对于大点云，全量显示法线会变成一团毛球，没有任何视觉意义且卡死显卡。
        // 我们限制用于显示法线的点数为 50,000 点。

        const size_t MAX_NORMAL_POINTS = 50000;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        size_t total = cloud->size();
        size_t step = (total > MAX_NORMAL_POINTS) ? (total / MAX_NORMAL_POINTS) : 1;

        temp_cloud->reserve(MAX_NORMAL_POINTS);

        // 遍历 Block 进行稀疏采样
        const auto& blocks = cloud->getBlocks();
        size_t global_idx = 0;

        for (const auto& block : blocks) {
            if (block->empty()) continue;

            size_t n = block->size();
            const auto& pts = block->m_points;
            const auto* colors = (block->m_colors) ? block->m_colors.get() : nullptr;
            const auto* norms = (block->m_normals) ? block->m_normals.get() : nullptr;

            if (!norms) continue; // 没有法线数据跳过

            for (size_t i = 0; i < n; ++i) {
                if (global_idx % step == 0) {
                    pcl::PointXYZRGBNormal p;
                    p.x = pts[i].x; p.y = pts[i].y; p.z = pts[i].z;

                    if (colors) { p.r=(*colors)[i].r; p.g=(*colors)[i].g; p.b=(*colors)[i].b; }
                    else { p.r=255; p.g=255; p.b=255; }

                    Eigen::Vector3f nv = (*norms)[i].get();
                    p.normal_x = nv.x(); p.normal_y = nv.y(); p.normal_z = nv.z();

                    temp_cloud->push_back(p);
                }
                global_idx++;

                // 如果凑够了就停，避免遍历完一亿个点
                if (temp_cloud->size() >= MAX_NORMAL_POINTS) goto done_sampling;
            }
        }

        done_sampling:

        if (!m_viewer->contains(id))
            m_viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(temp_cloud, level, scale, id);
        else {
            m_viewer->removePointCloud(id);
            m_viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(temp_cloud, level, scale, id);
        }

        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                   cloud->normalColor().rf(), cloud->normalColor().gf(),
                                                   cloud->normalColor().bf(), id);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCorrespondences(const Cloud::Ptr &source_points, const Cloud::Ptr &target_points,
                                       const pcl::CorrespondencesPtr &correspondences, const QString &id,
                                       int line_width)
    {
        std::string std_id = id.toStdString();

        // 清理旧的 shape
        if (m_viewer->contains(std_id))
            m_viewer->removeShape(std_id);

        auto srcPCL = source_points->toPCL_XYZRGB();
        auto tgtPCL = target_points->toPCL_XYZRGB();

        if (!m_viewer->contains(std_id))
            m_viewer->addCorrespondences<pcl::PointXYZRGB>(srcPCL, tgtPCL, *correspondences, std_id);
        else
            m_viewer->updateCorrespondences<pcl::PointXYZRGB>(srcPCL, tgtPCL, *correspondences, std_id);

        // 尝试设置线宽（部分 GPU 不支持 >1，但亮黄色细线也可辨识）
        if (line_width > 1) {
            auto shape_map = m_viewer->getShapeActorMap();
            for (int i = 0; i < (int)correspondences->size(); i++) {
                auto it = shape_map->find(std_id + "_line_" + std::to_string(i));
                if (it != shape_map->end()) {
                    auto* actor = vtkActor::SafeDownCast(it->second);
                    if (actor && actor->GetProperty()) {
                        actor->GetProperty()->SetLineWidth(line_width);
                        actor->GetProperty()->SetColor(1.0, 0.9, 0.0); // 亮黄色
                    }
                }
            }
        }

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolygon(const Cloud::Ptr &cloud, const QString &id, const ct::ColorRGB &rgb)
    {
        // TODO 同样需要转换格式，性能损耗
        std::string std_id = id.toStdString();
        auto pclCloud = cloud->toPCL_XYZRGB(); // 需要转换为 PCL 格式

        if (!m_viewer->contains(std_id))
            m_viewer->addPolygon<PointXYZRGB>(pclCloud, rgb.rf(), rgb.gf(), rgb.bf(), std_id);
        else
        {
            m_viewer->removeShape(std_id);
            m_viewer->addPolygon<PointXYZRGB>(pclCloud, rgb.rf(), rgb.gf(), rgb.bf(), std_id);
        }
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                              pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                              std_id);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolygonMesh(const pcl::PolygonMesh::Ptr& mesh, const QString& id, int viewport)
    {
        std::string std_id = id.toStdString();
        if (m_viewer->contains(std_id))
            m_viewer->removeShape(std_id);
        m_viewer->addPolygonMesh(*mesh, std_id, viewport);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addMeshActor(const pcl::PolygonMesh::Ptr& mesh, const QString& id)
    {
        // 先清理已有的 actor
        removeTexturedMesh(id);

        if (!mesh || mesh->polygons.empty()) return;

        // 将 pcl::PolygonMesh 转换为 vtkPolyData
        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        pcl::VTKUtils::mesh2vtk(*mesh, polydata);

        if (!polydata || polydata->GetNumberOfPoints() == 0) return;

        // 清除所有点数据（可能包含错误的方向法线和 RGB）
        polydata->GetPointData()->Initialize();

        // 创建顶点颜色数组（统一用 vertex coloring 控制颜色）
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetNumberOfTuples(polydata->GetNumberOfPoints());
        unsigned char gray = 204; // 0.8 * 255
        for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); ++i) {
            colors->SetTuple3(i, gray, gray, gray);
        }
        colors->SetName("MeshColors");
        polydata->GetPointData()->SetScalars(colors);

        // 重新计算一致的法线方向（修复 Hoppe 等算法产生的朝内法线问题）
        vtkSmartPointer<vtkPolyDataNormals> normalGen = vtkSmartPointer<vtkPolyDataNormals>::New();
        normalGen->SetInputData(polydata);
        normalGen->ConsistencyOn();        // 确保相邻面片法线方向一致
        normalGen->AutoOrientNormalsOn();  // 根据邻域全局方向自动翻转法线朝外
        normalGen->NonManifoldTraversalOff();
        normalGen->Update();
        polydata = normalGen->GetOutput();

        // 确保 MeshColors 数组在法线重建后仍然存在
        if (!polydata->GetPointData()->GetArray("MeshColors")) {
            vtkSmartPointer<vtkUnsignedCharArray> colors2 = vtkSmartPointer<vtkUnsignedCharArray>::New();
            colors2->SetNumberOfComponents(3);
            colors2->SetNumberOfTuples(polydata->GetNumberOfPoints());
            unsigned char gray2 = 204;
            for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); ++i) {
                colors2->SetTuple3(i, gray2, gray2, gray2);
            }
            colors2->SetName("MeshColors");
            polydata->GetPointData()->SetScalars(colors2);
        }

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetAmbient(0.1);
        actor->GetProperty()->SetDiffuse(0.8);

        m_render->AddActor(actor);
        QVector<vtkSmartPointer<vtkActor>> actors;
        actors.push_back(actor);
        m_textured_mesh_actors[id] = actors;

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addMeshActorFromPolydata(vtkSmartPointer<vtkPolyData> polydata, const QString& id)
    {
        removeTexturedMesh(id);
        if (!polydata || polydata->GetNumberOfPoints() == 0) return;

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetAmbient(0.1);
        actor->GetProperty()->SetDiffuse(0.8);

        m_render->AddActor(actor);
        QVector<vtkSmartPointer<vtkActor>> actors;
        actors.push_back(actor);
        m_textured_mesh_actors[id] = actors;

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addTexturedMesh(const QString& objFilePath, const QString& id)
    {
        removeTexturedMesh(id);

        QFileInfo objInfo(objFilePath);
        QDir objDir = objInfo.absoluteDir();

        // ============================================================
        // 1. 用 vtkOBJReader 读取完整几何
        //    VTK9+ 的 vtkOBJReader 会自动解析 MTL，并输出：
        //    - pointData 中每个材质一个 2-component UV 数组（以材质名命名）
        //    - cellData 中 MaterialIds（1 component）标明每个 cell 的材质编号
        //    - cellData 中 GroupIds
        // ============================================================
        vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName(objFilePath.toLocal8Bit().constData());
        reader->Update();
        vtkPolyData* fullPolyData = reader->GetOutput();
        if (!fullPolyData || fullPolyData->GetNumberOfPoints() == 0) return;

        vtkIdType totalCells = fullPolyData->GetNumberOfCells();

        // ============================================================
        // 2. 收集 pointData 中的 per-material UV 数组
        //    过滤条件：2 components 且不是 "Normals"
        //    按字母序排列，MaterialId=i 对应第 i 个 UV 数组
        // ============================================================
        QStringList uvArrayNames;
        for (int a = 0; a < fullPolyData->GetPointData()->GetNumberOfArrays(); a++) {
            vtkDataArray* arr = fullPolyData->GetPointData()->GetArray(a);
            if (arr && arr->GetNumberOfComponents() == 2 && arr->GetNumberOfTuples() > 0) {
                QString name = fullPolyData->GetPointData()->GetArrayName(a);
                if (name.compare("Normals", Qt::CaseInsensitive) != 0 &&
                    name.compare("TCoords", Qt::CaseInsensitive) != 0) {
                    uvArrayNames.append(name);
                }
            }
        }
        uvArrayNames.sort(); // MaterialId 按字母序对应

        // ============================================================
        // 3. 获取 cellData 中的 MaterialIds
        // ============================================================
        vtkDataArray* matIdArr = fullPolyData->GetCellData()->GetArray("MaterialIds");
        if (!matIdArr) {
            // 回退：无材质信息，用原始 TCoords 直接渲染
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(fullPolyData);
            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            m_render->AddActor(actor);
            QVector<vtkSmartPointer<vtkActor>> actors;
            actors.push_back(actor);
            m_textured_mesh_actors[id] = actors;
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
            return;
        }

        // ============================================================
        // 4. 解析 MTL 文件，获取材质颜色和纹理路径
        // ============================================================
        struct ObjMaterial {
            QString name;
            QString diffuseTexturePath;
            float kd[3] = {0.8f, 0.8f, 0.8f};
        };

        // 从 OBJ 文件提取 mtllib 引用
        QStringList mtlFiles;
        {
            QFile objFile(objFilePath);
            if (objFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QTextStream objIn(&objFile);
                while (!objIn.atEnd()) {
                    QString line = objIn.readLine().trimmed();
                    if (line.startsWith("mtllib "))
                        mtlFiles.append(line.mid(7).trimmed());
                }
                objFile.close();
            }
        }

        QMap<QString, ObjMaterial> materials;
        for (const QString& mtlFileName : mtlFiles) {
            QString mtlPath = objDir.absoluteFilePath(mtlFileName);
            QFile mtlFile(mtlPath);
            if (!mtlFile.open(QIODevice::ReadOnly | QIODevice::Text)) continue;

            ObjMaterial curMat;
            QTextStream mtlIn(&mtlFile);
            while (!mtlIn.atEnd()) {
                QString line = mtlIn.readLine().trimmed();
                if (line.isEmpty() || line.startsWith('#')) continue;

                if (line.startsWith("newmtl ")) {
                    if (!curMat.name.isEmpty())
                        materials[curMat.name] = curMat;
                    curMat = ObjMaterial();
                    curMat.name = line.mid(7).trimmed();
                } else if (line.startsWith("Kd ")) {
                    auto parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
                    if (parts.size() >= 4) {
                        curMat.kd[0] = parts[1].toFloat();
                        curMat.kd[1] = parts[2].toFloat();
                        curMat.kd[2] = parts[3].toFloat();
                    }
                } else if (line.startsWith("map_Kd", Qt::CaseInsensitive)) {
                    QString texPart = line.mid(6).trimmed();
                    QStringList parts = texPart.split(QRegularExpression("\\s+"));
                    if (!parts.isEmpty()) {
                        curMat.diffuseTexturePath = objDir.absoluteFilePath(parts.last());
                    }
                }
            }
            if (!curMat.name.isEmpty())
                materials[curMat.name] = curMat;
            mtlFile.close();
        }

        // ============================================================
        // 5. 按 MaterialIds 分组拆分 cell，每个材质创建独立 Actor
        //    每个材质使用自己的 UV 数组作为 TCoords
        // ============================================================
        QVector<vtkSmartPointer<vtkActor>> actors;

        // 找出有多少个不同的 materialId
        QMap<int, std::vector<vtkIdType>> matIdToCells;
        for (vtkIdType ci = 0; ci < totalCells; ci++) {
            int mid = static_cast<int>(matIdArr->GetTuple1(ci));
            matIdToCells[mid].push_back(ci);
        }

        for (auto groupIt = matIdToCells.constBegin(); groupIt != matIdToCells.constEnd(); ++groupIt) {
            int matId = groupIt.key();
            const auto& cellIds = groupIt.value();

            // 获取该材质对应的 UV 数组名
            QString uvArrayName = (matId < uvArrayNames.size()) ? uvArrayNames[matId] : QString();
            vtkDataArray* matUV = uvArrayName.isEmpty() ? nullptr
                : fullPolyData->GetPointData()->GetArray(uvArrayName.toUtf8().constData());

            // 在 MTL 中查找对应材质（UV 数组名就是材质名）
            const ObjMaterial* mat = materials.contains(uvArrayName) ? &materials[uvArrayName] : nullptr;

            // 提取该材质的 cell
            vtkSmartPointer<vtkCellArray> matCells = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType cid : cellIds) {
                vtkCell* cell = fullPolyData->GetCell(cid);
                if (cell) matCells->InsertNextCell(cell);
            }
            if (matCells->GetNumberOfCells() == 0) continue;

            // 构建该材质的 polydata
            vtkSmartPointer<vtkPolyData> matPolyData = vtkSmartPointer<vtkPolyData>::New();
            matPolyData->SetPoints(fullPolyData->GetPoints());
            matPolyData->SetPolys(matCells);

            // 复制法线（如果有）
            vtkDataArray* normals = fullPolyData->GetPointData()->GetNormals();
            if (normals) {
                matPolyData->GetPointData()->SetNormals(normals);
            }

            // 设置该材质专属的 UV 数组作为 TCoords
            if (matUV) {
                matPolyData->GetPointData()->SetTCoords(matUV);
            }

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(matPolyData);
            mapper->ScalarVisibilityOff();

            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);

            // 设置材质底色
            if (mat) {
                actor->GetProperty()->SetColor(mat->kd[0], mat->kd[1], mat->kd[2]);
            }

            // 加载纹理
            if (mat && !mat->diffuseTexturePath.isEmpty()) {
                QString texPath = mat->diffuseTexturePath;
                QString suffix = QFileInfo(texPath).suffix().toLower();
                vtkSmartPointer<vtkImageReader2> imgReader;
                if (suffix == "png") imgReader = vtkSmartPointer<vtkPNGReader>::New();
                else if (suffix == "jpg" || suffix == "jpeg") imgReader = vtkSmartPointer<vtkJPEGReader>::New();
                else if (suffix == "bmp") imgReader = vtkSmartPointer<vtkBMPReader>::New();
                else if (suffix == "tiff" || suffix == "tif") imgReader = vtkSmartPointer<vtkTIFFReader>::New();
                else if (suffix == "tga") imgReader = vtkSmartPointer<vtkBMPReader>::New();

                if (imgReader) {
                    imgReader->SetFileName(texPath.toLocal8Bit().constData());
                    imgReader->Update();
                    if (imgReader->GetOutput() && imgReader->GetOutput()->GetNumberOfPoints() > 0) {
                        vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
                        texture->SetInputConnection(imgReader->GetOutputPort());
                        texture->InterpolateOn();
                        texture->RepeatOn();
                        texture->EdgeClampOn();
                        actor->SetTexture(texture);
                    }
                }
            }

            m_render->AddActor(actor);
            actors.push_back(actor);
        }

        if (!actors.isEmpty()) {
            m_textured_mesh_actors[id] = actors;
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::removeTexturedMesh(const QString& id)
    {
        auto it = m_textured_mesh_actors.find(id);
        if (it != m_textured_mesh_actors.end()) {
            for (const auto& actor : it.value()) {
                m_render->RemoveActor(actor);
            }
            m_textured_mesh_actors.erase(it);
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::setTextureMeshOpacity(const QString& cloud_id, float opacity)
    {
        auto it = m_textured_mesh_actors.find(cloud_id);
        if (it == m_textured_mesh_actors.end()) return;
        for (const auto& actor : it.value()) {
            if (actor) actor->GetProperty()->SetOpacity(opacity);
        }
    }

    void CloudView::setTextureMeshColor(const QString& cloud_id, float r, float g, float b)
    {
        auto it = m_textured_mesh_actors.find(cloud_id);
        if (it == m_textured_mesh_actors.end()) return;
        for (const auto& actor : it.value()) {
            if (!actor) continue;
            // 直接修改 polydata 顶点颜色，绕过 actor color 机制
            vtkPolyData* pd = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
            if (!pd) continue;
            vtkDataArray* arr = pd->GetPointData()->GetArray("MeshColors");
            if (!arr) continue;
            unsigned char cr = static_cast<unsigned char>(r * 255);
            unsigned char cg = static_cast<unsigned char>(g * 255);
            unsigned char cb = static_cast<unsigned char>(b * 255);
            for (vtkIdType i = 0; i < arr->GetNumberOfTuples(); ++i) {
                arr->SetTuple3(i, cr, cg, cb);
            }
            arr->Modified();
            actor->GetMapper()->Modified();
        }
    }

    void CloudView::setTextureMeshRepresentation(const QString& cloud_id, int type)
    {
        auto it = m_textured_mesh_actors.find(cloud_id);
        if (it == m_textured_mesh_actors.end()) return;
        for (const auto& actor : it.value()) {
            if (actor) {
                actor->GetProperty()->SetRepresentation(type);
                if (type == 0) actor->GetProperty()->SetPointSize(3);
            }
        }
    }

    bool CloudView::hasTextureMeshDisplayed(const QString& cloud_id) const
    {
        return m_textured_mesh_actors.contains(cloud_id);
    }

    void CloudView::removeMeshShapes(const QString& id)
    {
        std::string std_id = id.toStdString();
        if (m_viewer->contains(std_id))
            m_viewer->removeShape(std_id);
    }

    void CloudView::addPolylineFromPolygonMesh(const pcl::PolygonMesh::Ptr& mesh, const QString& id, int viewport)
    {
        std::string std_id = id.toStdString();
        if (m_viewer->contains(std_id))
            m_viewer->removeShape(std_id);
        m_viewer->addPolylineFromPolygonMesh(*mesh, std_id, viewport);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolylineFromCloud(const Cloud::Ptr& cloud, const QString& id,
                                          const ColorRGB& rgb)
    {
        if (!cloud || cloud->size() < 2) return;

        // 先清理已有的同名 shape
        removeShape(id);

        auto pts = cloud->toPCL_XYZ();
        size_t n = pts->size();

        // 最近邻链式排序
        std::vector<int> order;
        order.reserve(n);
        std::vector<bool> visited(n, false);

        // 构建 KD-tree 加速最近邻搜索
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pts);

        // 从第一个点开始
        order.push_back(0);
        visited[0] = true;

        pcl::PointXYZ search_pt = pts->points[0];
        for (size_t step = 1; step < n; ++step) {
            std::vector<int> indices(1);
            std::vector<float> dists(1);
            // 找最近未访问点
            int found = -1;
            float best_dist = std::numeric_limits<float>::max();
            // 搜索稍多的邻居以提高效率
            tree->nearestKSearch(search_pt, std::min<int>(n, 10), indices, dists);
            for (size_t j = 0; j < indices.size(); ++j) {
                if (!visited[indices[j]]) {
                    found = indices[j];
                    best_dist = dists[j];
                    break;
                }
            }
            // 如果 k 近邻中全部已访问，暴力搜索
            if (found < 0) {
                for (size_t j = 0; j < n; ++j) {
                    if (!visited[j]) {
                        float dx = pts->points[j].x - search_pt.x;
                        float dy = pts->points[j].y - search_pt.y;
                        float dz = pts->points[j].z - search_pt.z;
                        float d = dx*dx + dy*dy + dz*dz;
                        if (d < best_dist) {
                            best_dist = d;
                            found = static_cast<int>(j);
                        }
                    }
                }
            }
            if (found < 0) break;
            visited[found] = true;
            order.push_back(found);
            search_pt = pts->points[found];
        }

        if (order.size() < 2) return;

        // 构建 VTK polyline
        vtkSmartPointer<vtkPoints> vtk_pts = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
        lines->InsertNextCell(static_cast<int>(order.size()));
        for (int idx : order) {
            vtk_pts->InsertNextPoint(pts->points[idx].x, pts->points[idx].y, pts->points[idx].z);
            lines->InsertCellPoint(vtk_pts->GetNumberOfPoints() - 1);
        }

        vtkSmartPointer<vtkPolyData> line_pd = vtkSmartPointer<vtkPolyData>::New();
        line_pd->SetPoints(vtk_pts);
        line_pd->SetLines(lines);

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(line_pd);

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(rgb.rf(), rgb.gf(), rgb.bf());
        actor->GetProperty()->SetLineWidth(2.0);
        actor->GetProperty()->SetAmbient(1.0);
        actor->GetProperty()->SetDiffuse(0.0);

        m_render->AddActor(actor);

        // 存入 shape actors 以便 removeShape 统一管理
        QVector<vtkSmartPointer<vtkActor>> actors;
        actors.push_back(actor);
        m_textured_mesh_actors[id] = actors;

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addArrow(const ct::PointXYZRGBN &pt1, const ct::PointXYZRGBN &pt2, const QString &id,
                             bool display_length, const ct::ColorRGB &rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addArrow(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), display_length, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addArrow(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), display_length, id.toStdString());
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::addCube(const pcl::ModelCoefficients::Ptr &coefficients, const QString &id)
    {
        std::string std_id = id.toStdString();
        if (!m_viewer->contains(std_id))
            m_viewer->addCube(*coefficients, std_id);
        else
        {
            m_viewer->removeShape(std_id);
            m_viewer->addCube(*coefficients, std_id);
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCube(const ct::PointXYZRGBN &min, ct::PointXYZRGBN &max, const QString &id, const ct::ColorRGB &rgb)
    {
        std::string std_id = id.toStdString();
        if (!m_viewer->contains(std_id))
            m_viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, rgb.rf(), rgb.gf(), rgb.bf(), std_id);
        else
        {
            m_viewer->removeShape(std_id);
            m_viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, rgb.rf(), rgb.gf(), rgb.bf(), std_id);
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCube(const ct::Box &box, const QString &id)
    {
        std::string std_id = id.toStdString();
        if (!m_viewer->contains(std_id))
            m_viewer->addCube(box.translation, box.rotation, box.width, box.height, box.depth, std_id);
        else
        {
            m_viewer->removeShape(std_id);
            m_viewer->addCube(box.translation, box.rotation, box.width, box.height, box.depth, std_id);
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::add3DLabel(const PointXYZRGBN& pos, const QString& text,
                                const QString& id, double scale, double r, double g, double b)
    {
        std::string std_id = id.toStdString();
        pcl::PointXYZ pcl_pos;
        pcl_pos.x = pos.x;
        pcl_pos.y = pos.y;
        pcl_pos.z = pos.z;

        if (!m_viewer->contains(std_id))
            m_viewer->addText3D(text.toStdString(), pcl_pos, scale, r, g, b, std_id);
        else
        {
            m_viewer->removeShape(std_id);
            m_viewer->addText3D(text.toStdString(), pcl_pos, scale, r, g, b, std_id);
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::add3DBadge(const PointXYZRGBN& pos, const QString& text,
                                 const QString& id, int font_size,
                                 double textR, double textG, double textB,
                                 double bgR, double bgG, double bgB, double bgOpacity)
    {
        // 移除已有的
        if (m_badge_actors.contains(id)) {
            m_render->RemoveActor(m_badge_actors[id]);
            m_badge_actors.remove(id);
        }

        vtkSmartPointer<vtkBillboardTextActor3D> actor = vtkSmartPointer<vtkBillboardTextActor3D>::New();
        actor->SetInput(text.toStdString().c_str());
        actor->SetPosition(pos.x, pos.y, pos.z);

        vtkTextProperty* prop = actor->GetTextProperty();
        prop->SetFontSize(font_size);
        prop->SetBold(true);
        prop->SetColor(textR, textG, textB);
        prop->SetBackgroundColor(bgR, bgG, bgB);
        prop->SetBackgroundOpacity(bgOpacity);
        prop->SetFrame(true);
        prop->SetFrameWidth(2);
        prop->SetFrameColor(textR * 0.5, textG * 0.5, textB * 0.5);
        prop->SetJustificationToCentered();
        prop->SetFontFamilyToCourier();

        m_render->AddActor(actor);

        // 禁用深度测试，确保标签始终显示在点云上层不被遮挡
        // vtkBillboardTextActor3D 内部包含多个 vtkActor，需逐一设置
        vtkActorCollection* actors = vtkActorCollection::New();
        actor->GetActors(actors);
        actors->InitTraversal();
        while (vtkActor* a = actors->GetNextActor()) {
            if (a->GetMapper()) {
                a->GetMapper()->SetResolveCoincidentTopologyToPolygonOffset();
                a->GetMapper()->SetRelativeCoincidentTopologyPolygonOffsetParameters(-1, -1);
            }
            a->SetForceOpaque(true);
        }
        actors->Delete();

        m_badge_actors[id] = actor;

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::remove3DBadge(const QString& id)
    {
        if (m_badge_actors.contains(id)) {
            m_render->RemoveActor(m_badge_actors[id]);
            m_badge_actors.remove(id);
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::removeAll3DBadges()
    {
        for (auto it = m_badge_actors.begin(); it != m_badge_actors.end(); ++it)
            m_render->RemoveActor(it.value());
        m_badge_actors.clear();
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    PointXYZRGBN CloudView::displayToWorld(const PointXY &pos)
    {
        double point[4];
        m_render->SetDisplayPoint(pos.x, pos.y, 0.1);
        m_render->DisplayToWorld();
        m_render->GetWorldPoint(point);

        if (point[3] != 0.0) {
            point[0] /= point[3];
            point[1] /= point[3];
            point[2] /= point[3];
        }

        return PointXYZRGBN(point[0], point[1], point[2], 0, 0, 0);
    }

    void CloudView::addPolygon2D(const std::vector<PointXY> &points, const QString &id, const ct::ColorRGB &rgb)
    {
        Cloud::Ptr cloud(new Cloud);

        Box box; box.width=10; box.height=10; box.depth=10; // 虚拟 Box
        cloud->initOctree(box);

        for (auto& i : points)
        {
            PointXYZRGBN point = this->displayToWorld(i);
            cloud->addPoint(PointXYZ(point.x, point.y, point.z));
        }

        this->addPolygon(cloud, id, rgb);
    }

    // point pick
    PickResult CloudView::singlePick(const ct::PointXY &pos, const QString& target_cloud_id)
    {
        PickResult result;
        result.valid = false;

        vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
        picker->SetTolerance(0.005);

        // 1. 构建 Pick List
        picker->InitializePickList();

        // 收集自定义渲染器的 Actor
        for (auto it = m_OctreeRenders.begin(); it != m_OctreeRenders.end(); ++it) {
            if (!target_cloud_id.isEmpty() && it.key() != target_cloud_id) continue;
            std::vector<vtkActor*> actors = it.value()->getActiveActors();
            for (auto actor : actors) picker->AddPickList(actor);
        }

        picker->PickFromListOn();
        picker->Pick(pos.x, pos.y, 0.0, m_render);

        vtkIdType pointId = picker->GetPointId();
        vtkActor* actor = picker->GetActor();

        if (pointId != -1 && actor != nullptr) {
            for (auto it = m_OctreeRenders.begin(); it != m_OctreeRenders.end(); ++it) {
                auto block = it.value()->getBlockFromActor(actor);
                if (block) {
                    // 找到了！构建 PickResult
                    result.valid = true;
                    result.cloud = it.value()->getCloud(); // OctreeRenderer 需要提供 getCloud()

                    // 获取点数据
                    const auto& pt = block->m_points[pointId];
                    result.point.x = pt.x; result.point.y = pt.y; result.point.z = pt.z;

                    if (block->m_colors) {
                        const auto& c = (*block->m_colors)[pointId];
                        result.point.r = c.r; result.point.g = c.g; result.point.b = c.b;
                    } else {
                        result.point.r = 255; result.point.g = 255; result.point.b = 255;
                    }

                    if (block->m_normals) {
                        Eigen::Vector3f n = (*block->m_normals)[pointId].get();
                        result.point.normal_x = n.x(); result.point.normal_y = n.y(); result.point.normal_z = n.z();
                    } else {
                        result.point.normal_x = 0; result.point.normal_y = 0; result.point.normal_z = 0;
                    }

                    // 获取标量数据
                    if (!block->m_scalar_fields.empty()) {
                        for(auto sit = block->m_scalar_fields.begin(); sit != block->m_scalar_fields.end(); ++sit) {
                            result.scalars.insert(QString::fromStdString(sit->first), sit->second[pointId]);
                        }
                    }

                    return result;
                }
            }
        }
        return result; // valid = false
    }

    Cloud::Ptr CloudView::areaPick(const std::vector<PointXY> &poly_points, const Cloud::Ptr &cloud, bool in_out)
    {
        if (poly_points.size() < 3 || !cloud) return nullptr;

        Cloud::Ptr result_cloud(new Cloud);
        // 初始化结果云的八叉树 (使用原云的 BBox，或者计算新 BBox)
        result_cloud->initOctree(cloud->box());

        if (cloud->hasColors()) result_cloud->enableColors();
        if (cloud->hasNormals()) result_cloud->enableNormals();

        // 复制标量场定义
        std::vector<std::string> scalar_names = cloud->getScalarFieldNames();

        // 1. 预计算多边形参数 (PIP算法)
        int size = poly_points.size();
        std::vector<float> constant(size);
        std::vector<float> multiple(size);
        int i, j = size - 1;
        for (i = 0; i < size; i++) {
            if (poly_points[j].y == poly_points[i].y) {
                constant[i] = poly_points[i].x;
                multiple[i] = 0;
            } else {
                constant[i] = poly_points[i].x - (poly_points[i].y * poly_points[j].x) / (poly_points[j].y - poly_points[i].y) +
                              (poly_points[i].y * poly_points[i].x) / (poly_points[j].y - poly_points[i].y);
                multiple[i] = (poly_points[j].x - poly_points[i].x) / (poly_points[j].y - poly_points[i].y);
            }
            j = i;
        }

        auto worldToDisplay = [&](const pcl::PointXYZ& pt, double out[3]) {
            m_render->SetWorldPoint(pt.x, pt.y, pt.z, 1.0);
            m_render->WorldToDisplay();
            m_render->GetDisplayPoint(out);
        };

        const auto& blocks = cloud->getBlocks();

        // 准备批量添加的容器，减少 addPoints 调用次数
        std::vector<PointXYZ> batch_pts;
        std::vector<ColorRGB> batch_colors;
        std::vector<CompressedNormal> batch_normals;
        std::unordered_map<std::string, std::vector<float>> batch_scalars;

        size_t batch_limit = 50000;
        batch_pts.reserve(batch_limit);
        // ... reserve others ...

        for (const auto& block : blocks) {
            if (block->empty()) continue;

            // TODO: Block 级视锥体剔除 / 包围盒投影剔除 (优化点)

            size_t n = block->size();
            for (size_t k = 0; k < n; k++) {
                const auto& pt = block->m_points[k];
                double p[3];
                worldToDisplay(pt, p);

                bool oddNodes = in_out;
                bool current = poly_points[size - 1].y > p[1];
                bool previous;

                for (int m = 0; m < size; m++) {
                    previous = current;
                    current = poly_points[m].y > p[1];
                    if (current != previous)
                        oddNodes ^= (p[1] * multiple[m] + constant[m] < p[0]);
                }

                if (oddNodes) {
                    // 收集数据
                    batch_pts.push_back(pt);

                    if (cloud->hasColors() && block->m_colors)
                        batch_colors.push_back((*block->m_colors)[k]);

                    if (cloud->hasNormals() && block->m_normals)
                        batch_normals.push_back((*block->m_normals)[k]);

                    for (const auto& name : scalar_names) {
                        if (block->m_scalar_fields.count(name)) {
                            batch_scalars[name].push_back(block->m_scalar_fields[name][k]);
                        } else {
                            batch_scalars[name].push_back(0.0f);
                        }
                    }

                    // 批满提交
                    if (batch_pts.size() >= batch_limit) {
                        result_cloud->addPoints(batch_pts,
                                                batch_colors.empty() ? nullptr : &batch_colors,
                                                batch_normals.empty() ? nullptr : &batch_normals,
                                                batch_scalars.empty() ? nullptr : &batch_scalars);

                        batch_pts.clear(); batch_colors.clear(); batch_normals.clear();
                        for(auto& v : batch_scalars) v.second.clear();
                    }
                }
            }
        }

        // 提交剩余
        if (!batch_pts.empty()) {
            result_cloud->addPoints(batch_pts,
                                    batch_colors.empty() ? nullptr : &batch_colors,
                                    batch_normals.empty() ? nullptr : &batch_normals,
                                    batch_scalars.empty() ? nullptr : &batch_scalars);
        }

        result_cloud->update();
        return result_cloud;
    }

    ///////////////////////////////////////////////////////////////////
    // remove
    void CloudView::removePointCloud(const QString &id)
    {
        std::string sid = id.toStdString();
        auto it = std::remove_if(m_visible_clouds.begin(), m_visible_clouds.end(),
                                 [&](const Cloud::Ptr& cloud) { return cloud->id() == sid; });
        m_visible_clouds.erase(it, m_visible_clouds.end());

        m_OctreeRenders.remove(id);

        // TODO 这里清理的是什么？
        std::string preview_id = id.toStdString() + "_preview";
        if (m_viewer->contains(preview_id)) m_viewer->removePointCloud(preview_id);
        if (m_viewer->contains(id.toStdString())) m_viewer->removePointCloud(id.toStdString());

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeShape(const QString& id)
    {
        std::string std_id = id.toStdString();
        if (m_viewer->contains(std_id)){
            m_viewer->removeShape(std_id);
        }
        // 同时清理 m_textured_mesh_actors（折线等直接添加到 m_render 的 actor）
        auto it = m_textured_mesh_actors.find(id);
        if (it != m_textured_mesh_actors.end()) {
            for (const auto& actor : it.value()) {
                if (actor) m_render->RemoveActor(actor);
            }
            m_textured_mesh_actors.erase(it);
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removePolygonMesh(const QString& id, int viewport)
    {
	m_viewer->removePolygonMesh(id.toStdString(), viewport);
	if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeCorrespondences(const QString &id)
    {
        m_viewer->removeCorrespondences(id.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllPointClouds()
    {
        m_visible_clouds.clear();
        m_OctreeRenders.clear();
        m_viewer->removeAllPointClouds();
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllShapes()
    {
        for (auto it = m_textured_mesh_actors.begin(); it != m_textured_mesh_actors.end(); ++it) {
            for (const auto& actor : it.value()) {
                m_render->RemoveActor(actor);
            }
        }
        m_textured_mesh_actors.clear();

        m_viewer->removeAllShapes();
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCoordinateSystem(const Coord& coord)
    {
        QString qid = QString::fromStdString(coord.id);
        if (m_viewer->contains(qid.toStdString()))
            m_viewer->removeCoordinateSystem(qid.toStdString());
        m_viewer->addCoordinateSystem(coord.scale, coord.pose, qid.toStdString());
        m_coord_ids.insert(qid);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeCoordinateSystem(const QString& id)
    {
        m_viewer->removeCoordinateSystem(id.toStdString());
        m_coord_ids.remove(id);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllCoordinateSystems()
    {
        for (const auto& id : m_coord_ids)
            m_viewer->removeCoordinateSystem(id.toStdString());
        m_coord_ids.clear();
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

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
        for(auto& c : m_visible_clouds) {
            if (c->id() == sid) {
                setPointCloudColor(c, rgb);
                return;
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
        for(auto& c : m_visible_clouds) {
            if (c->id() == sid) {
                c->setPointSize(size); // 假设 Cloud 类有这个 setter

                // 强制触发一次 update，应用新属性
                if (m_OctreeRenders.contains(id)) m_OctreeRenders[id]->update();
                break;
            }
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudOpacity(const QString &id, float value)
    {
        std::string sid = id.toStdString();
        for(auto& c : m_visible_clouds) {
            if (c->id() == sid) {
                c->setOpacity(value);
                if (m_OctreeRenders.contains(id)) m_OctreeRenders[id]->update();
                break;
            }
        }

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setBackgroundColor(const ct::ColorRGB &rgb)
    {
        m_render->GradientBackgroundOff();
        m_viewer->setBackgroundColor(rgb.rf(), rgb.gf(), rgb.bf());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetBackgroundColor()
    {
        // 恢复渐变背景
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
        // 设置模型表示类型
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION, type, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeVisibility(const QString& id, bool visible)
    {
        // 切换 m_textured_mesh_actors 中的 actor 可见性
        auto it = m_textured_mesh_actors.find(id);
        if (it != m_textured_mesh_actors.end()) {
            for (const auto& actor : it.value()) {
                if (actor) actor->SetVisibility(visible ? 1 : 0);
            }
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
            return;
        }
        // 回退到 PCL viewer shape 系统
        std::string std_id = id.toStdString();
        if (m_viewer->contains(std_id)) {
            m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                  visible ? 1.0 : 0.0, std_id);
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::setPointCloudVisibility(const QString &id, bool visible) {
        //  控制 OctreeRenderer
        if (m_OctreeRenders.contains(id)) {
            m_OctreeRenders[id]->setVisibility(visible);  // 添加这行
            m_OctreeRenders[id]->update();
            // TODO 建议：在 OctreeRenderer::update() 中加一个 m_visible 标记

            // 控制 PCL 管理的辅助对象
            std::string std_id = id.toStdString();
            std::string normal_id = id.toStdString() + "-normals";
            std::string box_id = id.toStdString() + "-box";

            // PCL Helper
            if (m_viewer->contains(normal_id)) m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, visible?1.0:0.0, normal_id);

            // Shapes
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
        // 比较获取优先级，维护最大的信息级别
        m_info_level = std::max(m_info_level, level);
        m_active_infos[level] = {text, rgb};

        std::string id = INFO_TEXT + std::to_string(level);
        // 设置视图器中的显示信息
        if (!m_viewer->contains(id))
            m_viewer->addText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id);
        else
            m_viewer->updateText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id);
        // 当视图器大小改变时，同步改变信息位置

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
        // 如果m_viewer不包含ID为INFO_CLOUD_ID的文本,使用addText函数将点云ID作为文本添加到可视化窗口中,如果存在就更新显示文本
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
            // 调用m_viewer的removeShape函数来移除显示的形状
            m_viewer->removeShape(INFO_CLOUD_ID);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setInteractorEnable(const bool &enable)
    {
        /**
         * @brief vtkNew<> 是 VTK中提供的一个模板类，用于简化对象的创建和管理
         * 提供一个智能指针的便利性来管理 VTK 对象的生命周期，并确保在不再需要时释放占用的内存。
         */
        // 禁用交互器
        if (!enable)
        {
            vtkNew<PCLDisableInteractorStyle> style;
            m_renderwindow->GetInteractor()->SetInteractorStyle(style);
        }
        // 启用交互器
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
                new_x, new_y, new_z, // Eye,相机位置
                fp[0], fp[1], fp[2], // Target (焦点/看向哪里)
                up.x(), up.y(), up.z() // Up (头顶朝向)
                );

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
        syncViewCubeOrientation();
    }

    void CloudView::setTopView() {
        // 俯视图：相机在 +Z，看向中心，头顶朝 +Y
        setView(Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 1, 0));
    }

    void CloudView::setBottomView() {
        // 底视图：相机在 -Z，看向中心，头顶朝 +Y
        setView(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f(0, 1, 0));
    }

    void CloudView::setFrontView() {
        // 正视图：通常定义为从 -Y 看向 +Y (或者从 +Y 看向 -Y，取决于你的坐标系习惯)
        // 这里假设 Z 是高，Y 是深。从 -Y 处看过去。头顶朝 +Z。
        setView(Eigen::Vector3f(0, -1, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setBackView() {
        // 后视图：相机在 +Y，头顶朝 +Z
        setView(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setLeftSideView() {
        // 左视图：相机在 -X，头顶朝 +Z
        setView(Eigen::Vector3f(-1, 0, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setRightSideView() {
        // 右视图：相机在 +X，头顶朝 +Z
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
        // 1. 更新所有 OctreeRenderer (确保可见性等状态正确应用)
        updateRenderers();

        // 2. 重置相机的 Clipping Range (防止物体被裁剪)
        m_render->ResetCameraClippingRange();

        // 3. 强制重绘
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
//        return QVTKOpenGLNativeWidget::mousePressEvent(event);
    }

    void CloudView::mouseReleaseEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            // getViewerPose() 是 PCLVisualizer 类中的一个成员函数。它返回当前视图器的视角或位置的描述，例如相机的位置和方向。
            emit viewerPose(m_viewer->getViewerPose());
            emit mouseLeftReleased(PointXY(this->interactor()->GetEventPosition()[0],
                                           this->interactor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightReleased(PointXY(this->interactor()->GetEventPosition()[0],
                                            this->interactor()->GetEventPosition()[1]));
        }
//        return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
    }

    void CloudView::mouseMoveEvent(QMouseEvent *event)
    {
        emit mouseMoved(PointXY(this->interactor()->GetEventPosition()[0],
                                this->interactor()->GetEventPosition()[1]));
//        return QVTKOpenGLNativeWidget::mouseMoveEvent(event);
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
        // ViewCube uses to-eye direction (pos - fp), not look direction (fp - pos)
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

        // 背景色
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

        // 背景色
        if (opts.use_gradient_bg) {
            m_render->GradientBackgroundOn();
            m_render->SetBackground2(opts.bg_color2[0], opts.bg_color2[1], opts.bg_color2[2]);
        } else {
            m_render->GradientBackgroundOff();
        }
        m_render->SetBackground(opts.bg_color[0], opts.bg_color[1], opts.bg_color[2]);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

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

} // namespace ct
