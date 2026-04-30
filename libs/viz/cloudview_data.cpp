#include "cloudview.h"

#include <vtkPointPicker.h>
#include <vtkBillboardTextActor3D.h>
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
#include <vtkUnsignedCharArray.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/search/kdtree.h>

#include <QRegularExpression>
#include <QFileInfo>
#include <QDir>
#include <QFile>
#include <QTextStream>

#include <cmath>
#include <limits>

namespace ct
{

    ///////////////////////////////////////////////////////////////////
    // add
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

        auto renderer = std::make_shared<OctreeRenderer>(cloud, m_render);
        m_OctreeRenders.insert(qid, renderer);

        renderer->invalidateCache();
        renderer->update();

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudFromRangeImage(const pcl::RangeImage::Ptr &image, const QString &id, const ct::ColorRGB &rgb)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color(image, rgb.r, rgb.g, rgb.b);
        if (!m_viewer->contains(id.toStdString()))
        {
            m_viewer->addPointCloud(image, range_image_color, id.toStdString());
        }
        else
            m_viewer->updatePointCloud(image, range_image_color, id.toStdString());
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
        std::string id = cloud->normalId();

        if (!cloud->hasNormals()) return;

        const size_t MAX_NORMAL_POINTS = 50000;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        size_t total = cloud->size();
        size_t step = (total > MAX_NORMAL_POINTS) ? (total / MAX_NORMAL_POINTS) : 1;

        temp_cloud->reserve(MAX_NORMAL_POINTS);

        const auto& blocks = cloud->getBlocks();
        size_t global_idx = 0;

        for (const auto& block : blocks) {
            if (block->empty()) continue;

            size_t n = block->size();
            const auto& pts = block->m_points;
            const auto* colors = (block->m_colors) ? block->m_colors.get() : nullptr;
            const auto* norms = (block->m_normals) ? block->m_normals.get() : nullptr;

            if (!norms) continue;

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

        if (m_viewer->contains(std_id))
            m_viewer->removeShape(std_id);

        auto srcPCL = source_points->toPCL_XYZRGB();
        auto tgtPCL = target_points->toPCL_XYZRGB();

        if (!m_viewer->contains(std_id))
            m_viewer->addCorrespondences<pcl::PointXYZRGB>(srcPCL, tgtPCL, *correspondences, std_id);
        else
            m_viewer->updateCorrespondences<pcl::PointXYZRGB>(srcPCL, tgtPCL, *correspondences, std_id);

        if (line_width > 1) {
            auto shape_map = m_viewer->getShapeActorMap();
            for (int i = 0; i < (int)correspondences->size(); i++) {
                auto it = shape_map->find(std_id + "_line_" + std::to_string(i));
                if (it != shape_map->end()) {
                    auto* actor = vtkActor::SafeDownCast(it->second);
                    if (actor && actor->GetProperty()) {
                        actor->GetProperty()->SetLineWidth(line_width);
                        actor->GetProperty()->SetColor(1.0, 0.9, 0.0);
                    }
                }
            }
        }

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolygon(const Cloud::Ptr &cloud, const QString &id, const ct::ColorRGB &rgb)
    {
        std::string std_id = id.toStdString();
        auto pclCloud = cloud->toPCL_XYZRGB();

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
        removeTexturedMesh(id);

        if (!mesh || mesh->polygons.empty()) return;

        vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
        pcl::VTKUtils::mesh2vtk(*mesh, polydata);

        if (!polydata || polydata->GetNumberOfPoints() == 0) return;

        polydata->GetPointData()->Initialize();

        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetNumberOfTuples(polydata->GetNumberOfPoints());
        unsigned char gray = 204;
        for (vtkIdType i = 0; i < polydata->GetNumberOfPoints(); ++i) {
            colors->SetTuple3(i, gray, gray, gray);
        }
        colors->SetName("MeshColors");
        polydata->GetPointData()->SetScalars(colors);

        vtkSmartPointer<vtkPolyDataNormals> normalGen = vtkSmartPointer<vtkPolyDataNormals>::New();
        normalGen->SetInputData(polydata);
        normalGen->ConsistencyOn();
        normalGen->AutoOrientNormalsOn();
        normalGen->NonManifoldTraversalOff();
        normalGen->Update();
        polydata = normalGen->GetOutput();

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
        // ============================================================
        vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName(objFilePath.toLocal8Bit().constData());
        reader->Update();
        vtkPolyData* fullPolyData = reader->GetOutput();
        if (!fullPolyData || fullPolyData->GetNumberOfPoints() == 0) return;

        vtkIdType totalCells = fullPolyData->GetNumberOfCells();

        // ============================================================
        // 2. 收集 pointData 中的 per-material UV 数组
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
        uvArrayNames.sort();

        // ============================================================
        // 3. 获取 cellData 中的 MaterialIds
        // ============================================================
        vtkDataArray* matIdArr = fullPolyData->GetCellData()->GetArray("MaterialIds");
        if (!matIdArr) {
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
        // 4. 解析 MTL 文件
        // ============================================================
        struct ObjMaterial {
            QString name;
            QString diffuseTexturePath;
            float kd[3] = {0.8f, 0.8f, 0.8f};
        };

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
        // ============================================================
        QVector<vtkSmartPointer<vtkActor>> actors;

        QMap<int, std::vector<vtkIdType>> matIdToCells;
        for (vtkIdType ci = 0; ci < totalCells; ci++) {
            int mid = static_cast<int>(matIdArr->GetTuple1(ci));
            matIdToCells[mid].push_back(ci);
        }

        for (auto groupIt = matIdToCells.constBegin(); groupIt != matIdToCells.constEnd(); ++groupIt) {
            int matId = groupIt.key();
            const auto& cellIds = groupIt.value();

            QString uvArrayName = (matId < uvArrayNames.size()) ? uvArrayNames[matId] : QString();
            vtkDataArray* matUV = uvArrayName.isEmpty() ? nullptr
                : fullPolyData->GetPointData()->GetArray(uvArrayName.toUtf8().constData());

            const ObjMaterial* mat = materials.contains(uvArrayName) ? &materials[uvArrayName] : nullptr;

            vtkSmartPointer<vtkCellArray> matCells = vtkSmartPointer<vtkCellArray>::New();
            for (vtkIdType cid : cellIds) {
                vtkCell* cell = fullPolyData->GetCell(cid);
                if (cell) matCells->InsertNextCell(cell);
            }
            if (matCells->GetNumberOfCells() == 0) continue;

            vtkSmartPointer<vtkPolyData> matPolyData = vtkSmartPointer<vtkPolyData>::New();
            matPolyData->SetPoints(fullPolyData->GetPoints());
            matPolyData->SetPolys(matCells);

            vtkDataArray* normals = fullPolyData->GetPointData()->GetNormals();
            if (normals) {
                matPolyData->GetPointData()->SetNormals(normals);
            }

            if (matUV) {
                matPolyData->GetPointData()->SetTCoords(matUV);
            }

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(matPolyData);
            mapper->ScalarVisibilityOff();

            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);

            if (mat) {
                actor->GetProperty()->SetColor(mat->kd[0], mat->kd[1], mat->kd[2]);
            }

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

        removeShape(id);

        auto pts = cloud->toPCL_XYZ();
        size_t n = pts->size();

        std::vector<int> order;
        order.reserve(n);
        std::vector<bool> visited(n, false);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pts);

        order.push_back(0);
        visited[0] = true;

        pcl::PointXYZ search_pt = pts->points[0];
        for (size_t step = 1; step < n; ++step) {
            std::vector<int> indices(1);
            std::vector<float> dists(1);
            int found = -1;
            float best_dist = std::numeric_limits<float>::max();
            tree->nearestKSearch(search_pt, std::min<int>(n, 10), indices, dists);
            for (size_t j = 0; j < indices.size(); ++j) {
                if (!visited[indices[j]]) {
                    found = indices[j];
                    best_dist = dists[j];
                    break;
                }
            }
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

    ////////////////////////////////////////////////////////
    // 2D->3D(display to world)
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

        Box box; box.width=10; box.height=10; box.depth=10;
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

        picker->InitializePickList();

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
                    result.valid = true;
                    result.cloud = it.value()->getCloud();

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

                    if (!block->m_scalar_fields.empty()) {
                        for(auto sit = block->m_scalar_fields.begin(); sit != block->m_scalar_fields.end(); ++sit) {
                            result.scalars.insert(QString::fromStdString(sit->first), sit->second[pointId]);
                        }
                    }

                    return result;
                }
            }
        }
        return result;
    }

    Cloud::Ptr CloudView::areaPick(const std::vector<PointXY> &poly_points, const Cloud::Ptr &cloud, bool in_out)
    {
        if (poly_points.size() < 3 || !cloud) return nullptr;

        Cloud::Ptr result_cloud(new Cloud);
        result_cloud->initOctree(cloud->box());

        if (cloud->hasColors()) result_cloud->enableColors();
        if (cloud->hasNormals()) result_cloud->enableNormals();

        std::vector<std::string> scalar_names = cloud->getScalarFieldNames();

        // PIP算法参数预计算
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

        std::vector<PointXYZ> batch_pts;
        std::vector<ColorRGB> batch_colors;
        std::vector<CompressedNormal> batch_normals;
        std::unordered_map<std::string, std::vector<float>> batch_scalars;

        size_t batch_limit = 50000;
        batch_pts.reserve(batch_limit);

        for (const auto& block : blocks) {
            if (block->empty()) continue;

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

        if (!batch_pts.empty()) {
            result_cloud->addPoints(batch_pts,
                                    batch_colors.empty() ? nullptr : &batch_colors,
                                    batch_normals.empty() ? nullptr : &batch_normals,
                                    batch_scalars.empty() ? nullptr : &batch_scalars);
        }

        result_cloud->update();
        return result_cloud;
    }

    std::pair<ct::Cloud::Ptr, ct::Cloud::Ptr>
    CloudView::areaPickSplit(const std::vector<PointXY>& poly_points,
                              const Cloud::Ptr& cloud, bool in_out)
    {
        if (poly_points.size() < 3 || !cloud) return {nullptr, nullptr};

        Cloud::Ptr inside_cloud(new Cloud);
        Cloud::Ptr outside_cloud(new Cloud);
        inside_cloud->initOctree(cloud->box());
        outside_cloud->initOctree(cloud->box());

        if (cloud->hasColors()) { inside_cloud->enableColors(); outside_cloud->enableColors(); }
        if (cloud->hasNormals()) { inside_cloud->enableNormals(); outside_cloud->enableNormals(); }

        std::vector<std::string> scalar_names = cloud->getScalarFieldNames();

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

        std::vector<PointXYZ> in_pts, out_pts;
        std::vector<ColorRGB> in_colors, out_colors;
        std::vector<CompressedNormal> in_normals, out_normals;
        std::unordered_map<std::string, std::vector<float>> in_scalars, out_scalars;

        size_t batch_limit = 50000;
        in_pts.reserve(batch_limit);
        out_pts.reserve(batch_limit);

        auto flushBatch = [&](Cloud::Ptr& target, std::vector<PointXYZ>& pts,
                              std::vector<ColorRGB>& colors,
                              std::vector<CompressedNormal>& normals,
                              std::unordered_map<std::string, std::vector<float>>& scalars) {
            target->addPoints(pts,
                              colors.empty() ? nullptr : &colors,
                              normals.empty() ? nullptr : &normals,
                              scalars.empty() ? nullptr : &scalars);
            pts.clear(); colors.clear(); normals.clear();
            for (auto& v : scalars) v.second.clear();
        };

        for (const auto& block : blocks) {
            if (block->empty()) continue;

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

                auto& dst_pts = oddNodes ? in_pts : out_pts;
                auto& dst_colors = oddNodes ? in_colors : out_colors;
                auto& dst_normals = oddNodes ? in_normals : out_normals;
                auto& dst_scalars = oddNodes ? in_scalars : out_scalars;

                dst_pts.push_back(pt);

                if (cloud->hasColors() && block->m_colors)
                    dst_colors.push_back((*block->m_colors)[k]);

                if (cloud->hasNormals() && block->m_normals)
                    dst_normals.push_back((*block->m_normals)[k]);

                for (const auto& name : scalar_names) {
                    if (block->m_scalar_fields.count(name))
                        dst_scalars[name].push_back(block->m_scalar_fields[name][k]);
                    else
                        dst_scalars[name].push_back(0.0f);
                }

                if (dst_pts.size() >= batch_limit)
                    flushBatch(oddNodes ? inside_cloud : outside_cloud,
                               dst_pts, dst_colors, dst_normals, dst_scalars);
            }
        }

        if (!in_pts.empty())
            flushBatch(inside_cloud, in_pts, in_colors, in_normals, in_scalars);
        if (!out_pts.empty())
            flushBatch(outside_cloud, out_pts, out_colors, out_normals, out_scalars);

        inside_cloud->update();
        outside_cloud->update();
        return {inside_cloud, outside_cloud};
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

} // namespace ct
