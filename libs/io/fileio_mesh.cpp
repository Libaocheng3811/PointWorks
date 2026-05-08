#include "fileio.h"

#include "pcl/io/ifs_io.h"
#include "pcl/io/obj_io.h"
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkSTLReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyData.h>

#include <QFileInfo>

#include <cfloat>

namespace pw
{

// ================================================================
// 通用模型加载 (OBJ, IFS, STL, VTK)
// ================================================================

bool FileIO::loadGeneralPCL(const QString &filename, Cloud::Ptr &cloud, pcl::PolygonMesh::Ptr &mesh) {
    emit progress(5);

    std::string path = filename.toLocal8Bit().toStdString();

    // 使用带法线和颜色的通用点类型加载顶点
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    int res = -1;
    if (filename.endsWith(".obj", Qt::CaseInsensitive)) {
        // 使用 VTK 的 OBJ Reader（比 PCL 的实现更健壮，能正确处理缺失材质文件）
        vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName(path.c_str());
        reader->Update();
        vtkPolyData* polydata = reader->GetOutput();
        if (polydata->GetNumberOfPoints() > 0) {
            pcl::PolygonMesh::Ptr obj_mesh(new pcl::PolygonMesh);
            pcl::VTKUtils::vtk2mesh(polydata, *obj_mesh);
            pcl::fromPCLPointCloud2(obj_mesh->cloud, *temp_cloud);
            if (!obj_mesh->polygons.empty()) {
                mesh = obj_mesh;
            }
            res = 0;

            // 检测 OBJ 是否引用了材质（有 MTL + map_Kd 即视为有纹理）
            QString textureImagePath = parseOBJMaterialTexture(filename);
            if (!textureImagePath.isEmpty()) {
                m_textured_mesh = std::make_shared<TexturedMesh>();
                m_textured_mesh->mesh = obj_mesh;
                m_textured_mesh->objFilePath = filename.toLocal8Bit().toStdString();
            }
        }
    }
    else if (filename.endsWith(".ifs", Qt::CaseInsensitive))
        res = pcl::io::loadIFSFile(path, *temp_cloud);
    else if (filename.endsWith(".stl", Qt::CaseInsensitive)) {
        // STL 只有面片，使用 VTK 读取
        vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
        reader->SetFileName(path.c_str());
        reader->Update();
        vtkPolyData* polydata = reader->GetOutput();
        if (polydata->GetNumberOfCells() > 0) {
            pcl::PolygonMesh::Ptr stl_mesh(new pcl::PolygonMesh);
            pcl::VTKUtils::vtk2mesh(polydata, *stl_mesh);
            pcl::fromPCLPointCloud2(stl_mesh->cloud, *temp_cloud);
            mesh = stl_mesh;
            res = 0;
        }
    }
    else if (filename.endsWith(".vtk", Qt::CaseInsensitive)) {
        // VTK 使用 VTK 库直接读取
        vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
        reader->SetFileName(path.c_str());
        reader->Update();
        vtkPolyData* polydata = reader->GetOutput();
        if (polydata->GetNumberOfCells() > 0) {
            pcl::PolygonMesh::Ptr vtk_mesh(new pcl::PolygonMesh);
            pcl::VTKUtils::vtk2mesh(polydata, *vtk_mesh);
            pcl::fromPCLPointCloud2(vtk_mesh->cloud, *temp_cloud);
            mesh = vtk_mesh;
            res = 0;
        }
    }

    if (res == -1 || temp_cloud->empty()) return false;

    emit progress(50);

    size_t num_points = temp_cloud->size();

    const auto& p0 = temp_cloud->points[0];
    const double THRESHOLD_XY = 10000.0;

    // 计算 Global Shift
    Eigen::Vector3d suggested_shift = Eigen::Vector3d::Zero();
    if (std::abs(p0.x) > THRESHOLD_XY || std::abs(p0.y) > THRESHOLD_XY) {
        double sx = -std::floor(p0.x / 1000.0) * 1000.0;
        double sy = -std::floor(p0.y / 1000.0) * 1000.0;
        double sz = 0.0;

        suggested_shift = Eigen::Vector3d(sx, sy, sz);

        bool skipped = false;
        emit requestGlobalShift(Eigen::Vector3d(p0.x, p0.y, p0.z), suggested_shift, skipped);

        if (!skipped) {
            cloud->setGlobalShift(-suggested_shift);
        }
    }

    float shift_x = (float)suggested_shift.x();
    float shift_y = (float)suggested_shift.y();
    float shift_z = (float)suggested_shift.z();

    // 预计算包围盒 (手动循环)
    PointXYZ min_pt(FLT_MAX, FLT_MAX, FLT_MAX);
    PointXYZ max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const auto& p : temp_cloud->points) {
        float x = p.x + shift_x;
        float y = p.y + shift_y;
        float z = p.z + shift_z;

        if (x < min_pt.x) min_pt.x = x;
        if (x > max_pt.x) max_pt.x = x;
        if (y < min_pt.y) min_pt.y = y;
        if (y > max_pt.y) max_pt.y = y;
        if (z < min_pt.z) min_pt.z = z;
        if (z > max_pt.z) max_pt.z = z;
    }

    Box box;
    box.width  = (max_pt.x - min_pt.x) * 1.01;
    box.height = (max_pt.y - min_pt.y) * 1.01;
    box.depth  = (max_pt.z - min_pt.z) * 1.01;
    box.translation = Eigen::Vector3f(
            (min_pt.x + max_pt.x) * 0.5f,
            (min_pt.y + max_pt.y) * 0.5f,
            (min_pt.z + max_pt.z) * 0.5f
    );
    cloud->initOctree(box);

    // 准备批处理
    // 假设 OBJ 都有颜色和法线 (temp_cloud 类型决定了它有这些字段，即使值为0)
    cloud->enableColors();
    cloud->enableNormals();

    CloudBatch batch;
    batch.reserve(BATCH_SIZE);

    int progress_interval = (num_points > 100) ? (num_points / 100) : 1;

    for (size_t i = 0; i < num_points; ++i) {
        if (m_is_canceled) return false;

        const auto& src = temp_cloud->points[i];

        // XYZ
        batch.points.emplace_back(src.x + shift_x, src.y + shift_y, src.z + shift_z);

        // Color
        batch.colors.emplace_back(src.r, src.g, src.b);

        // Normal
        CompressedNormal cn;
        cn.set(Eigen::Vector3f(src.normal_x, src.normal_y, src.normal_z));
        batch.normals.push_back(cn);

        // 提交批次
        if (batch.points.size() >= BATCH_SIZE) {
            batch.flushTo(cloud);
        }

        if (i % progress_interval == 0) {
            int p = 50 + (int)(i * 50 / num_points);
            emit progress(p);
        }
    }

    if (!batch.empty()) batch.flushTo(cloud);

    cloud->setHasColors(true);
    cloud->setHasNormals(true);

    cloud->makeAdaptive();
    emit progress(100);

    return true;
}

} // namespace pw
