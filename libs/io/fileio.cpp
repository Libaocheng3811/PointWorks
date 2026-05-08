#include "fileio.h"

#include "pcl/io/obj_io.h"
#include "pcl/io/ply_io.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkSTLWriter.h>
#include <vtkPolyDataWriter.h>

#include <QFileInfo>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>

#include <string>
#include <cfloat>

namespace pw
{

// ================================================================
// 调度器：加载点云 / 模型
// ================================================================

void FileIO::loadPointCloud(const QString &filename) {
    TicToc time;
    time.tic();
    m_is_canceled = false;
    m_textured_mesh.reset(); // 重置纹理网格
    emit progress(0);

    Cloud::Ptr cloud(new Cloud);
    pcl::PolygonMesh::Ptr mesh; // 尝试加载面片
    QFileInfo fileInfo(filename);
    QString suffix = fileInfo.suffix().toLower();
    bool is_success = false;

    //根据后缀分发处理
    if (suffix == "las" || suffix == "laz"){
        is_success = loadLAS(filename, cloud);
    }
    else if (suffix == "e57"){
        is_success = loadE57(filename, cloud);
    }
    else if (suffix == "ply" || suffix == "pcd"){
        is_success = loadPLY_PCD(filename, cloud);
        // PLY 文件可能包含面片，尝试作为 mesh 加载
        if (is_success && suffix == "ply") {
            mesh.reset(new pcl::PolygonMesh);
            if (pcl::io::loadPLYFile(filename.toLocal8Bit().toStdString(), *mesh) == 0
                && mesh->polygons.empty()) {
                mesh.reset(); // 没有面片，置空
            }
        }
    }
    else if (suffix == "txt" || suffix == "xyz" || suffix == "asc"){
        is_success = loadTXT(filename, cloud);
    }
    else{
        is_success = loadGeneralPCL(filename, cloud, mesh);
    }

    // 失败处理
    if (!is_success || m_is_canceled){
        QString errorMsg = m_is_canceled ? "Operation canceled by user"
            : QString("Failed to load '%1': unknown error").arg(filename);
        emit loadCloudResult(false, cloud, mesh, time.toc(), errorMsg);
        return;
    }

    emit progress(90);

    cloud->setId(fileInfo.baseName().toStdString());
    cloud->setFilepath(fileInfo.absoluteFilePath().toStdString());
    cloud->update(); //更新包围盒，统计信息

    emit progress(100);

    // 有纹理 mesh 时：先通过 loadCloudResult 正常传递 mesh，再通知纹理路径
    if (m_textured_mesh && *m_textured_mesh) {
        emit loadCloudResult(true, cloud, mesh, time.toc());
        emit loadTexturedMeshResult(
            QString::fromStdString(cloud->id()),
            QString::fromStdString(m_textured_mesh->objFilePath));
        m_textured_mesh.reset();
    } else {
        emit loadCloudResult(true, cloud, mesh, time.toc());
    }
}

// ================================================================
// 调度器：保存点云
// ================================================================

void FileIO::savePointCloud(const Cloud::Ptr &cloud, const QString &filename, bool isBinary) {
    TicToc time;
    time.tic();
    QFileInfo fileInfo(filename);
    QString suffix = fileInfo.suffix().toLower();
    bool is_success = false;

    // 恢复原始数据
    cloud->restoreColors();

    //检查是否存在偏移
    Eigen::Vector3d shift = cloud->getGlobalShift();
    bool has_shift = (shift != Eigen::Vector3d::Zero());

    if (suffix == "las" || suffix == "laz"){
        is_success = saveLAS(cloud, filename);
    }
    else if (suffix == "e57"){
        is_success = saveE57(cloud, filename);
    }
    else if (suffix == "txt" || suffix == "xyz" || suffix == "csv"){
        is_success = saveTXT(cloud, filename);
    }
    else {
        is_success = savePCL(cloud, filename, isBinary);
    }

    if (is_success) emit saveCloudResult(true, filename, time.toc());
    else emit saveCloudResult(false, filename, time.toc(),
             QString("Failed to save '%1'").arg(filename));
}

// ================================================================
// 调度器：保存 Mesh
// ================================================================

void FileIO::saveMesh(const pcl::PolygonMesh::Ptr &mesh, const QString &filename) {
    TicToc time;
    time.tic();

    bool is_success = saveMeshFile(mesh, filename);

    if (is_success) emit saveMeshResult(true, filename, time.toc());
    else emit saveMeshResult(false, filename, time.toc(),
             QString("Failed to save mesh '%1'").arg(filename));
}

bool FileIO::saveMeshFile(const pcl::PolygonMesh::Ptr &mesh, const QString &filename) {
    if (!mesh || mesh->polygons.empty()) return false;

    QFileInfo fileInfo(filename);
    QString suffix = fileInfo.suffix().toLower();
    std::string path = filename.toLocal8Bit().toStdString();

    emit progress(20);

    int res = -1;
    if (suffix == "obj") {
        res = pcl::io::saveOBJFile(path, *mesh);
    } else if (suffix == "stl") {
        // 使用 VTK 直接写 STL（更可靠）
        vtkSmartPointer<vtkPolyData> polydata;
        pcl::VTKUtils::mesh2vtk(*mesh, polydata);
        vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
        writer->SetFileName(path.c_str());
        writer->SetInputData(polydata);
        writer->SetFileTypeToBinary();
        res = writer->Write() ? 0 : -1;
    } else if (suffix == "vtk") {
        // 使用 VTK 直接写 VTK 格式
        vtkSmartPointer<vtkPolyData> polydata;
        pcl::VTKUtils::mesh2vtk(*mesh, polydata);
        vtkSmartPointer<vtkPolyDataWriter> writer = vtkSmartPointer<vtkPolyDataWriter>::New();
        writer->SetFileName(path.c_str());
        writer->SetInputData(polydata);
        writer->SetFileTypeToBinary();
        res = writer->Write() ? 0 : -1;
    } else {
        // 默认使用 PLY 格式保存 mesh（带面片）
        res = pcl::io::savePLYFile(path, *mesh);
    }

    emit progress(100);
    return res == 0;
}

// ================================================================
// 纹理加载辅助函数
// ================================================================

QString FileIO::parseOBJMaterialTexture(const QString &objPath)
{
    QFileInfo objInfo(objPath);
    QDir objDir = objInfo.absoluteDir();

    // 解析 OBJ 文件的 mtllib 行
    QFile objFile(objPath);
    if (!objFile.open(QIODevice::ReadOnly | QIODevice::Text)) return {};

    QString mtlFileName;
    QTextStream in(&objFile);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.startsWith("mtllib ", Qt::CaseInsensitive)) {
            mtlFileName = line.mid(7).trimmed();
            break;
        }
    }
    objFile.close();

    if (mtlFileName.isEmpty()) return {};

    // 解析 MTL 文件的 map_Kd（漫反射贴图）
    QString mtlPath = objDir.absoluteFilePath(mtlFileName);
    QFile mtlFile(mtlPath);
    if (!mtlFile.open(QIODevice::ReadOnly | QIODevice::Text)) return {};

    QTextStream mtlIn(&mtlFile);
    while (!mtlIn.atEnd()) {
        QString line = mtlIn.readLine().trimmed();
        if (line.startsWith("map_Kd", Qt::CaseInsensitive)) {
            QString texPart = line.mid(6).trimmed();
            // 取最后一个 token（处理内联选项如 "-bm 1.0 texture.png"）
            QStringList parts = texPart.split(QRegularExpression("\\s+"));
            if (!parts.isEmpty()) {
                return objDir.absoluteFilePath(parts.last());
            }
        }
    }

    return {};
}

} // namespace pw
