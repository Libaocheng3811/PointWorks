#include "cloud_io_controller.h"
#include "progress_manager.h"

#include "io/fileio.h"

#include <QTreeWidgetItem>

#include "dialog/fieldmappingdialog.h"
#include "dialog/txtimportdialog.h"
#include "dialog/txtexportdialog.h"
#include "dialog/globalshiftdialog.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QMetaType>

namespace pw
{

CloudIOController::CloudIOController(QObject* parent)
    : QObject(parent), m_parent_widget(nullptr), m_progress(nullptr),
      m_fileio(nullptr), m_hasLastShift(false), m_script_mode(false)
{}

CloudIOController::~CloudIOController()
{
    m_thread.quit();
    if (!m_thread.wait(3000))
    {
        m_thread.terminate();
        m_thread.wait();
    }
}

void CloudIOController::init(QWidget* parentWidget, ProgressManager* progress)
{
    m_parent_widget = parentWidget;
    m_progress = progress;

    qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr");
    qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr &");
    qRegisterMetaType<pcl::PolygonMesh::Ptr>("pcl::PolygonMesh::Ptr");
    qRegisterMetaType<QList<pw::FieldInfo>>("QList<pw::FieldInfo>");
    qRegisterMetaType<QMap<QString, QString>>("QMap<QString, QString>&");
    qRegisterMetaType<pw::TxtImportParams>("pw::TxtImportParams");
    qRegisterMetaType<pw::TxtExportParams>("pw::TxtExportParams");
    qRegisterMetaType<pw::TexturedMeshPtr>("pw::TexturedMeshPtr");
    qRegisterMetaType<std::vector<int>>("std::vector<int>");

    m_fileio = new FileIO;
    m_fileio->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_fileio, &QObject::deleteLater);

    connect(m_fileio, &FileIO::loadCloudResult, this, &CloudIOController::onLoadCloudResult);
    connect(m_fileio, &FileIO::loadTexturedMeshResult, this, &CloudIOController::onLoadTexturedMeshResult);
    connect(m_fileio, &FileIO::saveCloudResult, this, &CloudIOController::onSaveCloudResult);
    connect(m_fileio, &FileIO::saveMeshResult, this, &CloudIOController::onSaveMeshResult);

    connect(this, &CloudIOController::cloudLoaded, this, [](const Cloud::Ptr&, const pcl::PolygonMesh::Ptr&,
             QTreeWidgetItem*, float, const QString&) {});
    connect(this, &CloudIOController::texturedMeshLoaded, this, [](const QString&, const QString&) {});

    connect(m_fileio, &FileIO::requestFieldMapping, this, &CloudIOController::onFieldMappingRequested, Qt::BlockingQueuedConnection);
    connect(m_fileio, &FileIO::requestTxtImportSetup, this, &CloudIOController::onTxtImportRequested, Qt::BlockingQueuedConnection);
    connect(m_fileio, &FileIO::requestTxtExportSetup, this, &CloudIOController::onTxtExportRequested, Qt::BlockingQueuedConnection);
    connect(m_fileio, &FileIO::requestGlobalShift, this, &CloudIOController::onGlobalFilterRequested, Qt::BlockingQueuedConnection);

    m_thread.start();
}

void CloudIOController::addCloud()
{
    QString pointCloudFilters = "Point Cloud Files (*.ply *.pcd *.las *.laz *.e57 *.txt *.asc *.xyz);;"
                               "PLY (*.ply);;PCD (*.pcd);;LAS (*.las);;LAZ (*.laz);;E57 (*.e57);;TXT (*.txt *.asc *.xyz)";
    QString meshFilters = "Mesh Files (*.obj *.stl *.vtk *.ifs);;"
                         "OBJ (*.obj);;STL (*.stl);;VTK (*.vtk);;IFS (*.ifs)";
    QString filter = pointCloudFilters + ";;" + meshFilters + ";;All Supported(*.ply *.pcd *.las *.laz *.e57 *.txt *.asc *.xyz *.obj *.stl *.vtk *.ifs);;All Files(*.*)";

    QStringList filePathList = QFileDialog::getOpenFileNames(m_parent_widget, tr("Open Files"), m_path, filter);
    if (filePathList.isEmpty()) return;

    m_progress->setLoadingQueueCount(filePathList.size());
    m_progress->showProgress("Loading Point Cloud...");
    m_progress->bindWorker(m_fileio);

    for (auto& filepath : filePathList)
        QMetaObject::invokeMethod(m_fileio, "loadPointCloud", Qt::QueuedConnection,
                                   Q_ARG(QString, filepath));
}

void CloudIOController::loadCloudFile(const QString& filepath)
{
    m_pending_parents.clear();
    m_progress->showProgress("Loading Point Cloud...");
    m_progress->bindWorker(m_fileio);
    m_progress->setLoadingQueueCount(1);

    QMetaObject::invokeMethod(m_fileio, "loadPointCloud", Qt::QueuedConnection,
                               Q_ARG(QString, filepath));
}

void CloudIOController::loadCloudFile(const QString& filepath, QTreeWidgetItem* targetParent)
{
    m_pending_parents[QFileInfo(filepath).absoluteFilePath()] = targetParent;
    m_progress->showProgress("Loading Point Cloud...");
    m_progress->bindWorker(m_fileio);
    m_progress->setLoadingQueueCount(1);

    QMetaObject::invokeMethod(m_fileio, "loadPointCloud", Qt::QueuedConnection,
                               Q_ARG(QString, filepath));
}

void CloudIOController::saveCloudFile(const Cloud::Ptr& cloud, const QString& filepath, bool isBinary)
{
    if (m_progress->savingQueueCount() <= 1) {
        m_progress->showProgress("Saving Point Cloud...");
        m_progress->bindWorker(m_fileio);
        m_progress->setSavingQueueCount(1);
    }

    QMetaObject::invokeMethod(m_fileio, "savePointCloud", Qt::QueuedConnection,
                               Q_ARG(Cloud::Ptr, cloud), Q_ARG(QString, filepath), Q_ARG(bool, isBinary));
}

void CloudIOController::saveMeshFile(const pcl::PolygonMesh::Ptr& mesh, const QString& filename)
{
    if (m_progress->savingQueueCount() <= 1) {
        m_progress->showProgress("Saving Mesh...");
        m_progress->bindWorker(m_fileio);
        m_progress->setSavingQueueCount(1);
    }

    QMetaObject::invokeMethod(m_fileio, "saveMesh", Qt::QueuedConnection,
                               Q_ARG(pcl::PolygonMesh::Ptr, mesh), Q_ARG(QString, filename));
}

void CloudIOController::onLoadCloudResult(bool success, const Cloud::Ptr& cloud,
                                          const pcl::PolygonMesh::Ptr& mesh, float time, const QString& error)
{
    if (!success) {
        emit cloudLoaded(nullptr, nullptr, nullptr, time, error);
    } else {
        QTreeWidgetItem* parent = nullptr;
        if (!m_pending_parents.isEmpty()) {
            QString fp = QFileInfo(QString::fromStdString(cloud->filepath())).absoluteFilePath();
            parent = m_pending_parents.value(fp, nullptr);
            if (parent) m_pending_parents.remove(fp);
        }
        m_path = QFileInfo(QString::fromStdString(cloud->filepath())).path();
        emit cloudLoaded(cloud, mesh, parent, time, QString());
    }

    m_progress->setLoadingQueueCount(m_progress->loadingQueueCount() - 1);
    if (m_progress->loadingQueueCount() <= 0) {
        m_progress->setLoadingQueueCount(0);
        m_progress->closeProgress();
    } else {
        if (m_progress->dialog()) {
            m_progress->dialog()->setProgress(0);
            QString msg = QString("Loading Point Cloud... (%1 remaining)").arg(m_progress->loadingQueueCount());
            m_progress->dialog()->setMessage(msg);
        }
    }
}

void CloudIOController::onSaveCloudResult(bool success, const QString& path, float time, const QString& error)
{
    if (success) m_path = path;
    emit saveComplete(success, path, time, error);
    int remaining = m_progress->savingQueueCount() - 1;
    m_progress->setSavingQueueCount(remaining);
    if (remaining <= 0) {
        m_progress->setSavingQueueCount(0);
        m_progress->closeProgress();
    } else if (m_progress->dialog()) {
        m_progress->dialog()->setProgress(0);
        m_progress->dialog()->setMessage(QString("Saving... (%1 remaining)").arg(remaining));
    }
}

void CloudIOController::onSaveMeshResult(bool success, const QString& path, float time, const QString& error)
{
    if (success) m_path = path;
    emit meshSaveComplete(success, path, time, error);
    int remaining = m_progress->savingQueueCount() - 1;
    m_progress->setSavingQueueCount(remaining);
    if (remaining <= 0) {
        m_progress->setSavingQueueCount(0);
        m_progress->closeProgress();
    } else if (m_progress->dialog()) {
        m_progress->dialog()->setProgress(0);
        m_progress->dialog()->setMessage(QString("Saving... (%1 remaining)").arg(remaining));
    }
}

void CloudIOController::onLoadTexturedMeshResult(const QString& cloudId, const QString& objFilePath)
{
    if (cloudId.isEmpty() || objFilePath.isEmpty()) return;
    emit texturedMeshLoaded(cloudId, objFilePath);
}

void CloudIOController::onFieldMappingRequested(const QList<pw::FieldInfo>& fields, std::map<std::string, std::string>& result)
{
    if (m_script_mode) {
        for (const auto& f : fields) {
            QString name = QString::fromStdString(f.name).toLower();
            if (name == "x" || name == "y" || name == "z") {
                result[f.name] = "Axis " + QString::fromStdString(f.name).toUpper().toStdString();
            } else if (name == "rgba" || name == "rgb") {
                result[f.name] = "Color(Packed)";
            } else if (name.contains("red") || name == "r") {
                result[f.name] = "Red";
            } else if (name.contains("green") || name == "g") {
                result[f.name] = "Green";
            } else if (name.contains("blue") || name == "b") {
                result[f.name] = "Blue";
            } else if (name == "normal_x" || name == "nx") {
                result[f.name] = "Normal X";
            } else if (name == "normal_y" || name == "ny") {
                result[f.name] = "Normal Y";
            } else if (name == "normal_z" || name == "nz") {
                result[f.name] = "Normal Z";
            } else if (name == "curvature") {
                result[f.name] = "Curvature";
            } else if (name == "intensity") {
                result[f.name] = "Intensity";
            } else {
                result[f.name] = "Scalar Field";
            }
        }
        return;
    }
    FieldMappingDialog dlg(fields, m_parent_widget);
    if (dlg.exec() == QDialog::Accepted) {
        pw::MappingResult res = dlg.getMapping();
        result = std::move(res.field_map);
    } else {
        result.clear();
    }
}

void CloudIOController::onTxtImportRequested(const QStringList& preview_lines, pw::TxtImportParams& params)
{
    if (m_script_mode) return;
    TxtImportDialog dlg(preview_lines, m_parent_widget);
    if (dlg.exec() == QDialog::Accepted) {
        params = dlg.getParams();
    } else {
        params.col_map.clear();
    }
}

void CloudIOController::onTxtExportRequested(const QStringList& available_fields, pw::TxtExportParams& params)
{
    if (m_script_mode) return;
    TxtExportDialog dlg(available_fields, m_parent_widget);
    if (dlg.exec() == QDialog::Accepted) {
        params = dlg.getParams();
    } else {
        params.selected_fields.clear();
    }
}

void CloudIOController::onGlobalFilterRequested(const Eigen::Vector3d& min_pt, Eigen::Vector3d& suggested_shift, bool& skipped)
{
    if (m_script_mode) {
        m_last_shift = suggested_shift;
        m_hasLastShift = true;
        skipped = false;
        return;
    }
    QWidget* topLevel = m_parent_widget ? m_parent_widget->window() : nullptr;
    GlobalShiftDialog dlg(min_pt, suggested_shift, m_last_shift, m_hasLastShift, topLevel);

    if (dlg.exec() == QDialog::Accepted) {
        suggested_shift = dlg.getShiftValue();
        m_last_shift = suggested_shift;
        m_hasLastShift = true;
        skipped = false;
    } else {
        skipped = dlg.isSkipped();
        if (!skipped && dlg.result() == QDialog::Rejected) skipped = true;
    }
}

} // namespace pw
