#ifndef POINTWORKS_PYTHON_BRIDGE_H
#define POINTWORKS_PYTHON_BRIDGE_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <QMutex>
#include "cloud_registry.h"
#include "core/cloud.h"

#include <pcl/PolygonMesh.h>

namespace pw
{

/// Python->C++ signal bridge
///
/// Methods called from Python only emit signals, never directly operate UI.
/// Cloud registry and reference holding are delegated to PythonCloudRegistry (Qt-free).
class PythonBridge : public QObject
{
    Q_OBJECT

public:
    explicit PythonBridge(QObject* parent = nullptr);

    // ================================================================
    // PythonCloudRegistry access
    // ================================================================

    PythonCloudRegistry& registry() { return m_registry; }
    const PythonCloudRegistry& registry() const { return m_registry; }

    // ================================================================
    // Logging and progress
    // ================================================================

    void log(int level, const QString& msg)              { emit signalLog(level, msg); }
    void showProgress(const QString& title)             { emit signalShowProgress(title); }
    void setProgress(int percent)                   { emit signalSetProgress(percent); }
    void closeProgress()                             { emit signalCloseProgress(); }

    // ================================================================
    // Python stdio output routing (Python Console)
    // ================================================================

    void printStdout(const QString& text)                { emit signalPrintStdout(text); }
    void printStderr(const QString& text)                { emit signalPrintStderr(text); }

    // ================================================================
    // View control
    // ================================================================

    void refreshView()                                 { emit signalRefreshView(); }
    void cloudChanged(const QString& id)               { emit signalCloudChanged(id); }
    void resetCamera()                                 { emit signalResetCamera(); }
    void zoomToBounds()                               { emit signalZoomToBounds(); }
    void setAutoRender(bool enable)                   { emit signalSetAutoRender(enable); }
    void zoomToSelected()                             { emit signalZoomToSelected(); }

    void setTopView()                                 { emit signalSetTopView(); }
    void setFrontView()                               { emit signalSetFrontView(); }
    void setBackView()                                { emit signalSetBackView(); }
    void setLeftSideView()                            { emit signalSetLeftSideView(); }
    void setRightSideView()                           { emit signalSetRightSideView(); }
    void setBottomView()                              { emit signalSetBottomView(); }

    // ================================================================
    // Point cloud appearance
    // ================================================================

    void setPointSize(const QString& id, float size)  { emit signalSetPointSize(id, size); }
    void setOpacity(const QString& id, float value)   { emit signalSetOpacity(id, value); }
    void setCloudColorRGB(const QString& id, float r, float g, float b)
                                                      { emit signalSetCloudColorRGB(id, r, g, b); }
    void setCloudColorByAxis(const QString& id, const QString& axis)
                                                      { emit signalSetCloudColorByAxis(id, axis); }
    void resetCloudColor(const QString& id)            { emit signalResetCloudColor(id); }
    void setCloudVisibility(const QString& id, bool v){ emit signalSetCloudVisibility(id, v); }

    // ================================================================
    // Scene appearance
    // ================================================================

    void setBackgroundColor(float r, float g, float b){ emit signalSetBackgroundColor(r, g, b); }
    void resetBackgroundColor()                       { emit signalResetBackgroundColor(); }

    // ================================================================
    // Display toggles
    // ================================================================

    void showId(bool show)                           { emit signalShowId(show); }
    void showAxes(bool show)                         { emit signalShowAxes(show); }
    void showFPS(bool show)                          { emit signalShowFPS(show); }
    void showInfo(const QString& text)               { emit signalShowInfo(text); }
    void clearInfo()                                 { emit signalClearInfo(); }

    // ================================================================
    // Overlays
    // ================================================================

    void addCube(float cx, float cy, float cz, float size, const QString& id)
                                                      { emit signalAddCube(cx, cy, cz, size, id); }
    void add3DLabel(const QString& text, float x, float y, float z, const QString& id)
                                                      { emit signalAdd3DLabel(text, x, y, z, id); }
    void removeShape(const QString& id)               { emit signalRemoveShape(id); }
    void removeAllShapes()                           { emit signalRemoveAllShapes(); }

    void addArrow(float x1, float y1, float z1, float x2, float y2, float z2,
                  const QString& id, float r, float g, float b)
                                                      { emit signalAddArrow(x1,y1,z1,x2,y2,z2,id,r,g,b); }
    void addPolygonCloud(const QString& cloud_id, const QString& id,
                         float r, float g, float b)
                                                      { emit signalAddPolygon(cloud_id, id, r, g, b); }
    void setShapeColor(const QString& id, float r, float g, float b)
                                                      { emit signalSetShapeColor(id, r, g, b); }
    void setShapeSize(const QString& id, float size) { emit signalSetShapeSize(id, size); }
    void setShapeOpacity(const QString& id, float value){ emit signalSetShapeOpacity(id, value); }
    void setShapeLineWidth(const QString& id, float value){ emit signalSetShapeLineWidth(id, value); }
    void setShapeFontSize(const QString& id, float value){ emit signalSetShapeFontSize(id, value); }
    void setShapeRepresentation(const QString& id, int type)
                                                      { emit signalSetShapeRepresentation(id, type); }
    void zoomToBoundsXYZ(float min_x, float min_y, float min_z,
                         float max_x, float max_y, float max_z)
                                                      { emit signalZoomToBoundsXYZ(min_x,min_y,min_z,max_x,max_y,max_z); }
    void invalidateCloudRender(const QString& id)    { emit signalInvalidateCloudRender(id); }
    void setInteractorEnable(bool enable)            { emit signalSetInteractorEnable(enable); }

    // ================================================================
    // Cloud management (signals)
    // ================================================================

    void insertCloud(Cloud::Ptr cloud)               { emit signalInsertCloud(cloud); }
    void removeSelectedClouds()                       { emit signalRemoveSelectedClouds(); }
    void removeCloud(const QString& id)               { emit signalRemoveCloud(id); }
    void removeAllClouds()                            { emit signalRemoveAllClouds(); }
    void cloneCloud(const QString& id)                { emit signalCloneCloud(id); }
    void mergeClouds(const QStringList& ids)          { emit signalMergeClouds(ids); }
    void selectCloud(const QString& id)               { emit signalSelectCloud(id); }
    void loadCloud(const QString& filepath)           { emit signalLoadCloud(filepath); }
    void saveCloud(const QString& id, const QString& filepath, bool binary = true)
                                                      { emit signalSaveCloud(id, filepath, binary); }
    void addResultGroup(const QString& origin_id, const std::vector<Cloud::Ptr>& results, const QString& group_name)
                                                      { emit signalAddResultGroup(origin_id, results, group_name); }
    void updateCloud(const QString& id, const Cloud::Ptr& new_cloud)
                                                      { emit signalUpdateCloud(id, new_cloud); }
    void addMesh(const pcl::PolygonMesh::Ptr& mesh, const QString& id)
                                                      { emit signalAddMesh(mesh, id); }
    void removeMesh(const QString& id)                 { emit signalRemoveMesh(id); }

    // ================================================================
    // Script mode
    // ================================================================

    void setScriptMode(bool enabled) {
        m_registry.setScriptMode(enabled);
        emit signalSetScriptMode(enabled);
    }
    bool isScriptMode() const { return m_registry.isScriptMode(); }

    void clearScriptSession();
    void requestClearAll()                           { emit signalClearAll(); }
    void clearScriptData();

    // ================================================================
    // Legacy QString-based delegates (kept for python_connections.cpp compatibility)
    // ================================================================

    void registerCloud(Cloud::Ptr cloud)              { m_registry.registerCloud(cloud); }
    void unregisterCloud(const QString& id)            { m_registry.unregisterCloud(id.toStdString()); }
    Cloud::Ptr getCloud(const QString& name) const     { return m_registry.getCloud(name.toStdString()); }
    QStringList getCloudNames() const;
    void holdCloud(Cloud::Ptr cloud)                  { m_registry.holdCloud(cloud); }
    void releaseAllHeld()                             { m_registry.releaseAllHeld(); }
    void markCloudInUse(const QString& id)            { emit signalMarkCloudInUse(id); }
    void unmarkCloudInUse(const QString& id)          { emit signalUnmarkCloudInUse(id); }
    void releaseAllInUse()                            { emit signalReleaseAllInUse(); }

    void markScriptGenerated(const QString& id)       { m_registry.markScriptGenerated(id.toStdString()); }
    void markSceneMounted(const QString& id)          { m_registry.markSceneMounted(id.toStdString()); }

signals:
    // Logging
    void signalLog(int level, QString message);
    void signalPrint(QString message);

    // Python stdio
    void signalPrintStdout(QString text);
    void signalPrintStderr(QString text);

    // Progress
    void signalShowProgress(QString title);
    void signalSetProgress(int percent);
    void signalCloseProgress();

    // View control
    void signalCloudChanged(QString cloud_id);
    void signalResetCamera();
    void signalRefreshView();
    void signalZoomToBounds();
    void signalSetAutoRender(bool enable);
    void signalZoomToSelected();

    void signalSetTopView();
    void signalSetFrontView();
    void signalSetBackView();
    void signalSetLeftSideView();
    void signalSetRightSideView();
    void signalSetBottomView();

    // Point cloud appearance
    void signalSetPointSize(QString id, float size);
    void signalSetOpacity(QString id, float value);
    void signalSetCloudColorRGB(QString id, float r, float g, float b);
    void signalSetCloudColorByAxis(QString id, QString axis);
    void signalResetCloudColor(QString id);
    void signalSetCloudVisibility(QString id, bool visible);

    // Scene appearance
    void signalSetBackgroundColor(float r, float g, float b);
    void signalResetBackgroundColor();

    // Display toggles
    void signalShowId(bool show);
    void signalShowAxes(bool show);
    void signalShowFPS(bool show);
    void signalShowInfo(QString text);
    void signalClearInfo();

    // Overlays
    void signalAddCube(float cx, float cy, float cz, float size, QString id);
    void signalAdd3DLabel(QString text, float x, float y, float z, QString id);
    void signalRemoveShape(QString id);
    void signalRemoveAllShapes();

    void signalAddArrow(float x1, float y1, float z1, float x2, float y2, float z2,
                        QString id, float r, float g, float b);
    void signalAddPolygon(QString cloud_id, QString id, float r, float g, float b);
    void signalSetShapeColor(QString id, float r, float g, float b);
    void signalSetShapeSize(QString id, float size);
    void signalSetShapeOpacity(QString id, float value);
    void signalSetShapeLineWidth(QString id, float value);
    void signalSetShapeFontSize(QString id, float value);
    void signalSetShapeRepresentation(QString id, int type);
    void signalZoomToBoundsXYZ(float min_x, float min_y, float min_z,
                               float max_x, float max_y, float max_z);
    void signalInvalidateCloudRender(QString id);
    void signalSetInteractorEnable(bool enable);

    // Cloud management
    void signalInsertCloud(pw::Cloud::Ptr cloud);
    void signalRemoveSelectedClouds();
    void signalRemoveCloud(QString id);
    void signalRemoveAllClouds();
    void signalCloneCloud(QString id);
    void signalMergeClouds(QStringList ids);
    void signalSelectCloud(QString id);
    void signalLoadCloud(QString filepath);
    void signalSaveCloud(QString id, QString filepath, bool binary);

    // Algorithm result groups
    void signalAddResultGroup(QString origin_id, std::vector<pw::Cloud::Ptr> results, QString group_name);

    // In-place update
    void signalUpdateCloud(QString id, pw::Cloud::Ptr new_cloud);

    // Mesh display
    void signalAddMesh(pcl::PolygonMesh::Ptr mesh, QString id);
    void signalRemoveMesh(QString id);

    // Script mode
    void signalSetScriptMode(bool enabled);
    void signalClearScriptSession();
    void signalClearAll();
    void signalClearScriptData(QStringList ids);

    // Registry sync
    void signalCloudRegistered(QString name);
    void signalCloudUnregistered(QString id);

    // In-use
    void signalMarkCloudInUse(QString cloud_id);
    void signalUnmarkCloudInUse(QString cloud_id);
    void signalReleaseAllInUse();

private:
    PythonCloudRegistry m_registry;
};

} // namespace pw

#endif // POINTWORKS_PYTHON_BRIDGE_H
