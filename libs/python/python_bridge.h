#ifndef POINTWORKS_PYTHON_BRIDGE_H
#define POINTWORKS_PYTHON_BRIDGE_H

#include <QObject>
#include <QString>
#include <QMutex>
#include <QMap>
#include <vector>
#include "core/cloud.h"

namespace ct
{

/// Python→C++ 信号桥接
///
/// 供 Python 调用的方法内部只发射信号，不直接操作 UI。
/// 云注册表和引用持有均通过 QMutex 保护线程安全。
class PythonBridge : public QObject
{
    Q_OBJECT

public:
    explicit PythonBridge(QObject* parent = nullptr) : QObject(parent) {}

    // ================================================================
    // 日志与进度
    // ================================================================

    void log(int level, const QString& msg)              { emit signalLog(level, msg); }
    void showProgress(const QString& title)             { emit signalShowProgress(title); }
    void setProgress(int percent)                   { emit signalSetProgress(percent); }
    void closeProgress()                             { emit signalCloseProgress(); }

    // ================================================================
    // Python stdio 输出路由（Python Console 用）
    // ================================================================

    void printStdout(const QString& text)                { emit signalPrintStdout(text); }
    void printStderr(const QString& text)                { emit signalPrintStderr(text); }

    // ================================================================
    // 视图控制
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
    // 点云外观
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
    // 场景外观
    // ================================================================

    void setBackgroundColor(float r, float g, float b){ emit signalSetBackgroundColor(r, g, b); }
    void resetBackgroundColor()                       { emit signalResetBackgroundColor(); }

    // ================================================================
    // 显示开关
    // ================================================================

    void showId(bool show)                           { emit signalShowId(show); }
    void showAxes(bool show)                         { emit signalShowAxes(show); }
    void showFPS(bool show)                          { emit signalShowFPS(show); }
    void showInfo(const QString& text)               { emit signalShowInfo(text); }
    void clearInfo()                                 { emit signalClearInfo(); }

    // ================================================================
    // 叠加物
    // ================================================================

    void addCube(float cx, float cy, float cz, float size, const QString& id)
                                                      { emit signalAddCube(cx, cy, cz, size, id); }
    void add3DLabel(const QString& text, float x, float y, float z, const QString& id)
                                                      { emit signalAdd3DLabel(text, x, y, z, id); }
    void removeShape(const QString& id)               { emit signalRemoveShape(id); }
    void removeAllShapes()                           { emit signalRemoveAllShapes(); }

    // —— Phase 3: 高级叠加物 ——
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
    // 点云管理
    // ================================================================

    void insertCloud(Cloud::Ptr cloud)               { emit signalInsertCloud(cloud); }
    void removeSelectedClouds()                       { emit signalRemoveSelectedClouds(); }

    // —— Phase 1: 按名称操作 ——
    void removeCloud(const QString& id)               { emit signalRemoveCloud(id); }
    void removeAllClouds()                            { emit signalRemoveAllClouds(); }
    void cloneCloud(const QString& id)                { emit signalCloneCloud(id); }
    void mergeClouds(const QStringList& ids)          { emit signalMergeClouds(ids); }
    void selectCloud(const QString& id)               { emit signalSelectCloud(id); }
    void loadCloud(const QString& filepath)           { emit signalLoadCloud(filepath); }
    void saveCloud(const QString& id, const QString& filepath, bool binary = true)
                                                      { emit signalSaveCloud(id, filepath, binary); }

    // —— Phase 2: 算法结果组 ——
    void addResultGroup(const QString& origin_id, const std::vector<Cloud::Ptr>& results, const QString& group_name)
                                                      { emit signalAddResultGroup(origin_id, results, group_name); }

    // —— Phase 3: 就地更新点云 ——
    void updateCloud(const QString& id, const Cloud::Ptr& new_cloud)
                                                      { emit signalUpdateCloud(id, new_cloud); }

    // —— 脚本模式：跳过弹窗 ——
    void setScriptMode(bool enabled)                  { emit signalSetScriptMode(enabled); }

    // ================================================================
    // 云注册表（线程安全）
    // ================================================================

    void registerCloud(Cloud::Ptr cloud);
    void unregisterCloud(const QString& id);
    Cloud::Ptr getCloud(const QString& name) const;
    QStringList getCloudNames() const;

    // ================================================================
    // 引用持有 + In-use 标记
    // ================================================================

    void holdCloud(Cloud::Ptr cloud);
    void releaseAllHeld();
    void releaseAllInUse();
    void markCloudInUse(const QString& id)            { emit signalMarkCloudInUse(id); }
    void unmarkCloudInUse(const QString& id)          { emit signalUnmarkCloudInUse(id); }

signals:
    // 日志
    void signalLog(int level, QString message);
    void signalPrint(QString message);

    // Python stdio（Python Console 用）
    void signalPrintStdout(QString text);
    void signalPrintStderr(QString text);

    // 进度
    void signalShowProgress(QString title);
    void signalSetProgress(int percent);
    void signalCloseProgress();

    // 视图控制
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

    // 点云外观
    void signalSetPointSize(QString id, float size);
    void signalSetOpacity(QString id, float value);
    void signalSetCloudColorRGB(QString id, float r, float g, float b);
    void signalSetCloudColorByAxis(QString id, QString axis);
    void signalResetCloudColor(QString id);
    void signalSetCloudVisibility(QString id, bool visible);

    // 场景外观
    void signalSetBackgroundColor(float r, float g, float b);
    void signalResetBackgroundColor();

    // 显示开关
    void signalShowId(bool show);
    void signalShowAxes(bool show);
    void signalShowFPS(bool show);
    void signalShowInfo(QString text);
    void signalClearInfo();

    // 叠加物
    void signalAddCube(float cx, float cy, float cz, float size, QString id);
    void signalAdd3DLabel(QString text, float x, float y, float z, QString id);
    void signalRemoveShape(QString id);
    void signalRemoveAllShapes();

    // Phase 3: 高级叠加物 + 视图控制
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

    // 点云管理
    void signalInsertCloud(ct::Cloud::Ptr cloud);
    void signalRemoveSelectedClouds();
    void signalRemoveCloud(QString id);
    void signalRemoveAllClouds();
    void signalCloneCloud(QString id);
    void signalMergeClouds(QStringList ids);
    void signalSelectCloud(QString id);
    void signalLoadCloud(QString filepath);
    void signalSaveCloud(QString id, QString filepath, bool binary);

    // Phase 2: 算法结果组
    void signalAddResultGroup(QString origin_id, std::vector<ct::Cloud::Ptr> results, QString group_name);

    // Phase 3: 就地更新
    void signalUpdateCloud(QString id, ct::Cloud::Ptr new_cloud);

    // 脚本模式
    void signalSetScriptMode(bool enabled);

    // 注册表
    void signalCloudRegistered(QString name);
    void signalCloudUnregistered(QString id);

    // In-use
    void signalMarkCloudInUse(QString cloud_id);
    void signalUnmarkCloudInUse(QString cloud_id);
    void signalReleaseAllInUse();

private:
    mutable QMutex m_cloud_mutex;
    QMap<QString, Cloud::Ptr> m_cloud_registry;
    std::vector<Cloud::Ptr> m_held_clouds;
};

} // namespace ct

#endif // POINTWORKS_PYTHON_BRIDGE_H
