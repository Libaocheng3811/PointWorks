//
// Created by LBC on 2026/04/12.
//

#ifndef POINTWORKS_MEASURE_H
#define POINTWORKS_MEASURE_H

#include "ui/base/customdialog.h"
#include "core/common.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Measure; }
QT_END_NAMESPACE

class Measure : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Measure(QWidget *parent = nullptr);
    ~Measure() override;

    void init() override;
    void reset() override;
    void deinit() override;

private slots:
    // UI actions
    void onStartStop();
    void onClearAll();
    void onDeleteSelected();
    void onExport();
    void onShowLabelsChanged(int state);
    void onShowArrowsChanged(int state);
    void onPrecisionChanged(int value);
    void onMeasurementSelected(int row, int col);

    // CloudView mouse events
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

private:
    Ui::Measure *ui;

    // State
    bool m_measuring = false;
    bool m_first_point_set = false;
    int m_precision = 3;

    // Selected cloud
    ct::Cloud::Ptr m_selected_cloud;

    // Temporary marker cloud for current picking endpoints
    ct::Cloud::Ptr m_marker_cloud;

    // First point screen position (to detect drag vs click)
    ct::PointXY m_first_screen_pos;

    // Measurement result
    struct Measurement {
        int id;
        ct::PointXYZRGBN start_pt;
        ct::PointXYZRGBN end_pt;
        float distance;
        float dx, dy, dz;
    };

    QVector<Measurement> m_measurements;
    int m_next_id = 1;

    // ID generators for 3D annotations
    static QString arrowId(int measId)   { return QString("meas_arrow_%1").arg(measId); }
    static QString labelId(int measId)   { return QString("meas_label_%1").arg(measId); }
    static QString markerCloudId()        { return QString("meas_markers"); }
    static QString previewArrowId()       { return QString("meas_preview_arrow"); }
    static QString previewLabelId()       { return QString("meas_preview_label"); }

    // Internal methods
    void pickFirstPoint(const ct::PointXYZRGBN& pt3d, const ct::PointXY& screenPt);
    void pickSecondPoint(const ct::PointXYZRGBN& pt3d);
    void updatePreview(const ct::PointXYZRGBN& hover_pt);
    void cancelCurrentPick();

    void addMeasurementToScene(const Measurement& m);
    void removeMeasurementFromScene(const Measurement& m);
    void addMeasurementToTable(const Measurement& m);
    void removeMeasurementFromTable(int index);
    void refreshAllSceneDisplay();

    void updateMarkerCloud(const ct::PointXYZRGBN& pt);
    void clearMarkerCloud();

    void updateInfoText();
};

#endif // POINTWORKS_MEASURE_H
