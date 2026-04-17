#ifndef POINTWORKS_RANGEIMAGE_H
#define POINTWORKS_RANGEIMAGE_H

#include "ui/base/customdialog.h"
#include "algorithm/keypoints.h"

#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QTimer>
#include <QScrollArea>

class RangeImage : public ct::CustomDialog {
    Q_OBJECT

public:
    explicit RangeImage(QWidget* parent = nullptr);
    ~RangeImage() override;

    void init() override;
    void reset() override;

private slots:
    void onViewerPoseChanged(Eigen::Affine3f pose);
    void onParamChanged();
    void onExportImage();
    void onClose();
    void onTogglePointCloud(bool checked);
    void onDebounceTimeout();

protected:
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void showEvent(QShowEvent* event) override;

private:
    void setupUi();
    void updateDepthImage();
    void applyScale();
    void fitToView();

    // --- UI Controls ---
    QScrollArea* scroll_area_;
    QLabel* image_label_;
    QDoubleSpinBox* dspin_angular_res_;
    QDoubleSpinBox* dspin_h_fov_;
    QDoubleSpinBox* dspin_v_fov_;
    QPushButton* btn_export_;
    QPushButton* btn_close_;
    QCheckBox* check_show_cloud_;

    // --- Data ---
    ct::RangeImage::Ptr m_range_image;
    ct::Cloud::Ptr m_cloud;
    Eigen::Affine3f m_viewer_pose;
    QTimer* m_debounce_timer_;

    // --- Zoom / Pan ---
    QPixmap m_pixmap;
    qreal m_scale_factor_ = 1.0;
    QPoint m_last_mouse_pos_;
    bool m_dragging_ = false;
};

#endif // POINTWORKS_RANGEIMAGE_H
