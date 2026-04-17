#ifndef POINTWORKS_SHAPE_DETECTION_DIALOG_H
#define POINTWORKS_SHAPE_DETECTION_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/segmentation.h"

#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QButtonGroup>
#include <QRadioButton>

#include <QFutureWatcher>
#include <atomic>

class ShapeDetectionDialog : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit ShapeDetectionDialog(QWidget* parent = nullptr);
    ~ShapeDetectionDialog() override;

    void init() override;
    void reset() override;

private slots:
    void onModelChanged(int index);
    void onApply();
    void onCancel();

private:
    void setupUi();

    // --- 异步执行 ---
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<ct::SegmentationResult>* m_watcher = nullptr;

    // --- 控件（_ 后缀） ---
    // Target Shape
    QButtonGroup* bg_model_;
    QRadioButton* radio_plane_;
    QRadioButton* radio_sphere_;
    QRadioButton* radio_cylinder_;
    QRadioButton* radio_cone_;

    // Fitting Parameters
    QDoubleSpinBox* dspin_threshold_;
    QDoubleSpinBox* dspin_probability_;

    // Advanced
    QCheckBox* check_use_normals_;
    QDoubleSpinBox* dspin_distance_weight_;
    QDoubleSpinBox* dspin_distance_origin_;
    QCheckBox* check_optimize_;

    // Radius Range
    QGroupBox* group_radius_;
    QDoubleSpinBox* dspin_min_radius_;
    QDoubleSpinBox* dspin_max_radius_;

    // Method
    QComboBox* cbox_method_;

    // Iterations
    QSpinBox* spin_iterations_;

    // Output
    QCheckBox* check_colorize_;
    QCheckBox* check_split_inlier_;
    QCheckBox* check_create_mesh_;  // 二期灰显

    // Buttons
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;

    // --- 业务数据（m_ 前缀） ---
    ct::Cloud::Ptr m_cloud;
};

#endif // POINTWORKS_SHAPE_DETECTION_DIALOG_H
