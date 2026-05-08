#ifndef POINTWORKS_REGION_GROWING_DIALOG_H
#define POINTWORKS_REGION_GROWING_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/segmentation.h"

#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>

#include <QFutureWatcher>
#include <atomic>

class RegionGrowingDialog : public pw::CustomDialog
{
    Q_OBJECT

public:
    explicit RegionGrowingDialog(QWidget* parent = nullptr);
    ~RegionGrowingDialog() override;

    void init() override;
    void reset() override;

private slots:
    void onSmoothToggled(bool checked);
    void onCurvatureToggled(bool checked);
    void onColorToggled(bool checked);
    void onSeedModeChanged();
    void onPickSeed();
    void onApply();
    void onCancel();
    void mouseLeftPressed(const pw::PointXY& pt);

private:
    void setupUi();

    // --- 异步执行 ---
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<pw::SegmentationResult>* m_watcher = nullptr;

    // --- 控件（_ 后缀） ---
    // Search Settings
    QSpinBox* spin_neighbours_;

    // Growth Criteria
    QCheckBox* check_smooth_;
    QSlider* slider_smooth_;
    QDoubleSpinBox* dspin_smooth_;

    QCheckBox* check_curvature_;
    QDoubleSpinBox* dspin_curvature_;

    QCheckBox* check_residual_;
    QDoubleSpinBox* dspin_residual_;

    QCheckBox* check_color_;
    QDoubleSpinBox* dspin_point_color_;
    QDoubleSpinBox* dspin_region_color_;
    QDoubleSpinBox* dspin_distance_;
    QSpinBox* spin_color_neighbors_;
    QGroupBox* group_color_;

    // Seed Strategy
    QButtonGroup* bg_seed_;
    QRadioButton* radio_auto_;
    QRadioButton* radio_manual_;
    QPushButton* btn_pick_seed_;
    QLabel* label_seed_info_;

    // Output
    QSpinBox* spin_min_cluster_;
    QSpinBox* spin_max_cluster_;
    QCheckBox* check_split_;

    // Buttons
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;

    // --- 业务数据（m_ 前缀） ---
    pw::Cloud::Ptr m_cloud;
    bool m_picking = false;
    pw::PointXYZRGBN m_seed_point;
    bool m_has_seed = false;
};

#endif // POINTWORKS_REGION_GROWING_DIALOG_H
