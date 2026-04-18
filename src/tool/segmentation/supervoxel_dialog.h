#ifndef POINTWORKS_SUPERVOXEL_DIALOG_H
#define POINTWORKS_SUPERVOXEL_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/segmentation.h"

#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>

#include <QFutureWatcher>
#include <atomic>

class SupervoxelDialog : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit SupervoxelDialog(QWidget* parent = nullptr);
    ~SupervoxelDialog() override;

    void init() override;
    void reset() override;

private slots:
    void onApply();
    void onCancel();

private:
    void setupUi();

    // --- 异步执行 ---
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<ct::SegmentationResult>* m_watcher = nullptr;

    // --- 控件（_ 后缀） ---
    QDoubleSpinBox* dspin_voxel_resolution_;
    QDoubleSpinBox* dspin_seed_resolution_;
    QDoubleSpinBox* dspin_color_importance_;
    QDoubleSpinBox* dspin_spatial_importance_;
    QDoubleSpinBox* dspin_normal_importance_;
    QCheckBox* check_camera_transform_;
    QCheckBox* check_split_;

    // 按钮
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;

    // --- 业务数据（m_ 前缀） ---
    ct::Cloud::Ptr m_cloud;
};

#endif // POINTWORKS_SUPERVOXEL_DIALOG_H
