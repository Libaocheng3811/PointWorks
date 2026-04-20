#ifndef POINTWORKS_COMPUTE_HULL_DIALOG_H
#define POINTWORKS_COMPUTE_HULL_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/surface.h"
#include "viz/surface_viz_helper.h"

#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QButtonGroup>

#include <QFutureWatcher>
#include <atomic>

class ComputeHullDialog : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit ComputeHullDialog(QWidget* parent = nullptr);
    ~ComputeHullDialog() override;

    void init() override;
    void reset() override;

private slots:
    void onHullTypeChanged();
    void onCompute();
    void onCancel();

private:
    void setupUi();
    void setConcaveParamsEnabled(bool enabled);

    // --- 异步执行 ---
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<ct::SurfaceResult>* m_watcher = nullptr;

    // --- 控件（_ 后缀） ---
    QRadioButton* radio_convex_;
    QRadioButton* radio_concave_;
    QDoubleSpinBox* dspin_alpha_;
    QRadioButton* radio_3d_;
    QRadioButton* radio_2d_;
    QCheckBox* check_keep_info_;
    QPushButton* btn_compute_;
    QPushButton* btn_close_;
    QLabel* label_warning_;

    // --- 业务数据（m_ 前缀） ---
    ct::Cloud::Ptr m_cloud;
};

#endif // POINTWORKS_COMPUTE_HULL_DIALOG_H
