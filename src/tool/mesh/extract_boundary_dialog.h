#ifndef POINTWORKS_EXTRACT_BOUNDARY_DIALOG_H
#define POINTWORKS_EXTRACT_BOUNDARY_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/features.h"

#include <QRadioButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QButtonGroup>

#include <QFutureWatcher>
#include <atomic>

class ExtractBoundaryDialog : public pw::CustomDialog
{
    Q_OBJECT

public:
    explicit ExtractBoundaryDialog(QWidget* parent = nullptr);
    ~ExtractBoundaryDialog() override;

    void init() override;
    void reset() override;

private slots:
    void onExtract();
    void onCancel();

private:
    void setupUi();

    // --- 异步执行 ---
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<pw::Cloud::Ptr>* m_watcher = nullptr;

    // --- 控件（_ 后缀） ---
    QSpinBox* spin_k_;
    QDoubleSpinBox* dspin_search_radius_;
    QDoubleSpinBox* dspin_angle_threshold_;
    QRadioButton* radio_output_polyline_;
    QRadioButton* radio_output_select_;
    QPushButton* btn_extract_;
    QPushButton* btn_close_;

    // --- 业务数据（m_ 前缀） ---
    pw::Cloud::Ptr m_cloud;
};

#endif // POINTWORKS_EXTRACT_BOUNDARY_DIALOG_H
