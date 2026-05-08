#ifndef POINTWORKS_CLOSEST_POINT_SET_DIALOG_H
#define POINTWORKS_CLOSEST_POINT_SET_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/distancecalculator.h"

#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QLabel>
#include <atomic>
#include <QFutureWatcher>

class ClosestPointSetDialog : public pw::CustomDialog {
    Q_OBJECT
public:
    explicit ClosestPointSetDialog(QWidget* parent = nullptr);
    ~ClosestPointSetDialog() override = default;
    void init() override;
    void reset() override;

private slots:
    void onTargetChanged(int index);
    void onExtract();

private:
    void setupUi();
    void populateComboBoxes();
    void updateOutputName();

    std::atomic<bool> m_canceled{false};
    QFutureWatcher<pw::CPSResult>* m_watcher = nullptr;

    pw::Cloud::Ptr m_source_cloud;
    pw::Cloud::Ptr m_target_cloud;

    QComboBox* cbox_source_;
    QComboBox* cbox_target_;
    QCheckBox* check_keep_colors_;
    QCheckBox* check_keep_intensity_;
    QCheckBox* check_keep_scalars_;
    QCheckBox* check_limit_dist_;
    QDoubleSpinBox* dspin_max_dist_;
    QLineEdit* edit_output_name_;
    QPushButton* btn_extract_;
    QPushButton* btn_cancel_;
};

#endif // POINTWORKS_CLOSEST_POINT_SET_DIALOG_H
