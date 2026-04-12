#ifndef POINTWORKS_CLOUD_CLOUD_DIST_DIALOG_H
#define POINTWORKS_CLOUD_CLOUD_DIST_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/distancecalculator.h"

#include <QComboBox>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QButtonGroup>
#include <QGroupBox>
#include <QLabel>
#include <atomic>
#include <QFutureWatcher>

class CloudCloudDistDialog : public ct::CustomDialog {
    Q_OBJECT
public:
    explicit CloudCloudDistDialog(QWidget* parent = nullptr);
    ~CloudCloudDistDialog() override = default;
    void init() override;
    void reset() override;

private slots:
    void onMethodChanged(int id);
    void onCompute();

private:
    void setupUi();
    void populateComboBoxes();
    void autoSelectClouds();

    // async
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<ct::DistanceResult>* m_watcher = nullptr;

    // business data
    ct::Cloud::Ptr m_source_cloud;
    ct::Cloud::Ptr m_target_cloud;

    // controls
    QComboBox* cbox_source_;
    QComboBox* cbox_target_;
    QButtonGroup* method_group_;
    QRadioButton* radio_nearest_;
    QRadioButton* radio_knn_;
    QRadioButton* radio_radius_;
    QCheckBox* check_limit_dist_;
    QDoubleSpinBox* dspin_max_dist_;
    QSpinBox* spin_k_;
    QDoubleSpinBox* dspin_radius_;
    QCheckBox* check_color_map_;
    QLineEdit* edit_field_name_;
    QPushButton* btn_compute_;
    QPushButton* btn_cancel_;

    // param pages (for KNN / Radius specific params)
    QWidget* page_nearest_;
    QWidget* page_knn_;
    QWidget* page_radius_;
    QStackedWidget* param_pages_;
};

#endif // POINTWORKS_CLOUD_CLOUD_DIST_DIALOG_H
