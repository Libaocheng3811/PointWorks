#ifndef POINTWORKS_CLOUD_MESH_DIST_DIALOG_H
#define POINTWORKS_CLOUD_MESH_DIST_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/distancecalculator.h"

#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QGroupBox>
#include <QLabel>
#include <atomic>
#include <QFutureWatcher>

#include <pcl/PolygonMesh.h>

class CloudMeshDistDialog : public ct::CustomDialog {
    Q_OBJECT
public:
    explicit CloudMeshDistDialog(QWidget* parent = nullptr);
    ~CloudMeshDistDialog() override = default;
    void init() override;
    void reset() override;

private slots:
    void onSignedToggled(bool checked);
    void onCompute();

private:
    void setupUi();
    void populateComboBoxes();

    std::atomic<bool> m_canceled{false};
    QFutureWatcher<ct::DistanceResult>* m_watcher = nullptr;

    ct::Cloud::Ptr m_source_cloud;
    pcl::PolygonMesh::Ptr m_target_mesh;

    QComboBox* cbox_source_;
    QComboBox* cbox_mesh_;
    QCheckBox* check_signed_;
    QRadioButton* radio_outside_;
    QRadioButton* radio_inside_;
    QCheckBox* check_limit_dist_;
    QDoubleSpinBox* dspin_max_dist_;
    QCheckBox* check_color_map_;
    QLineEdit* edit_field_name_;
    QPushButton* btn_compute_;
    QPushButton* btn_cancel_;
};

#endif // POINTWORKS_CLOUD_MESH_DIST_DIALOG_H
