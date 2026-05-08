#ifndef POINTWORKS_CLOUD_PRIMITIVE_DIST_DIALOG_H
#define POINTWORKS_CLOUD_PRIMITIVE_DIST_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/distancecalculator.h"

#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QGroupBox>
#include <QLabel>
#include <atomic>
#include <QFutureWatcher>

class CloudPrimitiveDistDialog : public pw::CustomDialog {
    Q_OBJECT
public:
    explicit CloudPrimitiveDistDialog(QWidget* parent = nullptr);
    ~CloudPrimitiveDistDialog() override = default;
    void init() override;
    void reset() override;

private slots:
    void onPrimitiveChanged(int index);
    void onCompute();

private:
    void setupUi();
    QWidget* createPlaneParamPage();
    QWidget* createSphereParamPage();

    std::atomic<bool> m_canceled{false};
    QFutureWatcher<pw::DistanceResult>* m_watcher = nullptr;

    pw::Cloud::Ptr m_source_cloud;

    QComboBox* cbox_source_;
    QComboBox* cbox_primitive_;
    QStackedWidget* param_pages_;

    // Plane
    QDoubleSpinBox* dspin_plane_a_, *dspin_plane_b_, *dspin_plane_c_, *dspin_plane_d_;
    // Sphere
    QDoubleSpinBox* dspin_sphere_cx_, *dspin_sphere_cy_, *dspin_sphere_cz_, *dspin_sphere_r_;

    QCheckBox* check_color_map_;
    QLineEdit* edit_field_name_;
    QPushButton* btn_compute_;
    QPushButton* btn_cancel_;
};

#endif // POINTWORKS_CLOUD_PRIMITIVE_DIST_DIALOG_H
