#ifndef POINTWORKS_FINE_REGISTRATION_H
#define POINTWORKS_FINE_REGISTRATION_H

#include "ui/base/customdialog.h"
#include "ui/base/paramsnapshot.h"
#include "algorithm/registration.h"

#include <QComboBox>
#include <QStackedWidget>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QTextEdit>
#include <QPushButton>

#include <QFutureWatcher>
#include <atomic>

class FineRegistrationDialog : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit FineRegistrationDialog(QWidget* parent = nullptr);
    ~FineRegistrationDialog() override;

    void init() override;
    void reset() override;
    void deinit() override;

private slots:
    void onAlgorithmChanged(int index);
    void onCompute();
    void onReset();
    void onApply();
    void onCancel();

private:
    void setupUi();
    QWidget* createParamPage(int type);
    void refreshCloudList();

    // --- 控件（_ 后缀） ---
    QComboBox* cbox_source_;
    QComboBox* cbox_target_;
    QComboBox* cbox_algorithm_;
    QStackedWidget* param_pages_;

    // ICP 参数 (index 0)
    QSpinBox* spin_icp_max_iter_;
    QDoubleSpinBox* spin_icp_corr_dist_;
    QCheckBox* check_icp_reciprocal_;

    // ICP with Normals 参数 (index 1)
    QSpinBox* spin_icpn_max_iter_;
    QCheckBox* check_icpn_reciprocal_;
    QCheckBox* check_icpn_symmetric_;
    QCheckBox* check_icpn_enforce_normals_;

    // ICP NonLinear 参数 (index 2)
    QSpinBox* spin_icpnl_max_iter_;
    QDoubleSpinBox* spin_icpnl_corr_dist_;
    QCheckBox* check_icpnl_reciprocal_;

    // GICP 参数 (index 3)
    QSpinBox* spin_gicp_max_iter_;
    QSpinBox* spin_gicp_k_;
    QDoubleSpinBox* spin_gicp_rol_tolerance_;

    // NDT 参数 (index 4)
    QDoubleSpinBox* spin_ndt_resolution_;
    QDoubleSpinBox* spin_ndt_step_size_;
    QDoubleSpinBox* spin_ndt_outlier_ratio_;

    QTextEdit* txt_result_;
    QPushButton* btn_compute_;
    QPushButton* btn_reset_;
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;

    // --- 业务数据（m_ 前缀） ---
    ct::Cloud::Ptr m_source;
    ct::Cloud::Ptr m_target;
    QString m_source_id;
    Eigen::Matrix4f m_result_matrix;
    ct::Cloud::Ptr m_aligned_cloud;

    std::atomic<bool> m_canceled;
    ct::ParamSnapshot m_last_compute_snapshot;

    static constexpr const char* PREVIEW_ID = "fr_preview";
};

#endif // POINTWORKS_FINE_REGISTRATION_H
