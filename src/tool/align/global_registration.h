#ifndef POINTWORKS_GLOBAL_REGISTRATION_H
#define POINTWORKS_GLOBAL_REGISTRATION_H

#include "ui/base/customdialog.h"
#include "ui/base/paramsnapshot.h"
#include "algorithm/keypoints.h"
#include "algorithm/features.h"
#include "algorithm/registration.h"

#include <QComboBox>
#include <QStackedWidget>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QPushButton>

#include <QFutureWatcher>
#include <atomic>
#include <memory>

class GlobalRegistrationDialog : public pw::CustomDialog
{
    Q_OBJECT

public:
    explicit GlobalRegistrationDialog(QWidget* parent = nullptr);
    ~GlobalRegistrationDialog() override;

    void init() override;
    void reset() override;
    void deinit() override;

private slots:
    void onKeypointChanged(int index);
    void onDescriptorChanged(int index);
    void onAlignmentChanged(int index);
    void onCompute();
    void onComputeDone();
    void onToggleLines(bool checked);
    void onReset();
    void onApply();
    void onCancel();

private:
    void setupUi();
    QWidget* createKeypointPage(int type);
    QWidget* createDescriptorPage();
    QWidget* createAlignmentPage(int type);

    // --- 控件（_ 后缀） ---
    QComboBox* cbox_source_;
    QComboBox* cbox_target_;
    QComboBox* cbox_keypoint_;
    QComboBox* cbox_descriptor_;
    QComboBox* cbox_alignment_;
    QStackedWidget* keypoint_pages_;
    QStackedWidget* alignment_pages_;
    QCheckBox* check_show_lines_;
    QSpinBox* spin_topn_;

    // --- Keypoint 参数控件 ---
    // ISS
    QDoubleSpinBox* spin_iss_gamma21_;
    QDoubleSpinBox* spin_iss_gamma32_;
    QSpinBox* spin_iss_min_neighbors_;
    QDoubleSpinBox* spin_iss_angle_;
    // Harris3D
    QComboBox* cbox_harris_method_;
    QDoubleSpinBox* spin_harris_threshold_;
    // SIFT
    QDoubleSpinBox* spin_sift_min_scale_;
    QSpinBox* spin_sift_octaves_;
    QSpinBox* spin_sift_scales_per_octave_;
    QDoubleSpinBox* spin_sift_min_contrast_;
    // Trajkovic
    QComboBox* cbox_traj_method_;
    QSpinBox* spin_traj_window_;
    QDoubleSpinBox* spin_traj_thr1_;
    QDoubleSpinBox* spin_traj_thr2_;

    // --- Descriptor 参数控件 ---
    QSpinBox* spin_desc_k_;
    QDoubleSpinBox* spin_desc_radius_;

    // --- Alignment 参数控件 ---
    // SAC-IA
    QDoubleSpinBox* spin_sacia_min_dist_;
    QSpinBox* spin_sacia_samples_;
    QSpinBox* spin_sacia_k_;
    // SAC-Prerejective
    QSpinBox* spin_sacp_samples_;
    QSpinBox* spin_sacp_k_;
    QDoubleSpinBox* spin_sacp_similarity_;
    QDoubleSpinBox* spin_sacp_inlier_frac_;

    QTextEdit* txt_result_;
    QPushButton* btn_compute_;
    QPushButton* btn_reset_;
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;

private:
    // --- 业务数据（m_ 前缀） ---
    void refreshCloudList();
    void updateCorrespondenceLines();
    void clearCorrespondenceLines();

    pw::Cloud::Ptr m_source;
    pw::Cloud::Ptr m_target;
    QString m_source_id;
    Eigen::Matrix4f m_result_matrix;
    pw::Cloud::Ptr m_aligned_cloud;
    pw::Cloud::Ptr m_kp_source;
    pw::Cloud::Ptr m_kp_target;
    pw::CorrespondencesPtr m_correspondences;

    QFutureWatcher<pw::RegistrationResult> m_watcher;
    std::shared_ptr<std::atomic<bool>> m_cancel_flag;
    pw::ParamSnapshot m_last_compute_snapshot;

    static constexpr const char* PREVIEW_ID = "gr_preview";
    static constexpr const char* KP_SRC_ID = "gr_kp_src";
    static constexpr const char* KP_TGT_ID = "gr_kp_tgt";
};

#endif // POINTWORKS_GLOBAL_REGISTRATION_H
