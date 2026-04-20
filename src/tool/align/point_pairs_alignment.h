#ifndef POINTWORKS_POINT_PAIRS_ALIGNMENT_H
#define POINTWORKS_POINT_PAIRS_ALIGNMENT_H

#include "ui/base/customdialog.h"
#include "ui/base/paramsnapshot.h"
#include "algorithm/registration.h"
#include "core/cloud.h"

#include <QComboBox>
#include <QTableWidget>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QShowEvent>
#include <QCloseEvent>
#include <QFutureWatcher>
#include <atomic>


class PointPairsAlignment : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit PointPairsAlignment(QWidget* parent = nullptr);
    ~PointPairsAlignment() override;

    void init() override;
    void reset() override;
    void deinit() override;

protected:
    void showEvent(QShowEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

private slots:
    void onStartStop();
    void onReset();
    void onAlign();
    void onApply();
    void onCancel();
    void onMouseLeftPressed(const ct::PointXY& pt);
    void onToggleSourceVisibility(bool visible);
    void onToggleTargetVisibility(bool visible);
    void onAddSourcePoint();
    void onAddTargetPoint();
    void onClearSourcePoints();
    void onClearTargetPoints();
    void onImportTargetPoints();
    void onImportSourcePoints();
    void onSwapSourceTarget();
    void onRmsFilterToggled(bool checked);

private:
    void setupUi();
    void stopPicking();
    bool addPointManual(bool is_source, double x, double y, double z);
    void updateAllLines();
    void clearAllLines();
    void updateTables();
    void updateTableErrors(const ct::PointPairErrorResult& result);
    void deleteSourcePoint(int index);
    void deleteTargetPoint(int index);
    void rebuildMarkers();
    void clearLabels();
    void rebuildLabels();
    QVector<Eigen::Vector3d> displaySourcePoints() const;
    double calcLabelScale() const;
    void refreshCloudList();
    ct::PickResult doPick(const ct::PointXY& pt);
    ct::ConstrainedTransformParams currentConstraintParams() const;
    void hideNonSelectedClouds();
    void restoreAllCloudVisibility();

    // --- 控件（_ 后缀） ---
    QComboBox* cbox_source_;
    QComboBox* cbox_target_;
    QPushButton* btn_swap_;

    QGroupBox* group_source_;
    QCheckBox* check_show_source_;
    QPushButton* btn_clear_src_;
    QPushButton* btn_manual_src_;
    QPushButton* btn_import_src_;
    QTableWidget* table_source_;

    QGroupBox* group_target_;
    QCheckBox* check_show_target_;
    QPushButton* btn_clear_tgt_;
    QPushButton* btn_manual_tgt_;
    QPushButton* btn_import_tgt_;
    QTableWidget* table_target_;

    QLabel* label_pair_count_;
    QLabel* label_rms_;

    // 约束控件
    QCheckBox* check_adjust_scale_;
    QComboBox* cbox_rotation_;
    QCheckBox* check_tx_;
    QCheckBox* check_ty_;
    QCheckBox* check_tz_;
    QCheckBox* check_rms_filter_;
    QDoubleSpinBox* spin_rms_threshold_;

    QPushButton* btn_start_;
    QPushButton* btn_reset_;
    QPushButton* btn_align_;
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;
    QPushButton* btn_close_;
    QLabel* label_status_;

    // --- 业务数据（m_ 前缀） ---
    bool m_is_picking;
    bool m_is_computing;
    std::atomic<bool> m_canceled;
    QVector<Eigen::Vector3d> m_source_points;
    QVector<Eigen::Vector3d> m_target_points;
    ct::PointPairErrorResult m_last_result;
    bool m_has_preview;
    ct::Cloud::Ptr m_original_source_cloud;  // 缓存原始源点云用于还原
    QString m_source_id;                     // 源点云 ID
    ct::ParamSnapshot m_last_align_snapshot;

    static constexpr const char* MARKER_SRC_ID = "ppa_markers_src";
    static constexpr const char* MARKER_TGT_ID = "ppa_markers_tgt";
    static constexpr const char* LINES_ID = "point_pairs_lines";
    static constexpr const char* PREVIEW_ID = "ppa_preview";
};

#endif // POINTWORKS_POINT_PAIRS_ALIGNMENT_H
