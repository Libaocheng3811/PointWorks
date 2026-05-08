#ifndef POINTWORKS_RECONSTRUCT_SURFACE_DIALOG_H
#define POINTWORKS_RECONSTRUCT_SURFACE_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/surface.h"
#include "viz/surface_viz_helper.h"

#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QSlider>
#include <QLabel>
#include <QWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QGroupBox>

#include <QFutureWatcher>
#include <atomic>

class ReconstructSurfaceDialog : public pw::CustomDialog
{
    Q_OBJECT

public:
    explicit ReconstructSurfaceDialog(QWidget* parent = nullptr);
    ~ReconstructSurfaceDialog() override;

    void init() override;
    void reset() override;

private slots:
    void onAlgorithmChanged(int index);
    void onPreview();
    void onApply();
    void onCancel();
    void onClose();

private:
    void setupUi();
    QWidget* createPoissonParamPage();
    QWidget* createGreedyParamPage();
    QWidget* createMarchingCubesParamPage();
    QWidget* createGridProjectionParamPage();
    void checkNormalsWarning();
    void runReconstruct(bool is_preview);

    // --- 异步执行 ---
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<pw::SurfaceResult>* m_watcher = nullptr;

    // --- 控件（_ 后缀） ---
    QComboBox* cbox_algorithm_;
    QStackedWidget* param_pages_;
    QLabel* label_warning_;

    // Poisson 参数
    QSpinBox* spin_depth_;
    QDoubleSpinBox* dspin_point_weight_;
    QSpinBox* spin_min_depth_;
    QDoubleSpinBox* dspin_scale_;
    QSpinBox* spin_solver_divide_;
    QSpinBox* spin_iso_divide_;
    QDoubleSpinBox* dspin_samples_per_node_;
    QCheckBox* check_confidence_;
    QCheckBox* check_manifold_;

    // Greedy 参数
    QDoubleSpinBox* dspin_search_radius_;
    QDoubleSpinBox* dspin_mu_;
    QSpinBox* spin_max_neighbors_;
    QDoubleSpinBox* dspin_min_angle_;
    QDoubleSpinBox* dspin_max_angle_;
    QDoubleSpinBox* dspin_eps_angle_;
    QCheckBox* check_consistent_;

    // Marching Cubes 参数
    QDoubleSpinBox* dspin_iso_level_;
    QSpinBox* spin_grid_res_;
    QDoubleSpinBox* dspin_percentage_;
    QDoubleSpinBox* dspin_epsilon_;

    // Grid Projection 参数
    QDoubleSpinBox* dspin_resolution_;
    QSpinBox* spin_padding_size_;
    QSpinBox* spin_k_;

    // 后处理
    QCheckBox* check_extract_boundary_;

    // Preview 下采样
    QDoubleSpinBox* dspin_downsample_rate_;

    // 按钮
    QPushButton* btn_preview_;
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;
    QPushButton* btn_close_;

    // --- 业务数据（m_ 前缀） ---
    pw::Cloud::Ptr m_cloud;
    QStringList m_preview_ids;  // preview 添加的 mesh/boundary ID，关闭时清理
};

#endif // POINTWORKS_RECONSTRUCT_SURFACE_DIALOG_H
