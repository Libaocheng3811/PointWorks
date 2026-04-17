#ifndef POINTWORKS_CLUSTERING_DIALOG_H
#define POINTWORKS_CLUSTERING_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/segmentation.h"

#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>
#include <QWidget>
#include <QPushButton>
#include <QStackedWidget>
#include <QGroupBox>
#include <QSpinBox>

#include <QFutureWatcher>
#include <atomic>

class ClusteringDialog : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit ClusteringDialog(QWidget* parent = nullptr);
    ~ClusteringDialog() override;

    void init() override;
    void reset() override;

private slots:
    void onAlgorithmChanged(int index);
    void onDimensionToggled();
    void onWeightChanged();
    void onApply();
    void onCancel();

private:
    void setupUi();
    QWidget* createEuclideanParamPage();
    QWidget* createDBSCANParamPage();
    QWidget* createKMeansParamPage();
    QWidget* createWeightPanel();
    void updateWeightUI(double values[3]);
    int findChangedIndex(double* old_values, double* new_values);

    // --- 异步执行 ---
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<ct::SegmentationResult>* m_watcher = nullptr;

    // --- 控件（_ 后缀） ---
    QComboBox* cbox_algorithm_;
    QStackedWidget* param_pages_;

    // 维度勾选
    QGroupBox* dim_group_;
    QCheckBox* check_normal_;
    QCheckBox* check_color_;

    // 权重面板
    QWidget* weight_panel_;
    QWidget* row_normal_;
    QWidget* row_color_;
    QSlider* slider_pos_;
    QSlider* slider_normal_;
    QSlider* slider_color_;
    QDoubleSpinBox* dspin_pos_;
    QDoubleSpinBox* dspin_normal_;
    QDoubleSpinBox* dspin_color_;
    QLabel* label_weight_hint_;

    // Euclidean 参数
    QDoubleSpinBox* dspin_tolerance_;
    QSpinBox* spin_min_cluster_;
    QSpinBox* spin_max_cluster_;

    // DBSCAN 参数
    QDoubleSpinBox* dspin_eps_;
    QSpinBox* spin_min_pts_;

    // K-Means 参数
    QSpinBox* spin_k_;
    QSpinBox* spin_max_iter_;

    // 输出
    QCheckBox* check_split_;

    // 按钮
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;

    // --- 业务数据（m_ 前缀） ---
    ct::Cloud::Ptr m_cloud;
};

#endif // POINTWORKS_CLUSTERING_DIALOG_H
