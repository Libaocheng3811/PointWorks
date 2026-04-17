#ifndef POINTWORKS_MORPHOLOGICAL_FILTER_DIALOG_H
#define POINTWORKS_MORPHOLOGICAL_FILTER_DIALOG_H

#include "ui/base/customdialog.h"
#include "algorithm/segmentation.h"

#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>

#include <QFutureWatcher>
#include <atomic>

class MorphologicalFilterDialog : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit MorphologicalFilterDialog(QWidget* parent = nullptr);
    ~MorphologicalFilterDialog() override;

    void init() override;
    void reset() override;
    void deinit() override;

private slots:
    void onPreview();
    void onApply();
    void onReset();

private:
    void setupUi();
    void runFilter(bool preview);

    // --- 异步执行 ---
    std::atomic<bool> m_canceled{false};
    QFutureWatcher<ct::SegmentationResult>* m_watcher = nullptr;

    // --- 控件（_ 后缀） ---
    QSpinBox* spin_max_window_size_;
    QDoubleSpinBox* dspin_slope_;
    QDoubleSpinBox* dspin_max_distance_;
    QDoubleSpinBox* dspin_initial_distance_;
    QDoubleSpinBox* dspin_cell_size_;
    QDoubleSpinBox* dspin_base_;
    QCheckBox* check_negative_;

    // Buttons
    QPushButton* btn_preview_;
    QPushButton* btn_apply_;
    QPushButton* btn_reset_;

    // --- 业务数据（m_ 前缀） ---
    ct::Cloud::Ptr m_cloud;
    std::vector<ct::Cloud::Ptr> m_preview_clouds;

    static constexpr const char* PREVIEW_PREFIX = "mf_preview_";
};

#endif // POINTWORKS_MORPHOLOGICAL_FILTER_DIALOG_H
