#ifndef POINTWORKS_ALIGN_BY_CENTERS_H
#define POINTWORKS_ALIGN_BY_CENTERS_H

#include "ui/base/customdialog.h"
#include "ui/base/paramsnapshot.h"
#include "core/cloud.h"

#include <QComboBox>
#include <QLabel>
#include <QPushButton>

#include <QFutureWatcher>
#include <atomic>

class AlignByCentersDialog : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit AlignByCentersDialog(QWidget* parent = nullptr);
    ~AlignByCentersDialog() override;

    void init() override;
    void reset() override;
    void deinit() override;

private slots:
    void onAlign();
    void onAlignFinished();
    void onApply();
    void onCancel();

private:
    void setupUi();
    void refreshCloudList();

    QComboBox* cbox_source_;
    QComboBox* cbox_target_;
    QPushButton* btn_align_;
    QPushButton* btn_apply_;
    QPushButton* btn_cancel_;

    Eigen::Matrix4f m_matrix;
    std::atomic<bool> m_canceled;
    QString m_source_id;
    ct::Cloud::Ptr m_aligned_cloud;
    bool m_has_preview = false;
    ct::ParamSnapshot m_last_align_snapshot;

    static constexpr const char* PREVIEW_ID = "abc_preview";
};

#endif // POINTWORKS_ALIGN_BY_CENTERS_H
