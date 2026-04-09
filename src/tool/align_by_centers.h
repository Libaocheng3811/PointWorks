#ifndef POINTWORKS_ALIGN_BY_CENTERS_H
#define POINTWORKS_ALIGN_BY_CENTERS_H

#include "ui/base/customdialog.h"
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
    void deinit() override {}

private slots:
    void onAlign();
    void onAlignFinished();

private:
    void setupUi();
    void refreshCloudList();

    QComboBox* cbox_source_;
    QComboBox* cbox_target_;
    QPushButton* btn_align_;

    Eigen::Matrix4f m_matrix;
    std::atomic<bool> m_canceled;
    QString m_source_id;
};

#endif // POINTWORKS_ALIGN_BY_CENTERS_H
