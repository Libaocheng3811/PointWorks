//
// Created by LBC on 2026/3/24.
//

#ifndef POINTWORKS_SAMPLING_H
#define POINTWORKS_SAMPLING_H

#include "ui/base/customdialog.h"
#include "algorithm/filters.h"

namespace Ui
{
    class Sampling;
}

/**
 * @brief 采样对话框 - 模态对话框，执行采样后生成新点云挂到原点云下
 */
class Sampling : public pw::CustomDialog
{
    Q_OBJECT

public:
    explicit Sampling(QWidget* parent = nullptr);
    ~Sampling() override;

    void init() override;

private slots:
    void onOkClicked();
    void onCancelClicked();

    void handleSamplingResult(const pw::FilterResult& result);

private:
    Ui::Sampling* ui;

    // 当前正在处理的点云
    pw::Cloud::Ptr m_current_cloud;
    bool m_cancel = false;

    enum SamplingMethod {
        METHOD_DOWNSAMPLING = 0,
        METHOD_UNIFORMSAMPLING = 1,
        METHOD_RANDOMSAMPLING = 2,
        METHOD_RESAMPLING = 3,
        METHOD_SAMPLINGSURFACENORMAL = 4,
        METHOD_NORMALSPACESAMPLING = 5
    };
};

#endif //POINTWORKS_SAMPLING_H
