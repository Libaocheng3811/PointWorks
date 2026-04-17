//
// Created by LBC on 2026/1/18.
//

#ifndef POINTWORKS_CHANGEDETECTDIALOG_H
#define POINTWORKS_CHANGEDETECTDIALOG_H

#include <QDialog>
#include <QMetaType>

#include "core/common.h"
#include "algorithm/distancecalculator.h"
#include "ui/base/customdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class ChangeDetectPlugin; }
QT_END_NAMESPACE

Q_DECLARE_METATYPE(ct::DistanceParams)

class ChangeDetectPlugin : public ct::CustomDialog {
Q_OBJECT

public:
    explicit ChangeDetectPlugin(QWidget *parent = nullptr);

    ~ChangeDetectPlugin() override;

    void init() override;

private slots:
    void onApply();

    void onCancel();

    void onMethodChanged(int index);

private:
    Ui::ChangeDetectPlugin *ui;

    ct::Cloud::Ptr m_refCloud;
    ct::Cloud::Ptr m_compCloud;

    // 缓存当前参数
    double m_threshold;
};


#endif //POINTWORKS_CHANGEDETECTDIALOG_H
