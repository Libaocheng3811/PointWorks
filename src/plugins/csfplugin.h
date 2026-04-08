//
// Created by LBC on 2026/1/4.
//

#ifndef POINTWORKS_CSFPLUGIN_H
#define POINTWORKS_CSFPLUGIN_H

#include <QDialog>

#include "ui/base/customdialog.h"
#include "algorithm/csffilter.h"


QT_BEGIN_NAMESPACE
namespace Ui { class CSFPlugin; }
QT_END_NAMESPACE

class CSFPlugin : public ct::CustomDialog {
Q_OBJECT

public:
    explicit CSFPlugin(QWidget *parent = nullptr);

    ~CSFPlugin() override;

    void init() override;

private slots:
    void onApply();
    void onCancel();

private:
    Ui::CSFPlugin *ui;
    ct::Cloud::Ptr m_cloud;
};


#endif //POINTWORKS_CSFPLUGIN_H
