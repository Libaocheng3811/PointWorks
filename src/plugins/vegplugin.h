//
// Created by LBC on 2026/1/6.
//

#ifndef POINTWORKS_VEGPLUGIN_H
#define POINTWORKS_VEGPLUGIN_H

#include <QDialog>

#include "ui/base/customdialog.h"
#include "algorithm/vegfilter.h"

QT_BEGIN_NAMESPACE
namespace Ui { class VegPlugin; }
QT_END_NAMESPACE

class VegPlugin : public pw::CustomDialog {
Q_OBJECT

public:
    explicit VegPlugin(QWidget *parent = nullptr);

    ~VegPlugin() override;

    void init() override;

private slots:
    void onIndexChanged(int index);
    void onApply();

private:
    Ui::VegPlugin *ui;
    pw::Cloud::Ptr m_cloud;
};

#endif //POINTWORKS_VEGPLUGIN_H
