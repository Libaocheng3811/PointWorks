#ifndef POINTWORKS_M3C2PLUGIN_H
#define POINTWORKS_M3C2PLUGIN_H

#include <QDialog>
#include <QMetaType>

#include "core/common.h"
#include "algorithm/distancecalculator.h"
#include "ui/base/customdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class M3C2Plugin; }
QT_END_NAMESPACE

Q_DECLARE_METATYPE(ct::M3C2Params)

class M3C2Plugin : public ct::CustomDialog {
Q_OBJECT

public:
    explicit M3C2Plugin(QWidget *parent = nullptr);
    ~M3C2Plugin() override;

    void init() override;

private slots:
    void onApply();
    void onCancel();

private:
    Ui::M3C2Plugin *ui;
    ct::Cloud::Ptr m_refCloud;
    ct::Cloud::Ptr m_compCloud;
};

#endif // POINTWORKS_M3C2PLUGIN_H
