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

Q_DECLARE_METATYPE(pw::M3C2Params)

class M3C2Plugin : public pw::CustomDialog {
Q_OBJECT

public:
    explicit M3C2Plugin(QWidget *parent = nullptr);
    ~M3C2Plugin() override;

    void init() override;

private slots:
    void onApply();
    void onCancel();
    void onGuessParams();

private:
    Ui::M3C2Plugin *ui;
    pw::Cloud::Ptr m_refCloud;
    pw::Cloud::Ptr m_compCloud;
};

#endif // POINTWORKS_M3C2PLUGIN_H
