#ifndef CT_EDIT_SCALE_H
#define CT_EDIT_SCALE_H

#include "ui/base/customdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
    class Scale;
}
QT_END_NAMESPACE

#define SCALE_PRE_FLAG  "-scale"
#define SCALE_ADD_FLAG  "scaled-"

class Scale : public ct::CustomDialog {
    Q_OBJECT

public:
    explicit Scale(QWidget* parent = nullptr);
    ~Scale() override;

    void add();
    void apply();
    void reset() override;

signals:
    void scaleChanged(double x, double y, double z);

private slots:
    void preview(double x, double y, double z);

private:
    Ui::Scale* ui;
    std::map<std::string, ct::Cloud::Ptr> m_scale_map;
};

#endif // CT_EDIT_SCALE_H
