#ifndef POINTWORKS_COLOR_H
#define POINTWORKS_COLOR_H

#include "ui/base/customdialog.h"


QT_BEGIN_NAMESPACE
namespace Ui
{
    class Color;
}
QT_END_NAMESPACE

class Color : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Color(QWidget *parent = nullptr);

    ~Color();

    void apply();
    virtual void reset();

signals:
    void rgb(const QColor& rgb);
    void field(const QString& field);

public slots:
    void setColorRGB(const QColor& rgb);
    void setColorField(const QString& field);

private:
    Ui::Color *ui;
    QString m_field;
    // QColor 是 Qt 框架中的一个类，用于表示颜色,可存储颜色的不同属性，如红、绿、蓝（RGB）值，以及透明度（Alpha）值。
    QColor m_rgb;
    bool m_restore_default = false;
    bool m_applied = false;

protected:
    void closeEvent(QCloseEvent* event) override;
};


#endif //POINTWORKS_COLOR_H
