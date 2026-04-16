#ifndef POINTWORKS_COLOR_H
#define POINTWORKS_COLOR_H

#include "ui/base/customdialog.h"
#include "base/scenenodetype.h"

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
    QColor m_rgb;
    bool m_restore_default = false;
    bool m_applied = false;

    bool hasPointCloudSelection() const;
    bool hasMeshSelection() const;
    bool isPointCloudOnly() const;

    /**
     * @brief 更新 UI 控件状态（坐标轴按钮、目标下拉框可见性）
     */
    void updateUIState();

protected:
    void closeEvent(QCloseEvent* event) override;
    void showEvent(QShowEvent* event) override;
};


#endif //POINTWORKS_COLOR_H
