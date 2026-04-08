//
// Created by LBC on 2024/11/13.
//

#ifndef POINTWORKS_BOUNDINGBOX_H
#define POINTWORKS_BOUNDINGBOX_H

#include "ui/base/customdialog.h"


QT_BEGIN_NAMESPACE
namespace Ui
{
    class BoundingBox;
}
QT_END_NAMESPACE

class BoundingBox : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit BoundingBox(QWidget *parent = nullptr);

    ~BoundingBox() override;

    void preview();
    void apply();
    virtual void reset();

signals:
    void eulerAngles(float r, float p, float y);

public slots:
    void adjustEnable(bool state);
    void adjustBox(float r, float p, float y);

private:
    Ui::BoundingBox *ui;
    int m_box_type;
    std::map<std::string, ct::Box> m_box_map;
};


#endif //POINTWORKS_BOUNDINGBOX_H
