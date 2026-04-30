//
// Created by LBC on 2024/11/26.
//

#ifndef POINTWORKS_CUTTING_H
#define POINTWORKS_CUTTING_H

#include <QDialog>
#include "ui/base/customdialog.h"


QT_BEGIN_NAMESPACE
namespace Ui
{
    class Cutting;
}
QT_END_NAMESPACE

class Cutting : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Cutting(QWidget *parent = nullptr);

    ~Cutting();

    virtual void init();

    void selectIn();

    void selectOut();

    void confirm();

    void stopPicking();

    virtual void reset();

    virtual void deinit();

private:
    void updateInfo(int index);
    void updateButtonStates();

    void previewCutting(bool select_in);

public slots:
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

private:
    Ui::Cutting *ui;
    bool is_picking;
    bool pick_start;
    std::vector<ct::PointXY> m_pick_points;
    std::map<std::string, ct::Cloud::Ptr> m_cutting_map;
    // 预览时被隐藏的原点云 ID
    QSet<QString> m_hidden_originals;
    bool m_last_select_in = false;
    void restoreOriginals();
};


#endif //POINTWORKS_CUTTING_H
