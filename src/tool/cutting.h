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

    void add();

    void apply();

    void start();

    virtual void reset();

    virtual void deinit() { m_cloudview->clearInfo(); }

private:
    void updateInfo(int index);

    void cuttingCloud(bool select_in);

public slots:
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

private:
    Ui::Cutting *ui;
    // 表示当前是否处于选择模式，如果为false，表示当前没有在选择模式中，
    bool is_picking;
    // pick_start表示是否开始了裁剪，也就是是否开始框选点云
    bool pick_start;
    // m_pick_points用来存储裁剪时被选择的点的集合
    std::vector<ct::PointXY> m_pick_points;
    // m_cutting_map 存储了当前正在进行切割操作的点云信息。
    std::map<std::string, ct::Cloud::Ptr> m_cutting_map;
};


#endif //POINTWORKS_CUTTING_H
