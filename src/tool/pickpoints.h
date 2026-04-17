//
// Created by LBC on 2024/12/17.
//

#ifndef POINTWORKS_PICKPOINTS_H
#define POINTWORKS_PICKPOINTS_H

#include "ui/base/customdialog.h"
#include "core/common.h"

#include <QDateTime>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class PickPoints;
}
QT_END_NAMESPACE

enum PickMode{
    PICK_SINGLE = 0, // 单点选取
    PICK_PAIR = 1,   // 两点（显示距离）
    PICK_MULTI = 2   // 多点（多边形）
};

class PickPoints : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit PickPoints(QWidget *parent = nullptr);

    ~PickPoints() override;

    virtual void init();
    // 开始选点模式
    void start();
    // 将选中的点取出，单独成一项
    void add();
    // 重置功能
    virtual void reset();
    virtual void deinit() { m_cloudview->clearInfo(); }

private:
    void updateInfo(int index);
    void updatePanelInfo(const ct::PickResult& res);

public slots:
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

private:
    Ui::PickPoints *ui;
    bool is_picking;
    bool pick_start;
    // 选中的点云
    ct::Cloud::Ptr m_selected_cloud;
    ct::Cloud::Ptr m_pick_cloud; // 临时显示的红色选点
    // 拾取的点
    ct::PointXY m_pick_point;
};


#endif //POINTWORKS_PICKPOINTS_H
