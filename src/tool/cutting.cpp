//
// Created by LBC on 2024/11/26.
//

// You may need to build the project (run Qt uic code generator) to get "ui_Cutting.h" resolved

#include "cutting.h"
#include "base/cloudtree.h"
#include "ui_Cutting.h"
#include "core/cloud.h"

#include <pcl/filters/extract_indices.h>

#define CUT_TYPE_RECTANGULAR    0
#define CUT_TYPE_POLYGONAL      1

#define POLYGONAL_ID        "polygonal"

#define CUTTING_PRE_FLAG    "-cutting"
#define CUTTING_ADD_FLAG    "cutting-"

Cutting::Cutting(QWidget *parent) :CustomDialog(parent),
ui(new Ui::Cutting), is_picking(false), pick_start(false)
{
    ui->setupUi(this);

    connect(ui->btn_add, &QPushButton::clicked, this, &Cutting::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Cutting::apply);
    connect(ui->btn_start, &QPushButton::clicked, this, &Cutting::start);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Cutting::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Cutting::close);
    connect(ui->btn_selectin, &QPushButton::clicked, [=] {this->cuttingCloud(false); });
    connect(ui->btn_selectout, &QPushButton::clicked, [=] {this->cuttingCloud(true); });
    // 将下拉框的选项设置为第一个选项，下拉框的索引是从0开始的，
    ui->cbox_type->setCurrentIndex(0);
}

Cutting::~Cutting()
{
    delete ui;
}

void Cutting::init()
{
    /**
     * @brief 函数指针， 函数指针是指向函数的指针，可以通过该指针调用指向的函数。指针的类型必须与函数的签名相匹配，包括返回类型和参数类型。
     * 在 Qt 框架中，信号和槽机制依赖于函数指针来形成信号与槽的连接
     * @brief 成员函数指针与普通函数指针稍有不同，因为它们需要一个对象的对应实例来调用。
     * 成员函数指针的类型形态是 返回类型 (类名::*)(参数类型)。
     * @brief static_cast 是一种 C++ 中强制转换的方式，用于安全地转换指针和引用类型
     */
    connect(ui->cbox_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Cutting::updateInfo);
    this->updateInfo(ui->cbox_type->currentIndex());
}

void Cutting::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_cutting_map.find(cloud->id()) == m_cutting_map.end())
        {
            printW(QString("The Cloud[id:%1] has no cutted cloud!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_cutting_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
        m_cloudview->removeShape(QString::fromStdString(new_cloud->boxId()));
        new_cloud->setId(CUTTING_ADD_FLAG + cloud->id());
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        // 策略一：裁剪结果作为兄弟节点挂载
        m_cloudtree->insertCloud(new_cloud, item, true, ct::MountStrategy::Sibling);

        m_cutting_map.erase(cloud->id());
        printI(QString("Add cutted cloud[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_pick_points.clear();
}

void Cutting::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clods = m_cloudtree->getSelectedClouds();
    if (selected_clods.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clods)
    {
        if (m_cutting_map.find(cloud->id()) == m_cutting_map.end())
        {
            printW(QString("The cloud[id:%1] has no cutted cloud !").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_cutting_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
        m_cloudview->removeShape(QString::fromStdString(new_cloud->boxId()));
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_cutting_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply cutted cloud[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_pick_points.clear();

}

/**
 * @brief 问题找到了--之所以裁剪时鼠标事件没有反应是因为就没有发射鼠标事件信号
 * 问题1： 这个鼠标事件是在父类中怎么定义的，信号发射是由框架执行的吗？为什么要手动发射信号
 */
void Cutting::start()
{
    // 当前不在选择模式（可认为是编辑模式）时，进入选择模式，禁用一些状态
    if (!is_picking)
    {
        std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
        if (selected_clouds.empty())
        {
            printW("Please select a cloud!");
            return;
        }
        is_picking = true;
        // 设置文件树为不可选状态
        m_cloudtree->setEnabled(false);
        // 禁用视图交互
        m_cloudview->setInteractorEnable(false);
        for (auto& cloud : selected_clouds)
        {
            // 移除所选点云的包围盒
            m_cloudview->removeShape(QString::fromStdString(cloud->boxId()));
        }
        //
        for (auto& cut_cloud : m_cutting_map)
        {
            m_cloudview->removePointCloud(QString::fromStdString(cut_cloud.second->id()));
            m_cloudview->removeShape(QString::fromStdString(cut_cloud.second->boxId()));
        }
        // 移除法线
        m_cloudview->removeShape(POLYGONAL_ID);

        // 在启用选择（编辑）模式之后，设置更新显示的图标
        ui->btn_start->setIcon(QIcon(":/res/icon/stop.svg"));
        /**
         * @brief 信号和槽函数说明
         * 信号本身没有返回值，信号的主要作用是通知槽某个时间发生了，所以信号的返回类型为void
         * 槽函数可以有返回类型，但是实际中，建议槽的返回类型也定义为void，因为即使槽返回了值，也不会对信号发出者产生任何影响，，
         * 信号发出后，只会调用槽函数，而不关心槽的返回值。
         */

        // 这里将m_cloudview的鼠标事件信号与Cutting类的事件处理函数关联，以便在发生鼠标事件时，能发射信号并执行相关操作
        // 之所以不能裁剪，是因为在CloudView类中你没有定义事件处理器函数，当在CloudView类中发生鼠标事件时，事件处理函数没能执行，就不会发射对应的mouseLeftPressed信号
        connect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &Cutting::mouseLeftPressed);
        connect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &Cutting::mouseLeftReleased);
        connect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Cutting::mouseRightReleased);
        connect(m_cloudview, &ct::CloudView::mouseMoved, this, &Cutting::mouseMoved);
        this->updateInfo(ui->cbox_type->currentIndex());
    }
    // 当前处于选择模式时，退出选择模式，启用状态
    else
    {
        is_picking = false;
        // 这里只启用了文件树，未启用m_cloudview交互,
        m_cloudtree->setEnabled(true);
        // 解除信号和槽函数的关联
        disconnect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &Cutting::mouseLeftPressed);
        disconnect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &Cutting::mouseLeftReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Cutting::mouseRightReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Cutting::mouseRightReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseMoved, this, &Cutting::mouseMoved);
        // 更新按钮图标
        ui->btn_start->setIcon(QIcon(":/res/icon/start.svg"));
        // 调用 updateInfo 函数并传递当前选中的下拉框索引，以便更新界面或执行相关逻辑
        this->updateInfo(ui->cbox_type->currentIndex());
    }
}

void Cutting::reset()
{
    // 如果已经处于编辑模式，
    if (is_picking) this->start();
    pick_start = false;
    // 激活视图交互功能
    m_cloudview->setInteractorEnable(true);
    m_cloudview->removeShape(POLYGONAL_ID);
    for (auto& cut_cloud : m_cutting_map)
    {
        m_cloudview->removePointCloud(QString::fromStdString(cut_cloud.second->id()));
        m_cloudview->removeShape(QString::fromStdString(cut_cloud.second->boxId()));
    }
    m_pick_points.clear();
}

void Cutting::updateInfo(int index)
{
    if (index == CUT_TYPE_RECTANGULAR)
    {
        if (is_picking)
            m_cloudview->showInfo("Segmentation [ON] (rectangular selection)", 1);
        else
            m_cloudview->showInfo("Segmentation [OFF] (rectangular selection)", 1);
        m_cloudview->showInfo("Left/Right click : set opposite corners", 2);
    }
    else if (index == CUT_TYPE_POLYGONAL)
    {
        if (is_picking)
            m_cloudview->showInfo("Segmentation [ON] (polygonal selection)", 1);
        else
            m_cloudview->showInfo("Segmentation [OFF] (polygonal selection)", 1);
        m_cloudview->showInfo("Left click : add contour points / Right click : close", 2);
    }
}

void Cutting::cuttingCloud(bool select_in)
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_pick_points.size() < 3)
    {
        printW("Picked points are not enough!");
        return;
    }
    // 更新开始按钮状态
    if (is_picking) this->start();
    // 启用视图交互，并移除包围盒
    m_cloudview->setInteractorEnable(true);
    m_cloudview->removeShape(POLYGONAL_ID);
    // 遍历选中的点云
    for (auto& cloud : selected_clouds)
    {
        ct::Cloud::Ptr cut_cloud = m_cloudview->areaPick(m_pick_points, cloud, select_in);

        if (!cut_cloud || cut_cloud->empty()) {
            printW("No points selected.");
            continue;
        }

        cut_cloud->setId(cloud->id() + CUTTING_PRE_FLAG);

        m_cloudview->addPointCloud(cut_cloud);
        m_cloudview->addBox(cut_cloud);
        m_cloudview->setPointCloudColor(cut_cloud, ct::Color::Green); // 新接口
        m_cloudview->setPointCloudSize(QString::fromStdString(cut_cloud->id()), cloud->pointSize() + 2); // 保持原逻辑

        m_cutting_map[cloud->id()] = cut_cloud;
    }
}

void Cutting::mouseLeftPressed(const ct::PointXY &pt)
{
    // 初始化时pick_start为false，当你进入编辑状态并且在点云视图中按下鼠标左键时，此槽函数执行，并更新pick_start的状态，表示正在编辑中
    if (!pick_start)
    {
        m_pick_points.clear();
        // 将左键按下时的选择的点存储到m_pick_points中
        m_pick_points.push_back(pt);
        pick_start = true;
    }
}

void Cutting::mouseLeftReleased(const ct::PointXY &pt)
{
    // 只有在编辑模式下才生效（只有左键按下之后才有效）
    if (pick_start)
    {
        // front()函数获取容器的第一个元素
        // 如果左键释放时的位置和左键按下的位置一样，说明没有框选，直接返回
        if ((m_pick_points.front().x == pt.x) && (m_pick_points.front().y == pt.y))
            return;

        // 就是说有两种编辑情况：1、左键按下后不释放，拖动鼠标，直至绘制图形完成后释放，这算一种绘制编辑方式
        // 2、左键按下紧接着释放，拖动鼠标，图形绘制后再按下左键并释放，完成编辑绘制
        else
        {
            // 对应情况1的编辑方式
            if (ui->cbox_type->currentIndex() == CUT_TYPE_RECTANGULAR)
            {
                ct::PointXY p1(m_pick_points.front().x, pt.y);
                ct::PointXY p2(pt.x, m_pick_points.front().y);
                // 点要按一定的顺序添加，否则会出现交叉
                m_pick_points.push_back(p1);
                m_pick_points.push_back(pt);
                m_pick_points.push_back(p2);
                m_cloudview->addPolygon2D(m_pick_points, POLYGONAL_ID, ct::Color::Green);
                // 完成图形绘制后，设置编辑状态
                pick_start = false;
            }
            else
            {
                //
                m_pick_points.push_back(pt);
            }
        }
    }
}

void Cutting::mouseRightReleased(const ct::PointXY &pt)
{
    // 这里只写了右键释放时的槽函数，是因为不需要右键按下时执行任何操作，右键释放时执行这个槽函数
    if (pick_start)
    {
        // 无论是矩形还是任意多边形绘制模式，都更新pick_start状态为false，表示完成了框选，以便正确响应下次鼠标左键点击事件
        if (ui->cbox_type == CUT_TYPE_RECTANGULAR)
        {
            ct::PointXY p1(m_pick_points.front().x, pt.y);
            ct::PointXY p2(pt.x, m_pick_points.front().y);
            m_pick_points.push_back(p1);
            m_pick_points.push_back(pt);
            m_pick_points.push_back(p2);
            pick_start = false;
        }
        else
        {
            // 如果m_pick_points中已经有两个点时，将当前右键释放时的点也放入m_pick_points中，
            // 不是两个点的情况下，不将右键释放时的点放入m_pick_points中，因为此时已经可以构成多边形或不能构成多边形（只有一个点的情况）
            if (m_pick_points.size() == 2) m_pick_points.push_back(pt);
            // 这里不用单独判断只有一个点的情况是因为最终在PCL可视化类上绘制多边形时，只有一个点的情况下会直接返回，不执行任何操作
            m_cloudview->addPolygon2D(m_pick_points, POLYGONAL_ID, ct::Color::Green);
            pick_start = false;
        }
    }
}

void Cutting::mouseMoved(const ct::PointXY &pt)
{
    // 只有处于开始编辑状态下才执行
    if (pick_start)
    {
        // 判断当前绘制形状是否为矩形
        if (ui->cbox_type->currentIndex() == CUT_TYPE_RECTANGULAR)
        {
            // 创建向量来存储用于绘制矩形的点，
            // 一个点是鼠标左键按下时的点，一个点时鼠标当前位置的点，另外两个由这两个点推导而来
            std::vector<ct::PointXY> pre_points = m_pick_points;
            ct::PointXY p1(pre_points.front().x, pt.y);
            ct::PointXY p2(pt.x, pre_points.front().y);
            pre_points.push_back(p1);
            pre_points.push_back(pt);
            pre_points.push_back(p2);
            // 最终pre_points中包含四个二维点，用于绘制矩形
            m_cloudview->addPolygon2D(pre_points, POLYGONAL_ID, ct::Color::Green);
        }
        // 如果绘制模式为任意多边形模式
        else
        {
            // 将鼠标左键按下时的点和当前鼠标位置的点存储在pre_points中用于绘制任意多边形
            std::vector<ct::PointXY> pre_points = m_pick_points;
            pre_points.push_back(pt);
            m_cloudview->addPolygon2D(pre_points, POLYGONAL_ID, ct::Color::Green);
        }
    }
}