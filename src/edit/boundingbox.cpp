//
// Created by LBC on 2024/11/13.
//

// You may need to build the project (run Qt uic code generator) to get "ui_BoundingBox.h" resolved

#include "boundingbox.h"

#include "core/common.h"
#include "algorithm/features.h"

#include "ui_BoundingBox.h"

#define BOX_TYPE_POINTS     (0)
#define BOX_TYPE_WIREFRAME  (1)
#define BOX_TYPE_SURFACE    (2)


BoundingBox::BoundingBox(QWidget *parent) :
        CustomDialog(parent), ui(new Ui::BoundingBox), m_box_type(BOX_TYPE_WIREFRAME)
{
    ui->setupUi(this);

    connect(ui->btn_apply, &QPushButton::clicked, this, &BoundingBox::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &BoundingBox::reset);
    connect(ui->btn_preview, &QPushButton::clicked, this, &BoundingBox::preview);
    connect(ui->check_points, &QCheckBox::clicked, [=](bool checked)
            {
                if (checked)
                {
                    m_box_type = BOX_TYPE_POINTS;
                    ui->check_wireframe->setChecked(false);
                    ui->check_surface->setChecked(false);
                }
            });
    connect(ui->check_wireframe, &QCheckBox::clicked, [=](bool checked)
            {
                if (checked)
                {
                    m_box_type = BOX_TYPE_WIREFRAME;
                    ui->check_points->setChecked(false);
                    ui->check_surface->setChecked(false);
                }
            });
    connect(ui->check_surface, &QCheckBox::clicked, [=](bool checked)
            {
                if (checked)
                {
                    m_box_type = BOX_TYPE_SURFACE;
                    ui->check_points->setChecked(false);
                    ui->check_wireframe->setChecked(false);
                }
            });
    connect(ui->dspin_rx, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            { emit eulerAngles(value, ui->dspin_ry->value(), ui->dspin_rz->value()); });
    connect(ui->dspin_ry, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            { emit eulerAngles( ui->dspin_rx->value(), value, ui->dspin_rz->value()); });
    connect(ui->dspin_rz, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            { emit eulerAngles(ui->dspin_rx->value(), ui->dspin_ry->value(), value); });
}

BoundingBox::~BoundingBox() {
    delete ui;
}

void BoundingBox::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    this->adjustEnable(false);
    for (auto& cloud : selected_clouds)
    {
        ct::Box box;
        /**
         * @brief 包围盒知识
         * AABB包围盒（Axis-aligned bounding box）和OBB包围盒（Oriented bounding box）是两种常用的几何体包围盒算法，
         * 用于近似表示复杂几何体的空间位置，以便于进行快速的空间关系检测，如碰撞检测
         * AABB包围盒：用一个长方体将物体包裹起来，每个面都与某个坐标轴平面平行，但是不能随物体旋转
         * OBB包围盒：是一个可以旋转的长方体，能够更紧密地包围物体，减少包围体的个数
         */
        if (ui->rbtn_aabb->isChecked())
        {
            box = ct::Features::boundingBoxAABB(cloud);
            // 添加包围盒
            m_cloudview->addCube(box, QString::fromStdString(cloud->boxId()));
            // 设置包围盒颜色
            m_cloudview->setShapeColor(QString::fromStdString(cloud->boxId()), ct::Color::Red);
            m_cloudview->showInfo("Axis-Aligned Bounding Box", 1);
        }
        else
        {
            box = ct::Features::boundingBoxOBB(cloud);
            m_cloudview->addCube(box, QString::fromStdString(cloud->boxId()));
            m_cloudview->setShapeColor(QString::fromStdString(cloud->boxId()), ct::Color::Green);
            m_cloudview->showInfo("Oriented Bounding Box", 1);
        }
        // 设置包围盒的表示形式
        m_cloudview->setShapeRepersentation(QString::fromStdString(cloud->boxId()), m_box_type);
        switch (m_box_type)
        {
            case BOX_TYPE_POINTS:
                m_cloudview->setShapeSize(QString::fromStdString(cloud->boxId()), 5);
                break;
            case BOX_TYPE_WIREFRAME:
                m_cloudview->setShapeLineWidth(QString::fromStdString(cloud->boxId()), 3);
                break;
            case BOX_TYPE_SURFACE:
                m_cloudview->setShapeOpacity(QString::fromStdString(cloud->boxId()), 0.5);
                break;
        }
        // 在界面上显示该包围盒对应的欧拉角，如果是AABB包围盒，欧拉角都为0
        float roll, pitch, yaw;
        ct::getEulerAngles(box.pose, roll, pitch, yaw);
        ui->dspin_rx->setValue(roll);
        ui->dspin_ry->setValue(pitch);
        ui->dspin_rz->setValue(yaw);
        m_box_map[cloud->id()] = box;
    }

    this->adjustEnable(true);
}

void BoundingBox::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_box_map.find(cloud->id()) == m_box_map.end())
        {
            printW(QString("The cloud[id%1] has no matched boundingbox!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        cloud->setBox(m_box_map.find(cloud->id())->second);
        m_cloudview->addBox(cloud);
        printI(QString("Apply cloud[%1] boundingbox done.").arg(QString::fromStdString(cloud->id())));
    }
    // 清除视图器中的显示信息
    m_cloudview->clearInfo();
    // 设置欧拉角spinbox的启用状态
    this->adjustEnable(false);
}

void BoundingBox::reset()
{
    m_box_map.clear();
    m_cloudview->clearInfo();
    for (auto& cloud : m_cloudtree->getSelectedClouds())
    {
        m_cloudview->addBox(cloud);
        printI(QString("Reset cloud[id%1] boundingbox done.").arg(QString::fromStdString(cloud->id())));
    }
    this->adjustEnable(false);
}

void BoundingBox::adjustEnable(bool state)
{
    if (state)
    {
        connect(this, &BoundingBox::eulerAngles, this, &BoundingBox::adjustBox);
        ui->dspin_rx->setEnabled(true);
        ui->dspin_ry->setEnabled(true);
        ui->dspin_rz->setEnabled(true);
    }
    else
    {
        disconnect(this, &BoundingBox::eulerAngles, this, &BoundingBox::adjustBox);
        ui->dspin_rx->setEnabled(false);
        ui->dspin_ry->setEnabled(false);
        ui->dspin_rx->setEnabled(false);
    }
}

void BoundingBox::adjustBox(float r, float p, float y) {
    for (auto &cloud: m_cloudtree->getSelectedClouds()) {
        Eigen::Affine3f affine = ct::getTransformation(cloud->center()[0], cloud->center()[1], cloud->center()[2], r, p, y);
        ct::Box box = ct::Features::boundingBoxAdjust(cloud, affine.inverse());
        m_cloudview->addCube(box, QString::fromStdString(cloud->boxId()));
        m_cloudview->setShapeColor(QString::fromStdString(cloud->boxId()), ct::Color::Blue);
        m_cloudview->setShapeRepersentation(QString::fromStdString(cloud->boxId()), m_box_type);
        switch (m_box_type) {
            case BOX_TYPE_POINTS:
                m_cloudview->setShapeSize(QString::fromStdString(cloud->boxId()), 5);
                break;
            case BOX_TYPE_WIREFRAME:
                m_cloudview->setShapeLineWidth(QString::fromStdString(cloud->boxId()), 3);
                break;
            case BOX_TYPE_SURFACE:
                m_cloudview->setShapeOpacity(QString::fromStdString(cloud->boxId()), 0.5);
                break;
        }
        m_box_map[cloud->id()] = box;

        }

    }