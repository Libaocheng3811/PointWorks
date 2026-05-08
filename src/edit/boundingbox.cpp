//
// Created by LBC on 2024/11/13.
//

// You may need to build the project (run Qt uic code generator) to get "ui_BoundingBox.h" resolved

#include "boundingbox.h"

#include "core/common.h"
#include "algorithm/features.h"

#include "ui_BoundingBox.h"

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>

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
    std::vector<pw::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    this->adjustEnable(false);

    for (auto& cloud : selected_clouds)
    {
        bool aabb = ui->rbtn_aabb->isChecked();
        int box_type = m_box_type;

        auto future = QtConcurrent::run([cloud, aabb]() -> pw::Box {
            return aabb ? pw::Features::boundingBoxAABB(cloud)
                        : pw::Features::boundingBoxOBB(cloud);
        });

        auto* watcher = new QFutureWatcher<pw::Box>(this);
        QString cloud_id = QString::fromStdString(cloud->id());
        connect(watcher, &QFutureWatcher<pw::Box>::finished, this, [=]() {
            pw::Box box = watcher->result();
            m_cloudview->addCube(box, QString::fromStdString(cloud->id()) + "_preview");
            m_cloudview->setShapeColor(cloud_id + "_preview",
                                       aabb ? pw::Color::Red : pw::Color::Green);
            m_cloudview->setShapeRepersentation(cloud_id + "_preview", box_type);

            switch (box_type)
            {
                case BOX_TYPE_POINTS:
                    m_cloudview->setShapeSize(cloud_id + "_preview", 5);
                    break;
                case BOX_TYPE_WIREFRAME:
                    m_cloudview->setShapeLineWidth(cloud_id + "_preview", 3);
                    break;
                case BOX_TYPE_SURFACE:
                    m_cloudview->setShapeOpacity(cloud_id + "_preview", 0.5);
                    break;
            }

            float roll, pitch, yaw;
            pw::getEulerAngles(box.pose, roll, pitch, yaw);
            ui->dspin_rx->setValue(roll);
            ui->dspin_ry->setValue(pitch);
            ui->dspin_rz->setValue(yaw);
            m_box_map[cloud->id()] = box;
            this->adjustEnable(true);
            if (aabb)
                m_cloudview->showInfo("Axis-Aligned Bounding Box", 1);
            else
                m_cloudview->showInfo("Oriented Bounding Box", 1);
            watcher->deleteLater();
        });
        watcher->setFuture(future);
    }
}

void BoundingBox::apply()
{
    std::vector<pw::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
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
        Eigen::Affine3f affine = pw::getTransformation(cloud->center()[0], cloud->center()[1], cloud->center()[2], r, p, y);
        int box_type = m_box_type;

        auto future = QtConcurrent::run([cloud, affine]() -> pw::Box {
            return pw::Features::boundingBoxAdjust(cloud, affine.inverse());
        });

        auto* watcher = new QFutureWatcher<pw::Box>(this);
        connect(watcher, &QFutureWatcher<pw::Box>::finished, this, [=]() {
            pw::Box box = watcher->result();
            QString cloud_id = QString::fromStdString(cloud->id());
            m_cloudview->addCube(box, cloud_id + "_preview");
            m_cloudview->setShapeColor(cloud_id + "_preview", pw::Color::Blue);
            m_cloudview->setShapeRepersentation(cloud_id + "_preview", box_type);
            switch (box_type) {
                case BOX_TYPE_POINTS:
                    m_cloudview->setShapeSize(cloud_id + "_preview", 5);
                    break;
                case BOX_TYPE_WIREFRAME:
                    m_cloudview->setShapeLineWidth(cloud_id + "_preview", 3);
                    break;
                case BOX_TYPE_SURFACE:
                    m_cloudview->setShapeOpacity(cloud_id + "_preview", 0.5);
                    break;
            }
            m_box_map[cloud->id()] = box;
            watcher->deleteLater();
        });
        watcher->setFuture(future);
    }
}