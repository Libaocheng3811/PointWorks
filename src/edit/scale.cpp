#include "scale.h"
#include "ui_scale.h"

#include <pcl/common/transforms.h>
#include "core/cloudtype.h"

Scale::Scale(QWidget* parent)
    : CustomDialog(parent), ui(new Ui::Scale)
{
    ui->setupUi(this);

    connect(ui->btn_add, &QPushButton::clicked, this, &Scale::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Scale::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Scale::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Scale::close);

    // Same value: lock y/z
    connect(ui->check_samevalue, &QCheckBox::stateChanged, [=](int state) {
        ui->dspin_y->setEnabled(!state);
        ui->dspin_z->setEnabled(!state);
    });

    // X axis
    connect(ui->dspin_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double value) {
                if (ui->check_samevalue->isChecked()) {
                    ui->dspin_y->setValue(value);
                    ui->dspin_z->setValue(value);
                    emit scaleChanged(value, value, value);
                } else {
                    emit scaleChanged(value, ui->dspin_y->value(), ui->dspin_z->value());
                }
            });

    // Y axis
    connect(ui->dspin_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double value) {
                if (ui->check_samevalue->isChecked()) return;
                emit scaleChanged(ui->dspin_x->value(), value, ui->dspin_z->value());
            });

    // Z axis
    connect(ui->dspin_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double value) {
                if (ui->check_samevalue->isChecked()) return;
                emit scaleChanged(ui->dspin_x->value(), ui->dspin_y->value(), value);
            });

    ui->check_samevalue->setChecked(true);
    connect(this, &Scale::scaleChanged, this, &Scale::preview);
}

Scale::~Scale() { delete ui; }

void Scale::preview(double x, double y, double z)
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) {
        printW("Please select a cloud!");
        return;
    }

    m_cloudview->showInfo("Scale Pointcloud", 1);

    bool useCenter = (ui->cbox_type->currentIndex() == 0);

    for (auto& cloud : selected_clouds) {
        // Remove old preview
        std::string sid = cloud->id() + SCALE_PRE_FLAG;
        m_cloudview->removePointCloud(QString::fromStdString(sid));

        // Build scale matrix
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        trans(0, 0) = static_cast<float>(x);
        trans(1, 1) = static_cast<float>(y);
        trans(2, 2) = static_cast<float>(z);

        if (useCenter) {
            // Center mode: translate to origin -> scale -> translate back
            Eigen::Vector3f c = cloud->center().cast<float>();
            Eigen::Affine3f toOrigin = Eigen::Affine3f::Identity();
            toOrigin.translation() = -c;
            Eigen::Affine3f fromOrigin = Eigen::Affine3f::Identity();
            fromOrigin.translation() = c;
            trans = fromOrigin * trans * toOrigin;
        }

        // PCL transform
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::PointCloud<ct::PointXYZRGBN>::Ptr pcl_scaled(new pcl::PointCloud<ct::PointXYZRGBN>);
        pcl::transformPointCloud(*pcl_cloud, *pcl_scaled, trans);

        ct::Cloud::Ptr scaled_cloud = ct::Cloud::fromPCL_XYZRGBN(*pcl_scaled, cloud->getGlobalShift());
        scaled_cloud->setId(sid);

        if (ui->check_keepentity->isChecked())
            m_cloudview->resetCamera();

        m_cloudview->addPointCloud(scaled_cloud);
        m_scale_map[cloud->id()] = scaled_cloud;
    }
}

void Scale::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds) {
        auto it = m_scale_map.find(cloud->id());
        if (it == m_scale_map.end()) {
            printW(QString("The cloud[id:%1] has no scaled preview!")
                       .arg(QString::fromStdString(cloud->id())));
            continue;
        }
        std::string sid = cloud->id() + SCALE_PRE_FLAG;
        m_cloudview->removePointCloud(QString::fromStdString(sid));

        ct::Cloud::Ptr new_cloud = it->second;
        new_cloud->setId(SCALE_ADD_FLAG + cloud->id());
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        m_cloudtree->insertCloud(new_cloud, item, true, ct::MountStrategy::Sibling);
        m_scale_map.erase(cloud->id());
        printI(QString("Add scaled cloud[id:%1] done.")
                   .arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Scale::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds) {
        auto it = m_scale_map.find(cloud->id());
        if (it == m_scale_map.end()) {
            printW(QString("The cloud[id:%1] has no scaled preview!")
                       .arg(QString::fromStdString(cloud->id())));
            continue;
        }
        std::string sid = cloud->id() + SCALE_PRE_FLAG;
        m_cloudview->removePointCloud(QString::fromStdString(sid));

        ct::Cloud::Ptr new_cloud = it->second;
        new_cloud->setId(cloud->id());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_scale_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply scaled cloud[id:%1] done.")
                   .arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Scale::reset()
{
    for (auto& [sid, cloud] : m_scale_map)
        m_cloudview->removePointCloud(QString::fromStdString(sid + SCALE_PRE_FLAG));

    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);

    m_scale_map.clear();
    m_cloudview->clearInfo();
}
