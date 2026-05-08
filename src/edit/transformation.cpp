#include "transformation.h"
#include "ui_transformation.h"

#include <pcl/common/transforms.h>
#include "core/cloudtype.h"

#define TRANS_TYPE_MATRIX       (0)
#define TRANS_TYPE_EULERANGLE   (1)
#define TRANS_TYPE_ANGLEAXIS    (2)

Transformation::Transformation(QWidget* parent)
    : CustomDialog(parent), ui(new Ui::Transformation),
    m_affine(Eigen::Affine3f::Identity())
{
    ui->setupUi(this);
    connect(ui->btn_add, &QPushButton::clicked, this, &Transformation::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Transformation::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Transformation::reset);
    connect(ui->btn_preview, &QPushButton::clicked, [=] { preview(m_affine); });
    ui->tabWidget->setCurrentIndex(0);

    // matrix
    connect(ui->txt_matrix, &QTextEdit::textChanged, [=]
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_MATRIX) return;
                if (!parseMatrixText(ui->txt_matrix->toPlainText(), m_affine))
                {
                    printW("The transformation matrix format is wrong!");
                    return;
                }
                syncUI(m_affine);
                preview(m_affine);
            });

    // eulerAngle
    connect(ui->dspin_rx1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE) return;
                m_affine = pw::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_ry1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE) return;
                m_affine = pw::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_rz1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE) return;
                m_affine = pw::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_tx1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE) return;
                m_affine = pw::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_ty1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE) return;
                m_affine = pw::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_tz1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE) return;
                m_affine = pw::getTransformation(ui->dspin_tx1->value(), ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                                                 ui->dspin_rx1->value(), ui->dspin_ry1->value(), ui->dspin_rz1->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->txt_xyzeuler, &QLineEdit::textChanged, [=](const QString& text)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_EULERANGLE) return;
                if (!parseEulerText(text, m_affine))
                {
                    printW("The transformation xyzeuler format is wrong!");
                    return;
                }
                syncUI(m_affine);
                preview(m_affine);
            });

    // axisAngle
    connect(ui->dspin_angle, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = pw::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_ax, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = pw::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_ay, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = pw::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_az, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = pw::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_tx2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = pw::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_ty2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = pw::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                syncUI(m_affine);
                preview(m_affine);
            });
    connect(ui->dspin_tz2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->tabWidget->currentIndex() != TRANS_TYPE_ANGLEAXIS) return;
                m_affine = pw::getTransformation(ui->dspin_angle->value(), ui->dspin_ax->value(), ui->dspin_ay->value(), ui->dspin_az->value(),
                                                 ui->dspin_tx2->value(), ui->dspin_ty2->value(), ui->dspin_tz2->value());
                syncUI(m_affine);
                preview(m_affine);
            });
}

Transformation::~Transformation() { delete ui; }

void Transformation::syncUI(const Eigen::Affine3f& affine3f)
{
    float x, y, z, rx, ry, rz;
    pw::getTranslationAndEulerAngles(affine3f, x, y, z, rx, ry, rz);
    float alpha, axisX, axisY, axisZ;
    pw::getAngleAxis(affine3f, alpha, axisX, axisY, axisZ);
    int index = ui->tabWidget->currentIndex();
    if (index != TRANS_TYPE_MATRIX)
        ui->txt_matrix->setText(QString::fromStdString(pw::getTransformationString(affine3f.matrix(), 3)));
    if (index != TRANS_TYPE_EULERANGLE)
    {
        ui->dspin_rx1->setValue(rx);
        ui->dspin_ry1->setValue(ry);
        ui->dspin_rz1->setValue(rz);
        ui->dspin_tx1->setValue(x);
        ui->dspin_ty1->setValue(y);
        ui->dspin_tz1->setValue(z);
    }
    if (index != TRANS_TYPE_ANGLEAXIS)
    {
        ui->dspin_angle->setValue(alpha);
        ui->dspin_ax->setValue(axisX);
        ui->dspin_ay->setValue(axisY);
        ui->dspin_az->setValue(axisZ);
        ui->dspin_tx2->setValue(x);
        ui->dspin_ty2->setValue(y);
        ui->dspin_tz2->setValue(z);
    }
}

bool Transformation::parseMatrixText(const QString& text, Eigen::Affine3f& t)
{
    QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    if (lines.size() != 4) return false;

    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i)
    {
        QStringList vals = lines[i].split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (vals.size() != 4) return false;
        for (int j = 0; j < 4; ++j)
        {
            bool ok;
            mat(i, j) = vals[j].toFloat(&ok);
            if (!ok) return false;
        }
    }
    t = Eigen::Affine3f(mat);
    return true;
}

bool Transformation::parseEulerText(const QString& text, Eigen::Affine3f& t)
{
    QStringList vals = text.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    if (vals.size() != 6) return false;

    bool ok;
    float x = vals[0].toFloat(&ok); if (!ok) return false;
    float y = vals[1].toFloat(&ok); if (!ok) return false;
    float z = vals[2].toFloat(&ok); if (!ok) return false;
    float rx = vals[3].toFloat(&ok); if (!ok) return false;
    float ry = vals[4].toFloat(&ok); if (!ok) return false;
    float rz = vals[5].toFloat(&ok); if (!ok) return false;

    t = pw::getTransformation(x, y, z, rx, ry, rz);
    return true;
}

void Transformation::add()
{
    std::vector<pw::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_affine.matrix() == Eigen::Matrix4f::Identity()) return;
    for (auto& cloud : selected_clouds)
    {
        auto it = m_trans_map.find(cloud->id());
        if (it == m_trans_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched transformation!")
                       .arg(QString::fromStdString(cloud->id())));
            continue;
        }
        QString sid = QString::fromStdString(cloud->id()) + TRANS_PRE_FLAG;
        m_cloudview->removePointCloud(sid);
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::PointCloud<pw::PointXYZRGBN>::Ptr pcl_transformed(new pcl::PointCloud<pw::PointXYZRGBN>);
        pcl::transformPointCloud(*pcl_cloud, *pcl_transformed, it->second);
        pw::Cloud::Ptr new_cloud = pw::Cloud::fromPCL_XYZRGBN(*pcl_transformed, cloud->getGlobalShift());
        new_cloud->setId(TRANS_ADD_FLAG + cloud->id());
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        m_cloudtree->insertCloud(new_cloud, item, true, pw::MountStrategy::Sibling);
        m_trans_map.erase(cloud->id());
        printI(QString("Add transformed cloud[id:%1] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Transformation::apply()
{
    std::vector<pw::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_affine.matrix() == Eigen::Matrix4f::Identity()) return;
    for (auto& cloud : selected_clouds)
    {
        auto it = m_trans_map.find(cloud->id());
        if (it == m_trans_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched transformation!")
                       .arg(QString::fromStdString(cloud->id())));
            continue;
        }
        QString sid = QString::fromStdString(cloud->id()) + TRANS_PRE_FLAG;
        m_cloudview->removePointCloud(sid);
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, it->second);
        auto transformed = pw::Cloud::fromPCL_XYZRGBN(*pcl_cloud, cloud->getGlobalShift());
        transformed->setId(cloud->id());
        m_cloudtree->updateCloud(cloud, transformed);
        m_trans_map.erase(cloud->id());
        printI(QString("Apply transformed cloud[id:%1] done.").arg(QString::fromStdString(cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Transformation::reset()
{
    for (auto& [sid, affine] : m_trans_map)
        m_cloudview->removePointCloud(QString::fromStdString(sid) + TRANS_PRE_FLAG);
    m_trans_map.clear();
    m_cloudview->clearInfo();
}

void Transformation::preview(const Eigen::Affine3f& affine3f)
{
    m_affine = affine3f;
    if (!ui->check_refresh->isChecked()) return;
    std::vector<pw::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_affine.matrix() == Eigen::Matrix4f::Identity()) return;
    m_cloudview->showInfo("Transform Pointcloud", 1);
    for (auto& cloud : selected_clouds)
    {
        Eigen::Affine3f trans = ui->cbox_inverse->isChecked() ? m_affine.inverse() : m_affine;

        // 清除旧预览
        QString tid = QString::fromStdString(cloud->id()) + TRANS_PRE_FLAG;
        m_cloudview->removePointCloud(tid);

        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        pcl::PointCloud<pw::PointXYZRGBN>::Ptr pcl_transformed(new pcl::PointCloud<pw::PointXYZRGBN>);
        pcl::transformPointCloud(*pcl_cloud, *pcl_transformed, trans);
        pw::Cloud::Ptr trans_cloud = pw::Cloud::fromPCL_XYZRGBN(*pcl_transformed, cloud->getGlobalShift());
        trans_cloud->setId(tid.toStdString());

        m_cloudview->addPointCloud(trans_cloud);
        m_cloudview->setPointCloudSize(tid, 3);
        m_trans_map[cloud->id()] = trans;
    }
}
