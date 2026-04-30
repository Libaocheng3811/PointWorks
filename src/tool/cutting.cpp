//
// Created by LBC on 2024/11/26.
//

#include "cutting.h"
#include "base/cloudtree.h"
#include "ui_Cutting.h"
#include "core/cloud.h"

#include <QSet>

#define CUT_TYPE_RECTANGULAR    0
#define CUT_TYPE_POLYGONAL      1

#define POLYGONAL_ID        "polygonal"

#define CUTTING_PRE_FLAG    "-cutting"

Cutting::Cutting(QWidget *parent) :CustomDialog(parent),
ui(new Ui::Cutting), is_picking(false), pick_start(false)
{
    ui->setupUi(this);

    connect(ui->btn_confirm, &QPushButton::clicked, this, &Cutting::confirm);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Cutting::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Cutting::close);
    connect(ui->btn_selectin, &QPushButton::clicked, [=] {this->previewCutting(false); });
    connect(ui->btn_selectout, &QPushButton::clicked, [=] {this->previewCutting(true); });
    ui->cbox_type->setCurrentIndex(0);
    updateButtonStates();
}

Cutting::~Cutting()
{
    delete ui;
}

void Cutting::init()
{
    connect(ui->cbox_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Cutting::updateInfo);
    this->updateInfo(ui->cbox_type->currentIndex());

    // 打开弹窗即进入裁剪模式
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    is_picking = true;
    m_cloudtree->setEnabled(false);
    m_cloudview->setInteractorEnable(false);
    m_cloudview->setCursor(Qt::CrossCursor);

    for (auto& cloud : selected_clouds)
        m_cloudview->removeShape(QString::fromStdString(cloud->boxId()));

    for (auto& cut_cloud : m_cutting_map)
    {
        m_cloudview->removePointCloud(QString::fromStdString(cut_cloud.second->id()));
        m_cloudview->removeShape(QString::fromStdString(cut_cloud.second->boxId()));
    }
    m_cloudview->removeShape(POLYGONAL_ID);

    connect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &Cutting::mouseLeftPressed);
    connect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &Cutting::mouseLeftReleased);
    connect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Cutting::mouseRightReleased);
    connect(m_cloudview, &ct::CloudView::mouseMoved, this, &Cutting::mouseMoved);
    this->updateInfo(ui->cbox_type->currentIndex());
    updateButtonStates();
}

void Cutting::deinit()
{
    stopPicking();
    m_cloudview->setInteractorEnable(true);
    m_cloudview->clearInfo();
    // 清理预览点云并恢复原点云
    for (auto& cut_cloud : m_cutting_map)
    {
        m_cloudview->removePointCloud(QString::fromStdString(cut_cloud.second->id()));
        m_cloudview->removeShape(QString::fromStdString(cut_cloud.second->boxId()));
    }
    m_cutting_map.clear();
    restoreOriginals();
}

void Cutting::restoreOriginals()
{
    for (const auto& id : m_hidden_originals)
        m_cloudview->setPointCloudVisibility(id, true);
    m_hidden_originals.clear();
}

void Cutting::stopPicking()
{
    if (!is_picking) return;
    is_picking = false;

    m_cloudtree->setEnabled(true);
    disconnect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &Cutting::mouseLeftPressed);
    disconnect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &Cutting::mouseLeftReleased);
    disconnect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &Cutting::mouseRightReleased);
    disconnect(m_cloudview, &ct::CloudView::mouseMoved, this, &Cutting::mouseMoved);
    m_cloudview->unsetCursor();

    m_cloudview->removeShape(POLYGONAL_ID);
    this->updateInfo(ui->cbox_type->currentIndex());
}

void Cutting::confirm()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        auto it = m_cutting_map.find(cloud->id());
        if (it == m_cutting_map.end())
        {
            printW(QString("The Cloud[id:%1] has no preview cloud!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        ct::Cloud::Ptr preview_cloud = it->second;

        // 一次遍历同时得到区域内外两个点云
        auto [seg_cloud, remain_cloud] = m_cloudview->areaPickSplit(m_pick_points, cloud, m_last_select_in);

        // 移除预览点云
        m_cloudview->removePointCloud(QString::fromStdString(preview_cloud->id()));

        // 裁剪结果作为兄弟节点
        seg_cloud->setId(cloud->id() + "-segmented");
        QString orig_id = QString::fromStdString(cloud->id());
        QTreeWidgetItem* item = m_cloudtree->getItemById(orig_id);
        m_cloudtree->insertCloud(seg_cloud, item, true, ct::MountStrategy::Sibling, ct::SceneNodeType::NodeCloud, false);

        // 用剩余部分替换原点云（swap 原地替换，释放原数据）
        if (remain_cloud && !remain_cloud->empty())
        {
            m_cloudview->removePointCloud(orig_id);
            // 不设 id，让 swap 后 cloud 保留原 id
            m_cloudtree->updateCloud(cloud, remain_cloud, false);
            m_cloudtree->setCloudChecked(cloud);
            // 重命名：原 id → 原 id-remaining
            m_cloudtree->renameCloudById(orig_id, orig_id + "-remaining");
            cloud->setId((orig_id + "-remaining").toStdString());
        }

        m_cutting_map.erase(it);
        printI(QString("Cut cloud[id:%1] done.").arg(QString::fromStdString(seg_cloud->id())));
    }
    restoreOriginals();
    m_pick_points.clear();
    m_cloudview->removeShape(POLYGONAL_ID);
    updateButtonStates();
}

void Cutting::reset()
{
    pick_start = false;
    m_cloudview->removeShape(POLYGONAL_ID);
    for (auto& cut_cloud : m_cutting_map)
    {
        m_cloudview->removePointCloud(QString::fromStdString(cut_cloud.second->id()));
        m_cloudview->removeShape(QString::fromStdString(cut_cloud.second->boxId()));
    }
    m_cutting_map.clear();
    m_pick_points.clear();
    restoreOriginals();
    updateButtonStates();
}

void Cutting::updateInfo(int index)
{
    QString typeStr = (index == CUT_TYPE_RECTANGULAR) ? "rectangular selection" : "polygonal selection";
    QString status = is_picking ? "[ON]" : "[OFF]";
    m_cloudview->showInfo(QString("Cutting %1 (%2)").arg(status).arg(typeStr), 1);

    if (index == CUT_TYPE_RECTANGULAR)
        m_cloudview->showInfo("Left/Right click : set opposite corners", 2);
    else
        m_cloudview->showInfo("Left click : add contour points / Right click : close", 2);
}

void Cutting::updateButtonStates()
{
    bool areaDrawn = !pick_start && m_pick_points.size() >= 3;
    bool previewing = !m_cutting_map.empty();

    ui->btn_selectin->setEnabled(areaDrawn);
    ui->btn_selectout->setEnabled(areaDrawn);
    ui->btn_reset->setEnabled(areaDrawn || previewing);
    ui->btn_confirm->setEnabled(previewing);
}

void Cutting::previewCutting(bool select_in)
{
    m_last_select_in = select_in;

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

    // 清除上一次预览并恢复原点云
    for (auto& cut_cloud : m_cutting_map)
    {
        m_cloudview->removePointCloud(QString::fromStdString(cut_cloud.second->id()));
        m_cloudview->removeShape(QString::fromStdString(cut_cloud.second->boxId()));
    }
    m_cutting_map.clear();
    restoreOriginals();

    m_cloudview->removeShape(POLYGONAL_ID);

    for (auto& cloud : selected_clouds)
    {
        ct::Cloud::Ptr cut_cloud = m_cloudview->areaPick(m_pick_points, cloud, select_in);

        if (!cut_cloud || cut_cloud->empty()) {
            printW("No points selected.");
            continue;
        }

        cut_cloud->setId(cloud->id() + CUTTING_PRE_FLAG);

        // 隐藏原点云
        QString orig_id = QString::fromStdString(cloud->id());
        m_cloudview->setPointCloudVisibility(orig_id, false);
        m_hidden_originals.insert(orig_id);

        // 显示裁剪结果（保持原色、原大小）
        m_cloudview->addPointCloud(cut_cloud);
        m_cloudview->setPointCloudSize(QString::fromStdString(cut_cloud->id()), cloud->pointSize());

        m_cutting_map[cloud->id()] = cut_cloud;
    }
    updateButtonStates();
}

void Cutting::mouseLeftPressed(const ct::PointXY &pt)
{
    if (!pick_start)
    {
        m_pick_points.clear();
        m_pick_points.push_back(pt);
        pick_start = true;
    }
}

void Cutting::mouseLeftReleased(const ct::PointXY &pt)
{
    if (pick_start)
    {
        if ((m_pick_points.front().x == pt.x) && (m_pick_points.front().y == pt.y))
            return;

        if (ui->cbox_type->currentIndex() == CUT_TYPE_RECTANGULAR)
        {
            ct::PointXY p1(m_pick_points.front().x, pt.y);
            ct::PointXY p2(pt.x, m_pick_points.front().y);
            m_pick_points.push_back(p1);
            m_pick_points.push_back(pt);
            m_pick_points.push_back(p2);
            m_cloudview->addPolygon2D(m_pick_points, POLYGONAL_ID, ct::Color::Green);
            pick_start = false;
            updateButtonStates();
        }
        else
        {
            m_pick_points.push_back(pt);
        }
    }
}

void Cutting::mouseRightReleased(const ct::PointXY &pt)
{
    if (pick_start)
    {
        if (ui->cbox_type->currentIndex() == CUT_TYPE_RECTANGULAR)
        {
            ct::PointXY p1(m_pick_points.front().x, pt.y);
            ct::PointXY p2(pt.x, m_pick_points.front().y);
            m_pick_points.push_back(p1);
            m_pick_points.push_back(pt);
            m_pick_points.push_back(p2);
            pick_start = false;
            updateButtonStates();
        }
        else
        {
            if (m_pick_points.size() == 2) m_pick_points.push_back(pt);
            m_cloudview->addPolygon2D(m_pick_points, POLYGONAL_ID, ct::Color::Green);
            pick_start = false;
            updateButtonStates();
        }
    }
}

void Cutting::mouseMoved(const ct::PointXY &pt)
{
    if (pick_start)
    {
        if (ui->cbox_type->currentIndex() == CUT_TYPE_RECTANGULAR)
        {
            std::vector<ct::PointXY> pre_points = m_pick_points;
            ct::PointXY p1(pre_points.front().x, pt.y);
            ct::PointXY p2(pt.x, pre_points.front().y);
            pre_points.push_back(p1);
            pre_points.push_back(pt);
            pre_points.push_back(p2);
            m_cloudview->addPolygon2D(pre_points, POLYGONAL_ID, ct::Color::Green);
        }
        else
        {
            std::vector<ct::PointXY> pre_points = m_pick_points;
            pre_points.push_back(pt);
            m_cloudview->addPolygon2D(pre_points, POLYGONAL_ID, ct::Color::Green);
        }
    }
}
