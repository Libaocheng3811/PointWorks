//
// Created by LBC on 2024/11/9.
//

// You may need to build the project (run Qt uic code generator) to get "ui_Color.h" resolved

#include "color.h"
#include "ui_Color.h"

#include <QColorDialog>
#include <QCloseEvent>
#include <QShowEvent>

// Target 下拉框索引
#define CT_TARGET_POINTS    (0)
#define CT_TARGET_NORMALS   (1)

const QColor colors[5][10] = {
        {QColor("#ffffff"), QColor("#e5e5e7"), QColor("#cccccc"), QColor("#9a9a9a"),
                QColor("#7f7f7f"), QColor("#666666"), QColor("#4c4c4c"), QColor("#333333"),
                QColor("#191919"), QColor("#000000")},

        {QColor("#ffd6e7"), QColor("#ffccc7"), QColor("#ffe7ba"), QColor("#ffffb8"),
                QColor("#f4ffb8"), QColor("#d9f7be"), QColor("#b5f5ec"), QColor("#bae7ff"),
                QColor("#d6e4ff"), QColor("#efdbff")},

        {QColor("#ff85c0"), QColor("#ff7875"), QColor("#ffc069"), QColor("#fff566"),
                QColor("#d3f261"), QColor("#95de64"), QColor("#5cdbd3"), QColor("#69c0ff"),
                QColor("#85a5ff"), QColor("#b37feb")},

        {QColor("#f759ab"), QColor("#ff4d4f"), QColor("#ffa940"), QColor("#ffdf3d"),
                QColor("#a0d911"), QColor("#52c41a"), QColor("#13c2c2"), QColor("#1890ff"),
                QColor("#2f54eb"), QColor("#722ed1")},

        {QColor("#9e1068"), QColor("#a8171b"), QColor("#ad4e00"), QColor("#ad8b00"),
                QColor("#5b8c00"), QColor("#006075"), QColor("#006d75"), QColor("#0050b3"),
                QColor("#10239e"), QColor("#391085")}
};

Color::Color(QWidget *parent) :
        CustomDialog(parent), ui(new Ui::Color), m_field(""), m_rgb(QColor(255, 255, 255))
{
    ui->setupUi(this);
    ui->gridLayout->setSpacing(0);
    for (int row = 0; row < 5; row++)
    {
        for (int column = 0; column < 10; column++)
        {
            QPushButton* btn = new QPushButton();
            btn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            btn->setFixedHeight(20);
            if (row == 4 && column == 9)
            {
                btn->setText("+");
                btn->setStyleSheet(tr("QPushButton{border:none;border-radius:4px;background-color:transparent;}"
                                      "QPushButton:pressed{background-color:lightgray;}"));
                connect(btn, &QPushButton::clicked, [=]
                        {
                            emit rgb(QColorDialog::getColor(Qt::white, this, tr("select color")));
                        });
            }
            else
            {
                btn->setStyleSheet(tr("QPushButton{border:none;border-radius:4px;background-color:rgb(%1, %2, %3);}"
                                      "QPushButton:pressed{background-color:lightgray;}")
                                      .arg((colors[row][column]).red())
                                      .arg((colors[row][column]).green())
                                      .arg((colors[row][column]).blue()));
                connect(btn, &QPushButton::clicked, [=]
                        {
                            emit rgb(colors[row][column]);
                        });
            }
            ui->gridLayout->addWidget(btn, row, column);
        }
    }
    connect(ui->btn_x, &QPushButton::clicked, [=] {emit field("x"); });
    connect(ui->btn_y, &QPushButton::clicked, [=] {emit field("y"); });
    connect(ui->btn_z, &QPushButton::clicked, [=] {emit field("z");});
    connect(ui->btn_apply, &QPushButton::clicked, this, &Color::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Color::reset);
    connect(this, &Color::rgb, this, &Color::setColorRGB);
    connect(this, &Color::field, this, &Color::setColorField);
    // 切换目标时更新坐标轴按钮
    connect(ui->cbox_target, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int) {
        bool normalsMode = (ui->cbox_target->currentIndex() == CT_TARGET_NORMALS);
        ui->btn_x->setEnabled(!normalsMode && isPointCloudOnly());
        ui->btn_y->setEnabled(!normalsMode && isPointCloudOnly());
        ui->btn_z->setEnabled(!normalsMode && isPointCloudOnly());
    });
}

Color::~Color() {
    delete ui;
}

bool Color::hasPointCloudSelection() const
{
    for (auto& cloud : m_cloudtree->getSelectedClouds()) {
        auto* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        if (item && ct::CustomTree::getNodeType(item) == ct::NodeCloud)
            return true;
    }
    return false;
}

bool Color::hasMeshSelection() const
{
    for (auto& cloud : m_cloudtree->getSelectedClouds()) {
        auto* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        if (item && ct::CustomTree::getNodeType(item) == ct::NodeMesh)
            return true;
    }
    return false;
}

bool Color::isPointCloudOnly() const
{
    return hasPointCloudSelection() && !hasMeshSelection();
}

void Color::updateUIState()
{
    bool pcOnly = isPointCloudOnly();
    bool hasPC = hasPointCloudSelection();

    // Target 下拉框：仅点云时显示（模型只有一个目标，无需选择）
    ui->cbox_target->setVisible(hasPC);
    ui->cbox_target->setCurrentIndex(CT_TARGET_POINTS);

    // 坐标轴按钮：仅点云+Points 模式时启用
    bool normalsMode = (ui->cbox_target->currentIndex() == CT_TARGET_NORMALS);
    ui->btn_x->setEnabled(pcOnly && !normalsMode);
    ui->btn_y->setEnabled(pcOnly && !normalsMode);
    ui->btn_z->setEnabled(pcOnly && !normalsMode);
}

void Color::apply()
{
    bool was_restore = m_restore_default;
    bool targetNormals = ui->cbox_target->isVisible() &&
                         ui->cbox_target->currentIndex() == CT_TARGET_NORMALS;
    m_restore_default = false;
    m_applied = true;

    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("please select clouds!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        ct::SceneNodeType type = ct::CustomTree::getNodeType(item);

        if (type == ct::NodeCloud)
        {
            if (targetNormals)
            {
                ct::ColorRGB normalRGB{static_cast<uint8_t>(m_rgb.red()),
                                       static_cast<uint8_t>(m_rgb.green()),
                                       static_cast<uint8_t>(m_rgb.blue())};
                cloud->setNormalColor(normalRGB);
                QString normalId = QString::fromStdString(cloud->normalId());
                if (m_cloudview->contains(normalId))
                    m_cloudview->setPointCloudColor(normalId, normalRGB);
                printI(QString("Apply cloud[id:%1] normals color[r:%2, g:%3, b:%4] done.")
                        .arg(QString::fromStdString(cloud->id())).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
            }
            else
            {
                if (was_restore)
                {
                    cloud->restoreColors();
                    printI(QString("Restore cloud[id:%1] default color done.").arg(QString::fromStdString(cloud->id())));
                }
                else if (m_field != "")
                {
                    cloud->setCloudColor(m_field.toStdString());
                    printI(QString("Apply cloud[id:%1] point color[axis:%2] done.").arg(QString::fromStdString(cloud->id())).arg(m_field));
                }
                else
                {
                    cloud->setCloudColor(ct::ColorRGB{static_cast<uint8_t>(m_rgb.red()),
                                                     static_cast<uint8_t>(m_rgb.green()),
                                                     static_cast<uint8_t>(m_rgb.blue())});
                    printI(QString("Apply cloud[id%1] point cloud[r:%2, g:%3, b:%4] done.")
                            .arg(QString::fromStdString(cloud->id())).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
                }
                m_cloudview->addPointCloud(cloud);
            }
        }
        else if (type == ct::NodeMesh)
        {
            QString cloudId = QString::fromStdString(cloud->id());
            if (was_restore)
            {
                m_cloudview->setTextureMeshColor(cloudId, 0.8, 0.8, 0.8);
                printI(QString("Restore mesh[id:%1] default color done.").arg(cloudId));
            }
            else
            {
                m_cloudview->setTextureMeshColor(cloudId,
                    m_rgb.red() / 255.0f,
                    m_rgb.green() / 255.0f,
                    m_rgb.blue() / 255.0f);
                printI(QString("Apply mesh[id:%1] color[r:%2, g:%3, b:%4] done.")
                        .arg(cloudId).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
            }
            m_cloudview->refresh();
        }
    }
}

void Color::reset()
{
    m_rgb = QColor(255, 255, 255), m_field = "";
    m_restore_default = true;
    m_applied = true;

    bool targetNormals = ui->cbox_target->isVisible() &&
                         ui->cbox_target->currentIndex() == CT_TARGET_NORMALS;

    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    for (auto& cloud : selected_clouds)
    {
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        ct::SceneNodeType type = ct::CustomTree::getNodeType(item);

        if (type == ct::NodeCloud)
        {
            if (targetNormals)
            {
                ct::ColorRGB defaultNormal{0, 255, 0};
                cloud->setNormalColor(defaultNormal);
                QString normalId = QString::fromStdString(cloud->normalId());
                if (m_cloudview->contains(normalId))
                    m_cloudview->setPointCloudColor(normalId, defaultNormal);
            }
            else
            {
                m_cloudview->resetPointCloudColor(cloud);
            }
        }
        else if (type == ct::NodeMesh)
        {
            m_cloudview->setTextureMeshColor(QString::fromStdString(cloud->id()), 0.8, 0.8, 0.8);
            m_cloudview->refresh();
        }
        printI(QString("Reset cloud[id:%1] color done.").arg(QString::fromStdString(cloud->id())));
    }
}

void Color::closeEvent(QCloseEvent* event)
{
    if (!m_applied)
        reset();
    deinit();
    return QDialog::closeEvent(event);
}

void Color::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    updateUIState();
}

void Color::setColorRGB(const QColor &rgb)
{
    m_rgb = rgb, m_field = "";
    m_restore_default = false;
    m_applied = false;

    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }

    ct::ColorRGB colorRGB{static_cast<uint8_t>(rgb.red()),
                          static_cast<uint8_t>(rgb.green()),
                          static_cast<uint8_t>(rgb.blue())};

    bool targetNormals = ui->cbox_target->isVisible() &&
                         ui->cbox_target->currentIndex() == CT_TARGET_NORMALS;

    for (auto& cloud : selected_clouds)
    {
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        ct::SceneNodeType type = ct::CustomTree::getNodeType(item);

        if (type == ct::NodeCloud)
        {
            if (targetNormals)
            {
                cloud->setNormalColor(colorRGB);
                QString normalId = QString::fromStdString(cloud->normalId());
                if (m_cloudview->contains(normalId))
                    m_cloudview->setPointCloudColor(normalId, colorRGB);
            }
            else
            {
                m_cloudview->setPointCloudColor(cloud, colorRGB);
            }
        }
        else if (type == ct::NodeMesh)
        {
            m_cloudview->setTextureMeshColor(QString::fromStdString(cloud->id()),
                rgb.red() / 255.0f,
                rgb.green() / 255.0f,
                rgb.blue() / 255.0f);
            m_cloudview->refresh();
        }
    }
}

void Color::setColorField(const QString &field)
{
    m_field = field, m_rgb = QColor(255, 255, 255);
    m_restore_default = false;
    m_applied = false;

    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    // 坐标轴着色仅对点云+Points 模式有效
    for (auto& cloud : selected_clouds)
    {
        QTreeWidgetItem* item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        ct::SceneNodeType type = ct::CustomTree::getNodeType(item);

        if (type == ct::NodeCloud)
        {
            m_cloudview->setPointCloudColor(cloud, field);
        }
    }
}
