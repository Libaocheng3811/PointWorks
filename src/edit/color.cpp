//
// Created by LBC on 2024/11/9.
//

// You may need to build the project (run Qt uic code generator) to get "ui_Color.h" resolved

#include "color.h"
#include "ui_Color.h"

#include <QColorDialog>
#include <QCloseEvent>

// 通过宏定义为常量命名，并且为每个常量赋值，
#define CT_COLOR_POINTCLOUD    (0)
#define CT_COLOR_BACKGROUND    (1)
#define CT_COLOR_NORMALS       (2)
#define CT_COLOR_BOUNDINGBOX   (3)

/**
 * @brief 十六进制颜色代码
 * 十六进制颜色代码由一个井号（#）开头，后面跟着六个十六进制数字。这六个数字分为三对，每对代表一个颜色通道的值：
 * 第一个两位数（ff）代表红色通道的值。ff 是十六进制的最大值，等于十进制的255，表示红色通道的强度最大。
 * 往后依次代表绿色和蓝色通道的值，
 */
// QColor("#ffffff") 表示一个 QColor 对象，它被初始化为白色,
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
    // spacing 参数指定了控件之间的固定空隙，单位是像素
    ui->gridLayout->setSpacing(0);
    for (int row = 0; row < 5; row++)
    {
        for (int column = 0; column < 10; column++)
        {
            QPushButton* btn = new QPushButton();
            // setSizePolicy用于设置控件的尺寸策略
            // QSizePolicy::Ignored: 表示控件的尺寸忽略父布局的影响，它可以根据内容自由改变大小
            btn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            // 设置按钮固定高度为20像素
            btn->setFixedHeight(20);
            if (row == 4 && column == 9)
            {
                btn->setText("+");
                // 设置按钮的样式
                /**
                 * @brief QPushButton{...} 和 QPushButton:pressed{...} 是样式表的两个部分，用于设置不同状态下按钮的样式。
                 * border:none;：没有边框，按钮看起来更加简洁。
                 * border-radius:4px;：按钮的角半径设置为4像素，使得按钮的四个角呈现圆润的效果
                 * background-color:transparent;：背景颜色设置为透明
                 * background-color:lightgray;：当按钮被按下时，背景颜色变为浅灰色，以提供视觉反馈
                 */
                btn->setStyleSheet(tr("QPushButton{border:none;border-radius:4px;background-color:transparent;}"
                                      "QPushButton:pressed{background-color:lightgray;}"));
                // 下述情况中，当使用lambda表达式作为槽时，它本身就是一个函数对象，可以直接执行，不依赖于某个对象
                connect(btn, &QPushButton::clicked, [=]
                        {
                            emit rgb(QColorDialog::getColor(Qt::white, this, tr("select color")));
                        });
            }
            else
            {
                // 设置按钮样式为对应的颜色
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
            // 将指定控件添加到布局中，row 和 column 指定按钮在网格中的行和列的位置
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
}

Color::~Color() {
    delete ui;
}

void Color::apply()
{
    bool was_restore = m_restore_default;
    m_restore_default = false;
    m_applied = true;

    // 背景色不需要选择点云
    if (ui->cbox_type->currentIndex() == CT_COLOR_BACKGROUND)
    {
        if (was_restore)
        {
            m_cloudview->resetBackgroundColor();
            printI("Restore background color done.");
        }
        else
        {
            m_cloudview->setBackgroundColor({static_cast<uint8_t>(m_rgb.red()),
                                             static_cast<uint8_t>(m_rgb.green()),
                                             static_cast<uint8_t>(m_rgb.blue())});
            printI(QString("Set background color[r:%1, g:%2, b:%3] done.")
                    .arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
        }
        return;
    }

    // 存储选中的点云
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("please select clouds!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        switch (ui->cbox_type->currentIndex())
        {
            case CT_COLOR_POINTCLOUD:
                // 恢复默认颜色
                if (was_restore)
                {
                    cloud->restoreColors();
                    printI(QString("Restore cloud[id:%1] default color done.").arg(QString::fromStdString(cloud->id())));
                }
                // 为某个坐标轴设置颜色，例如x轴，就是颜色是根据点云点的x坐标变化的
                else if (m_field != "")
                {
                    cloud->setCloudColor(m_field.toStdString());
                    printI(QString("Apply cloud[id:%1] point color[axis:%2] done.").arg(QString::fromStdString(cloud->id())).arg(m_field));
                }
                // 为所有点云设置统一颜色
                else
                {
                    cloud->setCloudColor(ct::RGB{static_cast<uint8_t>(m_rgb.red()),
                                             static_cast<uint8_t>(m_rgb.green()),
                                             static_cast<uint8_t>(m_rgb.blue())});
                    printI(QString("Apply cloud[id%1] point cloud[r:%2, g:%3, b:%4] done.")
                            .arg(QString::fromStdString(cloud->id())).arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
                }
                m_cloudview->addPointCloud(cloud);
                break;
            case CT_COLOR_NORMALS:
                cloud->setNormalColor({static_cast<uint8_t>(m_rgb.red()),
                                       static_cast<uint8_t>(m_rgb.green()),
                                       static_cast<uint8_t>(m_rgb.blue())});
                printI(QString("Apply cloud[id:%1] normals color[r:%2, g:%3, b:%4] done.")
                        .arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
                break;
            case CT_COLOR_BOUNDINGBOX:
                cloud->setBoxColor({static_cast<uint8_t>(m_rgb.red()),
                                    static_cast<uint8_t>(m_rgb.green()),
                                    static_cast<uint8_t>(m_rgb.blue())});
                printI(QString("Apply cloud [id:%1] box color[r:%2, g:%3, b:%4] done.")
                        .arg(m_rgb.red()).arg(m_rgb.green()).arg(m_rgb.blue()));
                break;
        }
    }
}

void Color::reset()
{
    m_rgb = QColor(255, 255, 255), m_field = "";
    m_restore_default = true;
    m_applied = true; // Reset 本身就是"提交恢复"，关闭时不应撤销
    if (ui->cbox_type->currentIndex() == CT_COLOR_BACKGROUND)
    {
        m_cloudview->resetBackgroundColor();
        printI("Reset background color done.");
        return;
    }
    for (auto& cloud : m_cloudtree->getSelectedClouds())
    {
        switch (ui->cbox_type->currentIndex())
        {
            case CT_COLOR_POINTCLOUD:
                m_cloudview->resetPointCloudColor(cloud);
                break;
            case CT_COLOR_NORMALS:
                if (m_cloudview->contains(QString::fromStdString(cloud->normalId())))
                    m_cloudview->setPointCloudColor(QString::fromStdString(cloud->normalId()), cloud->normalColor());
                break;
            case CT_COLOR_BOUNDINGBOX:
                if (m_cloudview->contains(QString::fromStdString(cloud->boxId())))
                    m_cloudview->setShapeColor(QString::fromStdString(cloud->boxId()), cloud->boxColor());
                break;
        }
        printI(QString("Reset cloud[id:%1] color done.").arg(QString::fromStdString(cloud->id())));
    }
}

void Color::closeEvent(QCloseEvent* event)
{
    // 已 Apply 的更改应持久化，不应被关闭时撤销
    if (!m_applied)
        reset();
    deinit();
    return QDialog::closeEvent(event);
}

void Color::setColorRGB(const QColor &rgb)
{
    m_rgb = rgb, m_field = "";
    m_restore_default = false;
    m_applied = false; // 新预览，尚未提交
    if (ui->cbox_type->currentIndex() == CT_COLOR_BACKGROUND)
    {
        m_cloudview->setBackgroundColor({static_cast<uint8_t>(m_rgb.red()),
                                         static_cast<uint8_t>(m_rgb.green()),
                                         static_cast<uint8_t>(m_rgb.blue())});
        return;
    }

    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    switch (ui->cbox_type->currentIndex())
    {
        case CT_COLOR_POINTCLOUD:
            for (auto& cloud : selected_clouds)
                m_cloudview->setPointCloudColor(cloud, {static_cast<uint8_t>(m_rgb.red()),
                                                        static_cast<uint8_t>(m_rgb.green()),
                                                        static_cast<uint8_t>(m_rgb.blue())});
            break;

        case CT_COLOR_NORMALS:
            for (auto& cloud : selected_clouds)
                if (m_cloudview->contains(QString::fromStdString(cloud->normalId())))
                    m_cloudview->setPointCloudColor(QString::fromStdString(cloud->normalId()), {static_cast<uint8_t>(m_rgb.red()),
                                                                        static_cast<uint8_t>(m_rgb.green()),
                                                                        static_cast<uint8_t>(m_rgb.blue())});
            break;
        case CT_COLOR_BOUNDINGBOX:
            for (auto& cloud : selected_clouds)
                if (m_cloudview->contains(QString::fromStdString(cloud->boxId())))
                    m_cloudview->setShapeColor(QString::fromStdString(cloud->boxId()), {static_cast<uint8_t>(m_rgb.red()),
                                                                static_cast<uint8_t>(m_rgb.green()),
                                                                static_cast<uint8_t>(m_rgb.blue())});
            break;
    }
}

void Color::setColorField(const QString &field)
{
    // 更新变量成员的状态
    m_field = field, m_rgb = QColor(255, 255, 255);
    m_restore_default = false;
    m_applied = false; // 新预览，尚未提交
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    // 设置特定维度的点云颜色
    for (auto& i : selected_clouds)
        m_cloudview->setPointCloudColor(i, field);
}
