#include "displaysettings_colors_page.h"
#include "viz/cloudview.h"
#include "base/cloudtree.h"

const QColor ColorsPage::s_palette[5][10] = {
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

ColorsPage::ColorsPage(ct::CloudView* cloudview, ct::CloudTree* cloudtree, QWidget* parent)
    : DisplaySettingsPage(parent),
      m_cloudview(cloudview),
      m_cloudtree(cloudtree),
      m_bg_color(255, 255, 255),
      m_box_color(0, 255, 0)
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(10, 10, 10, 10);
    main_layout->setSpacing(12);

    // ====== 背景色 ======
    auto* bg_group = new QGroupBox("Background Color", this);
    auto* bg_layout = new QFormLayout(bg_group);
    bg_layout->setContentsMargins(8, 12, 8, 8);
    bg_layout->setSpacing(6);
    createColorRow(bg_layout, "Color:", m_bg_preview, m_bg_custom, m_bg_reset);
    main_layout->addWidget(bg_group);

    // ====== 包围盒色 ======
    auto* box_group = new QGroupBox("Bounding Box Color", this);
    auto* box_layout = new QFormLayout(box_group);
    box_layout->setContentsMargins(8, 12, 8, 8);
    box_layout->setSpacing(6);
    createColorRow(box_layout, "Color:", m_box_preview, m_box_custom, m_box_reset);
    main_layout->addWidget(box_group);

    main_layout->addStretch();

    // ====== 信号连接 ======
    // 背景色
    connect(m_bg_custom, &QPushButton::clicked, this, [=]() {
        QColor c = QColorDialog::getColor(m_bg_color, this, "Select Background Color");
        if (c.isValid()) {
            m_bg_color = c;
            m_bg_preview->setStyleSheet(
                QString("QPushButton{background-color:rgb(%1,%2,%3);border:1px solid #999;border-radius:3px;min-width:40px;min-height:22px;}")
                    .arg(c.red()).arg(c.green()).arg(c.blue()));
        }
    });
    connect(m_bg_reset, &QPushButton::clicked, this, [=]() {
        m_bg_color = QColor(255, 255, 255);
        m_bg_preview->setStyleSheet(
            "QPushButton{background-color:rgb(255,255,255);border:1px solid #999;border-radius:3px;min-width:40px;min-height:22px;}");
        m_cloudview->resetBackgroundColor();
    });

    // 包围盒色
    connect(m_box_custom, &QPushButton::clicked, this, [=]() {
        QColor c = QColorDialog::getColor(m_box_color, this, "Select Bounding Box Color");
        if (c.isValid()) {
            m_box_color = c;
            m_box_preview->setStyleSheet(
                QString("QPushButton{background-color:rgb(%1,%2,%3);border:1px solid #999;border-radius:3px;min-width:40px;min-height:22px;}")
                    .arg(c.red()).arg(c.green()).arg(c.blue()));
        }
    });
    connect(m_box_reset, &QPushButton::clicked, this, [=]() {
        m_box_color = QColor(0, 255, 0);
        m_box_preview->setStyleSheet(
            "QPushButton{background-color:rgb(0,255,0);border:1px solid #999;border-radius:3px;min-width:40px;min-height:22px;}");
    });

    // 初始化预览按钮样式
    m_bg_preview->setStyleSheet(
        "QPushButton{background-color:rgb(255,255,255);border:1px solid #999;border-radius:3px;min-width:40px;min-height:22px;}");
    m_box_preview->setStyleSheet(
        "QPushButton{background-color:rgb(0,255,0);border:1px solid #999;border-radius:3px;min-width:40px;min-height:22px;}");
}

void ColorsPage::createColorRow(QFormLayout* layout, const QString& label,
                                 QPushButton*& preview, QPushButton*& custom, QPushButton*& resetBtn)
{
    auto* row_layout = new QHBoxLayout();
    row_layout->setSpacing(6);

    preview = new QPushButton();
    preview->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    preview->setCursor(Qt::PointingHandCursor);

    custom = new QPushButton("Custom...");
    custom->setFixedWidth(85);

    resetBtn = new QPushButton("Reset");
    resetBtn->setFixedWidth(50);

    row_layout->addWidget(preview);
    row_layout->addWidget(custom);
    row_layout->addWidget(resetBtn);
    row_layout->addStretch();

    layout->addRow(label, row_layout);
}

void ColorsPage::apply()
{
    // 应用背景色
    m_cloudview->setBackgroundColor({
        static_cast<uint8_t>(m_bg_color.red()),
        static_cast<uint8_t>(m_bg_color.green()),
        static_cast<uint8_t>(m_bg_color.blue())});

    // 应用包围盒色到所有点云/模型
    ct::ColorRGB boxRGB{
        static_cast<uint8_t>(m_box_color.red()),
        static_cast<uint8_t>(m_box_color.green()),
        static_cast<uint8_t>(m_box_color.blue())};

    for (auto& cloud : m_cloudtree->getAllClouds()) {
        cloud->setBoxColor(boxRGB);
        QString boxId = QString::fromStdString(cloud->boxId());
        if (m_cloudview->contains(boxId)) {
            m_cloudview->setShapeColor(boxId, boxRGB);
        }
    }
    m_cloudview->refresh();
}

void ColorsPage::reset()
{
    // 重置背景色
    m_bg_color = QColor(255, 255, 255);
    m_bg_preview->setStyleSheet(
        "QPushButton{background-color:rgb(255,255,255);border:1px solid #999;border-radius:3px;min-width:40px;min-height:22px;}");
    m_cloudview->resetBackgroundColor();

    // 重置包围盒色
    m_box_color = QColor(0, 255, 0);
    m_box_preview->setStyleSheet(
        "QPushButton{background-color:rgb(0,255,0);border:1px solid #999;border-radius:3px;min-width:40px;min-height:22px;}");

    ct::ColorRGB defaultBoxColor{0, 255, 0};
    for (auto& cloud : m_cloudtree->getAllClouds()) {
        cloud->setBoxColor(defaultBoxColor);
        QString boxId = QString::fromStdString(cloud->boxId());
        if (m_cloudview->contains(boxId)) {
            m_cloudview->setShapeColor(boxId, defaultBoxColor);
        }
    }
    m_cloudview->refresh();
}
