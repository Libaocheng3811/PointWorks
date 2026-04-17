#include "displaysettings.h"
#include "displaysettings_colors_page.h"

DisplaySettingsDialog::DisplaySettingsDialog(QWidget* parent)
    : ct::CustomDialog(parent)
{
    setWindowTitle("Display Settings");
    resize(480, 360);

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // 中间区域：侧边栏 + 页面
    auto* center_layout = new QHBoxLayout();
    center_layout->setSpacing(0);

    // 左侧分类列表
    m_sidebar = new QListWidget(this);
    m_sidebar->setFixedWidth(110);
    m_sidebar->setIconSize(QSize(20, 20));
    m_sidebar->setSpacing(2);
    m_sidebar->setStyleSheet(
        "QListWidget { border: 1px solid #ccc; border-radius: 4px; padding: 4px; }"
        "QListWidget::item { padding: 6px 4px; border-radius: 3px; }"
        "QListWidget::item:selected { background-color: #1890ff; color: white; }"
        "QListWidget::item:hover { background-color: #e6f7ff; }"
    );

    // 右侧页面容器
    m_pages = new QStackedWidget(this);

    center_layout->addWidget(m_sidebar);
    center_layout->addWidget(m_pages, 1);
    main_layout->addLayout(center_layout);

    // 底部按钮
    auto* btn_layout = new QHBoxLayout();
    btn_layout->addStretch();

    m_btn_reset = new QPushButton("Reset");
    m_btn_reset->setFixedWidth(70);
    m_btn_apply = new QPushButton("Apply");
    m_btn_apply->setFixedWidth(70);

    btn_layout->addWidget(m_btn_reset);
    btn_layout->addWidget(m_btn_apply);
    main_layout->addLayout(btn_layout);

    // 信号连接
    connect(m_sidebar, &QListWidget::currentRowChanged, this, &DisplaySettingsDialog::onSidebarChanged);
    connect(m_btn_apply, &QPushButton::clicked, this, &DisplaySettingsDialog::onApply);
    connect(m_btn_reset, &QPushButton::clicked, this, &DisplaySettingsDialog::onReset);
}

DisplaySettingsDialog::~DisplaySettingsDialog() = default;

void DisplaySettingsDialog::init()
{
    // 清空已有页面
    while (m_pages->count() > 0)
        m_pages->removeWidget(m_pages->widget(0));
    m_sidebar->clear();
    m_all_pages.clear();

    // 注册设置模块
    addPage("Colors", new ColorsPage(m_cloudview, m_cloudtree, this));

    if (m_sidebar->count() > 0)
        m_sidebar->setCurrentRow(0);
}

void DisplaySettingsDialog::reset()
{
    onReset();
}

void DisplaySettingsDialog::addPage(const QString& name, DisplaySettingsPage* page)
{
    m_all_pages.append(page);
    m_sidebar->addItem(name);
    m_pages->addWidget(page);
}

void DisplaySettingsDialog::onSidebarChanged(int index)
{
    if (index >= 0 && index < m_pages->count())
        m_pages->setCurrentIndex(index);
}

void DisplaySettingsDialog::onApply()
{
    for (auto* page : m_all_pages) {
        page->apply();
    }
}

void DisplaySettingsDialog::onReset()
{
    for (auto* page : m_all_pages) {
        page->reset();
    }
}
