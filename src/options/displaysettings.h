#ifndef POINTWORKS_DISPLAYSETTINGS_H
#define POINTWORKS_DISPLAYSETTINGS_H

#include "ui/base/customdialog.h"
#include "displaysettings_page.h"

#include <QListWidget>
#include <QStackedWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

class DisplaySettingsDialog : public pw::CustomDialog
{
    Q_OBJECT
public:
    explicit DisplaySettingsDialog(QWidget* parent = nullptr);
    ~DisplaySettingsDialog() override;

    void init() override;
    void reset() override;

    /**
     * @brief 注册新的设置模块页面
     * @param name 侧边栏显示的名称
     * @param page 设置页面（生命周期由对话框管理）
     */
    void addPage(const QString& name, DisplaySettingsPage* page);

private slots:
    void onSidebarChanged(int index);
    void onApply();
    void onReset();

private:
    QListWidget* m_sidebar;
    QStackedWidget* m_pages;
    QPushButton* m_btn_apply;
    QPushButton* m_btn_reset;
    QList<DisplaySettingsPage*> m_all_pages;
};

#endif //POINTWORKS_DISPLAYSETTINGS_H
