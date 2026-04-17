#ifndef POINTWORKS_DISPLAYSETTINGS_PAGE_H
#define POINTWORKS_DISPLAYSETTINGS_PAGE_H

#include <QWidget>

/**
 * @brief Display Settings 模块页面基类
 * 每个设置模块继承此类，实现 apply() 和 reset() 方法。
 * 通过 DisplaySettingsDialog::addPage() 注册到容器对话框中。
 */
class DisplaySettingsPage : public QWidget
{
    Q_OBJECT
public:
    explicit DisplaySettingsPage(QWidget* parent = nullptr) : QWidget(parent) {}

    /// 应用当前页面的所有设置
    virtual void apply() = 0;

    /// 恢复当前页面的所有设置为默认值
    virtual void reset() = 0;
};

#endif //POINTWORKS_DISPLAYSETTINGS_PAGE_H
