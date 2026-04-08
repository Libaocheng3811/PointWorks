#ifndef POINTWORKS_CUSTOMDOCK_H
#define POINTWORKS_CUSTOMDOCK_H

#include "cloudtree.h"

#include "viz/cloudview.h"
#include "viz/console.h"
#include "core/exports.h"

#include <QDockWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QCloseEvent>
#include <map>
#include <set>

namespace ct
{
    class CustomDock : public QDockWidget
    {
        Q_OBJECT
    public:
        explicit CustomDock(QWidget* parent = nullptr) : QDockWidget(parent) {}

        ~CustomDock() {}

        void setCloudView(CloudView* cloudview) {m_cloudview = cloudview; }

        void setCloudTree(CloudTree* cloudtree) {m_cloudtree = cloudtree; }

        void setConsole(Console* console) {m_console = console; }

        virtual void init() {}

        virtual void reset() {}

        virtual void deinit() {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        void printI(const QString& message) {m_console->print(LOG_INFO, message); }
        void printW(const QString& message) {m_console->print(LOG_WARNING, message); }
        void printE(const QString& message) {m_console->print(LOG_ERROR, message); }

        void closeEvent(QCloseEvent* event) override
        {
            reset();
            deinit();

            // 先激活同 tab 组的其他 dock，防止整个 tab 组折叠
            if (auto* mw = qobject_cast<QMainWindow*>(window())) {
                for (auto* sibling : mw->tabifiedDockWidgets(this)) {
                    if (sibling != this) {
                        sibling->show();
                        sibling->raise();
                        break;
                    }
                }
            }

            event->ignore();
            hide();
        }
    public:
        CloudView* m_cloudview;
        CloudTree* m_cloudtree;
        Console* m_console;
    };
    static std::map<QString, CustomDock*> registed_docks;
    static std::set<QString> left_label;
    static std::set<QString> right_label;
    static std::map<QString, bool> docks_visible;

    /**
     * @brief 创建停靠窗口
     * @param parent 主窗口,停靠的窗口将被添加到这个主窗口中
     * @param label 窗口标签， 窗口停靠的标签，用于标识和管理
     * @param area 停靠区域  left/right/top/bottom， 默认为左侧停靠区域
     * @param dock 指向另一个窗口的指针，用于合并停靠窗口，
     */
    template <class T>
    void createDock(QMainWindow* parent, const QString& label, CloudView* cloudview = nullptr,
                   CloudTree* cloudtree = nullptr, Console* console = nullptr,
                   Qt::DockWidgetArea area = Qt::LeftDockWidgetArea, QDockWidget* dock = nullptr)
    {
        if (parent == nullptr) return;
        if (registed_docks.find(label) == registed_docks.end())  //register dock
            registed_docks[label] = nullptr;
        if (registed_docks.find(label)->second == nullptr)  // creat new dock
        {
            registed_docks[label] = new T(parent);
            if (cloudview)
                registed_docks[label]->setCloudView(cloudview);
            if (cloudtree)
                registed_docks[label]->setCloudTree(cloudtree);
            if (console)
                registed_docks[label]->setConsole(console);
            registed_docks[label]->init();
            QObject::connect(registed_docks[label], &QDockWidget::visibilityChanged, [=](bool state)
                            {docks_visible[label] = !state; });
            parent->addDockWidget(area, registed_docks[label]);
            if (area == Qt::LeftDockWidgetArea)
                left_label.insert(label);
            else
                right_label.insert(label);
            if (dock == nullptr)
                for (auto& dock : registed_docks)
                {
                    if (dock.first != label && (left_label.count(dock.first) > 0) &&
                        area == Qt::LeftDockWidgetArea && dock.second != nullptr)
                    {
                        parent->tabifyDockWidget(dock.second, registed_docks[label]);
                        break;
                    }
                    // right
                    if (dock.first != label && (right_label.count(dock.first) > 0) &&
                        area == Qt::RightDockWidgetArea && dock.second != nullptr)
                    {
                        parent->tabifyDockWidget(dock.second, registed_docks[label]);
                        break;
                    }
                }
            else
                parent->tabifyDockWidget(dock, registed_docks[label]);
            registed_docks[label]->setVisible(true);
            registed_docks[label]->raise();
        }
        else // update dock
        {
            if (docks_visible.find(label) == docks_visible.end()) return;
            if (docks_visible.find(label)->second)
            {
                registed_docks[label]->show();
                registed_docks[label]->raise();
            }
            else
            {
                registed_docks[label]->hide();
            }
        }
    }

    /**
     * @brief 获取窗口指针
     */
    template <class T>
    T* getDock(const QString& label)
    {
        if (registed_docks.find(label) == registed_docks.end())
            return nullptr;
        else
            return (T*)registed_docks.find(label)->second;
    }
}
#endif //POINTWORKS_CUSTOMDOCK_H
