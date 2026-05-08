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

namespace pw
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

        auto& reg = DialogRegistry::instance();
        if (reg.getDock(label) == nullptr)  // create new dock
        {
            reg.registerDock(label, new T(parent));
            if (cloudview)
                reg.getDock(label)->setCloudView(cloudview);
            if (cloudtree)
                reg.getDock(label)->setCloudTree(cloudtree);
            if (console)
                reg.getDock(label)->setConsole(console);
            reg.getDock(label)->init();
            QObject::connect(reg.getDock(label), &QDockWidget::visibilityChanged, [=](bool state)
                            { reg.setDockVisible(label, state); });
            parent->addDockWidget(area, reg.getDock(label));
            if (area == Qt::LeftDockWidgetArea)
                reg.addLeftLabel(label);
            else
                reg.addRightLabel(label);
            if (dock == nullptr)
            {
                const auto& labels = (area == Qt::LeftDockWidgetArea) ? reg.leftLabels() : reg.rightLabels();
                for (const auto& lbl : labels)
                {
                    if (lbl != label) {
                        auto* existing = reg.getDock(lbl);
                        if (existing) {
                            parent->tabifyDockWidget(existing, reg.getDock(label));
                            break;
                        }
                    }
                }
            }
            else
                parent->tabifyDockWidget(dock, reg.getDock(label));
            reg.getDock(label)->setVisible(true);
            reg.getDock(label)->raise();
        }
        else // update dock
        {
            if (reg.isDockVisible(label))
            {
                reg.getDock(label)->show();
                reg.getDock(label)->raise();
            }
            else
            {
                reg.getDock(label)->hide();
            }
        }
    }

    /**
     * @brief 获取窗口指针
     */
    template <class T>
    T* getDock(const QString& label)
    {
        return (T*)DialogRegistry::instance().getDock(label);
    }
}
#endif //POINTWORKS_CUSTOMDOCK_H
