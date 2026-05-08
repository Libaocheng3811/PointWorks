#ifndef POINTWORKS_CUSTOMDIALOG_H
#define POINTWORKS_CUSTOMDIALOG_H

#include "cloudtree.h"
#include "progress_manager.h"
#include "dialog_registry.h"
#include "viewport_manager.h"

#include "viz/cloudview.h"
#include "viz/console.h"
#include "core/exports.h"

#include <QDialog>
#include <QMainWindow>
#include <QPushButton>
#include <QResizeEvent>
#include <map>

namespace pw
{
    class CustomDialog : public QDialog
    {
        Q_OBJECT
    public:
        explicit CustomDialog(QWidget* parent = nullptr)
            : QDialog(parent), m_cloudview(nullptr), m_cloudtree(nullptr), m_console(nullptr), m_progress(nullptr), m_viewport_mgr(nullptr) {}

        ~CustomDialog() {}

        void setCloudView(CloudView* cloudview) {m_cloudview = cloudview; }

        void setCloudTree(CloudTree* cloudtree) {m_cloudtree = cloudtree; }

        void setConsole(Console* console) {m_console = console; }

        void setProgressManager(ProgressManager* progress) {m_progress = progress; }

        void setViewportManager(ViewportManager* mgr) {
            m_viewport_mgr = mgr;
            if (mgr) {
                connect(mgr, &ViewportManager::activeViewChanged, this, [this](CloudView* view) {
                    m_cloudview = view;
                });
            }
        }

        virtual void init() {}

        virtual void reset() {}

        virtual void deinit() {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    signals:
            void sizeChanged(const QSize&);

    protected:
        void printI(const QString& message) {m_console->print(LOG_INFO, message); }
        void printW(const QString& message) {m_console->print(LOG_WARNING, message); }
        void printE(const QString& message) {m_console->print(LOG_ERROR, message); }

        void closeEvent(QCloseEvent* event)
        {
            reset();
            deinit();
            return QDialog::closeEvent(event);
        }

        void resizeEvent(QResizeEvent* event)
        {
            emit sizeChanged(event->size());
            return QDialog::resizeEvent(event);
        }

    public:
        CloudView* m_cloudview;
        CloudTree* m_cloudtree;
        Console* m_console;
        ProgressManager* m_progress;
        ViewportManager* m_viewport_mgr;
    };

    /**
     * @brief 创建弹出窗口
     * @param parent 主窗口
     * @param label 窗口标签
     * @param cloudview, cloudtree, console 注入的组件
     * @param isToolWidget 是否为工具浮窗（无边框、跟随 CloudView 移动），默认为 true
     * @param isModal 是否为模态窗口（阻塞），默认为 false
     */
    template <class T>
    T* createDialog(QMainWindow* parent, const QString& label, CloudView* cloudview = nullptr,
                      CloudTree* cloudtree = nullptr, Console* console = nullptr,
                      bool isToolWidget = true, bool isModal = false)
    {
        if (parent == nullptr) return nullptr;

        auto& reg = DialogRegistry::instance();

        if (reg.getDialog(label) == nullptr) // create new dialog
        {
            // 互斥逻辑：已有对话框打开时，不允许打开其他对话框
            if (reg.hasOpenDialog()) return nullptr;

            reg.registerDialog(label, new T(parent));
            auto dlg = reg.getDialog(label);

            if (cloudview)
                dlg->setCloudView(cloudview);
            if (cloudtree)
                dlg->setCloudTree(cloudtree);
            if (console)
                dlg->setConsole(console);
            if (cloudtree)
                dlg->setProgressManager(cloudtree->progressManager());
            dlg->setAttribute(Qt::WA_DeleteOnClose);

            if (isToolWidget){
                // 工具浮窗模式
                dlg->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
                QPoint pos = cloudview->mapToGlobal(QPoint(0, 0));
                dlg->move(pos.x() + cloudview->width() - dlg->width() - 9, pos.y() + 9);

                // 跟随移动 — 监听活跃视窗的 posChanged 信号
                auto weakDlg = dlg;
                auto weakView = cloudview;
                // 由于活跃视窗会切换，我们直接使用 ViewportManager（如果可用）
                // 但 createDialog 不知道 ViewportManager，所以监听当前 cloudview
                QObject::connect(cloudview, &CloudView::posChanged, [weakDlg, weakView, &reg, label](const QPoint& pos)
                {
                    auto* d = weakDlg ? reg.getDialog(label) : nullptr;
                    if (d) {
                        int ax = pos.x() + weakView->width() - d->width() - 9;
                        int ay = pos.y() + 9;
                        d->move(ax, ay);
                    }
                });

                QObject::connect(dlg, &CustomDialog::sizeChanged, [weakDlg, weakView, &reg, label](const QSize& size)
                {
                    auto* d = weakDlg ? reg.getDialog(label) : nullptr;
                    if (d) {
                        QPoint pos = weakView->mapToGlobal(QPoint(0, 0));
                        int ax = pos.x() + weakView->width() - size.width() - 9;
                        int ay = pos.y() + 9;
                        d->move(ax, ay);
                    }
                });
            }
            else{
                // 普通/模态对话框模式
                dlg->setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);
                dlg->setWindowTitle(label);
            }

            dlg->init();

            QObject::connect(dlg, &QDialog::destroyed, [=] { DialogRegistry::instance().unregisterDialog(label); });

            if (isModal){
                dlg->setWindowModality(Qt::ApplicationModal);
                dlg->show();
            }
            else{
                dlg->show();
            }
        }
        else{ // update dialog (已存在则关闭，通常用于 toggle)
            reg.getDialog(label)->close();
        }
        return (T*)reg.getDialog(label);
    }

    /**
     * @brief 获取窗口指针
     * @param label 窗口标签
     */
    template <class T>
    T* getDialog(const QString& label)
    {
        return (T*)DialogRegistry::instance().getDialog(label);
    }
}
#endif //POINTWORKS_CUSTOMDIALOG_H
