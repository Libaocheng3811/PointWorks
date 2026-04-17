#ifndef POINTWORKS_CUSTOMDIALOG_H
#define POINTWORKS_CUSTOMDIALOG_H

#include "cloudtree.h"

#include "viz/cloudview.h"
#include "viz/console.h"
#include "core/exports.h"

#include <QDialog>
#include <QMainWindow>
#include <QPushButton>
#include <QResizeEvent>
#include <map>

namespace ct
{
    class CustomDialog : public QDialog
    {
        Q_OBJECT
    public:
        explicit CustomDialog(QWidget* parent = nullptr)
            : QDialog(parent), m_cloudview(nullptr), m_cloudtree(nullptr), m_console(nullptr) {}

        ~CustomDialog() {}

        void setCloudView(CloudView* cloudview) {m_cloudview = cloudview; }

        void setCloudTree(CloudTree* cloudtree) {m_cloudtree = cloudtree; }

        void setConsole(Console* console) {m_console = console; }

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
    };

    static std::map<QString, CustomDialog*> registed_dialogs;
    static std::map<QString, bool> dialogs_visible;

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

        if (registed_dialogs.find(label) == registed_dialogs.end()) // register dock
            registed_dialogs[label] = nullptr;

        if (registed_dialogs.find(label)->second == nullptr) // create new dialog
        {
            // 互斥逻辑：已有对话框打开时，不允许打开其他对话框
            for (auto& dialog : registed_dialogs){
                if (dialog.first != label && dialog.second != nullptr) return nullptr;
            }

            registed_dialogs[label] = new T(parent);
            auto dlg = registed_dialogs[label];

            if (cloudview)
                dlg->setCloudView(cloudview);
            if (cloudtree)
                dlg->setCloudTree(cloudtree);
            if (console)
                dlg->setConsole(console);
            dlg->setAttribute(Qt::WA_DeleteOnClose);

            if (isToolWidget){
                // 工具浮窗模式
                dlg->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
                QPoint pos = cloudview->mapToGlobal(QPoint(0, 0));
                dlg->move(pos.x() + cloudview->width() - dlg->width() - 9, pos.y() + 9);

                // 跟随移动
                QObject::connect(cloudview, &CloudView::posChanged, [=](const QPoint& pos)
                {
                    if (registed_dialogs[label] != nullptr) {
                        int ax = pos.x() + cloudview->width() - registed_dialogs[label]->width() - 9;
                        int ay = pos.y() + 9;
                        registed_dialogs[label]->move(ax, ay);
                    }
                });

                QObject::connect(dlg, &CustomDialog::sizeChanged, [=](const QSize& size)
                {
                    if (registed_dialogs[label] != nullptr) {
                        QPoint pos = cloudview->mapToGlobal(QPoint(0, 0));
                        int ax = pos.x() + cloudview->width() - size.width() - 9;
                        int ay = pos.y() + 9;
                        registed_dialogs[label]->move(ax, ay);
                    }
                });
            }
            else{
                // 普通/模态对话框模式
                dlg->setWindowFlags(Qt::Dialog | Qt::WindowCloseButtonHint);
                dlg->setWindowTitle(label);
            }

            dlg->init();

            QObject::connect(dlg, &QDialog::destroyed, [=] { registed_dialogs[label] = nullptr;});

            if (isModal){
//                dlg->exec();
                dlg->setWindowModality(Qt::ApplicationModal);
                dlg->show();
            }
            else{
                dlg->show();
            }
        }
        else{ // update dialog (已存在则关闭，通常用于 toggle)
            registed_dialogs[label]->close();
        }
        return (T*)registed_dialogs[label];
    }

    /**
     * @brief 获取窗口指针
     * @param label 窗口标签
     */
    // 模板函数
    template <class T>
    T* getDialog(const QString& label)
    {
        // 如果没有找到指定标签，返回空指针，表示没有注册的对话框
        if (registed_dialogs.find(label) == registed_dialogs.end())
            return nullptr;
        else
            // 否则返回与标签关联的对话框指针，强制转换为T类型的指针
            return (T*)registed_dialogs.find(label)->second;
    }
}
#endif //POINTWORKS_CUSTOMDIALOG_H
