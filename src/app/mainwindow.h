#ifndef POINTWORKS_MAINWINDOW_H
#define POINTWORKS_MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>

#include "ui_mainwindow.h"

#include "ui/base/customdialog.h"
#include "ui/base/viewport_manager.h"
#include "ui/base/scenenodetype.h"
#include "projectmanager.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

    template <class T>
    void createDialog(const QString& label)
    {
        ct::createDialog<T>(this, label, m_viewport_mgr->activeView(), ui->cloudtree, ui->console);
    }

    template <class T>
    void createModalDialog(const QString& label)
    {
        ct::createDialog<T>(this, label, m_viewport_mgr->activeView(), ui->cloudtree, ui->console, false, true);
    }

    template <class T>
    void createToolDialog(const QString& label)
    {
        ct::createDialog<T>(this, label, m_viewport_mgr->activeView(), ui->cloudtree, ui->console, false, false);
    }

protected:
    void moveEvent(QMoveEvent* event) override;
    void closeEvent(QCloseEvent* event) override;
    void changeEvent(QEvent* event) override;

private:
    Ui::MainWindow *ui;
    ProjectManager* m_project_manager = nullptr;
    ct::ViewportManager* m_viewport_mgr = nullptr;

private slots:
    void onTreeSelectionChanged();
    void updateActionEnableState(const ct::SelectionInfo& info);
};


#endif //POINTWORKS_MAINWINDOW_H
