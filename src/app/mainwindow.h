//
// Created by LBC on 2024/10/8.
//

#ifndef POINTWORKS_MAINWINDOW_H
#define POINTWORKS_MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>

#include "ui_mainwindow.h"

#include "ui/base/customdialog.h"
#include "projectmanager.h"
#include "recentprojects.h"

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
        ct::createDialog<T>(this, label, ui->cloudview, ui->cloudtree, ui->console);
    }

    template <class T>
    void createModalDialog(const QString& label)
    {
        ct::createDialog<T>(this, label, ui->cloudview, ui->cloudtree, ui->console, false, true);
    }

    template <class T>
    void createToolDialog(const QString& label)
    {
        ct::createDialog<T>(this, label, ui->cloudview, ui->cloudtree, ui->console, false, false);
    }

protected:
    void moveEvent(QMoveEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

private slots:
    void onNewProject();
    void onOpenProject();
    void onSaveProject();
    void onSaveProjectAs();
    void onOpenRecentProject();
    void updateRecentMenu();
    void onProjectModified(bool modified);
    void updateWindowTitle();

private:
    Ui::MainWindow *ui;
    ProjectManager* m_project_manager = nullptr;
    RecentProjects* m_recent_projects = nullptr;
    QMenu* m_open_recent_menu = nullptr;

    void saveProjectTo(const QString& path);
    void connectProjectSignals();
};


#endif //POINTWORKS_MAINWINDOW_H
