#include "mainwindow.h"
#include "python/python_manager.h"

#include <QApplication>
#include <QDesktopWidget>

int main(int argc, char* argv[]) {
    QApplication a(argc, argv);

    ct::PythonManager::instance().initialize();

    MainWindow w;
    w.showMaximized();

    int ret = a.exec();

    ct::PythonManager::instance().finalize();

    return ret;
}
