#include "mainwindow.h"
#include "python/python_manager.h"

#include "ui/base/language_manager.h"

#include <QApplication>
#include <QDesktopWidget>
#include <QSettings>

int main(int argc, char* argv[]) {
    QApplication a(argc, argv);

    // Read user's custom Python path from QSettings (before Python init)
    QSettings settings("PointWorks", "PointWorks");
    QString customPy = settings.value("python_home").toString();
    if (!customPy.isEmpty()) {
        pw::PythonManager::instance().setCustomPythonHome(customPy.toStdString());
    }

    pw::PythonManager::instance().initialize();

    // Restore saved language preference, default to English
    int langVal = settings.value("language", 0).toInt();
    auto lang = static_cast<pw::LanguageManager::Language>(langVal);
    pw::LanguageManager::instance().switchLanguage(lang);

    MainWindow w;
    w.showMaximized();

    int ret = a.exec();

    pw::PythonManager::instance().finalize();

    return ret;
}
