#ifndef POINTWORKS_PYTHON_MANAGER_H
#define POINTWORKS_PYTHON_MANAGER_H

#include "python_bridge.h"

#include <QString>
#include <memory>

namespace ct
{

class PythonWorker;

class PythonManager
{
public:
    static PythonManager& instance();

    /// 初始化 Python 解释器（QApplication 创建后调用）
    void initialize();

    /// 安全关闭 Python 解释器（QApplication 退出前调用）
    void finalize();

    /// 在 initialize() 之前设置自定义 Python 路径（来自 QSettings）
    void setCustomPythonHome(const QString& path);

    bool isInitialized() const { return m_initialized; }
    QString initMessage() const { return m_init_message; }

    PythonBridge* bridge() const { return m_bridge; }
    PythonWorker* worker() const { return m_worker; }

private:
    PythonManager();
    ~PythonManager();

    PythonManager(const PythonManager&) = delete;
    PythonManager& operator=(const PythonManager&) = delete;

    void redirectStdio();
    void addSearchPaths();
    void setupPythonHome();
    void addDllDirectories();

    PythonBridge* m_bridge = nullptr;
    PythonWorker* m_worker = nullptr;
    bool m_initialized = false;
    bool m_is_embedded = false;
    QString m_custom_python_home;
    QString m_detected_python_home;
    QString m_init_message;
};

} // namespace ct

#endif // POINTWORKS_PYTHON_MANAGER_H
