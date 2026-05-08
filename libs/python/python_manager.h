#ifndef POINTWORKS_PYTHON_MANAGER_H
#define POINTWORKS_PYTHON_MANAGER_H

#include <string>
#include <memory>

namespace pw
{

class PythonBridge;
class PythonWorker;
class PythonCloudRegistry;

class PythonManager
{
public:
    static PythonManager& instance();

    /// Initialize Python interpreter (call after QApplication is created)
    void initialize();

    /// Safe shutdown (call before QApplication exits)
    void finalize();

    /// Set custom Python path before initialize() (from QSettings)
    void setCustomPythonHome(const std::string& path);

    bool isInitialized() const { return m_initialized; }
    std::string initMessage() const { return m_init_message; }

    PythonBridge* bridge() const { return m_bridge; }
    PythonWorker* worker() const { return m_worker; }

    /// Qt-free access to the cloud registry
    PythonCloudRegistry* registry() const;

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
    std::string m_custom_python_home;
    std::string m_detected_python_home;
    std::string m_init_message;
};

} // namespace pw

#endif // POINTWORKS_PYTHON_MANAGER_H
