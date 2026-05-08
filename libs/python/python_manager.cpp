#include "python_manager.h"
#include "python_worker.h"
#include "python_bridge.h"
#include "cloud_registry.h"

#undef slots
#include <pybind11/embed.h>
#include <filesystem>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace py = pybind11;

namespace pw
{

// Python version constants (change only when upgrading Python)
static const char* const PY_DLL       = "python39.dll";
static const char* const PY_DLL_SHORT  = "python3.dll";
static const char* const PY_ZIP       = "python39.zip";
static wchar_t s_python_home[MAX_PATH] = {0};

PythonManager& PythonManager::instance()
{
    static PythonManager mgr;
    return mgr;
}

PythonManager::PythonManager() = default;

PythonManager::~PythonManager()
{
    if (m_initialized)
        finalize();
}

PythonCloudRegistry* PythonManager::registry() const
{
    return m_bridge ? &m_bridge->registry() : nullptr;
}

void PythonManager::initialize()
{
    if (m_initialized) return;

    try {
        // 0. Configure Python Home + DLL search path before Py_Initialize
        setupPythonHome();

        // 1. Start Python interpreter
        py::initialize_interpreter();

        // 1.5 Py_SetPath() sets sys.prefix to empty, restore manually
        if (m_is_embedded) {
            auto sys = py::module_::import("sys");
            sys.attr("prefix") = m_detected_python_home;
            sys.attr("exec_prefix") = m_detected_python_home;
            sys.attr("base_prefix") = m_detected_python_home;
        }

        // 2. Set sys.argv (required by numpy etc.)
        wchar_t argv0[] = L"pointworks";
        wchar_t* argv[] = { argv0 };
        PySys_SetArgv(1, argv);

        // 3. Redirect stdout/stderr
        redirectStdio();

        // 4. Register DLL search paths
        addDllDirectories();

        // 5. Add search paths
        addSearchPaths();

        // 6. Create Bridge and Worker
        m_bridge = new PythonBridge();
        m_worker = new PythonWorker(m_bridge);

        // 7. Release GIL on main thread
        PyEval_SaveThread();

        m_initialized = true;
        m_init_message = "Python initialized successfully";

    } catch (const py::error_already_set& e) {
        m_init_message = std::string("Python init failed: ") + e.what();
        std::cerr << m_init_message << std::endl;
    } catch (const std::exception& e) {
        m_init_message = std::string("Python init failed: ") + e.what();
        std::cerr << m_init_message << std::endl;
    }
}

void PythonManager::finalize()
{
    if (!m_initialized) return;

    if (m_worker) {
        m_worker->cancel();
        m_worker->wait(3000);
        if (m_worker->isRunning())
            m_worker->terminate();
        delete m_worker;
        m_worker = nullptr;
    }

    delete m_bridge;
    m_bridge = nullptr;

    m_initialized = false;
}

void PythonManager::setCustomPythonHome(const std::string& path)
{
    m_custom_python_home = path;
}

std::string getExeDir()
{
#ifdef _WIN32
    wchar_t exePath[MAX_PATH];
    GetModuleFileNameW(nullptr, exePath, MAX_PATH);
    std::filesystem::path p(exePath);
    return p.parent_path().string();
#else
    return ".";
#endif
}

std::string getEnvVar(const std::string& name)
{
#ifdef _WIN32
    char buf[32767];
    DWORD len = GetEnvironmentVariableA(name.c_str(), buf, sizeof(buf));
    if (len == 0) return "";
    return std::string(buf, len);
#else
    const char* val = std::getenv(name.c_str());
    return val ? val : "";
#endif
}

void setEnvVar(const std::string& name, const std::string& value)
{
#ifdef _WIN32
    SetEnvironmentVariableA(name.c_str(), value.c_str());
#else
    setenv(name.c_str(), value.c_str(), 1);
#endif
}

void PythonManager::setupPythonHome()
{
    std::string appDir = getExeDir();
    std::string pyHome;

    // Three-level priority:
    // 1. User custom path (from QSettings, injected before initialize())
    if (!m_custom_python_home.empty() && std::filesystem::exists(m_custom_python_home)) {
        pyHome = m_custom_python_home;
    }
    // 2. python/ directory next to exe (bundled embedded Python)
    else {
        std::string bundledPython = appDir + "/python";
        if (std::filesystem::exists(bundledPython) &&
            (std::filesystem::exists(bundledPython + "/" + PY_DLL) ||
             std::filesystem::exists(bundledPython + "/python.exe"))) {
            pyHome = bundledPython;
        }
    }
    // 3. Compile-time PYTHON_HOME macro (dev environment fallback)
    if (pyHome.empty()) {
        pyHome = PYTHON_HOME;
    }

    // Detect embedded Python (has python39.zip)
    m_is_embedded = std::filesystem::exists(pyHome + "/" + PY_ZIP);

    if (m_is_embedded) {
        // Embedded Python: use Py_SetPath() for immediate module search path
        std::string modulePathStr = pyHome + "/" + PY_ZIP + ";" +
                                    pyHome + ";" +
                                    pyHome + "/Lib;" +
                                    pyHome + "/Lib/site-packages";
        std::wstring modulePath(modulePathStr.begin(), modulePathStr.end());
        Py_SetPath(modulePath.c_str());
    } else {
        // Full Python install: use Py_SetPythonHome()
        std::wstring wPyHome(pyHome.begin(), pyHome.end());
        wcscpy_s(s_python_home, wPyHome.c_str());
        Py_SetPythonHome(s_python_home);
    }
    m_detected_python_home = pyHome;

    // Inject Python DLL directory into process PATH
    std::string dllDir = pyHome;
    if (!m_is_embedded) {
        dllDir += ";" + pyHome + "/DLLs";
    }
    std::string curPath = getEnvVar("PATH");
    setEnvVar("PATH", dllDir + ";" + curPath);
}

void PythonManager::addDllDirectories()
{
    py::exec(R"PY(
import sys, os, glob

py_home = sys.prefix
if not py_home or not os.path.isdir(py_home):
    raise RuntimeError("sys.prefix is not set or does not exist")

sp = os.path.join(py_home, 'Lib', 'site-packages')
if not os.path.isdir(sp):
    raise RuntimeError("site-packages not found: " + sp)

dll_dirs = set()

for d in [py_home, os.path.join(py_home, 'DLLs'), os.path.join(py_home, 'Library', 'bin')]:
    if os.path.isdir(d):
        dll_dirs.add(d)

for pattern in [
    os.path.join(sp, '*', '.libs'),
    os.path.join(sp, '*.libs'),
]:
    for d in glob.glob(pattern):
        if os.path.isdir(d):
            dll_dirs.add(d)

for d in dll_dirs:
    if hasattr(os, 'add_dll_directory'):
        try:
            os.add_dll_directory(d)
        except OSError:
            pass

valid = os.pathsep.join(d for d in dll_dirs if os.path.isdir(d))
if valid:
    os.environ['PATH'] = valid + os.pathsep + os.environ.get('PATH', '')
)PY");
}

void PythonManager::redirectStdio()
{
    py::exec(R"(
import os, sys

class _ConsoleRedirect:
    def __init__(self, fd):
        self._fd = fd
    def write(self, text):
        if text:
            os.write(self._fd, text.encode('utf-8'))
    def flush(self):
        pass

sys.stdout = _ConsoleRedirect(1)
sys.stderr = _ConsoleRedirect(2)
)");
}

void PythonManager::addSearchPaths()
{
    auto sys = py::module_::import("sys");
    auto paths = sys.attr("path").cast<py::list>();

    std::string appDir = getExeDir();
    std::string scriptsDir = appDir + "/scripts";

    if (std::filesystem::exists(scriptsDir)) {
        paths.append(scriptsDir);

        for (const auto& entry : std::filesystem::recursive_directory_iterator(scriptsDir)) {
            if (entry.is_directory())
                paths.append(entry.path().string());
        }
    }

    if (std::filesystem::exists(appDir + "/python")) {
        paths.append(appDir + "/python");
    }
}

} // namespace pw
