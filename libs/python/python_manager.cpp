#include "python_manager.h"
#include "python_worker.h"

#undef slots
#include <pybind11/embed.h>
#include <QCoreApplication>
#include <QDir>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace py = pybind11;

namespace ct
{

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

void PythonManager::initialize()
{
    if (m_initialized) return;

    try {
        // 0. 在 Py_Initialize 之前配置 Python Home + DLL 搜索路径
        setupPythonHome();

        // 1. 启动 Python 解释器
        py::initialize_interpreter();

        // 2. 设置 sys.argv（numpy 等库依赖）
        wchar_t argv0[] = L"pointworks";
        wchar_t* argv[] = { argv0 };
        PySys_SetArgv(1, argv);

        // 3. 重定向 stdout/stderr
        redirectStdio();

        // 4. 注册 DLL 搜索路径
        addDllDirectories();

        // 5. 添加搜索路径
        addSearchPaths();

        // 6. 创建 Bridge 和 Worker
        m_bridge = new PythonBridge();
        m_worker = new PythonWorker(m_bridge);

        // 7. 主线程释放 GIL
        PyEval_SaveThread();

        m_initialized = true;

    } catch (const py::error_already_set& e) {
        std::cerr << "[PythonManager] Init failed: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[PythonManager] Init failed: " << e.what() << std::endl;
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

    // 不调用 Py_Finalize()——NumPy/SciPy 的 C 扩展会触发段错误
    m_initialized = false;
}

void PythonManager::setupPythonHome()
{
#ifdef _WIN32
    // PYTHON_HOME 由 CMakeLists.txt 通过 -DPYTHON_HOME="..." 注入
    // 与 MY_NATIVE_PYTHON_DIR 保持一致，只需在 CMakeLists.txt 中修改一处
    QString pyHome = QString::fromUtf8(PYTHON_HOME);
    wcscpy_s(s_python_home, pyHome.toStdWString().c_str());
    Py_SetPythonHome(s_python_home);

    // 将 Python 安装目录的 DLL 路径注入到进程 PATH
    QStringList dllDirs = {
        pyHome,
        pyHome + "/DLLs",
    };
    QByteArray curPath = qgetenv("PATH");
    QString newPath = dllDirs.join(";") + ";" + QString::fromLocal8Bit(curPath);
    qputenv("PATH", newPath.toLocal8Bit());
#endif
}

void PythonManager::addDllDirectories()
{
    // 自动扫描 site-packages 下所有包含 DLL 的目录并注册到系统搜索路径
    // 这样任意安装新的 C 扩展库（numpy/scipy/torch 等）都无需额外配置
    py::exec(R"PY(
import sys, os, glob

py_home = sys.prefix
if not py_home or not os.path.isdir(py_home):
    raise RuntimeError("sys.prefix is not set or does not exist")

sp = os.path.join(py_home, 'Lib', 'site-packages')
if not os.path.isdir(sp):
    raise RuntimeError("site-packages not found: " + sp)

dll_dirs = set()

# 1. Python 自身的基础目录
for d in [py_home, os.path.join(py_home, 'DLLs'), os.path.join(py_home, 'Library', 'bin')]:
    if os.path.isdir(d):
        dll_dirs.add(d)

# 2. 自动扫描 site-packages 下所有 .libs 目录
#    pip 安装的带 C 扩展的包通常把 DLL 放在:
#    - site-packages/<pkg>/.libs/
#    - site-packages/<pkg>.libs/
#    - site-packages/<pkg>/core/  等
for pattern in [
    os.path.join(sp, '*', '.libs'),
    os.path.join(sp, '*.libs'),
]:
    for d in glob.glob(pattern):
        if os.path.isdir(d):
            dll_dirs.add(d)

# 3. 注入到系统 DLL 搜索路径
for d in dll_dirs:
    if hasattr(os, 'add_dll_directory'):
        try:
            os.add_dll_directory(d)
        except OSError:
            pass

# 同时注入到 PATH 环境变量作为兜底
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

    QString appDir = QCoreApplication::applicationDirPath();
    QStringList searchDirs = {
        appDir + "/scripts",
        appDir + "/python",
    };

    for (const auto& dir : searchDirs) {
        if (QDir(dir).exists()) {
            paths.append(dir.toStdString());
        }
    }
}

} // namespace ct
