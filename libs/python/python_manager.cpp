#include "python_manager.h"
#include "python_worker.h"

#undef slots
#include <pybind11/embed.h>
#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

namespace py = pybind11;

namespace ct
{

// Python 版本相关常量（升级 Python 时只需改这里）
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

void PythonManager::initialize()
{
    if (m_initialized) return;

    try {
        // 0. 在 Py_Initialize 之前配置 Python Home + DLL 搜索路径
        setupPythonHome();

        // 1. 启动 Python 解释器
        py::initialize_interpreter();

        // 1.5 Py_SetPath() 会将 sys.prefix 设为空字符串，手动恢复
        //     否则后续 addDllDirectories() 中 sys.prefix 检查会失败
        if (m_is_embedded) {
            auto sys = py::module_::import("sys");
            std::string prefix = m_detected_python_home.toStdString();
            sys.attr("prefix") = prefix;
            sys.attr("exec_prefix") = prefix;
            sys.attr("base_prefix") = prefix;
        }

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
        m_init_message = "Python initialized successfully";

    } catch (const py::error_already_set& e) {
        m_init_message = QString("Python init failed: ") + e.what();
        std::cerr << m_init_message.toStdString() << std::endl;
    } catch (const std::exception& e) {
        m_init_message = QString("Python init failed: ") + e.what();
        std::cerr << m_init_message.toStdString() << std::endl;
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

void PythonManager::setCustomPythonHome(const QString& path)
{
    m_custom_python_home = path;
}

void PythonManager::setupPythonHome()
{
#ifdef _WIN32
    QString appDir = QCoreApplication::applicationDirPath();
    QString pyHome;

    // 三级优先级：
    // 1. 用户自定义路径（QSettings，由 main.cpp 在 initialize() 前注入）
    if (!m_custom_python_home.isEmpty() && QDir(m_custom_python_home).exists()) {
        pyHome = m_custom_python_home;
    }
    // 2. exe 旁边的 python/ 目录（打包部署环境，含嵌入式 Python）
    else {
        QString bundledPython = appDir + "/python";
        if (QDir(bundledPython).exists() &&
            (QFile::exists(bundledPython + "/" + PY_DLL) ||
             QFile::exists(bundledPython + "/python.exe"))) {
            pyHome = bundledPython;
        }
    }
    // 3. 编译时 PYTHON_HOME 宏（开发环境 fallback）
    if (pyHome.isEmpty()) {
        pyHome = QString::fromUtf8(PYTHON_HOME);
    }

    // 检测是否为嵌入式 Python（有 python39.zip）
    m_is_embedded = QFile::exists(pyHome + "/" + PY_ZIP);

    if (m_is_embedded) {
        // 嵌入式 Python：用 Py_SetPath() 直接设置模块搜索路径
        // Py_SetPythonHome() + _pth 文件在 encodings 导入之后才生效，会报
        //   "failed to get the Python codec of the filesystem encoding"
        // Py_SetPath() 在 Py_Initialize() 之前立即生效，绕过 _pth 处理
        QStringList pathEntries = {
            pyHome + "/" + PY_ZIP,
            pyHome,
            pyHome + "/Lib",
            pyHome + "/Lib/site-packages",
        };
        std::wstring modulePath = pathEntries.join(";").toStdWString();
        Py_SetPath(modulePath.c_str());
    } else {
        // 完整 Python 安装：用 Py_SetPythonHome()，Python 会自动查找标准库
        wcscpy_s(s_python_home, pyHome.toStdWString().c_str());
        Py_SetPythonHome(s_python_home);
    }
    m_detected_python_home = pyHome;

    // 将 Python 安装目录的 DLL 路径注入到进程 PATH
    QStringList dllDirs = { pyHome };
    if (!m_is_embedded) {
        dllDirs.append(pyHome + "/DLLs");
    }
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
    QString scriptsDir = appDir + "/scripts";

    // 添加 scripts/ 目录本身
    if (QDir(scriptsDir).exists()) {
        paths.append(scriptsDir.toStdString());

        // 递归扫描 scripts/ 下所有子目录，用户可自由组织脚本
        QDirIterator it(scriptsDir, QDir::Dirs | QDir::NoDotAndDotDot,
                         QDirIterator::Subdirectories);
        while (it.hasNext()) {
            paths.append(it.next().toStdString());
        }
    }

    if (QDir(appDir + "/python").exists()) {
        paths.append((appDir + "/python").toStdString());
    }
}

} // namespace ct
