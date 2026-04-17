#include "python_worker.h"
#include "python_bridge.h"

// Qt 的 <QObject> 定义了 slots 宏，与 Python 的 object.h 冲突
#undef slots
#include <pybind11/embed.h>
#include <pybind11/eval.h>
#include <iostream>

namespace py = pybind11;

namespace ct
{

PythonWorker::PythonWorker(PythonBridge* bridge, QObject* parent)
    : QThread(parent), m_bridge(bridge)
{
}

void PythonWorker::execScript(const QString& script, const QString& filename)
{
    if (m_busy.load()) return;

    m_script = script;
    m_filename = filename.isEmpty() ? "<script>" : filename;
    m_is_file = false;
    m_cancel_flag.store(false);

    start();
}

void PythonWorker::execFile(const QString& filepath)
{
    if (m_busy.load()) return;

    m_filename = filepath;
    m_is_file = true;
    m_cancel_flag.store(false);

    start();
}

void PythonWorker::cancel()
{
    m_cancel_flag.store(true);

    // 向 Worker 的 Python 线程注入 KeyboardInterrupt 异常
    unsigned long tid = m_py_thread_id.load();
    if (tid != 0) {
        PyThreadState_SetAsyncExc(tid, PyExc_KeyboardInterrupt);
    }
}

void PythonWorker::run()
{
    m_busy.store(true);
    m_py_thread_id.store(0);
    emit scriptStarted();

    executePythonCode();

    // 脚本执行完毕，释放所有持有的云引用并通知 UI
    if (m_bridge) {
        m_bridge->releaseAllHeld();
        m_bridge->releaseAllInUse();
    }

    m_busy.store(false);
}

void PythonWorker::executePythonCode()
{
    // 获取 GIL
    PyGILState_STATE gstate = PyGILState_Ensure();

    // 记录当前线程在 Python 解释器中的 ID（用于 cancel 注入异常）
    m_py_thread_id.store(PyThreadState_Get()->thread_id);

    // === 临时重定向 sys.stdout / sys.stderr 到 PythonBridge ===
    py::object orig_stdout, orig_stderr;
    try {
        auto sys = py::module_::import("sys");
        orig_stdout = sys.attr("stdout");
        orig_stderr = sys.attr("stderr");

        // 创建 stdout 回调 → PythonBridge::printStdout
        auto stdout_cb = py::cpp_function([this](const std::string& text) {
            if (m_bridge) m_bridge->printStdout(QString::fromStdString(text));
        });
        // 创建 stderr 回调 → PythonBridge::printStderr
        auto stderr_cb = py::cpp_function([this](const std::string& text) {
            if (m_bridge) m_bridge->printStderr(QString::fromStdString(text));
        });

        // 定义 _OutputRedirect 类（带 flush 的轻量 write-only stream）
        py::exec(R"(
class _OutputRedirect:
    def __init__(self, callback):
        self._cb = callback
    def write(self, text):
        if text:
            self._cb(text)
    def flush(self):
        pass
)", py::globals());

        // 临时替换
        sys.attr("stdout") = py::globals()["_OutputRedirect"](stdout_cb);
        sys.attr("stderr") = py::globals()["_OutputRedirect"](stderr_cb);

    } catch (...) {
        // 重定向失败不阻塞脚本执行
    }

    try {
        if (m_cancel_flag.load()) {
            emit scriptFinished(false, "Script canceled before execution");
            // 恢复原始 stdio
            try {
                auto sys = py::module_::import("sys");
                if (orig_stdout) sys.attr("stdout") = orig_stdout;
                if (orig_stderr) sys.attr("stderr") = orig_stderr;
            } catch (...) {}
            m_py_thread_id.store(0);
            PyGILState_Release(gstate);
            return;
        }

        if (m_is_file) {
            py::eval_file(m_filename.toStdString());
        } else {
            py::exec(m_script.toStdString(), py::globals(), py::dict());
        }

        emit scriptFinished(true, "");

    } catch (py::error_already_set& e) {
        QString error = QString::fromStdString(e.what());
        std::cerr << "[PythonWorker] Error: " << error.toStdString() << std::endl;
        emit scriptFinished(false, error);

    } catch (const std::exception& e) {
        QString error = QString::fromStdString(e.what());
        std::cerr << "[PythonWorker] Error: " << error.toStdString() << std::endl;
        emit scriptFinished(false, error);

    } catch (...) {
        emit scriptFinished(false, "Unknown error during script execution");
    }

    // === 恢复原始 sys.stdout / sys.stderr ===
    try {
        auto sys = py::module_::import("sys");
        if (orig_stdout) sys.attr("stdout") = orig_stdout;
        if (orig_stderr) sys.attr("stderr") = orig_stderr;
    } catch (...) {}

    m_py_thread_id.store(0);
    // 释放 GIL
    PyGILState_Release(gstate);
}

} // namespace ct
