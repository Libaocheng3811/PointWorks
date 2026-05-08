#ifndef POINTWORKS_PYTHON_WORKER_H
#define POINTWORKS_PYTHON_WORKER_H

#include <QThread>
#include <QString>
#include <atomic>

namespace pw
{

class PythonBridge;

class PythonWorker : public QThread
{
    Q_OBJECT

public:
    explicit PythonWorker(PythonBridge* bridge, QObject* parent = nullptr);

    /// 设置要执行的脚本内容
    void execScript(const QString& script, const QString& filename = "");

    /// 设置要执行的脚本文件路径
    void execFile(const QString& filepath);

    /// 强制取消执行（向 Python 线程注入 KeyboardInterrupt）
    void cancel();

    bool isBusy() const { return m_busy.load(); }

signals:
    void scriptStarted();
    void scriptFinished(bool success, QString error);

protected:
    void run() override;

private:
    void executePythonCode();

    PythonBridge* m_bridge;
    QString m_script;
    QString m_filename;
    std::atomic<bool> m_cancel_flag{false};
    std::atomic<bool> m_busy{false};
    std::atomic<unsigned long> m_py_thread_id{0};
    bool m_is_file{false};
};

} // namespace pw

#endif // POINTWORKS_PYTHON_WORKER_H
