#include "python_console.h"
#include "python/python_manager.h"
#include "python/python_bridge.h"
#include "python/python_worker.h"

#include <QVBoxLayout>
#include <QKeyEvent>
#include <QAction>
#include <QScrollBar>
#include <QFont>

namespace ct
{

PythonConsole::PythonConsole(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    // === Toolbar ===
    m_toolbar = new QToolBar(this);
    m_toolbar->setIconSize(QSize(20, 20));
    m_toolbar->setMovable(false);

    auto* action_clear = m_toolbar->addAction(QIcon(":/res/icon/edit-clear.svg"), "Clear Console");
    auto* action_run   = m_toolbar->addAction(QIcon(":/res/icon/media-playback-start.svg"), "Run Command");

    connect(action_clear, &QAction::triggered, this, &PythonConsole::onClearConsole);
    connect(action_run,   &QAction::triggered, this, &PythonConsole::onRunCommand);

    layout->addWidget(m_toolbar);

    // === Output area ===
    m_output = new QTextBrowser(this);
    m_output->setReadOnly(true);
    m_output->setOpenExternalLinks(false);
    m_output->setFont(QFont("Consolas", 10));
    layout->addWidget(m_output, 1);

    // === Input area ===
    m_input = new QLineEdit(this);
    m_input->setFont(QFont("Consolas", 10));
    m_input->setPlaceholderText(">>> Enter Python expression...");
    m_input->installEventFilter(this);

    connect(m_input, &QLineEdit::returnPressed, this, &PythonConsole::onRunCommand);
    layout->addWidget(m_input);

    // === 显示 Python 初始化状态 ===
    auto& pm = ct::PythonManager::instance();
    if (pm.isInitialized()) {
        appendToOutput(QString::fromStdString(pm.initMessage()) + "\n", QColor("#52c41a"));
    } else {
        appendToOutput(QString::fromStdString(pm.initMessage()) + "\n", QColor("#f44747"));
    }

    // === Worker & Bridge 信号连接 ===
    auto* worker = ct::PythonManager::instance().worker();
    auto* bridge = ct::PythonManager::instance().bridge();

    if (worker) {
        connect(worker, &PythonWorker::scriptStarted,
                this, &PythonConsole::onScriptStarted);
        connect(worker, &PythonWorker::scriptFinished,
                this, &PythonConsole::onScriptFinished);
    }

    if (bridge) {
        connect(bridge, &PythonBridge::signalPrintStdout,
                this, &PythonConsole::appendStdout, Qt::QueuedConnection);
        connect(bridge, &PythonBridge::signalPrintStderr,
                this, &PythonConsole::appendStderr, Qt::QueuedConnection);
    }

    // 欢迎信息
    appendToOutput("Python Console — PointWorks\n"
                   "Use import ct to access the PointWorks API.\n",
                   QColor("#569cd6"));
}

void PythonConsole::onRunCommand()
{
    QString code = m_input->text().trimmed();
    if (code.isEmpty()) return;

    // 显示输入
    appendToOutput(">>> " + code, QColor("#dcdcaa"));
    m_input->clear();

    // 记录历史
    m_history.append(code);
    m_history_index = m_history.size();

    executeCode(code);
}

void PythonConsole::onClearConsole()
{
    m_output->clear();
}

void PythonConsole::onScriptStarted()
{
    m_busy = true;
    m_input->setEnabled(false);
}

void PythonConsole::onScriptFinished(bool ok, QString error)
{
    m_busy = false;
    m_input->setEnabled(true);
    m_input->setFocus();

    if (!ok && !error.isEmpty()) {
        appendToOutput(error, QColor("#cc0000"));
    }
}

void PythonConsole::appendStdout(QString text)
{
    appendToOutput(text, QColor("#6a9955"));
}

void PythonConsole::appendStderr(QString text)
{
    appendToOutput(text, QColor("#f44747"));
}

void PythonConsole::executeCode(const QString& code)
{
    auto* worker = ct::PythonManager::instance().worker();
    if (!worker) {
        appendToOutput("Error: Python interpreter not available\n", QColor("#f44747"));
        return;
    }

    if (worker->isBusy()) {
        appendToOutput("Error: Another script is running\n", QColor("#f44747"));
        return;
    }

    worker->execScript(code, "<console>");
}

void PythonConsole::appendToOutput(const QString& text, const QColor& color)
{
    QString escaped = text.toHtmlEscaped();
    escaped.replace("\n", "<br>");
    m_output->append(QString("<span style=\"color:%1;\">%2</span>")
                         .arg(color.name(), escaped));

    QScrollBar* sb = m_output->verticalScrollBar();
    sb->setValue(sb->maximum());
}

bool PythonConsole::eventFilter(QObject* obj, QEvent* event)
{
    if (obj == m_input && event->type() == QEvent::KeyPress) {
        auto* key_event = static_cast<QKeyEvent*>(event);
        if (key_event->key() == Qt::Key_Up) {
            if (m_history_index > 0) {
                m_history_index--;
                m_input->setText(m_history[m_history_index]);
            }
            return true;
        }
        if (key_event->key() == Qt::Key_Down) {
            if (m_history_index < m_history.size() - 1) {
                m_history_index++;
                m_input->setText(m_history[m_history_index]);
            } else {
                m_history_index = m_history.size();
                m_input->clear();
            }
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

} // namespace ct
