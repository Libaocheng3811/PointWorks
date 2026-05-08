#ifndef POINTWORKS_PYTHON_CONSOLE_H
#define POINTWORKS_PYTHON_CONSOLE_H

#include <QWidget>
#include <QTextBrowser>
#include <QLineEdit>
#include <QToolBar>
#include <QStringList>

namespace pw
{

class PythonConsole : public QWidget
{
    Q_OBJECT

public:
    explicit PythonConsole(QWidget* parent = nullptr);

private slots:
    void onRunCommand();
    void onClearConsole();
    void onScriptStarted();
    void onScriptFinished(bool ok, QString error);
    void appendStdout(QString text);
    void appendStderr(QString text);

private:
    QTextBrowser* m_output;
    QLineEdit*    m_input;
    QToolBar*     m_toolbar;
    QStringList   m_history;
    int           m_history_index = -1;
    bool          m_busy = false;

    void executeCode(const QString& code);
    void appendToOutput(const QString& text, const QColor& color);
    bool eventFilter(QObject* obj, QEvent* event) override;
};

} // namespace pw

#endif // POINTWORKS_PYTHON_CONSOLE_H
