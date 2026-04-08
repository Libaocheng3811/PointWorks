#ifndef POINTWORKS_PYTHON_EDITOR_H
#define POINTWORKS_PYTHON_EDITOR_H

#include <QWidget>
#include <QPlainTextEdit>
#include <QTabWidget>
#include <QToolBar>
#include <QSyntaxHighlighter>
#include <QRegularExpression>
#include <QList>
#include <QAction>

namespace ct
{

// ================================================================
// Python 语法高亮器
// ================================================================
class PythonSyntaxHighlighter : public QSyntaxHighlighter
{
    Q_OBJECT

public:
    explicit PythonSyntaxHighlighter(QTextDocument* parent = nullptr);

protected:
    void highlightBlock(const QString& text) override;

private:
    struct Rule {
        QRegularExpression pattern;
        QTextCharFormat format;
    };
    QList<Rule> m_rules;
};

// ================================================================
// 带行号 + 当前行高亮的代码编辑器
// ================================================================
class LineNumberArea;

class CodeEditor : public QPlainTextEdit
{
    Q_OBJECT

public:
    explicit CodeEditor(QWidget* parent = nullptr);

    void lineNumberAreaPaintEvent(QPaintEvent* event);
    int lineNumberAreaWidth();

protected:
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void updateLineNumberAreaWidth(int newBlockCount);
    void highlightCurrentLine();
    void updateLineNumberArea(const QRect& rect, int dy);

private:
    LineNumberArea* m_line_number_area;
};

// 行号面板 widget
class LineNumberArea : public QWidget
{
    Q_OBJECT

public:
    LineNumberArea(CodeEditor* editor) : QWidget(editor), m_editor(editor) {}

    QSize sizeHint() const override
    {
        return QSize(m_editor->lineNumberAreaWidth(), 0);
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        m_editor->lineNumberAreaPaintEvent(event);
    }

private:
    CodeEditor* m_editor;
};

// ================================================================
// 编辑器选项卡数据
// ================================================================
struct EditorTab {
    CodeEditor* editor = nullptr;
    PythonSyntaxHighlighter* highlighter = nullptr;
    QString filepath;       // 空表示未保存
    bool modified = false;
};

// ================================================================
// PythonEditor 独立编辑器窗口
// ================================================================
class PythonEditor : public QWidget
{
    Q_OBJECT

public:
    explicit PythonEditor(QWidget* parent = nullptr);

    void showEditor();  // 对外保留，供 mainwindow 调用后做 setGeometry/show

private slots:
    void onRun();
    void onStop();
    void onNew();
    void onOpen();
    void onSave();
    void onSaveAs();
    void onTabChanged(int index);
    void onTabCloseRequested(int index);
    void onScriptStarted();
    void onScriptFinished(bool ok, QString error);

private:
    QToolBar*     m_toolbar;
    QTabWidget*   m_tabs;
    QList<EditorTab> m_tab_list;
    QAction*      m_action_run;
    QAction*      m_action_stop;
    bool          m_busy = false;

    EditorTab createTab(const QString& title = "Untitled");
    void closeTab(int index);
    void updateTabTitle(int index);

    void keyPressEvent(QKeyEvent* event) override;
    void closeEvent(QCloseEvent* event) override;
};

} // namespace ct

#endif // POINTWORKS_PYTHON_EDITOR_H
