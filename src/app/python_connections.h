#pragma once

class QTabWidget;
class QAction;
class QDockWidget;

namespace pw {
class CloudView;
class CloudTree;
class Console;
class PythonBridge;
}

namespace pw {

/// 将 PythonBridge 的所有信号连接到 UI 组件。
/// 在 MainWindow 构造函数中调用一次即可，bridge 必须非空。
void connectPythonSignals(
    PythonBridge* bridge,
    CloudView* cloudview,
    CloudTree* cloudtree,
    Console* console);

} // namespace pw
