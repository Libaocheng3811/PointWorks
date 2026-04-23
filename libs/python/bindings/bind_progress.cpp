#include "bind_progress.h"
#include "python_bridge.h"

void registerProgressBindings(py::module_& m)
{
    m.def("show_progress", [](const std::string& title) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->showProgress(QString::fromStdString(title));
    }, py::arg("title"), "Show a progress dialog");

    m.def("set_progress", [](int percent) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setProgress(percent);
    }, py::arg("percent"), "Update progress (0-100)");

    m.def("close_progress", []() {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->closeProgress();
    }, "Close the progress dialog");

    m.def("set_script_mode", [](bool enabled) {
        auto* bridge = ct::PythonManager::instance().bridge();
        if (bridge) bridge->setScriptMode(enabled);
    }, py::arg("enabled"),
       "Enable script mode: skip file dialogs (field mapping, global shift), use defaults automatically");
}
