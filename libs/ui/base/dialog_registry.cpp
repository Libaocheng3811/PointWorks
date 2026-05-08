#include "dialog_registry.h"
#include "customdialog.h"
#include "customdock.h"

namespace pw
{

DialogRegistry& DialogRegistry::instance()
{
    static DialogRegistry reg;
    return reg;
}

// --- 对话框 ---

void DialogRegistry::registerDialog(const QString& name, CustomDialog* dlg)
{
    m_dialogs[name] = dlg;
    m_dialog_visibility[name] = (dlg != nullptr);
}

void DialogRegistry::unregisterDialog(const QString& name)
{
    m_dialogs.erase(name);
    m_dialog_visibility.erase(name);
}

CustomDialog* DialogRegistry::getDialog(const QString& name)
{
    auto it = m_dialogs.find(name);
    if (it == m_dialogs.end()) return nullptr;
    return it->second;
}

bool DialogRegistry::isDialogVisible(const QString& name) const
{
    auto it = m_dialog_visibility.find(name);
    if (it == m_dialog_visibility.end()) return false;
    return it->second;
}

bool DialogRegistry::hasOpenDialog() const
{
    for (auto& pair : m_dialogs) {
        if (pair.second != nullptr) return true;
    }
    return false;
}

// --- 停靠栏 ---

void DialogRegistry::registerDock(const QString& name, CustomDock* dock)
{
    m_docks[name] = dock;
}

void DialogRegistry::unregisterDock(const QString& name)
{
    m_docks.erase(name);
    m_dock_visibility.erase(name);
}

void DialogRegistry::setDockVisible(const QString& name, bool visible)
{
    m_dock_visibility[name] = visible;
}

CustomDock* DialogRegistry::getDock(const QString& name)
{
    auto it = m_docks.find(name);
    if (it == m_docks.end()) return nullptr;
    return it->second;
}

bool DialogRegistry::isDockVisible(const QString& name) const
{
    auto it = m_dock_visibility.find(name);
    if (it == m_dock_visibility.end()) return false;
    return it->second;
}

// --- 标签 ---

void DialogRegistry::addLeftLabel(const QString& name) { m_left_labels.insert(name); }
void DialogRegistry::addRightLabel(const QString& name) { m_right_labels.insert(name); }
const std::set<QString>& DialogRegistry::leftLabels() const { return m_left_labels; }
const std::set<QString>& DialogRegistry::rightLabels() const { return m_right_labels; }

}
