#include "python_bridge.h"

namespace pw
{

PythonBridge::PythonBridge(QObject* parent) : QObject(parent) {}

QStringList PythonBridge::getCloudNames() const
{
    auto names = m_registry.getCloudNames();
    QStringList result;
    result.reserve(static_cast<int>(names.size()));
    for (const auto& name : names)
        result.append(QString::fromStdString(name));
    return result;
}

void PythonBridge::clearScriptSession()
{
    auto ids = m_registry.clearAll();
    for (const auto& id : ids)
        emit signalRemoveCloud(QString::fromStdString(id));
    emit signalClearAll();
    m_registry.setScriptMode(false);
}

void PythonBridge::clearScriptData()
{
    auto to_remove = m_registry.clearScriptData();
    if (!to_remove.empty()) {
        QStringList qids;
        qids.reserve(static_cast<int>(to_remove.size()));
        for (const auto& id : to_remove)
            qids.append(QString::fromStdString(id));
        emit signalClearScriptData(qids);
    }
}

} // namespace pw
