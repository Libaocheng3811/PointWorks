#include "python_bridge.h"
#include <algorithm>

namespace ct
{

// ====================================================================
// Cloud Registry — 主线程调用
// ====================================================================

void PythonBridge::registerCloud(Cloud::Ptr cloud)
{
    if (!cloud) return;
    QMutexLocker locker(&m_cloud_mutex);
    QString id = QString::fromStdString(cloud->id());
    m_cloud_registry[id] = cloud;
    if (m_script_mode)
        m_script_generated_ids.insert(id);
}

void PythonBridge::unregisterCloud(const QString& id)
{
    QMutexLocker locker(&m_cloud_mutex);
    m_cloud_registry.remove(id);
    // 从持有列表中移除该云的引用
    m_held_clouds.erase(
        std::remove_if(m_held_clouds.begin(), m_held_clouds.end(),
            [&id](const Cloud::Ptr& c) { return c && c->id() == id.toStdString(); }),
        m_held_clouds.end());
}

Cloud::Ptr PythonBridge::getCloud(const QString& name) const
{
    QMutexLocker locker(&m_cloud_mutex);
    auto it = m_cloud_registry.find(name);
    if (it == m_cloud_registry.end()) return nullptr;
    return it.value();
}

QStringList PythonBridge::getCloudNames() const
{
    QMutexLocker locker(&m_cloud_mutex);
    return m_cloud_registry.keys();
}

// ====================================================================
// Reference Holding — Worker 线程调用（线程安全）
// ====================================================================

void PythonBridge::holdCloud(Cloud::Ptr cloud)
{
    if (!cloud) return;
    QMutexLocker locker(&m_cloud_mutex);
    m_held_clouds.push_back(cloud);
}
void PythonBridge::releaseAllHeld()
{
    QMutexLocker locker(&m_cloud_mutex);
    m_held_clouds.clear();
}
void PythonBridge::releaseAllInUse()
{
    // 通知主线程清除所有 in-use 标记
    emit signalReleaseAllInUse();
}

void PythonBridge::clearScriptSession()
{
    QStringList ids;
    {
        QMutexLocker locker(&m_cloud_mutex);
        for (auto it = m_cloud_registry.begin(); it != m_cloud_registry.end(); ++it) {
            ids.append(it.key());
        }
        m_cloud_registry.clear();
        m_held_clouds.clear();
        m_script_generated_ids.clear();
    }
    for (const auto& id : ids) {
        emit signalRemoveCloud(id);
    }
    emit signalClearAll();
    m_script_mode = false;
}

void PythonBridge::markScriptGenerated(const QString& id)
{
    QMutexLocker locker(&m_cloud_mutex);
    m_script_generated_ids.insert(id);
}

void PythonBridge::markSceneMounted(const QString& id)
{
    QMutexLocker locker(&m_cloud_mutex);
    m_script_generated_ids.remove(id);
}

void PythonBridge::clearScriptData()
{
    QStringList to_remove;
    {
        QMutexLocker locker(&m_cloud_mutex);
        for (const auto& id : m_script_generated_ids)
            to_remove.append(id);
        for (const auto& id : to_remove) {
            m_cloud_registry.remove(id);
            m_held_clouds.erase(
                std::remove_if(m_held_clouds.begin(), m_held_clouds.end(),
                    [&id](const Cloud::Ptr& c) { return c && c->id() == id.toStdString(); }),
                m_held_clouds.end());
        }
        m_script_generated_ids.clear();
    }
    if (!to_remove.isEmpty())
        emit signalClearScriptData(to_remove);
}

} // namespace ct
