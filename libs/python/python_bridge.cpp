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
    m_cloud_registry[QString::fromStdString(cloud->id())] = cloud;
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

} // namespace ct
