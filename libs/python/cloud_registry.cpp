#include "cloud_registry.h"
#include <algorithm>

namespace pw
{

void PythonCloudRegistry::registerCloud(Cloud::Ptr cloud)
{
    if (!cloud) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cloud_registry[cloud->id()] = cloud;
    if (m_script_mode.load(std::memory_order_relaxed))
        m_script_generated_ids.insert(cloud->id());
}

void PythonCloudRegistry::unregisterCloud(const std::string& id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cloud_registry.erase(id);
    m_held_clouds.erase(
        std::remove_if(m_held_clouds.begin(), m_held_clouds.end(),
            [&id](const Cloud::Ptr& c) { return c && c->id() == id; }),
        m_held_clouds.end());
}

Cloud::Ptr PythonCloudRegistry::getCloud(const std::string& name) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_cloud_registry.find(name);
    if (it == m_cloud_registry.end()) return nullptr;
    return it->second;
}

std::vector<std::string> PythonCloudRegistry::getCloudNames() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<std::string> names;
    names.reserve(m_cloud_registry.size());
    for (const auto& pair : m_cloud_registry)
        names.push_back(pair.first);
    return names;
}

// ====================================================================
// Reference Holding
// ====================================================================

void PythonCloudRegistry::holdCloud(Cloud::Ptr cloud)
{
    if (!cloud) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_held_clouds.push_back(cloud);
}

void PythonCloudRegistry::releaseAllHeld()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_held_clouds.clear();
}

// ====================================================================
// Script Data Tracking
// ====================================================================

void PythonCloudRegistry::markScriptGenerated(const std::string& id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_script_generated_ids.insert(id);
}

void PythonCloudRegistry::markSceneMounted(const std::string& id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_script_generated_ids.erase(id);
}

bool PythonCloudRegistry::isScriptGenerated(const std::string& id) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_script_generated_ids.count(id) > 0;
}

std::vector<std::string> PythonCloudRegistry::clearScriptData()
{
    std::vector<std::string> to_remove;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (const auto& id : m_script_generated_ids)
            to_remove.push_back(id);
        for (const auto& id : to_remove) {
            m_cloud_registry.erase(id);
            m_held_clouds.erase(
                std::remove_if(m_held_clouds.begin(), m_held_clouds.end(),
                    [&id](const Cloud::Ptr& c) { return c && c->id() == id; }),
                m_held_clouds.end());
        }
        m_script_generated_ids.clear();
    }
    return to_remove;
}

std::vector<std::string> PythonCloudRegistry::clearAll()
{
    std::vector<std::string> all_ids;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        all_ids.reserve(m_cloud_registry.size());
        for (const auto& pair : m_cloud_registry)
            all_ids.push_back(pair.first);
        m_cloud_registry.clear();
        m_held_clouds.clear();
        m_script_generated_ids.clear();
    }
    return all_ids;
}

// ====================================================================
// Script Mode
// ====================================================================

void PythonCloudRegistry::setScriptMode(bool enabled)
{
    m_script_mode.store(enabled, std::memory_order_relaxed);
}

bool PythonCloudRegistry::isScriptMode() const
{
    return m_script_mode.load(std::memory_order_relaxed);
}

} // namespace pw
