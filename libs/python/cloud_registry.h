#ifndef POINTWORKS_PYTHON_CLOUD_REGISTRY_H
#define POINTWORKS_PYTHON_CLOUD_REGISTRY_H

#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <atomic>
#include <memory>
#include "core/cloud.h"

namespace ct
{

/// Qt-free 的线程安全 Python 云注册表
///
/// 注意：命名避开 libs/ui/base/cloudregistry.h 中的 CloudRegistry
///
/// 管理脚本创建的点云、引用持有、脚本数据跟踪。
/// 被 PythonBridge 内部使用，同时暴露给纯算法绑定直接访问。
class PythonCloudRegistry
{
public:
    // ================================================================
    // 云注册表
    // ================================================================

    void registerCloud(Cloud::Ptr cloud);
    void unregisterCloud(const std::string& id);
    Cloud::Ptr getCloud(const std::string& name) const;
    std::vector<std::string> getCloudNames() const;

    // ================================================================
    // 引用持有
    // ================================================================

    void holdCloud(Cloud::Ptr cloud);
    void releaseAllHeld();

    // ================================================================
    // 脚本数据跟踪
    // ================================================================

    void markScriptGenerated(const std::string& id);
    void markSceneMounted(const std::string& id);
    bool isScriptGenerated(const std::string& id) const;
    /// 移除并返回待删除的 ID 列表
    std::vector<std::string> clearScriptData();

    /// 清理全部注册表和引用持有，返回所有云 ID
    std::vector<std::string> clearAll();

    // ================================================================
    // 脚本模式
    // ================================================================

    void setScriptMode(bool enabled);
    bool isScriptMode() const;

private:
    mutable std::mutex m_mutex;
    std::unordered_map<std::string, Cloud::Ptr> m_cloud_registry;
    std::vector<Cloud::Ptr> m_held_clouds;
    std::unordered_set<std::string> m_script_generated_ids;
    std::atomic<bool> m_script_mode{false};
};

} // namespace ct

#endif // POINTWORKS_PYTHON_CLOUD_REGISTRY_H
