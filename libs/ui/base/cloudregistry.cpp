#include "cloudregistry.h"

#include <QTreeWidgetItem>

namespace ct
{

CloudRegistry::CloudRegistry(QObject* parent)
    : QObject(parent)
{}

// ================================================================
// Cloud registration
// ================================================================

void CloudRegistry::registerCloud(QTreeWidgetItem* item, const Cloud::Ptr& cloud)
{
    if (!item || !cloud) return;
    m_cloud_map[item] = cloud;
    m_item_by_id[QString::fromStdString(cloud->id())] = item;
}

void CloudRegistry::unregisterCloud(QTreeWidgetItem* item)
{
    if (!item) return;
    Cloud::Ptr cloud = m_cloud_map.value(item);
    if (cloud) {
        m_item_by_id.remove(QString::fromStdString(cloud->id()));
    }
    m_cloud_map.remove(item);
}

void CloudRegistry::unregisterCloudById(const QString& id)
{
    QTreeWidgetItem* item = m_item_by_id.value(id, nullptr);
    if (item) {
        m_cloud_map.remove(item);
    }
    m_item_by_id.remove(id);
}

Cloud::Ptr CloudRegistry::getCloud(QTreeWidgetItem* item) const
{
    return m_cloud_map.value(item, nullptr);
}

QTreeWidgetItem* CloudRegistry::getItemById(const QString& id) const
{
    return m_item_by_id.value(id, nullptr);
}

std::vector<Cloud::Ptr> CloudRegistry::getAllClouds() const
{
    std::vector<Cloud::Ptr> result;
    for (auto it = m_cloud_map.constBegin(); it != m_cloud_map.constEnd(); ++it) {
        if (it.value()) result.push_back(it.value());
    }
    return result;
}

std::vector<Cloud::Ptr> CloudRegistry::getCloudsByItems(const QList<QTreeWidgetItem*>& items) const
{
    std::vector<Cloud::Ptr> result;
    for (QTreeWidgetItem* item : items) {
        Cloud::Ptr cloud = m_cloud_map.value(item, nullptr);
        if (cloud) result.push_back(cloud);
    }
    return result;
}

QList<QTreeWidgetItem*> CloudRegistry::getAllItems() const
{
    return m_cloud_map.keys();
}

void CloudRegistry::updateItemId(const QString& oldId, const QString& newId, QTreeWidgetItem* item)
{
    m_item_by_id.remove(oldId);
    m_item_by_id[newId] = item;
}

void CloudRegistry::clear()
{
    m_cloud_map.clear();
    m_item_by_id.clear();
    m_mesh_map.clear();
    m_textured_mesh_map.clear();
    m_shape_map.clear();
    m_cloud_polyline_map.clear();
}

// ================================================================
// PolygonMesh registration
// ================================================================

void CloudRegistry::registerMesh(const QString& cloudId, const pcl::PolygonMesh::Ptr& mesh)
{
    if (cloudId.isEmpty()) return;
    m_mesh_map[cloudId] = mesh;
}

void CloudRegistry::unregisterMesh(const QString& cloudId)
{
    m_mesh_map.remove(cloudId);
}

pcl::PolygonMesh::Ptr CloudRegistry::getMesh(const QString& cloudId) const
{
    return m_mesh_map.value(cloudId, nullptr);
}

bool CloudRegistry::hasMesh(const QString& cloudId) const
{
    return m_mesh_map.contains(cloudId);
}

QList<QPair<QString, pcl::PolygonMesh::Ptr>> CloudRegistry::getLoadedMeshes() const
{
    QList<QPair<QString, pcl::PolygonMesh::Ptr>> result;
    for (auto it = m_mesh_map.constBegin(); it != m_mesh_map.constEnd(); ++it) {
        result.append(qMakePair(it.key(), it.value()));
    }
    return result;
}

// ================================================================
// TexturedMesh registration
// ================================================================

void CloudRegistry::registerTexturedMesh(const QString& cloudId, const TexturedMeshPtr& mesh)
{
    if (cloudId.isEmpty()) return;
    m_textured_mesh_map[cloudId] = mesh;
}

void CloudRegistry::unregisterTexturedMesh(const QString& cloudId)
{
    m_textured_mesh_map.remove(cloudId);
}

TexturedMeshPtr CloudRegistry::getTexturedMesh(const QString& cloudId) const
{
    return m_textured_mesh_map.value(cloudId, nullptr);
}

// ================================================================
// Shape / Polyline registration
// ================================================================

void CloudRegistry::registerShape(const QString& shapeId, const pcl::PolygonMesh::Ptr& mesh)
{
    if (!shapeId.isEmpty() && mesh)
        m_shape_map[shapeId] = mesh;
}

pcl::PolygonMesh::Ptr CloudRegistry::getShape(const QString& shapeId) const
{
    return m_shape_map.value(shapeId, nullptr);
}

void CloudRegistry::unregisterShape(const QString& shapeId)
{
    m_shape_map.remove(shapeId);
}

void CloudRegistry::registerCloudPolyline(const QString& shapeId, const Cloud::Ptr& cloud)
{
    if (!shapeId.isEmpty() && cloud)
        m_cloud_polyline_map[shapeId] = cloud;
}

Cloud::Ptr CloudRegistry::getCloudPolyline(const QString& shapeId) const
{
    return m_cloud_polyline_map.value(shapeId, nullptr);
}

void CloudRegistry::unregisterCloudPolyline(const QString& shapeId)
{
    m_cloud_polyline_map.remove(shapeId);
}

// ================================================================
// In-use protection
// ================================================================

void CloudRegistry::markInUse(const QString& id)
{
    m_clouds_in_use.insert(id);
}

void CloudRegistry::unmarkInUse(const QString& id)
{
    m_clouds_in_use.remove(id);
}

void CloudRegistry::releaseAllInUse()
{
    m_clouds_in_use.clear();
}

bool CloudRegistry::isCloudInUse(const QString& id) const
{
    return m_clouds_in_use.contains(id);
}

} // namespace ct
