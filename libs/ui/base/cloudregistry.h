#ifndef POINTWORKS_CLOUDREGISTRY_H
#define POINTWORKS_CLOUDREGISTRY_H

#include "core/cloud.h"
#include "core/textured_mesh.h"

#include <QObject>
#include <QMap>
#include <QSet>
#include <QList>
#include <QPair>

#include <pcl/PolygonMesh.h>

class QTreeWidgetItem;

namespace pw
{

class CloudRegistry : public QObject
{
    Q_OBJECT
public:
    explicit CloudRegistry(QObject* parent = nullptr);

    // ================================================================
    // Cloud registration
    // ================================================================
    void registerCloud(QTreeWidgetItem* item, const Cloud::Ptr& cloud);
    void unregisterCloud(QTreeWidgetItem* item);
    void unregisterCloudById(const QString& id);
    Cloud::Ptr getCloud(QTreeWidgetItem* item) const;
    QTreeWidgetItem* getItemById(const QString& id) const;
    std::vector<Cloud::Ptr> getAllClouds() const;
    std::vector<Cloud::Ptr> getCloudsByItems(const QList<QTreeWidgetItem*>& items) const;
    QList<QTreeWidgetItem*> getAllItems() const;

    void updateItemId(const QString& oldId, const QString& newId, QTreeWidgetItem* item);
    void clear();

    // ================================================================
    // PolygonMesh registration
    // ================================================================
    void registerMesh(const QString& cloudId, const pcl::PolygonMesh::Ptr& mesh);
    void unregisterMesh(const QString& cloudId);
    pcl::PolygonMesh::Ptr getMesh(const QString& cloudId) const;
    bool hasMesh(const QString& cloudId) const;
    QList<QPair<QString, pcl::PolygonMesh::Ptr>> getLoadedMeshes() const;

    // ================================================================
    // TexturedMesh registration
    // ================================================================
    void registerTexturedMesh(const QString& cloudId, const TexturedMeshPtr& mesh);
    void unregisterTexturedMesh(const QString& cloudId);
    TexturedMeshPtr getTexturedMesh(const QString& cloudId) const;

    // ================================================================
    // Shape / Polyline registration
    // ================================================================
    void registerShape(const QString& shapeId, const pcl::PolygonMesh::Ptr& mesh);
    void unregisterShape(const QString& shapeId);
    pcl::PolygonMesh::Ptr getShape(const QString& shapeId) const;

    void registerCloudPolyline(const QString& shapeId, const Cloud::Ptr& cloud);
    void unregisterCloudPolyline(const QString& shapeId);
    Cloud::Ptr getCloudPolyline(const QString& shapeId) const;

    // ================================================================
    // In-use protection (script delete guard)
    // ================================================================
    void markInUse(const QString& id);
    void unmarkInUse(const QString& id);
    void releaseAllInUse();
    bool isCloudInUse(const QString& id) const;
    const QSet<QString>& cloudsInUse() const { return m_clouds_in_use; }

private:
    QMap<QTreeWidgetItem*, Cloud::Ptr> m_cloud_map;
    QMap<QString, QTreeWidgetItem*> m_item_by_id;
    QMap<QString, pcl::PolygonMesh::Ptr> m_mesh_map;
    QMap<QString, TexturedMeshPtr> m_textured_mesh_map;
    QMap<QString, pcl::PolygonMesh::Ptr> m_shape_map;
    QMap<QString, Cloud::Ptr> m_cloud_polyline_map;
    QSet<QString> m_clouds_in_use;
};

} // namespace pw

#endif // POINTWORKS_CLOUDREGISTRY_H
