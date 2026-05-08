#include "python_connections.h"

#include "python/python_bridge.h"
#include "viz/cloudview.h"
#include "ui/base/cloudtree.h"
#include "viz/console.h"

#include <pcl/conversions.h>
#include <algorithm>
#include <QObject>

namespace pw {

void connectPythonSignals(
    PythonBridge* bridge,
    CloudView* cloudview,
    CloudTree* cloudtree,
    Console* console)
{
    // ================================================================
    // 渲染缓存
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalCloudChanged,
            cloudview, &pw::CloudView::invalidateCloudRender,
            Qt::QueuedConnection);

    // ================================================================
    // 注册表同步
    // ================================================================
    QObject::connect(cloudtree, &pw::CloudTree::cloudInserted,
            bridge, &pw::PythonBridge::registerCloud, Qt::QueuedConnection);
    QObject::connect(cloudtree, &pw::CloudTree::removedCloudId,
            bridge, &pw::PythonBridge::unregisterCloud, Qt::QueuedConnection);

    // ================================================================
    // In-use 删除保护
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalMarkCloudInUse,
            cloudtree, &pw::CloudTree::markCloudInUse, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalUnmarkCloudInUse,
            cloudtree, &pw::CloudTree::unmarkCloudInUse, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalReleaseAllInUse,
            cloudtree, &pw::CloudTree::releaseAllInUse, Qt::QueuedConnection);

    // 脚本执行完毕后重置脚本模式
    QObject::connect(bridge, &pw::PythonBridge::signalReleaseAllInUse,
            cloudtree, [cloudtree]() { cloudtree->setScriptMode(false); },
            Qt::QueuedConnection);

    // ================================================================
    // 日志 → Console
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalLog,
            console, [console](int level, const QString &msg) {
                console->print(static_cast<pw::log_level>(level), msg);
            }, Qt::QueuedConnection);

    // ================================================================
    // 进度 → CloudTree
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalShowProgress,
            cloudtree, &pw::CloudTree::showProgress, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetProgress,
            cloudtree, &pw::CloudTree::setProgress, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalCloseProgress,
            cloudtree, &pw::CloudTree::closeProgress, Qt::QueuedConnection);

    // ================================================================
    // 视图控制 → CloudView
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalRefreshView,
            cloudview, [cloudview]() { cloudview->refresh(); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalResetCamera,
            cloudview, [cloudview]() { cloudview->resetCamera(); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalZoomToBounds,
            cloudtree, &pw::CloudTree::zoomToSelected, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetAutoRender,
            cloudview, [cloudview](bool en) { cloudview->setAutoRender(en); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalZoomToSelected,
            cloudtree, &pw::CloudTree::zoomToSelected, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetTopView,
            cloudview, [cloudview]() { cloudview->setTopView(); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetFrontView,
            cloudview, [cloudview]() { cloudview->setFrontView(); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetBackView,
            cloudview, [cloudview]() { cloudview->setBackView(); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetLeftSideView,
            cloudview, [cloudview]() { cloudview->setLeftSideView(); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetRightSideView,
            cloudview, [cloudview]() { cloudview->setRightSideView(); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetBottomView,
            cloudview, [cloudview]() { cloudview->setBottomView(); },
            Qt::QueuedConnection);

    // ================================================================
    // 点云外观 → CloudView
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalSetPointSize,
            cloudview, [cloudview](const QString &id, float s) {
                cloudview->setPointCloudSize(id, s);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetOpacity,
            cloudview, [cloudview](const QString &id, float v) {
                cloudview->setPointCloudOpacity(id, v);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetCloudColorRGB,
            cloudview, [cloudview](const QString &id, float r, float g, float b) {
                cloudview->setPointCloudColor(id, pw::fromFloatRGB(r, g, b));
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetCloudColorByAxis,
            cloudview, [cloudview, bridge](const QString &id, const QString &axis) {
                auto cloud = bridge->getCloud(id);
                if (cloud) cloudview->setPointCloudColor(cloud, axis);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalResetCloudColor,
            cloudview, [cloudview, bridge](const QString &id) {
                auto cloud = bridge->getCloud(id);
                if (cloud) cloudview->resetPointCloudColor(cloud);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetCloudVisibility,
            cloudview, [cloudview](const QString &id, bool v) {
                cloudview->setPointCloudVisibility(id, v);
            }, Qt::QueuedConnection);

    // ================================================================
    // 场景外观 → CloudView
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalSetBackgroundColor,
            cloudview, [cloudview](float r, float g, float b) {
                cloudview->setBackgroundColor(pw::fromFloatRGB(r, g, b));
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalResetBackgroundColor,
            cloudview, [cloudview]() { cloudview->resetBackgroundColor(); },
            Qt::QueuedConnection);

    // ================================================================
    // 显示开关 → CloudView
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalShowId,
            cloudview, [cloudview](bool en) { cloudview->setShowId(en); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalShowAxes,
            cloudview, [cloudview](bool en) { cloudview->setShowAxes(en); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalShowFPS,
            cloudview, [cloudview](bool en) { cloudview->setShowFPS(en); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalShowInfo,
            cloudview, [cloudview](const QString &text) {
                cloudview->showInfo(text, 1);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalClearInfo,
            cloudview, [cloudview]() { cloudview->clearInfo(); },
            Qt::QueuedConnection);

    // ================================================================
    // 叠加物 → CloudView
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalAddCube,
            cloudview, [cloudview](float cx, float cy, float cz, float size, const QString &id) {
                pw::Box box;
                box.translation = Eigen::Vector3f(cx, cy, cz);
                box.width = box.height = box.depth = size;
                cloudview->addCube(box, id);
            }, Qt::QueuedConnection);

    QObject::connect(bridge, &pw::PythonBridge::signalAdd3DLabel,
            cloudview, [cloudview](const QString& text, float x, float y, float z, const QString& id) {
        pw::PointXYZRGBN pos;
        pos.x = x; pos.y = y; pos.z = z;
        cloudview->add3DLabel(pos, text, id);
    }, Qt::QueuedConnection);

    QObject::connect(bridge, &pw::PythonBridge::signalRemoveShape,
            cloudview, [cloudview](const QString &id) { cloudview->removeShape(id); },
            Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalRemoveAllShapes,
            cloudview, [cloudview]() { cloudview->removeAllShapes(); },
            Qt::QueuedConnection);

    // ================================================================
    // 高级叠加物 + 视图控制
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalAddArrow,
            cloudview, [cloudview](float x1, float y1, float z1, float x2, float y2, float z2,
                                  const QString &id, float r, float g, float b) {
                pw::PointXYZRGBN pt1, pt2;
                pt1.x = x1; pt1.y = y1; pt1.z = z1;
                pt2.x = x2; pt2.y = y2; pt2.z = z2;
                cloudview->addArrow(pt1, pt2, id, false, pw::fromFloatRGB(r, g, b));
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalAddPolygon,
            cloudview, [cloudview, cloudtree](const QString &cloud_id, const QString &id, float r, float g, float b) {
                auto *item = cloudtree->getItemById(cloud_id);
                if (item) {
                    auto cloud = cloudtree->getCloud(item);
                    if (cloud) {
                        cloudview->addPolygon(cloud, id, pw::fromFloatRGB(r, g, b));
                    }
                }
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetShapeColor,
            cloudview, [cloudview](const QString &id, float r, float g, float b) {
                cloudview->setShapeColor(id, pw::fromFloatRGB(r, g, b));
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetShapeSize,
            cloudview, [cloudview](const QString &id, float size) {
                cloudview->setShapeSize(id, size);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetShapeOpacity,
            cloudview, [cloudview](const QString &id, float value) {
                cloudview->setShapeOpacity(id, value);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetShapeLineWidth,
            cloudview, [cloudview](const QString &id, float value) {
                cloudview->setShapeLineWidth(id, value);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetShapeFontSize,
            cloudview, [cloudview](const QString &id, float value) {
                cloudview->setShapeFontSize(id, value);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetShapeRepresentation,
            cloudview, [cloudview](const QString &id, int type) {
                cloudview->setShapeRepersentation(id, type);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalZoomToBoundsXYZ,
            cloudview, [cloudview](float min_x, float min_y, float min_z,
                                  float max_x, float max_y, float max_z) {
                cloudview->zoomToBounds(
                        Eigen::Vector3f(min_x, min_y, min_z),
                        Eigen::Vector3f(max_x, max_y, max_z));
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalInvalidateCloudRender,
            cloudview, [cloudview](const QString &id) {
                cloudview->invalidateCloudRender(id);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSetInteractorEnable,
            cloudview, [cloudview](bool enable) {
                cloudview->setInteractorEnable(enable);
            }, Qt::QueuedConnection);

    // ================================================================
    // 点云管理 → CloudTree
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalInsertCloud,
            cloudtree, [cloudtree, cloudview](pw::Cloud::Ptr cloud) {
                cloudtree->insertCloud(cloud);
                cloudview->addPointCloud(cloud);
                cloudview->refresh();
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalRemoveSelectedClouds,
            cloudtree, &pw::CloudTree::removeSelectedClouds, Qt::QueuedConnection);

    // ---- 按名称操作 ----
    QObject::connect(bridge, &pw::PythonBridge::signalRemoveCloud,
            cloudtree, [cloudtree](const QString &id) {
                auto *item = cloudtree->getItemById(id);
                if (item) {
                    for (auto *sel: cloudtree->selectedItems())
                        sel->setSelected(false);
                    cloudtree->setCloudSelected(cloudtree->getCloud(item), true);
                    cloudtree->removeSelectedClouds();
                }
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalRemoveAllClouds,
            cloudtree, &pw::CloudTree::removeAllClouds, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalCloneCloud,
            cloudtree, [cloudtree](const QString &id) {
                auto *item = cloudtree->getItemById(id);
                if (item) {
                    for (auto *sel: cloudtree->selectedItems())
                        sel->setSelected(false);
                    cloudtree->setCloudSelected(cloudtree->getCloud(item), true);
                    cloudtree->cloneSelectedClouds();
                }
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSelectCloud,
            cloudtree, [cloudtree](const QString &id) {
                auto *item = cloudtree->getItemById(id);
                if (item) {
                    for (auto *sel: cloudtree->selectedItems())
                        sel->setSelected(false);
                    cloudtree->setCloudSelected(cloudtree->getCloud(item), true);
                }
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalLoadCloud,
            cloudtree, [cloudtree](const QString &filepath) {
                cloudtree->loadCloudFile(filepath);
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalSaveCloud,
            cloudtree, [cloudtree](const QString &id, const QString &filepath, bool binary) {
                auto *item = cloudtree->getItemById(id);
                if (item) {
                    auto cloud = cloudtree->getCloud(item);
                    if (cloud) {
                        cloudtree->saveCloudFile(cloud, filepath, binary);
                    }
                }
            }, Qt::QueuedConnection);
    QObject::connect(bridge, &pw::PythonBridge::signalMergeClouds,
            cloudtree, [cloudtree](const QStringList &ids) {
                for (auto *sel: cloudtree->selectedItems())
                    sel->setSelected(false);
                for (const auto &id: ids) {
                    auto *item = cloudtree->getItemById(id);
                    if (item) {
                        cloudtree->setCloudSelected(cloudtree->getCloud(item), true);
                    }
                }
                cloudtree->mergeSelectedClouds();
            }, Qt::QueuedConnection);

    // ================================================================
    // 算法结果组
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalAddResultGroup,
            cloudtree,
            [cloudtree, cloudview](const QString &origin_id, const std::vector<pw::Cloud::Ptr> &results, const QString &group_name) {
                auto *origin_item = cloudtree->getItemById(origin_id);
                pw::Cloud::Ptr origin_cloud = origin_item ? cloudtree->getCloud(origin_item) : nullptr;
                if (origin_cloud && !results.empty()) {
                    cloudtree->addResultGroup(origin_cloud, results, group_name);
                    for (const auto &r: results)
                        cloudview->addPointCloud(r);
                    cloudview->refresh();
                }
            }, Qt::QueuedConnection);

    // ================================================================
    // 就地更新点云
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalUpdateCloud,
            cloudtree, [cloudtree, cloudview](const QString &id, const pw::Cloud::Ptr &new_cloud) {
                auto *item = cloudtree->getItemById(id);
                if (item) {
                    auto old_cloud = cloudtree->getCloud(item);
                    if (old_cloud) {
                        cloudtree->updateCloud(old_cloud, new_cloud, false);
                        cloudview->refresh();
                    }
                }
            }, Qt::QueuedConnection);

    // ================================================================
    // 脚本模式：跳过弹窗
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalSetScriptMode,
            cloudtree, &pw::CloudTree::setScriptMode, Qt::QueuedConnection);

    // ================================================================
    // 网格显示
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalAddMesh,
            cloudtree, [cloudtree](const pcl::PolygonMesh::Ptr& mesh, const QString& id) {
                // 如果树中已有该 ID 的节点，直接 attach mesh
                QTreeWidgetItem* existing = cloudtree->getItemById(id);
                if (existing) {
                    cloudtree->registerMesh(id, mesh);
                } else {
                    // 从 mesh 顶点创建占位 Cloud 节点插入树，再 attach mesh
                    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
                    pcl::fromPCLPointCloud2(mesh->cloud, cloud_xyz);
                    std::vector<pcl::PointXYZ> pts(cloud_xyz.begin(), cloud_xyz.end());
                    auto placeholder = std::make_shared<pw::Cloud>();
                    placeholder->setId(id.toStdString());
                    placeholder->addPoints(pts);
                    placeholder->makeAdaptive();
                    placeholder->update();
                    cloudtree->insertCloud(placeholder, nullptr, false,
                                           pw::MountStrategy::Sibling, pw::NodeMesh);
                    cloudtree->registerMesh(id, mesh);
                }
            }, Qt::QueuedConnection);

    QObject::connect(bridge, &pw::PythonBridge::signalRemoveMesh,
            cloudtree, [cloudtree](const QString& id) {
                cloudtree->unregisterMesh(id);
            }, Qt::QueuedConnection);

    // ================================================================
    // 手动清理 / 脚本模式关闭清理
    // ================================================================
    QObject::connect(bridge, &pw::PythonBridge::signalClearAll,
            cloudtree, [cloudtree, cloudview]() {
                cloudtree->removeAllClouds();
                cloudview->refresh();
            }, Qt::QueuedConnection);

    QObject::connect(bridge, &pw::PythonBridge::signalClearScriptSession,
            cloudtree, [cloudtree, cloudview]() {
                cloudview->refresh();
            }, Qt::QueuedConnection);

    QObject::connect(bridge, &pw::PythonBridge::signalClearScriptData,
            cloudtree, [cloudtree, cloudview](const QStringList& ids) {
                for (const auto& id : ids) {
                    auto *item = cloudtree->getItemById(id);
                    if (item) {
                        for (auto *sel: cloudtree->selectedItems())
                            sel->setSelected(false);
                        cloudtree->setCloudSelected(cloudtree->getCloud(item), true);
                        cloudtree->removeSelectedClouds();
                    }
                }
                cloudview->refresh();
            }, Qt::QueuedConnection);
}

} // namespace pw
