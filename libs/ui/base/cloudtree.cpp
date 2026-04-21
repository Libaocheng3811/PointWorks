#include "cloudtree.h"

#include "viz/cloudview.h"

#include "core/colormap.h"
#include "widgets/sf_display_panel.h"

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <cfloat>

#include <QCheckBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMenu>
#include <QMessageBox>
#include <QContextMenuEvent>
#include <QMouseEvent>
#include <QPushButton>
#include <QHeaderView>
#include <QSpinBox>
#include <QDropEvent>
#include <QMimeData>
#include <QUrl>
#include <QComboBox>
#include <QApplication>
#include <QTime>
#include <QSizePolicy>

namespace ct
{
    CloudTree::CloudTree(QWidget *parent)
        : CustomTree(parent),
        m_tree_menu(nullptr),
        m_progress(new ProgressManager(this)),
        m_registry(new CloudRegistry(this)),
        m_io(new CloudIOController(this))
    {
        m_io->init(this, m_progress);
        connect(m_io, &CloudIOController::cloudLoaded, this, &CloudTree::handleCloudLoaded);
        connect(m_io, &CloudIOController::texturedMeshLoaded, this, &CloudTree::handleTexturedMeshLoaded);
        connect(m_io, &CloudIOController::saveComplete, this, &CloudTree::handleSaveComplete);
        connect(m_io, &CloudIOController::meshSaveComplete, this, &CloudTree::handleMeshSaveComplete);
        connect(this, &QTreeWidget::itemChanged, this, &CloudTree::itemChangedEvent);
        connect(this, &CloudTree::itemSelectionChanged, this, &CloudTree::itemSelectionChangedEvent);
        this->setAcceptDrops(true);
    }

    CloudTree::~CloudTree()
    {
        m_registry->clear();
    }

    Cloud::Ptr CloudTree::getCloud(QTreeWidgetItem *item) {
        return m_registry->getCloud(item);
    }

    void CloudTree::addCloud()
    {
        m_io->addCloud();
    }

    void CloudTree::loadCloudFile(const QString& filepath)
    {
        m_io->loadCloudFile(filepath);
    }

    void CloudTree::loadCloudFile(const QString& filepath, QTreeWidgetItem* targetParent)
    {
        m_io->loadCloudFile(filepath, targetParent);
    }

    void CloudTree::saveCloudFile(const Cloud::Ptr& cloud, const QString& filepath, bool isBinary)
    {
        m_io->saveCloudFile(cloud, filepath, isBinary);
    }

    QTreeWidgetItem* CloudTree::getItemById(const QString &id) {
        return m_registry->getItemById(id);
    }


    void CloudTree::insertCloud(const Cloud::Ptr& cloud, QTreeWidgetItem* parentItem,
                                 bool selected, MountStrategy strategy, SceneNodeType nodeType)
    {
        // check cloud id
        if (cloud == nullptr) return;

        cloud->update();

        if (m_cloudview->contains(QString::fromStdString(cloud->id())))
        {
            int k = QMessageBox::warning(this, "WARNING", "Rename the exists id?", QMessageBox::Yes, QMessageBox::Cancel);
            if (k == QMessageBox::Yes)
            {
                bool ok = false;
                QString res = QInputDialog::getText(this, "Rename", "", QLineEdit::Normal, QString::fromStdString(cloud->id()), &ok);
                if (ok)
                {
                    if (res == QString::fromStdString(cloud->id()) || m_cloudview->contains(res))
                    {
                        printE(QString("The cloud id[%1] already exists!").arg(res));
                        return;
                    }
                    cloud->setId(res.toStdString());
                }
                else
                {
                    printW("Add cloud canceled.");
                    return;
                }
            }
            else
            {
                printW("Add cloud canceled.");
                return;
            }
        }

        // 根据策略确定实际父节点和节点类型
        QTreeWidgetItem* actualParent = parentItem;
        if (strategy == MountStrategy::Auto)
        {
            if (actualParent == nullptr)
            {
                // 无指定父节点 -> 直接作为根节点（NodeCloud）
                // 注意：loadCloudResult 负责创建 FileNode 再调用 insertCloud
            }
            // actualParent 非 nullptr 时直接作为其子节点
        }
        else if (strategy == MountStrategy::Sibling && actualParent != nullptr)
        {
            // 策略一：挂到 sourceCloud 的父节点下
            actualParent = actualParent->parent();
        }
        else if (strategy == MountStrategy::Child)
        {
            // 策略三：直接挂到指定节点下（不做改动）
        }
        else if (strategy == MountStrategy::Root)
        {
            actualParent = nullptr;
        }

        // 创建节点（阻塞信号避免 addItem 中 setCheckState 触发 itemChangedEvent）
        const bool wasBlocked = this->blockSignals(true);
        QTreeWidgetItem* newItem = addItem(actualParent, QString::fromStdString(cloud->id()), nodeType, false);
        this->blockSignals(wasBlocked);

        m_registry->registerCloud(newItem, cloud);

        // Mesh 节点由 registerMesh 通过 addMeshActor 渲染，不添加点云散点
        if (nodeType != NodeMesh) {
            m_cloudview->addPointCloud(cloud);

            // 根据点云类型决定显示属性
            if (cloud->pointSize() > 1) {
                m_cloudview->setPointCloudSize(QString::fromStdString(cloud->id()), cloud->pointSize());
                if (QString::fromStdString(cloud->id()).contains("picked-"))
                    m_cloudview->setPointCloudColor(cloud, ct::Color::Red);
            }
        }

        if (selected && nodeType != NodeMesh) {
            m_cloudview->addBox(cloud);
        }

        if (!cloud->empty()) {
            Eigen::Vector3f min_pt = cloud->min().getVector3fMap();
            Eigen::Vector3f max_pt = cloud->max().getVector3fMap();
            m_cloudview->zoomToBounds(min_pt, max_pt);
        }

        if (selected){
            this->setCurrentItem(newItem);
        }

        printI(QString("Add %1[id:%2] done.").arg(nodeType == NodeMesh ? "mesh" : "cloud").arg(QString::fromStdString(cloud->id())));

        // 通知 Bridge 注册该云
        emit cloudInserted(cloud);
    }

    void CloudTree::addSiblingCloud(const Cloud::Ptr& sourceCloud, const Cloud::Ptr& resultCloud,
                                     const QString& suffix)
    {
        if (!sourceCloud || !resultCloud) return;

        QTreeWidgetItem* sourceItem = getItemById(QString::fromStdString(sourceCloud->id()));
        if (!sourceItem)
        {
            // fallback: 作为根节点
            insertCloud(resultCloud);
            return;
        }

        QString newName = QString::fromStdString(sourceCloud->id()) + suffix;
        resultCloud->setId(newName.toStdString());

        insertCloud(resultCloud, sourceItem, true, MountStrategy::Sibling);
    }

    void CloudTree::updateCloud(const Cloud::Ptr &cloud, const Cloud::Ptr &new_cloud, bool update_name)
    {
        if (cloud == nullptr || new_cloud == nullptr) return;

        // 由于Cloud类现在不支持swap，我们需要使用移动赋值
        // 如果cloud和new_cloud不是同一个对象，将new_cloud的内容移动到cloud中
        if (cloud != new_cloud) {
            // 使用移动赋值操作符将new_cloud的内容转移到cloud
            cloud->swap(*new_cloud);
        }
        // 更新点云
        cloud->update();

        if (update_name){
            QTreeWidgetItem* item = getItemById(QString::fromStdString(cloud->id()));
            if (item) renameCloudItem(item, QString::fromStdString(new_cloud->id()));
        }
        m_cloudview->addPointCloud(cloud);

        QTreeWidgetItem* item = getItemById(QString::fromStdString(cloud->id()));
        if (item && item->isSelected()){
            m_cloudview->addBox(cloud);
        }
        printI(QString("Update cloud[id:%1, size:%2] to new cloud[id:%3, size:%4] done.")
                .arg(QString::fromStdString(cloud->id())).arg(cloud->size()).arg(QString::fromStdString(new_cloud->id())).arg(new_cloud->size()));
    }

    void CloudTree::removeCloudItem(QTreeWidgetItem *item)
    {
        if (!item) return;

        // 级联删除，在删除父项前先删除其所有子项
        QList<QTreeWidgetItem*> allChildren;
        getAllChildItems(item, allChildren);
        allChildren.append(item);

        // 删除保护：检查是否有被脚本使用的点云
        for (QTreeWidgetItem* c : allChildren){
            Cloud::Ptr cloud = getCloud(c);
            if (cloud && m_registry->isCloudInUse(QString::fromStdString(cloud->id()))) {
                printW(QString("Cloud[%1] is in use by script, cannot delete").arg(QString::fromStdString(cloud->id())));
                return;
            }
        }

        m_cloudview->setAutoRender(false);

        for (QTreeWidgetItem* c : allChildren){
            Cloud::Ptr cloud = getCloud(c);
            if (cloud) {
                QString cid = QString::fromStdString(cloud->id());
                emit removedCloudId(cid);
                m_cloudview->removePointCloud(cid);
                m_cloudview->removeShape(QString::fromStdString(cloud->boxId()));
                m_cloudview->removePointCloud(QString::fromStdString(cloud->normalId()));
                m_registry->unregisterCloud(c);
                // 清理关联的 PolygonMesh / TexturedMesh
                if (m_registry->getTexturedMesh(cid) != nullptr) {
                    unregisterTexturedMesh(cid);
                } else if (m_registry->hasMesh(cid)) {
                    unregisterMesh(cid);
                }
            } else {
                // 清理附属 shape（如边界多段线）
                QString shape_id = c->data(0, NodeShapeIdRole).toString();
                if (!shape_id.isEmpty()) {
                    m_cloudview->removeShape(shape_id);
                    m_registry->unregisterShape(shape_id);
                    m_registry->unregisterCloudPolyline(shape_id);
                }
            }
        }

        m_cloudview->setAutoRender(true);
        m_cloudview->refresh();

        removeItem(item);
        printI(QString("Remove cloud[id:%1] done."));
    }

    void CloudTree::removeAllClouds()
    {
        // 检查是否有任何点云正在被脚本使用
        if (!m_registry->cloudsInUse().isEmpty()) {
            printW(QString("Cannot remove all: %1 cloud(s) in use by script").arg(m_registry->cloudsInUse().size()));
            return;
        }

        m_cloudview->setAutoRender(false);

        // 遍历所有 TopLevel Items 并移除
        while (this->topLevelItemCount() > 0){
            removeCloudItem(this->topLevelItem(0));
        }

        m_cloudview->removeAllPointClouds();
        m_cloudview->removeAllShapes();
        m_registry->clear();

        m_cloudview->setAutoRender(true);
        m_cloudview->refresh();
        printI("remove all clouds done.");
    }

    void CloudTree::saveCloudItem(QTreeWidgetItem *item) {
        Cloud::Ptr cloud = getCloud(item);
        if (!cloud) return; // 如果是文件夹，不保存，返回空

        // 检查节点是否有关联的 mesh
        QString cloudId = QString::fromStdString(cloud->id());
        bool has_mesh = false;
        auto _save_mesh = m_registry->getMesh(cloudId);
        if (_save_mesh && !_save_mesh->polygons.empty()) has_mesh = true;

        // 根据是否有关联 mesh 构建不同的保存过滤器
        QString filter;
        if (has_mesh) {
            filter = "Mesh Files (*.obj *.stl *.vtk);;OBJ (*.obj);;STL (*.stl);;VTK (*.vtk);;"
                     "Point Cloud Files (*.ply *.pcd *.las *.e57 *.txt);;PLY (*.ply);;PCD (*.pcd);;LAS (*.las);;E57 (*.e57);;TXT (*.txt)";
        } else {
            filter = "Point Cloud Files (*.ply *.pcd *.las *.e57 *.txt);;PLY (*.ply);;PCD (*.pcd);;LAS (*.las);;E57 (*.e57);;TXT (*.txt)";
        }

        QString filepath = QFileDialog::getSaveFileName(this, tr("Save file"), QString::fromStdString(cloud->id()), filter);
        if (filepath.isEmpty()) return;

        QFileInfo fi(filepath);
        QString suffix = fi.suffix().toLower();

        // 判断是否走 mesh 保存路径
        // mesh 格式：obj, stl, vtk
        // PLY: 如果有关联 mesh，默认保存为 mesh（含面片）
        bool save_as_mesh = has_mesh && (suffix == "obj" || suffix == "stl" || suffix == "vtk" || suffix == "ply");

        if (save_as_mesh) {
            // Mesh 保存路径（PLY 也会保存面片信息）
            showProgress("Saving Mesh...");
            bindWorker(m_io);
            m_io->saveMeshFile(_save_mesh, filepath);
        } else if (suffix == "e57") {
            // E57 无需选择 binary/ascii（自带压缩）
            showProgress("Saving E57...");
            bindWorker(m_io);
            m_io->saveCloudFile(cloud, filepath, true);
        } else {
            // 点云保存路径（PLY mesh 也走这里，让用户选择 binary/ascii）
            QMessageBox message_box(QMessageBox::NoIcon, "Saved format", tr("Save in binary or ascii format?"),
                                    QMessageBox::NoButton, this);
            message_box.addButton(tr("Ascii"), QMessageBox::ActionRole);
            message_box.addButton(tr("Binary"), QMessageBox::ActionRole)->setDefault(true);
            message_box.addButton(QMessageBox::Cancel);
            int k = message_box.exec();
            if (k == QMessageBox::Cancel)
            {
                printW("Save cloud canceled.");
                return;
            }

            showProgress("Saving Point Cloud...");
            bindWorker(m_io);
            m_io->saveCloudFile(cloud, filepath, k);
        }
    }

    void CloudTree::renameCloudItem(QTreeWidgetItem *item, const QString &name) {
        Cloud::Ptr cloud = getCloud(item);
        if (!cloud) {
            item->setText(0, name);
            return;
        }

        if (m_cloudview->contains(name)){
            printW(QString("Cloud[id:%1] already exists, please rename it.").arg(name));
            return;
        }

        item->setText(0, name);
        // TODO:为什么这里是移除点云？
        m_cloudview->removePointCloud(QString::fromStdString(cloud->id()));
        m_cloudview->removePointCloud(QString::fromStdString(cloud->normalId()));
        m_cloudview->removeShape(QString::fromStdString(cloud->boxId()));

        m_registry->updateItemId(QString::fromStdString(cloud->id()), name, item);
        cloud->setId(name.toStdString());
        printI(QString("Rename done."));
    }

    std::vector<Cloud::Ptr> CloudTree::getSelectedClouds()
    {
        std::vector<Cloud::Ptr> clouds;
        QList<QTreeWidgetItem*> items = this->getSelectedItems();

        for (auto item : items){
            Cloud::Ptr c = getCloud(item);
            if (c) clouds.push_back(c);
        }
        return clouds;
    }

    std::vector<Cloud::Ptr> CloudTree::getAllClouds()
    {
        return m_registry->getAllClouds();
    }

    void CloudTree::removeSelectedClouds() {
        QList<QTreeWidgetItem*> items = getSelectedItems();
        //
        for (auto item : items){
            // 检查是否挂在树上
            if (item->treeWidget() == this) removeCloudItem(item);
        }
    }

    void CloudTree::saveSelectedClouds(){
        QList<QTreeWidgetItem*> items = getSelectedItems();
        for (auto item : items) saveCloudItem(item);
    }

    void CloudTree::mergeSelectedClouds()
    {
        std::vector<Cloud::Ptr> clouds = getSelectedClouds();
        // 如果选择的点云数量小于2，打印警告信息
        if (clouds.size() <= 1)
        {
            printW("The number of clouds to merge is not enough!");
            return;
        }

        // 1. 确定主坐标系 (以第一个点云为基准)
        Eigen::Vector3d master_shift = clouds[0]->getGlobalShift();

        // =========================================================
        // 【智能修正】防止包围盒爆炸
        // =========================================================
        // 检查是否存在极其遥远的点云 (例如 Rabbit 和 城市地图)
        // 如果它们相距超过 100km，我们假设用户并不想保留真实的地理关系，
        // 而是想把它们强行放在一起查看。
        bool force_local_merge = false;
        for (const auto& c : clouds) {
            double dist = (c->getGlobalShift() - master_shift).norm();
            if (dist > 100000.0) { // 100 km 阈值
                force_local_merge = true;
                printW("Detected large distance between clouds. Forcing local merge to avoid huge bounding box.");
                break;
            }
        }

        // 2. 预计算合并后的包围盒 (用于初始化八叉树)
        // 使用 double 防止大坐标溢出
        Eigen::Vector3d target_min(DBL_MAX, DBL_MAX, DBL_MAX);
        Eigen::Vector3d target_max(-DBL_MAX, -DBL_MAX, -DBL_MAX);
        bool has_valid_data = false;

        for (const auto& c : clouds) {
            if (!c || c->empty()) continue;

            // 获取当前点云的本地包围盒
            Eigen::Vector3d c_min = c->min().getVector3fMap().cast<double>();
            Eigen::Vector3d c_max = c->max().getVector3fMap().cast<double>();

            Eigen::Vector3d offset = Eigen::Vector3d::Zero();

            // 只有在保留地理关系时，才计算偏移
            if (!force_local_merge) {
                // 公式：Offset = CurrentShift - MasterShift
                // 解释：把 Current 的点移动到 Master 的坐标系下
                offset = c->getGlobalShift() - master_shift;
            }

            // 预测合并后的位置
            target_min = target_min.cwiseMin(c_min + offset);
            target_max = target_max.cwiseMax(c_max + offset);

            has_valid_data = true;
        }

        if (!has_valid_data) return;

        // 3. 初始化合并点云
        Cloud::Ptr merge_cloud(new Cloud);

        // 如果是地理合并，继承 Master Shift 以保持精度
        merge_cloud->setGlobalShift(master_shift);

        ct::Box box;
        box.width = target_max.x() - target_min.x();
        box.height = target_max.y() - target_min.y();
        box.depth = target_max.z() - target_min.z();

        // 增加 buffer
        box.width *= 1.05; box.height *= 1.05; box.depth *= 1.05;
        if (box.width < 1.0) box.width = 10.0;
        if (box.height < 1.0) box.height = 10.0;
        if (box.depth < 1.0) box.depth = 10.0;

        // 设置中心
        box.translation = (target_min + target_max).cast<float>() * 0.5f;

        // 显式初始化
        merge_cloud->initOctree(box);

        // 启用属性
        bool has_color = false;
        bool has_normal = false;
        for(const auto& c : clouds) {
            if(c->hasColors()) has_color = true;
            if(c->hasNormals()) has_normal = true;
        }
        if(has_color) merge_cloud->enableColors();
        if(has_normal) merge_cloud->enableNormals();

        // 4. 执行合并
        showProgress("Merging clouds...");

        for (auto& c : clouds) {
            if (c->empty()) continue;

            // 计算偏移
            Eigen::Vector3f offset(0,0,0);
            if (!force_local_merge) {
                offset = (c->getGlobalShift() - master_shift).cast<float>();
            }

            bool need_shift = (offset.norm() > 1e-5);

            const auto& blocks = c->getBlocks();
            for (const auto& block : blocks) {
                if (block->empty()) continue;

                // 拷贝数据
                std::vector<ct::PointXYZ> pts = block->m_points;

                // 应用偏移
                if (need_shift) {
                    for (auto& p : pts) {
                        p.x += offset.x();
                        p.y += offset.y();
                        p.z += offset.z();
                    }
                }

                const std::vector<ct::ColorRGB>* colors = (has_color && block->m_colors) ? block->m_colors.get() : nullptr;
                const std::vector<ct::CompressedNormal>* normals = (has_normal && block->m_normals) ? block->m_normals.get() : nullptr;
                const std::unordered_map<std::string, std::vector<float>>* scalars = (!block->m_scalar_fields.empty()) ? &block->m_scalar_fields : nullptr;

                merge_cloud->addPoints(pts, colors, normals, scalars);
            }
        }
        closeProgress();

        merge_cloud->setId(MERGE_ADD_FLAG + clouds.front()->id());
        merge_cloud->setFilepath(clouds.front()->filepath());
        merge_cloud->makeAdaptive();

        QString merge_id = QString::fromStdString(clouds[0]->id()) + QTime::currentTime().toString();
        QTreeWidgetItem* groupItem = addItem(nullptr, "Merged_" + merge_id, NodeGroup);
        // 合并点云默认作为根节点添加
        insertCloud(merge_cloud, groupItem, true);
        printI(QString("Merge clouds to new cloud[id:%1] done.").arg(QString::fromStdString(merge_cloud->id())));
    }

    void CloudTree::cloneSelectedClouds(){
        QList<QTreeWidgetItem*> items = getSelectedItems();
        for (auto item : items) cloneCloudItem(item);
    }

    void CloudTree::cloneCloudItem(QTreeWidgetItem *item){
        Cloud::Ptr cloud = getCloud(item);
        if (!cloud) return;

        Cloud::Ptr clone = cloud->clone();
        clone->setId(CLONE_ADD_FLAG + cloud->id());
        clone->setFilepath(cloud->filepath());

        // 策略一：克隆点云作为兄弟节点挂载
        insertCloud(clone, item->parent(), true, MountStrategy::Sibling);
    }

    void CloudTree::setCloudChecked(const Cloud::Ptr &cloud, bool checked)
    {
        if (!cloud) return;

        QTreeWidgetItem* item = getItemById(QString::fromStdString(cloud->id()));
        if (!item) return;

        if ((checked && item->checkState(0) == Qt::Checked) || (!checked && item->checkState(0) == Qt::Unchecked)){
            return;
        }

        item->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
    }

    void CloudTree::setCloudSelected(const Cloud::Ptr &cloud, bool selected)
    {
        if (!cloud) return;

        QTreeWidgetItem* item = getItemById(QString::fromStdString(cloud->id()));
        if (!item) return;
        item->setSelected(selected);
    }

    void CloudTree::handleCloudLoaded(const Cloud::Ptr& cloud, const pcl::PolygonMesh::Ptr& mesh,
                                       QTreeWidgetItem* targetParent, float time)
    {
        if (!cloud) {
            printE("load the file failed!");
            m_progress->closeProgress();
            return;
        }

        printI(QString("Load the file [path:%1] done, take time %2 ms.").arg(QFileInfo(QString::fromStdString(cloud->filepath())).absoluteFilePath()).arg(time));

        bool isMesh = mesh && !mesh->polygons.empty();
        SceneNodeType nodeType = isMesh ? NodeMesh : NodeCloud;

        if (targetParent) {
            // 恢复模式
            insertCloud(cloud, targetParent, true, MountStrategy::Auto, nodeType);
        } else {
            // 普通加载模式
            QString folderName = QFileInfo(QString::fromStdString(cloud->filepath())).fileName();
            QString fullPath = QFileInfo(QString::fromStdString(cloud->filepath())).absoluteFilePath();
            QTreeWidgetItem* fileNode = addItem(nullptr, folderName + "  (" + fullPath + ")", NodeFile);
            fileNode->setData(0, NodeFilePathRole, fullPath);
            insertCloud(cloud, fileNode, true, MountStrategy::Auto, nodeType);
        }

        if (mesh && !mesh->polygons.empty()) {
            QString cloudId = QString::fromStdString(cloud->id());
            registerMesh(cloudId, mesh);
            printI(QString("Mesh detected: %1 polygons").arg(mesh->polygons.size()));
        }
    }

    void CloudTree::handleSaveComplete(bool success, const QString& path, float time)
    {
        if (!success)
            printE("Save the file failed!");
        else
            printI(QString("Save the file [path:%1] done, take time %2 ms.").arg(path).arg(time));
    }

    void CloudTree::handleMeshSaveComplete(bool success, const QString& path, float time)
    {
        if (!success)
            printE("Save mesh file failed!");
        else
            printI(QString("Save mesh file [path:%1] done, take time %2 ms.").arg(path).arg(time));
    }

    // TODO: 对于文件树的修改，目前只验证了添加点云、选点、CSF和植被滤波功能，其他功能还未验证是否正常
    void CloudTree::itemChangedEvent(QTreeWidgetItem *item, int column){
        if (column != 0) return;

        const bool wasBlocked = this->blockSignals(true);
        bool isChecked = (item->checkState(0) == Qt::Checked);

        // 级联设置子节点勾选状态,向下递归，父->子
        setItemAndChildrenCheckState(item, isChecked ? Qt::Checked : Qt::Unchecked);

        // 向上递归，子->父
        QTreeWidgetItem* parent = item->parent();
        while (parent){
            // 如果父节点是点云节点（非文件夹），则跳过
            if (!isFolderNode(parent)){
                parent = parent->parent();
                continue;
            }

            // 针对纯文件夹节点逻辑
            bool allChecked = true;
            bool anyChecked = false;
            for (int i = 0; i < parent->childCount(); ++i){
                if (parent->child(i)->checkState(0) == Qt::Checked) anyChecked = true;
                else if (parent->child(i)->checkState(0) == Qt::Unchecked) allChecked = false;
                else { anyChecked = true; allChecked = false;}
            }
            if (allChecked) parent->setCheckState(0, Qt::Checked);
            else if (anyChecked) parent->setCheckState(0, Qt::PartiallyChecked);
            else parent->setCheckState(0, Qt::Unchecked);

            parent = parent->parent();
        }

        m_cloudview->setAutoRender(false);

        // 获取所有受影响节点 (自己 + 所有子孙 + 所有祖先)
        QList<QTreeWidgetItem*> affectedItems;
        affectedItems.append(item);
        getAllChildItems(item, affectedItems);

        QTreeWidgetItem* p = item->parent();
        while (p){
            affectedItems.append(p);
            p = p->parent();
        }

        for (QTreeWidgetItem* it : affectedItems){
            Cloud::Ptr cloud = getCloud(it);

            // --- NodeBoundary: 附属 shape 的可见性联动 ---
            if (!cloud) {
                QString shape_id = it->data(0, NodeShapeIdRole).toString();
                if (!shape_id.isEmpty()) {
                    bool show = (it->checkState(0) != Qt::Unchecked);
                    if (show) {
                        if (!m_cloudview->contains(shape_id)) {
                            // 优先从 mesh 提取折线，其次从 cloud 绘制折线
                            auto _shape_mesh = m_registry->getShape(shape_id);
                            if (_shape_mesh) {
                                m_cloudview->addPolylineFromPolygonMesh(_shape_mesh, shape_id);
                            } else {
                                auto _polyline_cloud = m_registry->getCloudPolyline(shape_id);
                                if (_polyline_cloud) {
                                    m_cloudview->addPolylineFromCloud(_polyline_cloud, shape_id);
                                }
                            }
                        }
                    } else {
                        m_cloudview->removeShape(shape_id);
                    }
                }
                continue;
            }

            // 只要不是 Unchecked，就显示 (Checked/Partially)
            SceneNodeType nodeType = getNodeType(it);
            bool shouldShow = (it->checkState(0) != Qt::Unchecked);

            // NodeMesh 节点不显示点云散点，由下方的 mesh 联动逻辑处理
            if (nodeType != NodeMesh) {
                bool exists = m_cloudview->contains(QString::fromStdString(cloud->id()));

                if(shouldShow){
                    if (exists){
                        m_cloudview->setPointCloudVisibility(QString::fromStdString(cloud->id()), true);
                    }
                    else {
                        m_cloudview->addPointCloud(cloud);
                        if (cloud->pointSize() > 1){
                            m_cloudview->setPointCloudSize(QString::fromStdString(cloud->id()), cloud->pointSize());
                            if (QString::fromStdString(cloud->id()).contains("picked-"))
                                m_cloudview->setPointCloudColor(cloud, ct::Color::Red);
                        }
                    }
                    if (it->isSelected() && cloud->size() > 100)
                        m_cloudview->addBox(cloud);
                }
                else{
                    if (exists){
                        m_cloudview->setPointCloudVisibility(QString::fromStdString(cloud->id()), false);
                    }
                }
            }

            // --- PolygonMesh / TexturedMesh 可见性联动 ---
            QString mesh_id = it->data(0, NodeMeshIdRole).toString();
            if (!mesh_id.isEmpty()) {
                bool shouldShow = (it->checkState(0) != Qt::Unchecked);
                if (shouldShow) {
                    // actor 已存在时只切换可见性，避免重复重建导致卡顿
                    if (m_cloudview->contains(mesh_id)) {
                        m_cloudview->setShapeVisibility(mesh_id, true);
                    } else {
                        auto _tex_mesh = m_registry->getTexturedMesh(mesh_id);
                        if (_tex_mesh && !_tex_mesh->objFilePath.empty()) {
                            // 纹理 mesh：隐藏点云散点，移除无纹理 mesh，显示带纹理 mesh 表面
                            m_cloudview->setPointCloudVisibility(mesh_id, false);
                            m_cloudview->addTexturedMesh(QString::fromStdString(_tex_mesh->objFilePath), mesh_id);
                        } else {
                            // 无纹理 mesh 或算法生成的 mesh：用 VTK actor 渲染
                            auto _mesh_actor = m_registry->getMesh(mesh_id);
                            if (_mesh_actor) {
                                m_cloudview->setPointCloudVisibility(mesh_id, false);
                                m_cloudview->addMeshActor(_mesh_actor, mesh_id);
                            }
                        }
                    }
                } else {
                    // 只隐藏，不销毁 actor，下次勾选时直接切换可见性避免重建卡顿
                    m_cloudview->setShapeVisibility(mesh_id, false);
                }
            }
        }

        // Sync scalar bar visibility with selected+checked state
        bool anyVisible = false;
        bool anySFSMode = false;
        for (QTreeWidgetItem* sel : getSelectedItems()) {
            if (sel->checkState(0) != Qt::Unchecked) {
                anyVisible = true;
                auto cloud = getCloud(sel);
                if (cloud) {
                    QString mode = QString::fromStdString(cloud->currentColorMode());
                    if (!mode.isEmpty() && cloud->hasScalarField(mode.toStdString()))
                        anySFSMode = true;
                }
                break;
            }
        }
        if (!anyVisible || !anySFSMode) {
            m_cloudview->hideScalarBar();
        } else {
            // Re-show scalar bar and refresh SFDisplayPanel if present
            m_cloudview->setScalarBarVisible(true);
            for (int row = 0; row < m_table->rowCount(); ++row) {
                auto* panel = qobject_cast<SFDisplayPanel*>(m_table->cellWidget(row, 0));
                if (panel) {
                    panel->reloadField();
                    break;
                }
            }
        }

        m_cloudview->setAutoRender(true);
        m_cloudview->refresh();
        this->blockSignals(wasBlocked);
    }

    void CloudTree::itemSelectionChangedEvent()
    {
        m_cloudview->setAutoRender(false);

        // ============================================================
        // Phase 1: 更新3D视图中的包围盒
        // ============================================================
        QList<QTreeWidgetItem*> allItems = m_registry->getAllItems();
        QList<QTreeWidgetItem*> selectedItems = getSelectedItems();

        for (QTreeWidgetItem* item : allItems){
            Cloud::Ptr cloud = m_registry->getCloud(item);
            if (!cloud) continue;

            bool isSelected = selectedItems.contains(item);
            bool isChecked = (item->checkState(0) != Qt::Unchecked);
            bool hasBox = m_cloudview->contains(QString::fromStdString(cloud->boxId()));

            if (getNodeType(item) == NodeMesh) {
                // NodeMesh 不添加包围盒，但需清理可能残留的 box
                if (hasBox) m_cloudview->removeShape(QString::fromStdString(cloud->boxId()));
                continue;
            }

            if (isSelected && isChecked){
                if (!hasBox && cloud->size() > 100)
                    m_cloudview->addBox(cloud);
            }
            else{
                if (hasBox) m_cloudview->removeShape(QString::fromStdString(cloud->boxId()));
            }
        }

        // ============================================================
        // Phase 2: 更新属性栏（根据节点类型动态构建）
        // ============================================================
        if (!m_table) {
            m_cloudview->setAutoRender(true);
            m_cloudview->refresh();
            return;
        }

        m_cloudview->showCloudId("");

        // 清空旧内容
        m_table->setRowCount(0);

        if (selectedItems.isEmpty()){
            m_cloudview->setAutoRender(true);
            m_cloudview->refresh();
            return;
        }

        QTreeWidgetItem* firstItem = selectedItems.first();
        SceneNodeType nodeType = getNodeType(firstItem);
        Cloud::Ptr cloud = getCloud(firstItem);
        if (!cloud) {
            m_cloudview->setAutoRender(true);
            m_cloudview->refresh();
            return;
        }

        QString cloudId = QString::fromStdString(cloud->id());
        bool isMesh = (nodeType == NodeMesh);
        bool hasTexture = m_registry->getTexturedMesh(cloudId) != nullptr;

        // 辅助 lambda：创建一个文本行
        auto addTextRow = [&](const QString& label, const QString& value, int rowHeight = 0) -> int {
            int row = m_table->rowCount();
            m_table->insertRow(row);
            QTableWidgetItem* labelItem = new QTableWidgetItem(label);
            labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
            m_table->setItem(row, 0, labelItem);
            QTableWidgetItem* valueItem = new QTableWidgetItem(value);
            valueItem->setFlags(valueItem->flags() & ~Qt::ItemIsEditable);
            m_table->setItem(row, 1, valueItem);
            if (rowHeight > 0) m_table->setRowHeight(row, rowHeight);
            return row;
        };

        // 辅助 lambda：创建分组标题行
        auto addSectionHeader = [&](const QString& title) -> int {
            int row = m_table->rowCount();
            m_table->insertRow(row);
            QTableWidgetItem* headerItem = new QTableWidgetItem(title);
            headerItem->setFlags(headerItem->flags() & ~Qt::ItemIsEditable);
            headerItem->setBackground(QColor("#E0E4E8"));
            QFont boldFont;
            boldFont.setBold(true);
            boldFont.setPointSize(10);
            headerItem->setFont(boldFont);
            // 合并两列
            m_table->setItem(row, 0, headerItem);
            m_table->setSpan(row, 0, 1, 2);
            m_table->setRowHeight(row, 24);
            return row;
        };

        // ============================================================
        // Common Section
        // ============================================================
        addSectionHeader("Common");

        addTextRow("Id", QString::fromStdString(cloud->id()));
        addTextRow("Type", QString::fromStdString(cloud->type()));
        addTextRow("Size", QString::number(cloud->size()));

        // BBox
        {
            ct::Box box = cloud->box();
            ct::PointXYZ cmin = cloud->min();
            ct::PointXYZ cmax = cloud->max();
            QString bboxText = QString("X: %1 (%2 - %3)\nY: %4 (%5 - %6)\nZ: %7 (%8 - %9)")
                .arg(box.width, 0, 'f', 3).arg(cmin.x, 0, 'f', 3).arg(cmax.x, 0, 'f', 3)
                .arg(box.height, 0, 'f', 3).arg(cmin.y, 0, 'f', 3).arg(cmax.y, 0, 'f', 3)
                .arg(box.depth, 0, 'f', 3).arg(cmin.z, 0, 'f', 3).arg(cmax.z, 0, 'f', 3);
            int bboxRow = addTextRow("BBox", bboxText, 66);
            if (auto item = m_table->item(bboxRow, 1))
                item->setTextAlignment(Qt::AlignLeft | Qt::AlignTop);
        }

        // Center
        {
            Eigen::Vector3f center = cloud->center();
            QString centerText = QString("X: %1\nY: %2\nZ: %3")
                .arg(center.x(), 0, 'f', 3)
                .arg(center.y(), 0, 'f', 3)
                .arg(center.z(), 0, 'f', 3);
            int centerRow = addTextRow("Center", centerText, 66);
            if (auto item = m_table->item(centerRow, 1))
                item->setTextAlignment(Qt::AlignLeft | Qt::AlignTop);
        }

        // 显示 ID
        m_cloudview->showCloudId(cloudId);

        // Opacity
        {
            int row = m_table->rowCount();
            m_table->insertRow(row);
            QTableWidgetItem* labelItem = new QTableWidgetItem("Opacity");
            labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
            m_table->setItem(row, 0, labelItem);

            QDoubleSpinBox* opacity = new QDoubleSpinBox;
            opacity->setSingleStep(0.1);
            opacity->setRange(0, 1);
            opacity->setValue(cloud->opacity());
            opacity->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            if (isMesh) {
                connect(opacity, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                        this, [=](double value){
                            cloud->setOpacity(value);
                            m_cloudview->setTextureMeshOpacity(cloudId, value);
                            m_cloudview->refresh();
                        });
            } else {
                connect(opacity, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                        this, [=](double value){
                            cloud->setOpacity(value);
                            m_cloudview->setPointCloudOpacity(cloudId, value);
                            m_cloudview->refresh();
                        });
            }
            m_table->setCellWidget(row, 1, opacity);
        }

        // ============================================================
        // Cloud Section（仅点云节点）
        // ============================================================
        if (!isMesh) {
            addSectionHeader("Cloud");

            // Resolution
            addTextRow("Resolution", QString::number(cloud->resolution()));

            // 当前颜色模式判定
            auto sf_fields = cloud->getScalarFieldNames();
            bool has_sf = !sf_fields.empty();
            QString currentMode = QString::fromStdString(cloud->currentColorMode());
            bool currently_sf = !currentMode.isEmpty()
                && cloud->hasScalarField(currentMode.toStdString());

            // Color
            QComboBox* color_mode = nullptr;
            {
                int row = m_table->rowCount();
                m_table->insertRow(row);
                QTableWidgetItem* labelItem = new QTableWidgetItem("Color");
                labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
                m_table->setItem(row, 0, labelItem);

                color_mode = new QComboBox;
                color_mode->addItem("RGB (Default)");
                if (has_sf) {
                    color_mode->addItem("Scalar Field");
                }

                color_mode->blockSignals(true);
                if (currently_sf) {
                    color_mode->setCurrentText("Scalar Field");
                } else {
                    int idx = color_mode->findText(currentMode);
                    color_mode->setCurrentIndex(idx >= 0 ? idx : 0);
                }
                color_mode->blockSignals(false);
                color_mode->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

                m_table->setCellWidget(row, 1, color_mode);
            }

            // Point Size
            {
                int row = m_table->rowCount();
                m_table->insertRow(row);
                QTableWidgetItem* labelItem = new QTableWidgetItem("PointSize");
                labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
                m_table->setItem(row, 0, labelItem);

                QSpinBox* point_size = new QSpinBox;
                point_size->setRange(1, 99);
                point_size->setValue(cloud->pointSize());
                point_size->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
                connect(point_size, QOverload<int>::of(&QSpinBox::valueChanged),
                        this, [=](int value){
                            cloud->setPointSize(value);
                            m_cloudview->setPointCloudSize(cloudId, value);
                            m_cloudview->refresh();
                        });
                m_table->setCellWidget(row, 1, point_size);
            }

            // Normals
            {
                int row = m_table->rowCount();
                m_table->insertRow(row);
                QTableWidgetItem* labelItem = new QTableWidgetItem("Normals");
                labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
                m_table->setItem(row, 0, labelItem);

                QWidget* normals_widget = new QWidget;
                QHBoxLayout* layout = new QHBoxLayout(normals_widget);
                layout->setContentsMargins(0,0,0,0);
                layout->setSpacing(5);

                QCheckBox* show_normals = new QCheckBox;
                QDoubleSpinBox* scale = new QDoubleSpinBox;
                scale->setSingleStep(0.01);
                scale->setRange(0, 9999);
                scale->setValue(0.05);
                scale->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

                show_normals->setEnabled(cloud->hasNormals());

                connect(scale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                        [=](double value){
                            if (cloud->hasNormals() && show_normals->isChecked())
                                m_cloudview->addPointCloudNormals(cloud, 1, value);
                        });
                connect(show_normals, &QCheckBox::stateChanged,
                        [=](int state){
                            if (state) m_cloudview->addPointCloudNormals(cloud, 1, scale->value());
                            else m_cloudview->removeShape(QString::fromStdString(cloud->normalId()));
                        });

                layout->addWidget(show_normals);
                layout->addWidget(scale);
                layout->addStretch();
                m_table->setCellWidget(row, 1, normals_widget);
            }

            // ============================================================
            // SF Display Section（仅当点云拥有标量场时）
            // ============================================================
            if (has_sf) {
                addSectionHeader("SF Display");
                int sf_section_row = m_table->rowCount() - 1;

                // Field 行
                QComboBox* combo_field = nullptr;
                int field_row = -1;
                {
                    int row = m_table->rowCount();
                    field_row = row;
                    m_table->insertRow(row);
                    QTableWidgetItem* labelItem = new QTableWidgetItem("Field");
                    labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
                    m_table->setItem(row, 0, labelItem);

                    combo_field = new QComboBox;
                    combo_field->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
                    for (const auto& f : sf_fields)
                        combo_field->addItem(QString::fromStdString(f));

                    if (currently_sf) {
                        int idx = combo_field->findText(currentMode);
                        if (idx >= 0) combo_field->setCurrentIndex(idx);
                    }

                    m_table->setCellWidget(row, 1, combo_field);
                }

                // Color Scale 行
                QComboBox* combo_cmap = nullptr;
                int cmap_row = -1;
                {
                    int row = m_table->rowCount();
                    cmap_row = row;
                    m_table->insertRow(row);
                    QTableWidgetItem* labelItem = new QTableWidgetItem("Color Scale");
                    labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
                    m_table->setItem(row, 0, labelItem);

                    combo_cmap = new QComboBox;
                    for (const auto& n : colormapNames())
                        combo_cmap->addItem(QString::fromStdString(n));
                    combo_cmap->setCurrentText("jet");
                    combo_cmap->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
                    m_table->setCellWidget(row, 1, combo_cmap);
                }

                // SFDisplayPanel（tab widget）
                SFDisplayPanel* sf_panel = nullptr;
                int panel_row = -1;
                {
                    sf_panel = new SFDisplayPanel();
                    sf_panel->onColorChanged = [=]() {
                        m_cloudview->invalidateCloudRender(cloudId);
                        m_cloudview->refresh();
                    };
                    QObject::connect(sf_panel, &SFDisplayPanel::scalarBarRequested,
                            m_cloudview, &CloudView::showScalarBar);
                    QObject::connect(sf_panel, &SFDisplayPanel::scalarBarDisplayRangeChanged,
                            m_cloudview, &CloudView::setScalarBarDisplayRange);
                    QObject::connect(sf_panel, &SFDisplayPanel::scalarBarHistogramChanged,
                            m_cloudview, &CloudView::setScalarBarHistogram);
                    QObject::connect(sf_panel, &SFDisplayPanel::scalarBarToggled,
                            m_cloudview, &CloudView::setScalarBarVisible);
                    QObject::connect(sf_panel, &SFDisplayPanel::scalarBarShowZero,
                            m_cloudview, &CloudView::setScalarBarShowZero);
                    QObject::connect(sf_panel, &SFDisplayPanel::scalarBarShowCurve,
                            m_cloudview, &CloudView::setScalarBarShowCurve);

                    sf_panel->bindCloud(cloud);

                    QObject::connect(combo_field, &QComboBox::currentTextChanged,
                            sf_panel, &SFDisplayPanel::setField);

                    QObject::connect(combo_cmap, &QComboBox::currentTextChanged,
                            this, [=](const QString& name) {
                                sf_panel->setColormap(colormapFromName(name.toStdString()));
                            });

                    panel_row = m_table->rowCount();
                    m_table->insertRow(panel_row);
                    m_table->setSpan(panel_row, 0, 1, 2);
                    m_table->setCellWidget(panel_row, 0, sf_panel);
                }

                // 显隐控制
                auto showSFRows = [=]() {
                    m_table->setRowHeight(sf_section_row, 28);
                    m_table->setRowHeight(field_row, 28);
                    m_table->setRowHeight(cmap_row, 28);
                    m_table->setRowHeight(panel_row, 320);
                    sf_panel->reloadField();
                };
                auto hideSFRows = [=]() {
                    m_table->setRowHeight(sf_section_row, 0);
                    m_table->setRowHeight(field_row, 0);
                    m_table->setRowHeight(cmap_row, 0);
                    m_table->setRowHeight(panel_row, 0);
                    m_cloudview->hideScalarBar();
                };

                if (currently_sf) {
                    showSFRows();
                } else {
                    hideSFRows();
                }

                // Color combo 联动
                connect(color_mode, &QComboBox::currentTextChanged,
                        this, [=](const QString& text) {
                            if (text == "Scalar Field") {
                                showSFRows();
                                auto cur_fields = cloud->getScalarFieldNames();
                                if (!cur_fields.empty()) {
                                    cloud->updateColorByField(cur_fields[0]);
                                    m_cloudview->addPointCloud(cloud);
                                }
                            } else {
                                hideSFRows();
                                if (text == "RGB (Default)") {
                                    m_cloudview->resetPointCloudColor(cloud);
                                }
                            }
                        });
            }
        }

        // ============================================================
        // Mesh Section（仅模型节点）
        // ============================================================
        if (isMesh) {
            addSectionHeader("Mesh");

            // Face Count
            int faceCount = 0;
            auto _face_mesh = m_registry->getMesh(cloudId);
            if (_face_mesh)
                faceCount = _face_mesh->polygons.size();
            auto _face_tex = m_registry->getTexturedMesh(cloudId);
            if (_face_tex)
                faceCount = _face_tex->mesh->polygons.size();
            addTextRow("Faces", QString::number(faceCount));

            // Texture（所有模型节点都显示，无纹理时禁用）
            {
                int row = m_table->rowCount();
                m_table->insertRow(row);
                QTableWidgetItem* labelItem = new QTableWidgetItem("Texture");
                labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
                m_table->setItem(row, 0, labelItem);

                QCheckBox* showTex = new QCheckBox;
                showTex->setChecked(hasTexture);
                showTex->setEnabled(hasTexture);
                connect(showTex, &QCheckBox::stateChanged,
                        this, [=](int state){
                            if (state) {
                                // 切换到纹理模式：用 VTK OBJ reader 重新加载带纹理的 mesh
                                auto _tex_cb = m_registry->getTexturedMesh(cloudId);
                                if (_tex_cb && !_tex_cb->objFilePath.empty())
                                    m_cloudview->addTexturedMesh(
                                        QString::fromStdString(_tex_cb->objFilePath), cloudId);
                            } else {
                                // 切换到无纹理模式：用 VTK actor 渲染（支持透明度等属性）
                                m_cloudview->removeTexturedMesh(cloudId);
                                auto _mesh_cb = m_registry->getMesh(cloudId);
                                if (_mesh_cb)
                                    m_cloudview->addMeshActor(_mesh_cb, cloudId);
                            }
                        });
                m_table->setCellWidget(row, 1, showTex);
            }

            // Representation
            {
                int row = m_table->rowCount();
                m_table->insertRow(row);
                QTableWidgetItem* labelItem = new QTableWidgetItem("Representation");
                labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
                m_table->setItem(row, 0, labelItem);

                QComboBox* repr = new QComboBox;
                repr->addItem("Surface");
                repr->addItem("Wireframe");
                repr->addItem("Points");
                repr->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
                repr->setCurrentIndex(0);

                connect(repr, &QComboBox::currentTextChanged,
                        this, [=](const QString& text){
                            // 所有 mesh 都使用 VTK actor 渲染，统一通过 actor property 控制
                            int type = 2;
                            if (text == "Wireframe") type = 1;
                            else if (text == "Points") type = 0;
                            m_cloudview->setTextureMeshRepresentation(cloudId, type);
                            m_cloudview->refresh();
                        });
                m_table->setCellWidget(row, 1, repr);
            }
        }

        m_cloudview->setAutoRender(true);
        m_cloudview->refresh();
    }

    void CloudTree::showProgress(const QString &message) { m_progress->showProgress(message); }

    void CloudTree::closeProgress() { m_progress->closeProgress(); }

    void CloudTree::setProgress(int percent) { m_progress->setProgress(percent); }

    void CloudTree::bindWorker(QObject *worker) { m_progress->bindWorker(worker); }

    void CloudTree::addResultGroup(const Cloud::Ptr &originCloud, const std::vector<Cloud::Ptr> &results,
                                   const QString &groupName) {
        if (!originCloud) return;

        QTreeWidgetItem* originItem = getItemById(QString::fromStdString(originCloud->id()));
        if (!originItem){
            // 如果原始点云被删除了，尝试将结果点云添加到根目录
            printW(QString("Origin cloud [%1] item not found, add results to root.").arg(QString::fromStdString(originCloud->id())));
            QTreeWidgetItem* rootGroup = addItem(nullptr, groupName, NodeGroup);
            for (const auto& cloud : results){
                insertCloud(cloud, rootGroup, true);
            }
            return;
        }

        QTreeWidgetItem* parentFolder = originItem->parent();

        // 清除同名的旧结果组（避免重复运行时产生冲突）
        const bool wasBlockedCleanup = this->blockSignals(true);
        for (int i = 0; i < (parentFolder ? parentFolder->childCount() : topLevelItemCount()); i++) {
            QTreeWidgetItem* child = parentFolder ? parentFolder->child(i) : topLevelItem(i);
            if (child && child->text(0) == groupName) {
                // 从 VTK 视图中移除所有子点云
                QList<QTreeWidgetItem*> children;
                getAllChildItems(child, children);
                for (QTreeWidgetItem* ch : children) {
                    Cloud::Ptr c = getCloud(ch);
                    if (c) {
                        m_cloudview->removePointCloud(QString::fromStdString(c->id()));
                        m_cloudview->removeShape(QString::fromStdString(c->id()));
                    }
                    m_registry->unregisterCloud(ch);
                }
                if (parentFolder) {
                    parentFolder->removeChild(child);
                } else {
                    takeTopLevelItem(i);
                }
                delete child;
                break;
            }
        }
        this->blockSignals(wasBlockedCleanup);

        // 创建结果组文件夹（策略二：新建 GroupNode）
        QTreeWidgetItem* groupItem = addItem(parentFolder, groupName, NodeGroup);

        groupItem->setExpanded(true);
        if (parentFolder) parentFolder->setExpanded(true);

        m_cloudview->setAutoRender(false);

        for (const auto& cloud : results){
            if (!cloud) continue;
            insertCloud(cloud, groupItem, true);
            m_cloudview->setPointCloudVisibility(QString::fromStdString(cloud->id()), true);
        }

        // 自动隐藏原始点云
        const bool wasBlocked = this->blockSignals(true);
        if (originItem->checkState(0) == Qt::Checked){
            originItem->setCheckState(0, Qt::Unchecked);
            m_cloudview->removePointCloud(QString::fromStdString(originCloud->id()));
            m_cloudview->removeShape(QString::fromStdString(originCloud->id()));
            m_cloudview->removePointCloud(QString::fromStdString(originCloud->normalId()));
        }
        this->blockSignals(wasBlocked);

        m_cloudview->setAutoRender(true);
        m_cloudview->resetCamera();
        m_cloudview->refresh();

        printI(QString("Add results to group [%1] done.").arg(groupName));
    }

    void CloudTree::registerMesh(const QString& cloudId, const pcl::PolygonMesh::Ptr& mesh)
    {
        m_registry->registerMesh(cloudId, mesh);

        QTreeWidgetItem* item = getItemById(cloudId);
        if (item) {
            item->setData(0, NodeMeshIdRole, cloudId);
            // 如果节点当前类型是 NodeCloud，升级为 NodeMesh 并更新图标
            SceneNodeType curType = CustomTree::getNodeType(item);
            if (curType == NodeCloud) {
                item->setData(0, NodeTypeRole, static_cast<int>(NodeMesh));
                item->setIcon(0, iconForType(NodeMesh));
            }
        }

        // 应用已保存的透明度
        Cloud::Ptr cloud = getCloud(item);
        float opacity = cloud ? cloud->opacity() : 1.0f;

        m_cloudview->addMeshActor(mesh, cloudId);
        if (opacity < 1.0f) {
            m_cloudview->setTextureMeshOpacity(cloudId, opacity);
        }

        // 如果该节点当前被选中，刷新属性栏以显示正确的面数等信息
        if (item && item->isSelected()) {
            itemSelectionChangedEvent();
        }
    }

    void CloudTree::registerMeshPrebuilt(const QString& cloudId, const pcl::PolygonMesh::Ptr& mesh,
                                           vtkSmartPointer<vtkPolyData> polydata)
    {
        m_registry->registerMesh(cloudId, mesh);

        QTreeWidgetItem* item = getItemById(cloudId);
        if (item) {
            item->setData(0, NodeMeshIdRole, cloudId);
            SceneNodeType curType = CustomTree::getNodeType(item);
            if (curType == NodeCloud) {
                item->setData(0, NodeTypeRole, static_cast<int>(NodeMesh));
                item->setIcon(0, iconForType(NodeMesh));
            }
        }

        Cloud::Ptr cloud = getCloud(item);
        float opacity = cloud ? cloud->opacity() : 1.0f;

        m_cloudview->addMeshActorFromPolydata(polydata, cloudId);
        if (opacity < 1.0f) {
            m_cloudview->setTextureMeshOpacity(cloudId, opacity);
        }

        if (item && item->isSelected()) {
            itemSelectionChangedEvent();
        }
    }

    void CloudTree::unregisterMesh(const QString& cloudId)
    {
        m_registry->unregisterMesh(cloudId);
        m_cloudview->removeTexturedMesh(cloudId);
        m_cloudview->removePolygonMesh(cloudId);
        m_cloudview->removeShape(cloudId);

        QTreeWidgetItem* item = getItemById(cloudId);
        if (item) {
            item->setData(0, NodeMeshIdRole, QVariant());
        }
    }

    void CloudTree::registerShape(const QString& parentCloudId, const QString& shapeId,
                                   const QString& displayName,
                                   const pcl::PolygonMesh::Ptr& mesh)
    {
        QTreeWidgetItem* parentItem = getItemById(parentCloudId);
        if (!parentItem) return;

        const bool wasBlocked = this->blockSignals(true);
        QTreeWidgetItem* shapeItem = addItem(parentItem, displayName, NodeBoundary, false);
        this->blockSignals(wasBlocked);

        shapeItem->setData(0, NodeShapeIdRole, shapeId);

        if (mesh) {
            m_registry->registerShape(shapeId, mesh);
        }
    }

    void CloudTree::registerCloudPolyline(const QString& parentCloudId, const QString& shapeId,
                                           const QString& displayName, const Cloud::Ptr& cloud)
    {
        QTreeWidgetItem* parentItem = getItemById(parentCloudId);
        if (!parentItem) return;

        const bool wasBlocked = this->blockSignals(true);
        QTreeWidgetItem* shapeItem = addItem(parentItem, displayName, NodeBoundary, false);
        this->blockSignals(wasBlocked);

        shapeItem->setData(0, NodeShapeIdRole, shapeId);
        m_registry->registerCloudPolyline(shapeId, cloud);

        // 立即绘制折线
        m_cloudview->addPolylineFromCloud(cloud, shapeId);
    }

    void CloudTree::registerTexturedMesh(const QString& cloudId, const TexturedMeshPtr& texturedMesh)
    {
        m_registry->registerTexturedMesh(cloudId, texturedMesh);
        m_registry->registerMesh(cloudId, texturedMesh->mesh); // 兼容保存等非纹理功能

        QTreeWidgetItem* item = getItemById(cloudId);
        if (item) {
            item->setData(0, NodeMeshIdRole, cloudId);
        }

        if (!texturedMesh->objFilePath.empty()) {
            // 纹理 mesh 用 VTK actor 渲染表面，移除之前注册的无纹理 mesh 和点云避免遮挡
            m_cloudview->removePolygonMesh(cloudId);
            m_cloudview->removePointCloud(cloudId);
            m_cloudview->addTexturedMesh(QString::fromStdString(texturedMesh->objFilePath), cloudId);
        } else {
            m_cloudview->addMeshActor(texturedMesh->mesh, cloudId);
        }
    }

    void CloudTree::unregisterTexturedMesh(const QString& cloudId)
    {
        m_registry->unregisterTexturedMesh(cloudId);
        m_cloudview->removeTexturedMesh(cloudId);

        // 同时清理普通 mesh 引用
        m_registry->unregisterMesh(cloudId);
        m_cloudview->removePolygonMesh(cloudId);
        m_cloudview->removeShape(cloudId);

        QTreeWidgetItem* item = getItemById(cloudId);
        if (item) {
            item->setData(0, NodeMeshIdRole, QVariant());
        }
    }

    void CloudTree::handleTexturedMeshLoaded(const QString& cloudId, const QString& objFilePath)
    {
        if (cloudId.isEmpty() || objFilePath.isEmpty()) return;

        auto _ltm_mesh = m_registry->getMesh(cloudId);
        if (!_ltm_mesh || _ltm_mesh->polygons.empty()) {
            printW("Textured mesh result received but mesh not found in registry for: " + cloudId);
            return;
        }

        printI(QString("Textured mesh detected: %1 polygons, obj: %2").arg(_ltm_mesh->polygons.size()).arg(objFilePath));

        TexturedMeshPtr texturedMesh = std::make_shared<TexturedMesh>();
        texturedMesh->mesh = _ltm_mesh;
        texturedMesh->objFilePath = objFilePath.toStdString();

        registerTexturedMesh(cloudId, texturedMesh);
        itemSelectionChangedEvent();
    }

    QList<QPair<QString, pcl::PolygonMesh::Ptr>> CloudTree::getLoadedMeshes() const
    {
        return m_registry->getLoadedMeshes();
    }

    bool CloudTree::hasMesh(const QString& cloudId) const
    {
        return m_registry->hasMesh(cloudId);
    }

    void CloudTree::zoomToSelected() {
        std::vector<Cloud::Ptr> targets;
        std::vector<Cloud::Ptr> selected = getSelectedClouds();

        if (!selected.empty()){
            targets = selected;
        }

        if (targets.empty()){
            // 场景为空
            m_cloudview->resetCamera();
            return;
        }

        Eigen::Vector3f global_min(FLT_MAX, FLT_MAX, FLT_MAX);
        Eigen::Vector3f global_max(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        bool valid = false;
        for (const auto& cloud : targets) {
            if (cloud->empty()) continue;

            Eigen::Vector3f c_min = cloud->min().getVector3fMap();
            Eigen::Vector3f c_max = cloud->max().getVector3fMap();

            // 更新全局最小值
            global_min = global_min.cwiseMin(c_min);
            // 更新全局最大值
            global_max = global_max.cwiseMax(c_max);

            valid = true;
        }

        if (valid) {
            m_cloudview->zoomToBounds(global_min, global_max);
            printI("Zoom to fit selected/visible objects.");
        } else {
            m_cloudview->resetCamera();
        }
    }

    // ================================================================
    // 脚本使用中保护
    // ================================================================

    void CloudTree::markCloudInUse(const QString& id)
    {
        m_registry->markInUse(id);
    }

    void CloudTree::unmarkCloudInUse(const QString& id)
    {
        m_registry->unmarkInUse(id);
    }

    void CloudTree::releaseAllInUse()
    {
        m_registry->releaseAllInUse();
    }

    // ================================================================
    // 右键上下文菜单
    // ================================================================

    void CloudTree::contextMenuEvent(QContextMenuEvent* event)
    {
        QTreeWidgetItem* item = itemAt(event->pos());
        if (!item) return;

        QMenu menu(this);

        if (isCloudNode(item))
        {
            Cloud::Ptr cloud = getCloud(item);

            menu.addAction("Show / Hide", this, [this, item]() {
                bool next = (item->checkState(0) != Qt::Checked);
                item->setCheckState(0, next ? Qt::Checked : Qt::Unchecked);
            });

            menu.addSeparator();
            menu.addAction("Rename", this, [this, item]() {
                bool ok = false;
                Cloud::Ptr cloud = getCloud(item);
                QString oldName = cloud ? QString::fromStdString(cloud->id()) : item->text(0);
                QString newName = QInputDialog::getText(this, "Rename", "New name:",
                    QLineEdit::Normal, oldName, &ok);
                if (ok && !newName.isEmpty()) renameCloudItem(item, newName);
            });

            menu.addAction("Save As...", this, [this, item]() { saveCloudItem(item); });
            menu.addAction("Clone", this, [this, item]() { cloneCloudItem(item); });

            menu.addSeparator();
            menu.addAction("Delete", this, [this, item]() { removeCloudItem(item); });

            menu.addSeparator();
            menu.addAction("Zoom to Fit", this, [this, item]() {
                setCurrentItem(item);
                zoomToSelected();
            });
        }
        else if (isFolderNode(item))
        {
            // FileNode / GroupNode 通用菜单
            menu.addAction("Expand All", this, [item]() {
                std::function<void(QTreeWidgetItem*)> expand;
                expand = [&](QTreeWidgetItem* p) {
                    p->setExpanded(true);
                    for (int i = 0; i < p->childCount(); ++i)
                        expand(p->child(i));
                };
                expand(item);
            });

            menu.addAction("Collapse All", this, [item]() {
                std::function<void(QTreeWidgetItem*)> collapse;
                collapse = [&](QTreeWidgetItem* p) {
                    p->setExpanded(false);
                    for (int i = 0; i < p->childCount(); ++i)
                        collapse(p->child(i));
                };
                collapse(item);
            });

            menu.addSeparator();

            if (getNodeType(item) == NodeGroup)
            {
                menu.addAction("Rename", this, [this, item]() {
                    bool ok = false;
                    QString newName = QInputDialog::getText(this, "Rename Group", "New name:",
                        QLineEdit::Normal, item->text(0), &ok);
                    if (ok && !newName.isEmpty()) renameCloudItem(item, newName);
                });
            }

            menu.addAction("Delete", this, [this, item]() { removeCloudItem(item); });
        }

        if (!menu.actions().isEmpty())
            menu.exec(event->globalPos());
    }
}

