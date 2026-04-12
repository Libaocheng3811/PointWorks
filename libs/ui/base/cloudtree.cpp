#include "cloudtree.h"

#include "core/field_types.h"

#include "dialog/fieldmappingdialog.h"
#include "dialog/txtimportdialog.h"
#include "dialog/txtexportdialog.h"

#include <cfloat>
#include <QMetaType>

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
    // ROOT_PATH指向程序根目录路径
    CloudTree::CloudTree(QWidget *parent)
        : CustomTree(parent),
        m_path(ROOT_PATH),
        m_thread(this),
        m_tree_menu(nullptr)
    {
        // register meat type
        // qRegisterMetaType 函数用于注册一个自定义类型，以便它可以被用于 Qt 的元对象系统
        qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr &");
        qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr");
        qRegisterMetaType<QList<ct::FieldInfo>>("QList<ct::FieldInfo>");
        qRegisterMetaType<QMap<QString, QString>>("QMap<QString, QString>&");
        qRegisterMetaType<ct::TxtImportParams>("ct::TxtImportParams");
        qRegisterMetaType<ct::TxtExportParams>("ct::TxtExportParams");

        // move to thread
        m_fileio = new FileIO;
        m_fileio->moveToThread(&m_thread);
        connect(&m_thread, &QThread::finished, m_fileio, &QObject::deleteLater);
        connect(m_fileio, &FileIO::loadCloudResult, this, &CloudTree::loadCloudResult);
        connect(m_fileio, &FileIO::saveCloudResult, this, &CloudTree::saveCloudResult);

        connect(this, &CloudTree::loadPointCloud, m_fileio, &FileIO::loadPointCloud);
        connect(this, &CloudTree::savePointCloud, m_fileio, &FileIO::savePointCloud);
        connect(this, &QTreeWidget::itemChanged, this, &CloudTree::itemChangedEvent);
        connect(this, &CloudTree::itemSelectionChanged, this, &CloudTree::itemSelectionChangedEvent);

        connect(m_fileio, &FileIO::requestFieldMapping, this, &CloudTree::onFieldMappingRequested, Qt::BlockingQueuedConnection);
        connect(m_fileio, &FileIO::requestTxtImportSetup, this, &CloudTree::onTxtImportRequested, Qt::BlockingQueuedConnection);
        connect(m_fileio, &FileIO::requestTxtExportSetup, this, &CloudTree::onTxtExportRequested, Qt::BlockingQueuedConnection);
        connect(m_fileio, &FileIO::requestGlobalShift, this, &CloudTree::onGlobalFilterRequested, Qt::BlockingQueuedConnection);
        // 设置窗口部件接受拖放操作
        this->setAcceptDrops(true);

        m_thread.start();
    }

    CloudTree::~CloudTree()
    {
        m_thread.quit();
        if (!m_thread.wait(3000))
        {
            m_thread.terminate();
            m_thread.wait();
        }
        m_cloud_map.clear();
    }

    Cloud::Ptr CloudTree::getCloud(QTreeWidgetItem *item) {
        return m_cloud_map.value(item, nullptr);
    }

    void CloudTree::addCloud()
    {
        /**
         * @note 文件过滤器
         * 文件过滤器的写法遵循一定的格式，filter = "filterName(pattern1 pattern2 ...)";
         * filterName 是一个描述性的名字，用于在文件对话框中显示给用户。如 all, ply
         * pattern1 pattern2 ... 是一个或多个文件模式，它们定义了过滤器匹配的文件类型, 如 *.*, *.ply
         */
        // 定义了一个文件过滤器
        QString filter = "All Supported(*.ply *.pcd *.las *.laz *.obj *.ifs *.txt *.asc *.xyz);;All Files(*.*)";
        // 打开文件对话框,可以选择多个文件
        QStringList filePathList = QFileDialog::getOpenFileNames(this, tr("open cloud files"), m_path, filter);
        if (filePathList.isEmpty()) return;

        m_loading_queue_count = filePathList.size();

        showProgress("Loading Point Cloud...");

        bindWorker(m_fileio);

        for (auto& i : filePathList)
            emit loadPointCloud(i);
    }

    void CloudTree::loadCloudFile(const QString& filepath)
    {
        m_pending_parents.clear();
        showProgress("Loading Point Cloud...");
        bindWorker(m_fileio);
        m_loading_queue_count = 1;
        emit loadPointCloud(filepath);
    }

    void CloudTree::loadCloudFile(const QString& filepath, QTreeWidgetItem* targetParent)
    {
        m_pending_parents[QFileInfo(filepath).absoluteFilePath()] = targetParent;
        showProgress("Loading Point Cloud...");
        bindWorker(m_fileio);
        m_loading_queue_count = 1;
        emit loadPointCloud(filepath);
    }

    void CloudTree::saveCloudFile(const Cloud::Ptr& cloud, const QString& filepath, bool isBinary)
    {
        showProgress("Saving Point Cloud...");
        bindWorker(m_fileio);
        emit savePointCloud(cloud, filepath, isBinary);
    }

    QTreeWidgetItem* CloudTree::getItemById(const QString &id) {
        return m_item_by_id.value(id, nullptr);
    }


    void CloudTree::insertCloud(const Cloud::Ptr& cloud, QTreeWidgetItem* parentItem,
                                 bool selected, MountStrategy strategy)
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
        SceneNodeType nodeType = NodeCloud;

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

        // 创建节点
        QTreeWidgetItem* newItem = addItem(actualParent, QString::fromStdString(cloud->id()), nodeType, false);

        m_cloud_map.insert(newItem, cloud);
        m_item_by_id.insert(QString::fromStdString(cloud->id()), newItem);
        m_cloudview->addPointCloud(cloud); // 渲染

        // 根据点云类型决定显示属性
        if (cloud->pointSize() > 1) {
            m_cloudview->setPointCloudSize(QString::fromStdString(cloud->id()), cloud->pointSize());
            if (QString::fromStdString(cloud->id()).contains("picked-")) m_cloudview->setPointCloudColor(cloud, ct::Color::Red);
        }

        if (selected && cloud->pointSize() > 100){
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

        printI(QString("Add cloud[id:%1] done.").arg(QString::fromStdString(cloud->id())));

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
            if (cloud && m_clouds_in_use.contains(QString::fromStdString(cloud->id()))) {
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
                m_cloud_map.remove(c);
                m_item_by_id.remove(cid);
                // 清理关联的 PolygonMesh
                if (m_mesh_map.contains(cid)) {
                    unregisterMesh(cid);
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
        if (!m_clouds_in_use.isEmpty()) {
            printW(QString("Cannot remove all: %1 cloud(s) in use by script").arg(m_clouds_in_use.size()));
            return;
        }

        m_cloudview->setAutoRender(false);

        // 遍历所有 TopLevel Items 并移除
        while (this->topLevelItemCount() > 0){
            removeCloudItem(this->topLevelItem(0));
        }

        m_cloudview->removeAllPointClouds();
        m_cloudview->removeAllShapes();
        m_cloud_map.clear();
        m_item_by_id.clear();

        m_cloudview->setAutoRender(true);
        m_cloudview->refresh();
        printI("remove all clouds done.");
    }

    void CloudTree::saveCloudItem(QTreeWidgetItem *item) {
        Cloud::Ptr cloud = getCloud(item);
        if (!cloud) return; // 如果是文件夹，不保存，返回空

        QString filter = "PLY(*.ply);;PCD(*.pcd);;LAS(*.las);;TXT(*.txt)";
        QString filepath = QFileDialog::getSaveFileName(this, tr("Save cloud file"), QString::fromStdString(cloud->id()), filter);
        if (filepath.isEmpty()) return;

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
        bindWorker(m_fileio);

        emit savePointCloud(cloud, filepath, k);
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

        m_item_by_id.remove(QString::fromStdString(cloud->id()));
        cloud->setId(name.toStdString());
        m_item_by_id.insert(name, item);
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
        std::vector<Cloud::Ptr> clouds;
        QList<Cloud::Ptr> values = m_cloud_map.values();

        for (const auto& c : values) clouds.push_back(c);
        return clouds;
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

                const std::vector<ct::RGB>* colors = (has_color && block->m_colors) ? block->m_colors.get() : nullptr;
                const std::vector<ct::CompressedNormal>* normals = (has_normal && block->m_normals) ? block->m_normals.get() : nullptr;
                const std::map<std::string, std::vector<float>>* scalars = (!block->m_scalar_fields.empty()) ? &block->m_scalar_fields : nullptr;

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

    void CloudTree::loadCloudResult(bool success, const Cloud::Ptr &cloud, float time)
    {
        if (!success)
            printE("load the file failed!");
        else
        {
            printI(QString("Load the file [path:%1] done, take time %2 ms.").arg(QFileInfo(QString::fromStdString(cloud->filepath())).absoluteFilePath()).arg(time));
            m_path = QFileInfo(QString::fromStdString(cloud->filepath())).path();

            if (!m_pending_parents.isEmpty())
            {
                // 恢复模式：查找对应的目标父节点（不创建 FileNode）
                QString fp = QFileInfo(QString::fromStdString(cloud->filepath())).absoluteFilePath();
                QTreeWidgetItem* parent = m_pending_parents.value(fp, nullptr);
                if (parent) {
                    insertCloud(cloud, parent, true);
                    m_pending_parents.remove(fp);
                } else {
                    // fallback：未找到匹配的父节点，按普通模式处理
                    QString folderName = QFileInfo(fp).fileName();
                    QTreeWidgetItem* fileNode = addItem(nullptr, folderName + "  (" + fp + ")", NodeFile);
                    fileNode->setData(0, NodeFilePathRole, fp);
                    insertCloud(cloud, fileNode, true);
                }
            }
            else
            {
                // 普通加载模式：创建 FileNode 作为根节点
                QString folderName = QFileInfo(QString::fromStdString(cloud->filepath())).fileName();
                QString fullPath = QFileInfo(QString::fromStdString(cloud->filepath())).absoluteFilePath();
                QTreeWidgetItem* fileNode = addItem(nullptr, folderName + "  (" + fullPath + ")", NodeFile);
                fileNode->setData(0, NodeFilePathRole, fullPath);
                insertCloud(cloud, fileNode, true);
            }
        }
        m_loading_queue_count--;
        if (m_loading_queue_count <= 0) {
            // 所有任务都完成了，关闭进度条
            m_loading_queue_count = 0; // 归零防守
            closeProgress();
        } else {
            // 还有任务在队列中，更新界面提示，而不是关闭
            if (m_processing_dialog) {
                // 重置进度条为 0，准备显示下一个文件的进度
                m_processing_dialog->setProgress(0);
                // 更新提示文字
                QString msg = QString("Loading Point Cloud... (%1 remaining)").arg(m_loading_queue_count);
                m_processing_dialog->setMessage(msg);
            }
        }
    }

    void CloudTree::saveCloudResult(bool success, const QString &path, float time)
    {
        if (!success)
            printE("Save the file failed!");
        else
        {
            m_path = path;
            printI(QString("Save the file [path:%1] done, take time %2 ms.").arg(path).arg(time));
        }

        closeProgress();
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
            if (!cloud) continue; // 如果是纯文件夹，跳过

            // 只要不是 Unchecked，就显示 (Checked/Partially)
            bool shouldShow = (it->checkState(0) != Qt::Unchecked);
            bool exists = m_cloudview->contains(QString::fromStdString(cloud->id()));

            if(shouldShow){
                if (exists){
                    // 若点云已经存在，设为可见
                    m_cloudview->setPointCloudVisibility(QString::fromStdString(cloud->id()), true);
                }
                else {
                    // 不存在，首次加载
                    m_cloudview->addPointCloud(cloud);
                    // 恢复属性
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
                    // 不移除点云，设为隐藏
                    m_cloudview->setPointCloudVisibility(QString::fromStdString(cloud->id()), false);
                }
            }

            // --- PolygonMesh 可见性联动 ---
            QString mesh_id = it->data(0, NodeMeshIdRole).toString();
            if (!mesh_id.isEmpty()) {
                bool shouldShow = (it->checkState(0) != Qt::Unchecked);
                if (shouldShow) {
                    auto mesh_it = m_mesh_map.find(mesh_id);
                    if (mesh_it != m_mesh_map.end()) {
                        m_cloudview->addPolygonMesh(mesh_it.value(), mesh_id);
                    }
                } else {
                    m_cloudview->removePolygonMesh(mesh_id);
                    m_cloudview->removeShape(mesh_id);
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

        // 获取所有节点和被选中的节点
        QList<QTreeWidgetItem*> allItems = m_cloud_map.keys();
        QList<QTreeWidgetItem*> selectedItems = getSelectedItems();

        // 更新3D视图中的包围盒
        for (QTreeWidgetItem* item : allItems){
            Cloud::Ptr cloud = m_cloud_map.value(item);
            if (!cloud) continue;

            bool isSelected = selectedItems.contains(item);
            bool isVisible = m_cloudview->contains(QString::fromStdString(cloud->id())); // 点云是否在视图中显示，只有显示的点云才绘制包围盒
            bool hasBox = m_cloudview->contains(QString::fromStdString(cloud->boxId()));

            if (isSelected && isVisible){
                // 只有当：被选中 + 已显示 + 是原始点云(点数多) 时，才显示包围盒
                if (!hasBox && cloud->size() > 100)
                    m_cloudview->addBox(cloud);
            }
            else{
                // 未选中，或者云本身都没显示，肯定要移除盒子
                if (hasBox) m_cloudview->removeShape(QString::fromStdString(cloud->boxId()));
            }
        }

        // 更新属性表
        if (m_table) {
            const int COLOR_MODE_ROW = 7;
            m_table->removeCellWidget(4, 1); // point size
            m_table->removeCellWidget(5, 1); // Opacity
            m_table->removeCellWidget(6, 1); // Normal Checkbox
            m_table->removeCellWidget(COLOR_MODE_ROW, 1); // Color Mode

            // 清除文本内容
            for(int i=0; i<4; ++i) {
                if (m_table->item(i, 1)) m_table->item(i, 1)->setText("");
                else m_table->setItem(i, 1, new QTableWidgetItem(""));
            }
            m_cloudview->showCloudId(""); // 清空左下角 ID

            QList<QTreeWidgetItem*> selectedItems = getSelectedItems();

            if (!selectedItems.isEmpty()){
                // 只显示第一个选中的点云
                QTreeWidgetItem* firstItem = selectedItems.first();
                Cloud::Ptr update_cloud = getCloud(firstItem);

                if (update_cloud){
                    // 1. 基础文本信息
                    m_table->setItem(0, 1, new QTableWidgetItem(QString::fromStdString(update_cloud->id())));
                    m_table->setItem(1, 1, new QTableWidgetItem(QString::fromStdString(update_cloud->type())));
                    m_table->setItem(2, 1, new QTableWidgetItem(QString::number(update_cloud->size())));
                    m_table->setItem(3, 1, new QTableWidgetItem(QString::number(update_cloud->resolution())));

                    // 2. 在视图左下角显示 ID
                    m_cloudview->showCloudId(QString::fromStdString(update_cloud->id()));

                    // 3. Point Size (SpinBox)
                    QSpinBox *point_size = new QSpinBox;
                    point_size->setRange(1, 99);
                    point_size->setValue(update_cloud->pointSize());
                    point_size->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
                    connect(point_size, QOverload<int>::of(&QSpinBox::valueChanged),
                            this, [=](int value){
                                update_cloud->setPointSize(value);
                                m_cloudview->setPointCloudSize(QString::fromStdString(update_cloud->id()), value);
                                m_cloudview->refresh();
                            });
                    m_table->setCellWidget(4, 1, point_size);

                    // 4. Opacity (DoubleSpinBox)
                    QDoubleSpinBox* opacity = new QDoubleSpinBox;
                    opacity->setSingleStep(0.1);
                    opacity->setRange(0, 1);
                    opacity->setValue(update_cloud->opacity());
                    opacity->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
                    connect(opacity, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                            this, [=](double value){
                                update_cloud->setOpacity(value);
                                m_cloudview->setPointCloudOpacity(QString::fromStdString(update_cloud->id()), value);
                                m_cloudview->refresh();
                            });
                    m_table->setCellWidget(5, 1, opacity);

                    // 5. Normals (Checkbox + Scale)
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

                    show_normals->setEnabled(update_cloud->hasNormals());

                    // 连接信号
                    connect(scale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                            [=](double value){
                                if (update_cloud->hasNormals() && show_normals->isChecked())
                                    m_cloudview->addPointCloudNormals(update_cloud, 1, value);
                            });
                    connect(show_normals, &QCheckBox::stateChanged,
                            [=](int state){
                                if (state) m_cloudview->addPointCloudNormals(update_cloud, 1, scale->value());
                                else m_cloudview->removeShape(QString::fromStdString(update_cloud->normalId()));
                            });

                    layout->addWidget(show_normals);
                    layout->addWidget(scale);
                    layout->addStretch();
                    m_table->setCellWidget(6, 1, normals_widget);

                    // 6. Color Mode (ComboBox)
                    if (m_table->rowCount() <= COLOR_MODE_ROW) m_table->setRowCount(COLOR_MODE_ROW + 1);

                    QComboBox* color_mode = new QComboBox;
                    color_mode->addItem("RGB (Default)");
                    color_mode->addItem("x");
                    color_mode->addItem("y");
                    color_mode->addItem("z");

                    // 添加标量场字段
                    std::vector<std::string> fields = update_cloud->getScalarFieldNames();
                    for (const std::string& f : fields) color_mode->addItem(QString::fromStdString(f));

                    QString currentMode = QString::fromStdString(update_cloud->currentColorMode());
                    // 尝试在列表中找到当前模式
                    int idx = color_mode->findText(currentMode);
                    if (idx >= 0) {
                        color_mode->setCurrentIndex(idx);
                    } else {
                        // 如果没找到（异常情况），默认选 RGB
                        color_mode->setCurrentIndex(0);
                    }

                    color_mode->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

                    connect(color_mode, &QComboBox::currentTextChanged,
                            [=](const QString& text){
                                if (text == "RGB (Default)") {
                                    m_cloudview->resetPointCloudColor(update_cloud);
                                } else if (text == "x" || text == "y" || text == "z") {
                                    update_cloud->setCloudColor(text.toLower().toStdString());
                                    m_cloudview->addPointCloud(update_cloud);
                                } else {
                                    update_cloud->updateColorByField(text.toStdString());
                                    m_cloudview->addPointCloud(update_cloud);
                                }
                            });
                    m_table->setCellWidget(COLOR_MODE_ROW, 1, color_mode);
                }
            }
        }

        m_cloudview->setAutoRender(true);
        m_cloudview->refresh();
    }

    void CloudTree::showProgress(const QString &message) {
        if (m_script_mode) return;  // 脚本模式：跳过进度条弹窗
        if (!m_processing_dialog){
            // 寻找最顶层的窗口作为父窗口，确保模态对话框居中显示
            QWidget* topLevel = this->window();
            m_processing_dialog = new ProcessingDialog(topLevel);
            m_processing_dialog->setWindowModality(Qt::WindowModal);
        }

        m_processing_dialog->reset();
        m_processing_dialog->setMessage(message);
        m_processing_dialog->show();
        QApplication::processEvents(); // 强制刷新UI
    }

    void CloudTree::closeProgress() {
        m_loading_queue_count = 0;
        if (m_processing_dialog){
            m_processing_dialog->close();
            delete m_processing_dialog;
            m_processing_dialog = nullptr;
        }
    }

    void CloudTree::setProgress(int percent) {
        if (m_processing_dialog)
            m_processing_dialog->setProgress(percent);
    }

    void CloudTree::bindWorker(QObject *worker) {
        if (!m_processing_dialog || !worker) return;

        // Worker -> Dialog (进度更新)
        connect(worker, SIGNAL(progress(int)), m_processing_dialog, SLOT(setProgress(int)), Qt::UniqueConnection);

        // Dialog -> Worker (取消请求),信号和槽连接方式为直接连接，确保能够快速响应取消请求
        connect(m_processing_dialog, SIGNAL(cancelRequested()), worker, SLOT(cancel()), Qt::DirectConnection);

        connect(m_processing_dialog, &ProcessingDialog::cancelRequested, this, &CloudTree::closeProgress);
    }

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
                    m_cloud_map.remove(ch);
                    m_item_by_id.remove(ch->text(0));
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
        m_mesh_map[cloudId] = mesh;

        QTreeWidgetItem* item = getItemById(cloudId);
        if (item) {
            item->setData(0, NodeMeshIdRole, cloudId);
        }
        m_cloudview->addPolygonMesh(mesh, cloudId);
    }

    void CloudTree::unregisterMesh(const QString& cloudId)
    {
        m_mesh_map.remove(cloudId);
        m_cloudview->removePolygonMesh(cloudId);
        m_cloudview->removeShape(cloudId);

        QTreeWidgetItem* item = getItemById(cloudId);
        if (item) {
            item->setData(0, NodeMeshIdRole, QVariant());
        }
    }

    QList<QPair<QString, pcl::PolygonMesh::Ptr>> CloudTree::getLoadedMeshes() const
    {
        QList<QPair<QString, pcl::PolygonMesh::Ptr>> result;
        for (auto it = m_mesh_map.constBegin(); it != m_mesh_map.constEnd(); ++it) {
            result.append(qMakePair(it.key(), it.value()));
        }
        return result;
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
        m_clouds_in_use.insert(id);
    }

    void CloudTree::unmarkCloudInUse(const QString& id)
    {
        m_clouds_in_use.remove(id);
    }

    void CloudTree::releaseAllInUse()
    {
        m_clouds_in_use.clear();
    }

    void CloudTree::onFieldMappingRequested(const QList<ct::FieldInfo>& fields, std::map<std::string, std::string>& result)
    {
        if (m_script_mode) {
            // 脚本模式：智能默认映射（与 FieldMappingDialog 预选逻辑一致）
            for (const auto& f : fields) {
                QString name = QString::fromStdString(f.name).toLower();
                if (name == "x" || name == "y" || name == "z") {
                    result[f.name] = "Axis " + QString::fromStdString(f.name).toUpper().toStdString();
                } else if (name == "rgba" || name == "rgb") {
                    result[f.name] = "Color(Packed)";
                } else if (name.contains("red") || name == "r") {
                    result[f.name] = "Red";
                } else if (name.contains("green") || name == "g") {
                    result[f.name] = "Green";
                } else if (name.contains("blue") || name == "b") {
                    result[f.name] = "Blue";
                } else if (name == "normal_x" || name == "nx") {
                    result[f.name] = "Normal X";
                } else if (name == "normal_y" || name == "ny") {
                    result[f.name] = "Normal Y";
                } else if (name == "normal_z" || name == "nz") {
                    result[f.name] = "Normal Z";
                } else if (name == "curvature") {
                    result[f.name] = "Curvature";
                } else if (name == "intensity") {
                    result[f.name] = "Intensity";
                } else {
                    result[f.name] = "Scalar Field";
                }
            }
            return;
        }
        // 这个函数运行在主线程 (UI线程)
        FieldMappingDialog dlg(fields, this);
        if (dlg.exec() == QDialog::Accepted) {
            // 用户点击 OK，获取结果
            ct::MappingResult res = dlg.getMapping();
            result = std::move(res.field_map);
        } else {
            // 用户取消，返回空结果
            result.clear();
        }
    }

    void CloudTree::onTxtImportRequested(const QStringList& preview_lines, ct::TxtImportParams& params){
        if (m_script_mode) {
            // 脚本模式：使用默认 TXT 导入参数
            return;
        }
        TxtImportDialog dlg(preview_lines, this);
        if (dlg.exec() == QDialog::Accepted) {
            params = dlg.getParams();
        } else{
            params.col_map.clear();
        }
    }

    void CloudTree::onTxtExportRequested(const QStringList &available_fields, ct::TxtExportParams &params) {
        if (m_script_mode) {
            return;
        }
        TxtExportDialog dlg(available_fields, this);
        if (dlg.exec() == QDialog::Accepted){
            params = dlg.getParams();
        }
        else{
            params.selected_fields.clear(); //标志取消
        }
    }

    void CloudTree::onGlobalFilterRequested(const Eigen::Vector3d &min_pt, Eigen::Vector3d &suggested_shift,
                                            bool &skipped) {
        if (m_script_mode) {
            // 脚本模式：直接使用建议的偏移值
            m_last_shift = suggested_shift;
            m_hasLastShift = true;
            skipped = false;
            return;
        }
        GlobalShiftDialog dlg(min_pt, suggested_shift, m_last_shift, m_hasLastShift, this->window());

        if (dlg.exec() == QDialog::Accepted){
            suggested_shift = dlg.getShiftValue();
            m_last_shift = suggested_shift;
            m_hasLastShift = true;
            skipped = false;
        }
        else{
            skipped = dlg.isSkipped();
            if (!skipped && dlg.result() == QDialog::Rejected) skipped = true;
        }
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

