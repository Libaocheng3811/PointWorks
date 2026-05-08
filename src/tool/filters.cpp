//
// Created by LBC on 2026/1/4.
//

// You may need to build the project (run Qt uic code generator) to get "ui_Filters.h" resolved

#include "filters.h"
#include "base/cloudtree.h"
#include "ui_filters.h"

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>
#include <QTimer>

#define FILTER_TYPE_PassThrough                         (0)
#define FILTER_TYPE_VoxelGrid                           (1)
#define FILTER_TYPE_StatisticalOutlierRemoval           (2)
#define FILTER_TYPE_RadiusOutlierRemoval                (3)
#define FILTER_TYPE_ConditionalRemoval                  (4)
#define FILTER_TYPE_GridMinimum                         (5)
#define FILTER_TYPE_LocalMaximum                        (6)
#define FILTER_TYPE_ShadowPoints                        (7)

#define FILTER_PRE_FLAG                     "-filter"
#define FILTER_ADD_FLAG                     "filtered-"

Filters::Filters(QWidget *parent) :
        CustomDialog(parent), ui(new Ui::Filters)
{
    ui->setupUi(this);

    connect(ui->btn_preview, &QPushButton::clicked, this, &Filters::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Filters::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Filters::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Filters::reset);

    // ConditionalRemoval
    connect(ui->btn_add_con, &QPushButton::clicked, [=]
    {
        // 获取当前表格行数
        int row = ui->table_condition->rowCount();
        // 增加表格行
        ui->table_condition->setRowCount(row + 1);

        // 如果当前新增行是表格第一行
        if (row == 0)
        {
            QComboBox* con = new QComboBox;
            con->addItems(QStringList({"And", "Or"}));
            // 将下拉框放置在表格的第一行第一列
            ui->table_condition->setCellWidget(row, 0, con);
        }
        // 如果是表格的第二行
        else if (row == 1)
        {
            // 获取第一行第一列单元格小部件，并将其转为QComboBox类型
            QComboBox* con = (QComboBox*)ui->table_condition->cellWidget(0, 0);

            if (con->currentText() == "And")
            {
                // 移除第一行第一列的下拉框，将第一行，第二行的第一列均设置为不可选项QTableWidgetItem，设置内容与con一致
                ui->table_condition->removeCellWidget(0, 0);
                ui->table_condition->setItem(0, 0, new QTableWidgetItem("And"));
                ui->table_condition->setItem(row, 0, new QTableWidgetItem("And"));
            }
            else
            {
                ui->table_condition->removeCellWidget(0, 0);
                ui->table_condition->setItem(0, 0, new QTableWidgetItem("Or"));
                ui->table_condition->setItem(row, 0, new QTableWidgetItem("Or"));
            }
        }
        // 如果新增行是第三行及之后
        else
        {
            // 与上面的行设置相同的属性即可
            if (ui->table_condition->item(0, 0)->text() == "And")
                ui->table_condition->setItem(row, 0, new QTableWidgetItem("And"));
            else
                ui->table_condition->setItem(row, 0, new QTableWidgetItem("'Or"));
        }
        // 设置其他列的项
        QComboBox* com = new QComboBox;
        com->addItems(QStringList({"x", "y", "z", "r", "g", "b"}));
        ui->table_condition->setCellWidget(row, 1, com);

        QComboBox* op = new QComboBox;
        op->addItems(QStringList({">", ">=", "<", "<=", "="}));
        ui->table_condition->setCellWidget(row, 2, op);

        QDoubleSpinBox* value = new QDoubleSpinBox;
        value->setDecimals(3);
        value->setRange(-99999, 99999);
        ui->table_condition->setCellWidget(row, 3, value);
    });
    connect(ui->btn_clear_con, &QPushButton::clicked, [=]
    {
        // 获取当前行数，并移除最后一行
        int row = ui->table_condition->rowCount();
        ui->table_condition->removeRow(row - 1);
    });


    // PassThrough
    // 将spinbox中的值传递到滑动条
    connect(ui->dspin_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
    {
        ui->slider_min->setValue(value * 1000);
    });

    /**
     * @brief 总结：信号&QDoubleSpinBox::valueChanged的类型是void(QDoubleSpinBox::*)(double)，类型是一个成员函数指针
     * lambda函数的类型是void，类型是一个普通的函数对象，
     * 那为什么类型不一致却可以这么写呢？---因为Qt会在后台进行适当的转换，它能正确地处理普通函数对象（lambda）和成员函数指针之间的关系，所以即使信号和槽的类型形式不完全一致
     */
    connect(ui->dspin_max, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
    {
        ui->slider_max->setValue(value * 1000);
    });
    connect(ui->slider_min, &QSlider::valueChanged, [=](int value)
    {
        ui->dspin_min->setValue((float)value / 1000);
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });
    connect(ui->slider_max, &QSlider::valueChanged, [=](int value)
    {
        ui->dspin_max->setValue((float)value / 1000);
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });

    // VoxelGrid
    connect(ui->check_same_value, &QCheckBox::stateChanged, [=](int state)
    {
        // 如果勾选check_same_value复选框，state为true，则禁用dspin_leafy和dspin_leafz；否则启用。
        if (state)
        {
            ui->dspin_leafy->setEnabled(false);
            ui->dspin_leafz->setEnabled(false);
        }
        else
        {
            ui->dspin_leafy->setEnabled(true);
            ui->dspin_leafz->setEnabled(true);
        }
    });

    connect(ui->dspin_leafx, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
    {
        // 如果勾选了相同值复选框，设置y,z的值与x相同
        if (ui->check_same_value->isChecked())
        {
            ui->dspin_leafy->setValue(value);
            ui->dspin_leafz->setValue(value);
        }
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });
    connect(ui->dspin_leafy, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (!ui->check_same_value->isChecked() && ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });
    connect(ui->dspin_leafz, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (!ui->check_same_value->isChecked() && ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });
    connect(ui->check_approximate, &QCheckBox::stateChanged, [=](int state)
    {
        if (state)
            ui->check_reverse->setEnabled(false);
        else
            ui->check_reverse->setEnabled(true);
    });

    // StatisticalOutlierRemoval
    connect(ui->spin_meank, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=]
    {
        // 同步更新滤波结果
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });
    connect(ui->dspin_stddevmulthresh, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });

    // RadiusOutlierRemoval
    connect(ui->dspin_radius, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });
    connect(ui->spin_minneiborsinradius, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });

    // GridMinimum
    connect(ui->dspin_resolution, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });

    // LocalMaximum
    connect(ui->dspin_radius_3, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });

    // ShadowPoints
    connect(ui->dspin_threshold, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) QTimer::singleShot(300, this, &Filters::preview);
    });

    connect(ui->cbox_field_name, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Filters::getRange);

    ui->cbox_type->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    ui->table_condition->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->check_refresh->setChecked(false);
}

Filters::~Filters() {
    delete ui;
}

void Filters::runFilter(std::function<pw::FilterResult()> filterFn, bool show_progress)
{
    // 始终异步执行，禁止主线程阻塞
    if (show_progress) {
        m_progress->showProgress("Filtering PointCloud...");
    }

    m_cancel = false;
    if (m_progress->dialog()) {
        connect(m_progress, &pw::ProgressManager::cancelRequested,
                this, [this]() { m_cancel = true; }, Qt::UniqueConnection);
    }

    // 进度回调：跨线程安全地更新进度条
    auto on_progress = [this](int pct) {
        if (m_progress->dialog()) {
            QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                      Qt::QueuedConnection, Q_ARG(int, pct));
        }
    };

    // 终止上一次未完成的预览任务
    if (m_preview_watcher && !m_preview_watcher->isFinished()) {
        m_cancel = true;
        m_preview_watcher->waitForFinished();
    }

    auto future = QtConcurrent::run(filterFn);
    if (!m_preview_watcher) {
        m_preview_watcher = new QFutureWatcher<pw::FilterResult>(this);
    }
    m_preview_watcher->setFuture(future);

    connect(m_preview_watcher, &QFutureWatcher<pw::FilterResult>::finished, this, [=]() {
        if (show_progress) m_progress->closeProgress();
        auto result = m_preview_watcher->result();
        handleFilterResult(result);
    });
}

void Filters::handleFilterResult(const pw::FilterResult& result)
{
    if (!result.result_cloud) return;

    auto cloud = result.result_cloud;

    printI(QString("Filter cloud[id:%1] done, take time %2 ms.").arg(QString::fromStdString(cloud->id())).arg(result.time_ms));
    std::string id = cloud->id();
    cloud->setId(id + FILTER_PRE_FLAG);

    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudColor(QString::fromStdString(cloud->id()), pw::Color::Green);
    m_cloudview->setPointCloudSize(QString::fromStdString(cloud->id()), cloud->pointSize() + 2);
    m_filter_map[id] = cloud;
}

void Filters::preview()
{
    std::vector<pw::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }

    bool show_progress = !ui->check_refresh->isChecked();
    bool negative = ui->check_reverse->isChecked();

    for (auto& cloud : selected_clouds)
    {
        // 进度回调（用于异步模式）
        auto on_progress = [this](int pct) {
            if (m_progress->dialog()) {
                QMetaObject::invokeMethod(m_progress->dialog(), "setProgress",
                                          Qt::QueuedConnection, Q_ARG(int, pct));
            }
        };

        // 判断滤波类型，构建对应的静态方法调用
        switch (ui->cbox_type->currentIndex()) {
            case FILTER_TYPE_PassThrough:
            {
                m_cloudview->showInfo("PassThrough", 1);
                std::string field = ui->cbox_field_name->currentText().toStdString();
                float min_val = (float)ui->slider_min->value() / 1000;
                float max_val = (float)ui->slider_max->value() / 1000;
                runFilter([cloud, field, min_val, max_val, negative, this, on_progress]() {
                    return pw::Filters::PassThrough(cloud, field, min_val, max_val, negative, &m_cancel, on_progress);
                }, show_progress);
                break;
            }
            case FILTER_TYPE_VoxelGrid:
            {
                m_cloudview->showInfo("VoxelGrid", 1);
                float lx = ui->dspin_leafx->value();
                float ly = ui->dspin_leafy->value();
                float lz = ui->dspin_leafz->value();
                runFilter([cloud, lx, ly, lz, negative, this, on_progress]() {
                    return pw::Filters::VoxelGrid(cloud, lx, ly, lz, negative, &m_cancel, on_progress);
                }, show_progress);
                break;
            }
            case FILTER_TYPE_StatisticalOutlierRemoval:
            {
                m_cloudview->showInfo("StatisticalOutlierRemoval", 1);
                int nr_k = ui->spin_meank->value();
                double stddev = ui->dspin_stddevmulthresh->value();
                runFilter([cloud, nr_k, stddev, negative, this, on_progress]() {
                    return pw::Filters::StatisticalOutlierRemoval(cloud, nr_k, stddev, negative, &m_cancel, on_progress);
                }, show_progress);
                break;
            }
            case FILTER_TYPE_RadiusOutlierRemoval:
            {
                m_cloudview->showInfo("RadiusOutlierRemoval", 1);
                double radius = ui->dspin_radius->value();
                int min_pts = ui->spin_minneiborsinradius->value();
                runFilter([cloud, radius, min_pts, negative, this, on_progress]() {
                    return pw::Filters::RadiusOutlierRemoval(cloud, radius, min_pts, negative, &m_cancel, on_progress);
                }, show_progress);
                break;
            }
            case FILTER_TYPE_ConditionalRemoval:
            {
                m_cloudview->showInfo("ConditionalRemoval", 1);
                auto con = this->getCondition();
                runFilter([cloud, con, negative, this, on_progress]() {
                    return pw::Filters::ConditionalRemoval(cloud, con, negative, &m_cancel, on_progress);
                }, show_progress);
                break;
            }
            case FILTER_TYPE_GridMinimum:
            {
                m_cloudview->showInfo("GridMinimum", 1);
                float resolution = ui->dspin_resolution->value();
                runFilter([cloud, resolution, negative, this, on_progress]() {
                    return pw::Filters::GridMinimun(cloud, resolution, negative, &m_cancel, on_progress);
                }, show_progress);
                break;
            }
            case FILTER_TYPE_LocalMaximum:
            {
                m_cloudview->showInfo("LocalMaximum", 1);
                float radius = ui->dspin_radius_3->value();
                runFilter([cloud, radius, negative, this, on_progress]() {
                    return pw::Filters::LocalMaximum(cloud, radius, negative, &m_cancel, on_progress);
                }, show_progress);
                break;
            }
            case FILTER_TYPE_ShadowPoints:
            {
                if (cloud->hasNormals())
                {
                    printW("Please estimate normals first!");
                    return;
                }
                m_cloudview->showInfo("ShadowPoints", 1);
                float threshold = ui->dspin_threshold->value();
                runFilter([cloud, threshold, negative, this, on_progress]() {
                    return pw::Filters::ShadowPoints(cloud, threshold, negative, &m_cancel, on_progress);
                }, show_progress);
                break;
            }
        }
    }
}

void Filters::add()
{
    std::vector<pw::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        return;
    }
    for (auto & cloud : selected_clouds)
    {
        if (m_filter_map.find(cloud->id()) == m_filter_map.end())
        {
            printW(QString("The cloud[id:1%] has no filtered cloud!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        pw::Cloud::Ptr new_cloud = m_filter_map.find(cloud->id())->second;
        // 从视图器中移除旧的过滤点云
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
        // 为过滤后的点云设置新的ID
        new_cloud->setId(FILTER_ADD_FLAG + cloud->id());
        // 策略一：滤波结果作为兄弟节点挂载
        QTreeWidgetItem * item = m_cloudtree->getItemById(QString::fromStdString(cloud->id()));
        m_cloudtree->insertCloud(new_cloud, item, true, pw::MountStrategy::Sibling);

        m_filter_map.erase(cloud->id());
        printI(QString("Add filtered cloud[id:1%] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Filters::apply()
{
    std::vector<pw::Cloud::Ptr > selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto & cloud : selected_clouds)
    {
        if (m_filter_map.find(cloud->id()) == m_filter_map.end())
        {
            printI(QString("The cloud[id:1%] has no filtered cloud!").arg(QString::fromStdString(cloud->id())));
            continue;
        }
        pw::Cloud::Ptr new_cloud = m_filter_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(QString::fromStdString(new_cloud->id()));
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_filter_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply filtered cloud[id:1%] done.").arg(QString::fromStdString(new_cloud->id())));
    }
    m_cloudview->clearInfo();
}

void Filters::reset()
{
    for (auto &cloud : m_cloudtree->getSelectedClouds())
    {
        m_cloudtree->setCloudChecked(cloud);
    }
    for (auto &cloud : m_filter_map)
        m_cloudview->removePointCloud(QString::fromStdString(cloud.second->id()));
    m_filter_map.clear();
    m_cloudview->clearInfo();
}

pw::ConditionBase::Ptr Filters::getCondition()
{
    int rowCount = ui->table_condition->rowCount();
    QString condition;
    if (rowCount == 0) return nullptr;
    // 只有一行的情况下，第一行一列单元格控件的类型是QComboBox,而当大于一行的情况下，第一列单元格控件的类型是QWidgetItem
    else if (rowCount == 1)
        condition = ((QComboBox*)ui->table_condition->cellWidget(0, 0))->currentText();
    else
        condition = ui->table_condition->item(0, 0)->text();
    std::string field;
    pw::CompareOp op;
    double value;
    if (condition == "And")
    {
        // pw::ConditionAnd是一个逻辑与条件，创建一个新对象add_cond
        pw::ConditionAnd::Ptr add_cond(new pw::ConditionAnd);
        for (int i = 0; i < rowCount; i++)
        {
            field = ((QComboBox*)ui->table_condition->cellWidget(i, 1))->currentText().toStdString();
            // op是从第i行第2列的单元格中的QComboBox获取的当前索引，并转换为ct::CompareOp枚举类型。
            op = pw::CompareOp(((QComboBox*)ui->table_condition->cellWidget(i, 2))->currentIndex());
            value = ((QDoubleSpinBox*)ui->table_condition->cellWidget(i, 3))->value();
            // fieldcomp是一个ct::FieldComparison对象，是一个条件比较对象，由字段名、比较操作符和比较值构造而成。
            pw::FieldComparison::Ptr fieldcomp(new pw::FieldComparison(field, op, value));
            // 将条件比较对象添加到逻辑对象中
            add_cond->addComparison(fieldcomp);
        }
        return add_cond;
    }
    else
    {
        pw::ConditionOr::Ptr or_cond(new pw::ConditionOr );
        for (int i = 0; i < rowCount; i++)
        {
            field = ((QComboBox*)ui->table_condition->cellWidget(i, 1))->currentText().toStdString();
            op = pw::CompareOp(((QComboBox*)ui->table_condition->cellWidget(i, 2))->currentIndex());
            value = ((QDoubleSpinBox*)ui->table_condition->cellWidget(i, 3))->value();
            pw::FieldComparison::Ptr fieldcomp(new pw::FieldComparison(field, op, value));
            or_cond->addComparison(fieldcomp);
        }
        return or_cond;
    }
}

void Filters::getRange(int index)
{
    std::vector<pw::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    // 将min初始化为float类型的最大值、将max初始化为负的float类型最大值
    float min = std::numeric_limits<float>::max(), max = -std::numeric_limits<float>::max();
    // 依次获取所选点云所设置字段的范围，并将其设置为滑动条的范围
    for (auto& cloud : selected_clouds)
    {
        switch (index)
        {
            case 0://x
                min = cloud->min().x < min ? cloud->min().x : min;
                max = cloud->max().x > max ? cloud->max().x : max;
                break;
            case 1://y
                min = cloud->min().y < min ? cloud->min().y : min;
                max = cloud->max().y > max ? cloud->max().y : max;
                break;
            case 2://z
                min = cloud->min().z < min ? cloud->min().z : min;
                max = cloud->max().z > max ? cloud->max().z : max;
                break;
            case 3://rgb
                min = 0, max = 1;
                break;
            case 4://curvature
                min = 0, max = 1;
                break;
        }
        min *= 1000, max *= 1000;
        ui->slider_min->setRange(min, max);
        ui->slider_max->setRange(min, max);
        ui->slider_min->setValue(min);
        ui->slider_max->setValue(max);
    }
}
