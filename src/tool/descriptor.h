//
// Created by LBC on 2025/1/10.
//

#ifndef CLOUDTOOL2_DESCRIPTOR_H
#define CLOUDTOOL2_DESCRIPTOR_H

#include "ui/base/customdialog.h"

#include "algorithm/features.h"

#include <pcl/visualization/pcl_plotter.h>

#include <QFutureWatcher>
#include <QtConcurrent>

QT_BEGIN_NAMESPACE
namespace Ui {
    class Descriptor;
}
QT_END_NAMESPACE

class Descriptor : public ct::CustomDialog {
Q_OBJECT

public:
    explicit Descriptor(QWidget *parent = nullptr);

    ~Descriptor();

    void preview();

    virtual void reset();

    ct::FeatureType::Ptr getDescriptor(const std::string &id) {
        if (m_descriptor_map.find(id) == m_descriptor_map.end())
            return nullptr;
        else
            return m_descriptor_map.find(id)->second;
    }

private:
    Ui::Descriptor *ui;
    // PCLPlotter 用于绘制直方图、散点图、曲线等二维图形
    pcl::visualization::PCLPlotter::Ptr m_plotter;
    std::map<std::string, ct::FeatureType::Ptr> m_descriptor_map;
    std::map<std::string, ct::ReferenceFrame::Ptr> m_lrf_map;

    void handleFeatureResult(const ct::FeatureResult& result);
    void handleLrfResult(const ct::LRFResult& result);
};


#endif //CLOUDTOOL2_DESCRIPTOR_H
