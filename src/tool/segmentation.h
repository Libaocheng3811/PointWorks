#ifndef CT_TOOL_SEGMENTATION_H
#define CT_TOOL_SEGMENTATION_H

// DEPRECATED: This file is deprecated.
// Use src/tool/segmentation/*.h instead (ShapeDetectionDialog, ClusteringDialog, etc.)

#include "ui/base/customdialog.h"
#include "algorithm/segmentation.h"

#include <QFutureWatcher>
#include <QtConcurrent>

QT_BEGIN_NAMESPACE
namespace Ui {
    class Segmentation;
}
QT_END_NAMESPACE

#define SEGMENTATION_PRE_FLAG              "-seg"
#define SEGMENTATION_ADD_FLAG              "seg-"

class Segmentation : public ct::CustomDialog {
Q_OBJECT

public:
    explicit Segmentation(QWidget* parent = nullptr);

    ~Segmentation() override;

    void preview();
    void add();
    void apply();
    void reset() override;

private:
    void handleSegmentationResult(const std::string& source_id, const ct::SegmentationResult& result);
    void runSegmentation(const std::string& source_id, std::function<ct::SegmentationResult()> fn);

    Ui::Segmentation* ui;
    QFutureWatcher<ct::SegmentationResult>* m_watcher = nullptr;
    std::map<std::string, std::vector<ct::Cloud::Ptr>> m_segmentation_map;
    std::atomic<bool> m_cancel{false};
};

#endif // CT_TOOL_SEGMENTATION_H
