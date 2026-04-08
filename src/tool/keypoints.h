#ifndef POINTWORKS_KEYPOINTS_H
#define POINTWORKS_KEYPOINTS_H

#include "ui/base/customdialog.h"
#include "algorithm/keypoints.h"
#include "tool/rangeimage.h"

#include <QFutureWatcher>
#include <QtConcurrent>

QT_BEGIN_NAMESPACE
namespace Ui {
    class KeyPoints;
}
QT_END_NAMESPACE

#define KEYPOINTS_PRE_FLAG                  "-keypoints"
#define KEYPOINTS_ADD_FLAG                  "keypoints-"

class KeyPoints : public ct::CustomDialog {
Q_OBJECT

public:
    explicit KeyPoints(QWidget* parent = nullptr);

    ~KeyPoints() override;

    void setRangeImage(RangeImage* rangeimage) {
        m_rangeimage = rangeimage;
        if (rangeimage) connect(rangeimage, &RangeImage::destroyed, [=] { m_rangeimage = nullptr; });
    }

    void preview();
    void add();
    void apply();
    void reset() override;

private:
    void handleKeypointResult(const ct::KeypointResult& result);
    void runKeypoint(std::function<ct::KeypointResult()> fn);

    Ui::KeyPoints* ui;
    RangeImage* m_rangeimage;
    QFutureWatcher<ct::KeypointResult>* m_watcher = nullptr;
    std::map<std::string, ct::Cloud::Ptr> m_keypoints_map;
    std::atomic<bool> m_cancel{false};
};

#endif // POINTWORKS_KEYPOINTS_H
