#ifndef CT_TOOL_BOUNDARY_H
#define CT_TOOL_BOUNDARY_H

#include "ui/base/customdialog.h"
#include "algorithm/features.h"

#include <QFutureWatcher>
#include <QtConcurrent>

QT_BEGIN_NAMESPACE
namespace Ui {
    class Boundary;
}
QT_END_NAMESPACE

#define BOUNDARY_PRE_FLAG              "-boundary"
#define BOUNDARY_ADD_FLAG              "boundary-"

class Boundary : public ct::CustomDialog {
    Q_OBJECT

public:
    explicit Boundary(QWidget* parent = nullptr);

    ~Boundary() override;

    void preview();
    void add();
    void apply();
    void reset() override;

private:
    void handleBoundaryResult(const std::string& source_id, const ct::Cloud::Ptr& boundary_cloud, float time_ms);
    void runBoundary(const std::string& source_id, std::function<ct::Cloud::Ptr(std::function<void(int)>)> fn);

    Ui::Boundary* ui;
    QFutureWatcher<ct::Cloud::Ptr>* m_watcher = nullptr;
    std::map<std::string, ct::Cloud::Ptr> m_boundary_map;
    std::atomic<bool> m_cancel{false};
};

#endif // CT_TOOL_BOUNDARY_H
