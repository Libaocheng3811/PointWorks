#ifndef CT_TOOL_SURFACE_H
#define CT_TOOL_SURFACE_H

// DEPRECATED: This file is deprecated.
// Use src/tool/mesh/*.h instead (ReconstructSurfaceDialog, ComputeHullDialog, ExtractBoundaryDialog)

#include "ui/base/customdialog.h"
#include "algorithm/surface.h"

#include <QFutureWatcher>
#include <QtConcurrent>

QT_BEGIN_NAMESPACE
namespace Ui {
    class Surface;
}
QT_END_NAMESPACE

#define SURFACE_PRE_FLAG              "-surface"

class Surface : public ct::CustomDialog {
    Q_OBJECT

public:
    explicit Surface(QWidget* parent = nullptr);

    ~Surface() override;

    void preview();
    void reset() override;

private:
    void handleSurfaceResult(const std::string& source_id, const ct::SurfaceResult& result);
    void runSurface(const std::string& source_id, std::function<ct::SurfaceResult(std::function<void(int)>)> fn);

    Ui::Surface* ui;
    QFutureWatcher<ct::SurfaceResult>* m_watcher = nullptr;
    std::map<std::string, ct::PolygonMesh::Ptr> m_surface_map;
    std::atomic<bool> m_cancel{false};
};

#endif // CT_TOOL_SURFACE_H
