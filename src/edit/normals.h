#ifndef CT_EDIT_NORMALS_H
#define CT_EDIT_NORMALS_H

#include "ui/base/customdialog.h"
#include "algorithm/normals.h"

#include <QFutureWatcher>
#include <QtConcurrent>

QT_BEGIN_NAMESPACE
namespace Ui {
    class Normals;
}
QT_END_NAMESPACE

#define NORMALS_ADD_FLAG    "normals-"

class Normals : public ct::CustomDialog {
    Q_OBJECT

public:
    explicit Normals(QWidget* parent = nullptr);
    ~Normals() override;

    void preview();
    void add();
    void apply();
    void reset() override;

    void reverseNormals();
    void updateNormals();

private:
    void handleNormalsResult(const std::string& source_id);
    void runNormals(const std::string& source_id, const ct::Cloud::Ptr& cloud,
                    float vpx, float vpy, float vpz);

    Ui::Normals* ui;
    QFutureWatcher<ct::NormalsResult>* m_watcher = nullptr;
    std::map<std::string, ct::Cloud::Ptr> m_normals_map;
    std::atomic<bool> m_cancel{false};
};

#endif // CT_EDIT_NORMALS_H
