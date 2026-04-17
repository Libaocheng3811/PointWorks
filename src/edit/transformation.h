#ifndef CT_EDIT_TRANSFORMATION_H
#define CT_EDIT_TRANSFORMATION_H

#include "ui/base/customdialog.h"
#include "core/common.h"

QT_BEGIN_NAMESPACE
namespace Ui {
    class Transformation;
}
QT_END_NAMESPACE

#define TRANS_PRE_FLAG          "-trans"
#define TRANS_ADD_FLAG          "transed-"

class Transformation : public ct::CustomDialog {
    Q_OBJECT

public:
    explicit Transformation(QWidget* parent = nullptr);

    ~Transformation() override;

    void add();
    void apply();
    void reset() override;

private:
    void preview(const Eigen::Affine3f& affine3f);
    void syncUI(const Eigen::Affine3f& affine3f);
    bool parseMatrixText(const QString& text, Eigen::Affine3f& t);
    bool parseEulerText(const QString& text, Eigen::Affine3f& t);

    Ui::Transformation* ui;
    Eigen::Affine3f m_affine;
    std::map<std::string, Eigen::Affine3f> m_trans_map;
};

#endif // CT_EDIT_TRANSFORMATION_H
