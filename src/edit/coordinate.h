#ifndef CT_EDIT_COORDINATE_H
#define CT_EDIT_COORDINATE_H

#include "ui/base/customdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
    class Coordinate;
}
QT_END_NAMESPACE

#define COORDINATE_ADD_FLAG   "-coord"

class Coordinate : public ct::CustomDialog {
    Q_OBJECT

public:
    explicit Coordinate(QWidget* parent = nullptr);
    ~Coordinate() override;

    void init() override;
    void add();
    void reset() override;
    void deinit() override;
    void addCoord();
    void closeCoord();

private:
    bool parseMatrixText(const QString& text, Eigen::Affine3f& t);

private:
    Ui::Coordinate* ui;
    ct::Coord m_origin_coord;
    std::map<std::string, ct::Coord> m_coord_map;
};

#endif // CT_EDIT_COORDINATE_H
