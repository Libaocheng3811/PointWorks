#ifndef PW_EDIT_COORDINATE_H
#define PW_EDIT_COORDINATE_H

#include "ui/base/customdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
    class Coordinate;
}
QT_END_NAMESPACE

#define COORDINATE_ADD_FLAG   "-coord"

class Coordinate : public pw::CustomDialog {
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
    pw::Coord m_origin_coord;
    std::map<std::string, pw::Coord> m_coord_map;
};

#endif // PW_EDIT_COORDINATE_H
