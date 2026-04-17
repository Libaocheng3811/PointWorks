#include "coordinate.h"
#include "ui_coordinate.h"

#include <QRegularExpression>

Coordinate::Coordinate(QWidget* parent)
    : CustomDialog(parent), ui(new Ui::Coordinate)
{
    ui->setupUi(this);
    connect(ui->btn_add, &QPushButton::clicked, this, &Coordinate::add);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Coordinate::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Coordinate::close);
    connect(ui->btn_add_coord, &QPushButton::clicked, this, &Coordinate::addCoord);
    connect(ui->btn_close_coord, &QPushButton::clicked, this, &Coordinate::closeCoord);
    connect(ui->btn_expand, &QPushButton::clicked, [=] {
        if (ui->widget->isHidden()) {
            ui->widget->show();
            ui->btn_expand->setIcon(QIcon(":/res/icon/collapse-text-input.svg"));
            this->setFixedHeight(155);
        } else {
            ui->widget->hide();
            ui->btn_expand->setIcon(QIcon(":/res/icon/expand-text-input.svg"));
            this->setFixedHeight(38);
        }
    });
    ui->widget->hide();
    this->setFixedHeight(38);
}

Coordinate::~Coordinate() { delete ui; }

void Coordinate::init()
{
    connect(ui->dspin_scale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double value) {
                m_origin_coord.scale = value;
                m_cloudview->addCoordinateSystem(m_origin_coord);
            });
    m_cloudview->addCoordinateSystem(m_origin_coord);
}

void Coordinate::deinit()
{
    m_cloudview->removeAllCoordinateSystems();
}

void Coordinate::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds) {
        std::string sid = cloud->id() + COORDINATE_ADD_FLAG;
        if (!m_cloudview->contains(QString::fromStdString(sid))) {
            ct::Coord cloud_coord(sid, ui->dspin_scale->value(), cloud->box().pose);
            m_cloudview->addCoordinateSystem(cloud_coord);
            m_coord_map[cloud->id()] = cloud_coord;
        }
        printI(QString("Add coordinate of cloud[id:%1] done.")
                   .arg(QString::fromStdString(cloud->id())));
    }
}

void Coordinate::reset()
{
    m_coord_map.clear();
    m_cloudview->removeAllCoordinateSystems();
    m_cloudview->addCoordinateSystem(m_origin_coord);
    printI("Reset all coordinates done.");
}

void Coordinate::addCoord()
{
    if (ui->lineEdit_coord_id->text().isEmpty()) return;
    QString id = ui->lineEdit_coord_id->text();
    if (m_cloudview->contains(id)) {
        printW(QString("The coordinate id[%1] already exists!").arg(id));
        return;
    }
    Eigen::Affine3f affine;
    if (!parseMatrixText(ui->txt_matrix->toPlainText(), affine)) {
        printW("The transformation matrix format is wrong");
        return;
    }
    ct::Coord add_coord(id.toStdString(), ui->dspin_scale->value(), affine);
    m_cloudview->addCoordinateSystem(add_coord);
    printI(QString("Add coordinate[id:%1] done.").arg(id));
}

void Coordinate::closeCoord()
{
    QString id = ui->lineEdit_coord_id->text();
    if (!m_cloudview->contains(id)) {
        printW("The id " + id + " does not exist!");
        return;
    }
    m_cloudview->removeCoordinateSystem(id);
    printI(QString("Remove coordinate[id:%1] done.").arg(id));
}

bool Coordinate::parseMatrixText(const QString& text, Eigen::Affine3f& t)
{
    QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    if (lines.size() != 4) return false;

    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i) {
        QStringList vals = lines[i].split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (vals.size() != 4) return false;
        for (int j = 0; j < 4; ++j) {
            bool ok;
            mat(i, j) = vals[j].toFloat(&ok);
            if (!ok) return false;
        }
    }
    t = Eigen::Affine3f(mat);
    return true;
}
