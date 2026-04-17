//
// Created by LBC on 2026/1/8.
//

// You may need to build the project (run Qt uic code generator) to get "ui_globalshiftdialog.h" resolved

#include "globalshiftdialog.h"

#include <utility>
#include "ui_globalshiftdialog.h"


GlobalShiftDialog::GlobalShiftDialog(Eigen::Vector3d  original_min, const Eigen::Vector3d& suggested_shift,
                                     const Eigen::Vector3d& last_shift, bool hasLast, QWidget *parent)
                                     : QDialog(parent), ui(new Ui::GlobalShiftDialog), m_suggested_shift(suggested_shift),
                                     m_original_point(std::move(original_min)), m_last_shift(last_shift), m_has_last(hasLast){
    ui->setupUi(this);

    ui->lblOriginalX->setText(QString::number(original_min.x(), 'f', 4));
    ui->lblOriginalY->setText(QString::number(original_min.y(), 'f', 4));
    ui->lblOriginalZ->setText(QString::number(original_min.z(), 'f', 4));

    // suggested:0
    ui->combo_shiftType->addItem("Suggested", 0);
    // lastInput:1
    if (m_has_last)
        ui->combo_shiftType->addItem("Last Input", 1);
    ui->combo_shiftType->addItem("Custom", 2);

    connect(ui->combo_shiftType, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GlobalShiftDialog::onProfileChanged);

    connect(ui->spinShiftX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GlobalShiftDialog::onShiftChanged);
    connect(ui->spinShiftY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GlobalShiftDialog::onShiftChanged);
    connect(ui->spinShiftZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GlobalShiftDialog::onShiftChanged);

    connect(ui->btnOk, &QPushButton::clicked, this, &GlobalShiftDialog::onBtnOKClicked);
    connect(ui->btnCancel, &QPushButton::clicked, this, &GlobalShiftDialog::onBtnCancelClicked);

    ui->combo_shiftType->setCurrentIndex(0);
    onProfileChanged(0);
}

GlobalShiftDialog::~GlobalShiftDialog() {
    delete ui;
}

Eigen::Vector3d GlobalShiftDialog::getShiftValue() const {
    return Eigen::Vector3d(ui->spinShiftX->value(), ui->spinShiftY->value(), ui->spinShiftZ->value());
}

bool GlobalShiftDialog::isSkipped() const {
    return m_skipped;
}

void GlobalShiftDialog::onProfileChanged(int index) {
    // 获取选中类型id
    int type = ui->combo_shiftType->currentData().toInt();

    ui->spinShiftX->blockSignals(true);
    ui->spinShiftY->blockSignals(true);
    ui->spinShiftZ->blockSignals(true);

    if (type == 0){
        ui->spinShiftX->setValue(m_suggested_shift.x());
        ui->spinShiftY->setValue(m_suggested_shift.y());
        ui->spinShiftZ->setValue(m_suggested_shift.z());
    }
    else if (type == 1){
        ui->spinShiftX->setValue(m_last_shift.x());
        ui->spinShiftY->setValue(m_last_shift.y());
        ui->spinShiftZ->setValue(m_last_shift.z());
    }
    // 如果是Custom，保持不变

    ui->spinShiftX->blockSignals(false);
    ui->spinShiftY->blockSignals(false);
    ui->spinShiftZ->blockSignals(false);

    updatePreviewText();
}

void GlobalShiftDialog::updatePreviewText() {
    double lx = m_original_point.x() + ui->spinShiftX->value();
    double ly = m_original_point.y() + ui->spinShiftY->value();
    double lz = m_original_point.z() + ui->spinShiftZ->value();

    ui->lblLocalX->setText(QString::number(lx, 'f', 4));
    ui->lblLocalY->setText(QString::number(ly, 'f', 4));
    ui->lblLocalZ->setText(QString::number(lz, 'f', 4));
}

void GlobalShiftDialog::onShiftChanged() {
    int customIndex = ui->combo_shiftType->count() - 1;
    if (ui->combo_shiftType->currentIndex() != customIndex) {
        ui->combo_shiftType->blockSignals(true);
        ui->combo_shiftType->setCurrentIndex(customIndex);
        ui->combo_shiftType->blockSignals(false);
    }

    // 更新显示
    updatePreviewText();
}

void GlobalShiftDialog::onBtnOKClicked() {
    m_skipped = false;
    accept();
}

void GlobalShiftDialog::onBtnCancelClicked() {
    m_skipped = true;
    this->reject();
}