//
// Created by LBC on 2025/12/23.
//

#ifndef POINTWORKS_FIELDMAPPINGDIALOG_H
#define POINTWORKS_FIELDMAPPINGDIALOG_H

#include "core/field_types.h"

#include <QDialog>
#include <QTableWidget>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QHeaderView>

#include <algorithm>

namespace pw {
    class FieldMappingDialog : public QDialog {
    Q_OBJECT
    public:
        explicit FieldMappingDialog(const QList<pw::FieldInfo> &input_fields, QWidget *parent = nullptr)
                : QDialog(parent) {
            setWindowTitle("Field Mapping");
            resize(500, 400);
            setFixedSize(500, 400);

            QVBoxLayout *layout = new QVBoxLayout(this);
            layout->addWidget(new QLabel("Map file fields to Cloud properties:"));

            QList<pw::FieldInfo> fields = input_fields; // 创建副本进行排序

            // 定义优先级计算函数 (越小越靠前)
            auto getFieldRank = [](const std::string& rawName) -> int {
                QString n = QString::fromStdString(rawName).toLower();
                // Rank 0: 坐标
                if (n == "x") return 0;
                if (n == "y") return 1;
                if (n == "z") return 2;

                // Rank 1: 颜色
                if (n == "r" || n == "red" || n.contains("red")) return 10;
                if (n == "g" || n == "green" || n.contains("green")) return 11;
                if (n == "b" || n == "blue" || n.contains("blue")) return 12;
                if (n == "rgb" || n == "rgba") return 13;

                // Rank 2: 法线
                if (n == "nx" || n == "normal_x") return 20;
                if (n == "ny" || n == "normal_y") return 21;
                if (n == "nz" || n == "normal_z") return 22;

                // Rank 3: 常用属性
                if (n == "intensity") return 30;
                if (n == "curvature") return 31;
                if (n == "time" || n == "gps_time") return 32;

                // Rank 4: 其他杂项 (放在最后)
                return 100;
            };

            std::sort(fields.begin(), fields.end(), [&](const pw::FieldInfo& a, const pw::FieldInfo& b){
                int rankA = getFieldRank(a.name);
                int rankB = getFieldRank(b.name);
                if (rankA != rankB) return rankA < rankB;
                return a.name < b.name; // std::string operator< 可直接比较
            });

            m_table = new QTableWidget(fields.size(), 3);
            m_table->setHorizontalHeaderLabels({"File Field", "Type", "Map To"});
            m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

            for (int i = 0; i < fields.size(); ++i) {
                m_table->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(fields[i].name)));
                m_table->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(fields[i].type)));

                QComboBox *combo = new QComboBox();
                combo->addItem("Ignore");

                // 智能预选
                QString lowerName = QString::fromStdString(fields[i].name).toLower();
                if (lowerName == "x" || lowerName == "y" || lowerName == "z") {
                    combo->addItem("Axis " + QString::fromStdString(fields[i].name).toUpper());
                    combo->setCurrentIndex(1);
                    combo->setCurrentText("Axis " + QString::fromStdString(fields[i].name).toUpper());
                    combo->setEnabled(false); // 坐标强制映射
                } else if (lowerName == "rgba" || lowerName == "rgb") {
                    combo->addItem("Color(Packed)");
                    combo->setCurrentText("Color(Packed)");
                } else if (lowerName.contains("red") || lowerName == "r") {
                    combo->addItem("Red");
                    combo->setCurrentText("Red");
                } else if (lowerName.contains("green") || lowerName == "g") {
                    combo->addItem("Green");
                    combo->setCurrentText("Green");
                } else if (lowerName.contains("blue") || lowerName == "b") {
                    combo->addItem("Blue");
                    combo->setCurrentText("Blue");
                } else if (lowerName == "normal_x" || lowerName == "nx") {
                    combo->addItem("Normal X");
                    combo->setCurrentText("Normal X");
                } else if (lowerName == "normal_y" || lowerName == "ny") {
                    combo->addItem("Normal Y");
                    combo->setCurrentText("Normal Y");
                } else if (lowerName == "normal_z" || lowerName == "nz") {
                    combo->addItem("Normal Z");
                    combo->setCurrentText("Normal Z");
                } else if (lowerName == "curvature") {
                    combo->addItem("Curvature");
                    combo->setCurrentText("Curvature");
                } else {
                    // 默认其他字段作为标量场导入
                    combo->addItem("Scalar Field");
                    if (lowerName == "intensity") {
                        combo->addItem("Intensity");
                        combo->setCurrentText("Intensity");
                    } else {
                        combo->setCurrentText("Scalar Field");
                    }
                }
                m_table->setCellWidget(i, 2, combo);
            }

            layout->addWidget(m_table);

            QPushButton *btnOk = new QPushButton("Import");
            connect(btnOk, &QPushButton::clicked, this, &QDialog::accept);
            layout->addWidget(btnOk);
        }

        MappingResult getMapping() {
            MappingResult res;
            for (int i = 0; i < m_table->rowCount(); ++i) {
                QString name = m_table->item(i, 0)->text();
                QComboBox *cb = qobject_cast<QComboBox *>(m_table->cellWidget(i, 2));
                res.field_map[name.toStdString()] = cb->currentText().toStdString();
            }
            return res;
        }

    private:
        QTableWidget *m_table;
    };
} // namespace pw
#endif //POINTWORKS_FIELDMAPPINGDIALOG_H
