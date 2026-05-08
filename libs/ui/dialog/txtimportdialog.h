//
// Created by LBC on 2025/12/29.
//

#ifndef POINTWORKS_TXTIMPORTDIALOG_H
#define POINTWORKS_TXTIMPORTDIALOG_H

#include "core/field_types.h"

#include <QDialog>
#include <QTableWidget>
#include <QComboBox>
#include <QSpinBox>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QHeaderView>
#include <QPushButton>
#include <QTextStream>

namespace pw{
    class TxtImportDialog : public QDialog{
        Q_OBJECT
        public:
            explicit TxtImportDialog(const QStringList& preview_lines, QWidget* parent = nullptr)
            : QDialog(parent), m_lines(preview_lines){
                setWindowTitle("ASCII/TXT Import Wizard");
                resize(800, 500);
                setupUI();
                detectSeparator(); // 自动检测分隔符
                updatePreview(); // 初始刷新
            }

            TxtImportParams getParams(){
                TxtImportParams params;
                params.skip_lines = m_spinSkip->value();
                if (m_radioComma->isChecked()) params.separator = ',';
                else if (m_radioSemi->isChecked()) params.separator = ':';
                else if (m_radioTab->isChecked()) params.separator = '\t';
                else params.separator = ' ';

                for (int c = 0; c < m_table->columnCount(); ++c){
                    QComboBox* combo = qobject_cast<QComboBox*>(m_table->cellWidget(0, c));
                    if (combo && combo->currentIndex() > 0){
                        params.col_map[c] = combo->currentData().toString().toStdString();
                    }
                }
                return params;
            }

        private:
            void setupUI(){
                QVBoxLayout* mainLayout = new QVBoxLayout(this);

                // 控制区
                QHBoxLayout* ctrlLayout = new QHBoxLayout();
                QGroupBox* grpSep = new QGroupBox("Separator");
                QHBoxLayout* sepLayout = new QHBoxLayout();
                m_radioSpace = new QRadioButton("Space");
                m_radioComma = new QRadioButton("Comma");
                m_radioSemi = new QRadioButton("Semicolon");
                m_radioTab = new QRadioButton("Tab");
                QButtonGroup* bg = new QButtonGroup(this);
                bg->addButton(m_radioSpace);
                bg->addButton(m_radioComma);
                bg->addButton(m_radioSemi);
                bg->addButton(m_radioTab);
                m_radioSpace->setChecked(true); // default
                sepLayout->addWidget(m_radioSpace);
                sepLayout->addWidget(m_radioComma);
                sepLayout->addWidget(m_radioSemi);
                sepLayout->addWidget(m_radioTab);
                grpSep->setLayout(sepLayout);

                // 连接信号,分隔符改变时刷新
                connect(bg, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked), this, &TxtImportDialog::updatePreview);

                m_spinSkip = new QSpinBox();
                m_spinSkip->setRange(0, 100); // 行数范围
                // 改变行数时刷新
                connect(m_spinSkip, QOverload<int>::of(&QSpinBox::valueChanged), this, &TxtImportDialog::updatePreview);

                ctrlLayout->addWidget(grpSep);
                ctrlLayout->addWidget(new QLabel("Skip lines:"));
                ctrlLayout->addWidget(m_spinSkip);
                ctrlLayout->addStretch();
                mainLayout->addLayout(ctrlLayout);

                // 表格区
                m_table = new QTableWidget();
                mainLayout->addWidget(m_table);

                // 按钮区
                QHBoxLayout* btnLayout = new QHBoxLayout();
                QPushButton* btnOK = new QPushButton("Apply");
                QPushButton* btnCancel = new QPushButton("Cancel");
                connect(btnOK, &QPushButton::clicked, this, &QDialog::accept);
                connect(btnCancel, &QPushButton::clicked, this, &QDialog::reject);
                btnLayout->addStretch();
                btnLayout->addWidget(btnOK);
                btnLayout->addWidget(btnCancel);
                mainLayout->addLayout(btnLayout);
            }

            void detectSeparator(){
                // 自动检测分隔符
                if (m_lines.isEmpty()) return;
                QString l = m_lines.first();
                if (l.count(',') > l.count(' ')) m_radioComma->setChecked(true);
                else if (l.count(':') > l.count(',')) m_radioSemi->setChecked(true);
                else if (l.count('\t')) m_radioTab->setChecked(true);
                else  m_radioSpace->setChecked(true);
            }

            void updatePreview(){
                m_table->clear();

                char sep = ' ';
                if (m_radioComma->isChecked()) sep = ',';
                else if (m_radioSemi->isChecked()) sep = ':';
                else if (m_radioTab->isChecked()) sep = '\t';

                // start_row表示数据的起始行,根据m_spinSkip来确定数据从哪行开始
                int start_row = m_spinSkip->value();
                int preview_limit = std::min(m_lines.size(), start_row + 20); // 最大行数限制

                if(start_row >= m_lines.size()) return;

                // 分析一行有效数据
                QString firstLine = m_lines[start_row];
                QStringList parts = splitLine(firstLine, sep); // 将一行数据按分隔符进行分割
                int col_count = parts.size(); // 分隔得到的列数

                m_table->setColumnCount(col_count);
                m_table->setRowCount((preview_limit - start_row) + 1); // +1 for  header combos

                // 设置表头，每列
                for (int c = 0; c < col_count; ++c){
                    QComboBox* combo = new QComboBox();
                    combo->addItem("None", "none");
                    combo->addItem("Axis X", "x");
                    combo->addItem("Axis Y", "y");
                    combo->addItem("Axis Z", "z");
                    combo->addItem("Red", "r");
                    combo->addItem("Green", "g");
                    combo->addItem("Blue", "b");
                    combo->addItem("Intensity", "Intensity");
                    combo->addItem("Scalar Field", QString("Scalar_%1").arg(c));

                    // 简单预测
                    if (c == 0) combo->setCurrentIndex(1); // x
                    else if (c == 1) combo->setCurrentIndex(2); // y
                    else if (c == 2) combo->setCurrentIndex(3); // z
                    else if (c == 3 && col_count == 4) combo->setCurrentIndex(7); // Intensity
                    else if (c == 3 && col_count >= 6) combo->setCurrentIndex(4); // Red
                    else if (c == 4 && col_count >= 6) combo->setCurrentIndex(5); // Green
                    else if (c == 5 && col_count >= 6) combo->setCurrentIndex(6); // Blue

                    m_table->setCellWidget(0, c, combo);
                }
                m_table->setVerticalHeaderItem(0, new QTableWidgetItem("Set:"));

                // 填充数据
                for(int r = start_row; r < preview_limit; ++r){
                    QStringList cells = splitLine(m_lines[r], sep);
                    int row_idx = (r - start_row) + 1;
                    m_table->setVerticalHeaderItem(row_idx, new QTableWidgetItem(QString::number(r + 1)));
                    for (int c = 0; c < std::min(cells.size(), col_count); ++c){
                        m_table->setItem(row_idx, c, new QTableWidgetItem(cells[c]));
                    }
                }
            }

            QStringList splitLine(const QString& line, char sep){
                // 处理连续空格
                if (sep == ' '){
                    return line.simplified().split(' ', QString::SkipEmptyParts);
                }
                return line.split(sep);
            }

        private:
            QStringList m_lines; // 存储读取的每行文件内容
            QTableWidget* m_table;
            QSpinBox* m_spinSkip;
            QRadioButton *m_radioSpace, *m_radioComma, *m_radioSemi, *m_radioTab;

    };
} // namespace pw
#endif //POINTWORKS_TXTIMPORTDIALOG_H
