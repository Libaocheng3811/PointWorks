//
// Created by LBC on 2025/12/29.
//

#ifndef POINTWORKS_TXTEXPORTDIALOG_H
#define POINTWORKS_TXTEXPORTDIALOG_H

#include "core/field_types.h"

#include <QDialog>
#include <QCheckBox>
#include <QComboBox>
#include <QSpinBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QListWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>

namespace pw{
    class TxtExportDialog : public QDialog {
        Q_OBJECT
    public:
        explicit TxtExportDialog(const QStringList& available_fields, QWidget* parent = nullptr)
            : QDialog(parent){
            setWindowTitle("ASCII/TXT Export Wizard");
            resize(400, 500);
            setupUI(available_fields);
        }

        TxtExportParams getParams(){
            TxtExportParams params;
            params.has_header = m_chkHeader->isChecked();
            params.precision = m_spinPrecision->value();

            if (m_radioComma->isChecked()) params.separator = ',';
            else if (m_radioSemi->isChecked()) params.separator = ';';
            else if (m_radioTab->isChecked()) params.separator = '\t';
            else params.separator = ' ';

            //获取用户勾选且排序后的字段
            for (int i = 0; i < m_listFields->count(); i++){
                QListWidgetItem* item = m_listFields->item(i);
                if (item->checkState() == Qt::Checked){
                    params.selected_fields.push_back(item->text().toStdString());
                }
            }
            return params;
        }

    private:
        void setupUI(const QStringList& fields){
            QVBoxLayout* mainLayout = new QVBoxLayout(this);

            //字段选择
            QGroupBox* grpFields = new QGroupBox("Columns to Export(Drag to reorder)", this);
            QVBoxLayout* fieldLayout = new QVBoxLayout();
            m_listFields = new QListWidget();
            m_listFields->setDragDropMode(QAbstractItemView::InternalMove); //允许拖拽排序

            for(const QString& f : fields){
                QListWidgetItem* item = new QListWidgetItem(f);
                item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
                item->setCheckState(Qt::Checked); //默认全选
                m_listFields->addItem(item);
            }
            fieldLayout->addWidget(m_listFields);
            grpFields->setLayout(fieldLayout);
            mainLayout->addWidget(grpFields);

            //格式控制
            QGroupBox* grpFmt = new QGroupBox("Formatting");
            QGridLayout* fmtLayout = new QGridLayout();

            //分隔符
            m_radioSpace = new QRadioButton("Space");
            m_radioComma = new QRadioButton("Comma");
            m_radioSemi = new QRadioButton("Semi-colon");
            m_radioTab = new QRadioButton("Tab");
            QButtonGroup* bg = new QButtonGroup(this);
            bg->addButton(m_radioSpace);
            bg->addButton(m_radioComma);
            bg->addButton(m_radioSemi);
            bg->addButton(m_radioTab);
            m_radioSpace->setChecked(true);

            QHBoxLayout* sepLayout = new QHBoxLayout();
            sepLayout->addWidget(m_radioSpace);
            sepLayout->addWidget(m_radioComma);
            sepLayout->addWidget(m_radioSemi);
            sepLayout->addWidget(m_radioTab);

            //精度
            m_spinPrecision = new QSpinBox();
            m_spinPrecision->setRange(0, 16);
            m_spinPrecision->setValue(6);
            m_chkHeader = new QCheckBox("Save Header line");
            m_chkHeader->setChecked(true);

            fmtLayout->addWidget(new QLabel("Separator:"), 0, 0);
            fmtLayout->addLayout(sepLayout, 0, 1);
            fmtLayout->addWidget(new QLabel("Precision:"), 1, 0);
            fmtLayout->addWidget(m_spinPrecision, 1, 1);
            fmtLayout->addWidget(m_chkHeader, 2, 0, 1, 2);

            grpFmt->setLayout(fmtLayout);
            mainLayout->addWidget(grpFmt);

            //按钮
            QHBoxLayout* btnLayout = new QHBoxLayout();
            QPushButton* btnOk = new QPushButton("Export");
            QPushButton* btnCancel = new QPushButton("Cancel");
            connect(btnOk, &QPushButton::clicked, this, &QDialog::accept);
            connect(btnCancel, &QPushButton::clicked, this, &QDialog::reject);
            btnLayout->addStretch();
            btnLayout->addWidget(btnOk);
            btnLayout->addWidget(btnCancel);
            mainLayout->addLayout(btnLayout);
        }

    private:
        QListWidget* m_listFields;
        QCheckBox* m_chkHeader;
        QSpinBox* m_spinPrecision;
        QRadioButton *m_radioSpace, *m_radioComma, *m_radioSemi, *m_radioTab;
    };
}

#endif //POINTWORKS_TXTEXPORTDIALOG_H
