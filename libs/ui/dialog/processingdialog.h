//
// Created by LBC on 2026/1/5.
//

#ifndef POINTWORKS_PROCESSINGDIALOG_H
#define POINTWORKS_PROCESSINGDIALOG_H

#include <QDialog>
#include <QProgressBar>
#include <QLabel>
#include <QVBoxLayout>
#include <QPushButton>

namespace ct{
    class ProcessingDialog : public QDialog
    {
        Q_OBJECT
    public:
        explicit ProcessingDialog(QWidget* parent = nullptr, const QString& title = "Processing...")
            : QDialog(parent){
            setWindowTitle(title);
            setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
            setWindowModality(Qt::WindowModal);
            resize(400, 120);
            setFixedSize(size());

            auto* layout = new QVBoxLayout(this);

            m_label = new QLabel("Please wait...", this);
            layout->addWidget(m_label);

            m_progressBar = new QProgressBar(this);
            m_progressBar->setRange(0,100);
            m_progressBar->setValue(0);
            layout->addWidget(m_progressBar);

            m_btnCancel = new QPushButton("Cancel", this);
            connect(m_btnCancel, &QPushButton::clicked, this, &ProcessingDialog::cancelRequested);
            layout->addWidget(m_btnCancel);
        }

        void setMessage(const QString& msg){
            m_label->setText(msg);
        }

        void reset(){
            m_progressBar->setValue(0);
            m_btnCancel->setEnabled(true);
            m_btnCancel->setText("Cancel");
        }

    public slots:
        void setProgress(int value){
            m_progressBar->setValue(value);
        }

    signals:
        void cancelRequested();

    private:
        QProgressBar* m_progressBar;
        QLabel* m_label;
        QPushButton* m_btnCancel;
    };
} // namespace ct
#endif //POINTWORKS_PROCESSINGDIALOG_H
