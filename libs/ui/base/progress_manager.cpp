#include "progress_manager.h"

#include <QApplication>

namespace ct
{

ProgressManager::ProgressManager(QObject* parent)
    : QObject(parent), m_dialog(nullptr), m_script_mode(false), m_loading_queue_count(0)
{}

ProgressManager::~ProgressManager()
{
    closeProgress();
}

void ProgressManager::showProgress(const QString& message)
{
    if (m_script_mode) return;

    if (!m_dialog) {
        QWidget* topLevel = qobject_cast<QWidget*>(parent());
        if (!topLevel) topLevel = nullptr;
        m_dialog = new ProcessingDialog(topLevel);
        m_dialog->setWindowModality(Qt::WindowModal);

        connect(m_dialog, &ProcessingDialog::cancelRequested, this, &ProgressManager::cancelRequested);
    }

    m_dialog->reset();
    m_dialog->setMessage(message);
    m_dialog->show();
    QApplication::processEvents();
}

void ProgressManager::closeProgress()
{
    m_loading_queue_count = 0;
    if (m_dialog) {
        m_dialog->close();
        delete m_dialog;
        m_dialog = nullptr;
    }
}

void ProgressManager::setProgress(int percent)
{
    if (m_dialog)
        m_dialog->setProgress(percent);
}

void ProgressManager::setMessage(const QString& message)
{
    if (m_dialog)
        m_dialog->setMessage(message);
}

void ProgressManager::bindWorker(QObject* worker)
{
    if (!m_dialog || !worker) return;

    connect(worker, SIGNAL(progress(int)), m_dialog, SLOT(setProgress(int)), Qt::UniqueConnection);
    connect(m_dialog, SIGNAL(cancelRequested()), worker, SLOT(cancel()), Qt::DirectConnection);
    connect(m_dialog, &ProcessingDialog::cancelRequested, this, &ProgressManager::closeProgress);
}

} // namespace ct
