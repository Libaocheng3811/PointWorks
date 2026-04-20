#ifndef POINTWORKS_PROGRESS_MANAGER_H
#define POINTWORKS_PROGRESS_MANAGER_H

#include "dialog/processingdialog.h"

#include <QObject>

namespace ct
{
    class ProgressManager : public QObject
    {
        Q_OBJECT
    public:
        explicit ProgressManager(QObject* parent = nullptr);
        ~ProgressManager() override;

        void showProgress(const QString& message);
        void closeProgress();
        void setProgress(int percent);
        void setMessage(const QString& message);

        void bindWorker(QObject* worker);

        ProcessingDialog* dialog() const { return m_dialog; }

        void setScriptMode(bool enabled) { m_script_mode = enabled; }
        bool scriptMode() const { return m_script_mode; }

        void setLoadingQueueCount(int count) { m_loading_queue_count = count; }
        int loadingQueueCount() const { return m_loading_queue_count; }

    signals:
        void cancelRequested();

    private:
        ProcessingDialog* m_dialog = nullptr;
        bool m_script_mode = false;
        int m_loading_queue_count = 0;
    };
}

#endif // POINTWORKS_PROGRESS_MANAGER_H
