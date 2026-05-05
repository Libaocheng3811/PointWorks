#ifndef POINTWORKS_PROGRESS_MANAGER_H
#define POINTWORKS_PROGRESS_MANAGER_H

#include "dialog/processingdialog.h"

#include <QObject>
#include <QFutureWatcher>
#include <QtConcurrent>
#include <atomic>
#include <functional>

namespace ct
{
    using ProgressCallback = std::function<void(int)>;

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

        void setSavingQueueCount(int count) { m_saving_queue_count = count; }
        int savingQueueCount() const { return m_saving_queue_count; }

        template<typename ResultType>
        void runAsync(const QString& title,
                      std::function<ResultType(std::atomic<bool>&, ProgressCallback)> task,
                      std::function<void(const ResultType&)> onFinished)
        {
            showProgress(title);

            auto* cancel = new std::atomic<bool>(false);
            auto* watcher = new QFutureWatcher<ResultType>(this);

            auto dlg = m_dialog;
            connect(this, &ProgressManager::cancelRequested, this,
                    [cancel]() { *cancel = true; });

            auto onFinishedWrapper = [this, onFinished, cancel, watcher]() {
                m_loading_queue_count = 0;
                if (m_dialog) {
                    m_dialog->close();
                    delete m_dialog;
                    m_dialog = nullptr;
                }
                delete cancel;
                if (onFinished) onFinished(watcher->result());
                watcher->deleteLater();
            };

            auto progressCb = [dlg](int pct) {
                if (dlg) {
                    QMetaObject::invokeMethod(dlg, "setProgress",
                                              Qt::QueuedConnection, Q_ARG(int, pct));
                }
            };

            connect(watcher, &QFutureWatcher<ResultType>::finished, this, onFinishedWrapper);

            auto future = QtConcurrent::run([task, cancel, progressCb]() {
                return task(*cancel, progressCb);
            });
            watcher->setFuture(future);
        }

    signals:
        void cancelRequested();

    private:
        ProcessingDialog* m_dialog = nullptr;
        bool m_script_mode = false;
        int m_loading_queue_count = 0;
        int m_saving_queue_count = 0;
    };
}

#endif // POINTWORKS_PROGRESS_MANAGER_H
