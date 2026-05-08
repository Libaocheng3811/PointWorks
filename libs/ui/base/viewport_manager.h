#pragma once

#include "viz/cloudview.h"
#include "core/exports.h"

#include <QObject>
#include <QWidget>
#include <QVBoxLayout>
#include <QSplitter>
#include <QVector>
#include <QFrame>
#include <QMap>

namespace pw
{

class ViewportManager : public QObject
{
    Q_OBJECT
public:
    enum LayoutMode {
        Single,
        HorizontalSplit,
        VerticalSplit,
        TripleSplit,
        QuadSplit
    };

    explicit ViewportManager(QWidget* container);
    ~ViewportManager();

    void setLayout(LayoutMode mode);
    LayoutMode layoutMode() const { return m_layout_mode; }

    CloudView* activeView() const { return m_active_view; }
    CloudView* viewAt(int index) const;
    int viewCount() const { return m_views.size(); }
    QList<CloudView*> allViews() const { return m_views.toList(); }

    void setActiveView(CloudView* view);

    void setSyncRotation(bool enable);
    bool isSyncRotation() const { return m_sync_rotation; }

    void setShowViewportLabels(bool show);
    bool showViewportLabels() const { return m_show_labels; }

    void syncPointCloudToAllViews(const Cloud::Ptr& cloud);
    void syncRemoveFromAllViews(const QString& id);

signals:
    void activeViewChanged(CloudView* view);
    void syncRotationChanged(bool enabled);
    void viewsRecreated();

private slots:
    void onCameraChanged(const Eigen::Affine3f& pose);

private:
    void clearViews();
    void applyLayout();
    QFrame* createViewFrame(CloudView* view, int index);
    void updateActiveHighlight();

    QWidget* m_container;
    QVBoxLayout* m_container_layout;
    QSplitter* m_main_splitter = nullptr;
    QVector<CloudView*> m_views;
    CloudView* m_active_view = nullptr;
    LayoutMode m_layout_mode = Single;
    bool m_sync_rotation = false;
    bool m_syncing = false;
    bool m_show_labels = true;
    QMap<CloudView*, QFrame*> m_view_frames;
};

} // namespace pw
