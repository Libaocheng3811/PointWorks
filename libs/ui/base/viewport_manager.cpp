#include "viewport_manager.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QStyle>
#include <QResizeEvent>

namespace ct
{

namespace {

class VpLabel : public QLabel
{
public:
    VpLabel(const QString& text, QWidget* parent) : QLabel(text, parent) {}
protected:
    void resizeEvent(QResizeEvent*) override {
        move(parentWidget()->width() - 36, 4);
    }
};

} // anonymous namespace

ViewportManager::ViewportManager(QWidget* container)
    : QObject(container), m_container(container)
{
    // 复用容器已有的 layout，或创建新的
    m_container_layout = qobject_cast<QVBoxLayout*>(m_container->layout());
    if (!m_container_layout) {
        if (m_container->layout()) {
            // 已有其他类型 layout，清空内容后重建
            QLayoutItem* child;
            while ((child = m_container->layout()->takeAt(0)) != nullptr) {
                delete child->widget();
                delete child;
            }
            delete m_container->layout();
        }
        m_container_layout = new QVBoxLayout(m_container);
    }
    m_container_layout->setContentsMargins(0, 0, 0, 0);
    m_container_layout->setSpacing(0);

    // 默认单窗口
    applyLayout();
}

ViewportManager::~ViewportManager()
{
    clearViews();
}

void ViewportManager::clearViews()
{
    // 清理布局中的 splitter
    if (m_main_splitter) {
        m_main_splitter->setParent(nullptr);
        delete m_main_splitter;
        m_main_splitter = nullptr;
    }
    m_view_frames.clear();
    m_views.clear();
    m_active_view = nullptr;
}

QFrame* ViewportManager::createViewFrame(CloudView* view, int index)
{
    auto* frame = new QFrame(m_container);
    frame->setFrameShape(QFrame::Box);
    frame->setLineWidth(2);
    frame->setFrameShadow(QFrame::Plain);

    auto* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(view);

    if (index >= 0) {
        auto* label = new VpLabel(QString("V%1").arg(index + 1), frame);
        label->setObjectName("vpLabel");
        label->setFixedSize(28, 20);
        label->setAlignment(Qt::AlignCenter);
        label->setStyleSheet(
            "QLabel {"
            "  color: white;"
            "  font-size: 13px;"
            "  font-weight: bold;"
            "  background-color: rgba(0,0,0,150);"
            "  border-radius: 4px;"
            "  padding: 1px 4px;"
            "}");
        label->raise();
    }

    m_view_frames[view] = frame;

    // 点击视窗切换活跃状态
    connect(view, &CloudView::mouseLeftPressed, this, [this, view]() {
        setActiveView(view);
    });

    // 同步旋转：监听相机变化
    connect(view, &CloudView::viewerPose, this, &ViewportManager::onCameraChanged);

    return frame;
}

void ViewportManager::updateActiveHighlight()
{
    for (auto it = m_view_frames.begin(); it != m_view_frames.end(); ++it) {
        if (it.key() == m_active_view) {
            it.value()->setStyleSheet("QFrame { border: 2px solid #4A90D9; }");
        } else {
            it.value()->setStyleSheet("QFrame { border: 2px solid #555555; }");
        }
    }
}

void ViewportManager::applyLayout()
{
    clearViews();

    int count = 0;
    switch (m_layout_mode) {
    case Single:           count = 1; break;
    case HorizontalSplit:  count = 2; break;
    case VerticalSplit:    count = 2; break;
    case TripleSplit:      count = 3; break;
    case QuadSplit:        count = 4; break;
    }

    // 创建 CloudView 实例
    for (int i = 0; i < count; ++i) {
        auto* view = new CloudView(m_container);
        auto* frame = createViewFrame(view, m_layout_mode == Single ? -1 : i);
        m_views.push_back(view);
    }

    // 根据模式构建布局
    switch (m_layout_mode) {
    case Single:
    {
        m_main_splitter = new QSplitter(Qt::Horizontal, m_container);
        m_main_splitter->addWidget(m_view_frames[m_views[0]]);
        break;
    }
    case HorizontalSplit:
    {
        m_main_splitter = new QSplitter(Qt::Horizontal, m_container);
        m_main_splitter->addWidget(m_view_frames[m_views[0]]);
        m_main_splitter->addWidget(m_view_frames[m_views[1]]);
        break;
    }
    case VerticalSplit:
    {
        m_main_splitter = new QSplitter(Qt::Vertical, m_container);
        m_main_splitter->addWidget(m_view_frames[m_views[0]]);
        m_main_splitter->addWidget(m_view_frames[m_views[1]]);
        break;
    }
    case TripleSplit:
    {
        m_main_splitter = new QSplitter(Qt::Horizontal, m_container);
        auto* rightSplitter = new QSplitter(Qt::Vertical, m_container);
        rightSplitter->addWidget(m_view_frames[m_views[1]]);
        rightSplitter->addWidget(m_view_frames[m_views[2]]);
        m_main_splitter->addWidget(m_view_frames[m_views[0]]);
        m_main_splitter->addWidget(rightSplitter);
        m_main_splitter->setStretchFactor(0, 3);
        m_main_splitter->setStretchFactor(1, 2);
        break;
    }
    case QuadSplit:
    {
        m_main_splitter = new QSplitter(Qt::Horizontal, m_container);
        auto* leftSplitter = new QSplitter(Qt::Vertical, m_container);
        auto* rightSplitter = new QSplitter(Qt::Vertical, m_container);
        leftSplitter->addWidget(m_view_frames[m_views[0]]);
        leftSplitter->addWidget(m_view_frames[m_views[2]]);
        rightSplitter->addWidget(m_view_frames[m_views[1]]);
        rightSplitter->addWidget(m_view_frames[m_views[3]]);
        m_main_splitter->addWidget(leftSplitter);
        m_main_splitter->addWidget(rightSplitter);
        break;
    }
    }

    m_container_layout->addWidget(m_main_splitter);

    // 默认第一个为活跃视窗
    m_active_view = m_views[0];
    updateActiveHighlight();
    emit activeViewChanged(m_active_view);
    emit viewsRecreated();
}

void ViewportManager::setLayout(LayoutMode mode)
{
    // 保存当前活跃视窗的相机参数
    CameraParams savedCamera;
    if (m_active_view) {
        savedCamera = m_active_view->getCameraParams();
    }

    if (mode == m_layout_mode) return;
    m_layout_mode = mode;
    applyLayout();

    // 新布局创建后，将所有视窗相机同步到保存的相机参数
    if (m_active_view) {
        for (auto* view : m_views) {
            view->setCameraParams(savedCamera);
        }
    }
}

void ViewportManager::setActiveView(CloudView* view)
{
    if (view == m_active_view) return;
    m_active_view = view;
    updateActiveHighlight();
    emit activeViewChanged(m_active_view);
}

CloudView* ViewportManager::viewAt(int index) const
{
    if (index >= 0 && index < m_views.size())
        return m_views[index];
    return nullptr;
}

void ViewportManager::setSyncRotation(bool enable)
{
    if (m_sync_rotation == enable) return;
    m_sync_rotation = enable;
    emit syncRotationChanged(enable);
}

void ViewportManager::setShowViewportLabels(bool show)
{
    if (m_show_labels == show) return;
    m_show_labels = show;
    for (auto* frame : m_view_frames) {
        for (auto* child : frame->findChildren<QLabel*>("vpLabel")) {
            child->setVisible(show);
            if (show) child->raise();
        }
    }
}

void ViewportManager::onCameraChanged(const Eigen::Affine3f& pose)
{
    if (m_syncing || !m_sync_rotation || !m_active_view) return;
    m_syncing = true;

    // 从 Affine3f 提取相机参数
    CameraParams params = m_active_view->getCameraParams();

    for (auto* view : m_views) {
        if (view && view != m_active_view) {
            view->setCameraParams(params);
        }
    }

    m_syncing = false;
}

void ViewportManager::syncPointCloudToAllViews(const Cloud::Ptr& cloud)
{
    for (auto* view : m_views) {
        if (view) {
            view->addPointCloud(cloud);
        }
    }
}

void ViewportManager::syncRemoveFromAllViews(const QString& id)
{
    for (auto* view : m_views) {
        if (view) {
            view->removePointCloud(id);
        }
    }
}

} // namespace ct
