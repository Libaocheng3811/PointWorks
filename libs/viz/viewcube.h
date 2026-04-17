//
// Created by LBC on 2026/04/12.
//

#ifndef POINTWORKS_VIEWCUBE_H
#define POINTWORKS_VIEWCUBE_H

#include <QWidget>
#include <QVector3D>
#include <QPolygonF>
#include <QTimer>
#include <Eigen/Core>
#include <QColor>
#include <functional>

class ViewCube : public QWidget
{
    Q_OBJECT

public:
    using CameraCallback = std::function<void(const Eigen::Vector3f&, const Eigen::Vector3f&, bool)>;

    explicit ViewCube(QWidget* parent = nullptr);
    ~ViewCube() override;

    void setCameraCallback(CameraCallback cb);
    void updateCameraOrientation(const Eigen::Vector3f& dir, const Eigen::Vector3f& up);
    void setViewCubeEnabled(bool enabled);
    bool isViewCubeEnabled() const { return m_enabled; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    enum RegionType { TYPE_FACE = 0, TYPE_EDGE = 1, TYPE_CORNER = 2 };

    struct Region {
        RegionType type;
        int index;
        QVector3D corners[4];
        int cornerCount;
        QString tooltip;
        Eigen::Vector3f viewDir;
        Eigen::Vector3f viewUp;
        QColor baseColor;
    };

    QPointF project(const Eigen::Vector3f& pt) const;
    QPolygonF projectRegion(const Region& r) const;
    bool isRegionVisible(const Region& r) const;
    float regionDepth(const Region& r) const;
    int hitTest(const QPoint& pos);

    void drawAxes(QPainter& p);
    void drawRegion(QPainter& p, const Region& r, bool hovered);
    void drawFaceLabel(QPainter& p, const Region& r);

    void navigateTo(int regionIndex);
    void applyDragRotation(int dx, int dy);

    void startAnimation(const Eigen::Vector3f& targetDir, const Eigen::Vector3f& targetUp);
    void onAnimTick();

    void buildRegions();
    void updateBasis();

    // State
    bool m_enabled = true;
    bool m_animating = false;
    int m_hovered = -1;

    // Drag state
    bool m_dragging = false;
    QPoint m_pressPos;
    QPoint m_lastDragPos;
    static constexpr int DRAG_THRESHOLD = 4;

    Eigen::Vector3f m_cameraDir = {0, -1, 0};
    Eigen::Vector3f m_cameraUp = {0, 0, 1};
    Eigen::Vector3f m_right = {1, 0, 0};
    Eigen::Vector3f m_projUp = {0, 0, 1};

    CameraCallback m_cameraCallback;
    QVector<Region> m_regions;

    // Animation
    QTimer* m_animTimer = nullptr;
    float m_animProgress = 0;
    static constexpr int ANIM_DURATION_MS = 200;
    Eigen::Vector3f m_animStartDir, m_animEndDir;
    Eigen::Vector3f m_animStartUp, m_animEndUp;

    float m_chamfer = 0.12f;
};

#endif // POINTWORKS_VIEWCUBE_H
