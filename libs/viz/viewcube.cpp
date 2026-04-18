//
// Created by LBC on 2026/04/12.
//

#include "viewcube.h"

#include <Eigen/Geometry>
#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QToolTip>
#include <cmath>
#include <algorithm>

using Eigen::Vector3f;

// ============================================================
// Construction
// ============================================================

ViewCube::ViewCube(QWidget* parent) : QWidget(parent)
{
    setAttribute(Qt::WA_NoSystemBackground);
    setMouseTracking(true);
    buildRegions();
    updateBasis();

    m_animTimer = new QTimer(this);
    m_animTimer->setInterval(16);
    connect(m_animTimer, &QTimer::timeout, this, &ViewCube::onAnimTick);
}

ViewCube::~ViewCube()
{
    if (m_animTimer) m_animTimer->stop();
}

void ViewCube::setCameraCallback(CameraCallback cb)
{
    m_cameraCallback = std::move(cb);
}

void ViewCube::setViewCubeEnabled(bool enabled)
{
    m_enabled = enabled;
    setVisible(enabled);
}

void ViewCube::resizeEvent(QResizeEvent* event)
{
    if (auto* p = parentWidget()) {
        int side = std::min(p->width(), p->height());
        int size = side / 5;
        size = std::max(size, 120);
        size = std::min(size, 280);
        setFixedSize(size, size);
    }
    QWidget::resizeEvent(event);
}

// ============================================================
// Camera sync
// ============================================================

void ViewCube::updateCameraOrientation(const Vector3f& dir, const Vector3f& up)
{
    // During drag or animation, ViewCube owns the state — ignore external updates
    if (m_animating || m_dragging) return;
    m_cameraDir = dir.normalized();
    m_cameraUp = up.normalized();
    updateBasis();
    update();
}

void ViewCube::updateBasis()
{
    m_right = m_cameraDir.cross(m_cameraUp).normalized();
    if (m_right.norm() < 1e-6f) {
        m_right = Vector3f(1, 0, 0);
        if (std::abs(m_cameraDir.x()) > 0.9f) m_right = Vector3f(0, 0, 1);
    }
    m_projUp = m_right.cross(m_cameraDir).normalized();
}

// ============================================================
// Projection
// ============================================================

QPointF ViewCube::project(const Vector3f& pt) const
{
    float scale = std::min(width(), height()) * 0.38f;
    float x = pt.dot(m_right) * scale + width() / 2.0f;
    float y = -pt.dot(m_projUp) * scale + height() / 2.0f;
    return QPointF(x, y);
}

QPolygonF ViewCube::projectRegion(const Region& r) const
{
    QPolygonF poly;
    for (int i = 0; i < r.cornerCount; ++i) {
        const auto& c = r.corners[i];
        poly.append(project(Eigen::Vector3f(c.x(), c.y(), c.z())));
    }
    return poly;
}

// ============================================================
// Visibility & Hit test
// ============================================================

bool ViewCube::isRegionVisible(const Region& r) const
{
    QPolygonF poly = projectRegion(r);
    double area = 0;
    int n = poly.size();
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        area += poly[i].x() * poly[j].y() - poly[j].x() * poly[i].y();
    }
    return area > 10.0;
}

float ViewCube::regionDepth(const Region& r) const
{
    Vector3f center = Vector3f::Zero();
    for (int i = 0; i < r.cornerCount; ++i)
        center += Eigen::Vector3f(r.corners[i].x(), r.corners[i].y(), r.corners[i].z());
    center /= r.cornerCount;
    return center.dot(m_cameraDir);
}

int ViewCube::hitTest(const QPoint& pos)
{
    int best = -1;
    float bestDepth = -1e10f;
    for (int i = 0; i < m_regions.size(); ++i) {
        if (!isRegionVisible(m_regions[i])) continue;
        QPolygonF poly = projectRegion(m_regions[i]);
        if (poly.containsPoint(pos, Qt::OddEvenFill)) {
            float d = regionDepth(m_regions[i]);
            if (d > bestDepth) { bestDepth = d; best = i; }
        }
    }
    return best;
}

// ============================================================
// Navigation
// ============================================================

void ViewCube::navigateTo(int regionIndex)
{
    if (regionIndex < 0 || regionIndex >= m_regions.size()) return;
    const auto& r = m_regions[regionIndex];
    startAnimation(r.viewDir, r.viewUp);
}

void ViewCube::applyDragRotation(int dx, int dy)
{
    float sensitivity = 0.008f;
    auto rotateAround = [](const Vector3f& v, const Vector3f& axis, float angle) -> Vector3f {
        float c = std::cos(angle), s = std::sin(angle);
        return v * c + axis.cross(v) * s + axis * (axis.dot(v)) * (1.0f - c);
    };

    float angleH = -dx * sensitivity;
    m_cameraDir = rotateAround(m_cameraDir, m_projUp, angleH).normalized();
    m_cameraUp  = rotateAround(m_cameraUp,  m_projUp, angleH).normalized();
    updateBasis();

    float angleV = dy * sensitivity;
    m_cameraDir = rotateAround(m_cameraDir, m_right, angleV).normalized();
    m_cameraUp  = rotateAround(m_cameraUp,  m_right, angleV).normalized();
    updateBasis();

    update();

    if (m_cameraCallback)
        m_cameraCallback(m_cameraDir, m_cameraUp, true);  // smooth=true
}

// ============================================================
// Animation
// ============================================================

void ViewCube::startAnimation(const Eigen::Vector3f& targetDir, const Eigen::Vector3f& targetUp)
{
    m_animStartDir = m_cameraDir;
    m_animStartUp = m_cameraUp;
    m_animEndDir = targetDir.normalized();
    m_animEndUp = targetUp.normalized();
    m_animProgress = 0;
    m_animating = true;
    m_animTimer->start();
}

void ViewCube::onAnimTick()
{
    m_animProgress += 16.0f / ANIM_DURATION_MS;
    if (m_animProgress >= 1.0f) m_animProgress = 1.0f;

    float t = m_animProgress;
    t = 1.0f - (1.0f - t) * (1.0f - t);  // ease-out quad

    auto slerp = [](const Vector3f& a, const Vector3f& b, float s) -> Vector3f {
        float d = std::min(1.0f, std::max(-1.0f, a.dot(b)));
        float theta = std::acos(d);
        if (theta < 1e-4f) return b;
        float sinT = std::sin(theta);
        return (std::sin((1.0f - s) * theta) / sinT) * a
             + (std::sin(s * theta) / sinT) * b;
    };

    Vector3f dir = slerp(m_animStartDir, m_animEndDir, t).normalized();
    Vector3f up  = slerp(m_animStartUp,  m_animEndUp,  t).normalized();
    m_cameraDir = dir;
    m_cameraUp = up;
    updateBasis();
    update();

    if (m_animProgress >= 1.0f) {
        m_animating = false;
        m_animTimer->stop();
    }

    if (m_cameraCallback)
        m_cameraCallback(dir, up, false);  // smooth=false → snap
}

// ============================================================
// Geometry generation
// ============================================================

void ViewCube::buildRegions()
{
    m_regions.clear();
    const float H = 0.5f;
    const float C = m_chamfer;
    const float I = H - C;

    auto makeV = [](float x, float y, float z) -> QVector3D {
        return QVector3D(x, y, z);
    };

    // Fix winding order so normals always point outward (away from cube center)
    // Without this, isRegionVisible() signed area test incorrectly culls many edges/corners
    auto fixWinding = [](Region& r) {
        // Compute center of region in 3D
        Vector3f center = Vector3f::Zero();
        for (int i = 0; i < r.cornerCount; ++i)
            center += Eigen::Vector3f(r.corners[i].x(), r.corners[i].y(), r.corners[i].z());
        center /= r.cornerCount;

        // Compute face normal via cross product of first two edges
        Vector3f v0(r.corners[0].x(), r.corners[0].y(), r.corners[0].z());
        Vector3f v1(r.corners[1].x(), r.corners[1].y(), r.corners[1].z());
        Vector3f v2(r.corners[2].x(), r.corners[2].y(), r.corners[2].z());
        Vector3f normal = (v1 - v0).cross(v2 - v0);

        // For convex shape at origin: outward normal aligns with center position.
        // If normal opposes center (dot < 0), it's inward — reverse winding.
        if (normal.dot(center) < 0) {
            std::reverse(r.corners, r.corners + r.cornerCount);
        }
    };

    // Helper: blend two colors
    auto blend2 = [](const QColor& a, const QColor& b) -> QColor {
        return QColor((a.red() + b.red()) / 2,
                      (a.green() + b.green()) / 2,
                      (a.blue() + b.blue()) / 2);
    };
    auto blend3 = [](const QColor& a, const QColor& b, const QColor& c) -> QColor {
        return QColor((a.red() + b.red() + c.red()) / 3,
                      (a.green() + b.green() + c.green()) / 3,
                      (a.blue() + b.blue() + c.blue()) / 3);
    };

    // Face colors
    const QColor cFront  = QColor(100, 160, 230);
    const QColor cBack   = QColor(230, 120, 100);
    const QColor cTop    = QColor(100, 210, 140);
    const QColor cBottom = QColor(210, 190, 100);
    const QColor cLeft   = QColor(180, 130, 220);
    const QColor cRight  = QColor(240, 160, 90);

    // ---- 6 Faces ----
    auto addFace = [&](int fi, QVector3D v0, QVector3D v1, QVector3D v2, QVector3D v3,
                        const QString& name, const Vector3f& dir, const Vector3f& up, const QColor& color) {
        Region r;
        r.type = TYPE_FACE; r.index = fi; r.cornerCount = 4;
        r.corners[0] = v0; r.corners[1] = v1; r.corners[2] = v2; r.corners[3] = v3;
        r.tooltip = name; r.viewDir = dir; r.viewUp = up; r.baseColor = color;
        m_regions.append(r);
    };

    addFace(0, makeV(-I,-H,-I), makeV(I,-H,-I), makeV(I,-H,I), makeV(-I,-H,I),
            "Front",  {0,-1,0}, {0,0,1}, cFront);
    addFace(1, makeV(I,H,-I), makeV(-I,H,-I), makeV(-I,H,I), makeV(I,H,I),
            "Back",   {0,1,0},  {0,0,1}, cBack);
    addFace(2, makeV(-I,-I,H), makeV(I,-I,H), makeV(I,I,H), makeV(-I,I,H),
            "Top",    {0,0,1},  {0,1,0}, cTop);
    addFace(3, makeV(-I,I,-H), makeV(I,I,-H), makeV(I,-I,-H), makeV(-I,-I,-H),
            "Bottom", {0,0,-1}, {0,1,0}, cBottom);
    addFace(4, makeV(-H,I,-I), makeV(-H,-I,-I), makeV(-H,-I,I), makeV(-H,I,I),
            "Left",   {-1,0,0}, {0,0,1}, cLeft);
    addFace(5, makeV(H,-I,-I), makeV(H,I,-I), makeV(H,I,I), makeV(H,-I,I),
            "Right",  {1,0,0},  {0,0,1}, cRight);

    // ---- 12 Edges ----
    auto addEdge = [&](int fixed1, int fixed2, int var,
                        float sign1, float sign2,
                        const QString& name,
                        const Vector3f& f1Dir, const Vector3f& f2Dir,
                        const QColor& color) {
        Region r;
        r.type = TYPE_EDGE; r.cornerCount = 4;
        float coords[4][3];
        for (int i = 0; i < 3; ++i) {
            if (i == var) {
                coords[0][i] = -I; coords[1][i] = I; coords[2][i] = I; coords[3][i] = -I;
            } else if (i == fixed1) {
                coords[0][i] = sign1 * H; coords[1][i] = sign1 * H;
                coords[2][i] = sign1 * I; coords[3][i] = sign1 * I;
            } else {
                coords[0][i] = sign2 * I; coords[1][i] = sign2 * I;
                coords[2][i] = sign2 * H; coords[3][i] = sign2 * H;
            }
        }
        for (int i = 0; i < 4; ++i)
            r.corners[i] = makeV(coords[i][0], coords[i][1], coords[i][2]);

        r.tooltip = name; r.baseColor = color;
        Vector3f dir = (f1Dir + f2Dir).normalized();
        r.viewDir = dir;
        Vector3f worldUp(0, 0, 1);
        if (std::abs(dir.z()) > 0.99f) worldUp = Vector3f(0, 1, 0);
        Vector3f right = dir.cross(worldUp).normalized();
        r.viewUp = right.cross(dir).normalized();
        fixWinding(r);
        m_regions.append(r);
    };

    // Edge definitions: addEdge(fixed1_axis, fixed2_axis, var_axis, sign1, sign2, ...)
    // fixed1 axis → H,H,I,I  |  fixed2 axis → I,I,H,H  |  var axis → -I,I,I,-I
    // y=axis1(y-/-), z=axis2(z+/-): x varies
    addEdge(1,2,0, -1.0f, 1.0f, "Top-Front",     {0,-1,0}, {0,0,1},   blend2(cFront, cTop));
    addEdge(1,2,0, -1.0f,-1.0f, "Bottom-Front",   {0,-1,0}, {0,0,-1},  blend2(cFront, cBottom));
    // x=axis0(x+/-), y=axis1(y-/-): z varies
    addEdge(0,1,2, -1.0f,-1.0f, "Left-Front",     {0,-1,0}, {-1,0,0},  blend2(cFront, cLeft));
    addEdge(0,1,2,  1.0f,-1.0f, "Right-Front",    {0,-1,0}, {1,0,0},   blend2(cFront, cRight));
    addEdge(0,1,2, -1.0f, 1.0f, "Left-Back",      {0,1,0},  {-1,0,0},  blend2(cBack, cLeft));
    addEdge(0,1,2,  1.0f, 1.0f, "Right-Back",     {0,1,0},  {1,0,0},   blend2(cBack, cRight));
    // x=axis0(x+/-), z=axis2(z+/-): y varies
    addEdge(0,2,1, -1.0f, 1.0f, "Top-Left",       {0,0,1},  {-1,0,0},  blend2(cTop, cLeft));
    addEdge(0,2,1,  1.0f, 1.0f, "Top-Right",      {0,0,1},  {1,0,0},   blend2(cTop, cRight));
    addEdge(0,2,1, -1.0f,-1.0f, "Bottom-Left",    {0,0,-1}, {-1,0,0},  blend2(cBottom, cLeft));
    addEdge(0,2,1,  1.0f,-1.0f, "Bottom-Right",   {0,0,-1}, {1,0,0},   blend2(cBottom, cRight));
    // y=axis1(y+), z=axis2(z-): x varies
    addEdge(1,2,0,  1.0f, 1.0f, "Top-Back",       {0,1,0},  {0,0,1},   blend2(cBack, cTop));
    addEdge(1,2,0,  1.0f,-1.0f, "Bottom-Back",    {0,1,0},  {0,0,-1},  blend2(cBack, cBottom));

    // ---- 8 Corners ----
    auto addCorner = [&](float sx, float sy, float sz,
                         const Vector3f& f1, const Vector3f& f2, const Vector3f& f3,
                         const QColor& color) {
        Region r;
        r.type = TYPE_CORNER; r.cornerCount = 3;
        r.corners[0] = makeV(sx * I, sy * H, sz * I);
        r.corners[1] = makeV(sx * H, sy * I, sz * I);
        r.corners[2] = makeV(sx * I, sy * I, sz * H);
        r.tooltip = "Isometric"; r.baseColor = color;
        Vector3f dir = (f1 + f2 + f3).normalized();
        r.viewDir = dir;
        Vector3f worldUp(0, 0, 1);
        if (std::abs(dir.z()) > 0.99f) worldUp = Vector3f(0, 1, 0);
        Vector3f right = dir.cross(worldUp).normalized();
        r.viewUp = right.cross(dir).normalized();
        fixWinding(r);
        m_regions.append(r);
    };

    addCorner( 1, -1,  1, {0,-1,0}, {1,0,0}, {0,0,1},   blend3(cFront, cRight, cTop));
    addCorner(-1, -1,  1, {0,-1,0}, {-1,0,0}, {0,0,1},   blend3(cFront, cLeft, cTop));
    addCorner( 1, -1, -1, {0,-1,0}, {1,0,0}, {0,0,-1},   blend3(cFront, cRight, cBottom));
    addCorner(-1, -1, -1, {0,-1,0}, {-1,0,0}, {0,0,-1},   blend3(cFront, cLeft, cBottom));
    addCorner( 1,  1,  1, {0,1,0}, {1,0,0}, {0,0,1},     blend3(cBack, cRight, cTop));
    addCorner(-1,  1,  1, {0,1,0}, {-1,0,0}, {0,0,1},     blend3(cBack, cLeft, cTop));
    addCorner( 1,  1, -1, {0,1,0}, {1,0,0}, {0,0,-1},    blend3(cBack, cRight, cBottom));
    addCorner(-1,  1, -1, {0,1,0}, {-1,0,0}, {0,0,-1},    blend3(cBack, cLeft, cBottom));
}

// ============================================================
// Painting
// ============================================================

void ViewCube::paintEvent(QPaintEvent*)
{
    if (!m_enabled) return;

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Minimal background for VTK compositing
    p.fillRect(rect(), QColor(0, 0, 0, 15));

    // Collect visible, sort back-to-front
    QVector<int> visible;
    for (int i = 0; i < m_regions.size(); ++i)
        if (isRegionVisible(m_regions[i])) visible.append(i);
    std::sort(visible.begin(), visible.end(), [this](int a, int b) {
        return regionDepth(m_regions[a]) < regionDepth(m_regions[b]);
    });

    drawAxes(p);

    for (int idx : visible)
        drawRegion(p, m_regions[idx], idx == m_hovered);

    for (int idx : visible)
        if (m_regions[idx].type == TYPE_FACE)
            drawFaceLabel(p, m_regions[idx]);
}

void ViewCube::drawAxes(QPainter& p)
{
    const float axisLen = 1.05f;
    float s = width() / 220.0f;
    const float arrowSize = 16.0f * s;
    const float labelOffset = 1.15f;

    struct Axis { Vector3f dir; QColor color; QString label; };
    Axis axes[] = {
        {{1,0,0}, QColor(230,70,70),   "X"},
        {{0,1,0}, QColor(70,195,70),   "Y"},
        {{0,0,1}, QColor(70,110,240),  "Z"},
    };

    QPointF origin = project(Vector3f::Zero());
    for (const auto& axis : axes) {
        QPointF tip = project(axis.dir * axisLen);
        QPointF labelPt = project(axis.dir * labelOffset);
        float dot = axis.dir.dot(m_cameraDir);
        float alpha = dot > 0 ? 240 : 140;
        QColor c = axis.color; c.setAlpha(alpha);

        QPen pen(c, (dot > 0 ? 5.0f : 3.0f) * s, dot > 0 ? Qt::SolidLine : Qt::DashLine);
        pen.setCapStyle(Qt::RoundCap);
        p.setPen(pen);
        p.drawLine(origin, tip);

        if (dot > 0) {
            QPointF unit = (tip - origin);
            qreal len = std::sqrt(unit.x()*unit.x() + unit.y()*unit.y());
            if (len > 0) unit /= len;
            QPointF perp(-unit.y(), unit.x());
            QPolygonF arrow;
            arrow << tip
                   << tip - QPointF(unit.x()*arrowSize + perp.x()*arrowSize*0.4,
                                   unit.y()*arrowSize + perp.y()*arrowSize*0.4)
                   << tip - QPointF(unit.x()*arrowSize - perp.x()*arrowSize*0.4,
                                   unit.y()*arrowSize - perp.y()*arrowSize*0.4);
            p.setPen(Qt::NoPen); p.setBrush(c);
            p.drawPolygon(arrow);
        }

        QFont font("Arial", std::max(8, static_cast<int>(15 * s)), QFont::Bold);
        p.setFont(font); p.setPen(c);
        p.drawText(labelPt, axis.label);
    }
}

void ViewCube::drawRegion(QPainter& p, const Region& r, bool hovered)
{
    float s = width() / 220.0f;
    QPolygonF poly = projectRegion(r);
    if (poly.size() < 3) return;

    // Use baseColor for all region types
    float facing = std::min(1.0f, 0.55f + 0.45f * r.viewDir.dot(m_cameraDir));
    int factor = static_cast<int>(facing * 60 + 180);
    QColor fillColor(
        std::min(255, r.baseColor.red()   * factor / 255),
        std::min(255, r.baseColor.green() * factor / 255),
        std::min(255, r.baseColor.blue()  * factor / 255),
        230
    );

    if (hovered) fillColor = QColor(255, 255, 255, 160);

    // Use same-color outline to cover anti-aliasing gaps between adjacent regions
    QColor outlineColor = fillColor;
    outlineColor.setAlpha(255);
    p.setPen(QPen(outlineColor, (hovered ? 2.5 : 2.0) * s));
    p.setBrush(fillColor);
    p.drawPolygon(poly);
}

void ViewCube::drawFaceLabel(QPainter& p, const Region& r)
{
    if (r.type != TYPE_FACE) return;
    float s = width() / 220.0f;
    QPointF center = project(r.viewDir * 0.5f);
    QFont font("Arial", std::max(6, static_cast<int>(10 * s)), QFont::Bold);
    p.setFont(font);
    QFontMetrics fm(font);
    QRect textRect = fm.boundingRect(r.tooltip);
    textRect.moveCenter(center.toPoint());
    p.setPen(QColor(30, 30, 30, 180));
    p.drawText(textRect.translated(1, 1), Qt::AlignCenter, r.tooltip);
    p.setPen(Qt::white);
    p.drawText(textRect, Qt::AlignCenter, r.tooltip);
}

// ============================================================
// Mouse interaction
// ============================================================

void ViewCube::mousePressEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) { event->ignore(); return; }
    m_pressPos = event->pos();
    m_lastDragPos = event->pos();
    m_dragging = false;
    event->accept();
}

void ViewCube::mouseMoveEvent(QMouseEvent* event)
{
    if (m_animating) return;

    if (event->buttons() & Qt::LeftButton) {
        if (!m_dragging) {
            int dx = event->pos().x() - m_pressPos.x();
            int dy = event->pos().y() - m_pressPos.y();
            if (dx*dx + dy*dy > DRAG_THRESHOLD * DRAG_THRESHOLD) {
                m_dragging = true;
                if (m_animTimer && m_animTimer->isActive()) {
                    m_animTimer->stop();
                    m_animating = false;
                }
                setCursor(Qt::ClosedHandCursor);
                QToolTip::hideText();
            }
        }
        if (m_dragging) {
            int dx = event->pos().x() - m_lastDragPos.x();
            int dy = event->pos().y() - m_lastDragPos.y();
            m_lastDragPos = event->pos();
            applyDragRotation(dx, dy);
        }
        return;
    }

    // Hover
    int hit = hitTest(event->pos());
    if (hit != m_hovered) {
        m_hovered = hit; update();
        if (hit >= 0) {
            QToolTip::showText(event->globalPos(), m_regions[hit].tooltip, this);
            setCursor(Qt::PointingHandCursor);
        } else {
            QToolTip::hideText(); setCursor(Qt::ArrowCursor);
        }
    } else if (hit >= 0) {
        QToolTip::showText(event->globalPos(), m_regions[hit].tooltip, this);
    }
}

void ViewCube::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) { event->ignore(); return; }
    if (!m_dragging) {
        int hit = hitTest(m_pressPos);
        if (hit >= 0) navigateTo(hit);
    } else {
        setCursor(Qt::ArrowCursor);
    }
    m_dragging = false;
    event->accept();
}

void ViewCube::leaveEvent(QEvent*)
{
    if (m_hovered != -1) { m_hovered = -1; update(); QToolTip::hideText(); setCursor(Qt::ArrowCursor); }
    m_dragging = false;
}
