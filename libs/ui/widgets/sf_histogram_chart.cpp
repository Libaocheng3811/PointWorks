#include "sf_histogram_chart.h"

#include <cmath>
#include <algorithm>
#include <cstring>
#include <QGraphicsLayout>

namespace pw {

SFHistogramChart::SFHistogramChart(QWidget* parent)
    : QChartView(parent), m_chart(new QChart)
{
    m_chart->setMargins({0, 0, 0, 0});
    m_chart->layout()->setContentsMargins(2, 2, 2, 2);
    m_chart->legend()->hide();
    m_chart->setBackgroundRoundness(0);

    m_x_axis = new QValueAxis;
    m_x_axis->setLabelFormat("%.2f");
    m_x_axis->setTitleText("");

    m_y_axis = new QValueAxis;
    m_y_axis->setTitleText("");
    m_y_axis->setLabelFormat("%d");

    m_chart->addAxis(m_x_axis, Qt::AlignBottom);
    m_chart->addAxis(m_y_axis, Qt::AlignLeft);

    // Boundary lines
    m_left_line = new QLineSeries;
    m_left_line->setPen(QPen(QColor(200, 0, 0), 2, Qt::DashLine));
    m_chart->addSeries(m_left_line);
    m_left_line->attachAxis(m_x_axis);
    m_left_line->attachAxis(m_y_axis);

    m_right_line = new QLineSeries;
    m_right_line->setPen(QPen(QColor(200, 0, 0), 2, Qt::DashLine));
    m_chart->addSeries(m_right_line);
    m_right_line->attachAxis(m_x_axis);
    m_right_line->attachAxis(m_y_axis);

    setChart(m_chart);
    setRenderHint(QPainter::Antialiasing);
    this->setRubberBand(QChartView::NoRubberBand);
    setCacheMode(QGraphicsView::CacheNone);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
}

void SFHistogramChart::setData(const std::vector<float>& values, int bin_count)
{
    m_values = values;
    m_bin_count = bin_count;

    if (m_values.empty()) {
        clear();
        return;
    }

    m_data_min = 1e30;
    m_data_max = -1e30;
    for (float v : m_values) {
        if (std::isnan(v)) continue;
        if (v < m_data_min) m_data_min = v;
        if (v > m_data_max) m_data_max = v;
    }

    m_display_min = m_data_min;
    m_display_max = m_data_max;

    rebuildBins();
    updateAxes();
    updateBoundaryLines();
}

void SFHistogramChart::clear()
{
    m_bin_counts.clear();
    m_values.clear();
    m_left_line->replace(QVector<QPointF>{});
    m_right_line->replace(QVector<QPointF>{});
    update();
}

void SFHistogramChart::rebuildBins()
{
    double range = m_data_max - m_data_min;
    if (range < 1e-12) range = 1.0;
    m_bin_width = range / m_bin_count;
    m_bin_counts.assign(m_bin_count, 0);

    for (float v : m_values) {
        if (std::isnan(v)) continue;
        int bin = static_cast<int>((v - m_data_min) / m_bin_width);
        if (bin >= m_bin_count) bin = m_bin_count - 1;
        if (bin < 0) bin = 0;
        m_bin_counts[bin]++;
    }
}

void SFHistogramChart::setDisplayRange(double min_val, double max_val)
{
    m_display_min = min_val;
    m_display_max = max_val;
    updateBoundaryLines();
    viewport()->repaint();
}

void SFHistogramChart::setFullRange(double min_val, double max_val)
{
    m_data_min = min_val;
    m_data_max = max_val;
    m_display_min = min_val;
    m_display_max = max_val;
    rebuildBins();
    updateAxes();
    updateBoundaryLines();
    viewport()->repaint();
}

void SFHistogramChart::setColormap(ColormapType type)
{
    m_colormap = type;
    update();
}

void SFHistogramChart::setColorRange(double min_val, double max_val)
{
    m_color_min = min_val;
    m_color_max = max_val;
    update();
}

void SFHistogramChart::setShowNaNGrey(bool show)
{
    m_show_nan_grey = show;
    update();
}

void SFHistogramChart::updateAxes()
{
    m_x_axis->setRange(m_data_min, m_data_max);
    m_x_axis->setTickCount(std::min(m_bin_count + 1, 7));

    int max_count = 0;
    if (!m_bin_counts.empty())
        max_count = *std::max_element(m_bin_counts.begin(), m_bin_counts.end());
    if (max_count <= 0) max_count = 1;
    m_y_axis->setRange(0, max_count * 1.05);
    m_y_axis->applyNiceNumbers();
}

void SFHistogramChart::updateBoundaryLines()
{
    if (!m_y_axis) return;

    double y_min = 0;
    double y_max = m_y_axis->max();

    m_left_line->replace(QVector<QPointF>{
        QPointF(m_display_min, y_min),
        QPointF(m_display_min, y_max)
    });
    m_right_line->replace(QVector<QPointF>{
        QPointF(m_display_max, y_min),
        QPointF(m_display_max, y_max)
    });
}

QRgb SFHistogramChart::colormapColor(double value) const
{
    double color_range = m_color_max - m_color_min;
    if (color_range < 1e-12) color_range = 1.0;
    double t = (value - m_color_min) / color_range;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    const auto& lut_data = getColormapLUT(m_colormap);
    int lut_size = static_cast<int>(lut_data.size());
    int src_idx = static_cast<int>(t * (lut_size - 1));
    if (src_idx >= lut_size) src_idx = lut_size - 1;

    float lutVal = lut_data[src_idx];
    uint32_t packed;
    std::memcpy(&packed, &lutVal, sizeof(packed));
    int r = (packed >> 16) & 0xFF;
    int g = (packed >> 8) & 0xFF;
    int b = packed & 0xFF;
    return qRgb(r, g, b);
}

void SFHistogramChart::paintEvent(QPaintEvent* event)
{
    QChartView::paintEvent(event);

    if (m_bin_counts.empty()) return;

    QPainter p(viewport());
    p.setRenderHint(QPainter::Antialiasing);

    int max_count = *std::max_element(m_bin_counts.begin(), m_bin_counts.end());
    if (max_count <= 0) return;

    double disp_range = m_display_max - m_display_min;
    if (disp_range < 1e-12) disp_range = 1.0;

    // Get boundary pixel positions for clipping
    QPointF disp_left_px = m_chart->mapToPosition(QPointF(m_display_min, 0));
    QPointF disp_right_px = m_chart->mapToPosition(QPointF(m_display_max, 0));
    int clip_left = static_cast<int>(disp_left_px.x());
    int clip_right = static_cast<int>(disp_right_px.x());

    for (int i = 0; i < m_bin_count; ++i) {
        double bin_start = m_data_min + i * m_bin_width;
        double bin_end = m_data_min + (i + 1) * m_bin_width;
        double bin_center = (bin_start + bin_end) / 2.0;
        int count = m_bin_counts[i];
        if (count <= 0) continue;

        // Map bin position to pixel coordinates
        QPointF px_left = m_chart->mapToPosition(QPointF(bin_start, 0));
        QPointF px_right = m_chart->mapToPosition(QPointF(bin_end, 0));
        QPointF px_top = m_chart->mapToPosition(QPointF(bin_center, count));
        QPointF px_base = m_chart->mapToPosition(QPointF(bin_center, 0));

        int bar_left = static_cast<int>(px_left.x());
        int bar_right = static_cast<int>(px_right.x());
        int bar_top = static_cast<int>(px_top.y());
        int bar_bottom = static_cast<int>(px_base.y());

        // Determine if the bar's pixel range overlaps with [clip_left, clip_right]
        bool fully_in = (bar_left >= clip_left && bar_right <= clip_right);
        bool fully_out = (bar_right <= clip_left || bar_left >= clip_right);

        p.setPen(Qt::NoPen);

        if (fully_out) {
            // Entire bar is outside display range -> grey
            p.setBrush(QColor(180, 180, 180));
            p.drawRect(bar_left, bar_top, bar_right - bar_left, bar_bottom - bar_top);
        } else if (fully_in) {
            // Entire bar is inside display range -> colormap color
            QRgb color = colormapColor(bin_center);
            p.setBrush(QColor(color));
            p.drawRect(bar_left, bar_top, bar_right - bar_left, bar_bottom - bar_top);
        } else {
            // Bar straddles boundary: split into in-range and out-of-range parts
            QRgb color = colormapColor(bin_center);

            // Left part (before clip_left or clip_right)
            int split = (bar_left < clip_left) ? clip_left :
                        (bar_right > clip_right) ? clip_right : bar_right;

            // Out-of-range portion
            p.setBrush(QColor(180, 180, 180));
            if (bar_left < clip_left) {
                p.drawRect(bar_left, bar_top, clip_left - bar_left, bar_bottom - bar_top);
            }
            if (bar_right > clip_right) {
                p.drawRect(clip_right, bar_top, bar_right - clip_right, bar_bottom - bar_top);
            }

            // In-range portion
            int in_left = std::max(bar_left, clip_left);
            int in_right = std::min(bar_right, clip_right);
            p.setBrush(QColor(color));
            p.drawRect(in_left, bar_top, in_right - in_left, bar_bottom - bar_top);
        }
    }
}

void SFHistogramChart::mousePressEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        m_drag_target = None;
        return;
    }

    QPointF chart_pos = m_chart->mapToValue(event->pos());
    double x = chart_pos.x();

    double dist_left = std::abs(x - m_display_min);
    double dist_right = std::abs(x - m_display_max);
    double threshold = (m_data_max - m_data_min) * 0.02;
    if (threshold < 1e-9) threshold = 0.01;

    if (dist_left < threshold && dist_left <= dist_right) {
        m_drag_target = Left;
        m_dragging = true;
    } else if (dist_right < threshold) {
        m_drag_target = Right;
        m_dragging = true;
    } else {
        m_drag_target = None;
    }
}

void SFHistogramChart::mouseMoveEvent(QMouseEvent* event)
{
    if (!m_dragging || m_drag_target == None) {
        QChartView::mouseMoveEvent(event);
        return;
    }

    QPointF chart_pos = m_chart->mapToValue(event->pos());
    double x = chart_pos.x();

    if (m_drag_target == Left) {
        if (x < m_data_min) x = m_data_min;
        if (x >= m_display_max - (m_data_max - m_data_min) * 0.01) return;
        m_display_min = x;
    } else if (m_drag_target == Right) {
        if (x > m_data_max) x = m_data_max;
        if (x <= m_display_min + (m_data_max - m_data_min) * 0.01) return;
        m_display_max = x;
    }

    updateBoundaryLines();
    viewport()->repaint();
}

void SFHistogramChart::mouseReleaseEvent(QMouseEvent* event)
{
    if (m_dragging && m_drag_target != None) {
        m_dragging = false;
        emit rangeChanged(m_display_min, m_display_max);
    }
    m_drag_target = None;
    QChartView::mouseReleaseEvent(event);
}

}  // namespace pw
