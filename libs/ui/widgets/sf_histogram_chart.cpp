#include "sf_histogram_chart.h"

#include <cmath>
#include <algorithm>
#include <QGraphicsLayout>

namespace ct {

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

    rebuildChart();
}

void SFHistogramChart::clear()
{
    if (m_bar_series) {
        m_chart->removeSeries(m_bar_series);
        delete m_bar_series;
        m_bar_series = nullptr;
    }
    m_bar_set_in = nullptr;
    m_bar_set_out = nullptr;
    m_bin_counts.clear();
    m_values.clear();
    m_left_line->replace(QVector<QPointF>{});
    m_right_line->replace(QVector<QPointF>{});
}

void SFHistogramChart::setDisplayRange(double min_val, double max_val)
{
    m_display_min = min_val;
    m_display_max = max_val;
    rebuildChart();
}

void SFHistogramChart::setFullRange(double min_val, double max_val)
{
    m_data_min = min_val;
    m_data_max = max_val;
    m_display_min = min_val;
    m_display_max = max_val;
    rebuildChart();
}

void SFHistogramChart::setColormap(ColormapType type)
{
    m_colormap = type;
    rebuildChart();
}

void SFHistogramChart::setShowNaNGrey(bool show)
{
    m_show_nan_grey = show;
    rebuildChart();
}

void SFHistogramChart::rebuildChart()
{
    if (m_bar_series) {
        m_chart->removeSeries(m_bar_series);
        delete m_bar_series;
    }

    // Two bar sets: in-range (colormap colored) and out-of-range (grey)
    m_bar_set_in = new QBarSet("in");
    m_bar_set_in->setColor(QColor(100, 149, 237)); // default, will be overwritten
    m_bar_set_out = new QBarSet("out");
    m_bar_set_out->setColor(QColor(180, 180, 180));

    m_bar_series = new QBarSeries;
    m_bar_series->append(m_bar_set_out);
    m_bar_series->append(m_bar_set_in);
    m_chart->addSeries(m_bar_series);
    m_bar_series->attachAxis(m_x_axis);
    m_bar_series->attachAxis(m_y_axis);

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

    // Use the colormap to set the in-range bar color
    const auto& lut = getColormapLUT(m_colormap);
    int lut_size = static_cast<int>(lut.size());
    float mid_t = static_cast<float>((0.5 - m_display_min) / (m_display_max - m_display_min));
    if (mid_t < 0.0f) mid_t = 0.0f;
    if (mid_t > 1.0f) mid_t = 1.0f;
    int mid_idx = static_cast<int>(mid_t * (lut_size - 1));
    float lutVal = lut[mid_idx >= 0 && mid_idx < lut_size ? mid_idx : 0];
    uint32_t packed;
    std::memcpy(&packed, &lutVal, sizeof(packed));
    int r = (packed >> 16) & 0xFF;
    int g = (packed >> 8) & 0xFF;
    int b = packed & 0xFF;
    m_bar_set_in->setColor(QColor(r, g, b));

    QList<qreal> counts_in;
    QList<qreal> counts_out;
    for (int c : m_bin_counts) {
        double bin_center = m_data_min + (counts_in.size() + 0.5) * m_bin_width;
        bool out_of_range = m_show_nan_grey &&
            (bin_center < m_display_min || bin_center > m_display_max);
        if (out_of_range) {
            counts_in << 0;
            counts_out << c;
        } else {
            counts_in << c;
            counts_out << 0;
        }
    }
    m_bar_set_in->append(counts_in);
    m_bar_set_out->append(counts_out);

    m_x_axis->setRange(m_data_min, m_data_max);
    m_x_axis->setTickCount(std::min(m_bin_count + 1, 7));

    int max_count = *std::max_element(m_bin_counts.begin(), m_bin_counts.end());
    if (max_count <= 0) max_count = 1;
    m_y_axis->setRange(0, max_count * 1.05);
    m_y_axis->applyNiceNumbers();

    updateBoundaryLines();
}

void SFHistogramChart::updateBoundaryLines()
{
    if (!m_y_axis) return;

    double y_min = m_y_axis->min();
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

    rebuildChart();
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

}  // namespace ct
