#include "scalar_bar_widget.h"

#include <QPainter>
#include <QPainterPath>
#include <QFontMetrics>
#include <algorithm>
#include <cstring>
#include <cmath>

namespace ct {

ScalarBarWidget::ScalarBarWidget(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_TranslucentBackground);
}

ScalarBarWidget::~ScalarBarWidget() = default;

QString ScalarBarWidget::formatLabel(double val) const
{
    double range = m_data_max - m_data_min;
    if (range <= 0) return QString::number(val, 'f', 2);

    // Determine decimal places so all labels have uniform width
    int decimals = 2;
    if (range < 0.1)       decimals = 4;
    else if (range < 1.0)  decimals = 3;
    else if (range < 100)  decimals = 2;
    else                   decimals = 1;

    return QString::number(val, 'f', decimals);
}

QRgb ScalarBarWidget::colorFromLUT(double t) const
{
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

void ScalarBarWidget::relayout()
{
    if (!parentWidget()) return;

    QFont labelFont("Arial", 9);
    labelFont.setBold(true);
    QFontMetrics labelFm(labelFont);

    m_label_width = 0;
    if (m_data_max != m_data_min) {
        for (int i = 0; i <= 4; ++i) {
            double val = m_data_min + (m_data_max - m_data_min) * i / 4.0;
            QString text = formatLabel(val);
            m_label_width = std::max(m_label_width, labelFm.horizontalAdvance(text));
        }
    }

    // Title font exactly matches label font
    QFont titleFont("Arial", 9);
    titleFont.setBold(true);
    QFontMetrics titleFm(titleFont);
    m_title_font_size = 9;
    m_title_height = m_title.isEmpty() ? 0 : (titleFm.height() + TITLE_BAR_GAP);

    // Bottom padding: account for label font descent so bottom label isn't clipped
    int label_descent = labelFm.descent();

    int curve_extra = m_bin_counts.empty() ? 0 : (CURVE_GAP + CURVE_WIDTH);
    int curve_w = (m_bin_counts.empty() || !m_show_curve) ? 0 : curve_extra;

    int bar_x = m_label_width + LABEL_GAP;
    int content_width = bar_x + BAR_WIDTH + curve_w + RIGHT_MARGIN;

    // Ensure title centered on bar doesn't clip past left edge
    m_left_pad = 0;
    if (!m_title.isEmpty()) {
        int title_w = titleFm.horizontalAdvance(m_title);
        int title_left = bar_x + BAR_WIDTH / 2 - title_w / 2;
        if (title_left < 0) m_left_pad = -title_left;
    }

    m_total_width = content_width + m_left_pad;

    int pw = parentWidget()->width();
    int ph = parentWidget()->height();

    int avail = ph - TOP_MARGIN - m_title_height - m_bottom_margin_px - label_descent - 2;
    m_bar_height = std::max(avail, 50);

    int x = pw - m_total_width - 6;
    int y = TOP_MARGIN;
    int total_h = m_bar_height + m_title_height + label_descent + 2;
    setGeometry(x, y, m_total_width, total_h);
}

QImage ScalarBarWidget::buildGradientImage(ColormapType type, int height) const
{
    const auto& lut_data = getColormapLUT(type);
    int lut_size = static_cast<int>(lut_data.size());

    QImage img(BAR_WIDTH, height, QImage::Format_RGB32);
    for (int y = 0; y < height; ++y) {
        float t = 1.0f - static_cast<float>(y) / std::max(height - 1, 1);
        int src_idx = static_cast<int>(t * (lut_size - 1));
        if (src_idx >= lut_size) src_idx = lut_size - 1;

        float lutVal = lut_data[src_idx];
        uint32_t packed;
        std::memcpy(&packed, &lutVal, sizeof(packed));
        int r = (packed >> 16) & 0xFF;
        int g = (packed >> 8) & 0xFF;
        int b = packed & 0xFF;

        QRgb color = qRgb(r, g, b);
        for (int x = 0; x < BAR_WIDTH; ++x) {
            img.setPixel(x, y, color);
        }
    }
    return img;
}

void ScalarBarWidget::paintEvent(QPaintEvent*)
{
    if (!m_visible) return;

    QPainter p(this);
    p.setRenderHint(QPainter::TextAntialiasing);
    p.setRenderHint(QPainter::Antialiasing);

    QFont labelFont("Arial", 9);
    labelFont.setBold(true);
    QFont titleFont("Arial", 9);
    titleFont.setBold(true);
    QFontMetrics titleFm(titleFont);
    QFontMetrics labelFm(labelFont);

    // Layout positions
    int bar_x = m_label_width + LABEL_GAP + m_left_pad;
    int draw_bar_y = m_title_height;
    int curve_x = bar_x + BAR_WIDTH + CURVE_GAP;
    int curve_w = m_bin_counts.empty() ? 0 : CURVE_WIDTH;

    // --- Draw title (centered on color bar) ---
    if (!m_title.isEmpty()) {
        p.setFont(titleFont);
        p.setPen(QColor(255, 255, 255));
        int title_center_x = bar_x + BAR_WIDTH / 2;
        int title_w = titleFm.horizontalAdvance(m_title);
        p.drawText(title_center_x - title_w / 2, titleFm.ascent(), m_title);
    }

    // --- Draw color gradient bar ---
    QImage grad = buildGradientImage(m_colormap, m_bar_height);
    p.drawImage(bar_x, draw_bar_y, grad);

    // Grey overlay for portions outside the actual data range
    bool has_display_range = (m_display_max > m_display_min + 1e-12);
    if (has_display_range) {
        double bar_range = m_data_max - m_data_min;
        if (bar_range > 1e-12) {
            // Bottom grey: from bar bottom to display_min
            double t_low = (m_display_min - m_data_min) / bar_range;
            if (t_low > 0.0) {
                int grey_h = static_cast<int>(t_low * m_bar_height);
                p.fillRect(bar_x, draw_bar_y + m_bar_height - grey_h,
                           BAR_WIDTH, grey_h,
                           QColor(180, 180, 180));
            }
            // Top grey: from display_max to bar top
            double t_high = (m_display_max - m_data_min) / bar_range;
            if (t_high < 1.0) {
                int grey_h = static_cast<int>((1.0 - t_high) * m_bar_height);
                p.fillRect(bar_x, draw_bar_y, BAR_WIDTH, grey_h,
                           QColor(180, 180, 180));
            }
        }
    }

    p.setPen(QPen(QColor(150, 150, 150), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(bar_x, draw_bar_y, BAR_WIDTH, m_bar_height);

    // --- Draw labels on the LEFT of color bar ---
    p.setFont(labelFont);
    if (m_data_max != m_data_min) {
        for (int i = 0; i <= 4; ++i) {
            double val = m_data_min + (m_data_max - m_data_min) * i / 4.0;
            int ly = draw_bar_y + m_bar_height - (i * m_bar_height / 4);

            p.setPen(QPen(QColor(150, 150, 150), 1));
            p.drawLine(bar_x - 2, ly, bar_x, ly);

            p.setPen(QColor(255, 255, 255));
            QString text = formatLabel(val);
            int text_w = labelFm.horizontalAdvance(text);
            int text_x = std::max(0, bar_x - LABEL_GAP - text_w);
            p.drawText(text_x, ly + labelFm.ascent() / 2, text);
        }
    }

    // --- Draw distribution curve on the RIGHT of color bar ---
    if (!m_bin_counts.empty() && curve_w > 0 && m_show_curve) {
        int bin_count = static_cast<int>(m_bin_counts.size());
        int max_count = *std::max_element(m_bin_counts.begin(), m_bin_counts.end());
        if (max_count <= 0) max_count = 1;

        double hist_range = m_hist_data_max - m_hist_data_min;
        if (hist_range < 1e-12) hist_range = 1.0;
        double data_range = m_data_max - m_data_min;
        if (data_range < 1e-12) data_range = 1.0;

        // Precompute curve points: px = curve_x + count/max*curve_w, py = bar position
        struct Pt { double px; double py; };
        std::vector<Pt> pts(bin_count);
        for (int i = 0; i < bin_count; ++i) {
            double bin_center = m_hist_data_min + (i + 0.5) * (hist_range / bin_count);
            int count = m_bin_counts[i];

            // y: map value to bar y (top=max, bottom=min)
            double val_t = (bin_center - m_data_min) / data_range;
            if (val_t < 0.0) val_t = 0.0;
            if (val_t > 1.0) val_t = 1.0;
            // val_t=1 -> top (y=0), val_t=0 -> bottom (y=bar_height)
            pts[i].py = draw_bar_y + static_cast<double>((1.0 - val_t) * m_bar_height);

            // x: map count to curve x offset
            pts[i].px = curve_x + static_cast<double>(count) / max_count * curve_w;
        }

        // Build smooth curve using Catmull-Rom -> Bezier conversion
        QPainterPath smooth_path;
        smooth_path.moveTo(pts[0].px, pts[0].py);
        for (int i = 0; i < bin_count - 1; ++i) {
            const Pt& p0 = pts[std::max(i - 1, 0)];
            const Pt& p1 = pts[i];
            const Pt& p2 = pts[i + 1];
            const Pt& p3 = pts[std::min(i + 2, bin_count - 1)];

            double tension = 0.3;
            double cp1x = p1.px + (p2.px - p0.px) * tension;
            double cp1y = p1.py + (p2.py - p0.py) * tension;
            double cp2x = p2.px - (p3.px - p1.px) * tension;
            double cp2y = p2.py - (p3.py - p1.py) * tension;

            smooth_path.cubicTo(cp1x, cp1y, cp2x, cp2y, p2.px, p2.py);
        }

        // Fill with a vertical gradient matching the color bar (top=max color, bottom=min color)
        // The curve area is bounded by: left = curve_x, right = the curve path
        // Close the path along the left edge (color bar side)
        QPainterPath fill_path = smooth_path;
        fill_path.lineTo(curve_x, pts[bin_count - 1].py);  // bottom-right of last point
        fill_path.lineTo(curve_x, pts[0].py);                // top-right of first point
        fill_path.closeSubpath();

        // Build a vertical linear gradient matching the color bar
        // Color bar: top (y=0) = max value (lut high end), bottom = min value (lut low end)
        QLinearGradient grad_fill(0, draw_bar_y, 0, draw_bar_y + m_bar_height);
        grad_fill.setSpread(QGradient::PadSpread);
        const auto& lut = getColormapLUT(m_colormap);
        int lut_size = static_cast<int>(lut.size());
        int num_stops = std::min(lut_size, 64);
        for (int s = 0; s < num_stops; ++s) {
            double stop_pos = static_cast<double>(s) / (num_stops - 1);
            // Reverse: stop_pos=0 (top) -> lut high end, stop_pos=1 (bottom) -> lut low end
            int lut_idx = static_cast<int>((1.0 - stop_pos) * (lut_size - 1));
            if (lut_idx >= lut_size) lut_idx = lut_size - 1;

            float lutVal = lut[lut_idx];
            uint32_t packed;
            std::memcpy(&packed, &lutVal, sizeof(packed));
            int r = (packed >> 16) & 0xFF;
            int g = (packed >> 8) & 0xFF;
            int b = packed & 0xFF;

            grad_fill.setColorAt(stop_pos, QColor(r, g, b, 80));
        }

        p.setPen(Qt::NoPen);
        p.setBrush(grad_fill);
        p.drawPath(fill_path);

        // Draw smooth curve line on top
        p.setPen(QPen(QColor(255, 255, 255, 200), 1.5));
        p.setBrush(Qt::NoBrush);
        p.drawPath(smooth_path);
    }

    // --- Zero line marker ---
    if (m_show_zero && m_data_min <= 0.0 && 0.0 <= m_data_max) {
        double range = m_data_max - m_data_min;
        if (range > 1e-9) {
            double t = -m_data_min / range;
            int zero_y = draw_bar_y + m_bar_height - static_cast<int>(t * m_bar_height);

            p.setPen(QPen(QColor(0, 0, 0), 2));
            p.drawLine(bar_x - 2, zero_y, bar_x + BAR_WIDTH + 1, zero_y);

            QFont zeroFont("Arial", 9);
            zeroFont.setBold(true);
            p.setFont(zeroFont);
            p.setPen(QColor(0, 0, 0));
            p.drawText(curve_x, zero_y + 4, "0");
        }
    }
}

void ScalarBarWidget::updateData(double min_val, double max_val,
                                  const QString& title,
                                  ColormapType colormap)
{
    m_data_min = min_val;
    m_data_max = max_val;
    m_display_min = min_val;
    m_display_max = max_val;
    m_title = title;
    m_colormap = colormap;
    if (m_visible) {
        relayout();
        update();
    }
}

void ScalarBarWidget::setBarVisible(bool visible)
{
    m_visible = visible;
    QWidget::setVisible(visible);
    if (visible) {
        relayout();
        update();
    }
}

bool ScalarBarWidget::isBarVisible() const
{
    return m_visible;
}

void ScalarBarWidget::setShowZeroLine(bool show)
{
    m_show_zero = show;
    if (m_visible) update();
}

void ScalarBarWidget::setBottomMargin(int pixels)
{
    m_bottom_margin_px = pixels;
    if (m_visible) {
        relayout();
        update();
    }
}

void ScalarBarWidget::setDisplayRange(double disp_min, double disp_max)
{
    m_display_min = disp_min;
    m_display_max = disp_max;
    if (m_visible) update();
}

void ScalarBarWidget::setHistogramData(const std::vector<int>& bin_counts,
                                        double data_min, double data_max,
                                        bool show_out_of_range_grey)
{
    m_bin_counts = bin_counts;
    m_hist_data_min = data_min;
    m_hist_data_max = data_max;
    if (m_visible) {
        relayout();
        update();
    }
}

void ScalarBarWidget::setShowCurve(bool show)
{
    m_show_curve = show;
    if (m_visible) {
        relayout();
        update();
    }
}

}  // namespace ct
