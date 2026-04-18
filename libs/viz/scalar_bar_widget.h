#ifndef POINTWORKS_SCALAR_BAR_WIDGET_H
#define POINTWORKS_SCALAR_BAR_WIDGET_H

#include "core/colormap.h"
#include "core/exports.h"

#include <QWidget>
#include <vector>

namespace ct {

class CT_VIZ_EXPORT ScalarBarWidget : public QWidget {
    Q_OBJECT
public:
    explicit ScalarBarWidget(QWidget* parent = nullptr);
    ~ScalarBarWidget() override;

    void updateData(double min_val, double max_val,
                    const QString& title = "",
                    ColormapType colormap = ColormapType::JET);

    void setBarVisible(bool visible);
    bool isBarVisible() const;

    void setShowZeroLine(bool show);
    void setBottomMargin(int pixels);
    void setDisplayRange(double disp_min, double disp_max);
    void setHistogramData(const std::vector<int>& bin_counts,
                          double data_min, double data_max,
                          bool show_out_of_range_grey = true);
    void setShowCurve(bool show);
    void relayout();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    static constexpr int BAR_WIDTH = 45;
    static constexpr int LABEL_GAP = 6;
    static constexpr int CURVE_WIDTH = 50;
    static constexpr int CURVE_GAP = 4;
    static constexpr int RIGHT_MARGIN = 50;
    static constexpr int TOP_MARGIN = 30;
    static constexpr int TITLE_BAR_GAP = 12;

    QRgb colorFromLUT(double t) const;
    QImage buildGradientImage(ColormapType type, int height) const;

    QString formatLabel(double val) const;

    double m_data_min = 0;
    double m_data_max = 1;
    QString m_title;
    ColormapType m_colormap = ColormapType::JET;
    bool m_visible = false;
    bool m_show_zero = false;
    int m_bottom_margin_px = 0;

    // Display range
    double m_display_min = 0;
    double m_display_max = 1;

    // Distribution curve data
    std::vector<int> m_bin_counts;
    double m_hist_data_min = 0;
    double m_hist_data_max = 1;
    bool m_show_curve = true;

    // Cached layout
    int m_bar_height = 0;
    int m_title_height = 0;
    int m_label_width = 0;
    int m_total_width = 0;
    int m_title_font_size = 10;
    int m_left_pad = 0;
};

}  // namespace ct

#endif  // POINTWORKS_SCALAR_BAR_WIDGET_H
