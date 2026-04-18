#ifndef POINTWORKS_SF_HISTOGRAM_CHART_H
#define POINTWORKS_SF_HISTOGRAM_CHART_H

#include "core/colormap.h"

#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLineSeries>

#include <QMouseEvent>
#include <QWidget>
#include <vector>

QT_CHARTS_USE_NAMESPACE

namespace ct {

class SFHistogramChart : public QChartView {
    Q_OBJECT
public:
    explicit SFHistogramChart(QWidget* parent = nullptr);

    void setData(const std::vector<float>& values, int bin_count = 50);
    void clear();

    void setDisplayRange(double min_val, double max_val);
    double displayMin() const { return m_display_min; }
    double displayMax() const { return m_display_max; }
    void setFullRange(double min_val, double max_val);

    void setColormap(ColormapType type);
    void setShowNaNGrey(bool show);

signals:
    void rangeChanged(double min_val, double max_val);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void rebuildChart();
    void updateBoundaryLines();

    std::vector<float> m_values;
    int m_bin_count = 50;
    double m_data_min = 0;
    double m_data_max = 1;
    double m_display_min = 0;
    double m_display_max = 1;
    ColormapType m_colormap = ColormapType::JET;
    bool m_show_nan_grey = true;

    QChart* m_chart = nullptr;
    QBarSet* m_bar_set_in = nullptr;
    QBarSet* m_bar_set_out = nullptr;
    QBarSeries* m_bar_series = nullptr;
    QValueAxis* m_x_axis = nullptr;
    QValueAxis* m_y_axis = nullptr;

    QLineSeries* m_left_line = nullptr;
    QLineSeries* m_right_line = nullptr;

    std::vector<int> m_bin_counts;
    double m_bin_width = 0;

    enum DragTarget { None, Left, Right };
    DragTarget m_drag_target = None;
    bool m_dragging = false;
    static constexpr int kHitThreshold = 10;
};

}  // namespace ct

#endif  // POINTWORKS_SF_HISTOGRAM_CHART_H
