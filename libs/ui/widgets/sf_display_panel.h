#ifndef POINTWORKS_SF_DISPLAY_PANEL_H
#define POINTWORKS_SF_DISPLAY_PANEL_H

#include "core/colormap.h"
#include "core/cloud.h"
#include "core/statistics.h"

#include <QWidget>
#include <QTabWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QLocale>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QSpacerItem>

namespace ct {

class SFHistogramChart;

class SFDisplayPanel : public QWidget {
    Q_OBJECT
public:
    explicit SFDisplayPanel(QWidget* parent = nullptr);

    void bindCloud(Cloud::Ptr cloud);
    void unbind();
    void setField(const QString& field_name);
    void setColormap(ColormapType colormap);
    void reloadField();

    std::function<void()> onColorChanged;

signals:
    void scalarBarRequested(double min_val, double max_val,
                            const QString& title, ColormapType colormap);
    void scalarBarToggled(bool visible);
    void scalarBarShowZero(bool show);

private slots:
    void onMinSpinBoxChanged(double val);
    void onMaxSpinBoxChanged(double val);
    void onHistogramRangeChanged(double min_val, double max_val);
    void onShowBarToggled(bool checked);
    void onBinsChanged(int count);
    void onShowNaNGreyToggled(bool checked);
    void onAlwaysShowZeroToggled(bool checked);
    void onSymmetricalToggled(bool checked);

private:
    void setupUi();
    void loadCurrentField();
    void refreshStatistics();
    void updateColorAndRefresh();

    Cloud::Ptr m_cloud;
    QString m_current_field;
    ColormapType m_current_colormap = ColormapType::JET;
    ScalarFieldStats m_stats;

    bool m_show_nan_grey = true;
    bool m_always_show_zero = false;
    bool m_symmetrical = false;

    QTabWidget* m_tab_widget;

    // Histogram Tab
    SFHistogramChart* m_histogram_chart;
    QDoubleSpinBox* m_spin_min;
    QDoubleSpinBox* m_spin_max;
    QLabel* m_label_stats;

    // Parameters Tab
    QCheckBox* m_check_show_bar;
    QSpinBox* m_spin_bins;
    QCheckBox* m_check_show_nan_grey;
    QCheckBox* m_check_always_show_zero;
    QCheckBox* m_check_symmetrical;
};

}  // namespace ct

#endif  // POINTWORKS_SF_DISPLAY_PANEL_H
