#include "sf_display_panel.h"
#include "sf_histogram_chart.h"

#include <QSignalBlocker>

namespace ct {

SFDisplayPanel::SFDisplayPanel(QWidget* parent)
    : QWidget(parent)
{
    setupUi();
}

void SFDisplayPanel::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(2, 4, 2, 4);
    main_layout->setSpacing(4);

    m_tab_widget = new QTabWidget(this);
    m_tab_widget->setDocumentMode(true);

    // ===== Tab 1: Histogram =====
    auto* hist_tab = new QWidget;
    auto* hist_layout = new QVBoxLayout(hist_tab);
    hist_layout->setContentsMargins(4, 4, 4, 4);
    hist_layout->setSpacing(4);

    // Min/Max SpinBox row
    auto* range_row = new QHBoxLayout;
    range_row->setSpacing(4);

    range_row->addWidget(new QLabel("Min"));
    m_spin_min = new QDoubleSpinBox;
    m_spin_min->setRange(-1e9, 1e9);
    m_spin_min->setDecimals(4);
    m_spin_min->setSingleStep(0.01);
    m_spin_min->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    range_row->addWidget(m_spin_min);

    range_row->addWidget(new QLabel("Max"));
    m_spin_max = new QDoubleSpinBox;
    m_spin_max->setRange(-1e9, 1e9);
    m_spin_max->setDecimals(4);
    m_spin_max->setSingleStep(0.01);
    m_spin_max->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    range_row->addWidget(m_spin_max);

    hist_layout->addLayout(range_row);

    // Histogram chart
    m_histogram_chart = new SFHistogramChart;
    m_histogram_chart->setMinimumHeight(160);
    m_histogram_chart->setMaximumHeight(240);
    hist_layout->addWidget(m_histogram_chart);

    // Statistics label
    m_label_stats = new QLabel;
    m_label_stats->setWordWrap(true);
    m_label_stats->setStyleSheet("color: #555;");
    hist_layout->addWidget(m_label_stats);

    hist_layout->addStretch();

    m_tab_widget->addTab(hist_tab, "Histogram");

    // ===== Tab 2: Parameters =====
    auto* params_tab = new QWidget;
    auto* params_layout = new QFormLayout(params_tab);
    params_layout->setContentsMargins(4, 8, 4, 4);
    params_layout->setSpacing(6);

    m_check_show_bar = new QCheckBox;
    m_check_show_bar->setChecked(true);
    params_layout->addRow("Show Bar:", m_check_show_bar);

    m_spin_bins = new QSpinBox;
    m_spin_bins->setRange(5, 500);
    m_spin_bins->setValue(50);
    params_layout->addRow("Bins:", m_spin_bins);

    m_check_show_nan_grey = new QCheckBox;
    m_check_show_nan_grey->setChecked(true);
    params_layout->addRow("Show NaN/out of range as grey:", m_check_show_nan_grey);

    m_check_always_show_zero = new QCheckBox;
    params_layout->addRow("Always show 0 in color scale:", m_check_always_show_zero);

    m_check_symmetrical = new QCheckBox;
    params_layout->addRow("Symmetrical color scale:", m_check_symmetrical);

    params_layout->addItem(new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding));

    m_tab_widget->addTab(params_tab, "Parameters");

    main_layout->addWidget(m_tab_widget);

    // ===== Connections =====
    connect(m_spin_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &SFDisplayPanel::onMinSpinBoxChanged);
    connect(m_spin_max, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &SFDisplayPanel::onMaxSpinBoxChanged);
    QObject::connect(m_histogram_chart, &SFHistogramChart::rangeChanged,
            this, &SFDisplayPanel::onHistogramRangeChanged);
    connect(m_check_show_bar, &QCheckBox::toggled,
            this, &SFDisplayPanel::onShowBarToggled);
    connect(m_spin_bins, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
            this, &SFDisplayPanel::onBinsChanged);
    connect(m_check_show_nan_grey, &QCheckBox::toggled,
            this, &SFDisplayPanel::onShowNaNGreyToggled);
    connect(m_check_always_show_zero, &QCheckBox::toggled,
            this, &SFDisplayPanel::onAlwaysShowZeroToggled);
    connect(m_check_symmetrical, &QCheckBox::toggled,
            this, &SFDisplayPanel::onSymmetricalToggled);
}

void SFDisplayPanel::bindCloud(Cloud::Ptr cloud)
{
    m_cloud = cloud;
    if (!cloud) return;

    // Load first field
    auto fields = cloud->getScalarFieldNames();
    if (!fields.empty()) {
        m_current_field = QString::fromStdString(fields[0]);
        loadCurrentField();
    }
}

void SFDisplayPanel::unbind()
{
    m_cloud.reset();
    m_histogram_chart->clear();
    m_label_stats->clear();
}

void SFDisplayPanel::setField(const QString& field_name)
{
    if (field_name == m_current_field) return;
    m_current_field = field_name;
    loadCurrentField();
    updateColorAndRefresh();
}

void SFDisplayPanel::setColormap(ColormapType colormap)
{
    if (colormap == m_current_colormap) return;
    m_current_colormap = colormap;
    m_histogram_chart->setColormap(m_current_colormap);
    updateColorAndRefresh();
}

void SFDisplayPanel::reloadField()
{
    loadCurrentField();
}

void SFDisplayPanel::loadCurrentField()
{
    if (!m_cloud || m_current_field.isEmpty()) return;

    auto* sf_data = m_cloud->getScalarField(m_current_field.toStdString());
    if (!sf_data) return;

    // Compute statistics
    m_stats = computeStatistics(*sf_data);

    // Update spin boxes
    QSignalBlocker bmin(m_spin_min);
    QSignalBlocker bmax(m_spin_max);
    m_spin_min->setValue(m_stats.min_val);
    m_spin_min->setRange(m_stats.min_val, m_stats.max_val);
    m_spin_max->setValue(m_stats.max_val);
    m_spin_max->setRange(m_stats.min_val, m_stats.max_val);

    // Update histogram
    m_histogram_chart->setData(*sf_data, m_spin_bins->value());
    m_histogram_chart->setDisplayRange(m_stats.min_val, m_stats.max_val);
    m_histogram_chart->setColormap(m_current_colormap);
    m_histogram_chart->setShowNaNGrey(m_show_nan_grey);

    // Update statistics label
    refreshStatistics();

    // Emit scalar bar request
    emit scalarBarRequested(m_stats.min_val, m_stats.max_val,
                            m_current_field, m_current_colormap);
}

void SFDisplayPanel::refreshStatistics()
{
    if (m_stats.total_points == 0) {
        m_label_stats->setText("No data");
        return;
    }

    double nan_pct = m_stats.total_points > 0
        ? (100.0 * m_stats.nan_points / m_stats.total_points)
        : 0.0;

    m_label_stats->setText(
        QString("Count: %1 | NaN: %2 (%3%)\n"
                "Mean: %4 | Std: %5 | Median: %6")
        .arg(QLocale().toString(m_stats.valid_points))
        .arg(m_stats.nan_points)
        .arg(nan_pct, 0, 'f', 2)
        .arg(m_stats.mean_val, 0, 'f', 4)
        .arg(m_stats.std_val, 0, 'f', 4)
        .arg(m_stats.median_val, 0, 'f', 4)
    );
}

void SFDisplayPanel::updateColorAndRefresh()
{
    if (!m_cloud || m_current_field.isEmpty()) return;

    float dmin = static_cast<float>(m_spin_min->value());
    float dmax = static_cast<float>(m_spin_max->value());

    m_cloud->updateColorByField(m_current_field.toStdString(),
                                 dmin, dmax,
                                 m_current_colormap,
                                 m_show_nan_grey);

    if (onColorChanged) onColorChanged();

    emit scalarBarRequested(dmin, dmax, m_current_field, m_current_colormap);
}

void SFDisplayPanel::onMinSpinBoxChanged(double val)
{
    if (val >= m_spin_max->value()) return;
    m_histogram_chart->setDisplayRange(val, m_spin_max->value());
    updateColorAndRefresh();
}

void SFDisplayPanel::onMaxSpinBoxChanged(double val)
{
    if (val <= m_spin_min->value()) return;
    m_histogram_chart->setDisplayRange(m_spin_min->value(), val);
    updateColorAndRefresh();
}

void SFDisplayPanel::onHistogramRangeChanged(double min_val, double max_val)
{
    QSignalBlocker bmin(m_spin_min);
    QSignalBlocker bmax(m_spin_max);
    m_spin_min->setValue(min_val);
    m_spin_max->setValue(max_val);
    updateColorAndRefresh();
}

void SFDisplayPanel::onShowBarToggled(bool checked)
{
    emit scalarBarToggled(checked);
}

void SFDisplayPanel::onBinsChanged(int count)
{
    if (!m_cloud || m_current_field.isEmpty()) return;
    auto* sf_data = m_cloud->getScalarField(m_current_field.toStdString());
    if (!sf_data) return;
    m_histogram_chart->setData(*sf_data, count);
    m_histogram_chart->setDisplayRange(m_spin_min->value(), m_spin_max->value());
    m_histogram_chart->setColormap(m_current_colormap);
}

void SFDisplayPanel::onShowNaNGreyToggled(bool checked)
{
    m_show_nan_grey = checked;
    m_histogram_chart->setShowNaNGrey(checked);
    updateColorAndRefresh();
}

void SFDisplayPanel::onAlwaysShowZeroToggled(bool checked)
{
    m_always_show_zero = checked;
    emit scalarBarShowZero(checked);
}

void SFDisplayPanel::onSymmetricalToggled(bool checked)
{
    m_symmetrical = checked;
    updateColorAndRefresh();
}

}  // namespace ct
