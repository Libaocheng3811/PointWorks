#include "scalar_bar_widget.h"

#include <QString>

#include <vtkTextProperty.h>

namespace ct {

ScalarBarWidget::ScalarBarWidget(vtkRenderer* renderer, QObject* parent)
    : QObject(parent), m_renderer(renderer)
{
    m_actor = vtkSmartPointer<vtkScalarBarActor>::New();
    m_actor->SetNumberOfLabels(5);
    m_actor->SetWidth(0.08);
    m_actor->SetHeight(0.72);
    m_actor->SetPosition(0.88, 0.24);
    m_actor->SetMaximumWidthInPixels(500);
    m_actor->SetMaximumHeightInPixels(1500);

    auto title_prop = m_actor->GetTitleTextProperty();
    title_prop->SetFontSize(4);
    title_prop->SetColor(0.3, 0.3, 0.3);
    title_prop->BoldOff();

    auto label_prop = m_actor->GetLabelTextProperty();
    label_prop->SetFontSize(3);
    label_prop->SetColor(0.4, 0.4, 0.4);

    m_lut = vtkSmartPointer<vtkLookupTable>::New();
    m_lut->SetNumberOfTableValues(256);
    buildLUT(ColormapType::JET);
    m_actor->SetLookupTable(m_lut);

    // Zero marker text actor
    m_zero_text_actor = vtkSmartPointer<vtkTextActor>::New();
    m_zero_text_actor->SetTextScaleModeToNone();
    m_zero_text_actor->SetInput("0");
    m_zero_text_actor->GetTextProperty()->SetFontSize(6);
    m_zero_text_actor->GetTextProperty()->SetColor(0.0, 0.0, 0.0);
    m_zero_text_actor->GetTextProperty()->SetBold(1);
    m_zero_text_actor->SetVisibility(0);
}

ScalarBarWidget::~ScalarBarWidget() = default;

void ScalarBarWidget::buildLUT(ColormapType type)
{
    const auto& lut_data = getColormapLUT(type);
    int lut_size = static_cast<int>(lut_data.size());
    int vtk_size = m_lut->GetNumberOfTableValues();

    for (int i = 0; i < vtk_size; ++i) {
        int src_idx = static_cast<int>(static_cast<float>(i) / (vtk_size - 1) * (lut_size - 1));
        if (src_idx >= lut_size) src_idx = lut_size - 1;

        float lutVal = lut_data[src_idx];
        uint32_t packed;
        std::memcpy(&packed, &lutVal, sizeof(packed));
        double r = ((packed >> 16) & 0xFF) / 255.0;
        double g = ((packed >> 8) & 0xFF) / 255.0;
        double b = (packed & 0xFF) / 255.0;
        m_lut->SetTableValue(i, r, g, b);
    }
    m_lut->Build();
}

void ScalarBarWidget::update(double min_val, double max_val,
                             const QString& title,
                             ColormapType colormap)
{
    m_data_min = min_val;
    m_data_max = max_val;

    buildLUT(colormap);
    m_lut->SetTableRange(min_val, max_val);
    m_lut->Build();

    m_actor->SetTitle(qPrintable(title));
    m_actor->SetLookupTable(m_lut);
    m_actor->SetNumberOfLabels(5);
    m_actor->SetVisibility(m_visible ? 1 : 0);

    if (!m_renderer->HasViewProp(m_actor))
        m_renderer->AddActor2D(m_actor);

    updateZeroLine(min_val, max_val);
}

void ScalarBarWidget::updateZeroLine(double display_min, double display_max)
{
    if (!m_show_zero || !m_visible) {
        m_zero_text_actor->SetVisibility(0);
        return;
    }

    // Check if 0 is within display range
    if (display_min <= 0.0 && 0.0 <= display_max) {
        double range = display_max - display_min;
        if (range < 1e-9) {
            m_zero_text_actor->SetVisibility(0);
            return;
        }
        double t = (0.0 - display_min) / range;

        // Position: to the left of the scalar bar, at the 0 position
        double* pos = m_actor->GetPosition();
        double height = m_actor->GetHeight();
        double x = pos[0] - 0.02;
        double y = pos[1] + t * height;

        m_zero_text_actor->SetPosition(x, y);
        m_zero_text_actor->SetVisibility(1);

        if (!m_renderer->HasViewProp(m_zero_text_actor))
            m_renderer->AddActor2D(m_zero_text_actor);
    } else {
        m_zero_text_actor->SetVisibility(0);
    }
}

void ScalarBarWidget::setVisible(bool visible)
{
    m_visible = visible;
    m_actor->SetVisibility(visible ? 1 : 0);
    if (visible && m_show_zero) {
        updateZeroLine(m_data_min, m_data_max);
    } else {
        m_zero_text_actor->SetVisibility(0);
    }
}

bool ScalarBarWidget::isVisible() const
{
    return m_visible;
}

void ScalarBarWidget::setShowZeroLine(bool show)
{
    m_show_zero = show;
    if (show && m_visible) {
        updateZeroLine(m_data_min, m_data_max);
    } else {
        m_zero_text_actor->SetVisibility(0);
    }
}

void ScalarBarWidget::setBottomMargin(double y_norm)
{
    m_bottom_margin = y_norm;
    double bar_height = 1.0 - m_bottom_margin - 0.03;
    m_actor->SetPosition(0.88, m_bottom_margin + 0.02);
    m_actor->SetHeight(bar_height);
    if (m_visible) {
        updateZeroLine(m_data_min, m_data_max);
    }
}

}  // namespace ct
