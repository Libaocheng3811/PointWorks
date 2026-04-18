#ifndef POINTWORKS_SCALAR_BAR_WIDGET_H
#define POINTWORKS_SCALAR_BAR_WIDGET_H

#include "core/colormap.h"
#include "core/exports.h"

#include <vtkSmartPointer.h>
#include <vtkScalarBarActor.h>
#include <vtkLookupTable.h>
#include <vtkRenderer.h>
#include <vtkTextActor.h>

#include <QObject>

namespace ct {

class CT_VIZ_EXPORT ScalarBarWidget : public QObject {
    Q_OBJECT
public:
    explicit ScalarBarWidget(vtkRenderer* renderer, QObject* parent = nullptr);
    ~ScalarBarWidget();

    void update(double min_val, double max_val,
                const QString& title = "",
                ColormapType colormap = ColormapType::JET);

    void setVisible(bool visible);
    bool isVisible() const;

    void setShowZeroLine(bool show);
    void setBottomMargin(double y_norm);

private:
    void buildLUT(ColormapType type);
    void updateZeroLine(double display_min, double display_max);

    vtkRenderer* m_renderer;
    vtkSmartPointer<vtkScalarBarActor> m_actor;
    vtkSmartPointer<vtkLookupTable> m_lut;
    vtkSmartPointer<vtkTextActor> m_zero_text_actor;
    bool m_visible = false;
    bool m_show_zero = false;

    double m_data_min = 0;
    double m_data_max = 1;
    double m_bottom_margin = 0.24;
};

}  // namespace ct

#endif  // POINTWORKS_SCALAR_BAR_WIDGET_H
