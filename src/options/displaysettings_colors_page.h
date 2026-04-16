#ifndef POINTWORKS_DISPLAYSETTINGS_COLORS_PAGE_H
#define POINTWORKS_DISPLAYSETTINGS_COLORS_PAGE_H

#include "displaysettings_page.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QColorDialog>
#include <QGridLayout>

namespace ct { class CloudView; class CloudTree; }

class ColorsPage : public DisplaySettingsPage
{
    Q_OBJECT
public:
    explicit ColorsPage(ct::CloudView* cloudview, ct::CloudTree* cloudtree, QWidget* parent = nullptr);

    void apply() override;
    void reset() override;

private:
    ct::CloudView* m_cloudview;
    ct::CloudTree* m_cloudtree;

    // 背景色
    QColor m_bg_color;
    QPushButton* m_bg_preview;
    QPushButton* m_bg_custom;
    QPushButton* m_bg_reset;

    // 包围盒色
    QColor m_box_color;
    QPushButton* m_box_preview;
    QPushButton* m_box_custom;
    QPushButton* m_box_reset;

    void createColorRow(QFormLayout* layout, const QString& label,
                        QPushButton*& preview, QPushButton*& custom, QPushButton*& resetBtn);

    // 调色板
    static const QColor s_palette[5][10];

    QWidget* createPaletteWidget(const std::function<void(const QColor&)>& onColorPicked);
};

#endif //POINTWORKS_DISPLAYSETTINGS_COLORS_PAGE_H
