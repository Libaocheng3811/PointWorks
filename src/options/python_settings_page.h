#ifndef POINTWORKS_PYTHON_SETTINGS_PAGE_H
#define POINTWORKS_PYTHON_SETTINGS_PAGE_H

#include "displaysettings_page.h"

class QRadioButton;
class QLineEdit;
class QPushButton;
class QLabel;

class PythonSettingsPage : public DisplaySettingsPage
{
    Q_OBJECT
public:
    explicit PythonSettingsPage(QWidget* parent = nullptr);

    void apply() override;
    void reset() override;

private slots:
    void onBrowse();
    void onRadioChanged();

private:
    QRadioButton* m_radioBundled;
    QRadioButton* m_radioCustom;
    QLineEdit* m_pathEdit;
    QPushButton* m_btnBrowse;
    QLabel* m_statusLabel;

    void updateStatus();
};

#endif // POINTWORKS_PYTHON_SETTINGS_PAGE_H
