#include "python_settings_page.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QRadioButton>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QSettings>
#include <QDir>

PythonSettingsPage::PythonSettingsPage(QWidget* parent)
    : DisplaySettingsPage(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(16, 12, 16, 12);
    layout->setSpacing(10);

    // --- Python source selection ---
    m_radioBundled = new QRadioButton(tr("Use bundled Python (default)"), this);
    m_radioCustom = new QRadioButton(tr("Use custom Python environment"), this);

    layout->addWidget(m_radioBundled);
    layout->addWidget(m_radioCustom);
    layout->addSpacing(8);

    // --- Path input ---
    auto* pathLayout = new QHBoxLayout();
    m_pathEdit = new QLineEdit(this);
    m_pathEdit->setPlaceholderText(tr("e.g. C:/Python39"));
    m_btnBrowse = new QPushButton(tr("Browse..."), this);
    m_btnBrowse->setFixedWidth(80);
    pathLayout->addWidget(m_pathEdit, 1);
    pathLayout->addWidget(m_btnBrowse);
    layout->addLayout(pathLayout);

    // --- Status label ---
    m_statusLabel = new QLabel(this);
    m_statusLabel->setWordWrap(true);
    m_statusLabel->setStyleSheet("color: #666; font-size: 11px;");
    layout->addWidget(m_statusLabel);
    layout->addStretch();

    // --- Load saved settings ---
    QSettings settings("PointWorks", "PointWorks");
    QString savedPath = settings.value("python_home").toString();
    if (!savedPath.isEmpty()) {
        m_radioCustom->setChecked(true);
        m_pathEdit->setText(savedPath);
    } else {
        m_radioBundled->setChecked(true);
    }

    // --- Connections ---
    connect(m_radioBundled, &QRadioButton::toggled, this, &PythonSettingsPage::onRadioChanged);
    connect(m_btnBrowse, &QPushButton::clicked, this, &PythonSettingsPage::onBrowse);

    onRadioChanged();
}

void PythonSettingsPage::apply()
{
    QSettings settings("PointWorks", "PointWorks");
    if (m_radioCustom->isChecked() && !m_pathEdit->text().trimmed().isEmpty()) {
        settings.setValue("python_home", m_pathEdit->text().trimmed());
    } else {
        settings.remove("python_home");
    }
}

void PythonSettingsPage::reset()
{
    m_radioBundled->setChecked(true);
    m_pathEdit->clear();
    onRadioChanged();
}

void PythonSettingsPage::onBrowse()
{
    QString dir = QFileDialog::getExistingDirectory(
        this, tr("Select Python Installation Directory"), m_pathEdit->text());
    if (!dir.isEmpty()) {
        m_pathEdit->setText(dir);
        updateStatus();
    }
}

void PythonSettingsPage::onRadioChanged()
{
    bool custom = m_radioCustom->isChecked();
    m_pathEdit->setEnabled(custom);
    m_btnBrowse->setEnabled(custom);
    updateStatus();
}

void PythonSettingsPage::updateStatus()
{
    if (m_radioCustom->isChecked()) {
        QString path = m_pathEdit->text().trimmed();
        if (path.isEmpty()) {
            m_statusLabel->setText(tr("Please specify a Python 3.9 installation path."));
            m_statusLabel->setStyleSheet("color: #faad14; font-size: 11px;");
        } else if (!QDir(path).exists()) {
            m_statusLabel->setText(tr("Directory does not exist: %1").arg(path));
            m_statusLabel->setStyleSheet("color: #ff4d4f; font-size: 11px;");
        } else {
            m_statusLabel->setText(tr("Changes will take effect after restarting the application."));
            m_statusLabel->setStyleSheet("color: #1890ff; font-size: 11px;");
        }
    } else {
        m_statusLabel->setText(tr("Using the Python environment bundled with the application."));
        m_statusLabel->setStyleSheet("color: #52c41a; font-size: 11px;");
    }
}
