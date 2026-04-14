#include "help_launcher.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QDialog>
#include <QDialogButtonBox>
#include <QPixmap>

namespace ct {

void HelpLauncher::showAbout(QWidget* parent) {
    QDialog dlg(parent);
    dlg.setWindowTitle("About PointWorks");
    dlg.setFixedSize(420, 280);
    dlg.setWindowFlags(dlg.windowFlags() & ~Qt::WindowContextHelpButtonHint);

    auto* layout = new QVBoxLayout(&dlg);

    // Logo
    auto* logoLabel = new QLabel(&dlg);
    QPixmap logo(":/res/logo/PointWorks.svg");
    if (!logo.isNull()) {
        logo = logo.scaled(80, 80, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        logoLabel->setPixmap(logo);
    }
    logoLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(logoLabel);

    // Title
    auto* titleLabel = new QLabel("<h2 style='margin:4px'>PointWorks</h2>", &dlg);
    titleLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(titleLabel);

    // Version
    auto* verLabel = new QLabel("Version 1.0.0", &dlg);
    verLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(verLabel);

    // Description
    auto* descLabel = new QLabel(
        "3D Point Cloud Processing Software\n"
        "Built with Qt5, VTK, PCL & Python",
        &dlg);
    descLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(descLabel);

    layout->addSpacing(10);

    // Credits
    auto* creditLabel = new QLabel(
        "<small>This software uses open-source libraries including "
        "Qt, PCL, VTK, pybind11, LASlib, and CSF.</small>",
        &dlg);
    creditLabel->setAlignment(Qt::AlignCenter);
    creditLabel->setWordWrap(true);
    layout->addWidget(creditLabel);

    // Copyright
    auto* copyLabel = new QLabel("<small>&copy; 2024-2026 LiBaocheng. All rights reserved.</small>", &dlg);
    copyLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(copyLabel);

    // OK button
    auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok, &dlg);
    QObject::connect(buttonBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    layout->addWidget(buttonBox);

    dlg.exec();
}

} // namespace ct
