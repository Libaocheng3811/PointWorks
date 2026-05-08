#include "rangeimage.h"
#include "base/cloudtree.h"
#include "viz/console.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QScrollBar>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QShowEvent>

#include <pcl/visualization/common/float_image_utils.h>

#define RANGE_IMAGE_FLAG    "range_image"

// ======================== Constructor ========================

RangeImage::RangeImage(QWidget* parent)
    : pw::CustomDialog(parent)
{
    setupUi();

    m_range_image = std::make_shared<pw::RangeImage>();
    m_debounce_timer_ = new QTimer(this);
    m_debounce_timer_->setSingleShot(true);
    m_debounce_timer_->setInterval(200);
    connect(m_debounce_timer_, &QTimer::timeout, this, &RangeImage::onDebounceTimeout);
}

RangeImage::~RangeImage() = default;

// ======================== setupUi ========================

void RangeImage::setupUi()
{
    setWindowTitle("Range Image");
    resize(420, 520);

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(6);

    // --- Parameters ---
    auto* param_group = new QGroupBox("Parameters", this);
    auto* param_layout = new QGridLayout(param_group);
    param_layout->setContentsMargins(6, 10, 6, 6);
    param_layout->setSpacing(4);

    param_layout->addWidget(new QLabel("Angular Resolution (deg):", this), 0, 0);
    dspin_angular_res_ = new QDoubleSpinBox(this);
    dspin_angular_res_->setRange(0.1, 10.0);
    dspin_angular_res_->setDecimals(1);
    dspin_angular_res_->setSingleStep(0.1);
    dspin_angular_res_->setValue(0.2);
    param_layout->addWidget(dspin_angular_res_, 0, 1);

    param_layout->addWidget(new QLabel("H-FOV (deg):", this), 1, 0);
    dspin_h_fov_ = new QDoubleSpinBox(this);
    dspin_h_fov_->setRange(1.0, 360.0);
    dspin_h_fov_->setDecimals(1);
    dspin_h_fov_->setSingleStep(10.0);
    dspin_h_fov_->setValue(360.0);
    param_layout->addWidget(dspin_h_fov_, 1, 1);

    param_layout->addWidget(new QLabel("V-FOV (deg):", this), 1, 2);
    dspin_v_fov_ = new QDoubleSpinBox(this);
    dspin_v_fov_->setRange(1.0, 180.0);
    dspin_v_fov_->setDecimals(1);
    dspin_v_fov_->setSingleStep(10.0);
    dspin_v_fov_->setValue(180.0);
    param_layout->addWidget(dspin_v_fov_, 1, 3);

    main_layout->addWidget(param_group);

    // --- Image Display ---
    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidgetResizable(false);
    scroll_area_->setAlignment(Qt::AlignCenter);
    scroll_area_->setStyleSheet("QScrollArea { background-color: #2b2b2b; border: 1px solid #555; }");

    image_label_ = new QLabel(this);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setText("No image");
    image_label_->setStyleSheet("QLabel { color: #888; }");
    scroll_area_->setWidget(image_label_);

    main_layout->addWidget(scroll_area_, 1);

    // --- Options ---
    check_show_cloud_ = new QCheckBox("Show Point Cloud from Range Image", this);
    main_layout->addWidget(check_show_cloud_);

    // --- Buttons ---
    auto* btn_layout = new QHBoxLayout();
    btn_export_ = new QPushButton("Export", this);
    btn_close_ = new QPushButton("Close", this);
    btn_layout->addStretch();
    btn_layout->addWidget(btn_export_);
    btn_layout->addStretch();
    btn_layout->addWidget(btn_close_);
    btn_layout->addStretch();
    main_layout->addLayout(btn_layout);

    // --- Connections ---
    connect(dspin_angular_res_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RangeImage::onParamChanged);
    connect(dspin_h_fov_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RangeImage::onParamChanged);
    connect(dspin_v_fov_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RangeImage::onParamChanged);
    connect(check_show_cloud_, &QCheckBox::toggled, this, &RangeImage::onTogglePointCloud);
    connect(btn_export_, &QPushButton::clicked, this, &RangeImage::onExportImage);
    connect(btn_close_, &QPushButton::clicked, this, &RangeImage::onClose);
}

// ======================== init ========================

void RangeImage::init()
{
    connect(m_cloudview, &pw::CloudView::viewerPose, this, &RangeImage::onViewerPoseChanged);

    // Get current viewer pose and trigger initial generation
    auto selected_clouds = m_cloudtree->getSelectedClouds();
    if (!selected_clouds.empty()) {
        Eigen::Affine3f pose = m_cloudview->getViewerPose();
        m_viewer_pose = pose;
        updateDepthImage();
    }
}

// ======================== reset ========================

void RangeImage::reset()
{
    disconnect(m_cloudview, &pw::CloudView::viewerPose, this, &RangeImage::onViewerPoseChanged);
    m_debounce_timer_->stop();

    m_cloudview->removePointCloud(RANGE_IMAGE_FLAG);
    m_cloud.reset();
    m_pixmap = QPixmap();
    image_label_->clear();
    image_label_->setText("No image");
    m_scale_factor_ = 1.0;
}

// ======================== Slots ========================

void RangeImage::onViewerPoseChanged(Eigen::Affine3f pose)
{
    m_viewer_pose = pose;
    m_debounce_timer_->start();
}

void RangeImage::onDebounceTimeout()
{
    updateDepthImage();
}

void RangeImage::onParamChanged()
{
    // Parameters changed, regenerate with current pose
    if (!m_cloud || m_cloud->empty()) return;
    updateDepthImage();
}

void RangeImage::onExportImage()
{
    if (m_pixmap.isNull()) {
        printW("No range image to export.");
        return;
    }

    QString default_name = m_cloud
        ? QString::fromStdString(m_cloud->id()) + "_range.png"
        : "range.png";

    QString path = QFileDialog::getSaveFileName(
        this, "Export Range Image", default_name,
        "PNG (*.png);;JPEG (*.jpg);;BMP (*.bmp)");

    if (!path.isEmpty()) {
        m_pixmap.save(path);
        printI(QString("Range image exported to: %1").arg(path));
    }
}

void RangeImage::onClose()
{
    reset();
    this->close();
}

void RangeImage::onTogglePointCloud(bool checked)
{
    if (checked && m_range_image) {
        m_cloudview->addPointCloudFromRangeImage(m_range_image, RANGE_IMAGE_FLAG, pw::Color::Green);
        m_cloudview->setPointCloudSize(RANGE_IMAGE_FLAG, 4);
    } else {
        m_cloudview->removePointCloud(RANGE_IMAGE_FLAG);
    }
}

// ======================== Depth Image Generation ========================

void RangeImage::updateDepthImage()
{
    auto selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) {
        m_pixmap = QPixmap();
        image_label_->clear();
        image_label_->setText("Please select a cloud");
        return;
    }

    m_cloud = selected_clouds.front();

    float angular_res = static_cast<float>(dspin_angular_res_->value());
    float h_fov = static_cast<float>(dspin_h_fov_->value());
    float v_fov = static_cast<float>(dspin_v_fov_->value());

    m_range_image = std::make_shared<pw::RangeImage>();
    m_range_image->createFromPointCloud(
        *m_cloud->toPCL_XYZRGBN(),
        pcl::deg2rad(angular_res),
        pcl::deg2rad(h_fov),
        pcl::deg2rad(v_fov),
        m_viewer_pose);

    float* ranges_array = m_range_image->getRangesArray();
    unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(
        ranges_array, m_range_image->width, m_range_image->height);

    // QImage needs deep copy since rgb_image may be freed by PCL
    int w = m_range_image->width;
    int h = m_range_image->height;
    QImage qimg(rgb_image, w, h, 3 * w, QImage::Format_RGB888);
    m_pixmap = QPixmap::fromImage(qimg.copy()); // deep copy
    fitToView();
    if (check_show_cloud_->isChecked()) {
        m_cloudview->removePointCloud(RANGE_IMAGE_FLAG);
        m_cloudview->addPointCloudFromRangeImage(m_range_image, RANGE_IMAGE_FLAG, pw::Color::Green);
        m_cloudview->setPointCloudSize(RANGE_IMAGE_FLAG, 4);
    }
}

// ======================== Zoom & Pan ========================

void RangeImage::fitToView()
{
    if (m_pixmap.isNull()) return;

    QSize available = scroll_area_->viewport()->size();
    if (!available.isValid() || available.width() <= 0 || available.height() <= 0) return;

    qreal sx = static_cast<qreal>(available.width()) / m_pixmap.width();
    qreal sy = static_cast<qreal>(available.height()) / m_pixmap.height();
    m_scale_factor_ = qMin(sx, sy);
    applyScale();
}

void RangeImage::applyScale()
{
    if (m_pixmap.isNull()) return;

    QSize scaled_size = m_pixmap.size() * m_scale_factor_;
    Qt::TransformationMode mode = (m_scale_factor_ > 1.0)
        ? Qt::FastTransformation   // 放大时保持像素锐利
        : Qt::SmoothTransformation; // 缩小时平滑抗锯齿
    image_label_->setPixmap(m_pixmap.scaled(
        scaled_size, Qt::KeepAspectRatio, mode));
    image_label_->setFixedSize(scaled_size);
}

void RangeImage::showEvent(QShowEvent* event)
{
    pw::CustomDialog::showEvent(event);
    fitToView();
}

void RangeImage::wheelEvent(QWheelEvent* event)
{
    if (m_pixmap.isNull()) return;

    qreal factor = (event->angleDelta().y() > 0) ? 1.15 : 1.0 / 1.15;
    m_scale_factor_ *= factor;
    m_scale_factor_ = qBound(0.1, m_scale_factor_, 20.0);

    applyScale();
    event->accept();
}

void RangeImage::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && !m_pixmap.isNull()) {
        m_dragging_ = true;
        m_last_mouse_pos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
    event->accept();
}

void RangeImage::mouseMoveEvent(QMouseEvent* event)
{
    if (m_dragging_) {
        QPoint delta = event->pos() - m_last_mouse_pos_;
        scroll_area_->horizontalScrollBar()->setValue(
            scroll_area_->horizontalScrollBar()->value() - delta.x());
        scroll_area_->verticalScrollBar()->setValue(
            scroll_area_->verticalScrollBar()->value() - delta.y());
        m_last_mouse_pos_ = event->pos();
    }
    event->accept();
}

void RangeImage::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        m_dragging_ = false;
        setCursor(Qt::ArrowCursor);
    }
    event->accept();
}
