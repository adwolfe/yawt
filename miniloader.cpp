#include "miniloader.h"
#include <QPainter>
#include <QDebug>

MiniLoader::MiniLoader(QWidget* parent)
    : QWidget(parent)
    , m_currentFrameNumber(-1)
    , m_centerPoint(0, 0)
    , m_cropOffset(0, 0)
    , m_cropSize(100, 100)
{
    setMinimumSize(100, 100);
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);
}

MiniLoader::~MiniLoader() = default;

QPointF MiniLoader::getLastCenterPoint() const
{
    return m_centerPoint;
}

void MiniLoader::updateWithCroppedFrame(int frameNumber, const QImage& croppedFrame,
                                        QPointF cropOffset, QSizeF cropSize,
                                        QPointF centerPoint)
{
    m_currentFrameNumber = frameNumber;
    m_croppedFrame = croppedFrame;
    m_cropOffset = cropOffset;
    m_cropSize = cropSize;
    m_centerPoint = centerPoint;
    
    update();
}

void MiniLoader::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event)
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    
    // Fill background
    painter.fillRect(rect(), Qt::black);
    
    if (m_croppedFrame.isNull()) {
        // Show "no crop available" message
        painter.setPen(Qt::lightGray);
        QFont font = painter.font();
        font.setPointSize(12);
        painter.setFont(font);
        painter.drawText(rect(), Qt::AlignCenter, "No crop available");
        return;
    }

    // Calculate how to scale and position the cropped image to fit the widget
    QRect widgetRect = rect();
    QSize croppedSize = m_croppedFrame.size();
    
    if (croppedSize.isEmpty() || widgetRect.isEmpty()) {
        return;
    }
    
    // Calculate scaling to fit the widget while maintaining aspect ratio
    double scaleX = static_cast<double>(widgetRect.width()) / croppedSize.width();
    double scaleY = static_cast<double>(widgetRect.height()) / croppedSize.height();
    double scale = qMin(scaleX, scaleY);
    
    // Calculate the size and position of the scaled image
    int scaledWidth = static_cast<int>(croppedSize.width() * scale);
    int scaledHeight = static_cast<int>(croppedSize.height() * scale);
    
    int x = (widgetRect.width() - scaledWidth) / 2;
    int y = (widgetRect.height() - scaledHeight) / 2;
    
    QRect targetRect(x, y, scaledWidth, scaledHeight);
    
    // Draw the cropped image
    painter.drawImage(targetRect, m_croppedFrame);
    
    // Draw a border around the image
    painter.setPen(QPen(Qt::white, 1));
    painter.drawRect(targetRect);
    
    // Draw frame info overlay
    painter.setPen(Qt::yellow);
    QFont font = painter.font();
    font.setPointSize(10);
    painter.setFont(font);
    
    QString info = QString("Frame %1\nCrop: %2x%3\nCenter: (%4, %5)")
                  .arg(m_currentFrameNumber)
                  .arg(static_cast<int>(m_cropSize.width()))
                  .arg(static_cast<int>(m_cropSize.height()))
                  .arg(static_cast<int>(m_centerPoint.x()))
                  .arg(static_cast<int>(m_centerPoint.y()));
    
    QRect textRect(5, 5, width() - 10, 80);
    painter.drawText(textRect, Qt::AlignLeft | Qt::AlignTop, info);
}

void MiniLoader::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    update();
}

QPointF MiniLoader::miniToVideo(const QPointF& miniPoint) const
{
    if (m_croppedFrame.isNull()) {
        return QPointF(-1, -1);
    }
    
    // Convert miniPoint (widget coordinates) to cropped image coordinates
    QRect widgetRect = rect();
    QSize croppedSize = m_croppedFrame.size();
    
    if (croppedSize.isEmpty() || widgetRect.isEmpty()) {
        return QPointF(-1, -1);
    }
    
    // Calculate the scaled image rect (same as in paintEvent)
    double scaleX = static_cast<double>(widgetRect.width()) / croppedSize.width();
    double scaleY = static_cast<double>(widgetRect.height()) / croppedSize.height();
    double scale = qMin(scaleX, scaleY);
    
    int scaledWidth = static_cast<int>(croppedSize.width() * scale);
    int scaledHeight = static_cast<int>(croppedSize.height() * scale);
    
    int x = (widgetRect.width() - scaledWidth) / 2;
    int y = (widgetRect.height() - scaledHeight) / 2;
    
    QRect targetRect(x, y, scaledWidth, scaledHeight);
    
    // Check if point is inside the image area
    if (!targetRect.contains(miniPoint.toPoint())) {
        return QPointF(-1, -1);
    }
    
    // Convert widget coordinates to cropped image coordinates
    double cropX = (miniPoint.x() - targetRect.x()) / scale;
    double cropY = (miniPoint.y() - targetRect.y()) / scale;
    
    // Convert cropped image coordinates to video coordinates
    double videoX = m_cropOffset.x() + cropX;
    double videoY = m_cropOffset.y() + cropY;
    
    return QPointF(videoX, videoY);
}

QPointF MiniLoader::videoToMini(const QPointF& videoPoint) const
{
    if (m_croppedFrame.isNull()) {
        return QPointF(-1, -1);
    }
    
    // Convert video coordinates to cropped image coordinates
    double cropX = videoPoint.x() - m_cropOffset.x();
    double cropY = videoPoint.y() - m_cropOffset.y();
    
    // Check if point is within the cropped area
    if (cropX < 0 || cropY < 0 || cropX >= m_croppedFrame.width() || cropY >= m_croppedFrame.height()) {
        return QPointF(-1, -1);
    }
    
    // Calculate the scaled image rect (same as in paintEvent)
    QRect widgetRect = rect();
    QSize croppedSize = m_croppedFrame.size();
    
    if (croppedSize.isEmpty() || widgetRect.isEmpty()) {
        return QPointF(-1, -1);
    }
    
    double scaleX = static_cast<double>(widgetRect.width()) / croppedSize.width();
    double scaleY = static_cast<double>(widgetRect.height()) / croppedSize.height();
    double scale = qMin(scaleX, scaleY);
    
    int scaledWidth = static_cast<int>(croppedSize.width() * scale);
    int scaledHeight = static_cast<int>(croppedSize.height() * scale);
    
    int x = (widgetRect.width() - scaledWidth) / 2;
    int y = (widgetRect.height() - scaledHeight) / 2;
    
    // Convert cropped image coordinates to widget coordinates
    double miniX = x + cropX * scale;
    double miniY = y + cropY * scale;
    
    return QPointF(miniX, miniY);
}

QRectF MiniLoader::getCurrentCropRectVideo() const
{
    if (m_croppedFrame.isNull()) {
        return QRectF();
    }
    
    return QRectF(m_cropOffset, m_cropSize);
}

QPointF MiniLoader::getCurrentCropOffset() const
{
    return m_cropOffset;
}

QSizeF MiniLoader::getCurrentCropSize() const
{
    return m_cropSize;
}