#include "miniloader.h"
#include <QPainter>
#include <QDebug>
#include "trackingdatastorage.h"

MiniLoader::MiniLoader(QWidget* parent)
    : QWidget(parent)
    , m_currentFrameNumber(-1)
    , m_centerPoint(0, 0)
    , m_cropOffset(0, 0)
    , m_cropSize(100, 100)
    , m_trackingDataStorage(nullptr)
    , m_showOverlays(true)
    , m_selectedWormId(-1)
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

void MiniLoader::setTrackingDataStorage(TrackingDataStorage* storage)
{
    m_trackingDataStorage = storage;
}

void MiniLoader::setShowOverlays(bool show)
{
    if (m_showOverlays == show) return;
    m_showOverlays = show;
    update();
}

void MiniLoader::onWormSelectionChanged(const QList<TableItems::ClickedItem>& selectedItems)
{
    if (selectedItems.isEmpty()) {
        clearSelection();
        return;
    }

    // Use the first selected item (assuming single selection for mini view)
    const TableItems::ClickedItem& selectedItem = selectedItems.first();
    setSelectedWorm(selectedItem.id);
}

void MiniLoader::setSelectedWorm(int wormId)
{
    if (m_selectedWormId == wormId) {
        return;  // No change
    }

    m_selectedWormId = wormId;

    qDebug() << "MiniLoader: Selected worm changed to" << wormId;

    if (m_selectedWormId < 0) {
        clearSelection();
    } else {
        update();  // Trigger repaint after selection change
    }

    emit selectedWormChanged(m_selectedWormId);
}

void MiniLoader::clearSelection()
{
    if (m_selectedWormId != -1) {
        m_selectedWormId = -1;
        update();
        emit selectedWormChanged(-1);
    }
}

bool MiniLoader::showOverlays() const
{
    return m_showOverlays;
}

int MiniLoader::getSelectedWorm() const
{
    return m_selectedWormId;
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

    // Draw overlays if enabled
    if (m_showOverlays) {
        drawOverlays(painter, targetRect);
    }

    // Draw frame info overlay
    painter.setPen(Qt::yellow);
    QFont font = painter.font();
    font.setPointSize(10);
    painter.setFont(font);

    QString info;
    if (m_selectedWormId >= 0) {
        info = QString("Worm %1 (Frame %2)\nCrop: %3x%4\nCenter: (%5, %6)")
                      .arg(m_selectedWormId)
                      .arg(m_currentFrameNumber)
                      .arg(static_cast<int>(m_cropSize.width()))
                      .arg(static_cast<int>(m_cropSize.height()))
                      .arg(static_cast<int>(m_centerPoint.x()))
                      .arg(static_cast<int>(m_centerPoint.y()));
    } else {
        info = QString("Frame %1\nCrop: %2x%3\nCenter: (%4, %5)")
                      .arg(m_currentFrameNumber)
                      .arg(static_cast<int>(m_cropSize.width()))
                      .arg(static_cast<int>(m_cropSize.height()))
                      .arg(static_cast<int>(m_centerPoint.x()))
                      .arg(static_cast<int>(m_centerPoint.y()));
    }

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

void MiniLoader::drawOverlays(QPainter& painter, const QRect& targetRect)
{
    qDebug() << "MiniLoader::drawOverlays called - wormId:" << m_selectedWormId << "frame:" << m_currentFrameNumber << "hasStorage:" << (m_trackingDataStorage != nullptr);
    
    if (!m_trackingDataStorage || m_selectedWormId < 0 || m_currentFrameNumber < 0) {
        qDebug() << "MiniLoader::drawOverlays early return - no data";
        return;
    }

    // Get blob data for the selected worm at current frame
    QMap<int, Tracking::DetectedBlob> blobMap = m_trackingDataStorage->getDetectedBlobsForFrame(m_currentFrameNumber);
    
    qDebug() << "MiniLoader::drawOverlays blob map contains keys:" << blobMap.keys();
    
    if (!blobMap.contains(m_selectedWormId)) {
        qDebug() << "MiniLoader::drawOverlays no blob found for worm" << m_selectedWormId;
        return;
    }

    const Tracking::DetectedBlob& blob = blobMap.value(m_selectedWormId);
    if (!blob.isValid || blob.contourPoints.empty()) {
        qDebug() << "MiniLoader::drawOverlays blob invalid or empty - isValid:" << blob.isValid << "contourSize:" << blob.contourPoints.size();
        return;
    }
    
    qDebug() << "MiniLoader::drawOverlays found valid blob with" << blob.contourPoints.size() << "contour points";
    qDebug() << "MiniLoader::drawOverlays crop info - offset:" << m_cropOffset << "size:" << m_cropSize << "targetRect:" << targetRect;

    // Convert blob contour points from video coordinates to widget coordinates
    QPolygonF poly;
    int pointsKept = 0;
    int pointsTotal = blob.contourPoints.size();
    
    for (const cv::Point& pt : blob.contourPoints) {
        // Convert video coordinates to cropped coordinates
        QPointF cropCoords = mapVideoToCropCoords(QPointF(pt.x, pt.y));
        
        // Check if point is within the cropped area
        if (cropCoords.x() < 0 || cropCoords.y() < 0 || 
            cropCoords.x() >= m_croppedFrame.width() || cropCoords.y() >= m_croppedFrame.height()) {
            continue;
        }
        
        // Convert cropped coordinates to widget coordinates
        QPointF widgetPoint(
            targetRect.left() + cropCoords.x() * static_cast<double>(targetRect.width()) / m_croppedFrame.width(),
            targetRect.top() + cropCoords.y() * static_cast<double>(targetRect.height()) / m_croppedFrame.height()
        );
        poly.append(widgetPoint);
        pointsKept++;
    }
    
    qDebug() << "MiniLoader::drawOverlays kept" << pointsKept << "of" << pointsTotal << "contour points";

    if (!poly.isEmpty()) {
        qDebug() << "MiniLoader::drawOverlays drawing polygon with" << poly.size() << "points";
        qDebug() << "MiniLoader::drawOverlays first few points:" << (poly.size() > 0 ? poly[0] : QPointF()) 
                 << (poly.size() > 1 ? poly[1] : QPointF()) << (poly.size() > 2 ? poly[2] : QPointF());
        
        // Draw contour outline
        painter.setPen(QPen(QColor(220, 60, 60), 2));
        painter.setBrush(Qt::NoBrush);
        painter.drawPolygon(poly);

        // Draw worm ID label at centroid
        QPointF videoCentroid(blob.centroid.x(), blob.centroid.y());
        QPointF cropCentroid = mapVideoToCropCoords(videoCentroid);

        if (cropCentroid.x() >= 0 && cropCentroid.y() >= 0 &&
            cropCentroid.x() < m_croppedFrame.width() && cropCentroid.y() < m_croppedFrame.height()) {

            QPointF widgetCentroid(
                targetRect.left() + cropCentroid.x() * static_cast<double>(targetRect.width()) / m_croppedFrame.width(),
                targetRect.top() + cropCentroid.y() * static_cast<double>(targetRect.height()) / m_croppedFrame.height()
            );

            QString label = QString::number(m_selectedWormId);
            QFont f = painter.font();
            f.setPointSize(12);
            f.setBold(true);
            painter.setFont(f);
            painter.setPen(Qt::yellow);
            painter.drawText(QRectF(widgetCentroid.x() - 15, widgetCentroid.y() - 15, 30, 30), Qt::AlignCenter, label);
            qDebug() << "MiniLoader::drawOverlays drew label" << label << "at" << widgetCentroid;
        }
    } else {
        qDebug() << "MiniLoader::drawOverlays polygon was empty - no overlay drawn";
    }
}

QPointF MiniLoader::mapVideoToCropCoords(const QPointF& videoCoords) const
{
    double cropX = videoCoords.x() - m_cropOffset.x();
    double cropY = videoCoords.y() - m_cropOffset.y();
    return QPointF(cropX, cropY);
}
