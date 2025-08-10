#include "minivideoloader.h"
#include "trackingdatastorage.h"
#include <QPainter>
#include <QDebug>
#include <QResizeEvent>
#include <QFont>
#include <algorithm>

MiniVideoLoader::MiniVideoLoader(QWidget *parent)
    : QWidget(parent)
    , m_currentFrameNumber(-1)
    , m_selectedWormId(-1)
    , m_cropMultiplier(DEFAULT_CROP_MULTIPLIER)
    , m_trackingDataStorage(nullptr)
    , m_hasValidData(false)
{
    // Set up widget appearance
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);
    
    // Set minimum size for the widget
    setMinimumSize(200, 200);
    
    qDebug() << "MiniVideoLoader: Initialized with crop multiplier" << m_cropMultiplier;
}

MiniVideoLoader::~MiniVideoLoader()
{
    // No special cleanup needed
}

void MiniVideoLoader::setTrackingDataStorage(TrackingDataStorage* storage)
{
    m_trackingDataStorage = storage;
    qDebug() << "MiniVideoLoader: TrackingDataStorage set to" << storage;
}

void MiniVideoLoader::setCropMultiplier(double multiplier)
{
    if (multiplier <= 0.0) {
        qWarning() << "MiniVideoLoader: Invalid crop multiplier" << multiplier << "- must be > 0";
        return;
    }
    
    if (qFuzzyCompare(m_cropMultiplier, multiplier)) {
        return;  // No significant change
    }
    
    m_cropMultiplier = multiplier;
    qDebug() << "MiniVideoLoader: Crop multiplier set to" << m_cropMultiplier;
    
    // Update the cropped image if we have valid data
    if (m_hasValidData) {
        updateCroppedImage();
        update();
    }
}

double MiniVideoLoader::getCropMultiplier() const
{
    return m_cropMultiplier;
}

int MiniVideoLoader::getSelectedWorm() const
{
    return m_selectedWormId;
}

bool MiniVideoLoader::hasValidCrop() const
{
    return m_hasValidData && !m_croppedFrame.isNull();
}

void MiniVideoLoader::updateFrame(int frameNumber, const QImage& frame)
{
    qDebug() << "MiniVideoLoader: updateFrame called - frame:" << frameNumber << "selectedWorm:" << m_selectedWormId;
    
    m_currentFrameNumber = frameNumber;
    m_currentFrame = frame;
    
    // Poll for worm position if we have a selected worm
    if (m_selectedWormId >= 0) {
        pollWormPosition();
        updateCroppedImage();
    }
    
    update();  // Trigger repaint
    repaint(); // Force immediate repaint
}

void MiniVideoLoader::onWormSelectionChanged(const QList<TableItems::ClickedItem>& selectedItems)
{
    if (selectedItems.isEmpty()) {
        clearSelection();
        return;
    }
    
    // Use the first selected item (assuming single selection for mini view)
    const TableItems::ClickedItem& selectedItem = selectedItems.first();
    setSelectedWorm(selectedItem.id);
}

void MiniVideoLoader::setSelectedWorm(int wormId)
{
    if (m_selectedWormId == wormId) {
        return;  // No change
    }
    
    int previousWormId = m_selectedWormId;
    m_selectedWormId = wormId;
    
    qDebug() << "MiniVideoLoader: Selected worm changed from" << previousWormId << "to" << wormId;
    
    if (m_selectedWormId < 0) {
        clearSelection();
    } else {
        // Poll for worm position immediately when selection changes
        if (pollWormPosition()) {
            updateCroppedImage();
            update();  // Trigger repaint after selection change
            repaint(); // Force immediate repaint
        }
    }
    
    emit selectedWormChanged(m_selectedWormId);
}

void MiniVideoLoader::clearSelection()
{
    m_selectedWormId = -1;
    m_hasValidData = false;
    m_wormPosition = QPointF();
    m_wormRoi = QRectF();
    m_cropRect = QRectF();
    m_croppedFrame = QImage();
    
    update();  // Trigger repaint to show "no selection" message
}

bool MiniVideoLoader::pollWormPosition()
{
    if (!m_trackingDataStorage || m_selectedWormId < 0 || m_currentFrameNumber < 0) {
        m_hasValidData = false;
        return false;
    }
    
    QPointF position;
    QRectF roi;
    
    qDebug() << "MiniVideoLoader: Trying to get worm data for frame" << m_currentFrameNumber << "worm ID" << m_selectedWormId;
    if (m_trackingDataStorage->getWormDataForFrame(m_selectedWormId, m_currentFrameNumber, position, roi)) {
        qDebug() << "MiniVideoLoader: Found current frame data - position:" << position << "ROI:" << roi;
        m_wormPosition = position;
        m_wormRoi = roi;
        m_hasValidData = true;
        return true;
    } else {
        qDebug() << "MiniVideoLoader: No current frame data, trying to get last known position before frame" << m_currentFrameNumber;
        // Try to get last known position before this frame (for lost tracking)
        if (m_trackingDataStorage->getLastKnownPositionBefore(m_selectedWormId, m_currentFrameNumber, position, roi)) {
            qDebug() << "MiniVideoLoader: Found last known position - position:" << position << "ROI:" << roi;
            m_wormPosition = position;
            m_wormRoi = roi;
            m_hasValidData = true;
            return true;
        } else {
            qDebug() << "MiniVideoLoader: No last known position found either";
            m_hasValidData = false;
            return false;
        }
    }
}

QRectF MiniVideoLoader::calculateCropRect() const
{
    if (!m_hasValidData || m_wormRoi.isEmpty()) {
        return QRectF();
    }
    
    // Calculate crop size based on worm ROI and multiplier
    double cropWidth = m_wormRoi.width() * m_cropMultiplier;
    double cropHeight = m_wormRoi.height() * m_cropMultiplier;
    
    // Ensure minimum crop size
    cropWidth = std::max(cropWidth, MIN_CROP_SIZE);
    cropHeight = std::max(cropHeight, MIN_CROP_SIZE);
    
    // Center the crop rectangle on the worm position
    QRectF cropRect(
        m_wormPosition.x() - cropWidth / 2.0,
        m_wormPosition.y() - cropHeight / 2.0,
        cropWidth,
        cropHeight
    );
    
    // Clamp to image bounds
    if (!m_currentFrame.isNull()) {
        QRectF imageBounds(0, 0, m_currentFrame.width(), m_currentFrame.height());
        cropRect = cropRect.intersected(imageBounds);
    }
    
    return cropRect;
}

void MiniVideoLoader::updateCroppedImage()
{
    if (m_currentFrame.isNull() || !m_hasValidData) {
        qDebug() << "MiniVideoLoader: updateCroppedImage - no frame or no valid data";
        m_croppedFrame = QImage();
        return;
    }
    
    m_cropRect = calculateCropRect();
    
    if (m_cropRect.isEmpty()) {
        qDebug() << "MiniVideoLoader: updateCroppedImage - empty crop rect";
        m_croppedFrame = QImage();
        return;
    }
    
    // Extract the cropped region from the full frame
    QRect cropRectInt = m_cropRect.toRect();
    
    // Ensure the crop rectangle is within image bounds
    QRect imageBounds(0, 0, m_currentFrame.width(), m_currentFrame.height());
    cropRectInt = cropRectInt.intersected(imageBounds);
    
    if (cropRectInt.isEmpty()) {
        qDebug() << "MiniVideoLoader: updateCroppedImage - crop rect outside image bounds";
        m_croppedFrame = QImage();
        return;
    }
    
    // Create the cropped image
    m_croppedFrame = m_currentFrame.copy(cropRectInt);
    
    qDebug() << "MiniVideoLoader: Updated cropped image, crop rect:" << m_cropRect 
             << "cropped size:" << m_croppedFrame.size();
}

void MiniVideoLoader::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    
    qDebug() << "MiniVideoLoader: paintEvent called - wormId:" << m_selectedWormId 
             << "hasValidData:" << m_hasValidData << "croppedFrame null:" << m_croppedFrame.isNull();
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    
    // Fill background
    painter.fillRect(rect(), Qt::black);
    
    if (m_selectedWormId < 0 || !m_hasValidData) {
        qDebug() << "MiniVideoLoader: Drawing no selection message";
        drawNoSelectionMessage(painter);
    } else if (!m_croppedFrame.isNull()) {
        qDebug() << "MiniVideoLoader: Drawing cropped view, crop size:" << m_croppedFrame.size();
        drawCroppedView(painter);
    } else {
        qDebug() << "MiniVideoLoader: Drawing 'not visible' message";
        // Show "worm not visible" message
        painter.setPen(Qt::yellow);
        painter.drawText(rect(), Qt::AlignCenter, 
                        QString("Worm %1\nNot visible in frame %2")
                        .arg(m_selectedWormId)
                        .arg(m_currentFrameNumber));
    }
}

void MiniVideoLoader::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    update();  // Trigger repaint to adjust scaling
}

void MiniVideoLoader::drawNoSelectionMessage(QPainter& painter)
{
    painter.setPen(Qt::lightGray);
    QFont font = painter.font();
    font.setPointSize(12);
    painter.setFont(font);
    
    QString message = "No worm selected\n\nSelect a worm in the\nBlob Table to view\ndetailed crop here";
    painter.drawText(rect(), Qt::AlignCenter, message);
}

void MiniVideoLoader::drawCroppedView(QPainter& painter)
{
    if (m_croppedFrame.isNull()) {
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
    double scale = std::min(scaleX, scaleY);
    
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
    
    // Draw worm info overlay
    if (m_selectedWormId >= 0) {
        painter.setPen(Qt::yellow);
        QFont font = painter.font();
        font.setPointSize(10);
        painter.setFont(font);
        
        QString info = QString("Worm %1 (Frame %2)\nCrop: %3x%4 @ %5x multiplier")
                      .arg(m_selectedWormId)
                      .arg(m_currentFrameNumber)
                      .arg(static_cast<int>(m_cropRect.width()))
                      .arg(static_cast<int>(m_cropRect.height()))
                      .arg(m_cropMultiplier, 0, 'f', 1);
        
        QRect textRect(5, 5, width() - 10, 60);
        painter.drawText(textRect, Qt::AlignLeft | Qt::AlignTop, info);
    }
}