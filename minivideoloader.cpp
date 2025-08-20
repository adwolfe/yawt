#include "minivideoloader.h"
#include "trackingdatastorage.h"
#include "trackingcommon.h"
#include "videoloader.h"
#include <QPainter>
#include <QDebug>
#include <QResizeEvent>
#include <QFont>
#include <QMouseEvent>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>

// Helper struct for presence info (local to this file)
struct WormPresenceInfo {
    QList<int> framesPresent;
    QMap<int, QPointF> posByFrame;
    QMap<int, QRectF> roiByFrame;
};

MiniVideoLoader::MiniVideoLoader(QWidget *parent)
    : QWidget(parent)
    , m_currentFrameNumber(-1)
    , m_selectedWormId(-1)
    , m_cropMultiplier(DEFAULT_CROP_MULTIPLIER)
    , m_trackingDataStorage(nullptr)
    , m_videoLoader(nullptr)
    , m_hasValidData(false)
    , m_hasBlobSelection(false)
    , m_showOtherWormOverlays(false)
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
}

void MiniVideoLoader::setVideoLoader(VideoLoader* videoLoader)
{
    m_videoLoader = videoLoader;
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
    m_currentFrameNumber = frameNumber;
    m_currentFrame = frame;
    
    // Convert QImage to cv::Mat for thresholding
    if (!frame.isNull()) {
        qDebug() << "MiniVideoLoader: Original QImage format:" << frame.format() << "size:" << frame.size();
        QImage rgbFrame = frame.convertToFormat(QImage::Format_RGB888);
        qDebug() << "MiniVideoLoader: Converted QImage format:" << rgbFrame.format();
        
        m_currentFrameCv = cv::Mat(rgbFrame.height(), rgbFrame.width(), CV_8UC3, 
                                   (void*)rgbFrame.constBits(), rgbFrame.bytesPerLine()).clone();
        qDebug() << "MiniVideoLoader: Initial cv::Mat - channels:" << m_currentFrameCv.channels() 
                 << "type:" << m_currentFrameCv.type() << "size:" << m_currentFrameCv.cols << "x" << m_currentFrameCv.rows;
        
        // Convert from RGB to BGR for OpenCV
        cv::cvtColor(m_currentFrameCv, m_currentFrameCv, cv::COLOR_RGB2BGR);
        qDebug() << "MiniVideoLoader: After RGB->BGR conversion - channels:" << m_currentFrameCv.channels() 
                 << "type:" << m_currentFrameCv.type();
    } else {
        m_currentFrameCv = cv::Mat();
    }
    
    if (m_selectedWormId >= 0) {
        pollWormPosition();
        updateCroppedImage();
    }
    
    update();  // Trigger repaint
    repaint(); // Force immediate repaint
}

void MiniVideoLoader::updateFrame(int frameNumber, const cv::Mat& frame)
{
    m_currentFrameNumber = frameNumber;
    m_currentFrameCv = frame.clone();
    
    // Clear the QImage version since we now have cv::Mat
    m_currentFrame = QImage();
    
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

void MiniVideoLoader::setShowOtherWormOverlays(bool show)
{
    if (m_showOtherWormOverlays == show) return;
    m_showOtherWormOverlays = show;
    update();
}

bool MiniVideoLoader::showOtherWormOverlays() const
{
    return m_showOtherWormOverlays;
}

// Find worms visible in the current crop across frames [current-radius .. current+radius]
QMap<int, WormPresenceInfo> findWormsInCrop(TrackingDataStorage* storage, const QRectF& cropRect, int centerFrame, int radius)
{
    QMap<int, WormPresenceInfo> result;
    if (!storage) return result;

    QSet<int> candidates = storage->getItemsWithTracks();
    if (candidates.isEmpty()) candidates = storage->getAllItemIds();

    for (int f = centerFrame - radius; f <= centerFrame + radius; ++f) {
        if (f < 0) continue;
        for (int wid : candidates) {
            QPointF pos; QRectF roi;
            if (!storage->getWormDataForFrame(wid, f, pos, roi)) continue;
            // membership: centroid inside crop OR ROI intersects crop
            bool inside = cropRect.contains(pos) || cropRect.intersects(roi);
            if (!inside) continue;
            WormPresenceInfo &info = result[wid];
            info.framesPresent.append(f);
            info.posByFrame.insert(f, pos);
            info.roiByFrame.insert(f, roi);
        }
    }
    return result;
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
    } else {
        qDebug() << "MiniVideoLoader: No current frame data, trying to get last known position before frame" << m_currentFrameNumber;
        // Try to get last known position before this frame (for lost tracking)
        if (m_trackingDataStorage->getLastKnownPositionBefore(m_selectedWormId, m_currentFrameNumber, position, roi)) {
            qDebug() << "MiniVideoLoader: Found last known position - position:" << position << "ROI:" << roi;
            m_wormPosition = position;
            m_wormRoi = roi;
            m_hasValidData = true;
        } else {
            qDebug() << "MiniVideoLoader: No last known position found either";
            m_hasValidData = false;
            return false;
        }
    }

    // If tracking storage also has a stored detected blob for this worm at this frame,
    // expand the stored ROI to include the blob's full contour bounding box. This ensures
    // the crop computed from m_wormRoi will include any contour pixels that extend
    // beyond the nominal ROI and avoids doing another crop calculation later.
    if (m_trackingDataStorage && m_selectedWormId >= 0 && m_currentFrameNumber >= 0) {
        QMap<int, Tracking::DetectedBlob> blobMap = m_trackingDataStorage->getDetectedBlobsForFrame(m_currentFrameNumber);
        if (!blobMap.isEmpty() && blobMap.contains(m_selectedWormId)) {
            const Tracking::DetectedBlob &db = blobMap.value(m_selectedWormId);
            if (db.isValid && !db.contourPoints.empty()) {
                int minx = std::numeric_limits<int>::max();
                int miny = std::numeric_limits<int>::max();
                int maxx = std::numeric_limits<int>::min();
                int maxy = std::numeric_limits<int>::min();
                for (const cv::Point &p : db.contourPoints) {
                    minx = std::min(minx, p.x);
                    miny = std::min(miny, p.y);
                    maxx = std::max(maxx, p.x);
                    maxy = std::max(maxy, p.y);
                }
                if (minx <= maxx && miny <= maxy) {
                    // Add a small margin so we don't clip contour edges
                    const int MARGIN = 4;
                    QRectF contourBbox((double)minx - MARGIN, (double)miny - MARGIN,
                                       (double)(maxx - minx + 1) + 2.0 * MARGIN,
                                       (double)(maxy - miny + 1) + 2.0 * MARGIN);
                    qDebug() << "MiniVideoLoader: Expanding worm ROI to include stored contour bbox:" << contourBbox;
                    m_wormRoi = m_wormRoi.united(contourBbox);
                }
            }
        }
    }

    // Clear any previous blob selection when worm position changes
    if (m_hasBlobSelection) {
        m_hasBlobSelection = false;
        m_selectedBlob = Tracking::DetectedBlob();
        m_selectedBlobContour.clear();
    }
    
    return true;
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
    
    // Determine image bounds from available frame representation (prefer QImage, otherwise cv::Mat)
    double imgW = 0.0, imgH = 0.0;
    if (!m_currentFrame.isNull()) {
        imgW = m_currentFrame.width();
        imgH = m_currentFrame.height();
    } else if (!m_currentFrameCv.empty()) {
        imgW = m_currentFrameCv.cols;
        imgH = m_currentFrameCv.rows;
    }

    if (imgW > 0 && imgH > 0) {
        QRectF imageBounds(0.0, 0.0, imgW, imgH);
        QRectF intersected = cropRect.intersected(imageBounds);

        // If intersection produced an extremely small rect (due to being at the edge),
        // attempt to expand to at least MIN_CROP_SIZE while keeping inside image bounds.
        double minW = MIN_CROP_SIZE;
        double minH = MIN_CROP_SIZE;

        if (intersected.width() < minW || intersected.height() < minH) {
            // Center desired crop on worm position, but clamp to image bounds
            double desiredW = qMax(minW, cropRect.width());
            double desiredH = qMax(minH, cropRect.height());

            double left = m_wormPosition.x() - desiredW / 2.0;
            double top = m_wormPosition.y() - desiredH / 2.0;

            // Clamp left/top so crop stays within image
            left = qBound(0.0, left, imgW - desiredW);
            top = qBound(0.0, top, imgH - desiredH);

            intersected = QRectF(left, top, desiredW, desiredH).intersected(imageBounds);
            qDebug() << "MiniVideoLoader: calculateCropRect adjusted small edge crop to" << intersected;
        }

        // Final clamp to image bounds and return
        cropRect = intersected;
    }

    return cropRect;
}

void MiniVideoLoader::updateCroppedImage()
{
    bool hasQImage = !m_currentFrame.isNull();
    bool hasCvMat = !m_currentFrameCv.empty();
    
    if ((!hasQImage && !hasCvMat) || !m_hasValidData) {
        qDebug() << "MiniVideoLoader: updateCroppedImage - no frame or no valid data";
        m_croppedFrame = QImage();
        m_croppedFrameCv = cv::Mat();
        m_thresholdedCropFrame = cv::Mat();
        return;
    }
    
    m_cropRect = calculateCropRect();
    
    if (m_cropRect.isEmpty()) {
        qDebug() << "MiniVideoLoader: updateCroppedImage - empty crop rect";
        m_croppedFrame = QImage();
        m_croppedFrameCv = cv::Mat();
        m_thresholdedCropFrame = cv::Mat();
        return;
    }
    
    // Extract the cropped region from the full frame
    QRect cropRectInt = m_cropRect.toRect();
    
    // Handle cv::Mat cropping if available
    if (hasCvMat) {
    // Ensure the crop rectangle is within cv::Mat bounds
    cv::Rect cvImageBounds(0, 0, m_currentFrameCv.cols, m_currentFrameCv.rows);
    cv::Rect cvCropRect(cropRectInt.x(), cropRectInt.y(), cropRectInt.width(), cropRectInt.height());
        
    // Log the computed crop rect before intersect (print cv::Rect fields explicitly)
    qDebug() << "MiniVideoLoader: Computed cropRect (int)" << cropRectInt
         << "cv image bounds: x=" << cvImageBounds.x << "y=" << cvImageBounds.y
         << "w=" << cvImageBounds.width << "h=" << cvImageBounds.height;

    // Intersect with image bounds
    cvCropRect &= cvImageBounds;  // OpenCV intersection operator
    qDebug() << "MiniVideoLoader: cvCropRect after intersection: x=" << cvCropRect.x << "y=" << cvCropRect.y
         << "w=" << cvCropRect.width << "h=" << cvCropRect.height << "area:" << cvCropRect.area();
        
        if (cvCropRect.area() > 0) {
            m_croppedFrameCv = m_currentFrameCv(cvCropRect).clone();
            
            // Apply thresholding to the cropped cv::Mat
            qDebug() << "MiniVideoLoader: About to apply thresholding to cropped frame, size:" << m_croppedFrameCv.cols << "x" << m_croppedFrameCv.rows;
            // First try to get tracker-derived contours from TrackingDataStorage for nearby frames
            m_detectedBlobs.clear();
            m_detectedBlobContours.clear();
            bool loadedFromStorage = false;
            if (m_trackingDataStorage && m_selectedWormId >= 0) {
                // Only use tracker-provided contours for the current frame
                int f = m_currentFrameNumber;
                if (f >= 0) {
                    QMap<int, Tracking::DetectedBlob> blobMap = m_trackingDataStorage->getDetectedBlobsForFrame(f);
                    if (!blobMap.isEmpty()) {
                        QList<int> keys = blobMap.keys();
                        qDebug() << "MiniVideoLoader: Tracker-provided blobs for frame" << f << "keys:" << keys;
                        int unsignedId = m_selectedWormId;
                        if (blobMap.contains(unsignedId)) {
                            const Tracking::DetectedBlob& db = blobMap.value(unsignedId);
                            qDebug() << "MiniVideoLoader: Found stored DetectedBlob for worm" << unsignedId
                                     << "isValid:" << db.isValid << "contourPoints.size():" << db.contourPoints.size();
                            if (db.isValid && !db.contourPoints.empty()) {
                                // Convert stored contour points (full-frame coords) to cropped coords
                                std::vector<cv::Point> contour;
                                int kept = 0;
                                for (const cv::Point& pt : db.contourPoints) {
                                    double cx = pt.x - m_cropRect.left();
                                    double cy = pt.y - m_cropRect.top();
                                    // Ensure point inside cropped image
                                    if (cx >= 0 && cy >= 0 && cx < m_croppedFrameCv.cols && cy < m_croppedFrameCv.rows) {
                                        contour.emplace_back(static_cast<int>(std::round(cx)), static_cast<int>(std::round(cy)));
                                        ++kept;
                                    }
                                }
                                qDebug() << "MiniVideoLoader: Converted contour points kept:" << kept << "of" << db.contourPoints.size();
                                if (!contour.empty()) {
                                    m_detectedBlobContours.push_back(contour);
                                    m_detectedBlobs.append(db);
                                    loadedFromStorage = true;
                                } else {
                                    qDebug() << "MiniVideoLoader: Stored contour exists but no points fall inside crop - skipping storage overlay and falling back to thresholding";
                                }
                            }
                        } else {
                            qDebug() << "MiniVideoLoader: No stored blob for selected worm id" << unsignedId << "in blobMap";
                        }
                    } else {
                        qDebug() << "MiniVideoLoader: No tracker-provided blobs stored for frame" << f;
                    }
                }
            }

            if (!loadedFromStorage) {
                applyThresholdingToCrop();
                qDebug() << "MiniVideoLoader: Thresholding completed, result size:" << m_thresholdedCropFrame.cols << "x" << m_thresholdedCropFrame.rows;

                // After thresholding, m_detectedBlobContours will be populated by applyThresholdingToCrop
            } else {
                // Convert cropped cv::Mat to QImage for display
                convertCroppedCvMatToQImage();
                qDebug() << "MiniVideoLoader: Loaded" << m_detectedBlobContours.size() << "contours from storage and updated cropped view";
            }
        } else {
            qDebug() << "MiniVideoLoader: cv::Mat crop rect outside image bounds";
            m_croppedFrameCv = cv::Mat();
            m_thresholdedCropFrame = cv::Mat();
            m_croppedFrame = QImage();
        }
    } else if (hasQImage) {
        // Fallback to QImage cropping
        QRect imageBounds(0, 0, m_currentFrame.width(), m_currentFrame.height());
        cropRectInt = cropRectInt.intersected(imageBounds);
        
        if (cropRectInt.isEmpty()) {
            qDebug() << "MiniVideoLoader: QImage crop rect outside image bounds";
            m_croppedFrame = QImage();
            return;
        }
        
        // Create the cropped image
        m_croppedFrame = m_currentFrame.copy(cropRectInt);
        
        qDebug() << "MiniVideoLoader: Updated cropped QImage, crop rect:" << m_cropRect 
                 << "cropped size:" << m_croppedFrame.size();
    }
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
    updateCroppedImage();
}

void MiniVideoLoader::mousePressEvent(QMouseEvent *event)
{
    // Clicking to select blobs is disabled for the overlay instance.
    // Ignore mouse presses so the overlay always shows detected blobs without requiring a click.
    Q_UNUSED(event)
    QWidget::mousePressEvent(event);
}

bool MiniVideoLoader::detectBlobAtCropPosition(const QPointF& cropCoords)
{
    qDebug() << "MiniVideoLoader: detectBlobAtCropPosition called with:" << cropCoords;
    qDebug() << "MiniVideoLoader: thresholded frame empty?" << m_thresholdedCropFrame.empty() 
             << "size:" << m_thresholdedCropFrame.cols << "x" << m_thresholdedCropFrame.rows;
    
    // Use real blob detection if we have thresholded data
    if (!m_thresholdedCropFrame.empty()) {
        // Clamp coordinates to be within the cropped frame bounds
        if (cropCoords.x() < 0 || cropCoords.y() < 0 ||
            cropCoords.x() >= m_thresholdedCropFrame.cols ||
            cropCoords.y() >= m_thresholdedCropFrame.rows) {
            qDebug() << "MiniVideoLoader: Click coordinates outside thresholded frame bounds";
            return false;
        }
        
        // Find all contours in the thresholded image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(m_thresholdedCropFrame.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (contours.empty()) {
            qDebug() << "MiniVideoLoader: No contours found in thresholded frame";
            return false;
        }
        
        // Find the contour that contains the click point
        cv::Point clickPoint(qRound(cropCoords.x()), qRound(cropCoords.y()));
        int bestContourIdx = -1;
        double minArea = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < 5.0 || area > 10000.0) continue; // Area filter
            
            // Check if the click point is inside this contour
            double result = cv::pointPolygonTest(contours[i], clickPoint, false);
            if (result >= 0) { // Point is inside or on the boundary
                // If multiple contours contain the point, choose the smallest one
                if (area < minArea) {
                    minArea = area;
                    bestContourIdx = i;
                }
            }
        }
        
        if (bestContourIdx >= 0) {
            // Found a valid contour
            const std::vector<cv::Point>& selectedContour = contours[bestContourIdx];
            cv::Moments mu = cv::moments(selectedContour);
            QPointF cropCentroid(mu.m10 / mu.m00, mu.m01 / mu.m00);
            
            // Store the contour and blob info
            m_selectedBlobContour = selectedContour;
            m_selectedBlob.isValid = true;
            m_selectedBlob.centroid = mapCropToVideoCoords(cropCentroid);
            m_selectedBlob.area = minArea;
            
            // Calculate bounding box from contour
            cv::Rect cropBoundingRect = cv::boundingRect(selectedContour);
            QPointF topLeft = mapCropToVideoCoords(QPointF(cropBoundingRect.x, cropBoundingRect.y));
            QPointF bottomRight = mapCropToVideoCoords(QPointF(cropBoundingRect.x + cropBoundingRect.width, 
                                                               cropBoundingRect.y + cropBoundingRect.height));
            m_selectedBlob.boundingBox = QRectF(topLeft, bottomRight);
            
            m_hasBlobSelection = true;
            
            qDebug() << "MiniVideoLoader: Found blob contour with" << selectedContour.size() << "points, area:" << minArea;
            return true;
        } else {
            qDebug() << "MiniVideoLoader: No contour contains the click point";
        }
    } else {
        qDebug() << "MiniVideoLoader: No thresholded frame available";
    }
    
    return false;
}


QPointF MiniVideoLoader::mapCropToVideoCoords(const QPointF& cropCoords) const
{
    if (m_cropRect.isEmpty()) {
        return QPointF(-1, -1);
    }
    
    // Scale from cropped image coordinates to crop rectangle coordinates
    QSizeF croppedSize = m_croppedFrame.size();
    if (croppedSize.isEmpty()) {
        return QPointF(-1, -1);
    }
    
    double normalizedX = cropCoords.x() / croppedSize.width();
    double normalizedY = cropCoords.y() / croppedSize.height();
    
    // Map to full video coordinates
    QPointF videoCoords(
        m_cropRect.left() + normalizedX * m_cropRect.width(),
        m_cropRect.top() + normalizedY * m_cropRect.height()
    );
    
    return videoCoords;
}

QPointF MiniVideoLoader::mapVideoToCropCoords(const QPointF& videoCoords) const
{
    if (m_cropRect.isEmpty()) {
        return QPointF(-1, -1);
    }
    
    // Map from video coordinates to crop rectangle coordinates
    double normalizedX = (videoCoords.x() - m_cropRect.left()) / m_cropRect.width();
    double normalizedY = (videoCoords.y() - m_cropRect.top()) / m_cropRect.height();
    
    // Scale to cropped image coordinates
    QSizeF croppedSize = m_croppedFrame.size();
    if (croppedSize.isEmpty()) {
        return QPointF(-1, -1);
    }
    
    QPointF cropCoords(
        normalizedX * croppedSize.width(),
        normalizedY * croppedSize.height()
    );
    
    return cropCoords;
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
    qDebug() << "MiniVideoLoader: Drawing cropped view, crop size:" << m_croppedFrame.size();
    
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
    
    // Previously we drew a user-selected blob outline here. For the overlay instance
    // we now draw outlines for all detected blobs (below) and don't rely on click-selection.
    
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

    // If overlay mode is enabled, draw outlines and labels for detected blobs in the current cropped frame
    if (m_showOtherWormOverlays && !m_detectedBlobContours.empty()) {
        const QString letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        int n = (int)m_detectedBlobContours.size();
        for (int i = 0; i < n; ++i) {
            const auto &contour = m_detectedBlobContours[i];
            if (contour.empty()) continue;

            QPolygonF poly;
            for (const cv::Point &p : contour) {
                QPointF widgetPoint(
                    targetRect.left() + p.x * (double)targetRect.width() / m_croppedFrame.width(),
                    targetRect.top() + p.y * (double)targetRect.height() / m_croppedFrame.height()
                );
                poly.append(widgetPoint);
            }

            painter.setPen(QPen(QColor(220, 60, 60), 2));
            painter.setBrush(Qt::NoBrush);
            painter.drawPolygon(poly);

            // centroid label
            cv::Moments mu = cv::moments(contour);
            if (mu.m00 == 0) continue;
            QPointF cropCentroid(mu.m10 / mu.m00, mu.m01 / mu.m00);
            QPointF widgetCentroid(
                targetRect.left() + cropCentroid.x() * (double)targetRect.width() / m_croppedFrame.width(),
                targetRect.top() + cropCentroid.y() * (double)targetRect.height() / m_croppedFrame.height()
            );

            QString label = QString(letters.mid(i % letters.size(), 1));
            if (i >= (int)letters.size()) label += QString::number(i / letters.size());
            QFont f = painter.font(); f.setPointSize(12); f.setBold(true); painter.setFont(f);
            painter.setPen(Qt::yellow);
            painter.drawText(QRectF(widgetCentroid.x() - 10, widgetCentroid.y() - 10, 20, 20), Qt::AlignCenter, label);
        }
    }
}

void MiniVideoLoader::applyThresholdingToCrop()
{
    qDebug() << "MiniVideoLoader: applyThresholdingToCrop called";
    
    if (m_croppedFrameCv.empty()) {
        qDebug() << "MiniVideoLoader: Cropped cv::Mat is empty, cannot threshold";
        m_thresholdedCropFrame = cv::Mat();
        return;
    }
    
    qDebug() << "MiniVideoLoader: Input to thresholding - channels:" << m_croppedFrameCv.channels() 
             << "type:" << m_croppedFrameCv.type() << "size:" << m_croppedFrameCv.cols << "x" << m_croppedFrameCv.rows;
    
    if (!m_videoLoader) {
        qDebug() << "MiniVideoLoader: No VideoLoader reference for threshold settings";
        return;
    }
    
    Thresholding::ThresholdSettings settings = getThresholdSettings();
    qDebug() << "MiniVideoLoader: Applying thresholding with algorithm:" << static_cast<int>(settings.algorithm) 
             << "threshold:" << settings.globalThresholdValue;
    
    ThresholdingUtils::applyThresholding(m_croppedFrameCv, m_thresholdedCropFrame, settings);
    
    if (m_thresholdedCropFrame.empty()) {
        qDebug() << "MiniVideoLoader: Thresholding failed - result is empty";
    } else {
        qDebug() << "MiniVideoLoader: Thresholding successful - result size:" << m_thresholdedCropFrame.cols << "x" << m_thresholdedCropFrame.rows 
                 << "type:" << m_thresholdedCropFrame.type();
        
        // Count foreground vs background pixels
        int totalPixels = m_thresholdedCropFrame.rows * m_thresholdedCropFrame.cols;
        int foregroundPixels = cv::countNonZero(m_thresholdedCropFrame);
        int backgroundPixels = totalPixels - foregroundPixels;
        double foregroundPercent = (double)foregroundPixels / totalPixels * 100.0;
        
        qDebug() << "MiniVideoLoader: Threshold analysis - total pixels:" << totalPixels 
                 << "foreground:" << foregroundPixels << "(" << QString::number(foregroundPercent, 'f', 1) << "%)"
                 << "background:" << backgroundPixels;
    }

    // After thresholding, extract contours to populate detected blobs for this cropped frame
    m_detectedBlobs.clear();
    m_detectedBlobContours.clear();
    if (!m_thresholdedCropFrame.empty()) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(m_thresholdedCropFrame.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &c : contours) {
            double area = cv::contourArea(c);
            if (area < 5.0 || area > 100000.0) continue; // filter extremes
            cv::Moments mu = cv::moments(c);
            if (mu.m00 == 0) continue;
            QPointF cropCentroid(mu.m10 / mu.m00, mu.m01 / mu.m00);
            QPointF videoCentroid = mapCropToVideoCoords(cropCentroid);

            cv::Rect bbox = cv::boundingRect(c);
            QPointF topLeft = mapCropToVideoCoords(QPointF(bbox.x, bbox.y));
            QPointF bottomRight = mapCropToVideoCoords(QPointF(bbox.x + bbox.width, bbox.y + bbox.height));

            Tracking::DetectedBlob db;
            db.isValid = true;
            db.centroid = videoCentroid;
            db.area = area;
            db.boundingBox = QRectF(topLeft, bottomRight);

            m_detectedBlobs.append(db);
            m_detectedBlobContours.push_back(c);
        }
        qDebug() << "MiniVideoLoader: Detected" << m_detectedBlobs.size() << "blobs in cropped frame";
    }
}

void MiniVideoLoader::convertCroppedCvMatToQImage()
{
    if (m_croppedFrameCv.empty()) {
        m_croppedFrame = QImage();
        return;
    }
    
    if (m_croppedFrameCv.channels() == 1) {
        m_croppedFrame = QImage(m_croppedFrameCv.data, m_croppedFrameCv.cols, m_croppedFrameCv.rows, 
                               m_croppedFrameCv.step, QImage::Format_Grayscale8).copy();
    } else if (m_croppedFrameCv.channels() == 3) {
        cv::Mat rgbFrame;
        cv::cvtColor(m_croppedFrameCv, rgbFrame, cv::COLOR_BGR2RGB);
        m_croppedFrame = QImage(rgbFrame.data, rgbFrame.cols, rgbFrame.rows, 
                               rgbFrame.step, QImage::Format_RGB888).copy();
    }
}

Thresholding::ThresholdSettings MiniVideoLoader::getThresholdSettings() const
{
    if (!m_videoLoader) {
        return Thresholding::ThresholdSettings();
    }
    
    Thresholding::ThresholdSettings settings;
    settings.algorithm = m_videoLoader->getCurrentThresholdAlgorithm();
    settings.globalThresholdValue = m_videoLoader->getThresholdValue();
    settings.assumeLightBackground = m_videoLoader->getAssumeLightBackground();
    
    // Debug: Log the threshold settings being used
    qDebug() << "MiniVideoLoader: Threshold settings - algorithm:" << static_cast<int>(settings.algorithm) 
             << "globalThresholdValue:" << settings.globalThresholdValue
             << "assumeLightBackground:" << settings.assumeLightBackground;
    settings.adaptiveBlockSize = m_videoLoader->getAdaptiveBlockSize();
    settings.adaptiveCValue = m_videoLoader->getAdaptiveCValue();
    settings.enableBlur = m_videoLoader->isBlurEnabled();
    settings.blurKernelSize = m_videoLoader->getBlurKernelSize();
    settings.blurSigmaX = m_videoLoader->getBlurSigmaX();
    
    return settings;
}