#include "miniloader.h"
#include <QPainter>
#include <QPainterPath>
#include <QDebug>
#include "../../utils/loggingcategories.h"
#include "trackingdatastorage.h"

MiniLoader::MiniLoader(QWidget* parent)
    : QWidget(parent)
    , m_currentFrameNumber(-1)
    , m_centerPoint(0, 0)
    , m_cropOffset(0, 0)
    , m_cropSize(100, 100)
    , m_trackingDataStorage(nullptr)
    , m_showOverlays(true)
    , m_showSkeleton(false)
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
    // Avoid reconnecting if the same storage pointer is provided
    if (m_trackingDataStorage == storage) {
        return;
    }

    // If we previously had a storage pointer, disconnect any signals from it to avoid duplicate connections
    if (m_trackingDataStorage) {
        // Disconnect all connections from the old storage to this MiniLoader
        disconnect(m_trackingDataStorage, nullptr, this, nullptr);
    }

    m_trackingDataStorage = storage;

    // Connect to the authoritative bulk itemsChanged signal so we can rebuild our id->color map.
    if (m_trackingDataStorage) {
        connect(m_trackingDataStorage, &TrackingDataStorage::itemsChanged,
                this, &MiniLoader::updateItemsToDisplay);
    }
}

/**
 * @brief Update the MiniLoader's id->color cache from the authoritative list of items
 * and trigger a repaint.
 *
 * This slot is intended to be connected to TrackingDataStorage::itemsChanged /
 * BlobTableModel::itemsChanged so the MiniLoader uses the canonical colors.
 */
void MiniLoader::updateItemsToDisplay(const QList<TableItems::ClickedItem>& items)
{
    m_idColors.clear();
    for (const TableItems::ClickedItem& item : items) {
        if (item.color.isValid()) {
            m_idColors.insert(item.id, item.color);
        }
    }

    // Trigger a redraw so overlays use the new colors
    update();
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

    YAWT_INFO(lcGuiMiniLoader) << "MiniLoader: Selected worm changed to" << wormId;

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

void MiniLoader::setShowSkeleton(bool show)
{
    if (m_showSkeleton == show) return;
    m_showSkeleton = show;
    update();
}

bool MiniLoader::showSkeleton() const
{
    return m_showSkeleton;
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

/**
 * Helper: compute absolute polygon area using shoelace formula
 */
static double polygonArea(const QPolygonF& poly) {
    if (poly.isEmpty()) return 0.0;
    double area = 0.0;
    int n = poly.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        area += (poly[j].x() * poly[i].y()) - (poly[i].x() * poly[j].y());
    }
    return qAbs(area) * 0.5;
}

/**
 * Helper: convert cv::contour to QPolygonF in crop-local coordinates by subtracting m_cropOffset.
 * (video coordinates -> crop-local)
 */
static QPolygonF contourToCropPolygon(const std::vector<cv::Point>& contour, const QPointF& cropOffset) {
    QPolygonF p;
    p.reserve(static_cast<int>(contour.size()));
    for (const cv::Point& pt : contour) {
        p.append(QPointF(pt.x - cropOffset.x(), pt.y - cropOffset.y()));
    }
    return p;
}

/**
 * Convert a polygon expressed in crop-local coords to widget coords (targetRect mapping)
 */
static QPolygonF cropPolygonToWidget(const QPolygonF& cropPoly, const QRect& targetRect, const QImage& croppedFrame) {
    QPolygonF widgetPoly;
    if (croppedFrame.width() <= 0 || croppedFrame.height() <= 0) return widgetPoly;
    double sx = static_cast<double>(targetRect.width()) / croppedFrame.width();
    double sy = static_cast<double>(targetRect.height()) / croppedFrame.height();
    for (const QPointF& pt : cropPoly) {
        widgetPoly.append(QPointF(targetRect.left() + pt.x() * sx,
                                  targetRect.top()  + pt.y() * sy));
    }
    return widgetPoly;
}

void MiniLoader::drawOverlays(QPainter& painter, const QRect& targetRect)
{
    YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::drawOverlays called - selectedWorm:" << m_selectedWormId << "frame:" << m_currentFrameNumber << "hasStorage:" << (m_trackingDataStorage != nullptr);

    // If updateWithCroppedFrames precomputed a per-frame visibility map, use it for this draw so
    // painting is consistent with the emitted per-frame signal. Do not emit visibility signals from paint().
    bool havePerFrame = (!m_visibleWormsByFrame.isEmpty() && m_visibleWormsByFrame.contains(m_currentFrameNumber));
    if (havePerFrame) {
        QSet<int> s = m_visibleWormsByFrame.value(m_currentFrameNumber);
        m_visibleWormIds = s.values();
    } else {
        // Clear previous visible IDs each draw; we'll repopulate below
        m_visibleWormIds.clear();
    }

    if (m_currentFrameNumber < 0) {
        YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::drawOverlays early return - bad frame";
        return;
    }

    // Get all detected blobs for the current frame (requires storage). If storage is missing,
    // we can't draw blob outlines so skip drawing; color lookups are preferred from m_idColors.
    QMap<int, Tracking::DetectedBlob> blobMap;
    if (m_trackingDataStorage) {
        blobMap = m_trackingDataStorage->getDetectedBlobsForFrame(m_currentFrameNumber);
        YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::drawOverlays blob map contains keys:" << blobMap.keys();
    } else {
        YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::drawOverlays: no storage available; skipping blob overlay drawing";
        return;
    }

    if (blobMap.isEmpty()) {
        YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::drawOverlays no blobs for frame" << m_currentFrameNumber;
        return;
    }

    // Crop rect in video coordinates
    QRectF cropRectVid = getCurrentCropRectVideo();
    if (cropRectVid.isEmpty()) {
        YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::drawOverlays crop rect empty";
        return;
    }

    // We'll use a small epsilon area (in crop pixels) to avoid tiny numeric intersections
    const double AREA_EPS = 0.5;

    // Iterate all blobs and draw their intersection with the crop (if any)
    for (auto it = blobMap.constBegin(); it != blobMap.constEnd(); ++it) {
        int wormId = it.key();
        const Tracking::DetectedBlob& blob = it.value();

        if (!blob.isValid || blob.contourPoints.empty()) {
            continue;
        }

        // Quick reject by bounding box (video coords)
        if (!cropRectVid.intersects(blob.boundingBox)) {
            continue;
        }

        // Convert blob contour to crop-local polygon (video -> crop-local)
        QPolygonF blobCropPoly = contourToCropPolygon(blob.contourPoints, m_cropOffset);

        // Build QPainterPath for blob and for crop rectangle (crop-local coords)
        QPainterPath blobPath;
        blobPath.addPolygon(blobCropPoly);
        blobPath.closeSubpath();

        // Punch out holes (ring topology from coiled/self-touching worm)
        for (const auto& holeContour : blob.holeContourPoints) {
            QPolygonF holePoly = contourToCropPolygon(holeContour, m_cropOffset);
            QPainterPath holePath;
            holePath.addPolygon(holePoly);
            holePath.closeSubpath();
            blobPath = blobPath.subtracted(holePath);
        }

        QPainterPath cropPath;
        cropPath.addRect(QRectF(0.0, 0.0, m_croppedFrame.width(), m_croppedFrame.height())); // crop-local dims

        // Intersect the two paths
        QPainterPath inter = blobPath.intersected(cropPath);

        // Convert intersection to fill polygon(s)
        QPolygonF interPoly = inter.toFillPolygon();

        double interArea = polygonArea(interPoly);

        if (interArea <= AREA_EPS) {
            // No meaningful intersection
            continue;
        }

        // Record this worm ID as visible (avoid duplicates)
        if (!m_visibleWormIds.contains(wormId)) {
            m_visibleWormIds.append(wormId);
        }

        // Determine color to draw: prefer the cached color map (m_idColors) rebuilt via itemsChanged.
        // Fall back to storage only if the cached color is not present.
        QColor fillColor(220, 60, 60, 100); // default semi-transparent red
        if (m_idColors.contains(wormId)) {
            QColor c = m_idColors.value(wormId);
            c.setAlpha(120);
            fillColor = c;
        } else {
            const TableItems::ClickedItem* item = nullptr;
            if (m_trackingDataStorage) item = m_trackingDataStorage->getItem(wormId);
            if (item) {
                QColor c = item->color;
                c.setAlpha(120);
                fillColor = c;
            }
        }

        // Convert intersection polygon (crop-local) -> widget coords, then draw
        QPolygonF widgetPoly = cropPolygonToWidget(interPoly, targetRect, m_croppedFrame);

        if (!widgetPoly.isEmpty()) {
            // Filled intersection
            painter.setPen(QPen(fillColor.lighter(130), 1));
            painter.setBrush(QBrush(fillColor));
            QPainterPath widgetPath;
            widgetPath.addPolygon(widgetPoly);
            widgetPath.closeSubpath();
            painter.drawPath(widgetPath);

            // Draw an outline for clarity
            QColor outline = fillColor.darker(120);
            outline.setAlpha(200);
            painter.setPen(QPen(outline, 1.5));
            painter.setBrush(Qt::NoBrush);
            painter.drawPath(widgetPath);

            // Draw the worm ID label at the centroid of intersection polygon (in crop coords)
            QPointF centroidCrop(0,0);
            for (const QPointF& p : interPoly) centroidCrop += p;
            centroidCrop /= static_cast<double>(interPoly.size());

            // Convert centroid to widget coords
            QPointF centroidWidget(
                targetRect.left() + centroidCrop.x() * static_cast<double>(targetRect.width()) / m_croppedFrame.width(),
                targetRect.top()  + centroidCrop.y() * static_cast<double>(targetRect.height()) / m_croppedFrame.height()
            );

            QString label = QString::number(wormId);
            QFont f = painter.font();
            f.setPointSize(10);
            f.setBold(true);
            painter.setFont(f);
            painter.setPen(Qt::yellow);
            // Scatter labels slightly (deterministic) so merged worms don't stack perfectly.
            int h = wormId * 37 + m_currentFrameNumber * 17;
            int dx = (h % 9) - 8;                 // [-4, 4]
            int dy = ((h / 9) % 9) - 8;          // [-4, 4]
            painter.drawText(QRectF(centroidWidget.x() - 12 + dx, centroidWidget.y() - 12 + dy, 24, 24),
                             Qt::AlignCenter, label);

            if (blob.hasCenterlineCutPoint) {
                const QPointF cutCrop(blob.centerlineCutPoint.x - m_cropOffset.x(),
                                      blob.centerlineCutPoint.y - m_cropOffset.y());
                const QPointF cutWidget(
                    targetRect.left() + cutCrop.x() * static_cast<double>(targetRect.width()) / m_croppedFrame.width(),
                    targetRect.top() + cutCrop.y() * static_cast<double>(targetRect.height()) / m_croppedFrame.height());
                painter.setPen(QPen(Qt::black, 1.0));
                painter.setBrush(QColor(255, 0, 255));
                painter.drawRect(QRectF(cutWidget.x() - 3.0, cutWidget.y() - 3.0, 6.0, 6.0));
            }
        }
    }

    // If a specific selected worm is set, draw its full contour outline on top for emphasis (optional)
    if (m_selectedWormId >= 0 && blobMap.contains(m_selectedWormId)) {
        const Tracking::DetectedBlob& selBlob = blobMap.value(m_selectedWormId);
        if (selBlob.isValid && !selBlob.contourPoints.empty()) {
            // Convert contour to crop-local polygon and then to widget coords for the full outline
            QPolygonF fullCropPoly = contourToCropPolygon(selBlob.contourPoints, m_cropOffset);
            QPolygonF fullWidgetPoly = cropPolygonToWidget(fullCropPoly, targetRect, m_croppedFrame);
            if (!fullWidgetPoly.isEmpty()) {
                painter.setPen(QPen(QColor(255, 200, 60), 2));
                painter.setBrush(Qt::NoBrush);
                painter.drawPolygon(fullWidgetPoly);
            }
        }
    }

    // Draw centerline skeletons for all visible blobs if enabled
    if (m_showSkeleton) {
        for (int wormId : m_visibleWormIds) {
            if (!blobMap.contains(wormId)) continue;
            const Tracking::DetectedBlob& blob = blobMap.value(wormId);
            if (!blob.isValid || blob.centerlinePoints.empty()) continue;

            QPolygonF linePoly;
            for (const cv::Point2f& pt : blob.centerlinePoints) {
                QPointF cropLocal(pt.x - m_cropOffset.x(), pt.y - m_cropOffset.y());
                QPointF widget(
                    targetRect.left() + cropLocal.x() * targetRect.width()  / m_croppedFrame.width(),
                    targetRect.top()  + cropLocal.y() * targetRect.height() / m_croppedFrame.height());
                linePoly << widget;
            }

            if (linePoly.size() >= 2) {
                const bool isSelected = (wormId == m_selectedWormId);
                painter.setPen(QPen(isSelected ? QColor(255, 255, 0) : QColor(0, 220, 255), isSelected ? 2.0 : 1.5));
                painter.setBrush(Qt::NoBrush);
                for (int i = 0; i < linePoly.size() - 1; ++i)
                    painter.drawLine(linePoly[i], linePoly[i + 1]);

                // Draw tip dots
                painter.setPen(Qt::NoPen);
                painter.setBrush(isSelected ? QColor(255, 100, 0) : QColor(0, 180, 255));
                painter.drawEllipse(linePoly.first(), 3.0, 3.0);
                painter.drawEllipse(linePoly.last(),  3.0, 3.0);
            }
        }
    }

    // Debug: list visible worm IDs found this draw
    YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::drawOverlays visible worm IDs:" << m_visibleWormIds;
    // Do not emit visibleWormsUpdated from paint anymore. Per-frame visibility is emitted from updateWithCroppedFrames
    // via visibleWormsUpdatedPerFrame so listeners get the full multi-frame map in one signal.
}

QPointF MiniLoader::mapVideoToCropCoords(const QPointF& videoCoords) const
{
    double cropX = videoCoords.x() - m_cropOffset.x();
    double cropY = videoCoords.y() - m_cropOffset.y();
    return QPointF(cropX, cropY);
}

QList<int> MiniLoader::getVisibleWormIds() const
{
    return m_visibleWormIds;
}

void MiniLoader::setVisibleWormIds(const QList<int>& ids)
{
    m_visibleWormIds = ids;
}

QMap<int, QSet<int>> MiniLoader::getVisibleWormsByFrame() const
{
    return m_visibleWormsByFrame;
}

void MiniLoader::setVisibleWormsByFrame(const QMap<int, QSet<int>>& map)
{
    m_visibleWormsByFrame = map;
}

void MiniLoader::updateWithCroppedFrames(int startFrameNumber,
                                         const QList<QImage>& croppedFrames,
                                         const QList<QPointF>& cropOffsets,
                                         const QList<QSizeF>& cropSizes,
                                         QPointF centerPoint)
{
    // Basic validation
    if (croppedFrames.isEmpty()) return;
    int n = croppedFrames.size();
    if (cropOffsets.size() != n || cropSizes.size() != n) return;

    // The caller supplies the absolute frame number corresponding to the first image in the list.
    // Determine the center index and corresponding center frame number.
    int centerIndex = n / 2; // integer division; caller should supply odd-length lists when possible
    int centerFrameNumber = startFrameNumber + centerIndex;

    // Store/display the central frame as the widget's current cropped frame
    m_currentFrameNumber = centerFrameNumber;
    m_croppedFrame = croppedFrames.value(centerIndex);
    m_cropOffset = cropOffsets.value(centerIndex);
    m_cropSize = cropSizes.value(centerIndex);
    m_centerPoint = centerPoint;

    // Compute per-frame visibility using the same intersection logic as drawOverlays (but without painting)
    m_visibleWormsByFrame.clear();
    m_visibleWormIds.clear();

    // Small epsilon area threshold to match drawOverlays behavior
    const double AREA_EPS = 0.5;

    for (int i = 0; i < n; ++i) {
        int frameNum = startFrameNumber + i;
        QSet<int> visibleSet;

        if (!m_trackingDataStorage || frameNum < 0) {
            m_visibleWormsByFrame.insert(frameNum, visibleSet);
            continue;
        }

        // Get all detected blobs for this frame
        QMap<int, Tracking::DetectedBlob> blobMap = m_trackingDataStorage->getDetectedBlobsForFrame(frameNum);
        if (blobMap.isEmpty()) {
            m_visibleWormsByFrame.insert(frameNum, visibleSet);
            continue;
        }

        // Build crop rectangle in video coordinates for this supplied frame
        QRectF cropRectVid = QRectF(cropOffsets[i], cropSizes[i]);
        if (cropRectVid.isEmpty()) {
            m_visibleWormsByFrame.insert(frameNum, visibleSet);
            continue;
        }

        // For each blob, test intersection with crop (using the same contour->crop-local conversion)
        for (auto it = blobMap.constBegin(); it != blobMap.constEnd(); ++it) {
            int wormId = it.key();
            const Tracking::DetectedBlob& blob = it.value();

            if (!blob.isValid || blob.contourPoints.empty()) continue;

            // Quick reject by bounding box (video coords)
            if (!cropRectVid.intersects(blob.boundingBox)) continue;

            // Convert blob contour to crop-local polygon (video -> crop-local for this frame)
            QPolygonF blobCropPoly = contourToCropPolygon(blob.contourPoints, cropOffsets[i]);

            // Build QPainterPath for blob and for crop rectangle (crop-local coords)
            QPainterPath blobPath;
            blobPath.addPolygon(blobCropPoly);
            blobPath.closeSubpath();

            // Punch out holes (ring topology from coiled/self-touching worm)
            for (const auto& holeContour : blob.holeContourPoints) {
                QPolygonF holePoly = contourToCropPolygon(holeContour, cropOffsets[i]);
                QPainterPath holePath;
                holePath.addPolygon(holePoly);
                holePath.closeSubpath();
                blobPath = blobPath.subtracted(holePath);
            }

            QPainterPath cropPath;
            cropPath.addRect(QRectF(0.0, 0.0, croppedFrames[i].width(), croppedFrames[i].height()));

            // Intersect the two paths
            QPainterPath inter = blobPath.intersected(cropPath);

            // Convert intersection to fill polygon(s)
            QPolygonF interPoly = inter.toFillPolygon();

            double interArea = polygonArea(interPoly);

            if (interArea <= AREA_EPS) {
                // No meaningful intersection
                continue;
            }

            visibleSet.insert(wormId);
        } // end blob loop

        m_visibleWormsByFrame.insert(frameNum, visibleSet);

        // If this is the center frame also populate the single-frame visible list used elsewhere
        if (i == centerIndex) {
            // Convert QSet -> QList preserving arbitrary order (caller normalizes if needed)
            m_visibleWormIds = visibleSet.values();
        }
    } // end frames loop

// Emit the single-frame visible IDs (center frame) so listeners depending on the current crop get notified.
// m_visibleWormIds was populated above for the center frame.
YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::updateWithCroppedFrames emitting center-frame visible ids:" << m_visibleWormIds;
emit visibleWormsUpdated(m_visibleWormIds);

// Compute the union of all visible worm IDs across the supplied frames so we can build a consistent
// id->color map to accompany the per-frame visibility signal.
QSet<int> unionSet;
for (auto it = m_visibleWormsByFrame.constBegin(); it != m_visibleWormsByFrame.constEnd(); ++it) {
    unionSet.unite(it.value());
}
QList<int> unionList = unionSet.values();

// Build a map from worm ID -> QColor using the TrackingDataStorage (so colors match the overlay).
QMap<int, QColor> idColors;
for (int id : unionSet) {
    // Prefer the authority-provided cached color map (m_idColors). If not available, fall back
    // to storage lookup; if neither yields a color, use a default gray fallback.
    if (m_idColors.contains(id)) {
        idColors.insert(id, m_idColors.value(id));
    } else {
        const TableItems::ClickedItem* item = nullptr;
        if (m_trackingDataStorage) item = m_trackingDataStorage->getItem(id);
        if (item) {
            idColors.insert(id, item->color);
        } else {
            // Fallback color matches MergeViewer's default gray
            idColors.insert(id, QColor(160, 160, 160));
        }
    }
}

// Emit the per-frame visibility map including the center frame and the id->color map so consumers
// (e.g. MergeViewer) can align segments correctly and use consistent colors matching the overlay.
int centerFrame = centerFrameNumber;
YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::updateWithCroppedFrames emitting per-frame map center:" << centerFrame << " ids:" << unionList;
emit visibleWormsUpdatedPerFrame(centerFrame, m_visibleWormsByFrame, idColors);

// Also emit the union list along with the center frame for backward-compatible consumers that need it.
YAWT_DEBUG(lcGuiMiniLoader) << "MiniLoader::updateWithCroppedFrames emitting union ids:" << unionList;
emit visibleWormsUnionUpdated(centerFrame, unionList);

// Trigger a repaint (central frame was updated above)
update();
}
