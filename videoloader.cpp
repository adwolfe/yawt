#include "VideoLoader.h"
#include <QDebug>
#include <QPainterPath>
#include <QtMath>
#include <QResizeEvent>

VideoLoader::VideoLoader(QWidget *parent)
    : QWidget(parent),
    playbackTimer(new QTimer(this)),
    totalFramesCount(0),
    framesPerSecond(0.0),
    currentFrameIdx(-1),
    m_isPlaying(false),
    m_currentMode(InteractionMode::PanZoom), // Default mode
    m_isPanning(false),
    roiBeingDefined(false),
    m_zoomFactor(1.0),
    m_panOffset(0.0, 0.0)
{
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);

    connect(playbackTimer, &QTimer::timeout, this, &VideoLoader::processNextFrame);
    setMouseTracking(true); // Needed for cursor updates on hover potentially
    updateCursorShape(); // Set initial cursor
}

VideoLoader::~VideoLoader() {
    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
}

// --- Public Methods ---

bool VideoLoader::isVideoLoaded() const {
    return videoCapture.isOpened() && totalFramesCount > 0;
}

int VideoLoader::getTotalFrames() const {
    return totalFramesCount;
}

double VideoLoader::getFPS() const {
    return framesPerSecond;
}

int VideoLoader::getCurrentFrameNumber() const {
    return currentFrameIdx;
}

QSize VideoLoader::getVideoFrameSize() const {
    return originalFrameSize;
}

double VideoLoader::getZoomFactor() const {
    return m_zoomFactor;
}

QRectF VideoLoader::getCurrentRoi() const {
    return currentRoiRect;
}

InteractionMode VideoLoader::getCurrentInteractionMode() const {
    return m_currentMode;
}


// --- Slots ---

bool VideoLoader::loadVideo(const QString &filePath) {
    if (m_isPlaying) {
        stop();
    }

    // Reset state completely
    m_zoomFactor = 1.0;
    m_panOffset = QPointF(0.0, 0.0);
    currentRoiRect = QRectF();
    // Don't reset mode, keep the user's selected mode
    // m_currentMode = InteractionMode::PanZoom;
    m_isPanning = false;
    roiBeingDefined = false;


    if (!openVideoFile(filePath)) {
        // Clear all video-related members
        currentFilePath.clear();
        totalFramesCount = 0;
        framesPerSecond = 0.0;
        currentFrameIdx = -1;
        originalFrameSize = QSize();
        currentQImageFrame = QImage();
        update(); // Clear display
        updateCursorShape(); // Reset cursor if video load fails
        emit videoLoadFailed(filePath, "Failed to open video file with OpenCV.");
        return false;
    }

    currentFilePath = filePath;
    framesPerSecond = videoCapture.get(cv::CAP_PROP_FPS);
    if (framesPerSecond <= 0) framesPerSecond = 25.0;

    totalFramesCount = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    originalFrameSize = QSize(static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH)),
                              static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT)));

    if (totalFramesCount <= 0 || originalFrameSize.isEmpty()) {
        emit videoLoadFailed(filePath, "Video file seems empty or metadata is corrupted.");
        videoCapture.release();
        currentFilePath.clear();
        update();
        updateCursorShape();
        return false;
    }

    seekToFrame(0); // Load first frame

    // Emit signals AFTER state is updated
    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor);
    emit roiDefined(currentRoiRect); // Emit initial empty ROI
    updateCursorShape(); // Update cursor for loaded video state
    qDebug() << "Video loaded:" << filePath << "Frames:" << totalFramesCount << "FPS:" << framesPerSecond;
    return true;
}

bool VideoLoader::openVideoFile(const QString &filePath) {
    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
    try {
        if (!videoCapture.open(filePath.toStdString())) {
            qWarning() << "OpenCV: Failed to open video" << filePath;
            return false;
        }
    } catch (const cv::Exception& ex) {
        qWarning() << "OpenCV exception while opening video:" << ex.what();
        return false;
    }
    return videoCapture.isOpened();
}


void VideoLoader::play() {
    if (!isVideoLoaded()) {
        return;
    }
    if (m_isPlaying) {
        pause();
        return;
    }
    if (currentFrameIdx >= totalFramesCount - 1) {
        seekToFrame(0);
        if (currentFrameIdx == -1) return;
    }
    m_isPlaying = true;
    int interval = (framesPerSecond > 0) ? qRound(1000.0 / framesPerSecond) : 40;
    playbackTimer->start(interval);
    emit playbackStateChanged(true);
    qDebug() << "Playback started.";
}

void VideoLoader::pause() {
    if (!isVideoLoaded()) {
        return;
    }
    m_isPlaying = false;
    playbackTimer->stop();
    emit playbackStateChanged(false);
    qDebug() << "Playback paused.";
}

void VideoLoader::stop() {
    if (!isVideoLoaded()) return;
    bool wasPlaying = m_isPlaying;
    m_isPlaying = false;
    playbackTimer->stop();
    int oldFrame = currentFrameIdx;
    seekToFrame(0);
    if (wasPlaying || oldFrame != 0) {
        emit playbackStateChanged(false);
    }
    qDebug() << "Playback stopped.";
}

void VideoLoader::seekToFrame(int frameNumber, bool suppressEmit) {
    if (!isVideoLoaded()) {
        return;
    }
    frameNumber = qBound(0, frameNumber, totalFramesCount > 0 ? totalFramesCount - 1 : 0);
    displayFrame(frameNumber, suppressEmit);
}

void VideoLoader::displayFrame(int frameNumber, bool suppressEmit) {
    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) {
        qWarning() << "displayFrame called with no video loaded or invalid state.";
        currentFrameIdx = -1;
        currentQImageFrame = QImage();
        update();
        return;
    }
    if (frameNumber < 0 || frameNumber >= totalFramesCount) {
        qWarning() << "displayFrame: frameNumber" << frameNumber << "out of range [0," << totalFramesCount - 1 << "]";
        currentFrameIdx = -1;
        currentQImageFrame = QImage();
        update();
        return;
    }

    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    if (currentPos != frameNumber) {
        if(!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) {
            qWarning() << "Failed to set frame position to" << frameNumber << "(current pos:" << currentPos << ")";
        }
    }

    if (videoCapture.read(currentCvFrame)) {
        if (!currentCvFrame.empty()) {
            convertCvMatToQImage(currentCvFrame, currentQImageFrame);
            currentFrameIdx = frameNumber;
            if (!suppressEmit)
            {
                emit frameChanged(currentFrameIdx, currentQImageFrame);
            }
        } else {
            qWarning() << "Read empty frame at index" << frameNumber;
            currentFrameIdx = frameNumber;
            currentQImageFrame = QImage();
            if (!suppressEmit)
            {
                emit frameChanged(currentFrameIdx, currentQImageFrame);
            }
        }
    } else {
        qWarning() << "Failed to read frame at index" << frameNumber;
        currentFrameIdx = frameNumber;
        currentQImageFrame = QImage();
        if (!suppressEmit)
        {
            emit frameChanged(currentFrameIdx, currentQImageFrame);
        }
        if (m_isPlaying) {
            stop();
        }
    }
    update();
}


void VideoLoader::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) {
    // ... (conversion logic remains the same as previous version)
    if (mat.empty()) {
        qimg = QImage();
        return;
    }
    if (mat.type() == CV_8UC3) {
        qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped();
    }
    else if (mat.type() == CV_8UC1) {
        qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8);
    }
    else {
        cv::Mat temp;
        try {
            if (mat.channels() == 4) {
                cv::cvtColor(mat, temp, cv::COLOR_BGRA2BGR);
                qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped();
            } else if (mat.channels() == 1) {
                cv::Mat temp8U;
                mat.convertTo(temp8U, CV_8U);
                qimg = QImage(temp8U.data, temp8U.cols, temp8U.rows, static_cast<int>(temp8U.step), QImage::Format_Grayscale8);
            } else {
                qWarning() << "Unsupported cv::Mat type for conversion:" << mat.type() << "channels:" << mat.channels();
                qimg = QImage();
                return;
            }
        } catch (const cv::Exception& ex) {
            qWarning() << "OpenCV exception during color conversion:" << ex.what();
            qimg = QImage();
        }
    }
}


void VideoLoader::processNextFrame() {
    if (!m_isPlaying || !isVideoLoaded()) {
        return;
    }
    if (currentFrameIdx < totalFramesCount - 1) {
        displayFrame(currentFrameIdx + 1);
    } else {
        stop();
    }
}

// Zooms towards the center of the widget
void VideoLoader::setZoomFactor(double factor) {
    setZoomFactorAtPoint(factor, rect().center());
}

// Set zoom factor, adjusting pan to keep the content under widgetPoint stationary
void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) {
    if (!isVideoLoaded()) return; // Don't zoom if no video

    double newZoomFactor = qBound(0.05, factor, 50.0); // Clamp zoom factor

    if (qFuzzyCompare(m_zoomFactor, newZoomFactor)) {
        return; // No significant change
    }

    QPointF videoPoint = mapPointToVideo(widgetPoint);
    // If mapping fails (e.g., point outside video), zoom towards center instead
    QPointF referenceWidgetPoint = widgetPoint;
    if (videoPoint.x() < 0) {
        referenceWidgetPoint = rect().center();
        videoPoint = mapPointToVideo(referenceWidgetPoint);
        if (videoPoint.x() < 0) { // Still failed? Abort.
            qWarning() << "Cannot determine video point for zoom anchor.";
            return;
        }
    }

    double oldZoomFactor = m_zoomFactor; // Store old factor before changing
    m_zoomFactor = newZoomFactor; // Apply new zoom factor

    // Calculate where the video point maps back *after* zoom
    QPointF widgetPointAfterZoom = mapPointFromVideo(videoPoint);

    // Calculate the pan correction needed
    QPointF panCorrection = referenceWidgetPoint - widgetPointAfterZoom;

    // Apply the correction to the pan offset
    m_panOffset += panCorrection;

    clampPanOffset(); // Ensure video stays reasonably in view
    update();
    emit zoomFactorChanged(m_zoomFactor);
}

// Switch interaction mode
void VideoLoader::setInteractionMode(InteractionMode mode) {
    if (m_currentMode == mode) return; // No change

    m_currentMode = mode;

    // Reset temporary states when switching modes
    m_isPanning = false;
    roiBeingDefined = false;

    updateCursorShape(); // Update cursor for the new mode
    emit interactionModeChanged(m_currentMode);
    qDebug() << "Interaction mode set to:" << static_cast<int>(m_currentMode);
}


void VideoLoader::clearRoi() {
    if (!currentRoiRect.isNull()) {
        currentRoiRect = QRectF();
        update();
        emit roiDefined(currentRoiRect);
        qDebug() << "ROI cleared.";
    }
}

// --- Event Handlers ---

void VideoLoader::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    if (currentQImageFrame.isNull() || !isVideoLoaded() || originalFrameSize.isEmpty()) {
        painter.fillRect(rect(), palette().color(QPalette::Window));
        painter.setPen(Qt::gray);
        painter.drawText(rect(), Qt::AlignCenter, "No video loaded or frame unavailable.");
        return;
    }

    // --- Calculate Target Rectangle (incorporating pan) ---
    QRectF targetRect = calculateTargetRect();

    // --- Draw Video Frame ---
    painter.drawImage(targetRect, currentQImageFrame, currentQImageFrame.rect());

    // --- Draw Final ROI ---
    if (!currentRoiRect.isNull() && currentRoiRect.isValid()) {
        QPointF roiTopLeftVideo = currentRoiRect.topLeft();
        QPointF roiBottomRightVideo = currentRoiRect.bottomRight();
        QPointF roiTopLeftWidget = mapPointFromVideo(roiTopLeftVideo);
        QPointF roiBottomRightWidget = mapPointFromVideo(roiBottomRightVideo);

        // Check if mapping was valid before drawing
        if (roiTopLeftWidget.x() >= 0 && roiBottomRightWidget.x() >= 0) {
            QRectF roiWidgetRect(roiTopLeftWidget, roiBottomRightWidget);
            painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
            painter.setBrush(Qt::NoBrush);
            painter.drawRect(roiWidgetRect.normalized());
        }
    }

    // --- Draw ROI Being Defined ---
    if (m_currentMode == InteractionMode::DrawROI && roiBeingDefined) {
        painter.setPen(QPen(Qt::cyan, 1, Qt::SolidLine));
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(QRect(roiStartPoint, roiEndPoint).normalized());
    }
}

// Helper function to calculate the target drawing rectangle
QRectF VideoLoader::calculateTargetRect() const {
    if (originalFrameSize.isEmpty()) return QRectF();

    QSizeF widgetSizeF = size();
    QSizeF originalFrameSizeF = originalFrameSize;
    QSizeF containedSize = originalFrameSizeF;
    containedSize.scale(widgetSizeF, Qt::KeepAspectRatio);
    QSizeF zoomedSize = containedSize * m_zoomFactor;

    // Start with the centered top-left position
    QPointF centeredTopLeft(
        (widgetSizeF.width() - zoomedSize.width()) / 2.0,
        (widgetSizeF.height() - zoomedSize.height()) / 2.0
        );

    // Apply the pan offset
    QPointF finalTopLeft = centeredTopLeft + m_panOffset;

    return QRectF(finalTopLeft, zoomedSize);
}


// Map Widget Coordinates -> Video Coordinates
QPointF VideoLoader::mapPointToVideo(const QPointF& widgetPoint) const {
    if (currentQImageFrame.isNull() || originalFrameSize.isEmpty() || m_zoomFactor <= 0) {
        return QPointF(-1, -1);
    }

    QRectF targetRect = calculateTargetRect(); // Use helper

    if (!targetRect.contains(widgetPoint) || targetRect.width() <= 0 || targetRect.height() <= 0) {
        return QPointF(-1, -1);
    }

    double normX = (widgetPoint.x() - targetRect.left()) / targetRect.width();
    double normY = (widgetPoint.y() - targetRect.top()) / targetRect.height();

    QSizeF originalFrameSizeF = originalFrameSize;
    double videoX = normX * originalFrameSizeF.width();
    double videoY = normY * originalFrameSizeF.height();

    // Clamp to valid video coordinate range [0, width] x [0, height]
    // Use slightly more robust clamping
    videoX = qBound(0.0, videoX, originalFrameSizeF.width());
    videoY = qBound(0.0, videoY, originalFrameSizeF.height());

    return QPointF(videoX, videoY);
}

// Map Video Coordinates -> Widget Coordinates
QPointF VideoLoader::mapPointFromVideo(const QPointF& videoPoint) const {
    if (currentQImageFrame.isNull() || originalFrameSize.isEmpty() || m_zoomFactor <= 0
        || originalFrameSize.width() <= 0 || originalFrameSize.height() <= 0) {
        return QPointF(-1, -1);
    }

    QRectF targetRect = calculateTargetRect(); // Use helper

    if (targetRect.width() <= 0 || targetRect.height() <= 0) {
        return QPointF(-1, -1);
    }

    QSizeF originalFrameSizeF = originalFrameSize;
    QPointF clampedVideoPoint(
        qBound(0.0, videoPoint.x(), originalFrameSizeF.width()),
        qBound(0.0, videoPoint.y(), originalFrameSizeF.height())
        );
    double normX = clampedVideoPoint.x() / originalFrameSizeF.width();
    double normY = clampedVideoPoint.y() / originalFrameSizeF.height();

    double widgetX = targetRect.left() + normX * targetRect.width();
    double widgetY = targetRect.top() + normY * targetRect.height();

    return QPointF(widgetX, widgetY);
}


void VideoLoader::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->position(); // Store start position for panning/ROI

    if (!isVideoLoaded()) {
        QWidget::mousePressEvent(event); // Pass event if no video
        return;
    }

    // --- Pan Mode ---
    if (m_currentMode == InteractionMode::PanZoom && event->button() == Qt::LeftButton) {
        m_isPanning = true;
        updateCursorShape(); // Set closed hand cursor
        event->accept();
    }
    // --- Draw ROI Mode ---
    else if (m_currentMode == InteractionMode::DrawROI && event->button() == Qt::LeftButton) {
        QPointF videoCoords = mapPointToVideo(event->position());
        if (videoCoords.x() >= 0) { // Check if click is inside video area
            roiStartPoint = event->pos();
            roiEndPoint = event->pos();
            roiBeingDefined = true;
            if (!currentRoiRect.isNull()) { // Clear previous final ROI visually
                currentRoiRect = QRectF();
                // Don't emit yet
            }
            update();
            qDebug() << "ROI definition started.";
            event->accept();
        } else {
            qDebug() << "ROI start ignored - click outside video area.";
            roiBeingDefined = false;
        }
    }
    else {
        QWidget::mousePressEvent(event); // Pass unhandled buttons/modes
    }
}

void VideoLoader::mouseMoveEvent(QMouseEvent *event) {
    QPointF currentPos = event->position();
    QPointF delta = currentPos - m_lastMousePos;

    if (!isVideoLoaded()) {
        QWidget::mouseMoveEvent(event);
        return;
    }

    // --- Panning ---
    if (m_isPanning && (event->buttons() & Qt::LeftButton)) {
        m_panOffset += delta; // Apply pan delta
        clampPanOffset();     // Keep video in view
        update();             // Redraw
        event->accept();
    }
    // --- Drawing ROI ---
    else if (m_currentMode == InteractionMode::DrawROI && roiBeingDefined && (event->buttons() & Qt::LeftButton)) {
        roiEndPoint = event->pos();
        update(); // Redraw to show ROI update
        event->accept();
    }
    else {
        QWidget::mouseMoveEvent(event); // Pass unhandled events
    }

    m_lastMousePos = currentPos; // Update last position for next delta
}

void VideoLoader::mouseReleaseEvent(QMouseEvent *event) {
    if (!isVideoLoaded()) {
        QWidget::mouseReleaseEvent(event);
        return;
    }

    // --- Panning End ---
    if (m_isPanning && event->button() == Qt::LeftButton) {
        m_isPanning = false;
        updateCursorShape(); // Reset to open hand or arrow
        event->accept();
    }
    // --- ROI Definition End ---
    else if (m_currentMode == InteractionMode::DrawROI && roiBeingDefined && event->button() == Qt::LeftButton) {
        roiEndPoint = event->pos();
        roiBeingDefined = false;

        QPointF videoRoiStart = mapPointToVideo(roiStartPoint);
        QPointF videoRoiEnd = mapPointToVideo(roiEndPoint);

        if(videoRoiStart.x() >= 0 && videoRoiEnd.x() >= 0) { // Check validity
            currentRoiRect = QRectF(videoRoiStart, videoRoiEnd).normalized();
            if (currentRoiRect.width() < 5.0 || currentRoiRect.height() < 5.0) {
                qDebug() << "ROI too small, cleared.";
                currentRoiRect = QRectF();
            }
            emit roiDefined(currentRoiRect);
            qDebug() << "ROI selection ended. Video_rect:" << currentRoiRect;
        } else {
            qDebug() << "ROI definition invalid (partially outside video area), cleared.";
            currentRoiRect = QRectF();
            emit roiDefined(currentRoiRect);
        }
        update(); // Show final ROI (or lack thereof)
        updateCursorShape(); // Reset to cross cursor
        event->accept();
    }
    else {
        QWidget::mouseReleaseEvent(event); // Pass unhandled events
    }
}

// Zoom towards the mouse cursor position
void VideoLoader::wheelEvent(QWheelEvent *event) {
    if (!isVideoLoaded() || m_currentMode != InteractionMode::PanZoom) {
        event->ignore(); // Only zoom in PanZoom mode with video loaded
        return;
    }

    // --- Zoom Factor Calculation ---
    int degrees = event->angleDelta().y() / 8;
    if (degrees == 0) {
        event->ignore();
        return;
    }
    int steps = degrees / 15;
    double zoomSpeedFactor = 0.15;
    double zoomMultiplier = qPow(1.0 + zoomSpeedFactor, steps);
    double newZoomFactor = m_zoomFactor * zoomMultiplier;
    newZoomFactor = qBound(0.05, newZoomFactor, 50.0);

    if (qFuzzyCompare(m_zoomFactor, newZoomFactor)) {
        event->accept();
        return;
    }

    // --- Zoom Towards Cursor Logic ---
    QPointF mousePointWidget = event->position();
    QPointF videoPoint = mapPointToVideo(mousePointWidget);
    QPointF referenceWidgetPoint = mousePointWidget; // Point to keep fixed

    if (videoPoint.x() < 0) { // If mouse is outside, zoom towards center
        referenceWidgetPoint = rect().center();
        videoPoint = mapPointToVideo(referenceWidgetPoint);
        if (videoPoint.x() < 0) {
            qWarning() << "Cannot determine video point for zoom anchor.";
            event->ignore(); // Cannot proceed
            return;
        }
    }

    // Apply zoom and calculate pan correction
    m_zoomFactor = newZoomFactor;
    QPointF widgetPointAfterZoom = mapPointFromVideo(videoPoint);
    QPointF panCorrection = referenceWidgetPoint - widgetPointAfterZoom;
    m_panOffset += panCorrection; // Apply pan correction

    clampPanOffset(); // Keep video in view
    update();
    emit zoomFactorChanged(m_zoomFactor);
    event->accept();
}

// Keep track of widget size changes to adjust view if necessary
void VideoLoader::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    // When the widget resizes, the relationship between pan offset and
    // the visual position changes. We might need to adjust pan/zoom
    // or simply repaint. Clamping pan ensures it stays reasonable.
    if (isVideoLoaded()) {
        clampPanOffset(); // Re-evaluate pan limits
        update(); // Repaint with new size
    }
}


// --- Private Helper Methods ---

// Update cursor shape based on mode and state
void VideoLoader::updateCursorShape() {
    if (!isVideoLoaded()) {
        setCursor(Qt::ArrowCursor);
        return;
    }

    switch (m_currentMode) {
    case InteractionMode::PanZoom:
        if (m_isPanning) {
            setCursor(Qt::ClosedHandCursor);
        } else {
            setCursor(Qt::OpenHandCursor); // Indicate pannable
        }
        break;
    case InteractionMode::DrawROI:
        // Keep cross cursor even when dragging for ROI
        setCursor(Qt::CrossCursor);
        break;
    default:
        setCursor(Qt::ArrowCursor);
        break;
    }
}

// Clamp pan offset to keep the video somewhat visible
void VideoLoader::clampPanOffset() {
    if (!isVideoLoaded() || m_zoomFactor <= 0) return;

    QRectF targetRect = calculateTargetRect(); // Get current target rect based on pan
    QSizeF widgetSize = size();

    // Calculate maximum allowed offsets to keep *some* part of the video visible
    // Allow panning until the video edge just touches the widget edge.
    double maxOffsetX = qMax(0.0, targetRect.width() - widgetSize.width());
    double maxOffsetY = qMax(0.0, targetRect.height() - widgetSize.height());

    // The pan offset is relative to the *centered* position.
    // We need to relate it back to the absolute targetRect position.
    // targetRect.topLeft() = centeredTopLeft + m_panOffset
    // We want to limit targetRect.left(), targetRect.top(), targetRect.right(), targetRect.bottom()

    // Limit how far left/up the video's top-left corner can go (relative to widget top-left)
    // Don't let targetRect.left() go beyond widget width - some margin (e.g., 50px)
    // Don't let targetRect.top() go beyond widget height - some margin
    double minX = -(targetRect.width() - 50.0); // Allow video right edge to reach 50px from widget left
    double minY = -(targetRect.height() - 50.0); // Allow video bottom edge to reach 50px from widget top

    // Limit how far right/down the video's top-left corner can go
    // Don't let targetRect.right() go below some margin
    // Don't let targetRect.bottom() go below some margin
    double maxX = widgetSize.width() - 50.0; // Allow video left edge to reach 50px from widget right
    double maxY = widgetSize.height() - 50.0; // Allow video top edge to reach 50px from widget bottom

    // Calculate the allowed range for finalTopLeft based on these limits
    QPointF currentFinalTopLeft = targetRect.topLeft();
    QPointF clampedFinalTopLeft;

    clampedFinalTopLeft.setX(qBound(minX, currentFinalTopLeft.x(), maxX));
    clampedFinalTopLeft.setY(qBound(minY, currentFinalTopLeft.y(), maxY));

    // If the video is smaller than the widget, force it towards center (no panning needed)
    if (targetRect.width() <= widgetSize.width()) {
        clampedFinalTopLeft.setX((widgetSize.width() - targetRect.width()) / 2.0);
    }
    if (targetRect.height() <= widgetSize.height()) {
        clampedFinalTopLeft.setY((widgetSize.height() - targetRect.height()) / 2.0);
    }


    // Recalculate the pan offset needed to achieve this clamped position
    QSizeF originalFrameSizeF = originalFrameSize;
    QSizeF containedSize = originalFrameSizeF;
    containedSize.scale(widgetSize, Qt::KeepAspectRatio);
    QSizeF zoomedSize = containedSize * m_zoomFactor;
    QPointF centeredTopLeft(
        (widgetSize.width() - zoomedSize.width()) / 2.0,
        (widgetSize.height() - zoomedSize.height()) / 2.0
        );

    QPointF requiredPanOffset = clampedFinalTopLeft - centeredTopLeft;

    // Only update if the clamped offset is different
    if (!qFuzzyCompare(m_panOffset.x(), requiredPanOffset.x()) ||
        !qFuzzyCompare(m_panOffset.y(), requiredPanOffset.y())) {
        m_panOffset = requiredPanOffset;
        // Don't trigger repaint here, caller (zoom/pan/resize) will do it.
    }
}
