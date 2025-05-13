#include "videoloader.h"    // Lowercase include
#include "trackingcommon.h" // Lowercase include for findClickedBlob

#include <QDebug>
#include <QPainterPath>
#include <QtMath>
#include <QResizeEvent>
#include <QMessageBox>
#include <QFileInfo>

// Default crop parameters
#define DEFAULT_CROP_EXTENSION ".mp4"
#define DEFAULT_CROP_FOURCC cv::VideoWriter::fourcc('H', '2', '6', '4')


VideoLoader::VideoLoader(QWidget *parent)
    : QWidget(parent),
    playbackTimer(new QTimer(this)),
    totalFramesCount(0),
    framesPerSecond(0.0),
    currentFrameIdx(-1),
    m_isPlaying(false),
    m_playbackSpeedMultiplier(1.0),
    m_currentMode(InteractionMode::PanZoom), // Default mode
    m_isPanning(false),
    m_isDefiningRoi(false),
    m_zoomFactor(1.0),
    m_panOffset(0.0, 0.0),
    // Initialize thresholding & pre-processing members to defaults
    m_showThresholdMask(false),
    m_thresholdAlgorithm(ThresholdAlgorithm::Global),
    m_thresholdValue(127),
    m_assumeLightBackground(true),
    m_adaptiveBlockSize(11),
    m_adaptiveC(2.0),
    m_enableBlur(false),
    m_blurKernelSize(5),
    m_blurSigmaX(0.0)
{
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black); // Dark background for the widget
    setPalette(pal);

    connect(playbackTimer, &QTimer::timeout, this, &VideoLoader::processNextFrame);
    setMouseTracking(true); // Enable mouse tracking for cursor changes and hover events if needed
    updateCursorShape();
}

VideoLoader::~VideoLoader() {
    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
}

// --- Public Methods & Getters ---
// (Implementations for isVideoLoaded, getTotalFrames, getFPS, etc. as before)
bool VideoLoader::isVideoLoaded() const { return videoCapture.isOpened() && totalFramesCount > 0; }
int VideoLoader::getTotalFrames() const { return totalFramesCount; }
double VideoLoader::getFPS() const { return framesPerSecond; }
int VideoLoader::getCurrentFrameNumber() const { return currentFrameIdx; }
QSize VideoLoader::getVideoFrameSize() const { return originalFrameSize; }
double VideoLoader::getZoomFactor() const { return m_zoomFactor; }
QRectF VideoLoader::getCurrentRoi() const { return m_activeRoiRect; }
InteractionMode VideoLoader::getCurrentInteractionMode() const { return m_currentMode; }
double VideoLoader::getPlaybackSpeed() const { return m_playbackSpeedMultiplier; }
bool VideoLoader::isThresholdViewEnabled() const { return m_showThresholdMask; }

ThresholdSettings VideoLoader::getCurrentThresholdSettings() const {
    ThresholdSettings settings;
    settings.assumeLightBackground = m_assumeLightBackground;
    settings.algorithm = m_thresholdAlgorithm;
    settings.globalThresholdValue = m_thresholdValue;
    settings.adaptiveBlockSize = m_adaptiveBlockSize;
    settings.adaptiveCValue = m_adaptiveC;
    settings.enableBlur = m_enableBlur;
    settings.blurKernelSize = m_blurKernelSize;
    settings.blurSigmaX = m_blurSigmaX;
    return settings;
}
// Convenience getters
ThresholdAlgorithm VideoLoader::getCurrentThresholdAlgorithm() const { return m_thresholdAlgorithm; }
int VideoLoader::getThresholdValue() const { return m_thresholdValue; }
bool VideoLoader::getAssumeLightBackground() const { return m_assumeLightBackground; }
int VideoLoader::getAdaptiveBlockSize() const { return m_adaptiveBlockSize; }
double VideoLoader::getAdaptiveCValue() const { return m_adaptiveC; }
bool VideoLoader::isBlurEnabled() const { return m_enableBlur; }
int VideoLoader::getBlurKernelSize() const { return m_blurKernelSize; }
double VideoLoader::getBlurSigmaX() const { return m_blurSigmaX; }


// --- Control Slots ---
bool VideoLoader::loadVideo(const QString &filePath) {
    if (m_isPlaying) {
        pause();
    }
    // Reset states for new video
    m_zoomFactor = 1.0;
    m_panOffset = QPointF(0.0, 0.0);
    m_activeRoiRect = QRectF();
    clearWormSelections(); // Clear any previous worm selections
    m_isPanning = false;
    m_isDefiningRoi = false;
    m_playbackSpeedMultiplier = 1.0;
    m_showThresholdMask = false; // Default to no threshold view on new video
    // Reset threshold parameters to default
    m_thresholdAlgorithm = ThresholdAlgorithm::Global;
    m_thresholdValue = 127;
    m_assumeLightBackground = true;
    m_adaptiveBlockSize = 11;
    m_adaptiveC = 2.0;
    m_enableBlur = false;
    m_blurKernelSize = 5;
    m_blurSigmaX = 0.0;

    if (!openVideoFile(filePath)) {
        currentFilePath.clear(); totalFramesCount = 0; framesPerSecond = 0.0; currentFrameIdx = -1;
        originalFrameSize = QSize(); currentQImageFrame = QImage(); m_thresholdedFrame_mono = cv::Mat();
        update(); updateCursorShape();
        emit videoLoadFailed(filePath, "Failed to open video file with OpenCV.");
        return false;
    }
    currentFilePath = filePath;
    framesPerSecond = videoCapture.get(cv::CAP_PROP_FPS);
    if (framesPerSecond <= 0) framesPerSecond = 25.0; // Default FPS
    totalFramesCount = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    originalFrameSize = QSize(static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH)),
                              static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT)));
    if (totalFramesCount <= 0 || originalFrameSize.isEmpty()) {
        emit videoLoadFailed(filePath, "Video file seems empty or metadata is corrupted.");
        videoCapture.release(); currentFilePath.clear();
        update(); updateCursorShape();
        return false;
    }
    seekToFrame(0); // Load and display the first frame
    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor);
    emit roiDefined(m_activeRoiRect);
    emit playbackSpeedChanged(m_playbackSpeedMultiplier);
    emitThresholdParametersChanged(); // Emit initial threshold settings
    updateCursorShape();
    qDebug() << "Video loaded:" << filePath << "Frames:" << totalFramesCount << "FPS:" << framesPerSecond;
    return true;
}

QString VideoLoader::getCurrentVideoPath() const {
    return currentFilePath; // currentFilePath is already a member variable
}


// play, pause, setPlaybackSpeed, seekToFrame, setZoomFactor*, clearRoi as before
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
        qWarning() << "OpenCV exception while opening video:" << filePath << "Error:" << ex.what();
        return false;
    }
    return videoCapture.isOpened();
}

void VideoLoader::updateTimerInterval() {
    if (framesPerSecond > 0 && m_playbackSpeedMultiplier > 0) {
        int interval = qRound(1000.0 / (framesPerSecond * m_playbackSpeedMultiplier));
        playbackTimer->setInterval(qMax(1, interval));
    } else {
        playbackTimer->setInterval(40); // Default for 25 FPS
    }
}

void VideoLoader::play() {
    if (!isVideoLoaded()) return;
    if (m_isPlaying) { pause(); }
    else {
        if (currentFrameIdx >= totalFramesCount - 1 && totalFramesCount > 0) seekToFrame(0);
        if (currentFrameIdx == -1 && totalFramesCount > 0) return; // Seek failed or no frames
        m_isPlaying = true;
        updateTimerInterval();
        playbackTimer->start();
        emit playbackStateChanged(true, m_playbackSpeedMultiplier);
        qDebug() << "Playback started. Speed:" << m_playbackSpeedMultiplier << "x";
    }
}

void VideoLoader::pause() {
    if (!m_isPlaying && !playbackTimer->isActive()) return; // Already paused
    m_isPlaying = false;
    playbackTimer->stop();
    emit playbackStateChanged(false, m_playbackSpeedMultiplier);
    qDebug() << "Playback paused.";
}

void VideoLoader::setPlaybackSpeed(double multiplier) {
    double newSpeed = qBound(0.1, multiplier, 10.0); // Speed range 0.1x to 10x
    if (qFuzzyCompare(m_playbackSpeedMultiplier, newSpeed)) return;
    m_playbackSpeedMultiplier = newSpeed;
    qDebug() << "Playback speed set to:" << m_playbackSpeedMultiplier << "x";
    if (m_isPlaying) { // If playing, update timer immediately
        playbackTimer->stop();
        updateTimerInterval();
        playbackTimer->start();
    }
    emit playbackSpeedChanged(m_playbackSpeedMultiplier);
    emit playbackStateChanged(m_isPlaying, m_playbackSpeedMultiplier); // Update UI
}

void VideoLoader::seekToFrame(int frameNumber, bool suppressEmit) {
    if (!isVideoLoaded()) return;
    frameNumber = qBound(0, frameNumber, totalFramesCount > 0 ? totalFramesCount - 1 : 0);
    displayFrame(frameNumber, suppressEmit);
}

void VideoLoader::setZoomFactor(double factor) { setZoomFactorAtPoint(factor, rect().center()); }

void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) {
    if (!isVideoLoaded()) return;
    double newZoomFactor = qBound(0.05, factor, 50.0);
    if (qFuzzyCompare(m_zoomFactor, newZoomFactor)) return;
    QPointF videoPointBeforeZoom = mapPointToVideo(widgetPoint);
    m_zoomFactor = newZoomFactor;
    if (videoPointBeforeZoom.x() < 0 || videoPointBeforeZoom.y() < 0) {
        QSizeF ws = size();
        QRectF targetRectNow = calculateTargetRect(); // Recalculate with new zoom
        QPointF currentTopLeft = targetRectNow.topLeft() - m_panOffset; // Original top-left if pan was 0
        // Recenter based on the new zoomed size if the pivot point was invalid
        m_panOffset.setX((ws.width() - targetRectNow.width()) / 2.0 - currentTopLeft.x());
        m_panOffset.setY((ws.height() - targetRectNow.height()) / 2.0 - currentTopLeft.y());
    } else {
        QPointF widgetPointAfterZoom = mapPointFromVideo(videoPointBeforeZoom);
        m_panOffset += (widgetPoint - widgetPointAfterZoom);
    }
    clampPanOffset();
    update();
    emit zoomFactorChanged(m_zoomFactor);
}

void VideoLoader::clearRoi() {
    if (!m_activeRoiRect.isNull()) {
        m_activeRoiRect = QRectF();
        update();
        emit roiDefined(m_activeRoiRect);
        qDebug() << "Active ROI cleared.";
    }
}

void VideoLoader::clearWormSelections() {
    if (!m_selectedCentroids_temp.isEmpty() || !m_selectedBounds_temp.isEmpty()) {
        m_selectedCentroids_temp.clear();
        m_selectedBounds_temp.clear();
        update(); // Trigger repaint to remove drawn selections
        qDebug() << "Cleared temporary worm selections from VideoLoader.";
    }
}

void VideoLoader::setInteractionMode(InteractionMode mode) {
    if (m_currentMode == mode) return;
    m_currentMode = mode;
    m_isPanning = false;   // Reset panning state
    m_isDefiningRoi = false; // Reset ROI definition state
    // Note: m_selectedCentroids_temp are NOT cleared here.
    // MainWindow should call clearWormSelections() if it wants to clear them when mode changes.
    updateCursorShape();
    emit interactionModeChanged(m_currentMode);
    qDebug() << "Interaction mode set to:" << static_cast<int>(m_currentMode);
    update(); // Update to reflect new cursor or selection drawing state
}


// --- Thresholding & Pre-processing Control Slots (implementations as before) ---
void VideoLoader::toggleThresholdView(bool enabled) {
    if (m_showThresholdMask == enabled) return;
    m_showThresholdMask = enabled;
    qDebug() << "Threshold view" << (enabled ? "enabled" : "disabled");
    if (isVideoLoaded() && currentFrameIdx >= 0) {
        displayFrame(currentFrameIdx, true); // Re-process and display
    }
    update(); // Repaint
}
// ... (setThresholdAlgorithm, setThresholdValue, setAssumeLightBackground, etc. as before) ...
void VideoLoader::setThresholdAlgorithm(ThresholdAlgorithm algorithm) {
    if (m_thresholdAlgorithm == algorithm) return;
    m_thresholdAlgorithm = algorithm;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setThresholdValue(int value) {
    value = qBound(0, value, 255); if (m_thresholdValue == value) return;
    m_thresholdValue = value;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && m_thresholdAlgorithm == ThresholdAlgorithm::Global) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setAssumeLightBackground(bool isLight) {
    if (m_assumeLightBackground == isLight) return;
    m_assumeLightBackground = isLight;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setAdaptiveThresholdBlockSize(int blockSize) {
    if (blockSize < 3) blockSize = 3; if (blockSize % 2 == 0) blockSize += 1;
    if (m_adaptiveBlockSize == blockSize) return;
    m_adaptiveBlockSize = blockSize;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && (m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveMean || m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveGaussian)) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setAdaptiveThresholdC(double cValue) {
    cValue = qBound(-50.0, cValue, 50.0); if (qFuzzyCompare(m_adaptiveC, cValue)) return;
    m_adaptiveC = cValue;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && (m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveMean || m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveGaussian)) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setEnableBlur(bool enabled) {
    if (m_enableBlur == enabled) return;
    m_enableBlur = enabled;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >= 0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setBlurKernelSize(int kernelSize) {
    if (kernelSize < 3) kernelSize = 3; if (kernelSize % 2 == 0) kernelSize += 1;
    if (m_blurKernelSize == kernelSize) return;
    m_blurKernelSize = kernelSize;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >= 0 && m_enableBlur) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setBlurSigmaX(double sigmaX) {
    sigmaX = qMax(0.0, sigmaX); if (qFuzzyCompare(m_blurSigmaX, sigmaX)) return;
    m_blurSigmaX = sigmaX;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >= 0 && m_enableBlur) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}

// --- displayFrame, processNextFrame, convertCvMatToQImage ---
void VideoLoader::displayFrame(int frameNumber, bool suppressEmit) {
    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) {
        currentFrameIdx = -1; currentQImageFrame = QImage(); m_thresholdedFrame_mono = cv::Mat(); update(); return;
    }
    if (frameNumber < 0 || frameNumber >= totalFramesCount) {
        qWarning() << "Attempted to display invalid frame number:" << frameNumber;
        if (m_isPlaying) pause();
        currentFrameIdx = -1; currentQImageFrame = QImage(); m_thresholdedFrame_mono = cv::Mat(); update(); return;
    }

    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    if (currentPos != frameNumber && !(currentPos == frameNumber -1 && m_isPlaying) ) {
        if(!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) {
            qWarning() << "Failed to set frame position to" << frameNumber;
        }
    }

    if (videoCapture.read(currentCvFrame)) {
        if (!currentCvFrame.empty()) {
            currentFrameIdx = frameNumber;
            // Crucial: Always call applyThresholding to ensure m_thresholdedFrame_mono
            // is up-to-date for SelectWorms mode, even if m_showThresholdMask is false.
            applyThresholding();
        } else {
            currentCvFrame = cv::Mat(); // Ensure it's empty
            m_thresholdedFrame_mono = cv::Mat(); // Clear if frame is empty
            qWarning() << "Read empty frame at index" << frameNumber;
        }
    } else {
        currentCvFrame = cv::Mat(); // Ensure it's empty
        m_thresholdedFrame_mono = cv::Mat(); // Clear if read fails
        qWarning() << "Failed to read frame at index" << frameNumber;
        if (m_isPlaying) pause();
    }

    // Update the QImage to be displayed based on m_showThresholdMask
    if (m_showThresholdMask && !m_thresholdedFrame_mono.empty()) {
        convertCvMatToQImage(m_thresholdedFrame_mono, currentQImageFrame);
    } else if (!currentCvFrame.empty()) {
        convertCvMatToQImage(currentCvFrame, currentQImageFrame);
    } else {
        currentQImageFrame = QImage(); // Clear display if no valid frame
    }

    if (!suppressEmit) {
        emit frameChanged(currentFrameIdx, currentQImageFrame);
    }
    update(); // Schedule repaint
}

void VideoLoader::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) {
    if (mat.empty()) { qimg = QImage(); return; }
    if (mat.type() == CV_8UC3) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped(); }
    else if (mat.type() == CV_8UC1) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8); }
    else { /* ... (same conversion logic as before for other types) ... */
        cv::Mat temp;
        try {
            if (mat.channels() == 4) { cv::cvtColor(mat, temp, cv::COLOR_BGRA2BGR); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 3 && mat.type() != CV_8UC3) { mat.convertTo(temp, CV_8UC3, 255.0); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 1 && mat.type() != CV_8UC1) { mat.convertTo(temp, CV_8UC1, 255.0); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_Grayscale8); }
            else { qWarning() << "Unsupported cv::Mat type for conversion:" << mat.type(); qimg = QImage(); return; }
        } catch (const cv::Exception& ex) { qWarning() << "OpenCV exception during color conversion:" << ex.what(); qimg = QImage(); }
    }
}

void VideoLoader::processNextFrame() {
    if (!m_isPlaying || !isVideoLoaded()) return;
    if (currentFrameIdx < totalFramesCount - 1) {
        displayFrame(currentFrameIdx + 1);
    } else {
        pause(); // Reached end of video
    }
}

// --- Event Handlers ---
void VideoLoader::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    if (currentQImageFrame.isNull() || !isVideoLoaded()) {
        painter.fillRect(rect(), palette().color(QPalette::Window));
        painter.setPen(Qt::gray);
        painter.drawText(rect(), Qt::AlignCenter, "No video loaded.");
        return;
    }

    QRectF targetRect = calculateTargetRect();
    painter.drawImage(targetRect, currentQImageFrame, currentQImageFrame.rect());

    // Draw active ROI (for DrawROI mode)
    if (m_currentMode == InteractionMode::DrawROI && !m_activeRoiRect.isNull() && m_activeRoiRect.isValid()) {
        QPointF roiTopLeftWidget = mapPointFromVideo(m_activeRoiRect.topLeft());
        QPointF roiBottomRightWidget = mapPointFromVideo(m_activeRoiRect.bottomRight());
        if (roiTopLeftWidget.x() >= 0 && roiBottomRightWidget.x() >= 0) { // Check mapping validity
            painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
            painter.drawRect(QRectF(roiTopLeftWidget, roiBottomRightWidget).normalized());
        }
    }

    // Draw ROI/Crop rectangle being defined
    if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi) {
        painter.setPen(QPen(Qt::cyan, 1, Qt::SolidLine));
        painter.drawRect(QRect(m_roiStartPointWidget, m_roiEndPointWidget).normalized());
    }

    // Draw selected worm centroids and bounding boxes for feedback
    if (m_currentMode == InteractionMode::SelectWorms) {
        // Draw Bounding Boxes First (so centroids are on top)
        painter.setPen(QPen(Qt::yellow, 1, Qt::DotLine)); // Pen for bounding boxes
        for (const QRectF& boundsVideo : m_selectedBounds_temp) {
            QPointF topLeftWidget = mapPointFromVideo(boundsVideo.topLeft());
            QPointF bottomRightWidget = mapPointFromVideo(boundsVideo.bottomRight());
            if (topLeftWidget.x() >=0 && bottomRightWidget.x() >=0) { // Check mapping validity
                painter.drawRect(QRectF(topLeftWidget, bottomRightWidget).normalized());
            }
        }
        // Draw Centroids
        painter.setPen(QPen(Qt::green, 2)); // Pen for centroids
        for (const QPointF& centroidVideo : m_selectedCentroids_temp) {
            QPointF centroidWidget = mapPointFromVideo(centroidVideo);
            if (centroidWidget.x() >= 0) { // Check if mapping is valid
                painter.drawEllipse(centroidWidget, 3, 3); // Small circle for centroid (radius 3)
            }
        }
    }
}

void VideoLoader::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->position();
    if (!isVideoLoaded()) { QWidget::mousePressEvent(event); return; }

    if (event->button() == Qt::LeftButton) {
        if (m_currentMode == InteractionMode::PanZoom) {
            m_isPanning = true;
            updateCursorShape();
            event->accept();
        } else if (m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) {
            QPointF videoCoords = mapPointToVideo(event->position());
            if (videoCoords.x() >= 0) { // Click is on the video
                m_roiStartPointWidget = event->pos();
                m_roiEndPointWidget = event->pos();
                m_isDefiningRoi = true;
                update(); // Repaint to show ROI being drawn
                event->accept();
            } else {
                m_isDefiningRoi = false; // Click was outside video area
            }
        } else if (m_currentMode == InteractionMode::SelectWorms) {
            // m_thresholdedFrame_mono should be up-to-date thanks to displayFrame()
            if (!m_thresholdedFrame_mono.empty()) {
                QPointF clickVideoPoint = mapPointToVideo(event->position());
                if (clickVideoPoint.x() >= 0) { // Click is on the video
                    // Use the helper function to find the blob
                    // Parameters for findClickedBlob can be made configurable if needed
                    TrackingHelper::DetectedBlob blob = TrackingHelper::findClickedBlob(
                        m_thresholdedFrame_mono, clickVideoPoint, 5.0, 30.0);

                    if (blob.isValid) {
                        qDebug() << "VideoLoader: Selected blob centroid:" << blob.centroid << "Bounds:" << blob.boundingBox;
                        // Add to temporary lists for drawing feedback within VideoLoader
                        m_selectedCentroids_temp.append(blob.centroid);
                        m_selectedBounds_temp.append(blob.boundingBox);
                        // Emit signal for MainWindow to collect this selection
                        emit wormBlobSelected(blob.centroid, blob.boundingBox);
                        update(); // Repaint to show new selection marker
                    } else {
                        qDebug() << "VideoLoader: No valid blob found at click point:" << clickVideoPoint;
                    }
                }
            } else {
                qDebug() << "VideoLoader: Cannot select worm, thresholded image is not available or current frame is empty.";
            }
            event->accept();
        }
    } else { // Other mouse buttons
        QWidget::mousePressEvent(event);
    }
}

// mouseMoveEvent, mouseReleaseEvent, wheelEvent, resizeEvent as before
// ... (ensure these functions are present from previous updates) ...
void VideoLoader::mouseMoveEvent(QMouseEvent *event) {
    QPointF currentPos = event->position();
    QPointF delta = currentPos - m_lastMousePos;
    m_lastMousePos = currentPos;
    if (!isVideoLoaded()) { QWidget::mouseMoveEvent(event); return; }
    if (m_isPanning && (event->buttons() & Qt::LeftButton)) {
        m_panOffset += delta; clampPanOffset(); update(); event->accept();
    } else if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi && (event->buttons() & Qt::LeftButton)) {
        m_roiEndPointWidget = event->pos(); update(); event->accept();
    } else { QWidget::mouseMoveEvent(event); }
}
void VideoLoader::mouseReleaseEvent(QMouseEvent *event) {
    if (!isVideoLoaded()) { QWidget::mouseReleaseEvent(event); return; }
    if (event->button() == Qt::LeftButton) {
        if (m_isPanning) {
            m_isPanning = false; updateCursorShape(); event->accept();
        } else if (m_isDefiningRoi && (m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop)) {
            m_roiEndPointWidget = event->pos(); m_isDefiningRoi = false;
            QPointF videoStart = mapPointToVideo(m_roiStartPointWidget);
            QPointF videoEnd = mapPointToVideo(m_roiEndPointWidget);
            QRectF definedRectVideo;
            if(videoStart.x() >= 0 && videoEnd.x() >= 0) {
                definedRectVideo = QRectF(videoStart, videoEnd).normalized();
                if(definedRectVideo.width() < 5 || definedRectVideo.height() < 5) definedRectVideo = QRectF();
            }
            if (m_currentMode == InteractionMode::DrawROI) {
                m_activeRoiRect = definedRectVideo; emit roiDefined(m_activeRoiRect);
            } else if (m_currentMode == InteractionMode::Crop) {
                if (!definedRectVideo.isNull() && definedRectVideo.isValid()) handleRoiDefinedForCrop(definedRectVideo);
                else setInteractionMode(InteractionMode::PanZoom);
            }
            update(); updateCursorShape(); event->accept();
        }
    } else { QWidget::mouseReleaseEvent(event); }
}
void VideoLoader::wheelEvent(QWheelEvent *event) {
    if (!isVideoLoaded() ) { event->ignore(); return; }
    int degrees = event->angleDelta().y() / 8; if (degrees == 0) { event->ignore(); return; }
    int steps = degrees / 15; double zoomSensitivity = 0.15;
    double multiplier = qPow(1.0 + zoomSensitivity, steps);
    setZoomFactorAtPoint(m_zoomFactor * multiplier, event->position());
    event->accept();
}
void VideoLoader::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event); if (isVideoLoaded()) { clampPanOffset(); update(); }
}

// --- Private Helper Methods ---
void VideoLoader::updateCursorShape() {
    if (!isVideoLoaded()) { setCursor(Qt::ArrowCursor); return; }
    switch (m_currentMode) {
    case InteractionMode::PanZoom: setCursor(m_isPanning ? Qt::ClosedHandCursor : Qt::OpenHandCursor); break;
    case InteractionMode::DrawROI: case InteractionMode::Crop: setCursor(Qt::CrossCursor); break;
    case InteractionMode::SelectWorms: setCursor(Qt::PointingHandCursor); break; // Cursor for selection
    default: setCursor(Qt::ArrowCursor); break;
    }
}

// clampPanOffset, handleRoiDefinedForCrop, performVideoCrop, applyThresholding, emitThresholdParametersChanged as before
// ... (ensure these functions are present from previous updates) ...
void VideoLoader::clampPanOffset() {
    if (!isVideoLoaded() || m_zoomFactor <= 0) return;
    QRectF targetRect = calculateTargetRect();
    QSizeF widgetSz = size();
    QPointF currentTopLeftNoPan = targetRect.topLeft() - m_panOffset; // Top-left if pan was zero
    QPointF centerOffset((widgetSz.width() - targetRect.width()) / 2.0, (widgetSz.height() - targetRect.height()) / 2.0);

    if (targetRect.width() <= widgetSz.width()) {
        m_panOffset.setX(centerOffset.x() - currentTopLeftNoPan.x());
    } else {
        qreal minPanX = widgetSz.width() - targetRect.width() - currentTopLeftNoPan.x() - 50 ; // Allow 50px margin
        qreal maxPanX = -currentTopLeftNoPan.x() + 50;
        m_panOffset.setX(qBound(minPanX, m_panOffset.x(), maxPanX));
    }
    if (targetRect.height() <= widgetSz.height()) {
        m_panOffset.setY(centerOffset.y() - currentTopLeftNoPan.y());
    } else {
        qreal minPanY = widgetSz.height() - targetRect.height() - currentTopLeftNoPan.y() - 50;
        qreal maxPanY = -currentTopLeftNoPan.y() + 50;
        m_panOffset.setY(qBound(minPanY, m_panOffset.y(), maxPanY));
    }
}
void VideoLoader::handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords) {
    if (cropRoiVideoCoords.isNull()||!cropRoiVideoCoords.isValid()){ setInteractionMode(InteractionMode::PanZoom); return; }
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Confirm Crop", QString("Crop video to ROI?"), QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        emit videoProcessingStarted("Cropping video..."); QString croppedFilePath;
        bool success = performVideoCrop(cropRoiVideoCoords, croppedFilePath);
        if (success && !croppedFilePath.isEmpty()) {
            emit videoProcessingFinished("Video cropped.",true); loadVideo(croppedFilePath);
        } else {
            emit videoProcessingFinished("Crop failed.",false); QMessageBox::critical(this,"Crop Error","Failed to crop video.");
        }
    }
    setInteractionMode(InteractionMode::PanZoom); m_isDefiningRoi=false; update();
}
bool VideoLoader::performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath) {
    if (currentFilePath.isEmpty()||!videoCapture.isOpened()||cropRectVideoCoords.isNull()||!cropRectVideoCoords.isValid()||cropRectVideoCoords.width()<=0||cropRectVideoCoords.height()<=0) return false;
    cv::VideoCapture origVid; if(!origVid.open(currentFilePath.toStdString())) return false;
    double origFps=origVid.get(cv::CAP_PROP_FPS); if(origFps<=0)origFps=framesPerSecond>0?framesPerSecond:25.0;
    cv::Rect cvCR(static_cast<int>(qRound(cropRectVideoCoords.x())),static_cast<int>(qRound(cropRectVideoCoords.y())), static_cast<int>(qRound(cropRectVideoCoords.width())),static_cast<int>(qRound(cropRectVideoCoords.height())));
    cvCR.x=qMax(0,cvCR.x); cvCR.y=qMax(0,cvCR.y);
    if(originalFrameSize.width()>0&&originalFrameSize.height()>0){ cvCR.width=qMin(cvCR.width,originalFrameSize.width()-cvCR.x); cvCR.height=qMin(cvCR.height,originalFrameSize.height()-cvCR.y); }
    if(cvCR.width<=0||cvCR.height<=0){origVid.release();return false;}
    cv::Size cfs(cvCR.width,cvCR.height);
    QFileInfo ofi(currentFilePath); QString bn=ofi.completeBaseName(), suff=ofi.suffix().isEmpty()?QString(DEFAULT_CROP_EXTENSION).remove(0,1):ofi.suffix();
    outCroppedFilePath = ofi.absolutePath()+"/"+bn+"_cropped."+suff;
    cv::VideoWriter writer; int fourcc = DEFAULT_CROP_FOURCC;
    if(!writer.open(outCroppedFilePath.toStdString(),fourcc,origFps,cfs,true)){origVid.release();return false;}
    cv::Mat frame,croppedF; int fCount=0;
    while(origVid.read(frame)){if(frame.empty())continue; try{croppedF=frame(cvCR);writer.write(croppedF);fCount++;}catch(const cv::Exception&){break;}}
    origVid.release();writer.release(); return fCount>0;
}
void VideoLoader::applyThresholding() {
    if (currentCvFrame.empty()) { m_thresholdedFrame_mono = cv::Mat(); return; }
    cv::Mat grayFrame;
    if (currentCvFrame.channels() == 3 || currentCvFrame.channels() == 4) cv::cvtColor(currentCvFrame, grayFrame, cv::COLOR_BGR2GRAY);
    else grayFrame = currentCvFrame.clone();
    if (m_enableBlur && m_blurKernelSize >= 3) {
        try { cv::GaussianBlur(grayFrame, grayFrame, cv::Size(m_blurKernelSize, m_blurKernelSize), m_blurSigmaX); }
        catch (const cv::Exception& ex) { qWarning() << "OpenCV GaussianBlur Exception:" << ex.what(); }
    }
    int thresholdTypeOpenCV = m_assumeLightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    try {
        switch (m_thresholdAlgorithm) {
        case ThresholdAlgorithm::Global: cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255, thresholdTypeOpenCV); break;
        case ThresholdAlgorithm::Otsu: cv::threshold(grayFrame, m_thresholdedFrame_mono, 0, 255, thresholdTypeOpenCV | cv::THRESH_OTSU); break;
        case ThresholdAlgorithm::AdaptiveMean: if(m_adaptiveBlockSize>=3) cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255, cv::ADAPTIVE_THRESH_MEAN_C, thresholdTypeOpenCV, m_adaptiveBlockSize, m_adaptiveC); else m_thresholdedFrame_mono = cv::Mat(); break;
        case ThresholdAlgorithm::AdaptiveGaussian: if(m_adaptiveBlockSize>=3) cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, thresholdTypeOpenCV, m_adaptiveBlockSize, m_adaptiveC); else m_thresholdedFrame_mono = cv::Mat(); break;
        default: cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255, thresholdTypeOpenCV); break;
        }
    } catch (const cv::Exception& ex) { qWarning() << "OpenCV Thresholding Exception:" << ex.what(); m_thresholdedFrame_mono = cv::Mat(); }
}
void VideoLoader::emitThresholdParametersChanged() { emit thresholdParametersChanged(getCurrentThresholdSettings()); }

// --- Private Helper Methods ---

/**
 * @brief Maps a point from widget coordinates to original video frame coordinates.
 * Takes into account the current zoom factor and pan offset.
 * @param widgetPoint The point in widget coordinates.
 * @return The corresponding point in original video frame coordinates.
 * Returns QPointF(-1, -1) if the mapping is not possible (e.g., video not loaded,
 * widgetPoint is outside the displayed video area, or internal error).
 */
QPointF VideoLoader::mapPointToVideo(const QPointF& widgetPoint) const {
    // Check for invalid states
    if (currentQImageFrame.isNull() || originalFrameSize.isEmpty() || m_zoomFactor <= 0) {
        return QPointF(-1, -1); // Cannot map if no video or invalid zoom
    }

    // Get the rectangle where the video frame is currently drawn on the widget
    QRectF targetRect = calculateTargetRect();

    // Check if targetRect is valid and if the widgetPoint is within this displayed area
    if (!targetRect.isValid() || targetRect.width() <= 0 || targetRect.height() <= 0) {
        return QPointF(-1, -1); // Cannot map if the display area is invalid
    }
    if (!targetRect.contains(widgetPoint)) {
        return QPointF(-1, -1); // Point is outside the currently displayed video area
    }

    // Normalize the widget point to a 0-1 range relative to the targetRect
    // (i.e., where is the click within the displayed image, from 0,0 top-left to 1,1 bottom-right)
    double normalizedX = (widgetPoint.x() - targetRect.left()) / targetRect.width();
    double normalizedY = (widgetPoint.y() - targetRect.top()) / targetRect.height();

    // Scale these normalized coordinates by the original video frame's dimensions
    double videoX = normalizedX * originalFrameSize.width();
    double videoY = normalizedY * originalFrameSize.height();

    // Ensure the calculated video coordinates are within the bounds of the original video frame
    return QPointF(qBound(0.0, videoX, static_cast<qreal>(originalFrameSize.width())),
                   qBound(0.0, videoY, static_cast<qreal>(originalFrameSize.height())));
}

/**
 * @brief Maps a point from original video frame coordinates to widget coordinates.
 * Takes into account the current zoom factor and pan offset.
 * @param videoPoint The point in original video frame coordinates.
 * @return The corresponding point in widget coordinates.
 * Returns QPointF(-1, -1) if the mapping is not possible (e.g., video not loaded,
 * or internal error).
 */
QPointF VideoLoader::mapPointFromVideo(const QPointF& videoPoint) const {
    // Check for invalid states
    if (currentQImageFrame.isNull() || originalFrameSize.isEmpty() || m_zoomFactor <= 0 ||
        originalFrameSize.width() <= 0 || originalFrameSize.height() <= 0) { // Also check originalFrameSize dimensions
        return QPointF(-1, -1); // Cannot map if no video or invalid dimensions/zoom
    }

    // Get the rectangle where the video frame is currently drawn on the widget
    QRectF targetRect = calculateTargetRect();

    // Check if targetRect is valid
    if (!targetRect.isValid() || targetRect.width() <= 0 || targetRect.height() <= 0) {
        return QPointF(-1, -1); // Cannot map if the display area is invalid
    }

    // Clamp the input videoPoint to be within the actual video frame dimensions
    // This prevents issues if an out-of-bounds videoPoint is provided.
    QPointF clampedVideoPoint(
        qBound(0.0, videoPoint.x(), static_cast<qreal>(originalFrameSize.width())),
        qBound(0.0, videoPoint.y(), static_cast<qreal>(originalFrameSize.height()))
        );

    // Normalize the (clamped) video point to a 0-1 range relative to the originalFrameSize
    double normalizedX = clampedVideoPoint.x() / originalFrameSize.width();
    double normalizedY = clampedVideoPoint.y() / originalFrameSize.height();

    // Scale these normalized coordinates by the targetRect's dimensions and add its top-left offset
    double widgetX = targetRect.left() + normalizedX * targetRect.width();
    double widgetY = targetRect.top() + normalizedY * targetRect.height();

    return QPointF(widgetX, widgetY);
}

/**
 * @brief Calculates the rectangle on the widget where the video frame is currently displayed.
 * This takes into account the widget's aspect ratio, the video's aspect ratio,
 * the current zoom factor, and the pan offset.
 * @return QRectF representing the target display area in widget coordinates.
 * Returns an invalid QRectF if video is not loaded or zoom factor is invalid.
 */
QRectF VideoLoader::calculateTargetRect() const {
    // If no video is loaded, or original frame size is invalid, or zoom is non-positive, return an empty rect
    if (originalFrameSize.isEmpty() || m_zoomFactor <= 0) {
        return QRectF(); // Invalid or empty rectangle
    }

    QSizeF widgetSz = size(); // Current size of the VideoLoader widget
    QSizeF imageSz = originalFrameSize; // Original size of the video frame

    // 1. Scale the image to fit within the widget while maintaining aspect ratio
    // This gives the size of the image if it were displayed at 100% zoom, fitting the widget.
    QSizeF scaledImageSize = imageSz;
    scaledImageSize.scale(widgetSz, Qt::KeepAspectRatio);

    // 2. Apply the zoom factor to this scaled size
    QSizeF zoomedDisplaySize = scaledImageSize * m_zoomFactor;

    // 3. Calculate the top-left position of this zoomed image so it's centered in the widget
    // (before applying any pan)
    QPointF centeredTopLeft(
        (widgetSz.width() - zoomedDisplaySize.width()) / 2.0,
        (widgetSz.height() - zoomedDisplaySize.height()) / 2.0
        );

    // 4. Apply the pan offset to this centered position
    QPointF finalTopLeft = centeredTopLeft + m_panOffset;

    // The target rectangle is defined by this final top-left position and the zoomed display size
    return QRectF(finalTopLeft, zoomedDisplaySize);
}
