#include "VideoLoader.h"
#include <QDebug>
#include <QPainterPath>
#include <QtMath> // For qBound, qFuzzyCompare, qRound
#include <QResizeEvent>
#include <QMessageBox>
#include <QFileInfo>

// Default crop parameters from user's context
#define DEFAULT_CROP_EXTENSION ".mp4"
#define DEFAULT_CROP_FOURCC cv::VideoWriter::fourcc('H', '2', '6', '4')
// Or use 'M', 'P', '4', 'V' for MP4 on some systems, or 'a', 'v', 'c', '1'
// For AVI: cv::VideoWriter::fourcc('M', 'J', 'P', 'G')

VideoLoader::VideoLoader(QWidget *parent)
    : QWidget(parent),
    playbackTimer(new QTimer(this)),
    totalFramesCount(0),
    framesPerSecond(0.0), // This is the ORIGINAL video FPS
    currentFrameIdx(-1),
    m_isPlaying(false),
    m_playbackSpeedMultiplier(1.0), // Default to normal speed
    m_currentMode(InteractionMode::PanZoom),
    m_isPanning(false),
    m_isDefiningRoi(false),
    m_zoomFactor(1.0),
    m_panOffset(0.0, 0.0),
    // Initialize thresholding members
    m_showThresholdMask(false),
    m_thresholdAlgorithm(ThresholdAlgorithm::Global), // Default algorithm
    m_thresholdValue(127),
    m_assumeLightBackground(true), // Assume light background, dark objects
    m_adaptiveBlockSize(11),       // Default for adaptive, must be odd
    m_adaptiveC(2.0)               // Default for adaptive
{
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);

    connect(playbackTimer, &QTimer::timeout, this, &VideoLoader::processNextFrame);
    setMouseTracking(true);
    updateCursorShape();
}

VideoLoader::~VideoLoader() {
    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
}

// --- Public Methods ---
bool VideoLoader::isVideoLoaded() const { return videoCapture.isOpened() && totalFramesCount > 0; }
int VideoLoader::getTotalFrames() const { return totalFramesCount; }
double VideoLoader::getFPS() const { return framesPerSecond; } // Returns ORIGINAL video FPS
int VideoLoader::getCurrentFrameNumber() const { return currentFrameIdx; }
QSize VideoLoader::getVideoFrameSize() const { return originalFrameSize; }
double VideoLoader::getZoomFactor() const { return m_zoomFactor; }
QRectF VideoLoader::getCurrentRoi() const { return m_activeRoiRect; }
InteractionMode VideoLoader::getCurrentInteractionMode() const { return m_currentMode; }
double VideoLoader::getPlaybackSpeed() const { return m_playbackSpeedMultiplier; } // Getter for current speed multiplier

bool VideoLoader::isThresholdViewEnabled() const { return m_showThresholdMask; }
ThresholdAlgorithm VideoLoader::getCurrentThresholdAlgorithm() const { return m_thresholdAlgorithm; }
int VideoLoader::getThresholdValue() const { return m_thresholdValue; }
bool VideoLoader::getAssumeLightBackground() const { return m_assumeLightBackground; }
int VideoLoader::getAdaptiveBlockSize() const { return m_adaptiveBlockSize; }
double VideoLoader::getAdaptiveCValue() const { return m_adaptiveC; }


// --- Slots ---
bool VideoLoader::loadVideo(const QString &filePath) {
    if (m_isPlaying) {
        pause();
    }

    // Reset states for new video
    m_zoomFactor = 1.0;
    m_panOffset = QPointF(0.0, 0.0);
    m_activeRoiRect = QRectF();
    m_isPanning = false;
    m_isDefiningRoi = false;
    m_playbackSpeedMultiplier = 1.0; // Reset to normal speed on new video load
    // m_showThresholdMask = false; // Optionally reset threshold view

    if (!openVideoFile(filePath)) {
        currentFilePath.clear(); totalFramesCount = 0; framesPerSecond = 0.0; currentFrameIdx = -1;
        originalFrameSize = QSize(); currentQImageFrame = QImage(); m_thresholdedFrame_mono = cv::Mat();
        update(); updateCursorShape();
        emit videoLoadFailed(filePath, "Failed to open video file with OpenCV.");
        return false;
    }

    currentFilePath = filePath;
    framesPerSecond = videoCapture.get(cv::CAP_PROP_FPS); // Store original FPS
    if (framesPerSecond <= 0) framesPerSecond = 25.0; // Default if not available

    totalFramesCount = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    originalFrameSize = QSize(static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH)),
                              static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT)));

    if (totalFramesCount <= 0 || originalFrameSize.isEmpty()) {
        emit videoLoadFailed(filePath, "Video file seems empty or metadata is corrupted.");
        videoCapture.release(); currentFilePath.clear();
        update(); updateCursorShape();
        return false;
    }

    seekToFrame(0);

    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor);
    emit roiDefined(m_activeRoiRect);
    emit playbackSpeedChanged(m_playbackSpeedMultiplier); // Emit initial speed
    updateCursorShape();
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
        qWarning() << "OpenCV exception while opening video:" << filePath << "Error:" << ex.what();
        return false;
    }
    return videoCapture.isOpened();
}

void VideoLoader::updateTimerInterval() {
    if (framesPerSecond > 0 && m_playbackSpeedMultiplier > 0) {
        int interval = qRound(1000.0 / (framesPerSecond * m_playbackSpeedMultiplier));
        playbackTimer->setInterval(qMax(1, interval)); // Ensure interval is at least 1ms
    } else {
        playbackTimer->setInterval(40); // Default interval (25 FPS) if data is invalid
    }
}

void VideoLoader::play() {
    if (!isVideoLoaded()) {
        return;
    }
    if (m_isPlaying) {
        pause();
    } else {
        if (currentFrameIdx >= totalFramesCount - 1 && totalFramesCount > 0) {
            seekToFrame(0);
            if (currentFrameIdx == -1 && totalFramesCount > 0) return;
        }
        m_isPlaying = true;
        updateTimerInterval(); // Set interval based on current speed
        playbackTimer->start();
        emit playbackStateChanged(true, m_playbackSpeedMultiplier);
        qDebug() << "Playback started. Speed:" << m_playbackSpeedMultiplier << "x";
    }
}

void VideoLoader::pause() {
    if (!m_isPlaying && !playbackTimer->isActive()) return;
    m_isPlaying = false;
    playbackTimer->stop();
    emit playbackStateChanged(false, m_playbackSpeedMultiplier);
    qDebug() << "Playback paused.";
}

void VideoLoader::setPlaybackSpeed(double multiplier) {
    // Basic validation for multiplier, adjust range as needed
    // e.g., 0.1x to 10x
    double newSpeed = qBound(0.1, multiplier, 10.0);

    if (qFuzzyCompare(m_playbackSpeedMultiplier, newSpeed)) {
        return; // No change
    }
    m_playbackSpeedMultiplier = newSpeed;
    qDebug() << "Playback speed set to:" << m_playbackSpeedMultiplier << "x";

    if (m_isPlaying) {
        // If playing, stop and restart timer with new interval
        playbackTimer->stop();
        updateTimerInterval();
        playbackTimer->start();
    } else {
        // If paused, just update the interval for when play is next called
        // (updateTimerInterval() will be called in play())
    }
    emit playbackSpeedChanged(m_playbackSpeedMultiplier);
    // Also emit playbackStateChanged if you want the UI to update the speed display even when paused
    emit playbackStateChanged(m_isPlaying, m_playbackSpeedMultiplier);
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
        currentFrameIdx = -1; currentQImageFrame = QImage(); m_thresholdedFrame_mono = cv::Mat(); update(); return;
    }
    if (frameNumber < 0 || frameNumber >= totalFramesCount) {
        // This case should ideally be prevented by qBound in seekToFrame or processNextFrame logic
        qWarning() << "Attempted to display invalid frame number:" << frameNumber;
        if (m_isPlaying) pause(); // Stop playback if we hit an invalid frame
        currentFrameIdx = -1; currentQImageFrame = QImage(); m_thresholdedFrame_mono = cv::Mat(); update(); return;
    }

    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    // Only set if significantly different or if it's not the next frame in sequence
    // (videoCapture.set is expensive)
    if (currentPos != frameNumber && !(currentPos == frameNumber -1 && m_isPlaying) ) {
        if(!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) {
            qWarning() << "Failed to set frame position to" << frameNumber;
            // Consider this a critical error that might require stopping playback or reloading
        }
    }

    if (videoCapture.read(currentCvFrame)) {
        if (!currentCvFrame.empty()) {
            currentFrameIdx = frameNumber;
            if (m_showThresholdMask) {
                applyThresholding();
            }
        } else {
            currentCvFrame = cv::Mat();
            m_thresholdedFrame_mono = cv::Mat();
            qWarning() << "Read empty frame at index" << frameNumber;
        }
    } else {
        currentCvFrame = cv::Mat();
        m_thresholdedFrame_mono = cv::Mat();
        qWarning() << "Failed to read frame at index" << frameNumber;
        if (m_isPlaying) pause();
    }

    if (m_showThresholdMask && !m_thresholdedFrame_mono.empty()) {
        convertCvMatToQImage(m_thresholdedFrame_mono, currentQImageFrame);
    } else if (!currentCvFrame.empty()) {
        convertCvMatToQImage(currentCvFrame, currentQImageFrame);
    } else {
        currentQImageFrame = QImage();
    }

    // currentFrameIdx = frameNumber; // Already set if frame read successfully

    if (!suppressEmit) {
        emit frameChanged(currentFrameIdx, currentQImageFrame);
    }
    update();
}


void VideoLoader::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) {
    if (mat.empty()) { qimg = QImage(); return; }
    if (mat.type() == CV_8UC3) { // BGR format from OpenCV
        qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped();
    } else if (mat.type() == CV_8UC1) { // Grayscale
        qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8);
    } else {
        cv::Mat temp;
        try {
            if (mat.channels() == 4) { // BGRA to BGR then to QImage
                cv::cvtColor(mat, temp, cv::COLOR_BGRA2BGR);
                qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped();
            } else if (mat.channels() == 3 && mat.type() != CV_8UC3) { // e.g. CV_32FC3
                mat.convertTo(temp, CV_8UC3, 255.0); // Basic scaling
                qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped();
            } else if (mat.channels() == 1 && mat.type() != CV_8UC1) { // e.g. CV_32FC1
                mat.convertTo(temp, CV_8UC1, 255.0); // Basic scaling
                qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_Grayscale8);
            } else {
                qWarning() << "Unsupported cv::Mat type for conversion:" << mat.type();
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
    if (!m_isPlaying || !isVideoLoaded()) return;
    if (currentFrameIdx < totalFramesCount - 1) {
        displayFrame(currentFrameIdx + 1);
    } else {
        pause(); // Reached end of video
    }
}

// --- Zoom, Pan, ROI, Interaction Mode methods (largely unchanged) ---
void VideoLoader::setZoomFactor(double factor) { setZoomFactorAtPoint(factor, rect().center()); }
void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) {
    if (!isVideoLoaded()) return;
    double newZoomFactor = qBound(0.05, factor, 50.0); // Min 5%, Max 5000% zoom
    if (qFuzzyCompare(m_zoomFactor, newZoomFactor)) return;

    QPointF videoPointBeforeZoom = mapPointToVideo(widgetPoint);

    m_zoomFactor = newZoomFactor;

    // If the reference point was outside the video before zoom, or video not valid,
    // we might need to adjust pan differently or simply center.
    // For simplicity, if videoPointBeforeZoom is invalid, we might pan relative to center.
    if (videoPointBeforeZoom.x() < 0 || videoPointBeforeZoom.y() < 0) {
        // Center view if mapped point is invalid (e.g. zooming out when panned far)
        QSizeF ws = size();
        QSizeF videoDisplaySize = originalFrameSize * m_zoomFactor;
        videoDisplaySize.scale(ws, Qt::KeepAspectRatio); // This should be calculateTargetRect().size()
        m_panOffset.setX((ws.width() - videoDisplaySize.width()) / 2.0);
        m_panOffset.setY((ws.height() - videoDisplaySize.height()) / 2.0);
    } else {
        QPointF widgetPointAfterZoom = mapPointFromVideo(videoPointBeforeZoom);
        m_panOffset += (widgetPoint - widgetPointAfterZoom);
    }

    clampPanOffset();
    update();
    emit zoomFactorChanged(m_zoomFactor);
}

void VideoLoader::setInteractionMode(InteractionMode mode) {
    if (m_currentMode == mode) return;
    m_currentMode = mode;
    m_isPanning = false;
    m_isDefiningRoi = false;
    updateCursorShape();
    emit interactionModeChanged(m_currentMode);
    qDebug() << "Interaction mode set to:" << static_cast<int>(m_currentMode);
}

void VideoLoader::clearRoi() {
    if (!m_activeRoiRect.isNull()) {
        m_activeRoiRect = QRectF();
        update();
        emit roiDefined(m_activeRoiRect);
        qDebug() << "Active ROI cleared.";
    }
}

// --- Thresholding Control Slots (unchanged) ---
void VideoLoader::toggleThresholdView(bool enabled) {
    if (m_showThresholdMask == enabled) return;
    m_showThresholdMask = enabled;
    qDebug() << "Threshold view" << (enabled ? "enabled" : "disabled");
    if (isVideoLoaded() && currentFrameIdx >= 0) {
        displayFrame(currentFrameIdx, true);
    }
    update();
}

void VideoLoader::setThresholdAlgorithm(ThresholdAlgorithm algorithm) {
    if (m_thresholdAlgorithm == algorithm) return;
    m_thresholdAlgorithm = algorithm;
    qDebug() << "Threshold algorithm set to:" << static_cast<int>(algorithm);
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) {
        displayFrame(currentFrameIdx, true);
    }
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC);
    update();
}

void VideoLoader::setThresholdValue(int value) {
    value = qBound(0, value, 255);
    if (m_thresholdValue == value) return;
    m_thresholdValue = value;
    qDebug() << "Threshold value set to:" << value;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && m_thresholdAlgorithm == ThresholdAlgorithm::Global) {
        displayFrame(currentFrameIdx, true);
    }
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC);
    update();
}

void VideoLoader::setAssumeLightBackground(bool isLight) {
    if (m_assumeLightBackground == isLight) return;
    m_assumeLightBackground = isLight;
    qDebug() << "Assume light background set to:" << isLight;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) {
        displayFrame(currentFrameIdx, true);
    }
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC);
    update();
}

void VideoLoader::setAdaptiveThresholdBlockSize(int blockSize) {
    if (blockSize < 3) blockSize = 3;
    if (blockSize % 2 == 0) blockSize += 1;
    if (m_adaptiveBlockSize == blockSize) return;
    m_adaptiveBlockSize = blockSize;
    qDebug() << "Adaptive block size set to:" << blockSize;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 &&
        (m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveMean || m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveGaussian)) {
        displayFrame(currentFrameIdx, true);
    }
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC);
    update();
}

void VideoLoader::setAdaptiveThresholdC(double cValue) {
    if (qFuzzyCompare(m_adaptiveC, cValue)) return;
    m_adaptiveC = cValue;
    qDebug() << "Adaptive C value set to:" << cValue;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 &&
        (m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveMean || m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveGaussian)) {
        displayFrame(currentFrameIdx, true);
    }
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC);
    update();
}


// --- Event Handlers (largely unchanged) ---
void VideoLoader::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    if (currentQImageFrame.isNull() || !isVideoLoaded()) {
        painter.fillRect(rect(), palette().color(QPalette::Window)); // Use QPalette::Window for background
        painter.setPen(Qt::gray);
        painter.drawText(rect(), Qt::AlignCenter, "No video loaded.");
        return;
    }

    QRectF targetRect = calculateTargetRect();
    painter.drawImage(targetRect, currentQImageFrame, currentQImageFrame.rect());

    if (m_currentMode == InteractionMode::DrawROI && !m_activeRoiRect.isNull() && m_activeRoiRect.isValid()) {
        QPointF roiTopLeftVideo = m_activeRoiRect.topLeft();
        QPointF roiBottomRightVideo = m_activeRoiRect.bottomRight();
        QPointF roiTopLeftWidget = mapPointFromVideo(roiTopLeftVideo);
        QPointF roiBottomRightWidget = mapPointFromVideo(roiBottomRightVideo);

        if (roiTopLeftWidget.x() >= 0 && roiBottomRightWidget.x() >= 0) {
            QRectF roiWidgetRect(roiTopLeftWidget, roiBottomRightWidget);
            painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
            painter.setBrush(Qt::NoBrush);
            painter.drawRect(roiWidgetRect.normalized());
        }
    }

    if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi) {
        painter.setPen(QPen(Qt::cyan, 1, Qt::SolidLine));
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(QRect(m_roiStartPointWidget, m_roiEndPointWidget).normalized());
    }
}

QRectF VideoLoader::calculateTargetRect() const {
    if (originalFrameSize.isEmpty() || m_zoomFactor <= 0) return QRectF();

    QSizeF widgetSize = size();
    QSizeF imageSize = originalFrameSize;

    // Scale image to fit widget while maintaining aspect ratio
    QSizeF scaledImageSize = imageSize;
    scaledImageSize.scale(widgetSize, Qt::KeepAspectRatio);

    // Apply zoom factor
    QSizeF zoomedSize = scaledImageSize * m_zoomFactor;

    // Calculate top-left point to center the zoomed image
    QPointF topLeft((widgetSize.width() - zoomedSize.width()) / 2.0,
                    (widgetSize.height() - zoomedSize.height()) / 2.0);

    // Apply pan offset
    topLeft += m_panOffset;

    return QRectF(topLeft, zoomedSize);
}

QPointF VideoLoader::mapPointToVideo(const QPointF& widgetPoint) const {
    if (currentQImageFrame.isNull() || originalFrameSize.isEmpty() || m_zoomFactor <= 0) return QPointF(-1, -1);

    QRectF targetRect = calculateTargetRect();
    if (!targetRect.isValid() || targetRect.width() <= 0 || targetRect.height() <= 0) return QPointF(-1, -1);
    if (!targetRect.contains(widgetPoint)) return QPointF(-1,-1); // Point is outside the displayed image

    // Normalize widget point to 0-1 range within targetRect
    double normX = (widgetPoint.x() - targetRect.left()) / targetRect.width();
    double normY = (widgetPoint.y() - targetRect.top()) / targetRect.height();

    // Map normalized point to original video frame coordinates
    double videoX = normX * originalFrameSize.width();
    double videoY = normY * originalFrameSize.height();

    return QPointF(qBound(0.0, videoX, static_cast<qreal>(originalFrameSize.width())),
                   qBound(0.0, videoY, static_cast<qreal>(originalFrameSize.height())));
}

QPointF VideoLoader::mapPointFromVideo(const QPointF& videoPoint) const {
    if (currentQImageFrame.isNull() || originalFrameSize.isEmpty() || m_zoomFactor <= 0 ||
        originalFrameSize.width() <= 0 || originalFrameSize.height() <= 0) return QPointF(-1, -1);

    QRectF targetRect = calculateTargetRect();
    if (!targetRect.isValid() || targetRect.width() <= 0 || targetRect.height() <= 0) return QPointF(-1, -1);

    // Clamp video point to be within original frame dimensions
    QPointF clampedVideoPoint(qBound(0.0, videoPoint.x(), static_cast<qreal>(originalFrameSize.width())),
                              qBound(0.0, videoPoint.y(), static_cast<qreal>(originalFrameSize.height())));

    // Normalize video point to 0-1 range within originalFrameSize
    double normX = clampedVideoPoint.x() / originalFrameSize.width();
    double normY = clampedVideoPoint.y() / originalFrameSize.height();

    // Map normalized point to widget coordinates within targetRect
    double widgetX = targetRect.left() + normX * targetRect.width();
    double widgetY = targetRect.top() + normY * targetRect.height();

    return QPointF(widgetX, widgetY);
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
            if (videoCoords.x() >= 0) { // Check if click is within displayed video
                m_roiStartPointWidget = event->pos();
                m_roiEndPointWidget = event->pos();
                m_isDefiningRoi = true;
                update();
                qDebug() << (m_currentMode == InteractionMode::Crop ? "Crop" : "ROI") << "definition started at widget" << m_roiStartPointWidget;
                event->accept();
            } else {
                m_isDefiningRoi = false;
            }
        }
    } else {
        QWidget::mousePressEvent(event);
    }
}

void VideoLoader::mouseMoveEvent(QMouseEvent *event) {
    QPointF currentPos = event->position();
    QPointF delta = currentPos - m_lastMousePos;
    m_lastMousePos = currentPos; // Update last mouse position for next event

    if (!isVideoLoaded()) { QWidget::mouseMoveEvent(event); return; }

    if (m_isPanning && (event->buttons() & Qt::LeftButton)) {
        m_panOffset += delta;
        clampPanOffset();
        update();
        event->accept();
    } else if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi && (event->buttons() & Qt::LeftButton)) {
        m_roiEndPointWidget = event->pos();
        update();
        event->accept();
    } else {
        QWidget::mouseMoveEvent(event);
    }
}

void VideoLoader::mouseReleaseEvent(QMouseEvent *event) {
    if (!isVideoLoaded()) { QWidget::mouseReleaseEvent(event); return; }

    if (event->button() == Qt::LeftButton) {
        if (m_isPanning) {
            m_isPanning = false;
            updateCursorShape();
            event->accept();
        } else if (m_isDefiningRoi && (m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop)) {
            m_roiEndPointWidget = event->pos();
            m_isDefiningRoi = false;

            QPointF videoStart = mapPointToVideo(m_roiStartPointWidget);
            QPointF videoEnd = mapPointToVideo(m_roiEndPointWidget);
            QRectF definedRectVideo;

            if(videoStart.x() >= 0 && videoEnd.x() >= 0) { // Both points mapped successfully
                definedRectVideo = QRectF(videoStart, videoEnd).normalized();
                // Add a minimum size check for the ROI in video coordinates
                if(definedRectVideo.width() < 5 || definedRectVideo.height() < 5) { // Minimum 5x5 pixels in video
                    definedRectVideo = QRectF(); // Invalidate if too small
                }
            } else {
                qDebug() << "ROI definition failed: points could not be mapped to video.";
            }

            if (m_currentMode == InteractionMode::DrawROI) {
                m_activeRoiRect = definedRectVideo;
                emit roiDefined(m_activeRoiRect);
                qDebug() << "ROI selection ended. Video_rect:" << m_activeRoiRect;
            } else if (m_currentMode == InteractionMode::Crop) {
                if (!definedRectVideo.isNull() && definedRectVideo.isValid()) {
                    handleRoiDefinedForCrop(definedRectVideo);
                } else {
                    qDebug() << "Crop area invalid or too small.";
                    setInteractionMode(InteractionMode::PanZoom); // Revert mode if crop invalid
                }
            }
            update();
            updateCursorShape();
            event->accept();
        }
    } else {
        QWidget::mouseReleaseEvent(event);
    }
}

void VideoLoader::wheelEvent(QWheelEvent *event) {
    if (!isVideoLoaded() || m_currentMode != InteractionMode::PanZoom) {
        event->ignore();
        return;
    }

    int degrees = event->angleDelta().y() / 8;
    if (degrees == 0) { event->ignore(); return; }
    int steps = degrees / 15; // Standard step for mouse wheel

    // Define zoom sensitivity, e.g., 15% per step
    double zoomSensitivity = 0.15;
    double multiplier = qPow(1.0 + zoomSensitivity, steps);

    setZoomFactorAtPoint(m_zoomFactor * multiplier, event->position());
    event->accept();
}

void VideoLoader::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    if (isVideoLoaded()) {
        clampPanOffset(); // Recalculate pan limits based on new widget size
        update();
    }
}

// --- Private Helper Methods ---
void VideoLoader::updateCursorShape() {
    if (!isVideoLoaded()) { setCursor(Qt::ArrowCursor); return; }
    switch (m_currentMode) {
    case InteractionMode::PanZoom: setCursor(m_isPanning ? Qt::ClosedHandCursor : Qt::OpenHandCursor); break;
    case InteractionMode::DrawROI: case InteractionMode::Crop: setCursor(Qt::CrossCursor); break;
    default: setCursor(Qt::ArrowCursor); break;
    }
}

void VideoLoader::clampPanOffset() {
    if (!isVideoLoaded() || m_zoomFactor <= 0) return;

    QRectF targetRect = calculateTargetRect(); // This is the rect of the image as displayed
    QSizeF widgetSz = size();

    // If image is smaller than widget, center it (panOffset should be effectively (0,0) relative to centered pos)
    if (targetRect.width() <= widgetSz.width()) {
        m_panOffset.setX((widgetSz.width() - targetRect.width()) / 2.0 - targetRect.left() + m_panOffset.x());
    } else {
        // Limit panning so image edges don't go past widget edges too much (e.g. 50px margin)
        qreal minX = widgetSz.width() - targetRect.width() - (targetRect.left() - m_panOffset.x()) - 50 ;
        qreal maxX = -(targetRect.left() - m_panOffset.x()) + 50;
        m_panOffset.setX(qBound(minX, m_panOffset.x(), maxX));
    }

    if (targetRect.height() <= widgetSz.height()) {
        m_panOffset.setY((widgetSz.height() - targetRect.height()) / 2.0 - targetRect.top() + m_panOffset.y());
    } else {
        qreal minY = widgetSz.height() - targetRect.height() - (targetRect.top() - m_panOffset.y()) - 50;
        qreal maxY = -(targetRect.top() - m_panOffset.y()) + 50;
        m_panOffset.setY(qBound(minY, m_panOffset.y(), maxY));
    }
}


void VideoLoader::handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords) {
    if (cropRoiVideoCoords.isNull()||!cropRoiVideoCoords.isValid()){
        setInteractionMode(InteractionMode::PanZoom);
        return;
    }
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Confirm Crop",
                                                              QString("Crop video to selected ROI (X:%1 Y:%2 W:%3 H:%4)?")
                                                                  .arg(cropRoiVideoCoords.x()).arg(cropRoiVideoCoords.y())
                                                                  .arg(cropRoiVideoCoords.width()).arg(cropRoiVideoCoords.height()),
                                                              QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        emit videoProcessingStarted("Cropping video...");
        QString croppedFilePath;
        bool success = performVideoCrop(cropRoiVideoCoords, croppedFilePath);
        if (success && !croppedFilePath.isEmpty()) {
            emit videoProcessingFinished("Video cropped successfully to: " + croppedFilePath, true);
            loadVideo(croppedFilePath); // Load the newly cropped video
        } else {
            emit videoProcessingFinished("Crop failed.", false);
            QMessageBox::critical(this,"Crop Error","Failed to crop video. Ensure selection is valid and disk space is available.");
        }
    }
    setInteractionMode(InteractionMode::PanZoom);
    m_isDefiningRoi=false;
    update();
}

bool VideoLoader::performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath) {
    if (currentFilePath.isEmpty() || !videoCapture.isOpened() ||
        cropRectVideoCoords.isNull() || !cropRectVideoCoords.isValid() ||
        cropRectVideoCoords.width() <=0 || cropRectVideoCoords.height() <=0) {
        qWarning() << "performVideoCrop: Invalid parameters for cropping.";
        return false;
    }

    cv::VideoCapture originalVid;
    if(!originalVid.open(currentFilePath.toStdString())) {
        qWarning() << "performVideoCrop: Failed to reopen original video for cropping.";
        return false;
    }

    double origFps = originalVid.get(cv::CAP_PROP_FPS);
    if(origFps <= 0) origFps = framesPerSecond > 0 ? framesPerSecond : 25.0; // Use current FPS or default

    // Ensure CV Rect is within original frame dimensions
    cv::Rect cvCropRect(static_cast<int>(qRound(cropRectVideoCoords.x())),
                        static_cast<int>(qRound(cropRectVideoCoords.y())),
                        static_cast<int>(qRound(cropRectVideoCoords.width())),
                        static_cast<int>(qRound(cropRectVideoCoords.height())));

    cvCropRect.x = qMax(0, cvCropRect.x);
    cvCropRect.y = qMax(0, cvCropRect.y);
    if (originalFrameSize.width() > 0 && originalFrameSize.height() > 0) { // Check if originalFrameSize is valid
        cvCropRect.width = qMin(cvCropRect.width, originalFrameSize.width() - cvCropRect.x);
        cvCropRect.height = qMin(cvCropRect.height, originalFrameSize.height() - cvCropRect.y);
    } else {
        qWarning() << "performVideoCrop: Original frame size unknown, cannot validate crop rect fully.";
        // Proceed with caution if originalFrameSize is not set
    }


    if(cvCropRect.width <= 0 || cvCropRect.height <= 0){
        qWarning() << "performVideoCrop: Crop rectangle has zero or negative dimensions after clamping.";
        originalVid.release();
        return false;
    }
    cv::Size croppedFrameSize(cvCropRect.width, cvCropRect.height);

    QFileInfo originalFileInfo(currentFilePath);
    QString baseName = originalFileInfo.completeBaseName();
    QString suffix = originalFileInfo.suffix().isEmpty() ? QString(DEFAULT_CROP_EXTENSION).remove(0,1) : originalFileInfo.suffix();
    outCroppedFilePath = originalFileInfo.absolutePath() + "/" + baseName + "_cropped." + suffix;

    cv::VideoWriter writer;
    int fourcc = DEFAULT_CROP_FOURCC; // Use defined FOURCC

    if(!writer.open(outCroppedFilePath.toStdString(), fourcc, origFps, croppedFrameSize, true)){ // true for isColor
        qWarning() << "performVideoCrop: Failed to open VideoWriter for path:" << outCroppedFilePath
                   << "FOURCC:" << fourcc << "FPS:" << origFps << "Size:" << croppedFrameSize.width << "x" << croppedFrameSize.height;
        originalVid.release();
        return false;
    }

    cv::Mat frame, croppedFrame;
    int framesWrittenCount = 0;
    long totalFramesToRead = static_cast<long>(originalVid.get(cv::CAP_PROP_FRAME_COUNT));
    long currentFrameRead = 0;

    qDebug() << "Starting crop. Total frames to process:" << totalFramesToRead;

    while(originalVid.read(frame)){
        currentFrameRead++;
        if(frame.empty()){
            qWarning() << "performVideoCrop: Read empty frame at" << currentFrameRead-1;
            continue;
        }
        try{
            croppedFrame = frame(cvCropRect); // Perform the crop
            writer.write(croppedFrame);
            framesWrittenCount++;
        } catch(const cv::Exception& ex) {
            qWarning() << "performVideoCrop: OpenCV exception during cropping/writing frame" << currentFrameRead-1 << ":" << ex.what();
            break; // Stop processing on error
        }
        if (currentFrameRead % 100 == 0) { // Log progress
            qDebug() << "Cropping progress:" << currentFrameRead << "/" << totalFramesToRead;
        }
    }
    qDebug() << "Finished cropping. Frames written:" << framesWrittenCount;

    originalVid.release();
    writer.release();
    return framesWrittenCount > 0; // Success if at least one frame was written
}

// New private method to apply thresholding
void VideoLoader::applyThresholding() {
    if (currentCvFrame.empty()) {
        m_thresholdedFrame_mono = cv::Mat();
        return;
    }

    cv::Mat grayFrame;
    if (currentCvFrame.channels() == 3 || currentCvFrame.channels() == 4) {
        cv::cvtColor(currentCvFrame, grayFrame, cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = currentCvFrame.clone();
    }

    // --- Apply Gaussian Blur if enabled (from ThresholdSettings) ---
    // This part needs to be updated if ThresholdSettings struct is passed or member variables are used
    // For now, assuming fixed blur from original context or no blur if not specified
    // cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5, 5), 0); // Example fixed blur

    int thresholdTypeOpenCV = m_assumeLightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;

    switch (m_thresholdAlgorithm) {
    case ThresholdAlgorithm::Global:
        cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255, thresholdTypeOpenCV);
        break;
    case ThresholdAlgorithm::Otsu:
        cv::threshold(grayFrame, m_thresholdedFrame_mono, 0, 255, thresholdTypeOpenCV | cv::THRESH_OTSU);
        break;
    case ThresholdAlgorithm::AdaptiveMean:
        cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255,
                              cv::ADAPTIVE_THRESH_MEAN_C, thresholdTypeOpenCV,
                              m_adaptiveBlockSize, m_adaptiveC);
        break;
    case ThresholdAlgorithm::AdaptiveGaussian:
        cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255,
                              cv::ADAPTIVE_THRESH_GAUSSIAN_C, thresholdTypeOpenCV,
                              m_adaptiveBlockSize, m_adaptiveC);
        break;
    default:
        cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255, thresholdTypeOpenCV);
        break;
    }
}

