#include "VideoLoader.h"
#include <QDebug>
#include <QPainterPath>
#include <QtMath>
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
    framesPerSecond(0.0),
    currentFrameIdx(-1),
    m_isPlaying(false),
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
double VideoLoader::getFPS() const { return framesPerSecond; }
int VideoLoader::getCurrentFrameNumber() const { return currentFrameIdx; }
QSize VideoLoader::getVideoFrameSize() const { return originalFrameSize; }
double VideoLoader::getZoomFactor() const { return m_zoomFactor; }
QRectF VideoLoader::getCurrentRoi() const { return m_activeRoiRect; }
InteractionMode VideoLoader::getCurrentInteractionMode() const { return m_currentMode; }
bool VideoLoader::isThresholdViewEnabled() const { return m_showThresholdMask; }
ThresholdAlgorithm VideoLoader::getCurrentThresholdAlgorithm() const { return m_thresholdAlgorithm; }
int VideoLoader::getThresholdValue() const { return m_thresholdValue; }
bool VideoLoader::getAssumeLightBackground() const { return m_assumeLightBackground; }
int VideoLoader::getAdaptiveBlockSize() const { return m_adaptiveBlockSize; }
double VideoLoader::getAdaptiveCValue() const { return m_adaptiveC; }


// --- Slots ---
bool VideoLoader::loadVideo(const QString &filePath) {
    if (m_isPlaying) {
        pause(); // Use pause instead of stop
    }

    m_zoomFactor = 1.0;
    m_panOffset = QPointF(0.0, 0.0);
    m_activeRoiRect = QRectF();
    m_isPanning = false;
    m_isDefiningRoi = false;
    // m_showThresholdMask = false; // Optionally reset threshold view on new video load
    // currentQImageFrame = QImage(); // Clear display immediately

    if (!openVideoFile(filePath)) {
        currentFilePath.clear(); totalFramesCount = 0; framesPerSecond = 0.0; currentFrameIdx = -1;
        originalFrameSize = QSize(); currentQImageFrame = QImage(); m_thresholdedFrame_mono = cv::Mat();
        update(); updateCursorShape();
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
        videoCapture.release(); currentFilePath.clear();
        update(); updateCursorShape();
        return false;
    }

    seekToFrame(0); // This will call displayFrame, which handles thresholding if active

    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor);
    emit roiDefined(m_activeRoiRect);
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

// Play now toggles play/pause
void VideoLoader::play() {
    if (!isVideoLoaded()) {
        return;
    }
    if (m_isPlaying) { // If already playing, then pause
        pause();
    } else { // If paused or stopped, then play
        if (currentFrameIdx >= totalFramesCount - 1 && totalFramesCount > 0) { // If at the end, restart
            seekToFrame(0);
            if (currentFrameIdx == -1 && totalFramesCount > 0) return; // Seek failed
        }
        m_isPlaying = true;
        int interval = (framesPerSecond > 0) ? qRound(1000.0 / framesPerSecond) : 40;
        playbackTimer->start(interval);
        emit playbackStateChanged(true);
        qDebug() << "Playback started.";
    }
}

void VideoLoader::pause() {
    if (!m_isPlaying && !playbackTimer->isActive()) return; // Already paused
    m_isPlaying = false;
    playbackTimer->stop();
    emit playbackStateChanged(false);
    qDebug() << "Playback paused.";
}

// seekToFrame and displayFrame with suppressEmit (from user's context)
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
        currentFrameIdx = -1; currentQImageFrame = QImage(); m_thresholdedFrame_mono = cv::Mat(); update(); return;
    }

    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    if (currentPos != frameNumber) {
        if(!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) {
            qWarning() << "Failed to set frame position to" << frameNumber;
        }
    }

    if (videoCapture.read(currentCvFrame)) {
        if (!currentCvFrame.empty()) {
            currentFrameIdx = frameNumber; // Set index before potential thresholding
            if (m_showThresholdMask) {
                applyThresholding(); // Updates m_thresholdedFrame_mono
                // currentQImageFrame will be updated from m_thresholdedFrame_mono in paintEvent or here
            }
            // convertCvMatToQImage(currentCvFrame, currentQImageFrame); // Convert original for non-threshold view
            // The QImage to display will be determined in paintEvent or set here based on m_showThresholdMask
        } else {
            currentCvFrame = cv::Mat(); // Ensure it's empty
            m_thresholdedFrame_mono = cv::Mat();
            qWarning() << "Read empty frame at index" << frameNumber;
        }
    } else {
        currentCvFrame = cv::Mat();
        m_thresholdedFrame_mono = cv::Mat();
        qWarning() << "Failed to read frame at index" << frameNumber;
        if (m_isPlaying) pause(); // Use pause
    }

    // Update currentQImageFrame based on whether threshold view is active
    if (m_showThresholdMask && !m_thresholdedFrame_mono.empty()) {
        convertCvMatToQImage(m_thresholdedFrame_mono, currentQImageFrame);
    } else if (!currentCvFrame.empty()) {
        convertCvMatToQImage(currentCvFrame, currentQImageFrame);
    } else {
        currentQImageFrame = QImage(); // Clear if both are empty
    }

    currentFrameIdx = frameNumber; // Ensure index is correct

    if (!suppressEmit) {
        emit frameChanged(currentFrameIdx, currentQImageFrame);
    }
    update(); // Schedule repaint
}


void VideoLoader::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) {
    if (mat.empty()) { qimg = QImage(); return; }
    if (mat.type() == CV_8UC3) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped(); }
    else if (mat.type() == CV_8UC1) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8); }
    else {
        cv::Mat temp;
        try {
            if (mat.channels() == 4) { cv::cvtColor(mat, temp, cv::COLOR_BGRA2BGR); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 3 && mat.type() != CV_8UC3) { // e.g. CV_32FC3
                mat.convertTo(temp, CV_8UC3, 255.0); // Basic scaling, might need normalization
                qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped();
            }
            else if (mat.channels() == 1 && mat.type() != CV_8UC1) { // e.g. CV_32FC1
                mat.convertTo(temp, CV_8UC1, 255.0); // Basic scaling
                qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_Grayscale8);
            }
            else { qWarning() << "Unsupported cv::Mat type for conversion:" << mat.type(); qimg = QImage(); return; }
        } catch (const cv::Exception& ex) { qWarning() << "OpenCV exception during color conversion:" << ex.what(); qimg = QImage(); }
    }
}

void VideoLoader::processNextFrame() {
    if (!m_isPlaying || !isVideoLoaded()) return;
    if (currentFrameIdx < totalFramesCount - 1) displayFrame(currentFrameIdx + 1); else pause(); // Use pause at end
}

void VideoLoader::setZoomFactor(double factor) { setZoomFactorAtPoint(factor, rect().center()); }
void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) {
    if (!isVideoLoaded()) return;
    double newZoomFactor = qBound(0.05, factor, 50.0);
    if (qFuzzyCompare(m_zoomFactor, newZoomFactor)) return;
    QPointF videoPoint = mapPointToVideo(widgetPoint);
    QPointF refWidgetPoint = widgetPoint;
    if (videoPoint.x() < 0) { refWidgetPoint = rect().center(); videoPoint = mapPointToVideo(refWidgetPoint); if (videoPoint.x() < 0) return; }
    m_zoomFactor = newZoomFactor;
    QPointF widgetPointAfterZoom = mapPointFromVideo(videoPoint);
    m_panOffset += (refWidgetPoint - widgetPointAfterZoom);
    clampPanOffset(); update(); emit zoomFactorChanged(m_zoomFactor);
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

// --- Thresholding Control Slots ---
void VideoLoader::toggleThresholdView(bool enabled) {
    if (m_showThresholdMask == enabled) return;
    m_showThresholdMask = enabled;
    qDebug() << "Threshold view" << (enabled ? "enabled" : "disabled");
    if (isVideoLoaded() && currentFrameIdx >= 0) {
        // Re-process and display the current frame with the new threshold view state
        displayFrame(currentFrameIdx, true); // Suppress emit, update will trigger repaint
    }
    update(); // Ensure repaint
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
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && m_thresholdAlgorithm == ThresholdAlgorithm::Global) { // Only re-proc if global
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
    if (blockSize < 3) blockSize = 3;       // Must be >=3
    if (blockSize % 2 == 0) blockSize += 1; // Must be odd
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


// --- Event Handlers ---
void VideoLoader::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    // currentQImageFrame should already be set by displayFrame to be either original or thresholded
    if (currentQImageFrame.isNull() || !isVideoLoaded()) {
        painter.fillRect(rect(), palette().color(QPalette::Window));
        painter.setPen(Qt::gray);
        painter.drawText(rect(), Qt::AlignCenter, "No video loaded.");
        return;
    }

    QRectF targetRect = calculateTargetRect();
    painter.drawImage(targetRect, currentQImageFrame, currentQImageFrame.rect());

    // Draw the active ROI (for DrawROI mode) - if it's not null and valid
    if (m_currentMode == InteractionMode::DrawROI && !m_activeRoiRect.isNull() && m_activeRoiRect.isValid()) {
        QPointF roiTopLeftVideo = m_activeRoiRect.topLeft();
        QPointF roiBottomRightVideo = m_activeRoiRect.bottomRight();
        QPointF roiTopLeftWidget = mapPointFromVideo(roiTopLeftVideo);
        QPointF roiBottomRightWidget = mapPointFromVideo(roiBottomRightVideo);

        if (roiTopLeftWidget.x() >= 0 && roiBottomRightWidget.x() >= 0) { // Check mapping validity
            QRectF roiWidgetRect(roiTopLeftWidget, roiBottomRightWidget);
            painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
            painter.setBrush(Qt::NoBrush);
            painter.drawRect(roiWidgetRect.normalized());
        }
    }

    // Draw ROI/Crop rectangle being defined (temporary visual feedback)
    if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi) {
        painter.setPen(QPen(Qt::cyan, 1, Qt::SolidLine));
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(QRect(m_roiStartPointWidget, m_roiEndPointWidget).normalized());
    }
}

QRectF VideoLoader::calculateTargetRect() const { /* ... (same as before) ... */
    if (originalFrameSize.isEmpty()) return QRectF();
    QSizeF ws = size(), ofs = originalFrameSize, cs = ofs;
    cs.scale(ws, Qt::KeepAspectRatio); QSizeF zs = cs * m_zoomFactor;
    QPointF ctl((ws.width()-zs.width())/2.0, (ws.height()-zs.height())/2.0);
    return QRectF(ctl + m_panOffset, zs);
}
QPointF VideoLoader::mapPointToVideo(const QPointF& wp) const { /* ... (same as before) ... */
    if (currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0) return QPointF(-1,-1);
    QRectF tr = calculateTargetRect();
    if (!tr.contains(wp)||tr.width()<=0||tr.height()<=0) return QPointF(-1,-1);
    double nx=(wp.x()-tr.left())/tr.width(), ny=(wp.y()-tr.top())/tr.height();
    QSizeF ofs = originalFrameSize;
    return QPointF(qBound(0.0, nx*ofs.width(), ofs.width()), qBound(0.0, ny*ofs.height(), ofs.height()));
}
QPointF VideoLoader::mapPointFromVideo(const QPointF& vp) const { /* ... (same as before) ... */
    if (currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0||originalFrameSize.width()<=0||originalFrameSize.height()<=0) return QPointF(-1,-1);
    QRectF tr = calculateTargetRect(); if (tr.width()<=0||tr.height()<=0) return QPointF(-1,-1);
    QSizeF ofs = originalFrameSize; QPointF cvp(qBound(0.0,vp.x(),ofs.width()), qBound(0.0,vp.y(),ofs.height()));
    double nx=cvp.x()/ofs.width(), ny=cvp.y()/ofs.height();
    return QPointF(tr.left()+nx*tr.width(), tr.top()+ny*tr.height());
}

void VideoLoader::mousePressEvent(QMouseEvent *event) { /* ... (same as user's v6) ... */
    m_lastMousePos = event->position();
    if (!isVideoLoaded()) { QWidget::mousePressEvent(event); return; }
    if (event->button() == Qt::LeftButton) {
        if (m_currentMode == InteractionMode::PanZoom) {
            m_isPanning = true; updateCursorShape(); event->accept();
        } else if (m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) {
            QPointF videoCoords = mapPointToVideo(event->position());
            if (videoCoords.x() >= 0) {
                m_roiStartPointWidget = event->pos(); m_roiEndPointWidget = event->pos(); m_isDefiningRoi = true;
                update(); qDebug() << (m_currentMode == InteractionMode::Crop ? "Crop" : "ROI") << "definition started."; event->accept();
            } else { m_isDefiningRoi = false; }
        }
    } else { QWidget::mousePressEvent(event); }
}
void VideoLoader::mouseMoveEvent(QMouseEvent *event) { /* ... (same as user's v6) ... */
    QPointF currentPos = event->position(); QPointF delta = currentPos - m_lastMousePos;
    if (!isVideoLoaded()) { QWidget::mouseMoveEvent(event); return; }
    if (m_isPanning && (event->buttons() & Qt::LeftButton)) {
        m_panOffset += delta; clampPanOffset(); update(); event->accept();
    } else if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi && (event->buttons() & Qt::LeftButton)) {
        m_roiEndPointWidget = event->pos(); update(); event->accept();
    } else { QWidget::mouseMoveEvent(event); }
    m_lastMousePos = currentPos;
}
void VideoLoader::mouseReleaseEvent(QMouseEvent *event) { /* ... (same as user's v6, calls handleRoiDefinedForCrop) ... */
    if (!isVideoLoaded()) { QWidget::mouseReleaseEvent(event); return; }
    if (event->button() == Qt::LeftButton) {
        if (m_isPanning) {
            m_isPanning = false; updateCursorShape(); event->accept();
        } else if (m_isDefiningRoi && (m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop)) {
            m_roiEndPointWidget = event->pos(); m_isDefiningRoi = false;
            QPointF vs = mapPointToVideo(m_roiStartPointWidget), ve = mapPointToVideo(m_roiEndPointWidget);
            QRectF definedRect;
            if(vs.x()>=0 && ve.x()>=0) { definedRect = QRectF(vs,ve).normalized(); if(definedRect.width()<5||definedRect.height()<5) definedRect=QRectF(); }
            if (m_currentMode == InteractionMode::DrawROI) {
                m_activeRoiRect = definedRect; emit roiDefined(m_activeRoiRect); qDebug() << "ROI selection ended. Video_rect:" << m_activeRoiRect;
            } else if (m_currentMode == InteractionMode::Crop) {
                if (!definedRect.isNull()&&definedRect.isValid()) handleRoiDefinedForCrop(definedRect);
                else { qDebug() << "Crop area invalid."; setInteractionMode(InteractionMode::PanZoom); }
            }
            update(); updateCursorShape(); event->accept();
        }
    } else { QWidget::mouseReleaseEvent(event); }
}
void VideoLoader::wheelEvent(QWheelEvent *event) { /* ... (same as user's v6) ... */
    if (!isVideoLoaded() || m_currentMode != InteractionMode::PanZoom) { event->ignore(); return; }
    int deg = event->angleDelta().y()/8; if(deg==0){event->ignore();return;} int steps=deg/15; double zs=0.15, mult=qPow(1.0+zs,steps);
    double nz = qBound(0.05, m_zoomFactor*mult, 50.0); if(qFuzzyCompare(m_zoomFactor,nz)){event->accept();return;}
    QPointF mpw=event->position(), vp=mapPointToVideo(mpw), rpw=mpw;
    if(vp.x()<0){rpw=rect().center(); vp=mapPointToVideo(rpw); if(vp.x()<0){event->ignore();return;}}
    m_zoomFactor=nz; m_panOffset+=(rpw-mapPointFromVideo(vp)); clampPanOffset();update();emit zoomFactorChanged(m_zoomFactor);event->accept();
}
void VideoLoader::resizeEvent(QResizeEvent *event) { /* ... (same as user's v6) ... */
    QWidget::resizeEvent(event); if (isVideoLoaded()) { clampPanOffset(); update(); }
}

// --- Private Helper Methods ---
void VideoLoader::updateCursorShape() { /* ... (same as user's v6) ... */
    if (!isVideoLoaded()) { setCursor(Qt::ArrowCursor); return; }
    switch (m_currentMode) {
    case InteractionMode::PanZoom: setCursor(m_isPanning ? Qt::ClosedHandCursor : Qt::OpenHandCursor); break;
    case InteractionMode::DrawROI: case InteractionMode::Crop: setCursor(Qt::CrossCursor); break;
    default: setCursor(Qt::ArrowCursor); break;
    }
}
void VideoLoader::clampPanOffset() { /* ... (same as user's v6) ... */
    if (!isVideoLoaded()||m_zoomFactor<=0)return; QRectF tr=calculateTargetRect();QSizeF ws=size();
    double mx=-(tr.width()-50.0),my=-(tr.height()-50.0),mX=ws.width()-50.0,mY=ws.height()-50.0;
    QPointF cftl=tr.topLeft(),clftl; clftl.setX(qBound(mx,cftl.x(),mX));clftl.setY(qBound(my,cftl.y(),mY));
    if(tr.width()<=ws.width())clftl.setX((ws.width()-tr.width())/2.0); if(tr.height()<=ws.height())clftl.setY((ws.height()-tr.height())/2.0);
    QSizeF ofs=originalFrameSize,cs=ofs;cs.scale(ws,Qt::KeepAspectRatio);QSizeF zs=cs*m_zoomFactor;
    QPointF ctl_center((ws.width()-zs.width())/2.0,(ws.height()-zs.height())/2.0);
    QPointF rpo=clftl-ctl_center; if(!qFuzzyCompare(m_panOffset.x(),rpo.x())||!qFuzzyCompare(m_panOffset.y(),rpo.y()))m_panOffset=rpo;
}

void VideoLoader::handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords) { /* ... (same as user's v6) ... */
    if (cropRoiVideoCoords.isNull()||!cropRoiVideoCoords.isValid()){setInteractionMode(InteractionMode::PanZoom);return;}
    QMessageBox::StandardButton reply = QMessageBox::question(this,"Confirm Crop",QString("Crop video to ROI?"),QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        emit videoProcessingStarted("Cropping video..."); QString croppedFilePath; bool success = performVideoCrop(cropRoiVideoCoords, croppedFilePath);
        if (success && !croppedFilePath.isEmpty()) {
            emit videoProcessingFinished("Video cropped.",true); loadVideo(croppedFilePath);
        } else {
            emit videoProcessingFinished("Crop failed.",false); QMessageBox::critical(this,"Crop Error","Failed to crop video.");
        }
    }
    setInteractionMode(InteractionMode::PanZoom); m_isDefiningRoi=false; update();
}

bool VideoLoader::performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath) { /* ... (same as user's v6, uses DEFAULT_CROP_FOURCC) ... */
    if (currentFilePath.isEmpty()||!videoCapture.isOpened()||cropRectVideoCoords.isNull()||!cropRectVideoCoords.isValid()) return false;
    cv::VideoCapture origVid; if(!origVid.open(currentFilePath.toStdString())) return false;
    double origFps=origVid.get(cv::CAP_PROP_FPS); if(origFps<=0)origFps=25.0;
    cv::Rect cvCR(static_cast<int>(qRound(cropRectVideoCoords.x())),static_cast<int>(qRound(cropRectVideoCoords.y())),
                  static_cast<int>(qRound(cropRectVideoCoords.width())),static_cast<int>(qRound(cropRectVideoCoords.height())));
    if(cvCR.x<0)cvCR.x=0; if(cvCR.y<0)cvCR.y=0;
    if(cvCR.x+cvCR.width > originalFrameSize.width()) cvCR.width=originalFrameSize.width()-cvCR.x;
    if(cvCR.y+cvCR.height > originalFrameSize.height()) cvCR.height=originalFrameSize.height()-cvCR.y;
    if(cvCR.width<=0||cvCR.height<=0){origVid.release();return false;}
    cv::Size cfs(cvCR.width,cvCR.height);
    QFileInfo ofi(currentFilePath); QString bn=ofi.completeBaseName(), suff=ofi.suffix().isEmpty()?QString(DEFAULT_CROP_EXTENSION).remove(0,1):ofi.suffix();
    outCroppedFilePath = ofi.absolutePath()+"/"+bn+"_cropped."+suff; // Use defined extension
    cv::VideoWriter writer; int fourcc = DEFAULT_CROP_FOURCC;
    if(!writer.open(outCroppedFilePath.toStdString(),fourcc,origFps,cfs,true)){origVid.release();return false;}
    cv::Mat frame,croppedF; int fCount=0;
    while(origVid.read(frame)){if(frame.empty())continue; try{croppedF=frame(cvCR);writer.write(croppedF);fCount++;}catch(const cv::Exception&){break;}}
    origVid.release();writer.release(); return fCount>0;
}

// New private method to apply thresholding
void VideoLoader::applyThresholding() {
    if (currentCvFrame.empty()) {
        m_thresholdedFrame_mono = cv::Mat(); // Clear if no source frame
        return;
    }

    cv::Mat grayFrame;
    if (currentCvFrame.channels() == 3 || currentCvFrame.channels() == 4) {
        cv::cvtColor(currentCvFrame, grayFrame, cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = currentCvFrame.clone(); // Already grayscale or single channel
    }

    // Optional: Apply Gaussian blur to reduce noise before thresholding
    // Adjust kernel size (e.g., (5,5)) and sigma as needed. Must be odd.
    cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5, 5), 0);

    int thresholdType = m_assumeLightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;

    switch (m_thresholdAlgorithm) {
    case ThresholdAlgorithm::Global:
        cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255, thresholdType);
        break;
    case ThresholdAlgorithm::Otsu:
        // m_thresholdValue is ignored by Otsu, it calculates it.
        cv::threshold(grayFrame, m_thresholdedFrame_mono, 0, 255, thresholdType | cv::THRESH_OTSU);
        break;
    case ThresholdAlgorithm::AdaptiveMean:
        cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255,
                              cv::ADAPTIVE_THRESH_MEAN_C, thresholdType,
                              m_adaptiveBlockSize, m_adaptiveC);
        break;
    case ThresholdAlgorithm::AdaptiveGaussian:
        cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255,
                              cv::ADAPTIVE_THRESH_GAUSSIAN_C, thresholdType,
                              m_adaptiveBlockSize, m_adaptiveC);
        break;
    default:
        // Fallback to global if algorithm unknown (should not happen)
        cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255, thresholdType);
        break;
    }
}

