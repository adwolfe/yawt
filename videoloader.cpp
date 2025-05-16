#include "videoloader.h"    // Lowercase include

#include <QDebug>
#include <QPainterPath>
#include <QtMath>
#include <QResizeEvent>
#include <QMessageBox>
#include <QFileInfo>
#include <QWheelEvent>       // Ensure this is included for wheelEvent
#include <QRandomGenerator> // For generating distinct track colors

// Default crop parameters
#define DEFAULT_CROP_EXTENSION ".mp4"
#define DEFAULT_CROP_FOURCC cv::VideoWriter::fourcc('H', '2', '6', '4')

// Define a click tolerance for selecting track points (in widget pixels)
const qreal TRACK_POINT_CLICK_TOLERANCE = 5.0; // Example value


VideoLoader::VideoLoader(QWidget *parent)
    : QWidget(parent),
    playbackTimer(new QTimer(this)),
    totalFramesCount(0),
    framesPerSecond(0.0),
    currentFrameIdx(-1),
    m_isPlaying(false),
    m_playbackSpeedMultiplier(1.0),
    m_currentMode(InteractionMode::PanZoom),
    m_isPanning(false),
    m_isDefiningRoi(false),
    m_zoomFactor(1.0),
    m_panOffset(0.0, 0.0),
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

// --- Public Methods & Getters (as in your provided file) ---
bool VideoLoader::isVideoLoaded() const { return videoCapture.isOpened() && totalFramesCount > 0; }
int VideoLoader::getTotalFrames() const { return totalFramesCount; }
double VideoLoader::getFPS() const { return framesPerSecond; }
int VideoLoader::getCurrentFrameNumber() const { return currentFrameIdx; }
QSize VideoLoader::getVideoFrameSize() const { return originalFrameSize; }
double VideoLoader::getZoomFactor() const { return m_zoomFactor; }
QRectF VideoLoader::getCurrentRoi() const { return m_activeRoiRect; }
InteractionMode VideoLoader::getCurrentInteractionMode() const { return m_currentMode; }
double VideoLoader::getPlaybackSpeed() const { return m_playbackSpeedMultiplier; }
QString VideoLoader::getCurrentVideoPath() const { return currentFilePath;}
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
ThresholdAlgorithm VideoLoader::getCurrentThresholdAlgorithm() const { return m_thresholdAlgorithm; }
int VideoLoader::getThresholdValue() const { return m_thresholdValue; }
bool VideoLoader::getAssumeLightBackground() const { return m_assumeLightBackground; }
int VideoLoader::getAdaptiveBlockSize() const { return m_adaptiveBlockSize; }
double VideoLoader::getAdaptiveCValue() const { return m_adaptiveC; }
bool VideoLoader::isBlurEnabled() const { return m_enableBlur; }
int VideoLoader::getBlurKernelSize() const { return m_blurKernelSize; }
double VideoLoader::getBlurSigmaX() const { return m_blurSigmaX; }


// --- Control Slots (as in your provided file) ---
bool VideoLoader::loadVideo(const QString &filePath) {
    if (m_isPlaying) pause();
    m_zoomFactor = 1.0; m_panOffset = QPointF(0.0, 0.0); m_activeRoiRect = QRectF();
    clearWormSelections(); clearDisplayedTracks();
    m_isPanning = false; m_isDefiningRoi = false; m_playbackSpeedMultiplier = 1.0;
    m_showThresholdMask = false; m_thresholdAlgorithm = ThresholdAlgorithm::Global;
    m_thresholdValue = 127; m_assumeLightBackground = true; m_adaptiveBlockSize = 11;
    m_adaptiveC = 2.0; m_enableBlur = false; m_blurKernelSize = 5; m_blurSigmaX = 0.0;

    if (!openVideoFile(filePath)) { /* error handling */ return false; }
    currentFilePath = filePath; framesPerSecond = videoCapture.get(cv::CAP_PROP_FPS);
    if (framesPerSecond <= 0) framesPerSecond = 25.0;
    totalFramesCount = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    originalFrameSize = QSize(static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH)),
                              static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT)));
    if (totalFramesCount <= 0 || originalFrameSize.isEmpty()) { /* error handling */ return false; }
    seekToFrame(0);
    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor); emit roiDefined(m_activeRoiRect);
    emit playbackSpeedChanged(m_playbackSpeedMultiplier); emitThresholdParametersChanged();
    updateCursorShape(); qDebug() << "Video loaded:" << filePath; return true;
}
bool VideoLoader::openVideoFile(const QString &filePath) {
    if (videoCapture.isOpened()) videoCapture.release();
    try { return videoCapture.open(filePath.toStdString()); }
    catch (const cv::Exception& ex) { qWarning() << "OpenCV exception:" << ex.what(); return false; }
}
void VideoLoader::play() {
    if (!isVideoLoaded()) return; if (m_isPlaying) { pause(); }
    else { if (currentFrameIdx >= totalFramesCount - 1 && totalFramesCount > 0) seekToFrame(0);
        if (currentFrameIdx == -1 && totalFramesCount > 0) return;
        m_isPlaying = true; updateTimerInterval(); playbackTimer->start();
        emit playbackStateChanged(true, m_playbackSpeedMultiplier);
    }
}
void VideoLoader::pause() {
    if (!m_isPlaying && !playbackTimer->isActive()) return;
    m_isPlaying = false; playbackTimer->stop();
    emit playbackStateChanged(false, m_playbackSpeedMultiplier);
}
void VideoLoader::seekToFrame(int frameNumber, bool suppressEmit) {
    if (!isVideoLoaded()) return;
    frameNumber = qBound(0, frameNumber, totalFramesCount > 0 ? totalFramesCount - 1 : 0);
    displayFrame(frameNumber, suppressEmit);
}
void VideoLoader::setZoomFactor(double factor) { setZoomFactorAtPoint(factor, rect().center()); }
void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) {
    if (!isVideoLoaded()) return; double newZoomFactor = qBound(0.05, factor, 50.0);
    if (qFuzzyCompare(m_zoomFactor, newZoomFactor)) return;
    QPointF videoPointBeforeZoom = mapPointToVideo(widgetPoint); m_zoomFactor = newZoomFactor;
    if (videoPointBeforeZoom.x() < 0 || videoPointBeforeZoom.y() < 0) {
        QSizeF ws = size(); QRectF targetRectNow = calculateTargetRect();
        QPointF currentTopLeft = targetRectNow.topLeft() - m_panOffset;
        m_panOffset.setX((ws.width() - targetRectNow.width()) / 2.0 - currentTopLeft.x());
        m_panOffset.setY((ws.height() - targetRectNow.height()) / 2.0 - currentTopLeft.y());
    } else {
        QPointF widgetPointAfterZoom = mapPointFromVideo(videoPointBeforeZoom);
        m_panOffset += (widgetPoint - widgetPointAfterZoom);
    } clampPanOffset(); update(); emit zoomFactorChanged(m_zoomFactor);
}
void VideoLoader::clearRoi() {
    if (!m_activeRoiRect.isNull()) { m_activeRoiRect = QRectF(); update(); emit roiDefined(m_activeRoiRect); }
}
void VideoLoader::setPlaybackSpeed(double multiplier) {
    double newSpeed = qBound(0.1, multiplier, 10.0);
    if (qFuzzyCompare(m_playbackSpeedMultiplier, newSpeed)) return;
    m_playbackSpeedMultiplier = newSpeed; if (m_isPlaying) { playbackTimer->stop(); updateTimerInterval(); playbackTimer->start(); }
    emit playbackSpeedChanged(m_playbackSpeedMultiplier); emit playbackStateChanged(m_isPlaying, m_playbackSpeedMultiplier);
}
void VideoLoader::clearWormSelections() {
    if (!m_selectedCentroids_temp.isEmpty() || !m_selectedBounds_temp.isEmpty()) {
        m_selectedCentroids_temp.clear(); m_selectedBounds_temp.clear(); update();
    }
}
void VideoLoader::toggleThresholdView(bool enabled) {
    if (m_showThresholdMask == enabled) return; m_showThresholdMask = enabled;
    if (isVideoLoaded() && currentFrameIdx >= 0) displayFrame(currentFrameIdx, true); update();
}
void VideoLoader::setThresholdAlgorithm(ThresholdAlgorithm algorithm) {
    if (m_thresholdAlgorithm == algorithm) return; m_thresholdAlgorithm = algorithm;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setThresholdValue(int value) {
    value = qBound(0, value, 255); if (m_thresholdValue == value) return; m_thresholdValue = value;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && m_thresholdAlgorithm == ThresholdAlgorithm::Global) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setAssumeLightBackground(bool isLight) {
    if (m_assumeLightBackground == isLight) return; m_assumeLightBackground = isLight;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setAdaptiveThresholdBlockSize(int blockSize) {
    if (blockSize < 3) blockSize = 3; if (blockSize % 2 == 0) blockSize += 1;
    if (m_adaptiveBlockSize == blockSize) return; m_adaptiveBlockSize = blockSize;
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
    if (m_enableBlur == enabled) return; m_enableBlur = enabled;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >= 0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setBlurKernelSize(int kernelSize) {
    if (kernelSize < 3) kernelSize = 3; if (kernelSize % 2 == 0) kernelSize += 1;
    if (m_blurKernelSize == kernelSize) return; m_blurKernelSize = kernelSize;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >= 0 && m_enableBlur) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setBlurSigmaX(double sigmaX) {
    sigmaX = qMax(0.0, sigmaX); if (qFuzzyCompare(m_blurSigmaX, sigmaX)) return;
    m_blurSigmaX = sigmaX;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >= 0 && m_enableBlur) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}

void VideoLoader::setInteractionMode(InteractionMode mode) {
    if (m_currentMode == mode) return;
    m_currentMode = mode;
    m_isPanning = false;
    m_isDefiningRoi = false;
    updateCursorShape();
    emit interactionModeChanged(m_currentMode);
    qDebug() << "Interaction mode set to:" << static_cast<int>(m_currentMode);
    update();
}

// --- Slots for Track Display (as in your provided file) ---
void VideoLoader::setTracksToDisplay(const AllWormTracks& tracks) {
    m_allTracksToDisplay = tracks;
    m_trackColors.clear();
    update();
    qDebug() << "VideoLoader: Tracks set for display. Count:" << m_allTracksToDisplay.size();
}

void VideoLoader::setVisibleTrackIDs(const QSet<int>& visibleTrackIDs) {
    if (m_visibleTrackIDs == visibleTrackIDs) return;
    m_visibleTrackIDs = visibleTrackIDs;
    update();
    qDebug() << "VideoLoader: Visible track IDs updated. Count:" << m_visibleTrackIDs.size();
}

void VideoLoader::clearDisplayedTracks() {
    m_allTracksToDisplay.clear();
    m_visibleTrackIDs.clear();
    m_trackColors.clear();
    update();
    qDebug() << "VideoLoader: All displayed tracks cleared.";
}

// --- displayFrame, processNextFrame, convertCvMatToQImage (as in your provided file) ---
void VideoLoader::displayFrame(int frameNumber, bool suppressEmit) {
    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) { return; }
    if (frameNumber < 0 || frameNumber >= totalFramesCount) { return; }
    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    if (currentPos != frameNumber && !(currentPos == frameNumber -1 && m_isPlaying) ) {
        if(!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) { /* ... */ }
    }
    if (videoCapture.read(currentCvFrame)) {
        if (!currentCvFrame.empty()) {
            currentFrameIdx = frameNumber;
            applyThresholding(); // Always update m_thresholdedFrame_mono
        } else {
            currentCvFrame = cv::Mat(); m_thresholdedFrame_mono = cv::Mat();
        }
    } else {
        currentCvFrame = cv::Mat(); m_thresholdedFrame_mono = cv::Mat();
    }
    if (m_showThresholdMask && !m_thresholdedFrame_mono.empty()) {
        convertCvMatToQImage(m_thresholdedFrame_mono, currentQImageFrame);
    } else if (!currentCvFrame.empty()) {
        convertCvMatToQImage(currentCvFrame, currentQImageFrame);
    } else {
        currentQImageFrame = QImage();
    }
    if (!suppressEmit) emit frameChanged(currentFrameIdx, currentQImageFrame);
    update();
}

void VideoLoader::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) {
    if (mat.empty()) { qimg = QImage(); return; }
    if (mat.type() == CV_8UC3) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped(); }
    else if (mat.type() == CV_8UC1) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8); }
    else { cv::Mat temp; try {
            if (mat.channels() == 4) { cv::cvtColor(mat, temp, cv::COLOR_BGRA2BGR); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 3 && mat.type() != CV_8UC3) { mat.convertTo(temp, CV_8UC3, 255.0); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 1 && mat.type() != CV_8UC1) { mat.convertTo(temp, CV_8UC1, 255.0); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_Grayscale8); }
            else { qimg = QImage(); }
        } catch (const cv::Exception& ex) { qWarning() << "OpenCV conversion exception:" << ex.what(); qimg = QImage(); }}
}
void VideoLoader::processNextFrame() {
    if (!m_isPlaying || !isVideoLoaded()) return;
    if (currentFrameIdx < totalFramesCount - 1) displayFrame(currentFrameIdx + 1); else pause();
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

    if (m_currentMode == InteractionMode::DrawROI && !m_activeRoiRect.isNull() && m_activeRoiRect.isValid()) {
        QPointF roiTopLeftWidget = mapPointFromVideo(m_activeRoiRect.topLeft());
        QPointF roiBottomRightWidget = mapPointFromVideo(m_activeRoiRect.bottomRight());
        if (roiTopLeftWidget.x() >= 0 && roiBottomRightWidget.x() >= 0) {
            painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
            painter.drawRect(QRectF(roiTopLeftWidget, roiBottomRightWidget).normalized());
        }
    }

    if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi) {
        painter.setPen(QPen(Qt::cyan, 1, Qt::SolidLine));
        painter.drawRect(QRect(m_roiStartPointWidget, m_roiEndPointWidget).normalized());
    }

    if (m_currentMode == InteractionMode::SelectWorms) {
        painter.setPen(QPen(Qt::yellow, 1, Qt::DotLine));
        for (const QRectF& boundsVideo : qAsConst(m_selectedBounds_temp)) {
            QPointF topLeftWidget = mapPointFromVideo(boundsVideo.topLeft());
            QPointF bottomRightWidget = mapPointFromVideo(boundsVideo.bottomRight());
            if (topLeftWidget.x() >=0 && bottomRightWidget.x() >=0) {
                painter.drawRect(QRectF(topLeftWidget, bottomRightWidget).normalized());
            }
        }
        painter.setPen(QPen(Qt::green, 2));
        for (const QPointF& centroidVideo : qAsConst(m_selectedCentroids_temp)) {
            QPointF centroidWidget = mapPointFromVideo(centroidVideo);
            if (centroidWidget.x() >= 0) {
                painter.drawEllipse(centroidWidget, 3, 3);
            }
        }
    }

    if (m_currentMode == InteractionMode::ViewEditTracks && !m_allTracksToDisplay.empty()) {
        painter.setRenderHint(QPainter::Antialiasing, true);
        for (int trackId : qAsConst(m_visibleTrackIDs)) {
            if (m_allTracksToDisplay.count(trackId)) {
                const std::vector<WormTrackPoint>& trackPoints = m_allTracksToDisplay.at(trackId);
                if (trackPoints.size() < 2) continue;
                QPainterPath path;
                QColor trackColor = getTrackColor(trackId);
                QPen trackPen(trackColor, 2);
                painter.setPen(trackPen);
                for (size_t i = 0; i < trackPoints.size(); ++i) {
                    const WormTrackPoint& pt = trackPoints[i];
                    QPointF currentPointVideo(pt.position.x, pt.position.y);
                    QPointF currentPointWidget = mapPointFromVideo(currentPointVideo);
                    if (currentPointWidget.x() < 0) continue;
                    if (i == 0) { path.moveTo(currentPointWidget); }
                    else { path.lineTo(currentPointWidget); }
                    painter.setBrush(trackColor);
                    painter.drawEllipse(currentPointWidget, 2, 2);
                    painter.setBrush(Qt::NoBrush);
                }
                painter.strokePath(path, trackPen);
            }
        }
    }
}


void VideoLoader::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->position();
    if (!isVideoLoaded()) { QWidget::mousePressEvent(event); return; }

    if (event->button() == Qt::LeftButton) {
        if (m_currentMode == InteractionMode::PanZoom) {
            m_isPanning = true; updateCursorShape(); event->accept();
        } else if (m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) {
            QPointF videoCoords = mapPointToVideo(event->position());
            if (videoCoords.x() >= 0) {
                m_roiStartPointWidget = event->pos(); m_roiEndPointWidget = event->pos();
                m_isDefiningRoi = true; update(); event->accept();
            } else { m_isDefiningRoi = false; }
        } else if (m_currentMode == InteractionMode::SelectWorms) {
            // m_thresholdedFrame_mono is always updated by displayFrame -> applyThresholding
            if (!m_thresholdedFrame_mono.empty()) {
                QPointF clickVideoPoint = mapPointToVideo(event->position());
                if (clickVideoPoint.x() >= 0) { // Click is on the video
                    // Call the helper function from TrackingHelper
                    // Default minArea, maxArea, and maxDistanceForSelection are used from trackinghelper.h
                    TrackingHelper::DetectedBlob blob = TrackingHelper::findClickedBlob(
                        m_thresholdedFrame_mono, clickVideoPoint);

                    if (blob.isValid) {
                        qDebug() << "VideoLoader: Selected blob centroid:" << blob.centroid << "Bounds:" << blob.boundingBox;
                        m_selectedCentroids_temp.append(blob.centroid);
                        m_selectedBounds_temp.append(blob.boundingBox);
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
        } else if (m_currentMode == InteractionMode::ViewEditTracks) {
            QPointF clickWidgetPoint = event->position();
            int bestTrackId = -1; int bestFrameNum = -1; QPointF bestVideoPoint;
            double minDistanceSq = TRACK_POINT_CLICK_TOLERANCE * TRACK_POINT_CLICK_TOLERANCE;
            for (int trackId : qAsConst(m_visibleTrackIDs)) {
                if (m_allTracksToDisplay.count(trackId)) {
                    const auto& trackPoints = m_allTracksToDisplay.at(trackId);
                    for (const auto& pt : trackPoints) {
                        QPointF videoPt(pt.position.x, pt.position.y);
                        QPointF widgetPt = mapPointFromVideo(videoPt);
                        if (widgetPt.x() < 0) continue;
                        double dx = widgetPt.x() - clickWidgetPoint.x();
                        double dy = widgetPt.y() - clickWidgetPoint.y();
                        double distSq = dx * dx + dy * dy;
                        if (distSq < minDistanceSq) {
                            minDistanceSq = distSq; bestTrackId = trackId;
                            bestFrameNum = pt.frameNumberOriginal; bestVideoPoint = videoPt;
                        }
                    }
                }
            }
            if (bestTrackId != -1) {
                qDebug() << "Track point clicked: Worm ID" << bestTrackId << "Frame" << bestFrameNum;
                emit trackPointClicked(bestTrackId, bestFrameNum, bestVideoPoint);
            }
            event->accept();
        }
    } else {
        QWidget::mousePressEvent(event);
    }
}

// mouseMoveEvent, mouseReleaseEvent, wheelEvent, resizeEvent (as in your provided file)
void VideoLoader::mouseMoveEvent(QMouseEvent *event) {
    QPointF currentPos = event->position(); QPointF delta = currentPos - m_lastMousePos;
    m_lastMousePos = currentPos; if (!isVideoLoaded()) { QWidget::mouseMoveEvent(event); return; }
    if (m_isPanning && (event->buttons() & Qt::LeftButton)) {
        m_panOffset += delta; clampPanOffset(); update(); event->accept();
    } else if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi && (event->buttons() & Qt::LeftButton)) {
        m_roiEndPointWidget = event->pos(); update(); event->accept();
    } else { QWidget::mouseMoveEvent(event); }
}
void VideoLoader::mouseReleaseEvent(QMouseEvent *event) {
    if (!isVideoLoaded()) { QWidget::mouseReleaseEvent(event); return; }
    if (event->button() == Qt::LeftButton) {
        if (m_isPanning) { m_isPanning = false; updateCursorShape(); event->accept(); }
        else if (m_isDefiningRoi && (m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop)) {
            m_roiEndPointWidget = event->pos(); m_isDefiningRoi = false;
            QPointF vs = mapPointToVideo(m_roiStartPointWidget), ve = mapPointToVideo(m_roiEndPointWidget);
            QRectF dRect; if(vs.x()>=0 && ve.x()>=0) { dRect = QRectF(vs,ve).normalized(); if(dRect.width()<5||dRect.height()<5) dRect=QRectF(); }
            if (m_currentMode == InteractionMode::DrawROI) { m_activeRoiRect = dRect; emit roiDefined(m_activeRoiRect); }
            else if (m_currentMode == InteractionMode::Crop) { if (!dRect.isNull()&&dRect.isValid()) handleRoiDefinedForCrop(dRect); else setInteractionMode(InteractionMode::PanZoom); }
            update(); updateCursorShape(); event->accept();
        }
    } else { QWidget::mouseReleaseEvent(event); }
}
void VideoLoader::wheelEvent(QWheelEvent *event) {
    if (!isVideoLoaded()) { event->ignore(); return; } // Changed: only process if PanZoom mode is NOT required
    int deg = event->angleDelta().y()/8; if(deg==0){event->ignore();return;} int steps=deg/15; double zs=0.15, mult=qPow(1.0+zs,steps);
    setZoomFactorAtPoint(m_zoomFactor*mult, event->position()); event->accept();
}
void VideoLoader::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event); if (isVideoLoaded()) { clampPanOffset(); update(); }
}

// --- Private Helper Methods (as in your provided file, ensure they are complete) ---
void VideoLoader::updateCursorShape() {
    if (!isVideoLoaded()) { setCursor(Qt::ArrowCursor); return; }
    switch (m_currentMode) {
    case InteractionMode::PanZoom: setCursor(m_isPanning ? Qt::ClosedHandCursor : Qt::OpenHandCursor); break;
    case InteractionMode::DrawROI: case InteractionMode::Crop: setCursor(Qt::CrossCursor); break;
    case InteractionMode::SelectWorms: setCursor(Qt::PointingHandCursor); break;
    case InteractionMode::ViewEditTracks: setCursor(Qt::ArrowCursor); break;
    default: setCursor(Qt::ArrowCursor); break;
    }
}
QRectF VideoLoader::calculateTargetRect() const {
    if (originalFrameSize.isEmpty() || m_zoomFactor <= 0) return QRectF();
    QSizeF ws = size(), imageSz = originalFrameSize, scaledSize = imageSz;
    scaledSize.scale(ws, Qt::KeepAspectRatio); QSizeF zoomedSize = scaledSize * m_zoomFactor;
    QPointF ctl((ws.width()-zoomedSize.width())/2.0, (ws.height()-zoomedSize.height())/2.0);
    return QRectF(ctl + m_panOffset, zoomedSize);
}
QPointF VideoLoader::mapPointToVideo(const QPointF& wp) const {
    if (currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0) return QPointF(-1,-1);
    QRectF tr=calculateTargetRect(); if(!tr.isValid()||tr.width()<=0||tr.height()<=0||!tr.contains(wp)) return QPointF(-1,-1);
    double nx=(wp.x()-tr.left())/tr.width(), ny=(wp.y()-tr.top())/tr.height();
    return QPointF(qBound(0.0,nx*originalFrameSize.width(),(qreal)originalFrameSize.width()), qBound(0.0,ny*originalFrameSize.height(),(qreal)originalFrameSize.height()));
}
QPointF VideoLoader::mapPointFromVideo(const QPointF& vp) const {
    if (currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0||originalFrameSize.width()<=0||originalFrameSize.height()<=0) return QPointF(-1,-1);
    QRectF tr=calculateTargetRect(); if(!tr.isValid()||tr.width()<=0||tr.height()<=0) return QPointF(-1,-1);
    QPointF cvp(qBound(0.0,vp.x(),(qreal)originalFrameSize.width()), qBound(0.0,vp.y(),(qreal)originalFrameSize.height()));
    double nx=cvp.x()/originalFrameSize.width(), ny=cvp.y()/originalFrameSize.height();
    return QPointF(tr.left()+nx*tr.width(), tr.top()+ny*tr.height());
}
void VideoLoader::clampPanOffset() {
    if (!isVideoLoaded() || m_zoomFactor <= 0) return;
    QRectF targetRect = calculateTargetRect(); QSizeF widgetSz = size();
    QPointF currentTopLeftNoPan = targetRect.topLeft() - m_panOffset;
    QPointF centerOffset((widgetSz.width() - targetRect.width()) / 2.0, (widgetSz.height() - targetRect.height()) / 2.0);
    if (targetRect.width() <= widgetSz.width()) m_panOffset.setX(centerOffset.x() - currentTopLeftNoPan.x());
    else { qreal minPanX = widgetSz.width() - targetRect.width() - currentTopLeftNoPan.x() - 50 ; qreal maxPanX = -currentTopLeftNoPan.x() + 50; m_panOffset.setX(qBound(minPanX, m_panOffset.x(), maxPanX)); }
    if (targetRect.height() <= widgetSz.height()) m_panOffset.setY(centerOffset.y() - currentTopLeftNoPan.y());
    else { qreal minPanY = widgetSz.height() - targetRect.height() - currentTopLeftNoPan.y() - 50; qreal maxPanY = -currentTopLeftNoPan.y() + 50; m_panOffset.setY(qBound(minPanY, m_panOffset.y(), maxPanY)); }
}
void VideoLoader::handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords) {
    if (cropRoiVideoCoords.isNull()||!cropRoiVideoCoords.isValid()){ setInteractionMode(InteractionMode::PanZoom); return; }
    if (QMessageBox::question(this, "Confirm Crop", "Crop video to ROI?", QMessageBox::Yes|QMessageBox::No) == QMessageBox::Yes) {
        emit videoProcessingStarted("Cropping video..."); QString croppedFilePath;
        if (performVideoCrop(cropRoiVideoCoords, croppedFilePath) && !croppedFilePath.isEmpty()) {
            emit videoProcessingFinished("Video cropped.",true); loadVideo(croppedFilePath);
        } else { emit videoProcessingFinished("Crop failed.",false); QMessageBox::critical(this,"Crop Error","Failed to crop video."); }
    } setInteractionMode(InteractionMode::PanZoom); m_isDefiningRoi=false; update();
}
bool VideoLoader::performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath) {
    if (currentFilePath.isEmpty()||!videoCapture.isOpened()||cropRectVideoCoords.isNull()||!cropRectVideoCoords.isValid()||cropRectVideoCoords.width()<=0||cropRectVideoCoords.height()<=0) return false;
    cv::VideoCapture origVid; if(!origVid.open(currentFilePath.toStdString())) return false;
    double origFps=origVid.get(cv::CAP_PROP_FPS); if(origFps<=0)origFps=framesPerSecond>0?framesPerSecond:25.0;
    cv::Rect cvCR(static_cast<int>(qRound(cropRectVideoCoords.x())),static_cast<int>(qRound(cropRectVideoCoords.y())), static_cast<int>(qRound(cropRectVideoCoords.width())),static_cast<int>(qRound(cropRectVideoCoords.height())));
    cvCR.x=qMax(0,cvCR.x); cvCR.y=qMax(0,cvCR.y);
    if(originalFrameSize.width()>0&&originalFrameSize.height()>0){ cvCR.width=qMin(cvCR.width,originalFrameSize.width()-cvCR.x); cvCR.height=qMin(cvCR.height,originalFrameSize.height()-cvCR.y); }
    if(cvCR.width<=0||cvCR.height<=0){origVid.release();return false;}
    cv::Size cfs(cvCR.width,cvCR.height); QFileInfo ofi(currentFilePath); QString bn=ofi.completeBaseName(), suff=ofi.suffix().isEmpty()?QString(DEFAULT_CROP_EXTENSION).remove(0,1):ofi.suffix();
    outCroppedFilePath = ofi.absolutePath()+"/"+bn+"_cropped."+suff; cv::VideoWriter writer; int fourcc = DEFAULT_CROP_FOURCC;
    if(!writer.open(outCroppedFilePath.toStdString(),fourcc,origFps,cfs,true)){origVid.release();return false;}
    cv::Mat frame,croppedF; int fCount=0;
    while(origVid.read(frame)){if(frame.empty())continue; try{croppedF=frame(cvCR);writer.write(croppedF);fCount++;}catch(const cv::Exception&){break;}}
    origVid.release();writer.release(); return fCount>0;
}
void VideoLoader::applyThresholding() {
    if (currentCvFrame.empty()) { m_thresholdedFrame_mono = cv::Mat(); return; }
    cv::Mat grayFrame; if (currentCvFrame.channels() >= 3) cv::cvtColor(currentCvFrame, grayFrame, cv::COLOR_BGR2GRAY); else grayFrame = currentCvFrame.clone();
    if (m_enableBlur && m_blurKernelSize >= 3) { try { cv::GaussianBlur(grayFrame, grayFrame, cv::Size(m_blurKernelSize, m_blurKernelSize), m_blurSigmaX); } catch (const cv::Exception& ex) { qWarning() << "GaussianBlur Exception:" << ex.what(); }}
    int type = m_assumeLightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    try { switch (m_thresholdAlgorithm) {
        case ThresholdAlgorithm::Global: cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255, type); break;
        case ThresholdAlgorithm::Otsu: cv::threshold(grayFrame, m_thresholdedFrame_mono, 0, 255, type | cv::THRESH_OTSU); break;
        case ThresholdAlgorithm::AdaptiveMean: if(m_adaptiveBlockSize>=3) cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255, cv::ADAPTIVE_THRESH_MEAN_C, type, m_adaptiveBlockSize, m_adaptiveC); else m_thresholdedFrame_mono=cv::Mat(); break;
        case ThresholdAlgorithm::AdaptiveGaussian: if(m_adaptiveBlockSize>=3) cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, type, m_adaptiveBlockSize, m_adaptiveC); else m_thresholdedFrame_mono=cv::Mat(); break;
        default: cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255, type); break;
        }} catch (const cv::Exception& ex) { qWarning() << "Thresholding Exception:" << ex.what(); m_thresholdedFrame_mono = cv::Mat(); }
}
void VideoLoader::updateTimerInterval() {
    if (framesPerSecond > 0 && m_playbackSpeedMultiplier > 0) playbackTimer->setInterval(qMax(1, qRound(1000.0 / (framesPerSecond * m_playbackSpeedMultiplier)))); else playbackTimer->setInterval(40);
}
void VideoLoader::emitThresholdParametersChanged() {
    emit thresholdParametersChanged(getCurrentThresholdSettings());
}

QColor VideoLoader::getTrackColor(int trackId) const {
    // Ensure m_trackColors is mutable if we want to cache here, or pre-populate.
    // For simplicity, let's assume m_trackColors can be modified.
    // If it's already in the cache, return it.
    if (m_trackColors.contains(trackId)) {
        return m_trackColors.value(trackId);
    }

    // Generate a new color if not cached.
    // Using QRandomGenerator for better random colors than simple hue cycling for many tracks.
    // Seed with trackId for some consistency if the same IDs appear across sessions,
    // though a global QRandomGenerator might be better for true randomness if needed.
    quint32 seed = static_cast<quint32>(trackId);
    QRandomGenerator generator(seed);

    // Generate HSV components for vibrant and distinct colors
    // Hue: 0-359
    // Saturation: High (e.g., 200-255) for vibrancy
    // Value (Brightness): Reasonably high (e.g., 200-255) to be visible on dark background
    int hue = generator.bounded(360);           // 0-359
    int saturation = 200 + generator.bounded(56); // 200-255
    int value = 200 + generator.bounded(56);      // 200-255

    QColor color = QColor::fromHsv(hue, saturation, value);

    // Cache the generated color - this requires m_trackColors to be non-const
    // or the function to be non-const, or use const_cast (which is not ideal).
    // Since m_trackColors is a member, and this function is const, we'd need to make m_trackColors mutable.
    // For now, let's assume m_trackColors was made mutable in the .h file.
    // If not, this line will cause a compile error.
    // const_cast<VideoLoader*>(this)->m_trackColors.insert(trackId, color); // One way if it must be const
    // A better way: make getTrackColor non-const, or m_trackColors mutable.
    // For this example, I'll assume m_trackColors is mutable as per the .h change.
    (const_cast<VideoLoader*>(this))->m_trackColors.insert(trackId, color);


    return color;
}
