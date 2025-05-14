#include "videoloader.h"    // Lowercase include

#include <QDebug>
#include <QPainterPath>
#include <QtMath>
#include <QResizeEvent>
#include <QMessageBox>
#include <QFileInfo>
#include <QRandomGenerator> // For generating distinct track colors

// Default crop parameters
#define DEFAULT_CROP_EXTENSION ".mp4"
#define DEFAULT_CROP_FOURCC cv::VideoWriter::fourcc('H', '2', '6', '4')

// Define a click tolerance for selecting track points (in widget pixels)
const qreal TRACK_POINT_CLICK_TOLERANCE = 5.0;


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
    setMouseTracking(true); // Enable for hover effects later if needed for tracks
    updateCursorShape();
}

VideoLoader::~VideoLoader() {
    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
}

// --- Public Methods & Getters ---
// (Existing getters: isVideoLoaded, getTotalFrames, getFPS, etc. are assumed to be implemented)
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


// --- Control Slots ---
// (loadVideo, play, pause, seekToFrame, setZoomFactor*, clearRoi, setPlaybackSpeed, threshold setters as before)
// ... (Implementations for these are assumed to be present and correct from previous versions) ...
bool VideoLoader::loadVideo(const QString &filePath) {
    if (m_isPlaying) pause();
    m_zoomFactor = 1.0; m_panOffset = QPointF(0.0, 0.0); m_activeRoiRect = QRectF();
    clearWormSelections(); clearDisplayedTracks(); // Clear tracks on new video
    m_isPanning = false; m_isDefiningRoi = false; m_playbackSpeedMultiplier = 1.0;
    m_showThresholdMask = false; m_thresholdAlgorithm = ThresholdAlgorithm::Global;
    m_thresholdValue = 127; m_assumeLightBackground = true; m_adaptiveBlockSize = 11;
    m_adaptiveC = 2.0; m_enableBlur = false; m_blurKernelSize = 5; m_blurSigmaX = 0.0;

    if (!openVideoFile(filePath)) { /* ... error handling ... */ return false; }
    currentFilePath = filePath; framesPerSecond = videoCapture.get(cv::CAP_PROP_FPS);
    if (framesPerSecond <= 0) framesPerSecond = 25.0;
    totalFramesCount = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    originalFrameSize = QSize(static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH)),
                              static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT)));
    if (totalFramesCount <= 0 || originalFrameSize.isEmpty()) { /* ... error handling ... */ return false; }
    seekToFrame(0);
    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor); emit roiDefined(m_activeRoiRect);
    emit playbackSpeedChanged(m_playbackSpeedMultiplier); emitThresholdParametersChanged();
    updateCursorShape(); qDebug() << "Video loaded:" << filePath; return true;
}
bool VideoLoader::openVideoFile(const QString &filePath) { /* ... as before ... */
    if (videoCapture.isOpened()) videoCapture.release();
    try { return videoCapture.open(filePath.toStdString()); }
    catch (const cv::Exception& ex) { qWarning() << "OpenCV exception:" << ex.what(); return false; }
}
void VideoLoader::play() { /* ... as before ... */
    if (!isVideoLoaded()) return; if (m_isPlaying) { pause(); }
    else { if (currentFrameIdx >= totalFramesCount - 1 && totalFramesCount > 0) seekToFrame(0);
        if (currentFrameIdx == -1 && totalFramesCount > 0) return;
        m_isPlaying = true; updateTimerInterval(); playbackTimer->start();
        emit playbackStateChanged(true, m_playbackSpeedMultiplier);
    }
}
void VideoLoader::pause() { /* ... as before ... */
    if (!m_isPlaying && !playbackTimer->isActive()) return;
    m_isPlaying = false; playbackTimer->stop();
    emit playbackStateChanged(false, m_playbackSpeedMultiplier);
}
void VideoLoader::seekToFrame(int frameNumber, bool suppressEmit) { /* ... as before ... */
    if (!isVideoLoaded()) return;
    frameNumber = qBound(0, frameNumber, totalFramesCount > 0 ? totalFramesCount - 1 : 0);
    displayFrame(frameNumber, suppressEmit);
}
void VideoLoader::setZoomFactor(double factor) { setZoomFactorAtPoint(factor, rect().center()); }
void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) { /* ... as before ... */
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
void VideoLoader::clearRoi() { /* ... as before ... */
    if (!m_activeRoiRect.isNull()) { m_activeRoiRect = QRectF(); update(); emit roiDefined(m_activeRoiRect); }
}
void VideoLoader::setPlaybackSpeed(double multiplier) { /* ... as before ... */
    double newSpeed = qBound(0.1, multiplier, 10.0);
    if (qFuzzyCompare(m_playbackSpeedMultiplier, newSpeed)) return;
    m_playbackSpeedMultiplier = newSpeed; if (m_isPlaying) { playbackTimer->stop(); updateTimerInterval(); playbackTimer->start(); }
    emit playbackSpeedChanged(m_playbackSpeedMultiplier); emit playbackStateChanged(m_isPlaying, m_playbackSpeedMultiplier);
}
void VideoLoader::clearWormSelections() { /* ... as before ... */
    if (!m_selectedCentroids_temp.isEmpty() || !m_selectedBounds_temp.isEmpty()) {
        m_selectedCentroids_temp.clear(); m_selectedBounds_temp.clear(); update();
    }
}
void VideoLoader::toggleThresholdView(bool enabled) { /* ... as before ... */
    if (m_showThresholdMask == enabled) return; m_showThresholdMask = enabled;
    if (isVideoLoaded() && currentFrameIdx >= 0) displayFrame(currentFrameIdx, true); update();
}
void VideoLoader::setThresholdAlgorithm(ThresholdAlgorithm algorithm) { /* ... as before ... */
    if (m_thresholdAlgorithm == algorithm) return; m_thresholdAlgorithm = algorithm;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setThresholdValue(int value) { /* ... as before ... */
    value = qBound(0, value, 255); if (m_thresholdValue == value) return; m_thresholdValue = value;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && m_thresholdAlgorithm == ThresholdAlgorithm::Global) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setAssumeLightBackground(bool isLight) { /* ... as before ... */
    if (m_assumeLightBackground == isLight) return; m_assumeLightBackground = isLight;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setAdaptiveThresholdBlockSize(int blockSize) { /* ... as before ... */
    if (blockSize < 3) blockSize = 3; if (blockSize % 2 == 0) blockSize += 1;
    if (m_adaptiveBlockSize == blockSize) return; m_adaptiveBlockSize = blockSize;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && (m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveMean || m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveGaussian)) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setAdaptiveThresholdC(double cValue) { /* ... as before ... */
    cValue = qBound(-50.0, cValue, 50.0); if (qFuzzyCompare(m_adaptiveC, cValue)) return;
    m_adaptiveC = cValue;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && (m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveMean || m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveGaussian)) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setEnableBlur(bool enabled) { /* ... as before ... */
    if (m_enableBlur == enabled) return; m_enableBlur = enabled;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >= 0) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setBlurKernelSize(int kernelSize) { /* ... as before ... */
    if (kernelSize < 3) kernelSize = 3; if (kernelSize % 2 == 0) kernelSize += 1;
    if (m_blurKernelSize == kernelSize) return; m_blurKernelSize = kernelSize;
    if (m_showThresholdMask && isVideoLoaded() && currentFrameIdx >= 0 && m_enableBlur) displayFrame(currentFrameIdx, true);
    emitThresholdParametersChanged(); update();
}
void VideoLoader::setBlurSigmaX(double sigmaX) { /* ... as before ... */
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
    // When changing mode, we don't automatically clear selected worms or tracks,
    // that should be an explicit action by the user or MainWindow if desired.
    updateCursorShape();
    emit interactionModeChanged(m_currentMode);
    qDebug() << "Interaction mode set to:" << static_cast<int>(m_currentMode);
    update(); // Update to reflect new cursor or drawing state
}


// --- New Slots for Track Display ---
void VideoLoader::setTracksToDisplay(const AllWormTracks& tracks) {
    m_allTracksToDisplay = tracks;
    m_trackColors.clear(); // Clear old color cache
    // Pre-assign colors or assign on demand in getTrackColor
    update(); // Trigger repaint
    qDebug() << "VideoLoader: Tracks set for display. Count:" << m_allTracksToDisplay.size();
}

void VideoLoader::setVisibleTrackIDs(const QSet<int>& visibleTrackIDs) {
    if (m_visibleTrackIDs == visibleTrackIDs) return;
    m_visibleTrackIDs = visibleTrackIDs;
    update(); // Trigger repaint
    qDebug() << "VideoLoader: Visible track IDs updated. Count:" << m_visibleTrackIDs.size();
}

void VideoLoader::clearDisplayedTracks() {
    m_allTracksToDisplay.clear();
    m_visibleTrackIDs.clear();
    m_trackColors.clear();
    update(); // Trigger repaint
    qDebug() << "VideoLoader: All displayed tracks cleared.";
}


// --- displayFrame, processNextFrame, convertCvMatToQImage (as before, ensure applyThresholding is always called in displayFrame) ---
void VideoLoader::displayFrame(int frameNumber, bool suppressEmit) {
    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) { /* ... */ return; }
    if (frameNumber < 0 || frameNumber >= totalFramesCount) { /* ... */ return; }
    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    if (currentPos != frameNumber && !(currentPos == frameNumber -1 && m_isPlaying) ) {
        if(!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) { /* ... */ }
    }
    if (videoCapture.read(currentCvFrame)) {
        if (!currentCvFrame.empty()) {
            currentFrameIdx = frameNumber;
            applyThresholding(); // Always update m_thresholdedFrame_mono
        } else {
            currentCvFrame = cv::Mat(); m_thresholdedFrame_mono = cv::Mat(); /* ... */
        }
    } else {
        currentCvFrame = cv::Mat(); m_thresholdedFrame_mono = cv::Mat(); /* ... */
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

void VideoLoader::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) { /* ... as before ... */
    if (mat.empty()) { qimg = QImage(); return; }
    if (mat.type() == CV_8UC3) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped(); }
    else if (mat.type() == CV_8UC1) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8); }
    else { cv::Mat temp; try { /* ... conversions ... */ } catch (const cv::Exception& ex) { /* ... */ qimg = QImage(); }}
}
void VideoLoader::processNextFrame() { /* ... as before ... */
    if (!m_isPlaying || !isVideoLoaded()) return;
    if (currentFrameIdx < totalFramesCount - 1) displayFrame(currentFrameIdx + 1); else pause();
}


// --- Event Handlers ---
void VideoLoader::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    // Draw video frame
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
        if (roiTopLeftWidget.x() >= 0 && roiBottomRightWidget.x() >= 0) {
            painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
            painter.drawRect(QRectF(roiTopLeftWidget, roiBottomRightWidget).normalized());
        }
    }

    // Draw ROI/Crop rectangle being defined
    if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi) {
        painter.setPen(QPen(Qt::cyan, 1, Qt::SolidLine));
        painter.drawRect(QRect(m_roiStartPointWidget, m_roiEndPointWidget).normalized());
    }

    // Draw selected worm centroids and bounding boxes for feedback (SelectWorms mode)
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

    // --- Draw Tracks (ViewEditTracks mode or always if tracks are set) ---
    // For now, let's draw if in ViewEditTracks mode and tracks are available.
    // You might want to draw them always if m_allTracksToDisplay is not empty,
    // regardless of mode, or have a separate toggle.
    if (m_currentMode == InteractionMode::ViewEditTracks && !m_allTracksToDisplay.empty()) {
        painter.setRenderHint(QPainter::Antialiasing, true); // Smoother lines for tracks

        for (int trackId : qAsConst(m_visibleTrackIDs)) {
            if (m_allTracksToDisplay.count(trackId)) { // Check if trackId exists in the map
                const std::vector<WormTrackPoint>& trackPoints = m_allTracksToDisplay.at(trackId);
                if (trackPoints.size() < 2) continue; // Need at least 2 points to draw a line

                QPainterPath path;
                QPointF prevPointWidget;

                // Get a color for this track
                QColor trackColor = getTrackColor(trackId);
                QPen trackPen(trackColor, 2); // Pen width 2
                painter.setPen(trackPen);

                for (size_t i = 0; i < trackPoints.size(); ++i) {
                    const WormTrackPoint& pt = trackPoints[i];
                    // Only draw points that are relevant to the current view (e.g., visible frames)
                    // For simplicity now, draw all points of visible tracks.
                    // Optimization: only draw segments if at least one end is on screen.

                    QPointF currentPointVideo(pt.position.x, pt.position.y);
                    QPointF currentPointWidget = mapPointFromVideo(currentPointVideo);

                    if (currentPointWidget.x() < 0) continue; // Skip if point is off-screen after mapping

                    if (i == 0) {
                        path.moveTo(currentPointWidget);
                    } else {
                        path.lineTo(currentPointWidget);
                    }
                    // Draw a small circle at each track point for better click targeting
                    painter.setBrush(trackColor); // Fill circles with track color
                    painter.drawEllipse(currentPointWidget, 2, 2); // Radius 2
                    painter.setBrush(Qt::NoBrush); // Reset brush
                }
                painter.strokePath(path, trackPen); // Draw the connected lines
            }
        }
    }
}


void VideoLoader::mousePressEvent(QMouseEvent *event) {
    m_lastMousePos = event->position();
    if (!isVideoLoaded()) { QWidget::mousePressEvent(event); return; }

    if (event->button() == Qt::LeftButton) {
        if (m_currentMode == InteractionMode::PanZoom) { /* ... as before ... */
            m_isPanning = true; updateCursorShape(); event->accept();
        } else if (m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) { /* ... as before ... */
            QPointF videoCoords = mapPointToVideo(event->position());
            if (videoCoords.x() >= 0) {
                m_roiStartPointWidget = event->pos(); m_roiEndPointWidget = event->pos();
                m_isDefiningRoi = true; update(); event->accept();
            } else { m_isDefiningRoi = false; }
        } else if (m_currentMode == InteractionMode::SelectWorms) { /* ... as before ... */
            if (!m_thresholdedFrame_mono.empty()) {
                QPointF clickVideoPoint = mapPointToVideo(event->position());
                if (clickVideoPoint.x() >= 0) {
                    TrackingHelper::DetectedBlob blob = TrackingHelper::findClickedBlob(m_thresholdedFrame_mono, clickVideoPoint);
                    if (blob.isValid) {
                        m_selectedCentroids_temp.append(blob.centroid);
                        m_selectedBounds_temp.append(blob.boundingBox);
                        emit wormBlobSelected(blob.centroid, blob.boundingBox);
                        update();
                    }
                }
            } event->accept();
        } else if (m_currentMode == InteractionMode::ViewEditTracks) {
            QPointF clickWidgetPoint = event->position();
            // Find the closest track point to the click
            int bestTrackId = -1;
            int bestFrameNum = -1;
            QPointF bestVideoPoint;
            double minDistanceSq = TRACK_POINT_CLICK_TOLERANCE * TRACK_POINT_CLICK_TOLERANCE;

            for (int trackId : qAsConst(m_visibleTrackIDs)) {
                if (m_allTracksToDisplay.count(trackId)) {
                    const auto& trackPoints = m_allTracksToDisplay.at(trackId);
                    for (const auto& pt : trackPoints) {
                        QPointF videoPt(pt.position.x, pt.position.y);
                        QPointF widgetPt = mapPointFromVideo(videoPt);
                        if (widgetPt.x() < 0) continue; // Off-screen

                        double dx = widgetPt.x() - clickWidgetPoint.x();
                        double dy = widgetPt.y() - clickWidgetPoint.y();
                        double distSq = dx * dx + dy * dy;

                        if (distSq < minDistanceSq) {
                            minDistanceSq = distSq;
                            bestTrackId = trackId;
                            bestFrameNum = pt.frameNumberOriginal;
                            bestVideoPoint = videoPt;
                        }
                    }
                }
            }

            if (bestTrackId != -1) {
                qDebug() << "Track point clicked: Worm ID" << bestTrackId << "Frame" << bestFrameNum;
                // Option 1: Emit a signal for MainWindow to handle
                emit trackPointClicked(bestTrackId, bestFrameNum, bestVideoPoint);
                // Option 2: Directly seek (if VideoLoader should control this directly)
                // seekToFrame(bestFrameNum);
            }
            event->accept();
        }
    } else {
        QWidget::mousePressEvent(event);
    }
}

// mouseMoveEvent, mouseReleaseEvent, wheelEvent, resizeEvent as before
// ... (ensure these functions are present from previous updates) ...
void VideoLoader::mouseMoveEvent(QMouseEvent *event) { /* ... as before ... */
    QPointF currentPos = event->position(); QPointF delta = currentPos - m_lastMousePos;
    m_lastMousePos = currentPos; if (!isVideoLoaded()) { QWidget::mouseMoveEvent(event); return; }
    if (m_isPanning && (event->buttons() & Qt::LeftButton)) {
        m_panOffset += delta; clampPanOffset(); update(); event->accept();
    } else if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi && (event->buttons() & Qt::LeftButton)) {
        m_roiEndPointWidget = event->pos(); update(); event->accept();
    } else { QWidget::mouseMoveEvent(event); }
}
void VideoLoader::mouseReleaseEvent(QMouseEvent *event) { /* ... as before ... */
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
void VideoLoader::wheelEvent(QWheelEvent *event) { /* ... as before ... */
    if (!isVideoLoaded() || m_currentMode != InteractionMode::PanZoom) { event->ignore(); return; }
    int deg = event->angleDelta().y()/8; if(deg==0){event->ignore();return;} int steps=deg/15; double zs=0.15, mult=qPow(1.0+zs,steps);
    setZoomFactorAtPoint(m_zoomFactor*mult, event->position()); event->accept();
}
void VideoLoader::resizeEvent(QResizeEvent *event) { /* ... as before ... */
    QWidget::resizeEvent(event); if (isVideoLoaded()) { clampPanOffset(); update(); }
}


// --- Private Helper Methods ---
// updateCursorShape, clampPanOffset, handleRoiDefinedForCrop, performVideoCrop, applyThresholding, updateTimerInterval, emitThresholdParametersChanged, calculateTargetRect, mapPointToVideo, mapPointFromVideo as before
// ... (ensure these functions are present from previous updates) ...
void VideoLoader::updateCursorShape() {
    if (!isVideoLoaded()) { setCursor(Qt::ArrowCursor); return; }
    switch (m_currentMode) {
    case InteractionMode::PanZoom: setCursor(m_isPanning ? Qt::ClosedHandCursor : Qt::OpenHandCursor); break;
    case InteractionMode::DrawROI: case InteractionMode::Crop: setCursor(Qt::CrossCursor); break;
    case InteractionMode::SelectWorms: setCursor(Qt::PointingHandCursor); break;
    case InteractionMode::ViewEditTracks: setCursor(Qt::ArrowCursor); break; // Or PointingHand if tracks are "hot"
    default: setCursor(Qt::ArrowCursor); break;
    }
}
QRectF VideoLoader::calculateTargetRect() const { /* ... as before ... */
    if (originalFrameSize.isEmpty() || m_zoomFactor <= 0) return QRectF();
    QSizeF ws = size(), imageSz = originalFrameSize, scaledSize = imageSz;
    scaledSize.scale(ws, Qt::KeepAspectRatio); QSizeF zoomedSize = scaledSize * m_zoomFactor;
    QPointF ctl((ws.width()-zoomedSize.width())/2.0, (ws.height()-zoomedSize.height())/2.0);
    return QRectF(ctl + m_panOffset, zoomedSize);
}
QPointF VideoLoader::mapPointToVideo(const QPointF& wp) const { /* ... as before ... */
    if (currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0) return QPointF(-1,-1);
    QRectF tr=calculateTargetRect(); if(!tr.isValid()||tr.width()<=0||tr.height()<=0||!tr.contains(wp)) return QPointF(-1,-1);
    double nx=(wp.x()-tr.left())/tr.width(), ny=(wp.y()-tr.top())/tr.height();
    return QPointF(qBound(0.0,nx*originalFrameSize.width(),(qreal)originalFrameSize.width()), qBound(0.0,ny*originalFrameSize.height(),(qreal)originalFrameSize.height()));
}
QPointF VideoLoader::mapPointFromVideo(const QPointF& vp) const { /* ... as before ... */
    if (currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0||originalFrameSize.width()<=0||originalFrameSize.height()<=0) return QPointF(-1,-1);
    QRectF tr=calculateTargetRect(); if(!tr.isValid()||tr.width()<=0||tr.height()<=0) return QPointF(-1,-1);
    QPointF cvp(qBound(0.0,vp.x(),(qreal)originalFrameSize.width()), qBound(0.0,vp.y(),(qreal)originalFrameSize.height()));
    double nx=cvp.x()/originalFrameSize.width(), ny=cvp.y()/originalFrameSize.height();
    return QPointF(tr.left()+nx*tr.width(), tr.top()+ny*tr.height());
}
void VideoLoader::clampPanOffset() { /* ... as before ... */
    if (!isVideoLoaded() || m_zoomFactor <= 0) return;
    QRectF targetRect = calculateTargetRect(); QSizeF widgetSz = size();
    QPointF currentTopLeftNoPan = targetRect.topLeft() - m_panOffset;
    QPointF centerOffset((widgetSz.width() - targetRect.width()) / 2.0, (widgetSz.height() - targetRect.height()) / 2.0);
    if (targetRect.width() <= widgetSz.width()) m_panOffset.setX(centerOffset.x() - currentTopLeftNoPan.x());
    else { qreal minPanX = widgetSz.width() - targetRect.width() - currentTopLeftNoPan.x() - 50 ; qreal maxPanX = -currentTopLeftNoPan.x() + 50; m_panOffset.setX(qBound(minPanX, m_panOffset.x(), maxPanX)); }
    if (targetRect.height() <= widgetSz.height()) m_panOffset.setY(centerOffset.y() - currentTopLeftNoPan.y());
    else { qreal minPanY = widgetSz.height() - targetRect.height() - currentTopLeftNoPan.y() - 50; qreal maxPanY = -currentTopLeftNoPan.y() + 50; m_panOffset.setY(qBound(minPanY, m_panOffset.y(), maxPanY)); }
}
void VideoLoader::handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords) { /* ... as before ... */
    if (cropRoiVideoCoords.isNull()||!cropRoiVideoCoords.isValid()){ setInteractionMode(InteractionMode::PanZoom); return; }
    if (QMessageBox::question(this, "Confirm Crop", "Crop video to ROI?", QMessageBox::Yes|QMessageBox::No) == QMessageBox::Yes) {
        emit videoProcessingStarted("Cropping video..."); QString croppedFilePath;
        if (performVideoCrop(cropRoiVideoCoords, croppedFilePath) && !croppedFilePath.isEmpty()) {
            emit videoProcessingFinished("Video cropped.",true); loadVideo(croppedFilePath);
        } else { emit videoProcessingFinished("Crop failed.",false); QMessageBox::critical(this,"Crop Error","Failed to crop video."); }
    } setInteractionMode(InteractionMode::PanZoom); m_isDefiningRoi=false; update();
}
bool VideoLoader::performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath) { /* ... as before ... */
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
void VideoLoader::applyThresholding() { /* ... as before ... */
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
void VideoLoader::updateTimerInterval() { /* ... as before ... */
    if (framesPerSecond > 0 && m_playbackSpeedMultiplier > 0) playbackTimer->setInterval(qMax(1, qRound(1000.0 / (framesPerSecond * m_playbackSpeedMultiplier)))); else playbackTimer->setInterval(40);
}
void VideoLoader::emitThresholdParametersChanged() { /* ... as before ... */
    emit thresholdParametersChanged(getCurrentThresholdSettings());
}

QColor VideoLoader::getTrackColor(int trackId) const {
    if (m_trackColors.contains(trackId)) {
        return m_trackColors.value(trackId);
    }
    // Generate a new color if not cached - simple hue cycling
    // For more distinct colors, you might use a predefined list or a more sophisticated generator.
    // QRandomGenerator is better than rand() for this.
    // Using a static generator to get different colors across calls if this function is called multiple times for new IDs.
    // However, for consistency across runs for the same ID, caching is better.
    // If m_trackColors is mutable, we can add it here.
    // For now, let's make it const and assume colors are pre-assigned or this is just for demo.
    // A better approach would be for MainWindow to assign colors and pass them with the tracks.
    // For a quick solution:
    const int hueStep = 360 / 12; // Cycle through 12 distinct hues
    int hue = (trackId * hueStep) % 360;
    // For more distinctness, vary saturation and lightness slightly too
    int saturation = 200 + (trackId * 10 % 55); // 200-255
    int lightness = 128 + (trackId * 5 % 100);  // 128-228 (avoid too dark/light)
    QColor color = QColor::fromHsv(hue, qBound(0,saturation,255), qBound(0,lightness,255));
    // To cache (requires m_trackColors to be non-const or use a const_cast if really needed, not ideal):
    // const_cast<VideoLoader*>(this)->m_trackColors.insert(trackId, color);
    return color;
}

