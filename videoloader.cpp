#include "videoloader.h"
#include "wormtracker.h" // Include the new WormTracker header
#include "worm.h"        // Include the new Worm header

#include <QDebug>
#include <QPainterPath>
#include <QtMath>
#include <QResizeEvent>
#include <QMessageBox>
#include <QFileInfo>
#include <vector> // For cv::findContours

// Default crop parameters
#define DEFAULT_CROP_EXTENSION ".mp4"
#define DEFAULT_CROP_FOURCC cv::VideoWriter::fourcc('H', '2', '6', '4')

VideoLoader::VideoLoader(QWidget *parent)
    : QWidget(parent),
    playbackTimer(new QTimer(this)),
    totalFramesCount(0),
    framesPerSecond(0.0),
    currentFrameIdx(-1), // Will represent the keyframe for tracking
    m_isPlaying(false),
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
    m_numberOfWormsToSelect(0), // Initialize
    m_wormsSelectedCount(0),
    m_allTrackingTasksCompleted(false)
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
    stopAllTracking(); // Ensure threads are stopped and cleaned up
    qDeleteAll(m_wormTrackers);
    m_wormTrackers.clear();
    qDeleteAll(m_selectedWormsData); // Worm data objects are owned by WormTrackers after selection
    m_selectedWormsData.clear();

    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
}

// --- Public Methods ---
// ... (Getters remain mostly the same) ...
bool VideoLoader::isVideoLoaded() const { return videoCapture.isOpened() && totalFramesCount > 0; }
int VideoLoader::getTotalFrames() const { return totalFramesCount; }
double VideoLoader::getFPS() const { return framesPerSecond; }
int VideoLoader::getCurrentFrameNumber() const { return currentFrameIdx; } // This is the keyframe
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

QVariantMap VideoLoader::getCurrentThresholdParameters() const {
    QVariantMap params;
    params["algorithm"] = static_cast<int>(m_thresholdAlgorithm);
    params["value"] = m_thresholdValue;
    params["lightBackground"] = m_assumeLightBackground;
    params["adaptiveBlockSize"] = m_adaptiveBlockSize;
    params["adaptiveC"] = m_adaptiveC;
    return params;
}


// --- Slots ---
bool VideoLoader::loadVideo(const QString &filePath) {
    stopAllTracking(); // Stop any previous tracking
    qDeleteAll(m_wormTrackers);
    m_wormTrackers.clear();
    qDeleteAll(m_selectedWormsData);
    m_selectedWormsData.clear();
    m_wormsSelectedCount = 0;
    m_allTrackingTasksCompleted = false;


    if (m_isPlaying) {
        pause();
    }

    m_zoomFactor = 1.0;
    m_panOffset = QPointF(0.0, 0.0);
    m_activeRoiRect = QRectF();
    m_isPanning = false;
    m_isDefiningRoi = false;

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

    // When a new video is loaded, set currentFrameIdx to a sensible default (e.g., 0 or middle)
    // This frame will be the one displayed for worm selection if that mode is entered.
    seekToFrame(0); // Or totalFramesCount / 2 for a middle frame

    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor);
    emit roiDefined(m_activeRoiRect);
    updateCursorShape();
    qDebug() << "Video loaded:" << filePath << "Frames:" << totalFramesCount << "FPS:" << framesPerSecond;
    return true;
}

// ... (openVideoFile, play, pause, seekToFrame, displayFrame, convertCvMatToQImage, processNextFrame are largely the same as v7)
bool VideoLoader::openVideoFile(const QString &filePath) {
    if (videoCapture.isOpened()) { videoCapture.release(); }
    try { if (!videoCapture.open(filePath.toStdString())) { qWarning() << "OpenCV: Failed to open video" << filePath; return false; }
    } catch (const cv::Exception& ex) { qWarning() << "OpenCV exception opening video:" << filePath << "Error:" << ex.what(); return false; }
    return videoCapture.isOpened();
}
void VideoLoader::play() {
    if (!isVideoLoaded()) return;
    if (m_isPlaying) { pause(); }
    else {
        if (currentFrameIdx >= totalFramesCount - 1 && totalFramesCount > 0) { seekToFrame(0); if (currentFrameIdx == -1 && totalFramesCount > 0) return; }
        m_isPlaying = true; int interval = (framesPerSecond > 0) ? qRound(1000.0 / framesPerSecond) : 40;
        playbackTimer->start(interval); emit playbackStateChanged(true); qDebug() << "Playback started.";
    }
}
void VideoLoader::pause() {
    if (!m_isPlaying && !playbackTimer->isActive()) return;
    m_isPlaying = false; playbackTimer->stop(); emit playbackStateChanged(false); qDebug() << "Playback paused.";
}
void VideoLoader::seekToFrame(int frameNumber, bool suppressEmit) {
    if (!isVideoLoaded()) return;
    frameNumber = qBound(0, frameNumber, totalFramesCount > 0 ? totalFramesCount - 1 : 0);
    // When seeking, this frame becomes the new potential keyframe for worm selection
    // if the user enters SelectWorms mode.
    displayFrame(frameNumber, suppressEmit);
}
void VideoLoader::displayFrame(int frameNumber, bool suppressEmit) {
    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) { /* ... */ return; }
    if (frameNumber < 0 || frameNumber >= totalFramesCount) { /* ... */ return; }
    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    if (currentPos != frameNumber) { if(!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) { /* ... */ }}
    if (videoCapture.read(currentCvFrame)) {
        if (!currentCvFrame.empty()) {
            // currentFrameIdx = frameNumber; // Set by seekToFrame or processNextFrame
            if (m_showThresholdMask) { applyThresholding(); }
        } else { currentCvFrame = cv::Mat(); m_thresholdedFrame_mono = cv::Mat(); /* ... */ }
    } else { currentCvFrame = cv::Mat(); m_thresholdedFrame_mono = cv::Mat(); /* ... */ if (m_isPlaying) pause(); }
    if (m_showThresholdMask && !m_thresholdedFrame_mono.empty()) { convertCvMatToQImage(m_thresholdedFrame_mono, currentQImageFrame); }
    else if (!currentCvFrame.empty()) { convertCvMatToQImage(currentCvFrame, currentQImageFrame); }
    else { currentQImageFrame = QImage(); }
    currentFrameIdx = frameNumber; // Ensure this is the displayed frame index
    if (!suppressEmit) { emit frameChanged(currentFrameIdx, currentQImageFrame); }
    update();
}
void VideoLoader::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) { /* ... (same as v7) ... */
    if (mat.empty()) { qimg = QImage(); return; }
    if (mat.type() == CV_8UC3) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped(); }
    else if (mat.type() == CV_8UC1) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8); }
    else { cv::Mat temp; try {
            if (mat.channels() == 4) { cv::cvtColor(mat, temp, cv::COLOR_BGRA2BGR); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 3 && mat.type() != CV_8UC3) { mat.convertTo(temp, CV_8UC3, 255.0); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 1 && mat.type() != CV_8UC1) { mat.convertTo(temp, CV_8UC1, 255.0); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_Grayscale8); }
            else { qWarning() << "Unsupported cv::Mat type for conversion:" << mat.type(); qimg = QImage(); return; }
        } catch (const cv::Exception& ex) { qWarning() << "OpenCV exception during color conversion:" << ex.what(); qimg = QImage(); }}
}
void VideoLoader::processNextFrame() { if (!m_isPlaying || !isVideoLoaded()) return; if (currentFrameIdx < totalFramesCount - 1) displayFrame(currentFrameIdx + 1); else pause(); }


void VideoLoader::setZoomFactor(double factor) { setZoomFactorAtPoint(factor, rect().center()); }
void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) { /* ... (same as v7) ... */
    if (!isVideoLoaded()) return; double nZ = qBound(0.05,factor,50.0); if(qFuzzyCompare(m_zoomFactor,nZ))return;
    QPointF vp=mapPointToVideo(widgetPoint),rpw=widgetPoint; if(vp.x()<0){rpw=rect().center();vp=mapPointToVideo(rpw);if(vp.x()<0)return;}
    m_zoomFactor=nZ; QPointF wap=mapPointFromVideo(vp); m_panOffset+=(rpw-wap); clampPanOffset();update();emit zoomFactorChanged(m_zoomFactor);
}

void VideoLoader::setInteractionMode(InteractionMode mode) {
    if (m_currentMode == mode) return;

    // Reset states from previous mode if necessary
    m_isPanning = false;
    m_isDefiningRoi = false;

    m_currentMode = mode;

    if (m_currentMode == InteractionMode::SelectWorms) {
        if (!m_showThresholdMask) {
            // Automatically turn on threshold view if not already on for worm selection
            // Or, you could prompt the user or make it a prerequisite.
            toggleThresholdView(true);
            qInfo() << "Threshold view enabled for worm selection.";
        }
        m_wormsSelectedCount = 0; // Reset count for new selection session
        qDeleteAll(m_selectedWormsData); // Clear any previously selected temp data
        m_selectedWormsData.clear();
        if (m_numberOfWormsToSelect <= 0) {
            qWarning() << "SelectWorms mode entered, but number of worms to select is not set or invalid.";
            // Potentially revert to PanZoom or show a message
        } else {
            qInfo() << "SelectWorms mode: Click on" << m_numberOfWormsToSelect << "worms. Selected:" << m_wormsSelectedCount;
        }
    }

    updateCursorShape();
    emit interactionModeChanged(m_currentMode);
    qDebug() << "Interaction mode set to:" << static_cast<int>(m_currentMode);
}

void VideoLoader::clearRoi() { /* ... (same as v7) ... */
    if (!m_activeRoiRect.isNull()) { m_activeRoiRect=QRectF(); update(); emit roiDefined(m_activeRoiRect); qDebug()<<"Active ROI cleared.";}
}

// --- Thresholding Control Slots ---
// ... (toggleThresholdView, setThresholdAlgorithm, etc. are the same as v7) ...
void VideoLoader::toggleThresholdView(bool enabled) {
    if(m_showThresholdMask == enabled) return; m_showThresholdMask = enabled;
    qDebug()<<"Threshold view" << (enabled?"enabled":"disabled");
    if(isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx, true); update();
}
void VideoLoader::setThresholdAlgorithm(ThresholdAlgorithm algo){
    if(m_thresholdAlgorithm == algo) return; m_thresholdAlgorithm = algo;
    if(m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx, true);
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC); update();
}
void VideoLoader::setThresholdValue(int val){
    val = qBound(0,val,255); if(m_thresholdValue==val)return; m_thresholdValue=val;
    if(m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && m_thresholdAlgorithm == ThresholdAlgorithm::Global) displayFrame(currentFrameIdx,true);
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC); update();
}
void VideoLoader::setAssumeLightBackground(bool lightBg){
    if(m_assumeLightBackground == lightBg) return; m_assumeLightBackground = lightBg;
    if(m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0) displayFrame(currentFrameIdx,true);
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC); update();
}
void VideoLoader::setAdaptiveThresholdBlockSize(int bs){
    if(bs<3)bs=3; if(bs%2==0)bs++; if(m_adaptiveBlockSize==bs)return; m_adaptiveBlockSize=bs;
    if(m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && (m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveMean || m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveGaussian)) displayFrame(currentFrameIdx,true);
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC); update();
}
void VideoLoader::setAdaptiveThresholdC(double cVal){
    if(qFuzzyCompare(m_adaptiveC,cVal))return; m_adaptiveC=cVal;
    if(m_showThresholdMask && isVideoLoaded() && currentFrameIdx >=0 && (m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveMean || m_thresholdAlgorithm == ThresholdAlgorithm::AdaptiveGaussian)) displayFrame(currentFrameIdx,true);
    emit thresholdParametersChanged(m_thresholdAlgorithm, m_thresholdValue, m_assumeLightBackground, m_adaptiveBlockSize, m_adaptiveC); update();
}


// --- Worm Tracking Control Slots ---
void VideoLoader::setNumberOfWormsToSelect(int num) {
    if (num <= 0) {
        qWarning() << "Number of worms must be positive. Value not set:" << num;
        m_numberOfWormsToSelect = 0;
        return;
    }
    m_numberOfWormsToSelect = num;
    qDebug() << "Number of worms to select set to:" << m_numberOfWormsToSelect;
    // If already in SelectWorms mode, update user or reset selection count
    if (m_currentMode == InteractionMode::SelectWorms) {
        m_wormsSelectedCount = 0;
        qDeleteAll(m_selectedWormsData);
        m_selectedWormsData.clear();
        qInfo() << "Number of worms updated. Reselect worms. Click on" << m_numberOfWormsToSelect << "worms.";
    }
}

void VideoLoader::startTrackingSelectedWorms() {
    if (m_selectedWormsData.count() != m_numberOfWormsToSelect || m_numberOfWormsToSelect <= 0) {
        QMessageBox::warning(this, "Tracking Error",
                             QString("Cannot start tracking. Expected %1 worms, but %2 selected.")
                                 .arg(m_numberOfWormsToSelect).arg(m_selectedWormsData.count()));
        return;
    }
    if (currentFilePath.isEmpty() || currentFrameIdx < 0) {
        QMessageBox::warning(this, "Tracking Error", "No video loaded or keyframe not set for tracking.");
        return;
    }

    stopAllTracking(); // Stop any existing trackers
    qDeleteAll(m_wormTrackers);
    m_wormTrackers.clear();
    m_allTrackingTasksCompleted = false;

    qInfo() << "Starting tracking for" << m_selectedWormsData.count() << "worms from keyframe" << currentFrameIdx;
    emit videoProcessingStarted(QString("Initializing tracking for %1 worms...").arg(m_selectedWormsData.count()));

    QVariantMap trackingParams = getCurrentThresholdParameters(); // Pass current threshold settings

    for (Worm* wormData : m_selectedWormsData) {
        // WormTracker takes ownership of wormData if you design it that way,
        // or VideoLoader keeps ownership of m_selectedWormsData and WormTracker just references.
        // For now, let's assume WormTracker copies or takes essential info.
        // Or, better, WormTracker takes ownership.
        WormTracker* tracker = new WormTracker(wormData, currentFilePath, currentFrameIdx, trackingParams, this); // Parent to VideoLoader for auto-cleanup

        connect(tracker, &WormTracker::wormTrackerFinished, this, &VideoLoader::onWormTrackerFinished);
        connect(tracker, &WormTracker::wormTrackerError, this, &VideoLoader::onWormTrackerError);
        connect(tracker, &WormTracker::newDisplayableCrop, this, &VideoLoader::onWormTrackerDisplayUpdate);
        // connect(tracker, &WormTracker::trackUpdated, this, ...); // For drawing tracks on main display

        m_wormTrackers.append(tracker);
        tracker->startTracking(); // Start its forward/backward threads
    }
    m_selectedWormsData.clear(); // Data now managed by WormTrackers (or should be copied)

    setInteractionMode(InteractionMode::PanZoom); // Revert to pan/zoom after starting tracking
    // Do not clear m_wormsSelectedCount here, it's reset when entering SelectWorms mode.
}

void VideoLoader::stopAllTracking() {
    qInfo() << "Stopping all worm tracking threads...";
    for (WormTracker* tracker : m_wormTrackers) {
        tracker->stopTracking();
    }
    // Threads will take a moment to stop. Wait for them if necessary, or handle cleanup via finished signals.
    // For now, we assume they stop relatively quickly.
    // qDeleteAll(m_wormTrackers); // This should happen after threads have truly finished.
    // m_wormTrackers.clear();
    // Consider clearing display elements related to tracking here.
}


// --- Event Handlers ---
void VideoLoader::paintEvent(QPaintEvent *event) { /* ... (same as v7, consider drawing selected worm markers) ... */
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    if (currentQImageFrame.isNull() || !isVideoLoaded()) { /* ... */ painter.drawText(rect(), Qt::AlignCenter, "No video loaded."); return; }
    QRectF targetRect = calculateTargetRect();
    painter.drawImage(targetRect, currentQImageFrame, currentQImageFrame.rect());

    if (m_currentMode == InteractionMode::DrawROI && !m_activeRoiRect.isNull() && m_activeRoiRect.isValid()) { /* ... draw m_activeRoiRect ... */
        QPointF tlW = mapPointFromVideo(m_activeRoiRect.topLeft()), brW = mapPointFromVideo(m_activeRoiRect.bottomRight());
        if(tlW.x()>=0 && brW.x()>=0) { painter.setPen(QPen(Qt::red,1,Qt::DashLine)); painter.drawRect(QRectF(tlW,brW).normalized()); }
    }
    if ((m_currentMode == InteractionMode::DrawROI || m_currentMode == InteractionMode::Crop) && m_isDefiningRoi) { /* ... draw temp ROI ... */
        painter.setPen(QPen(Qt::cyan,1,Qt::SolidLine)); painter.drawRect(QRect(m_roiStartPointWidget, m_roiEndPointWidget).normalized());
    }

    // Draw markers for selected worms in SelectWorms mode
    if (m_currentMode == InteractionMode::SelectWorms && isVideoLoaded()) {
        for (const Worm* wormData : m_selectedWormsData) {
            QPointF videoCentroid = wormData->initialCentroid();
            QPointF widgetCentroid = mapPointFromVideo(videoCentroid);
            if (widgetCentroid.x() >= 0) { // Check if it's on screen
                painter.setPen(QPen(wormData->trackColor(), 2));
                painter.setBrush(Qt::NoBrush);
                painter.drawEllipse(widgetCentroid, 5, 5); // Draw a small circle
                painter.drawText(widgetCentroid + QPointF(7, 7), QString::number(wormData->id()));
            }
        }
        // Indicate how many more worms to select
        painter.setPen(Qt::white);
        painter.drawText(10, 20, QString("Select %1 more worm(s). Click on a blob.").arg(m_numberOfWormsToSelect - m_wormsSelectedCount));

    }
    // TODO: Add drawing of worm tracks if tracking is active
}
QRectF VideoLoader::calculateTargetRect() const { /* ... (same as v7) ... */
    if(originalFrameSize.isEmpty())return QRectF(); QSizeF ws=size(),ofs=originalFrameSize,cs=ofs; cs.scale(ws,Qt::KeepAspectRatio);
    QSizeF zs=cs*m_zoomFactor; QPointF ctl((ws.width()-zs.width())/2.0,(ws.height()-zs.height())/2.0); return QRectF(ctl+m_panOffset,zs);
}
QPointF VideoLoader::mapPointToVideo(const QPointF& wp) const { /* ... (same as v7) ... */
    if(currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0)return QPointF(-1,-1); QRectF tr=calculateTargetRect();
    if(!tr.contains(wp)||tr.width()<=0||tr.height()<=0)return QPointF(-1,-1); double nx=(wp.x()-tr.left())/tr.width(),ny=(wp.y()-tr.top())/tr.height();
    QSizeF ofs=originalFrameSize; return QPointF(qBound(0.0,nx*ofs.width(),ofs.width()),qBound(0.0,ny*ofs.height(),ofs.height()));
}
QPointF VideoLoader::mapPointFromVideo(const QPointF& vp) const { /* ... (same as v7) ... */
    if(currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0||originalFrameSize.width()<=0||originalFrameSize.height()<=0)return QPointF(-1,-1);
    QRectF tr=calculateTargetRect();if(tr.width()<=0||tr.height()<=0)return QPointF(-1,-1); QSizeF ofs=originalFrameSize;
    QPointF cvp(qBound(0.0,vp.x(),ofs.width()),qBound(0.0,vp.y(),ofs.height())); double nx=cvp.x()/ofs.width(),ny=cvp.y()/ofs.height();
    return QPointF(tr.left()+nx*tr.width(),tr.top()+ny*tr.height());
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
                m_roiStartPointWidget = event->pos(); m_roiEndPointWidget = event->pos(); m_isDefiningRoi = true;
                update(); qDebug() << (m_currentMode == InteractionMode::Crop ? "Crop" : "ROI") << "definition started."; event->accept();
            } else { m_isDefiningRoi = false; }
        } else if (m_currentMode == InteractionMode::SelectWorms) {
            if (m_wormsSelectedCount < m_numberOfWormsToSelect) {
                handleWormSelection(event->position());
            } else {
                qInfo() << "All" << m_numberOfWormsToSelect << "worms already selected. Start tracking or change mode.";
            }
            event->accept();
        }
    } else {
        QWidget::mousePressEvent(event);
    }
}
void VideoLoader::mouseMoveEvent(QMouseEvent *event) { /* ... (same as v7) ... */
    QPointF cp=event->position(), d=cp-m_lastMousePos; if(!isVideoLoaded()){QWidget::mouseMoveEvent(event);return;}
    if(m_isPanning&&(event->buttons()&Qt::LeftButton)){m_panOffset+=d;clampPanOffset();update();event->accept();}
    else if((m_currentMode==InteractionMode::DrawROI||m_currentMode==InteractionMode::Crop)&&m_isDefiningRoi&&(event->buttons()&Qt::LeftButton))
    {m_roiEndPointWidget=event->pos();update();event->accept();}
    else QWidget::mouseMoveEvent(event); m_lastMousePos=cp;
}
void VideoLoader::mouseReleaseEvent(QMouseEvent *event) { /* ... (same as v7, calls handleRoiDefinedForCrop) ... */
    if(!isVideoLoaded()){QWidget::mouseReleaseEvent(event);return;}
    if(event->button()==Qt::LeftButton){
        if(m_isPanning){m_isPanning=false;updateCursorShape();event->accept();}
        else if(m_isDefiningRoi&&(m_currentMode==InteractionMode::DrawROI||m_currentMode==InteractionMode::Crop)){
            m_roiEndPointWidget=event->pos();m_isDefiningRoi=false; QPointF vs=mapPointToVideo(m_roiStartPointWidget),ve=mapPointToVideo(m_roiEndPointWidget);
            QRectF dr; if(vs.x()>=0&&ve.x()>=0){dr=QRectF(vs,ve).normalized();if(dr.width()<5||dr.height()<5)dr=QRectF();}
            if(m_currentMode==InteractionMode::DrawROI){m_activeRoiRect=dr;emit roiDefined(m_activeRoiRect);qDebug()<<"ROI end:"<<m_activeRoiRect;}
            else if(m_currentMode==InteractionMode::Crop){if(!dr.isNull()&&dr.isValid())handleRoiDefinedForCrop(dr); else {qDebug()<<"Crop area invalid.";setInteractionMode(InteractionMode::PanZoom);}}
            update();updateCursorShape();event->accept();
        } // No else if for SelectWorms, as selection happens on press
    } else QWidget::mouseReleaseEvent(event);
}
void VideoLoader::wheelEvent(QWheelEvent *event) { /* ... (same as v7) ... */
    if(!isVideoLoaded()||m_currentMode!=InteractionMode::PanZoom){event->ignore();return;}
    int deg=event->angleDelta().y()/8;if(deg==0){event->ignore();return;}int steps=deg/15;double zs=0.15,mult=qPow(1.0+zs,steps);
    double nz=qBound(0.05,m_zoomFactor*mult,50.0);if(qFuzzyCompare(m_zoomFactor,nz)){event->accept();return;}
    QPointF mpw=event->position(),vp=mapPointToVideo(mpw),rpw=mpw; if(vp.x()<0){rpw=rect().center();vp=mapPointToVideo(rpw);if(vp.x()<0){event->ignore();return;}}
    m_zoomFactor=nz;m_panOffset+=(rpw-mapPointFromVideo(vp));clampPanOffset();update();emit zoomFactorChanged(m_zoomFactor);event->accept();
}
void VideoLoader::resizeEvent(QResizeEvent *event) { /* ... (same as v7) ... */
    QWidget::resizeEvent(event); if(isVideoLoaded()){clampPanOffset();update();}
}

// --- Private Helper Methods ---
void VideoLoader::updateCursorShape() { /* ... (add SelectWorms case) ... */
    if (!isVideoLoaded()) { setCursor(Qt::ArrowCursor); return; }
    switch (m_currentMode) {
    case InteractionMode::PanZoom: setCursor(m_isPanning ? Qt::ClosedHandCursor : Qt::OpenHandCursor); break;
    case InteractionMode::DrawROI: case InteractionMode::Crop: setCursor(Qt::CrossCursor); break;
    case InteractionMode::SelectWorms: setCursor(Qt::PointingHandCursor); break; // Or a target cursor
    default: setCursor(Qt::ArrowCursor); break;
    }
}
void VideoLoader::clampPanOffset() { /* ... (same as v7) ... */
    if(!isVideoLoaded()||m_zoomFactor<=0)return;QRectF tr=calculateTargetRect();QSizeF ws=size();
    double mx=-(tr.width()-50.0),my=-(tr.height()-50.0),mX=ws.width()-50.0,mY=ws.height()-50.0;
    QPointF cftl=tr.topLeft(),clftl;clftl.setX(qBound(mx,cftl.x(),mX));clftl.setY(qBound(my,cftl.y(),mY));
    if(tr.width()<=ws.width())clftl.setX((ws.width()-tr.width())/2.0);if(tr.height()<=ws.height())clftl.setY((ws.height()-tr.height())/2.0);
    QSizeF ofs=originalFrameSize,cs=ofs;cs.scale(ws,Qt::KeepAspectRatio);QSizeF zs=cs*m_zoomFactor;
    QPointF ctl_c((ws.width()-zs.width())/2.0,(ws.height()-zs.height())/2.0);
    QPointF rpo=clftl-ctl_c;if(!qFuzzyCompare(m_panOffset.x(),rpo.x())||!qFuzzyCompare(m_panOffset.y(),rpo.y()))m_panOffset=rpo;
}

void VideoLoader::handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords) { /* ... (same as v7) ... */
    if(cropRoiVideoCoords.isNull()||!cropRoiVideoCoords.isValid()){setInteractionMode(InteractionMode::PanZoom);return;}
    QMessageBox::StandardButton r=QMessageBox::question(this,"Confirm Crop","Crop video?",QMessageBox::Yes|QMessageBox::No);
    if(r==QMessageBox::Yes){emit videoProcessingStarted("Cropping...");QString cfp;bool s=performVideoCrop(cropRoiVideoCoords,cfp);
        if(s&&!cfp.isEmpty()){emit videoProcessingFinished("Cropped.",true);loadVideo(cfp);}
        else{emit videoProcessingFinished("Crop failed.",false);QMessageBox::critical(this,"Crop Error","Failed to crop.");}}
    setInteractionMode(InteractionMode::PanZoom);m_isDefiningRoi=false;update();
}
bool VideoLoader::performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath) { /* ... (same as v7) ... */
    if(currentFilePath.isEmpty()||!videoCapture.isOpened()||cropRectVideoCoords.isNull()||!cropRectVideoCoords.isValid())return false;
    cv::VideoCapture ov;if(!ov.open(currentFilePath.toStdString()))return false; double fps=ov.get(cv::CAP_PROP_FPS);if(fps<=0)fps=25.0;
    cv::Rect cvCR(qRound(cropRectVideoCoords.x()),qRound(cropRectVideoCoords.y()),qRound(cropRectVideoCoords.width()),qRound(cropRectVideoCoords.height()));
    if(cvCR.x<0)cvCR.x=0;if(cvCR.y<0)cvCR.y=0; if(cvCR.x+cvCR.width>originalFrameSize.width())cvCR.width=originalFrameSize.width()-cvCR.x;
    if(cvCR.y+cvCR.height>originalFrameSize.height())cvCR.height=originalFrameSize.height()-cvCR.y; if(cvCR.width<=0||cvCR.height<=0){ov.release();return false;}
    cv::Size cfs(cvCR.width,cvCR.height); QFileInfo ofi(currentFilePath);QString bn=ofi.completeBaseName(),suff=ofi.suffix().isEmpty()?QString(DEFAULT_CROP_EXTENSION).remove(0,1):ofi.suffix();
    outCroppedFilePath=ofi.absolutePath()+"/"+bn+"_cropped."+suff; cv::VideoWriter w;int fourcc=DEFAULT_CROP_FOURCC;
    if(!w.open(outCroppedFilePath.toStdString(),fourcc,fps,cfs,true)){ov.release();return false;} cv::Mat fr,cf;int fc=0;
    while(ov.read(fr)){if(fr.empty())continue;try{cf=fr(cvCR);w.write(cf);fc++;}catch(const cv::Exception&){break;}}
    ov.release();w.release();return fc>0;
}
void VideoLoader::applyThresholding() { /* ... (same as v7) ... */
    if(currentCvFrame.empty()){m_thresholdedFrame_mono=cv::Mat();return;} cv::Mat gf;
    if(currentCvFrame.channels()==3||currentCvFrame.channels()==4)cv::cvtColor(currentCvFrame,gf,cv::COLOR_BGR2GRAY); else gf=currentCvFrame.clone();
    cv::GaussianBlur(gf,gf,cv::Size(5,5),0); int tt=m_assumeLightBackground?cv::THRESH_BINARY_INV:cv::THRESH_BINARY;
    switch(m_thresholdAlgorithm){
    case ThresholdAlgorithm::Global:cv::threshold(gf,m_thresholdedFrame_mono,m_thresholdValue,255,tt);break;
    case ThresholdAlgorithm::Otsu:cv::threshold(gf,m_thresholdedFrame_mono,0,255,tt|cv::THRESH_OTSU);break;
    case ThresholdAlgorithm::AdaptiveMean:cv::adaptiveThreshold(gf,m_thresholdedFrame_mono,255,cv::ADAPTIVE_THRESH_MEAN_C,tt,m_adaptiveBlockSize,m_adaptiveC);break;
    case ThresholdAlgorithm::AdaptiveGaussian:cv::adaptiveThreshold(gf,m_thresholdedFrame_mono,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,tt,m_adaptiveBlockSize,m_adaptiveC);break;
    default:cv::threshold(gf,m_thresholdedFrame_mono,m_thresholdValue,255,tt);break;
    }
}

// New private method for handling worm selection clicks
void VideoLoader::handleWormSelection(const QPointF& widgetClickPos) {
    if (!m_showThresholdMask || m_thresholdedFrame_mono.empty()) {
        QMessageBox::warning(this, "Selection Error", "Threshold view must be active to select worms.");
        return;
    }
    if (m_wormsSelectedCount >= m_numberOfWormsToSelect) {
        qInfo() << "All" << m_numberOfWormsToSelect << "worms already selected.";
        return;
    }

    QPointF videoClickPos = mapPointToVideo(widgetClickPos);
    if (videoClickPos.x() < 0) { // Click outside video area
        qDebug() << "Worm selection click outside video area.";
        return;
    }

    // Find contours in the current thresholded frame
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // Use a copy of m_thresholdedFrame_mono because findContours can modify the input image
    cv::Mat contoursInput = m_thresholdedFrame_mono.clone();
    cv::findContours(contoursInput, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point videoClickCvPoint(static_cast<int>(videoClickPos.x()), static_cast<int>(videoClickPos.y()));

    bool wormFoundAtClick = false;
    for (size_t i = 0; i < contours.size(); ++i) {
        if (cv::pointPolygonTest(contours[i], videoClickCvPoint, false) >= 0) { // Point is inside or on the contour
            cv::Rect cvBoundingBox = cv::boundingRect(contours[i]);
            cv::Moments M = cv::moments(contours[i]);
            QPointF centroidVideo( (M.m10 / M.m00) , (M.m01 / M.m00) ); // Calculate centroid

            // Check if this blob (centroid) has already been selected (simple check)
            bool alreadySelected = false;
            for(const Worm* existingWorm : m_selectedWormsData) {
                // Use a small tolerance for centroid comparison
                if ( (existingWorm->initialCentroid() - centroidVideo).manhattanLength() < 5.0 ) {
                    alreadySelected = true;
                    break;
                }
            }
            if (alreadySelected) {
                qDebug() << "This blob appears to be already selected.";
                continue; // Skip this contour
            }


            QRectF boundingBoxVideo(cvBoundingBox.x, cvBoundingBox.y, cvBoundingBox.width, cvBoundingBox.height);

            m_wormsSelectedCount++;
            Worm* newWormData = new Worm(m_wormsSelectedCount, centroidVideo, boundingBoxVideo);
            m_selectedWormsData.append(newWormData); // Store temporarily

            qInfo() << "Worm" << m_wormsSelectedCount << "selected at video pos:" << centroidVideo
                    << "BBox:" << boundingBoxVideo;
            emit wormSelected(m_wormsSelectedCount, centroidVideo);
            wormFoundAtClick = true;
            update(); // Repaint to show selection marker

            if (m_wormsSelectedCount == m_numberOfWormsToSelect) {
                qInfo() << "All" << m_numberOfWormsToSelect << "worms selected!";
                emit allWormsSelected();
                // Optionally, you could automatically switch mode or prompt user to start tracking
                // setInteractionMode(InteractionMode::PanZoom); // Example
            }
            break; // Found a worm at this click, don't check other contours for this click
        }
    }

    if (!wormFoundAtClick) {
        qDebug() << "No distinct worm (blob) found at click position:" << videoClickPos;
    }
}

// --- Slots for WormTracker signals ---
void VideoLoader::onWormTrackerFinished(int wormId) {
    qDebug() << "Tracking completed for worm ID:" << wormId;
    // Check if all trackers are done
    bool allDone = true;
    for (WormTracker* tracker : m_wormTrackers) {
        // This requires WormTracker to have a method like `isFinished()`
        // For now, we'll manage a counter or check thread states indirectly.
        // This logic needs refinement based on WormTracker's actual state reporting.
    }
    // For simplicity, assume we need a counter
    // This is a placeholder - proper completion check is needed
    static int completedTrackers = 0; // Static for demo, needs better state management
    completedTrackers++;
    if (completedTrackers >= m_wormTrackers.size()) {
        m_allTrackingTasksCompleted = true;
        emit allTrackingCompleted();
        emit videoProcessingFinished(QString("%1 worms tracked.").arg(m_wormTrackers.size()), true);
        qInfo() << "All worm tracking tasks completed.";
        completedTrackers = 0; // Reset for next run
    }
    emit wormTrackingCompleted(wormId);
}

void VideoLoader::onWormTrackerError(int wormId, const QString& message) {
    qWarning() << "Error in tracker for worm ID:" << wormId << "-" << message;
    // Handle error, maybe stop other trackers or notify user
}

void VideoLoader::onWormTrackerDisplayUpdate(int wormId, TrackingDirection direction, int frameNum, const QImage& cropImage) {
    // This signal is ready to be connected to a UI element that displays individual worm crops
    emit trackingUpdateForDisplay(wormId, direction, frameNum, cropImage);
}


