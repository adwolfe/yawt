#include "videoloader.h"
#include "wormtracker.h"
#include "worm.h"
#include "thresholdvideoworker.h" // Include the new worker header

#include <QDebug>
#include <QPainterPath>
#include <QtMath>
#include <QResizeEvent>
#include <QMessageBox>
#include <QFileInfo>
#include <QFileDialog> // For prompting for save location
#include <vector>

// Default crop parameters
#define DEFAULT_CROP_EXTENSION ".mp4"
#define DEFAULT_CROP_FOURCC cv::VideoWriter::fourcc('H', '2', '6', '4')
#define PRETHRES_SUFFIX "_thresholded"


VideoLoader::VideoLoader(QWidget *parent)
    : QWidget(parent),
    playbackTimer(new QTimer(this)),
    // ... (initializations from v8 for video properties, playback, interaction, ROI, zoom/pan) ...
    totalFramesCount(0), framesPerSecond(0.0), currentFrameIdx(-1), m_isPlaying(false),
    m_currentMode(InteractionMode::PanZoom), m_isPanning(false), m_isDefiningRoi(false),
    m_zoomFactor(1.0), m_panOffset(0.0, 0.0),
    // Initialize thresholding members
    m_showThresholdMask(false),
    m_thresholdAlgorithm(ThresholdAlgorithm::Global),
    m_thresholdValue(127),
    m_assumeLightBackground(true),
    m_adaptiveBlockSize(11),
    m_adaptiveC(2.0),
    m_thresholdWorker(nullptr), // Initialize worker pointer
    // Initialize worm tracking members
    m_numberOfWormsToSelect(0), m_wormsSelectedCount(0), m_allTrackingTasksCompleted(false)
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
    if (m_thresholdWorker && m_thresholdWorker->isRunning()) {
        m_thresholdWorker->stopProcessing();
        m_thresholdWorker->wait(5000); // Wait for it to finish
    }
    delete m_thresholdWorker; // It's parented to QObject, but explicit delete is fine

    stopAllTracking();
    qDeleteAll(m_wormTrackers);
    m_wormTrackers.clear();
    qDeleteAll(m_selectedWormsData);
    m_selectedWormsData.clear();

    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
}

// --- Public Methods ---
// ... (Getters from v8) ...
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
bool VideoLoader::isPreThresholdingInProgress() const { return m_thresholdWorker && m_thresholdWorker->isRunning(); }

QVariantMap VideoLoader::getCurrentThresholdParameters() const {
    QVariantMap params;
    params["algorithm"] = static_cast<int>(m_thresholdAlgorithm);
    params["value"] = m_thresholdValue;
    params["lightBackground"] = m_assumeLightBackground;
    params["adaptiveBlockSize"] = m_adaptiveBlockSize;
    params["adaptiveC"] = m_adaptiveC;
    // Add any other relevant params like blurKernelSize if you make it configurable
    params["blurKernelSize"] = 5; // Example, make this a member if configurable
    return params;
}


// --- Slots ---
bool VideoLoader::loadVideo(const QString &filePath) {
    if (isPreThresholdingInProgress()) {
        QMessageBox::warning(this, "Busy", "Cannot load new video while pre-thresholding is in progress.");
        return false;
    }
    stopAllTracking();
    qDeleteAll(m_wormTrackers); m_wormTrackers.clear();
    qDeleteAll(m_selectedWormsData); m_selectedWormsData.clear();
    m_wormsSelectedCount = 0; m_allTrackingTasksCompleted = false;
    m_preThresholdedVideoPath.clear(); // Clear path to any previous pre-thresholded video

    // ... (rest of loadVideo from v8, ensure m_preThresholdedVideoPath is cleared) ...
    if (m_isPlaying) pause();
    m_zoomFactor = 1.0; m_panOffset = QPointF(0.0,0.0); m_activeRoiRect = QRectF();
    m_isPanning = false; m_isDefiningRoi = false;

    if (!openVideoFile(filePath)) { /* ... error handling ... */ return false; }
    currentFilePath = filePath;
    framesPerSecond = videoCapture.get(cv::CAP_PROP_FPS); if(framesPerSecond<=0)framesPerSecond=25.0;
    totalFramesCount = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    originalFrameSize = QSize(static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH)), static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT)));
    if (totalFramesCount <= 0 || originalFrameSize.isEmpty()) { /* ... error handling ... */ return false; }
    seekToFrame(0);
    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor); emit roiDefined(m_activeRoiRect);
    updateCursorShape(); qDebug() << "Video loaded:" << filePath;
    return true;
}

// ... (openVideoFile, play, pause, seekToFrame, displayFrame, convertCvMatToQImage, processNextFrame are same as v8) ...
bool VideoLoader::openVideoFile(const QString &filePath) { /* ... v8 ... */
    if(videoCapture.isOpened())videoCapture.release(); try{if(!videoCapture.open(filePath.toStdString()))return false;}catch(const cv::Exception&){return false;} return true;
}
void VideoLoader::play() { /* ... v8 ... */
    if(!isVideoLoaded())return; if(m_isPlaying){pause();}else{if(currentFrameIdx>=totalFramesCount-1&&totalFramesCount>0){seekToFrame(0);if(currentFrameIdx==-1&&totalFramesCount>0)return;}
        m_isPlaying=true;int i=(framesPerSecond>0)?qRound(1000.0/framesPerSecond):40;playbackTimer->start(i);emit playbackStateChanged(true);}
}
void VideoLoader::pause() { /* ... v8 ... */
    if(!m_isPlaying&&!playbackTimer->isActive())return;m_isPlaying=false;playbackTimer->stop();emit playbackStateChanged(false);
}
void VideoLoader::seekToFrame(int fn, bool se) { /* ... v8 ... */
    if(!isVideoLoaded())return;fn=qBound(0,fn,totalFramesCount>0?totalFramesCount-1:0);displayFrame(fn,se);
}
void VideoLoader::displayFrame(int fn, bool se) { /* ... v8 ... */
    if(!videoCapture.isOpened()||originalFrameSize.isEmpty()){/* clear */return;} if(fn<0||fn>=totalFramesCount){/* clear */return;}
    int cp=static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES)); if(cp!=fn){if(!videoCapture.set(cv::CAP_PROP_POS_FRAMES,static_cast<double>(fn))){/*warn*/}}
    if(videoCapture.read(currentCvFrame)){if(!currentCvFrame.empty()){if(m_showThresholdMask)applyThresholding();}else{currentCvFrame=cv::Mat();m_thresholdedFrame_mono=cv::Mat();}}
    else{currentCvFrame=cv::Mat();m_thresholdedFrame_mono=cv::Mat();if(m_isPlaying)pause();}
    if(m_showThresholdMask&&!m_thresholdedFrame_mono.empty()){convertCvMatToQImage(m_thresholdedFrame_mono,currentQImageFrame);}
    else if(!currentCvFrame.empty()){convertCvMatToQImage(currentCvFrame,currentQImageFrame);}else{currentQImageFrame=QImage();}
    currentFrameIdx=fn; if(!se)emit frameChanged(currentFrameIdx,currentQImageFrame); update();
}
void VideoLoader::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) { /* ... v8 ... */
    if(mat.empty()){qimg=QImage();return;}if(mat.type()==CV_8UC3){qimg=QImage(mat.data,mat.cols,mat.rows,static_cast<int>(mat.step),QImage::Format_RGB888).rgbSwapped();}
    else if(mat.type()==CV_8UC1){qimg=QImage(mat.data,mat.cols,mat.rows,static_cast<int>(mat.step),QImage::Format_Grayscale8);}
    else{cv::Mat t;try{if(mat.channels()==4){cv::cvtColor(mat,t,cv::COLOR_BGRA2BGR);qimg=QImage(t.data,t.cols,t.rows,static_cast<int>(t.step),QImage::Format_RGB888).rgbSwapped();}
            else if(mat.channels()==3&&mat.type()!=CV_8UC3){mat.convertTo(t,CV_8UC3,255.0);qimg=QImage(t.data,t.cols,t.rows,static_cast<int>(t.step),QImage::Format_RGB888).rgbSwapped();}
            else if(mat.channels()==1&&mat.type()!=CV_8UC1){mat.convertTo(t,CV_8UC1,255.0);qimg=QImage(t.data,t.cols,t.rows,static_cast<int>(t.step),QImage::Format_Grayscale8);}
            else{qimg=QImage();return;}}catch(const cv::Exception&){qimg=QImage();}}
}
void VideoLoader::processNextFrame() { /* ... v8 ... */
    if(!m_isPlaying||!isVideoLoaded())return;if(currentFrameIdx<totalFramesCount-1)displayFrame(currentFrameIdx+1);else pause();
}

void VideoLoader::setZoomFactor(double factor) { setZoomFactorAtPoint(factor, rect().center()); }
void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) { /* ... (same as v8) ... */
    if(!isVideoLoaded())return;double nZ=qBound(0.05,factor,50.0);if(qFuzzyCompare(m_zoomFactor,nZ))return;
    QPointF vp=mapPointToVideo(widgetPoint),rpw=widgetPoint;if(vp.x()<0){rpw=rect().center();vp=mapPointToVideo(rpw);if(vp.x()<0)return;}
    m_zoomFactor=nZ;QPointF wap=mapPointFromVideo(vp);m_panOffset+=(rpw-wap);clampPanOffset();update();emit zoomFactorChanged(m_zoomFactor);
}
void VideoLoader::setInteractionMode(InteractionMode mode) { /* ... (same as v8, including SelectWorms logic) ... */
    if(m_currentMode==mode)return; m_isPanning=false;m_isDefiningRoi=false;m_currentMode=mode;
    if(m_currentMode==InteractionMode::SelectWorms){if(!m_showThresholdMask)toggleThresholdView(true);
        m_wormsSelectedCount=0;qDeleteAll(m_selectedWormsData);m_selectedWormsData.clear();
        if(m_numberOfWormsToSelect<=0){/*warn*/}else{qInfo()<<"SelectWorms mode: Click "<<m_numberOfWormsToSelect<<" worms.";}}
    updateCursorShape();emit interactionModeChanged(m_currentMode);
}
void VideoLoader::clearRoi() { /* ... (same as v8) ... */
    if(!m_activeRoiRect.isNull()){m_activeRoiRect=QRectF();update();emit roiDefined(m_activeRoiRect);qDebug()<<"ROI cleared.";}
}

// --- Thresholding Control Slots ---
// ... (toggleThresholdView, setThresholdAlgorithm, etc. are the same as v8) ...
void VideoLoader::toggleThresholdView(bool en){if(m_showThresholdMask==en)return;m_showThresholdMask=en;if(isVideoLoaded()&&currentFrameIdx>=0)displayFrame(currentFrameIdx,true);update();}
void VideoLoader::setThresholdAlgorithm(ThresholdAlgorithm algo){if(m_thresholdAlgorithm==algo)return;m_thresholdAlgorithm=algo;if(m_showThresholdMask&&isVideoLoaded()&&currentFrameIdx>=0)displayFrame(currentFrameIdx,true);emit thresholdParametersChanged(m_thresholdAlgorithm,m_thresholdValue,m_assumeLightBackground,m_adaptiveBlockSize,m_adaptiveC);update();}
void VideoLoader::setThresholdValue(int v){v=qBound(0,v,255);if(m_thresholdValue==v)return;m_thresholdValue=v;if(m_showThresholdMask&&isVideoLoaded()&&currentFrameIdx>=0&&m_thresholdAlgorithm==ThresholdAlgorithm::Global)displayFrame(currentFrameIdx,true);emit thresholdParametersChanged(m_thresholdAlgorithm,m_thresholdValue,m_assumeLightBackground,m_adaptiveBlockSize,m_adaptiveC);update();}
void VideoLoader::setAssumeLightBackground(bool l){if(m_assumeLightBackground==l)return;m_assumeLightBackground=l;if(m_showThresholdMask&&isVideoLoaded()&&currentFrameIdx>=0)displayFrame(currentFrameIdx,true);emit thresholdParametersChanged(m_thresholdAlgorithm,m_thresholdValue,m_assumeLightBackground,m_adaptiveBlockSize,m_adaptiveC);update();}
void VideoLoader::setAdaptiveThresholdBlockSize(int bs){if(bs<3)bs=3;if(bs%2==0)bs++;if(m_adaptiveBlockSize==bs)return;m_adaptiveBlockSize=bs;if(m_showThresholdMask&&isVideoLoaded()&&currentFrameIdx>=0&&(m_thresholdAlgorithm==ThresholdAlgorithm::AdaptiveMean||m_thresholdAlgorithm==ThresholdAlgorithm::AdaptiveGaussian))displayFrame(currentFrameIdx,true);emit thresholdParametersChanged(m_thresholdAlgorithm,m_thresholdValue,m_assumeLightBackground,m_adaptiveBlockSize,m_adaptiveC);update();}
void VideoLoader::setAdaptiveThresholdC(double cv){if(qFuzzyCompare(m_adaptiveC,cv))return;m_adaptiveC=cv;if(m_showThresholdMask&&isVideoLoaded()&&currentFrameIdx>=0&&(m_thresholdAlgorithm==ThresholdAlgorithm::AdaptiveMean||m_thresholdAlgorithm==ThresholdAlgorithm::AdaptiveGaussian))displayFrame(currentFrameIdx,true);emit thresholdParametersChanged(m_thresholdAlgorithm,m_thresholdValue,m_assumeLightBackground,m_adaptiveBlockSize,m_adaptiveC);update();}


void VideoLoader::startVideoPreThresholding() {
    if (!isVideoLoaded()) {
        QMessageBox::warning(this, "Error", "No video loaded to pre-threshold.");
        return;
    }
    if (m_thresholdWorker && m_thresholdWorker->isRunning()) {
        QMessageBox::information(this, "In Progress", "Pre-thresholding is already in progress.");
        return;
    }

    QFileInfo originalFileInfo(currentFilePath);
    QString defaultName = originalFileInfo.absolutePath() + "/" +
                          originalFileInfo.completeBaseName() +
                          PRETHRES_SUFFIX +
                          BINARY_VIDEO_EXTENSION; // Use defined extension for binary video

    QString outputFilePath = QFileDialog::getSaveFileName(this, "Save Thresholded Video As",
                                                          defaultName,
                                                          QString("Video Files (*%1);;All Files (*)").arg(BINARY_VIDEO_EXTENSION));
    if (outputFilePath.isEmpty()) {
        return; // User cancelled
    }
    // Ensure the chosen file has the correct extension
    QFileInfo outInfo(outputFilePath);
    if (outInfo.suffix().isEmpty() || outInfo.suffix() != QString(BINARY_VIDEO_EXTENSION).remove(0,1)) {
        outputFilePath += BINARY_VIDEO_EXTENSION;
    }


    m_preThresholdedVideoPath.clear(); // Clear any old path
    QVariantMap currentParams = getCurrentThresholdParameters();

    m_thresholdWorker = new ThresholdVideoWorker(currentFilePath, outputFilePath, currentParams, this); // Parent to this
    connect(m_thresholdWorker, &ThresholdVideoWorker::progressUpdated, this, &VideoLoader::preThresholdingProgress);
    connect(m_thresholdWorker, &ThresholdVideoWorker::processingFinished, this, &VideoLoader::onPreThresholdingWorkerFinished);
    connect(m_thresholdWorker, &QThread::finished, m_thresholdWorker, &QObject::deleteLater); // Auto-delete

    emit videoProcessingStarted("Starting pre-thresholding of video...");
    // Disable UI elements like load video, start tracking, threshold param changes, etc.
    m_thresholdWorker->start();
}

void VideoLoader::cancelVideoPreThresholding() {
    if (m_thresholdWorker && m_thresholdWorker->isRunning()) {
        m_thresholdWorker->stopProcessing();
        qInfo() << "Cancellation request sent to pre-thresholding worker.";
        // Worker will emit processingFinished with success=false and "cancelled" message
    } else {
        qInfo() << "No pre-thresholding process active to cancel.";
    }
}


// --- Worm Tracking Control Slots ---
void VideoLoader::setNumberOfWormsToSelect(int num) { /* ... (same as v8) ... */
    if(num<=0){m_numberOfWormsToSelect=0;return;}m_numberOfWormsToSelect=num;
    if(m_currentMode==InteractionMode::SelectWorms){m_wormsSelectedCount=0;qDeleteAll(m_selectedWormsData);m_selectedWormsData.clear();qInfo()<<"Reselect worms.";}
}

void VideoLoader::startTrackingSelectedWorms() {
    if (m_preThresholdedVideoPath.isEmpty()) {
        QMessageBox::warning(this, "Tracking Error", "Pre-thresholded video not generated. Please generate it first.");
        return;
    }
    if (m_selectedWormsData.count() != m_numberOfWormsToSelect || m_numberOfWormsToSelect <= 0) {
        QMessageBox::warning(this, "Tracking Error", QString("Expected %1 worms, but %2 selected.").arg(m_numberOfWormsToSelect).arg(m_selectedWormsData.count()));
        return;
    }
    if (currentFrameIdx < 0) { // Keyframe must be set
        QMessageBox::warning(this, "Tracking Error", "Keyframe not set for tracking (currentFrameIdx is invalid).");
        return;
    }

    stopAllTracking();
    qDeleteAll(m_wormTrackers); m_wormTrackers.clear();
    m_allTrackingTasksCompleted = false;

    qInfo() << "Starting tracking using pre-thresholded video:" << m_preThresholdedVideoPath
            << "for" << m_selectedWormsData.count() << "worms from keyframe" << currentFrameIdx;
    emit videoProcessingStarted(QString("Initializing tracking for %1 worms...").arg(m_selectedWormsData.count()));

    // Prepare tracking parameters. Indicate that the video is pre-thresholded.
    QVariantMap trackingParams = getCurrentThresholdParameters();
    trackingParams["isVideoPreThresholded"] = true; // Add a flag

    // Need total frames and FPS of the *pre-thresholded* video.
    // For simplicity, assume they are same as original, or re-read from pre-thresholded file.
    // Let's assume they are the same for now.
    int framesForTracking = totalFramesCount; // Should ideally be from m_preThresholdedVideoPath
    double fpsForTracking = framesPerSecond; // Same assumption

    for (Worm* wormData : m_selectedWormsData) {
        WormTracker* tracker = new WormTracker(wormData, m_preThresholdedVideoPath, currentFrameIdx,
                                               trackingParams, framesForTracking, fpsForTracking, this);
        connect(tracker, &WormTracker::trackingForWormCompleted, this, &VideoLoader::onWormTrackerFinished); // Corrected signal
        connect(tracker, &WormTracker::trackingErrorForWorm, this, &VideoLoader::onWormTrackerError);     // Corrected signal
        connect(tracker, &WormTracker::newDisplayableCrop, this, &VideoLoader::onWormTrackerDisplayUpdate);
        m_wormTrackers.append(tracker);
        tracker->startTracking();
    }
    m_selectedWormsData.clear(); // WormData objects are now owned by WormTrackers

    setInteractionMode(InteractionMode::PanZoom);
}

void VideoLoader::stopAllTracking() { /* ... (same as v8) ... */
    qInfo()<<"Stopping all trackers."; for(WormTracker* t:m_wormTrackers)t->stopTracking();
}


// --- Event Handlers ---
// ... (paintEvent, mouse events, wheelEvent, resizeEvent are mostly same as v8, ensure SelectWorms paint logic is good) ...
void VideoLoader::paintEvent(QPaintEvent *ev) {
    Q_UNUSED(ev); QPainter p(this); p.setRenderHint(QPainter::SmoothPixmapTransform);
    if(currentQImageFrame.isNull()||!isVideoLoaded()){p.fillRect(rect(),palette().color(QPalette::Window));p.setPen(Qt::gray);p.drawText(rect(),Qt::AlignCenter,"No video.");return;}
    QRectF tr=calculateTargetRect(); p.drawImage(tr,currentQImageFrame,currentQImageFrame.rect());
    if(m_currentMode==InteractionMode::DrawROI&&!m_activeRoiRect.isNull()&&m_activeRoiRect.isValid()){
        QPointF tlw=mapPointFromVideo(m_activeRoiRect.topLeft()),brw=mapPointFromVideo(m_activeRoiRect.bottomRight());
        if(tlw.x()>=0&&brw.x()>=0){p.setPen(QPen(Qt::red,1,Qt::DashLine));p.drawRect(QRectF(tlw,brw).normalized());}}
    if((m_currentMode==InteractionMode::DrawROI||m_currentMode==InteractionMode::Crop)&&m_isDefiningRoi){
        p.setPen(QPen(Qt::cyan,1,Qt::SolidLine));p.drawRect(QRect(m_roiStartPointWidget,m_roiEndPointWidget).normalized());}
    if(m_currentMode==InteractionMode::SelectWorms&&isVideoLoaded()){
        for(const Worm*wd:m_selectedWormsData){QPointF vc=wd->initialCentroid(),wc=mapPointFromVideo(vc);if(wc.x()>=0){
                p.setPen(QPen(wd->trackColor(),2));p.drawEllipse(wc,5,5);p.drawText(wc+QPointF(7,7),QString::number(wd->id()));}}
        p.setPen(Qt::white);p.drawText(10,20,QString("Select %1 more worm(s). Click on a blob.").arg(m_numberOfWormsToSelect-m_wormsSelectedCount));
    }
}
QRectF VideoLoader::calculateTargetRect() const { /* v8 */ if(originalFrameSize.isEmpty())return QRectF();QSizeF ws=size(),ofs=originalFrameSize,cs=ofs;cs.scale(ws,Qt::KeepAspectRatio);QSizeF zs=cs*m_zoomFactor;QPointF ctl((ws.width()-zs.width())/2.0,(ws.height()-zs.height())/2.0);return QRectF(ctl+m_panOffset,zs); }
QPointF VideoLoader::mapPointToVideo(const QPointF& wp) const { /* v8 */ if(currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0)return QPointF(-1,-1);QRectF tr=calculateTargetRect();if(!tr.contains(wp)||tr.width()<=0||tr.height()<=0)return QPointF(-1,-1);double nx=(wp.x()-tr.left())/tr.width(),ny=(wp.y()-tr.top())/tr.height();QSizeF ofs=originalFrameSize;return QPointF(qBound(0.0,nx*ofs.width(),ofs.width()),qBound(0.0,ny*ofs.height(),ofs.height())); }
QPointF VideoLoader::mapPointFromVideo(const QPointF& vp) const { /* v8 */ if(currentQImageFrame.isNull()||originalFrameSize.isEmpty()||m_zoomFactor<=0||originalFrameSize.width()<=0||originalFrameSize.height()<=0)return QPointF(-1,-1);QRectF tr=calculateTargetRect();if(tr.width()<=0||tr.height()<=0)return QPointF(-1,-1);QSizeF ofs=originalFrameSize;QPointF cvp(qBound(0.0,vp.x(),ofs.width()),qBound(0.0,vp.y(),ofs.height()));double nx=cvp.x()/ofs.width(),ny=cvp.y()/ofs.height();return QPointF(tr.left()+nx*tr.width(),tr.top()+ny*tr.height()); }
void VideoLoader::mousePressEvent(QMouseEvent *ev) { /* v8 + SelectWorms */
    m_lastMousePos=ev->position();if(!isVideoLoaded()){QWidget::mousePressEvent(ev);return;}
    if(ev->button()==Qt::LeftButton){
        if(m_currentMode==InteractionMode::PanZoom){m_isPanning=true;updateCursorShape();ev->accept();}
        else if(m_currentMode==InteractionMode::DrawROI||m_currentMode==InteractionMode::Crop){
            QPointF vc=mapPointToVideo(ev->position());if(vc.x()>=0){m_roiStartPointWidget=ev->pos();m_roiEndPointWidget=ev->pos();m_isDefiningRoi=true;update();ev->accept();}else{m_isDefiningRoi=false;}}
        else if(m_currentMode==InteractionMode::SelectWorms){if(m_wormsSelectedCount<m_numberOfWormsToSelect)handleWormSelection(ev->position());else qInfo()<<"All worms selected.";ev->accept();}
    }else QWidget::mousePressEvent(ev);
}
void VideoLoader::mouseMoveEvent(QMouseEvent *ev) { /* v8 */ QPointF cp=ev->position(),d=cp-m_lastMousePos;if(!isVideoLoaded()){QWidget::mouseMoveEvent(ev);return;} if(m_isPanning&&(ev->buttons()&Qt::LeftButton)){m_panOffset+=d;clampPanOffset();update();ev->accept();} else if((m_currentMode==InteractionMode::DrawROI||m_currentMode==InteractionMode::Crop)&&m_isDefiningRoi&&(ev->buttons()&Qt::LeftButton)){m_roiEndPointWidget=ev->pos();update();ev->accept();} else QWidget::mouseMoveEvent(ev);m_lastMousePos=cp;}
void VideoLoader::mouseReleaseEvent(QMouseEvent *ev) { /* v8 */ if(!isVideoLoaded()){QWidget::mouseReleaseEvent(ev);return;} if(ev->button()==Qt::LeftButton){ if(m_isPanning){m_isPanning=false;updateCursorShape();ev->accept();} else if(m_isDefiningRoi&&(m_currentMode==InteractionMode::DrawROI||m_currentMode==InteractionMode::Crop)){m_roiEndPointWidget=ev->pos();m_isDefiningRoi=false;QPointF vs=mapPointToVideo(m_roiStartPointWidget),ve=mapPointToVideo(m_roiEndPointWidget);QRectF dr;if(vs.x()>=0&&ve.x()>=0){dr=QRectF(vs,ve).normalized();if(dr.width()<5||dr.height()<5)dr=QRectF();} if(m_currentMode==InteractionMode::DrawROI){m_activeRoiRect=dr;emit roiDefined(m_activeRoiRect);} else if(m_currentMode==InteractionMode::Crop){if(!dr.isNull()&&dr.isValid())handleRoiDefinedForCrop(dr);else{setInteractionMode(InteractionMode::PanZoom);}} update();updateCursorShape();ev->accept();}} else QWidget::mouseReleaseEvent(ev);}
void VideoLoader::wheelEvent(QWheelEvent *ev) { /* v8 */ if(!isVideoLoaded()||m_currentMode!=InteractionMode::PanZoom){ev->ignore();return;}int deg=ev->angleDelta().y()/8;if(deg==0){ev->ignore();return;}int st=deg/15;double zs=0.15,ml=qPow(1.0+zs,st);double nz=qBound(0.05,m_zoomFactor*ml,50.0);if(qFuzzyCompare(m_zoomFactor,nz)){ev->accept();return;}QPointF mpw=ev->position(),vp=mapPointToVideo(mpw),rpw=mpw;if(vp.x()<0){rpw=rect().center();vp=mapPointToVideo(rpw);if(vp.x()<0){ev->ignore();return;}}m_zoomFactor=nz;m_panOffset+=(rpw-mapPointFromVideo(vp));clampPanOffset();update();emit zoomFactorChanged(m_zoomFactor);ev->accept();}
void VideoLoader::resizeEvent(QResizeEvent *ev) { /* v8 */ QWidget::resizeEvent(ev);if(isVideoLoaded()){clampPanOffset();update();}}

// --- Private Helper Methods ---
void VideoLoader::updateCursorShape() { /* ... (same as v8) ... */ if(!isVideoLoaded()){setCursor(Qt::ArrowCursor);return;}switch(m_currentMode){case InteractionMode::PanZoom:setCursor(m_isPanning?Qt::ClosedHandCursor:Qt::OpenHandCursor);break;case InteractionMode::DrawROI:case InteractionMode::Crop:setCursor(Qt::CrossCursor);break;case InteractionMode::SelectWorms:setCursor(Qt::PointingHandCursor);break;default:setCursor(Qt::ArrowCursor);break;}}
void VideoLoader::clampPanOffset() { /* ... (same as v8) ... */ if(!isVideoLoaded()||m_zoomFactor<=0)return;QRectF tr=calculateTargetRect();QSizeF ws=size();double mx=-(tr.width()-50.0),my=-(tr.height()-50.0),mX=ws.width()-50.0,mY=ws.height()-50.0;QPointF cftl=tr.topLeft(),clftl;clftl.setX(qBound(mx,cftl.x(),mX));clftl.setY(qBound(my,cftl.y(),mY));if(tr.width()<=ws.width())clftl.setX((ws.width()-tr.width())/2.0);if(tr.height()<=ws.height())clftl.setY((ws.height()-tr.height())/2.0);QSizeF ofs=originalFrameSize,cs=ofs;cs.scale(ws,Qt::KeepAspectRatio);QSizeF zs=cs*m_zoomFactor;QPointF ctl_c((ws.width()-zs.width())/2.0,(ws.height()-zs.height())/2.0);QPointF rpo=clftl-ctl_c;if(!qFuzzyCompare(m_panOffset.x(),rpo.x())||!qFuzzyCompare(m_panOffset.y(),rpo.y()))m_panOffset=rpo;}
void VideoLoader::handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords) { /* ... (same as v8) ... */ if(cropRoiVideoCoords.isNull()||!cropRoiVideoCoords.isValid()){setInteractionMode(InteractionMode::PanZoom);return;}QMessageBox::StandardButton r=QMessageBox::question(this,"Confirm Crop","Crop video?",QMessageBox::Yes|QMessageBox::No);if(r==QMessageBox::Yes){emit videoProcessingStarted("Cropping...");QString cfp;bool s=performVideoCrop(cropRoiVideoCoords,cfp);if(s&&!cfp.isEmpty()){emit videoProcessingFinished("Cropped.",true);loadVideo(cfp);}else{emit videoProcessingFinished("Crop failed.",false);QMessageBox::critical(this,"Crop Error","Failed to crop.");}}setInteractionMode(InteractionMode::PanZoom);m_isDefiningRoi=false;update();}
bool VideoLoader::performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath) { /* ... (same as v8) ... */ if(currentFilePath.isEmpty()||!videoCapture.isOpened()||cropRectVideoCoords.isNull()||!cropRectVideoCoords.isValid())return false;cv::VideoCapture ov;if(!ov.open(currentFilePath.toStdString()))return false;double fps=ov.get(cv::CAP_PROP_FPS);if(fps<=0)fps=25.0;cv::Rect cvCR(qRound(cropRectVideoCoords.x()),qRound(cropRectVideoCoords.y()),qRound(cropRectVideoCoords.width()),qRound(cropRectVideoCoords.height()));if(cvCR.x<0)cvCR.x=0;if(cvCR.y<0)cvCR.y=0;if(cvCR.x+cvCR.width>originalFrameSize.width())cvCR.width=originalFrameSize.width()-cvCR.x;if(cvCR.y+cvCR.height>originalFrameSize.height())cvCR.height=originalFrameSize.height()-cvCR.y;if(cvCR.width<=0||cvCR.height<=0){ov.release();return false;}cv::Size cfs(cvCR.width,cvCR.height);QFileInfo ofi(currentFilePath);QString bn=ofi.completeBaseName(),suff=ofi.suffix().isEmpty()?QString(DEFAULT_CROP_EXTENSION).remove(0,1):ofi.suffix();outCroppedFilePath=ofi.absolutePath()+"/"+bn+"_cropped"+DEFAULT_CROP_EXTENSION;cv::VideoWriter w;int fourcc=DEFAULT_CROP_FOURCC;if(!w.open(outCroppedFilePath.toStdString(),fourcc,fps,cfs,true)){ov.release();return false;}cv::Mat fr,cf;int fc=0;while(ov.read(fr)){if(fr.empty())continue;try{cf=fr(cvCR);w.write(cf);fc++;}catch(const cv::Exception&){break;}}ov.release();w.release();return fc>0;}
void VideoLoader::applyThresholding() { /* ... (same as v8) ... */ if(currentCvFrame.empty()){m_thresholdedFrame_mono=cv::Mat();return;}cv::Mat gf;if(currentCvFrame.channels()==3||currentCvFrame.channels()==4)cv::cvtColor(currentCvFrame,gf,cv::COLOR_BGR2GRAY);else gf=currentCvFrame.clone();cv::GaussianBlur(gf,gf,cv::Size(5,5),0);int tt=m_assumeLightBackground?cv::THRESH_BINARY_INV:cv::THRESH_BINARY;switch(m_thresholdAlgorithm){case ThresholdAlgorithm::Global:cv::threshold(gf,m_thresholdedFrame_mono,m_thresholdValue,255,tt);break;case ThresholdAlgorithm::Otsu:cv::threshold(gf,m_thresholdedFrame_mono,0,255,tt|cv::THRESH_OTSU);break;case ThresholdAlgorithm::AdaptiveMean:cv::adaptiveThreshold(gf,m_thresholdedFrame_mono,255,cv::ADAPTIVE_THRESH_MEAN_C,tt,m_adaptiveBlockSize,m_adaptiveC);break;case ThresholdAlgorithm::AdaptiveGaussian:cv::adaptiveThreshold(gf,m_thresholdedFrame_mono,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,tt,m_adaptiveBlockSize,m_adaptiveC);break;default:cv::threshold(gf,m_thresholdedFrame_mono,m_thresholdValue,255,tt);break;}}
void VideoLoader::handleWormSelection(const QPointF& widgetClickPos) { /* ... (same as v8) ... */
    if(!m_showThresholdMask||m_thresholdedFrame_mono.empty()){QMessageBox::warning(this,"Sel Error","Threshold view must be active.");return;}
    if(m_wormsSelectedCount>=m_numberOfWormsToSelect){qInfo()<<"All worms selected.";return;}
    QPointF vc=mapPointToVideo(widgetClickPos);if(vc.x()<0){return;}
    std::vector<std::vector<cv::Point>>contours;cv::Mat ci=m_thresholdedFrame_mono.clone();cv::findContours(ci,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    cv::Point vcp(static_cast<int>(vc.x()),static_cast<int>(vc.y()));bool found=false;
    for(size_t i=0;i<contours.size();++i){if(cv::pointPolygonTest(contours[i],vcp,false)>=0){
            cv::Rect bb=cv::boundingRect(contours[i]);cv::Moments M=cv::moments(contours[i]);QPointF cent( (M.m10/M.m00),(M.m01/M.m00));
            bool as=false;for(const Worm*ew:m_selectedWormsData){if((ew->initialCentroid()-cent).manhattanLength()<5.0){as=true;break;}}if(as)continue;
            m_wormsSelectedCount++;Worm*nwd=new Worm(m_wormsSelectedCount,cent,QRectF(bb.x,bb.y,bb.width,bb.height));m_selectedWormsData.append(nwd);
            emit wormSelected(m_wormsSelectedCount,cent);found=true;update();if(m_wormsSelectedCount==m_numberOfWormsToSelect)emit allWormsSelected();break;}}
    if(!found)qDebug()<<"No blob at click.";
}

// --- Slots for WormTracker signals ---
// ... (onWormTrackerFinished, onWormTrackerError, onWormTrackerDisplayUpdate are same as v8) ...
void VideoLoader::onWormTrackerFinished(int wormId){
    qDebug()<<"Tracker finished for worm"<<wormId; static int completed=0; completed++; // Placeholder completion logic
    if(completed>=m_wormTrackers.size()){m_allTrackingTasksCompleted=true;emit allTrackingCompleted();emit videoProcessingFinished("All worms tracked.",true);completed=0;}
    emit wormTrackingCompleted(wormId);
}
void VideoLoader::onWormTrackerError(int wormId, const QString& msg){qWarning()<<"Tracker error worm"<<wormId<<":"<<msg;}
void VideoLoader::onWormTrackerDisplayUpdate(int wId,TrackingDirection dir,int fNum,const QImage&cImg){emit trackingUpdateForDisplay(wId,dir,fNum,cImg);}


// --- Slot for ThresholdVideoWorker ---
void VideoLoader::onPreThresholdingWorkerFinished(const QString& outputPath, bool success, const QString& message) {
    qDebug() << "Pre-thresholding worker finished. Success:" << success << "Path:" << outputPath << "Message:" << message;
    if (m_thresholdWorker) {
        // Worker should auto-deleteLater, no need to delete m_thresholdWorker here if connected to finished.
        m_thresholdWorker = nullptr;
    }

    if (success) {
        m_preThresholdedVideoPath = outputPath;
        QMessageBox::information(this, "Pre-Thresholding Complete",
                                 QString("Pre-thresholded video saved to:\n%1\n\nYou can now start tracking.").arg(outputPath));
        emit preThresholdingCompleted(outputPath, true, message);
    } else {
        QMessageBox::critical(this, "Pre-Thresholding Failed", "Failed to generate pre-thresholded video.\n" + message);
        emit preThresholdingCompleted(outputPath, false, message);
    }
    // Re-enable UI elements
    emit videoProcessingFinished(message, success); // Use general signal to indicate processing is done
}

