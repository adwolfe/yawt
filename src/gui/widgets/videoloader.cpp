#include "videoloader.h"
#include "frameloader.h"

#include <QDebug>
#include "../../utils/loggingcategories.h"
#include <QFileInfo>
#include <QMessageBox>
#include <QPainterPath>
#include <QRandomGenerator>
#include <QResizeEvent>
#include <QWheelEvent>
#include <QtMath>
#include <QStandardPaths>
#include <algorithm> // Required for std::find_if

// Default crop parameters
#define DEFAULT_CROP_EXTENSION ".mp4"
#define DEFAULT_CROP_FOURCC cv::VideoWriter::fourcc('H', '2', '6', '4')

namespace {
QRectF squareFromDiagonal(const QPointF& start, const QPointF& end) {
    const qreal dx = end.x() - start.x();
    const qreal dy = end.y() - start.y();
    const qreal side = qMax(qAbs(dx), qAbs(dy));
    if (side <= 0.0) {
        return QRectF();
    }

    const QPointF topLeft(start.x() + (dx < 0.0 ? -side : 0.0),
                          start.y() + (dy < 0.0 ? -side : 0.0));
    return QRectF(topLeft, QSizeF(side, side)).normalized();
}

bool pointInCircleRect(const QPointF& point, const QRectF& circleRect) {
    if (circleRect.isNull() || !circleRect.isValid()) {
        return false;
    }

    const QPointF center = circleRect.center();
    const qreal radius = circleRect.width() / 2.0;
    const qreal dx = point.x() - center.x();
    const qreal dy = point.y() - center.y();
    return (dx * dx + dy * dy) <= radius * radius;
}
}

// ============================================================================
// FrameCache Implementation
// ============================================================================

FrameCache::FrameCache(int maxCacheSize)
    : m_maxSize(maxCacheSize), m_hits(0), m_requests(0) {
    YAWT_DEBUG(lcGuiVideoLoader) << "FrameCache created with max size:" << m_maxSize;
}

FrameCache::~FrameCache() {
    clear();
    YAWT_INFO(lcGuiVideoLoader) << "FrameCache destroyed. Final hit rate:" << hitRate() << "%";
}

void FrameCache::insertFrame(int frameNumber, const cv::Mat& frame) {
    if (frame.empty()) return;

    QMutexLocker locker(&m_mutex);

    // Remove existing frame if present
    m_frames.remove(frameNumber);

    // Add new frame
    m_frames.insert(frameNumber, CachedFrame(frameNumber, frame));

    // Evict if over capacity
    while (m_frames.size() > m_maxSize) {
        evictLRU();
    }

    YAWT_DEBUG(lcGuiVideoLoader) << "FrameCache: Cached frame" << frameNumber << "- Cache size:" << m_frames.size();
}

bool FrameCache::getFrame(int frameNumber, cv::Mat& outFrame) {
    QMutexLocker locker(&m_mutex);
    m_requests.fetchAndAddOrdered(1);

    auto it = m_frames.find(frameNumber);
    if (it != m_frames.end() && it->isValid) {
        // Return a shallow copy to avoid an expensive deep clone on the main thread.
        outFrame = it->rawFrame;
        updateAccessTime(frameNumber);
        m_hits.fetchAndAddOrdered(1);
        return true;
    }

    return false;
}

bool FrameCache::hasFrame(int frameNumber) const {
    QMutexLocker locker(&m_mutex);
    auto it = m_frames.find(frameNumber);
    return (it != m_frames.end() && it->isValid);
}

void FrameCache::clear() {
    QMutexLocker locker(&m_mutex);
    m_frames.clear();
    YAWT_DEBUG(lcGuiVideoLoader) << "FrameCache: Cleared all frames";
}

void FrameCache::setMaxSize(int maxSize) {
    QMutexLocker locker(&m_mutex);
    m_maxSize = maxSize;
    while (m_frames.size() > m_maxSize) {
        evictLRU();
    }
}

int FrameCache::size() const {
    QMutexLocker locker(&m_mutex);
    return m_frames.size();
}

int FrameCache::maxSize() const {
    QMutexLocker locker(&m_mutex);
    return m_maxSize;
}

double FrameCache::hitRate() const {
    int requests = m_requests;
    int hits = m_hits;
    return requests > 0 ? (static_cast<double>(hits) / requests) * 100.0 : 0.0;
}

void FrameCache::evictLRU() {
    if (m_frames.isEmpty()) return;

    // Find frame with oldest access time
    auto oldest = m_frames.begin();
    for (auto it = m_frames.begin(); it != m_frames.end(); ++it) {
        if (it->lastAccessed < oldest->lastAccessed) {
            oldest = it;
        }
    }

    YAWT_DEBUG(lcGuiVideoLoader) << "FrameCache: Evicting frame" << oldest->frameNumber;
    m_frames.erase(oldest);
}

void FrameCache::updateAccessTime(int frameNumber) {
    auto it = m_frames.find(frameNumber);
    if (it != m_frames.end()) {
        it->lastAccessed = QDateTime::currentDateTime();
    }
}

/* FrameLoader implementation moved to src/gui/widgets/frameloader.cpp.
 * The class declaration is available via the included "frameloader.h".
 */

// Define a click tolerance for selecting track points (in widget pixels)
const qreal TRACK_POINT_CLICK_TOLERANCE = 5.0;
const qreal TRACK_LINE_WIDTH = 2.0;
const qreal CENTERLINE_LINE_WIDTH = 4.0;

VideoLoader::VideoLoader(QWidget* parent)
    : QWidget(parent),
    playbackTimer(new QTimer(this)),
    totalFramesCount(0),
    framesPerSecond(0.0),
    currentFrameIdx(-1),
    m_isPlaying(false),
    m_playbackSpeedMultiplier(1.0),
    m_currentInteractionMode(InteractionMode::PanZoom),
    m_activeViewModes(ViewModeOption::None),
    m_isPanning(false),
    m_isDefiningRoi(false),
    m_cropCircleEditMode(CropCircleEditMode::None),
    m_zoomFactor(1.0),
    m_panOffset(0.0, 0.0),
    m_storage(nullptr),
    m_thresholdAlgorithm(Thresholding::ThresholdAlgorithm::Global),
    m_thresholdValue(100),
    m_assumeLightBackground(true),
    m_adaptiveBlockSize(11),
    m_adaptiveC(2.0),
    m_enableBlur(false),
    m_blurKernelSize(5),
    m_blurSigmaX(0.0),
    m_dataDirectory(),
    m_frameCache(nullptr),
    m_frameLoader(nullptr),
    m_frameLoaderThread(nullptr),
    m_lastPreloadCenter(-1),
    m_pendingSeekFrame(-1),
    m_cacheStreamNextFrame(-1),
    m_trackDisplayMode(TrackDisplayMode::Centroid) {
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);
    connect(playbackTimer, &QTimer::timeout, this,
            &VideoLoader::processNextFrame);
    setMouseTracking(true);
    updateCursorShape();

    // Initialize frame caching system
    m_frameCache = new FrameCache(500); // Default cache size of 50 frames
    startFrameLoader();
}

void VideoLoader::setTrackingDataStorage(TrackingDataStorage* storage) {
    m_storage = storage;

    // If we have valid storage, connect to its signals
    if (m_storage) {
        // Connect to storage's signals to update display when data changes
        // Use the bulk itemsChanged signal to receive the full items list so we can
        // update our internal color map via updateItemsToDisplay(...)
        connect(m_storage, &TrackingDataStorage::itemsChanged, this, &VideoLoader::updateItemsToDisplay);

        connect(m_storage, &TrackingDataStorage::allDataChanged, this, [this]() {
            // Full refresh when all data changes
            update(); // Trigger repaint
        });
    }
}

VideoLoader::~VideoLoader() {
    stopFrameLoader();

    if (m_frameCache) {
        delete m_frameCache;
        m_frameCache = nullptr;
    }

    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
}

// --- Public Methods & Getters ---
bool VideoLoader::isVideoLoaded() const {
    return videoCapture.isOpened() && totalFramesCount > 0;
}
int VideoLoader::getTotalFrames() const { return totalFramesCount; }
double VideoLoader::getFPS() const { return framesPerSecond; }
int VideoLoader::getCurrentFrameNumber() const { return currentFrameIdx; }
QImage VideoLoader::getCurrentQImageFrame() const { return currentQImageFrame;}
QSize VideoLoader::getVideoFrameSize() const { return originalFrameSize; }
double VideoLoader::getZoomFactor() const {
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::getZoomFactor() called - returning:" << m_zoomFactor;
    return m_zoomFactor;
}
QRectF VideoLoader::getCurrentRoi() const { return m_activeRoiRect; }
VideoLoader::InteractionMode VideoLoader::getCurrentInteractionMode() const {
    return m_currentInteractionMode;
}
VideoLoader::ViewModeOptions VideoLoader::getActiveViewModes() const {
    return m_activeViewModes;
}
double VideoLoader::getPlaybackSpeed() const {
    return m_playbackSpeedMultiplier;
}
QString VideoLoader::getCurrentVideoPath() const { return currentFilePath; }

QString VideoLoader::getDataDirectory() const { return m_dataDirectory; }

Thresholding::ThresholdSettings VideoLoader::getCurrentThresholdSettings() const {
    Thresholding::ThresholdSettings settings;
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
Thresholding::ThresholdAlgorithm VideoLoader::getCurrentThresholdAlgorithm() const {
    return m_thresholdAlgorithm;
}
int VideoLoader::getThresholdValue() const { return m_thresholdValue; }
bool VideoLoader::getAssumeLightBackground() const {
    return m_assumeLightBackground;
}
int VideoLoader::getAdaptiveBlockSize() const { return m_adaptiveBlockSize; }
double VideoLoader::getAdaptiveCValue() const { return m_adaptiveC; }
bool VideoLoader::isBlurEnabled() const { return m_enableBlur; }
int VideoLoader::getBlurKernelSize() const { return m_blurKernelSize; }
double VideoLoader::getBlurSigmaX() const { return m_blurSigmaX; }

// --- Control Slots ---
bool VideoLoader::loadVideo(const QString& filePath) {
    if (m_isPlaying) pause();

    // If there are existing annotations in storage, prompt the user before clearing them.
    // Loading a new video will remove all blobs, tracks, and annotations.
    if (m_storage && m_storage->getItemCount() > 0) {
        int resp = QMessageBox::question(this,
                                         "Clear Annotations?",
                                         "Loading a new video will remove all blobs, tracks, and annotations. Do you want to continue?",
                                         QMessageBox::Yes | QMessageBox::No);
        if (resp != QMessageBox::Yes) {
            YAWT_INFO(lcGuiVideoLoader) << "VideoLoader::loadVideo - user cancelled loading due to existing annotations.";
            return false;
        }
        // User confirmed: clear the central storage (this emits signals so models/views update)
        m_storage->removeAllItems();
    }

    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::loadVideo() - resetting zoom factor from" << m_zoomFactor << "to 1.0";
    m_zoomFactor = 1.0;
    m_panOffset = QPointF(0.0, 0.0);
    m_activeRoiRect = QRectF();
    m_activeCropCircleRect = QRectF();
    m_itemsToDisplay.clear();
    clearDisplayedTracks();

    m_isPanning = false;
    m_isDefiningRoi = false;
    m_cropCircleEditMode = CropCircleEditMode::None;

    m_currentInteractionMode = InteractionMode::PanZoom;
    m_activeViewModes = ViewModeOption::None;

    if (!openVideoFile(filePath)) {
        emit videoLoadFailed(filePath, "Failed to open video file with OpenCV.");
        return false;
    }

    currentFilePath = filePath;

    // Clear frame cache for new video
    if (m_frameCache) {
        m_frameCache->clear();
    }

    // Set up frame loader for new video
    if (m_frameLoader) {
        m_frameLoader->clearRequests();
        m_frameLoader->setVideoPath(filePath);
    }

    m_lastPreloadCenter = -1;
    m_pendingSeekFrame = -1;

    // Create data directory for storing analysis results
    m_dataDirectory = createDataDirectory(filePath);
    if (!m_dataDirectory.isEmpty()) {
        emit dataDirectoryChanged(m_dataDirectory);
    }
    framesPerSecond = videoCapture.get(cv::CAP_PROP_FPS);
    if (framesPerSecond <= 0) {
        YAWT_WARN(lcGuiVideoLoader) << "Video FPS reported as 0 or less, defaulting to 25.0";
        framesPerSecond = 25.0;
    }
    totalFramesCount = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    originalFrameSize = QSize(static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH)),
                              static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT)));

    if (totalFramesCount <= 0 || originalFrameSize.isEmpty()) {
        emit videoLoadFailed(filePath, "Video has no frames or invalid dimensions.");
        videoCapture.release();
        return false;
    }

    seekToFrame(0);
    cacheWindowAroundFrame(0, 4); // warm cache for initial display (frames 0-4)
    emit videoLoaded(filePath, totalFramesCount, framesPerSecond, originalFrameSize);
    emit zoomFactorChanged(m_zoomFactor);
    emit roiDefined(m_activeRoiRect);
    emit playbackSpeedChanged(m_playbackSpeedMultiplier);
    emitThresholdParametersChanged();
    emit interactionModeChanged(m_currentInteractionMode);
    emit activeViewModesChanged(m_activeViewModes);
    updateCursorShape();
    YAWT_INFO(lcGuiVideoLoader) << "Video loaded:" << filePath << "- Cache hit rate:" << getCacheHitRate() << "%";
    return true;
}

bool VideoLoader::openVideoFile(const QString& filePath) {
    if (videoCapture.isOpened()) {
        videoCapture.release();
    }
    try {
        return videoCapture.open(filePath.toStdString());
    } catch (const cv::Exception& ex) {
        YAWT_WARN(lcGuiVideoLoader) << "OpenCV exception while opening video:" << ex.what();
        return false;
    }
}

void VideoLoader::play() {
    if (!isVideoLoaded() || m_isPlaying) return;
    if (currentFrameIdx >= totalFramesCount - 1 && totalFramesCount > 0) {
        seekToFrame(0, true);
    }
    if (currentFrameIdx == -1 && totalFramesCount > 0) {
        seekToFrame(0, true);
    }
    m_isPlaying = true;
    updateTimerInterval();
    playbackTimer->start();
    emit playbackStateChanged(true, m_playbackSpeedMultiplier);
}

void VideoLoader::pause() {
    if (!m_isPlaying && !playbackTimer->isActive()) return;
    m_isPlaying = false;
    playbackTimer->stop();
    emit playbackStateChanged(false, m_playbackSpeedMultiplier);
}

void VideoLoader::seekToFrame(int frameNumber, bool suppressEmit) {
    if (!isVideoLoaded()) return;
    frameNumber = qBound(0, frameNumber, totalFramesCount > 0 ? totalFramesCount - 1 : 0);
    displayFrame(frameNumber, suppressEmit);
}

void VideoLoader::setZoomFactor(double factor) {
    setZoomFactorAtPoint(factor, rect().center());
}

void VideoLoader::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint) {
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::setZoomFactorAtPoint() called with factor:" << factor << "current zoom:" << m_zoomFactor;
    if (!isVideoLoaded()) return;
    double newZoomFactor = qBound(0.05, factor, 50.0);
    if (qFuzzyCompare(m_zoomFactor, newZoomFactor)) return;
    QPointF videoPointBeforeZoom = mapPointToVideo(widgetPoint);
    m_zoomFactor = newZoomFactor;
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::setZoomFactorAtPoint() - zoom changed to:" << m_zoomFactor;

    if (videoPointBeforeZoom.x() < 0 || videoPointBeforeZoom.y() < 0) {
        QSizeF widgetSize = size();
        QRectF targetRectNow = calculateTargetRect();
        m_panOffset.setX((widgetSize.width() - targetRectNow.width()) / 2.0 - (targetRectNow.topLeft().x() - m_panOffset.x()));
        m_panOffset.setY((widgetSize.height() - targetRectNow.height()) / 2.0 - (targetRectNow.topLeft().y() - m_panOffset.y()));
    } else {
        QPointF widgetPointAfterZoom = mapPointFromVideo(videoPointBeforeZoom);
        m_panOffset += (widgetPoint - widgetPointAfterZoom);
    }
    clampPanOffset();
    update();
    emit zoomFactorChanged(m_zoomFactor);
}

void VideoLoader::centerOnVideoPoint(const QPointF& videoPoint) {
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::centerOnVideoPoint called with point:" << videoPoint;
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: isVideoLoaded():" << isVideoLoaded() << "zoomFactor:" << m_zoomFactor;

    if (!isVideoLoaded() || m_zoomFactor <= 0) {
        YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::centerOnVideoPoint - early return: video not loaded or invalid zoom";
        return;
    }

    // Bounds checking for video point
    if (videoPoint.isNull() || videoPoint.x() < 0 || videoPoint.y() < 0) {
        YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::centerOnVideoPoint - early return: invalid video point";
        return;
    }

    // Ensure point is within video frame bounds
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: originalFrameSize:" << originalFrameSize << "videoPoint:" << videoPoint;
    if (videoPoint.x() >= originalFrameSize.width() || videoPoint.y() >= originalFrameSize.height()) {
        YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::centerOnVideoPoint - early return: point outside frame bounds";
        return;
    }

    // Get the center of the widget
    QSizeF widgetSize = size();
    QPointF widgetCenter(widgetSize.width() / 2.0, widgetSize.height() / 2.0);
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: widgetSize:" << widgetSize << "widgetCenter:" << widgetCenter;

    // Calculate where the video point currently maps to in widget coordinates
    QPointF currentWidgetPoint = mapPointFromVideo(videoPoint);
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: currentWidgetPoint:" << currentWidgetPoint;

    // If the mapping failed (returns -1, -1 for errors), don't adjust
    if (currentWidgetPoint == QPointF(-1, -1)) {
        YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::centerOnVideoPoint - early return: mapping to widget coordinates failed";
        return;
    }

    // Calculate the offset needed to move the video point to the center
    QPointF offsetNeeded = widgetCenter - currentWidgetPoint;
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: offsetNeeded:" << offsetNeeded << "current m_panOffset:" << m_panOffset;

    // Apply the offset to the pan
    m_panOffset += offsetNeeded;
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: new m_panOffset:" << m_panOffset;

    // Clamp to valid bounds and update
    clampPanOffset();
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: m_panOffset after clamp:" << m_panOffset;
    update();
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader::centerOnVideoPoint - completed successfully";
}

// --- Mode Setting Slots ---
void VideoLoader::setInteractionMode(InteractionMode mode) {
    if (m_currentInteractionMode == mode) return;
    m_currentInteractionMode = mode;
    m_isPanning = false;
    m_isDefiningRoi = false;
    m_cropCircleEditMode = CropCircleEditMode::None;
    if (mode != InteractionMode::Crop) {
        m_activeCropCircleRect = QRectF();
    }
    updateCursorShape();
    emit interactionModeChanged(m_currentInteractionMode);
    YAWT_INFO(lcGuiVideoLoader) << "Interaction mode set to:" << static_cast<int>(m_currentInteractionMode);
    update();
}

void VideoLoader::setViewModeOption(VideoLoader::ViewModeOption option, bool active) {
    ViewModeOptions oldModes = m_activeViewModes;

    m_activeViewModes.setFlag(option, active);

    if (oldModes == m_activeViewModes) {
        return;
    }

    YAWT_DEBUG(lcGuiVideoLoader) << "Active view modes changed. New flags:" << QString::number(static_cast<int>(m_activeViewModes), 16);

    if (isVideoLoaded() && currentFrameIdx >= 0) {
        bool thresholdStateChanged = (oldModes.testFlag(ViewModeOption::Threshold) != m_activeViewModes.testFlag(ViewModeOption::Threshold));
        if (thresholdStateChanged) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emit activeViewModesChanged(m_activeViewModes);
    update();
}


void VideoLoader::clearRoi() {
    if (!m_activeRoiRect.isNull()) {
        m_activeRoiRect = QRectF();
        update();
        emit roiDefined(m_activeRoiRect);
    }
}

void VideoLoader::setPlaybackSpeed(double multiplier) {
    double newSpeed = qBound(0.1, multiplier, 10.0);
    if (qFuzzyCompare(m_playbackSpeedMultiplier, newSpeed)) return;
    m_playbackSpeedMultiplier = newSpeed;
    if (m_isPlaying) {
        playbackTimer->stop();
        updateTimerInterval();
        playbackTimer->start();
    }
    emit playbackSpeedChanged(m_playbackSpeedMultiplier);
    emit playbackStateChanged(m_isPlaying, m_playbackSpeedMultiplier);
}

// --- Thresholding & Pre-processing Control Slots ---
void VideoLoader::setThresholdAlgorithm(Thresholding::ThresholdAlgorithm algorithm) {
    if (m_thresholdAlgorithm == algorithm) return;
    m_thresholdAlgorithm = algorithm;
    if (isVideoLoaded() && currentFrameIdx >= 0) {
        applyThresholding();
        if (m_activeViewModes.testFlag(ViewModeOption::Threshold)) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emitThresholdParametersChanged();
}

void VideoLoader::setThresholdValue(int value) {
    value = qBound(0, value, 255);
    if (m_thresholdValue == value) return;
    m_thresholdValue = value;
    if (isVideoLoaded() && currentFrameIdx >= 0 && m_thresholdAlgorithm == Thresholding::ThresholdAlgorithm::Global) {
        applyThresholding();
        if (m_activeViewModes.testFlag(ViewModeOption::Threshold)) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emitThresholdParametersChanged();
}

void VideoLoader::setAssumeLightBackground(bool isLight) {
    if (m_assumeLightBackground == isLight) return;
    m_assumeLightBackground = isLight;
    if (isVideoLoaded() && currentFrameIdx >= 0) {
        applyThresholding();
        if (m_activeViewModes.testFlag(ViewModeOption::Threshold)) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emitThresholdParametersChanged();
}

void VideoLoader::setAdaptiveThresholdBlockSize(int blockSize) {
    if (blockSize < 3) blockSize = 3;
    if (blockSize % 2 == 0) blockSize += 1;
    if (m_adaptiveBlockSize == blockSize) return;
    m_adaptiveBlockSize = blockSize;
    if (isVideoLoaded() && currentFrameIdx >= 0 &&
        (m_thresholdAlgorithm == Thresholding::ThresholdAlgorithm::AdaptiveMean ||
         m_thresholdAlgorithm == Thresholding::ThresholdAlgorithm::AdaptiveGaussian)) {
        applyThresholding();
        if (m_activeViewModes.testFlag(ViewModeOption::Threshold)) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emitThresholdParametersChanged();
}

void VideoLoader::setAdaptiveThresholdC(double cValue) {
    cValue = qBound(-50.0, cValue, 50.0);
    if (qFuzzyCompare(m_adaptiveC, cValue)) return;
    m_adaptiveC = cValue;
    if (isVideoLoaded() && currentFrameIdx >= 0 &&
        (m_thresholdAlgorithm == Thresholding::ThresholdAlgorithm::AdaptiveMean ||
         m_thresholdAlgorithm == Thresholding::ThresholdAlgorithm::AdaptiveGaussian)) {
        applyThresholding();
        if (m_activeViewModes.testFlag(ViewModeOption::Threshold)) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emitThresholdParametersChanged();
}

void VideoLoader::setEnableBlur(bool enabled) {
    if (m_enableBlur == enabled) return;
    m_enableBlur = enabled;
    if (isVideoLoaded() && currentFrameIdx >= 0) {
        applyThresholding();
        if (m_activeViewModes.testFlag(ViewModeOption::Threshold)) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emitThresholdParametersChanged();
}

void VideoLoader::setBlurKernelSize(int kernelSize) {
    if (kernelSize < 3) kernelSize = 3;
    if (kernelSize % 2 == 0) kernelSize += 1;
    if (m_blurKernelSize == kernelSize) return;
    m_blurKernelSize = kernelSize;
    if (isVideoLoaded() && currentFrameIdx >= 0 && m_enableBlur) {
        applyThresholding();
        if (m_activeViewModes.testFlag(ViewModeOption::Threshold)) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emitThresholdParametersChanged();
}

void VideoLoader::setBlurSigmaX(double sigmaX) {
    sigmaX = qMax(0.0, sigmaX);
    if (qFuzzyCompare(m_blurSigmaX, sigmaX)) return;
    m_blurSigmaX = sigmaX;
    if (isVideoLoaded() && currentFrameIdx >= 0 && m_enableBlur) {
        applyThresholding();
        if (m_activeViewModes.testFlag(ViewModeOption::Threshold)) {
            displayFrame(currentFrameIdx, true);
        }
    }
    emitThresholdParametersChanged();
}

// --- Slots for Data Display from Models ---
void VideoLoader::updateItemsToDisplay(const QList<TableItems::ClickedItem>& items) {
    // For backward compatibility - simply store in our local cache
    // In the future, this won't be needed as we'll get items directly from storage
    m_itemsToDisplay = items;
    for (const TableItems::ClickedItem& item : items) {
        if (item.color.isValid()) {
            m_trackColors[item.id] = item.color;
        }
    }
    if (m_activeViewModes.testFlag(ViewModeOption::Blobs) ||
        m_activeViewModes.testFlag(ViewModeOption::Skeletons)) {
        update();
    }
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: Items to display updated. Count:" << m_itemsToDisplay.size();
}

void VideoLoader::setTracksToDisplay(const Tracking::AllWormTracks& tracks) {
    // For backward compatibility - in the future we'll get tracks directly from storage
    m_allTracksToDisplay = tracks;
    rebuildCenterlineMidpointCache();
    if (m_activeViewModes.testFlag(ViewModeOption::Tracks) ||
        m_activeViewModes.testFlag(ViewModeOption::Blobs) ||
        m_activeViewModes.testFlag(ViewModeOption::Skeletons)) {
        update();
    }
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: Tracks set for display. Count:" << m_allTracksToDisplay.size();
}

void VideoLoader::setVisibleTrackIDs(const QSet<int>& visibleTrackIDs) {
    if (m_visibleTrackIDs == visibleTrackIDs) return;
    m_visibleTrackIDs = visibleTrackIDs;
    if (m_activeViewModes.testFlag(ViewModeOption::Tracks) ||
        m_activeViewModes.testFlag(ViewModeOption::Blobs) ||
        m_activeViewModes.testFlag(ViewModeOption::Skeletons)) {
        update();
    }
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: Visible track IDs updated. Count:" << m_visibleTrackIDs.size();
}

void VideoLoader::clearDisplayedTracks() {
    m_allTracksToDisplay.clear();
    m_visibleTrackIDs.clear();
    m_centerlineMidpointCache.clear();
    // m_trackColors is cleared on new video load.
    if (m_activeViewModes.testFlag(ViewModeOption::Tracks) ||
        m_activeViewModes.testFlag(ViewModeOption::Blobs) ||
        m_activeViewModes.testFlag(ViewModeOption::Skeletons)) {
        update();
    }
    YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: All displayed tracks (data) cleared.";
}

void VideoLoader::setTrackDisplayMode(TrackDisplayMode mode) {
    if (m_trackDisplayMode == mode) return;
    m_trackDisplayMode = mode;
    if (m_activeViewModes.testFlag(ViewModeOption::Tracks))
        update();
}

void VideoLoader::rebuildCenterlineMidpointCache() {
    m_centerlineMidpointCache.clear();
    if (!m_storage) return;
    for (const auto& [wormId, trackPoints] : m_allTracksToDisplay) {
        QMap<int, QPointF>& wormCache = m_centerlineMidpointCache[wormId];
        for (const auto& pt : trackPoints) {
            if (pt.quality == Tracking::TrackPointQuality::Lost) continue;
            const QMap<int, Tracking::DetectedBlob> blobs =
                m_storage->getDetectedBlobsForFrame(pt.frameNumberOriginal);
            auto it = blobs.constFind(wormId);
            if (it != blobs.constEnd() && it->isValid && !it->centerlinePoints.empty()) {
                const auto& clPts = it->centerlinePoints;
                const cv::Point2f& mid = clPts[clPts.size() / 2];
                wormCache[pt.frameNumberOriginal] = QPointF(mid.x, mid.y);
            }
        }
    }
}

/* Per-item color update slot removed.
 *
 * Color updates are now propagated via the bulk `itemsChanged(const QList<TableItems::ClickedItem>&)`
 * signal emitted by `TrackingDataStorage` / `BlobTableModel`. Consumers (including VideoLoader)
 * should rebuild their id->color maps from that list (see setTrackingDataStorage() where the
 * `itemsChanged` connection triggers updates/repaints).
 *
 * The previous per-item `updateWormColor` implementation has been removed to centralize and
 * simplify color propagation.
 */

// --- Frame Processing and Display ---
void VideoLoader::displayFrame(int frameNumber, bool suppressEmit) {
    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) { return; }
    if (frameNumber < 0 || frameNumber >= totalFramesCount) { return; }

    // First, try to get frame from cache
    cv::Mat cachedFrame;
    bool frameFromCache = false;

    if (getCachedFrame(frameNumber, cachedFrame)) {
        // Frame found in cache - use it directly
        currentCvFrame = cachedFrame;
        frameFromCache = true;
    } else {
        // Frame not in cache - load from disk
        int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
        if (currentPos != frameNumber && !(currentPos == frameNumber - 1 && m_isPlaying)) {
            if (!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) {
                // If direct loading fails, try background loading
                m_pendingSeekFrame = frameNumber;
                if (m_frameLoader) {
                    m_frameLoader->requestSingleFrame(frameNumber, 100); // High priority
                }
                YAWT_INFO(lcGuiVideoLoader) << "Seek FAILED. Requesting frame" << frameNumber << "via FrameLoader";
                return;
            }
            // ADD THIS LINE
            YAWT_DEBUG(lcGuiVideoLoader) << "Seek SUCCEEDED. Now reading frame" << frameNumber << "from disk.";

        } else {
            YAWT_DEBUG(lcGuiVideoLoader) << "Not playing -- " << frameNumber << ". Reading directly.";
        }

        if (videoCapture.read(currentCvFrame)) {
            if (!currentCvFrame.empty()) {
                // Cache the newly loaded frame
                if (m_frameCache) {
                    m_frameCache->insertFrame(frameNumber, currentCvFrame);
                }

                // Avoid background preloading when paused; cache windows are managed explicitly elsewhere.
                if (m_isPlaying && m_frameLoader) {
                    int f1 = frameNumber + 1;
                    int f2 = frameNumber + 2;

                    if (f1 >= 0 && (totalFramesCount <= 0 || f1 < totalFramesCount)) {
                        if (!m_frameCache || !m_frameCache->hasFrame(f1)) {
                            m_frameLoader->requestSingleFrame(f1, 100);
                        }
                    }

                    if (f2 >= 0 && (totalFramesCount <= 0 || f2 < totalFramesCount)) {
                        if (!m_frameCache || !m_frameCache->hasFrame(f2)) {
                            m_frameLoader->requestSingleFrame(f2, 100);
                        }
                    }
                }

            } else {
                currentCvFrame = cv::Mat(); m_thresholdedFrame_mono = cv::Mat();
                return;
            }
        } else {
            currentCvFrame = cv::Mat(); m_thresholdedFrame_mono = cv::Mat();
            if (m_isPlaying) pause();
            return;
        }
    }

    if (!currentCvFrame.empty()) {
        currentFrameIdx = frameNumber;
        applyThresholding();

        // Trigger preloading of adjacent frames (but not during rapid seeking)
        static QDateTime lastPreloadTime;
        QDateTime now = QDateTime::currentDateTime();
        //if (lastPreloadTime.msecsTo(now) > 100) { // Throttle preloading
        //    preloadAdjacentFrames(frameNumber, m_isPlaying ? 10 : 5);
        //    lastPreloadTime = now;
        //}
    }

    if (m_activeViewModes.testFlag(ViewModeOption::Threshold) && !m_thresholdedFrame_mono.empty()) {
        convertCvMatToQImage(m_thresholdedFrame_mono, currentQImageFrame);
    } else if (!currentCvFrame.empty()) {
        convertCvMatToQImage(currentCvFrame, currentQImageFrame);
    } else {
        currentQImageFrame = QImage(originalFrameSize, QImage::Format_RGB888);
        currentQImageFrame.fill(Qt::darkGray);
    }

    if (!suppressEmit) {
        emit frameChanged(currentFrameIdx, currentQImageFrame);
    }
    update();
}

void VideoLoader::convertCvMatToQImage(const cv::Mat& mat, QImage& qimg) {
    if (mat.empty()) { qimg = QImage(); return; }
    if (mat.type() == CV_8UC3) {
        qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped();
    } else if (mat.type() == CV_8UC1) {
        qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8);
    } else {
        cv::Mat temp;
        try {
            if (mat.channels() == 4) { cv::cvtColor(mat, temp, cv::COLOR_BGRA2BGR); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 3 && mat.type() != CV_8UC3) { mat.convertTo(temp, CV_8UC3, 255.0); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888).rgbSwapped(); }
            else if (mat.channels() == 1 && mat.type() != CV_8UC1) { mat.convertTo(temp, CV_8UC1, 255.0); qimg = QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_Grayscale8); }
            else { qimg = QImage(); }
        } catch (const cv::Exception& ex) {
            YAWT_WARN(lcGuiVideoLoader) << "OpenCV conversion exception:" << ex.what(); qimg = QImage();
        }
    }
}

void VideoLoader::processNextFrame() {
    if (!m_isPlaying || !isVideoLoaded()) return;
    if (currentFrameIdx < totalFramesCount - 1) {
        displayFrame(currentFrameIdx + 1);
    } else {
        pause();
    }
}

// --- Event Handlers ---
void VideoLoader::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    if (currentQImageFrame.isNull() || !isVideoLoaded()) {
        painter.fillRect(rect(), palette().color(QPalette::Window));
        painter.setPen(Qt::gray);
        painter.drawText(rect(), Qt::AlignCenter, "Video not loaded or frame unavailable.");
        return;
    }

    QRectF targetRect = calculateTargetRect();
    painter.drawImage(targetRect, currentQImageFrame, currentQImageFrame.rect());

    // Draw general purpose ROI if active
    if (!m_activeRoiRect.isNull() && m_activeRoiRect.isValid() &&
        m_currentInteractionMode == InteractionMode::DrawROI) {
        QPointF roiTopLeftWidget = mapPointFromVideo(m_activeRoiRect.topLeft());
        QPointF roiBottomRightWidget = mapPointFromVideo(m_activeRoiRect.bottomRight());
        if (roiTopLeftWidget.x() >= 0 && roiBottomRightWidget.x() >= 0) {
            painter.setPen(QPen(Qt::red, 2, Qt::DashLine));
            painter.drawRect(QRectF(roiTopLeftWidget, roiBottomRightWidget).normalized());
        }
    }

    if (m_currentInteractionMode == InteractionMode::Crop &&
        !m_activeCropCircleRect.isNull() && m_activeCropCircleRect.isValid()) {
        const QPointF cropTopLeftWidget = mapPointFromVideo(m_activeCropCircleRect.topLeft());
        const QPointF cropBottomRightWidget = mapPointFromVideo(m_activeCropCircleRect.bottomRight());
        if (cropTopLeftWidget.x() >= 0 && cropBottomRightWidget.x() >= 0) {
            const QRectF cropCircleWidget = QRectF(cropTopLeftWidget, cropBottomRightWidget).normalized();
            painter.setPen(QPen(Qt::red, 2, Qt::DashLine));
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(cropCircleWidget);

            const QPointF handlePoint = QPointF(cropCircleWidget.right(), cropCircleWidget.center().y());
            painter.setPen(QPen(Qt::cyan, 2, Qt::SolidLine));
            painter.drawRect(QRectF(handlePoint.x() - 4.0, handlePoint.y() - 4.0, 8.0, 8.0));
        }
    }

    // Draw temporary ROI during definition
    if (m_currentInteractionMode == InteractionMode::DrawROI && m_isDefiningRoi) {
        painter.setPen(QPen(Qt::cyan, 1, Qt::SolidLine));
        painter.drawRect(QRect(m_roiStartPointWidget, m_roiEndPointWidget).normalized());
    }

    auto drawPointMarker = [&](const QPointF& centerWidget, const QColor& color) {
        QPen pointPen(color, 2);
        painter.setPen(pointPen);
        painter.setBrush(Qt::NoBrush);
        const qreal radius = 6.0;
        const qreal plusHalf = 4.0;
        painter.drawEllipse(centerWidget, radius, radius);
        painter.drawLine(QPointF(centerWidget.x() - plusHalf, centerWidget.y()),
                         QPointF(centerWidget.x() + plusHalf, centerWidget.y()));
        painter.drawLine(QPointF(centerWidget.x(), centerWidget.y() - plusHalf),
                         QPointF(centerWidget.x(), centerWidget.y() + plusHalf));
    };

    auto isPointItem = [](const TableItems::ClickedItem& item) {
        if (item.type == TableItems::ItemType::StartPoint ||
            item.type == TableItems::ItemType::EndPoint ||
            item.type == TableItems::ItemType::ControlPoint) {
            return true;
        }
        if (item.type == TableItems::ItemType::ROI) {
            const QRectF& ob = item.originalClickedBoundingBox;
            return ob.isNull() || ob.isEmpty() || ob.width() <= 0.0 || ob.height() <= 0.0;
        }
        return false;
    };

    auto drawPersistentPointItems = [&]() {
        const QList<TableItems::ClickedItem>& itemsToDisplay =
            m_storage ? m_storage->getAllItems() : m_itemsToDisplay;

        for (const TableItems::ClickedItem& item : std::as_const(itemsToDisplay)) {
            if (!item.visible || !isPointItem(item)) {
                continue;
            }

            const QPointF centroidWidget = mapPointFromVideo(item.initialCentroid);
            if (centroidWidget.x() >= 0) {
                drawPointMarker(centroidWidget, getTrackColor(item.id));
            }
        }
    };

    // --- MODIFIED BLOB DRAWING LOGIC ---
    if (m_activeViewModes.testFlag(ViewModeOption::Blobs)) {
        if (!m_allTracksToDisplay.empty() && currentFrameIdx >= 0) {
            // Tracking has run, display current frame's blob positions from tracks
            for (int trackId : std::as_const(m_visibleTrackIDs)) {
                if (m_allTracksToDisplay.count(trackId)) {
                    // Check if this track's item is set to visible
                    bool isVisible = false;
                    if (m_storage) {
                        // Get item from storage if available
                        const TableItems::ClickedItem* item = m_storage->getItem(trackId);
                        if (item) {
                            isVisible = item->visible;
                        }
                    } else {
                        // Fallback to legacy method
                        for (const TableItems::ClickedItem& item : std::as_const(m_itemsToDisplay)) {
                            if (item.id == trackId) {
                                isVisible = item.visible;
                                break;
                            }
                        }
                    }
                    if (!isVisible) continue;

                    // Use optimized lookup instead of linear search
                    QPointF wormPosition;
                    QRectF wormRoi;
                    if (m_storage && m_storage->getWormDataForFrame(trackId, currentFrameIdx, wormPosition, wormRoi)) {
                        QColor itemColor = getTrackColor(trackId);

                        // Draw Bounding Box for current frame from track data
                        QRectF bboxVideo = wormRoi;
                        if (bboxVideo.isValid()) {
                            QPointF bbTopLeftWidget = mapPointFromVideo(bboxVideo.topLeft());
                            QPointF bbBottomRightWidget = mapPointFromVideo(bboxVideo.bottomRight());
                            if (bbTopLeftWidget.x() >= 0 && bbBottomRightWidget.x() >= 0) { // Check if on screen
                                QPen blobPen(itemColor); // Create a pen with the item's color
                                blobPen.setStyle(Qt::DotLine); // Set style for bounding box
                                blobPen.setWidth(1);          // Set width for bounding box
                                painter.setPen(blobPen);
                                painter.drawRect(QRectF(bbTopLeftWidget, bbBottomRightWidget).normalized());
                            }
                        }

                        // Draw Centroid for current frame from track data
                        QPointF centroidVideo = wormPosition;
                        QPointF centroidWidget = mapPointFromVideo(centroidVideo);
                        if (centroidWidget.x() >= 0) { // Check if on screen
                            painter.setPen(QPen(itemColor, 2)); // Outline for centroid
                            painter.setBrush(itemColor);        // Fill for centroid
                            painter.drawEllipse(centroidWidget, 3, 3); // Draw a small circle for the centroid
                            painter.setBrush(Qt::NoBrush);      // Reset brush
                        }
                    }
                }
            }
        } else if ((m_storage && m_storage->getItemCount() > 0) || !m_itemsToDisplay.isEmpty()) {
            // Fallback: Tracking not run or no tracks, display initial blob selections

            // If storage is available, use it
            const QList<TableItems::ClickedItem>& itemsToDisplay =
                m_storage ? m_storage->getAllItems() : m_itemsToDisplay;

            for (const TableItems::ClickedItem& item : std::as_const(itemsToDisplay)) {
                // Only show items with visible checkbox checked
                if (!item.visible) continue;

                // No need to filter by selection - visibility is controlled only by the checkbox

                QColor itemColor = getTrackColor(item.id);

                if (isPointItem(item)) continue;

                // Draw Initial Bounding Box
                QRectF bboxVideo = item.initialBoundingBox;
                QPointF bbTopLeftWidget = mapPointFromVideo(bboxVideo.topLeft());
                QPointF bbBottomRightWidget = mapPointFromVideo(bboxVideo.bottomRight());
                if (bbTopLeftWidget.x() >= 0 && bbBottomRightWidget.x() >= 0) {
                    QPen blobPen(itemColor);
                    blobPen.setStyle(Qt::SolidLine); // Solid line for initial selections
                    blobPen.setWidth(1);
                    painter.setPen(blobPen);
                    painter.drawRect(QRectF(bbTopLeftWidget, bbBottomRightWidget).normalized());
                }

                // Draw Initial Centroid
                QPointF centroidVideo = item.initialCentroid;
                QPointF centroidWidget = mapPointFromVideo(centroidVideo);
                if (centroidWidget.x() >= 0) {
                    painter.setPen(QPen(itemColor, 2));
                    painter.setBrush(itemColor);
                    painter.drawEllipse(centroidWidget, 3, 3);
                    painter.setBrush(Qt::NoBrush);
                }

                if (item.hasHeadTailSeed) {
                    const QPointF headWidget = mapPointFromVideo(item.initialHeadPoint);
                    const QPointF tailWidget = mapPointFromVideo(item.initialTailPoint);
                    if (headWidget.x() >= 0) {
                        painter.setPen(QPen(QColor(0, 220, 255), 2));
                        painter.setBrush(QColor(0, 220, 255));
                        painter.drawEllipse(headWidget, 4, 4);
                        painter.drawText(headWidget + QPointF(5, -5), "H");
                    }
                    if (tailWidget.x() >= 0) {
                        painter.setPen(QPen(QColor(255, 80, 80), 2));
                        painter.setBrush(QColor(255, 80, 80));
                        painter.drawEllipse(tailWidget, 4, 4);
                        painter.drawText(tailWidget + QPointF(5, -5), "T");
                    }
                    painter.setBrush(Qt::NoBrush);
                }
            }
        }
    }
    // --- END OF MODIFIED BLOB DRAWING LOGIC ---

    drawPersistentPointItems();

    if (m_activeViewModes.testFlag(ViewModeOption::Skeletons) && m_storage && currentFrameIdx >= 0) {
        const QMap<int, Tracking::DetectedBlob> blobs = m_storage->getDetectedBlobsForFrame(currentFrameIdx);
        QSet<int> skeletonIds = m_visibleTrackIDs;
        if (skeletonIds.isEmpty()) {
            for (auto it = blobs.constBegin(); it != blobs.constEnd(); ++it) {
                skeletonIds.insert(it.key());
            }
        }

        for (int wormId : std::as_const(skeletonIds)) {
            const TableItems::ClickedItem* item = m_storage->getItem(wormId);
            if (!item || !item->visible) {
                continue;
            }

            const Tracking::DetectedBlob blob = blobs.value(wormId);
            if (!blob.isValid || blob.contourPoints.empty()) {
                continue;
            }

            const QList<QPointF> centerlinePoints =
                Tracking::extractResampledCenterlinePoints(blob, 10);
            if (centerlinePoints.isEmpty()) {
                continue;
            }

            QPolygonF centerlinePolyline;
            centerlinePolyline.reserve(centerlinePoints.size());
            for (int pointIndex = 0; pointIndex < centerlinePoints.size(); ++pointIndex) {
                const QPointF widgetPoint = mapPointFromVideo(centerlinePoints.at(pointIndex));
                if (widgetPoint.x() < 0) {
                    continue;
                }
                centerlinePolyline.append(widgetPoint);
            }

            if (centerlinePolyline.size() < 2) {
                continue;
            }

            const QColor centerlineColor = getTrackColor(wormId);
            QPen centerlinePen(centerlineColor, CENTERLINE_LINE_WIDTH);
            centerlinePen.setCosmetic(true);
            centerlinePen.setCapStyle(Qt::RoundCap);
            centerlinePen.setJoinStyle(Qt::RoundJoin);
            painter.setPen(centerlinePen);
            painter.setBrush(Qt::NoBrush);
            painter.drawPolyline(centerlinePolyline);

            // Mark nose (front), tail (back) and centerline-centroid (middle).
            // Different fills make head/tail orientation legible at a glance and
            // the centroid distinguishes the centerline-derived center from the
            // blob centroid (which can be inside the hole of a coiled worm).
            const qreal endpointRadius = CENTERLINE_LINE_WIDTH * 1.4;
            const qreal centroidRadius = CENTERLINE_LINE_WIDTH * 1.1;
            const QPointF nosePoint = centerlinePolyline.first();
            const QPointF tailPoint = centerlinePolyline.last();
            const QPointF midPoint  = centerlinePolyline.at(centerlinePolyline.size() / 2);

            QPen dotPen(Qt::black, 1.0);
            dotPen.setCosmetic(true);
            painter.setPen(dotPen);

            painter.setBrush(QColor(0x70, 0xE6, 0x9D));  // head
            painter.drawEllipse(nosePoint, endpointRadius, endpointRadius);
            painter.setBrush(QColor(0xD0, 0xB8, 0xE0));  // tail
            painter.drawEllipse(tailPoint, endpointRadius, endpointRadius);
            painter.setBrush(QColor(255, 255, 0));   // yellow = centerline-centroid
            painter.drawEllipse(midPoint, centroidRadius, centroidRadius);

            if (blob.hasCenterlineCutPoint) {
                const QPointF cutPoint = mapPointFromVideo(
                    QPointF(blob.centerlineCutPoint.x, blob.centerlineCutPoint.y));
                if (cutPoint.x() >= 0) {
                    const qreal cutRadius = CENTERLINE_LINE_WIDTH * 1.25;
                    painter.setPen(QPen(Qt::black, 1.0));
                    painter.setBrush(QColor(255, 0, 255));
                    painter.drawRect(QRectF(cutPoint.x() - cutRadius,
                                            cutPoint.y() - cutRadius,
                                            cutRadius * 2.0,
                                            cutRadius * 2.0));
                }
            }
        }
    }

    // ── Tip-candidate overlay ────────────────────────────────────────────────
    // Phase-B preprocessing dots: nose/tail candidates produced per-frame from
    // the blob's geometry alone (no temporal state). Skeleton-derived
    // candidates render as filled circles; curvature-peak candidates as
    // hollow squares — so it's visually obvious which detector surfaced each
    // tip during debugging.
    if (m_activeViewModes.testFlag(ViewModeOption::TipCandidates) && m_storage && currentFrameIdx >= 0) {
        const QMap<int, Tracking::DetectedBlob> tipBlobs = m_storage->getDetectedBlobsForFrame(currentFrameIdx);
        QSet<int> tipIds = m_visibleTrackIDs;
        if (tipIds.isEmpty()) {
            for (auto it = tipBlobs.constBegin(); it != tipBlobs.constEnd(); ++it) {
                tipIds.insert(it.key());
            }
        }

        for (int wormId : std::as_const(tipIds)) {
            const TableItems::ClickedItem* item = m_storage->getItem(wormId);
            if (!item || !item->visible) continue;

            const Tracking::DetectedBlob blob = tipBlobs.value(wormId);
            if (!blob.isValid || blob.tipCandidates.empty()) continue;

            const qreal dotRadius  = CENTERLINE_LINE_WIDTH * 1.05;
            const qreal ringRadius = CENTERLINE_LINE_WIDTH * 1.95;
            QPen edgePen(Qt::black, 1.2);
            edgePen.setCosmetic(true);

            // Phase B: candidate dots (one per tipCandidate).
            for (const Tracking::TipCandidate& tc : blob.tipCandidates) {
                const QPointF pt = mapPointFromVideo(QPointF(tc.point.x, tc.point.y));
                if (pt.x() < 0) continue;

                painter.setPen(edgePen);
                if (tc.source == Tracking::TipCandidate::Source::SkeletonEndpoint) {
                    // Filled green circle — skeleton degree-1 endpoint.
                    painter.setBrush(QColor(60, 220, 80));
                    painter.drawEllipse(pt, dotRadius, dotRadius);
                } else if (tc.source == Tracking::TipCandidate::Source::CurvaturePeak) {
                    // Hollow magenta square — curvature peak.
                    painter.setBrush(Qt::NoBrush);
                    QPen peakPen(QColor(220, 60, 220), 1.6);
                    peakPen.setCosmetic(true);
                    painter.setPen(peakPen);
                    painter.drawRect(QRectF(pt.x() - dotRadius, pt.y() - dotRadius,
                                            dotRadius * 2.0, dotRadius * 2.0));
                } else {
                    // Hollow orange square — D-3 hypothesized hidden tip.
                    painter.setBrush(Qt::NoBrush);
                    QPen hypPen(QColor(255, 165, 0), 1.6);
                    hypPen.setCosmetic(true);
                    painter.setPen(hypPen);
                    painter.drawRect(QRectF(pt.x() - dotRadius, pt.y() - dotRadius,
                                            dotRadius * 2.0, dotRadius * 2.0));
                }
            }

            // Phase C.1: head/tail assignment overlay. Drawn over the
            // candidate dots so the assignment is visually obvious without
            // hiding which detector surfaced each tip.
            auto drawRoleRing = [&](int idx, const QColor& color) {
                if (idx < 0 || idx >= static_cast<int>(blob.tipCandidates.size())) return;
                const cv::Point2f& cv = blob.tipCandidates[idx].point;
                const QPointF pt = mapPointFromVideo(QPointF(cv.x, cv.y));
                if (pt.x() < 0) return;
                QPen ringPen(color, 2.0);
                ringPen.setCosmetic(true);
                painter.setPen(ringPen);
                painter.setBrush(Qt::NoBrush);
                painter.drawEllipse(pt, ringRadius, ringRadius);
            };
            drawRoleRing(blob.assignedHeadTipIdx, QColor(0x70, 0xE6, 0x9D));  // head
            drawRoleRing(blob.assignedTailTipIdx, QColor(0xD0, 0xB8, 0xE0));  // tail
        }
    }

    // Draw Tracks if ViewMode is Tracks
    if (m_activeViewModes.testFlag(ViewModeOption::Tracks) && !m_allTracksToDisplay.empty() && currentFrameIdx >= 0) {
        // Paint tracks for all worms
        // For each track, paint its path
        for (auto it_map = m_allTracksToDisplay.cbegin(); it_map != m_allTracksToDisplay.cend(); ++it_map) { // Use different iterator name
            int trackId = it_map->first;
            const std::vector<Tracking::WormTrackPoint>& trackPoints = it_map->second;

            // Only show tracks for items with visible checkbox checked
            bool isVisible = false;
            if (m_storage) {
                // Get item from storage if available
                const TableItems::ClickedItem* item = m_storage->getItem(trackId);
                if (item) {
                    isVisible = item->visible;
                }
            } else {
                // Fallback to legacy method
                for (const TableItems::ClickedItem& item : std::as_const(m_itemsToDisplay)) {
                    if (item.id == trackId) {
                        isVisible = item.visible;
                        break;
                    }
                }
            }

            if (!isVisible || !m_visibleTrackIDs.contains(trackId) || trackPoints.empty()) continue;

            QPainterPath path;
            QColor trackColorWithAlpha = getTrackColor(trackId); // Assuming getTrackColor provides color with desired alpha
            // If getTrackColor returns opaque, set alpha here:
            // trackColorWithAlpha.setAlphaF(0.5); // Example: 50% opacity for lines

            QPen trackPen(trackColorWithAlpha, TRACK_LINE_WIDTH);
            painter.setPen(trackPen);

            const QMap<int, QPointF>* clCache = nullptr;
            if (m_trackDisplayMode == TrackDisplayMode::CenterlineMidpoint) {
                auto cit = m_centerlineMidpointCache.constFind(trackId);
                if (cit != m_centerlineMidpointCache.constEnd())
                    clCache = &(*cit);
            }

            bool firstPoint = true;
            for (const Tracking::WormTrackPoint& pt : trackPoints) {
                // Skip lost tracking points - they create gaps in the track display
                if (pt.quality == Tracking::TrackPointQuality::Lost) {
                    continue;
                }

                QPointF currentPointVideo;
                if (clCache) {
                    auto fit = clCache->constFind(pt.frameNumberOriginal);
                    currentPointVideo = (fit != clCache->constEnd())
                        ? *fit
                        : QPointF(pt.position.x, pt.position.y);
                } else {
                    currentPointVideo = QPointF(pt.position.x, pt.position.y);
                }

                QPointF currentPointWidget = mapPointFromVideo(currentPointVideo);
                if (currentPointWidget.x() < 0) continue; // Skip points not visible on screen

                if (firstPoint) {
                    path.moveTo(currentPointWidget);
                    firstPoint = false;
                } else {
                    path.lineTo(currentPointWidget);
                }
                // To make points less obtrusive if lines are the focus:
                // painter.setBrush(trackColorWithAlpha);
                // painter.drawEllipse(currentPointWidget, 1, 1); // Smaller points
                // painter.setBrush(Qt::NoBrush);
            }
            if (!firstPoint) { // If path has content
                painter.strokePath(path, trackPen); // Draw the continuous line
            }
        }
    }
} // End of paintEvent

void VideoLoader::mousePressEvent(QMouseEvent* event) {
    m_lastMousePos = event->position();
    if (!isVideoLoaded()) { QWidget::mousePressEvent(event); return; }

    if (event->button() == Qt::LeftButton) {
        if (m_currentInteractionMode == InteractionMode::Crop) {
            const QPointF videoCoords = mapPointToVideo(event->position());
            if (videoCoords.x() >= 0) {
                m_cropCircleEditMode = CropCircleEditMode::Creating;
                if (!m_activeCropCircleRect.isNull() && m_activeCropCircleRect.isValid()) {
                    const QPointF center = m_activeCropCircleRect.center();
                    const qreal radius = m_activeCropCircleRect.width() / 2.0;
                    const qreal dx = videoCoords.x() - center.x();
                    const qreal dy = videoCoords.y() - center.y();
                    const qreal distance = qSqrt(dx * dx + dy * dy);
                    const qreal edgeTolerance = qMax(8.0 / qMax(m_zoomFactor, 0.001), radius * 0.05);
                    if (qAbs(distance - radius) <= edgeTolerance) {
                        m_cropCircleEditMode = CropCircleEditMode::Resizing;
                    } else if (distance < radius) {
                        m_cropCircleEditMode = CropCircleEditMode::Moving;
                    } else {
                        m_activeCropCircleRect = QRectF();
                    }
                }

                m_cropDragStartVideo = videoCoords;
                m_cropDragStartRect = m_activeCropCircleRect;
                m_roiStartPointWidget = event->pos();
                m_roiEndPointWidget = event->pos();
                m_isDefiningRoi = true;
                update();
                event->accept();
            } else {
                m_isDefiningRoi = false;
                m_cropCircleEditMode = CropCircleEditMode::None;
            }
            return;
        }

        if (m_currentInteractionMode == InteractionMode::DrawROI) {
            QPointF videoCoords = mapPointToVideo(event->position());
            if (videoCoords.x() >= 0) {
                m_roiStartPointWidget = event->pos(); m_roiEndPointWidget = event->pos();
                m_isDefiningRoi = true; update(); event->accept();
            } else { m_isDefiningRoi = false; }
            return;
        }

        if (m_currentInteractionMode == InteractionMode::Point) {
            QPointF videoPoint = mapPointToVideo(event->position());
            if (videoPoint.x() >= 0) {
                emit pointDefined(videoPoint);
                event->accept();
                return;
            }
        }

        // For other modes, first attempt mode-specific handling, otherwise fall back to panning.
        if (m_currentInteractionMode == InteractionMode::EditBlobs) {
            bool handled = false;
            if (!m_thresholdedFrame_mono.empty()) {
                QPointF clickVideoPoint = mapPointToVideo(event->position());
                if (clickVideoPoint.x() >= 0) {
                    Tracking::DetectedBlob blobData = Tracking::findClickedBlob(m_thresholdedFrame_mono, clickVideoPoint);
                    if (blobData.isValid) {
                        YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: Blob clicked for addition. Centroid:" << blobData.centroid;
                        emit blobClickedForAddition(blobData, clickVideoPoint);
                        handled = true;
                    } else {
                        YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: No valid blob found at click point:" << clickVideoPoint;
                    }
                }
            } else {
                YAWT_INFO(lcGuiVideoLoader) << "VideoLoader: Cannot select blob; internal thresholded frame is not available.";
            }

            if (!handled) {
                // Fallback to panning
                m_isPanning = true; updateCursorShape(); event->accept();
            } else {
                event->accept();
            }
            return;
        }

        if (m_currentInteractionMode == InteractionMode::EditTracks) {
            QPointF clickWidgetPoint = event->position();
            int bestTrackId = -1; int bestFrameNum = -1; QPointF bestVideoPoint;
            double minDistanceSq = TRACK_POINT_CLICK_TOLERANCE * TRACK_POINT_CLICK_TOLERANCE;
            for (int trackId : std::as_const(m_visibleTrackIDs)) {
                // Only interact with tracks for items with visible checkbox checked
                bool isVisible = false;
                if (m_storage) {
                    // Get item from storage if available
                    const TableItems::ClickedItem* item = m_storage->getItem(trackId);
                    if (item) {
                        isVisible = item->visible;
                    }
                } else {
                    // Fallback to legacy method
                    for (const TableItems::ClickedItem& item : std::as_const(m_itemsToDisplay)) {
                        if (item.id == trackId) {
                            isVisible = item.visible;
                            break;
                        }
                    }
                }
                if (!isVisible) continue;

                if (m_allTracksToDisplay.count(trackId)) {
                    const auto& trackPoints = m_allTracksToDisplay.at(trackId);
                    for (const auto& pt : trackPoints) {
                        // Skip lost tracking points for mouse interaction
                        if (pt.quality == Tracking::TrackPointQuality::Lost) {
                            continue;
                        }

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
                YAWT_INFO(lcGuiVideoLoader) << "Track point clicked: Worm ID" << bestTrackId << "Frame" << bestFrameNum;
                emit trackPointClicked(bestTrackId, bestFrameNum, bestVideoPoint);
                event->accept();
            } else {
                // Fallback to panning
                m_isPanning = true; updateCursorShape(); event->accept();
            }
            return;
        }

        // Default / PanZoom or any other modes: start panning
        m_isPanning = true; updateCursorShape(); event->accept();
    } else {
        QWidget::mousePressEvent(event);
    }
}

// ... (mouseMoveEvent, mouseReleaseEvent, wheelEvent, resizeEvent, and private helpers remain the same) ...
// (Make sure to copy the full content of these functions from your previous complete version)
void VideoLoader::mouseMoveEvent(QMouseEvent* event) {
    QPointF currentPos = event->position();
    QPointF delta = currentPos - m_lastMousePos;
    m_lastMousePos = currentPos;
    if (!isVideoLoaded()) {
        QWidget::mouseMoveEvent(event);
        return;
    }
    if (m_currentInteractionMode == InteractionMode::Crop &&
        m_isDefiningRoi && (event->buttons() & Qt::LeftButton)) {
        const QPointF videoCoords = mapPointToVideo(event->position());
        if (videoCoords.x() >= 0) {
            m_roiEndPointWidget = event->pos();
            if (m_cropCircleEditMode == CropCircleEditMode::Creating) {
                m_activeCropCircleRect = constrainCropCircleToFrame(
                    squareFromDiagonal(m_cropDragStartVideo, videoCoords));
            } else if (m_cropCircleEditMode == CropCircleEditMode::Moving) {
                const QPointF delta = videoCoords - m_cropDragStartVideo;
                m_activeCropCircleRect = constrainCropCircleToFrame(
                    m_cropDragStartRect.translated(delta));
            } else if (m_cropCircleEditMode == CropCircleEditMode::Resizing) {
                const QPointF center = m_cropDragStartRect.center();
                const qreal dx = videoCoords.x() - center.x();
                const qreal dy = videoCoords.y() - center.y();
                m_activeCropCircleRect = cropCircleFromCenterAndRadius(center, qSqrt(dx * dx + dy * dy));
            }
            update();
        }
        event->accept();
    } else if (m_isPanning && (event->buttons() & Qt::LeftButton)) {
        m_panOffset += delta;
        clampPanOffset();
        update();
        event->accept();
    } else if (m_currentInteractionMode == InteractionMode::DrawROI &&
               m_isDefiningRoi && (event->buttons() & Qt::LeftButton)) {
        m_roiEndPointWidget = event->pos();
        update();
        event->accept();
    } else {
        QWidget::mouseMoveEvent(event);
    }
}
void VideoLoader::mouseReleaseEvent(QMouseEvent* event) {
    if (!isVideoLoaded()) {
        QWidget::mouseReleaseEvent(event);
        return;
    }
    if (event->button() == Qt::LeftButton) {
        if (m_isPanning) {
            m_isPanning = false;
            updateCursorShape();
            event->accept();
        } else if (m_isDefiningRoi && m_currentInteractionMode == InteractionMode::Crop) {
            m_isDefiningRoi = false;
            if (m_activeCropCircleRect.width() < 5.0 || m_activeCropCircleRect.height() < 5.0) {
                m_activeCropCircleRect = QRectF();
            }
            m_cropCircleEditMode = CropCircleEditMode::None;
            update();
            updateCursorShape();
            event->accept();
        } else if (m_isDefiningRoi && m_currentInteractionMode == InteractionMode::DrawROI) {
            m_roiEndPointWidget = event->pos();
            m_isDefiningRoi = false;
            QPointF vs = mapPointToVideo(m_roiStartPointWidget),
                ve = mapPointToVideo(m_roiEndPointWidget);
            QRectF dRect;
            if (vs.x() >= 0 && ve.x() >= 0) {
                dRect = QRectF(vs, ve).normalized();
                if (dRect.width() < 5 || dRect.height() < 5) dRect = QRectF();
            }

            m_activeRoiRect = dRect;
            emit roiDefined(m_activeRoiRect);
            update();
            updateCursorShape();
            event->accept();
        }
    } else {
        QWidget::mouseReleaseEvent(event);
    }
}

void VideoLoader::mouseDoubleClickEvent(QMouseEvent* event) {
    if (!isVideoLoaded()) {
        QWidget::mouseDoubleClickEvent(event);
        return;
    }

    if (event->button() == Qt::LeftButton &&
        m_currentInteractionMode == InteractionMode::Crop &&
        !m_activeCropCircleRect.isNull() && m_activeCropCircleRect.isValid()) {
        const QPointF videoCoords = mapPointToVideo(event->position());
        if (videoCoords.x() >= 0 && pointInCircleRect(videoCoords, m_activeCropCircleRect)) {
            handleRoiDefinedForCrop(m_activeCropCircleRect);
            event->accept();
            return;
        }
    }

    QWidget::mouseDoubleClickEvent(event);
}


void VideoLoader::wheelEvent(QWheelEvent* event) {
    if (!isVideoLoaded()) {
        event->ignore();
        return;
    }

    int steps = 0;
    double zs_for_this_event; // Zoom speed factor (e.g., 0.15 for 15%)
    QPointF zoomAtPos = event->position();

    bool isTouchpad = (event->device()->type() == QInputDevice::DeviceType::TouchPad);
    bool hasPixelDelta = !event->pixelDelta().isNull() && event->pixelDelta().y() != 0;
    bool hasAngleDelta = !event->angleDelta().isNull() && event->angleDelta().y() != 0;

    if (isTouchpad && hasPixelDelta) {
        // SCENARIO: Touchpad, pixelDelta().y() is small (e.g., +/-1), events are frequent.
        //qDebug() << "Trackpad event (using pixelDelta):" << event->pixelDelta().y();

        // Directly use the small delta as steps.
        // If pixelDelta().y() is indeed +/-1, then steps will be +/-1.
        steps = event->pixelDelta().y();

        // *** KEY ADJUSTMENT: Use a much smaller zoom factor for these events. ***
        // Instead of 0.15 (15%), try something like 0.01 to 0.03 (1% to 3%).
        // This means each of the many small scroll events contributes a tiny zoom.
        zs_for_this_event = 0.02; // TUNABLE: Start with 2%. Adjust as needed (e.g., 0.01, 0.015, 0.025, 0.03)

    } else if (hasAngleDelta) {
        // STANDARD MOUSE WHEEL or other devices primarily using angleDelta.
        // This also catches touchpads that might only provide angleDelta.
        YAWT_DEBUG(lcGuiVideoLoader) << "Mouse Wheel/Other (using angleDelta):" << event->angleDelta().y();
        int angleY = event->angleDelta().y();

        // angleDelta() is typically in 1/8ths of a degree.
        // A standard mouse wheel "notch" or "tick" is 15 degrees (15 * 8 = 120 delta units).
        // This calculation results in steps = +/-1 for each physical notch of the mouse wheel.
        steps = angleY / 120;

        // Use the original, larger zoom factor for these less frequent, distinct "notch" events.
        zs_for_this_event = 0.15; // 15% zoom per notch.

        // Special consideration if it's a touchpad using angleDelta:
        if (isTouchpad) {
            YAWT_DEBUG(lcGuiVideoLoader) << "Touchpad is using angleDelta. Steps:" << steps << "with zs:" << zs_for_this_event;
            // If this still feels too fast/slow for a touchpad using angleDelta,
            // you might need to adjust 'steps' or 'zs_for_this_event' specifically here.
            // For example, if touchpad angleDelta is also small and frequent:
            // steps = angleY / 60; // Make steps more sensitive
            // zs_for_this_event = 0.05; // Use a smaller zoom factor
        }

    } else if (hasPixelDelta) {
        // For non-touchpad devices that might provide pixelDelta (e.g., some high-resolution mice)
        // but were not caught by the (isTouchpad && hasPixelDelta) condition.
        YAWT_DEBUG(lcGuiVideoLoader) << "High-resolution Mouse/Other (using pixelDelta):" << event->pixelDelta().y();
        int pixelY = event->pixelDelta().y();

        // These pixel values might be larger than the touchpad's +/-1.
        // Normalize them to small step counts (e.g. by dividing).
        int divisor = 20; // TUNABLE: How many pixels for one logical "step".
        steps = pixelY / divisor;
        if (pixelY != 0 && steps == 0) { // Ensure at least one step for small movements
            steps = (pixelY > 0) ? 1 : -1;
        }
        // Use the standard zoom factor if steps are now normalized to +/-1, +/-2, etc.
        zs_for_this_event = 0.15;

    } else {
        // No recognized scroll delta information in the event.
        event->ignore();
        return;
    }

    if (steps == 0) {
        // If, after processing, there are no effective steps to take, ignore the event.
        event->ignore();
        return;
    }

    double mult = qPow(1.0 + zs_for_this_event, steps);
    setZoomFactorAtPoint(m_zoomFactor * mult, zoomAtPos);
    event->accept();
}


void VideoLoader::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    if (isVideoLoaded()) {
        clampPanOffset();
        update();
    }
}

// --- Private Helper Methods ---
void VideoLoader::updateCursorShape() {
    if (!isVideoLoaded()) {
        setCursor(Qt::ArrowCursor);
        return;
    }
    // If panning is active in any interaction mode, show the hand cursor
    if (m_isPanning) {
        setCursor(Qt::ClosedHandCursor);
        return;
    }
    switch (m_currentInteractionMode) {
    case InteractionMode::PanZoom:
        setCursor(Qt::OpenHandCursor);
        break;
    case InteractionMode::DrawROI:
    case InteractionMode::Point:
    case InteractionMode::Crop:
        setCursor(Qt::CrossCursor);
        break;
    case InteractionMode::EditBlobs:
        setCursor(Qt::PointingHandCursor);
        break;
    case InteractionMode::EditTracks:
        setCursor(Qt::ArrowCursor);
        break;
    default:
        setCursor(Qt::ArrowCursor);
        break;
    }
}

QRectF VideoLoader::calculateTargetRect() const {
    if (originalFrameSize.isEmpty() || m_zoomFactor <= 0) return QRectF();
    QSizeF ws = size(), imageSz = originalFrameSize, scaledSize = imageSz;
    scaledSize.scale(ws, Qt::KeepAspectRatio);
    QSizeF zoomedSize = scaledSize * m_zoomFactor;
    QPointF ctl((ws.width() - zoomedSize.width()) / 2.0,
                (ws.height() - zoomedSize.height()) / 2.0);
    return QRectF(ctl + m_panOffset, zoomedSize);
}
QPointF VideoLoader::mapPointToVideo(
    const QPointF& wp) const {
    if (currentQImageFrame.isNull() || originalFrameSize.isEmpty() ||
        m_zoomFactor <= 0)
        return QPointF(-1, -1);
    QRectF tr = calculateTargetRect();
    if (!tr.isValid() || tr.width() <= 0 || tr.height() <= 0 || !tr.contains(wp))
        return QPointF(-1, -1);
    double nx = (wp.x() - tr.left()) / tr.width(),
        ny = (wp.y() - tr.top()) / tr.height();
    return QPointF(qBound(0.0, nx * originalFrameSize.width(),
                          (qreal)originalFrameSize.width()),
                   qBound(0.0, ny * originalFrameSize.height(),
                          (qreal)originalFrameSize.height()));
}
QPointF VideoLoader::mapPointFromVideo(
    const QPointF& vp) const {
    if (currentQImageFrame.isNull() || originalFrameSize.isEmpty() ||
        m_zoomFactor <= 0 || originalFrameSize.width() <= 0 ||
        originalFrameSize.height() <= 0)
        return QPointF(-1, -1);
    QRectF tr = calculateTargetRect();
    if (!tr.isValid() || tr.width() <= 0 || tr.height() <= 0)
        return QPointF(-1, -1);
    QPointF cvp(qBound(0.0, vp.x(), (qreal)originalFrameSize.width()),
                qBound(0.0, vp.y(), (qreal)originalFrameSize.height()));
    double nx = cvp.x() / originalFrameSize.width(),
        ny = cvp.y() / originalFrameSize.height();
    return QPointF(tr.left() + nx * tr.width(), tr.top() + ny * tr.height());
}
void VideoLoader::clampPanOffset() {
    if (!isVideoLoaded() || m_zoomFactor <= 0) return;
    QRectF targetRect = calculateTargetRect();
    QSizeF widgetSz = size();
    QPointF currentTopLeftNoPan = targetRect.topLeft() - m_panOffset;
    if (targetRect.width() <= widgetSz.width())
        m_panOffset.setX((widgetSz.width() - targetRect.width()) / 2.0 -
                         currentTopLeftNoPan.x());
    else {
        qreal minPanX =
            widgetSz.width() - targetRect.width() - currentTopLeftNoPan.x() - 50;
        qreal maxPanX = -currentTopLeftNoPan.x() + 50;
        m_panOffset.setX(qBound(minPanX, m_panOffset.x(), maxPanX));
    }
    if (targetRect.height() <= widgetSz.height())
        m_panOffset.setY((widgetSz.height() - targetRect.height()) / 2.0 -
                         currentTopLeftNoPan.y());
    else {
        qreal minPanY =
            widgetSz.height() - targetRect.height() - currentTopLeftNoPan.y() - 50;
        qreal maxPanY = -currentTopLeftNoPan.y() + 50;
        m_panOffset.setY(qBound(minPanY, m_panOffset.y(), maxPanY));
    }
}

QRectF VideoLoader::constrainCropCircleToFrame(const QRectF& cropCircleRect) const {
    if (cropCircleRect.isNull() || !cropCircleRect.isValid()) {
        return QRectF();
    }

    const qreal frameWidth = static_cast<qreal>(originalFrameSize.width());
    const qreal frameHeight = static_cast<qreal>(originalFrameSize.height());
    if (frameWidth <= 0.0 || frameHeight <= 0.0) {
        return QRectF();
    }

    qreal side = qMin(cropCircleRect.width(), cropCircleRect.height());
    side = qMin(side, qMin(frameWidth, frameHeight));
    if (side <= 0.0) {
        return QRectF();
    }

    const qreal x = qBound(0.0, cropCircleRect.x(), frameWidth - side);
    const qreal y = qBound(0.0, cropCircleRect.y(), frameHeight - side);
    return QRectF(x, y, side, side);
}

QRectF VideoLoader::cropCircleFromCenterAndRadius(const QPointF& center, qreal radius) const {
    const qreal frameWidth = static_cast<qreal>(originalFrameSize.width());
    const qreal frameHeight = static_cast<qreal>(originalFrameSize.height());
    if (frameWidth <= 0.0 || frameHeight <= 0.0) {
        return QRectF();
    }

    const qreal maxRadius = qMin(qMin(center.x(), frameWidth - center.x()),
                                 qMin(center.y(), frameHeight - center.y()));
    radius = qBound(2.5, radius, maxRadius);
    return QRectF(center.x() - radius, center.y() - radius,
                  radius * 2.0, radius * 2.0);
}

void VideoLoader::handleRoiDefinedForCrop(
    const QRectF& cropRoiVideoCoords) {
    if (cropRoiVideoCoords.isNull() || !cropRoiVideoCoords.isValid()) {
        setInteractionMode(InteractionMode::PanZoom);
        return;
    }
    if (QMessageBox::question(this, "Confirm Crop", "Crop video to circular ROI?",
                              QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        m_isDefiningRoi = false;
        update();
        return;
    }

    emit videoProcessingStarted("Cropping video...");
    QString croppedFilePath;
    if (performVideoCrop(cropRoiVideoCoords, croppedFilePath) &&
        !croppedFilePath.isEmpty()) {
        emit videoProcessingFinished("Video cropped.", true);
        m_activeCropCircleRect = QRectF();
        loadVideo(croppedFilePath);
    } else {
        emit videoProcessingFinished("Crop failed.", false);
        QMessageBox::critical(this, "Crop Error", "Failed to crop video.");
    }
    setInteractionMode(InteractionMode::PanZoom);
    m_isDefiningRoi = false;
    update();
}
bool VideoLoader::performVideoCrop(
    const QRectF& cropRectVideoCoords,
    QString& outCroppedFilePath) {
    if (currentFilePath.isEmpty() || !videoCapture.isOpened() ||
        cropRectVideoCoords.isNull() || !cropRectVideoCoords.isValid() ||
        cropRectVideoCoords.width() <= 0 || cropRectVideoCoords.height() <= 0)
        return false;
    cv::VideoCapture origVid;
    if (!origVid.open(currentFilePath.toStdString())) return false;
    double origFps = origVid.get(cv::CAP_PROP_FPS);
    if (origFps <= 0) origFps = framesPerSecond > 0 ? framesPerSecond : 25.0;
    int frameWidth = originalFrameSize.width();
    int frameHeight = originalFrameSize.height();
    if (frameWidth <= 0) frameWidth = static_cast<int>(origVid.get(cv::CAP_PROP_FRAME_WIDTH));
    if (frameHeight <= 0) frameHeight = static_cast<int>(origVid.get(cv::CAP_PROP_FRAME_HEIGHT));
    if (frameWidth <= 0 || frameHeight <= 0) {
        origVid.release();
        return false;
    }

    int side = static_cast<int>(qRound(qMin(cropRectVideoCoords.width(), cropRectVideoCoords.height())));
    side = qMin(side, qMin(frameWidth, frameHeight));
    if (side > 2 && (side % 2) != 0) {
        --side;
    }

    cv::Rect cvCR(static_cast<int>(qRound(cropRectVideoCoords.x())),
                  static_cast<int>(qRound(cropRectVideoCoords.y())),
                  side,
                  side);
    cvCR.x = qBound(0, cvCR.x, frameWidth - cvCR.width);
    cvCR.y = qBound(0, cvCR.y, frameHeight - cvCR.height);
    if (cvCR.width <= 0 || cvCR.height <= 0) {
        origVid.release();
        return false;
    }
    cv::Size cfs(cvCR.width, cvCR.height);
    QFileInfo ofi(currentFilePath);
    QString bn = ofi.completeBaseName(),
        suff = ofi.suffix().isEmpty()
                   ? QString(DEFAULT_CROP_EXTENSION).remove(0, 1)
                   : ofi.suffix();
    outCroppedFilePath = ofi.absolutePath() + "/" + bn + "_cropped." + suff;
    cv::VideoWriter writer;
    int fourcc = DEFAULT_CROP_FOURCC;
    if (!writer.open(outCroppedFilePath.toStdString(), fourcc, origFps, cfs,
                     true)) {
        origVid.release();
        return false;
    }
    cv::Mat frame;
    int fCount = 0;
    cv::Mat circleMask(cvCR.height, cvCR.width, CV_8UC1, cv::Scalar(0));
    cv::circle(circleMask,
               cv::Point(cvCR.width / 2, cvCR.height / 2),
               qMin(cvCR.width, cvCR.height) / 2,
               cv::Scalar(255),
               cv::FILLED);
    const cv::Scalar outsideFill = m_assumeLightBackground ? cv::Scalar::all(255) : cv::Scalar::all(0);
    while (origVid.read(frame)) {
        if (frame.empty()) continue;
        try {
            cv::Mat croppedF = frame(cvCR);
            cv::Mat circularCrop(croppedF.size(), croppedF.type(), outsideFill);
            croppedF.copyTo(circularCrop, circleMask);
            writer.write(circularCrop);
            fCount++;
        } catch (const cv::Exception&) {
            break;
        }
    }
    origVid.release();
    writer.release();
    return fCount > 0;
}

void VideoLoader::applyThresholding() {
    if (currentCvFrame.empty()) {
        m_thresholdedFrame_mono = cv::Mat();
        return;
    }
    cv::Mat grayFrame;
    if (currentCvFrame.channels() >= 3)
        cv::cvtColor(currentCvFrame, grayFrame, cv::COLOR_BGR2GRAY);
    else
        grayFrame = currentCvFrame.clone();

    // Ensure 8-bit grayscale for thresholding; some videos can load as 16-bit.
    if (grayFrame.type() != CV_8UC1) {
        cv::Mat gray8;
        double minVal = 0.0;
        double maxVal = 0.0;
        cv::minMaxLoc(grayFrame, &minVal, &maxVal);
        if (maxVal > minVal) {
            grayFrame.convertTo(
                gray8,
                CV_8U,
                255.0 / (maxVal - minVal),
                -minVal * 255.0 / (maxVal - minVal));
        } else {
            gray8 = cv::Mat::zeros(grayFrame.size(), CV_8U);
        }
        grayFrame = gray8;
    }
    if (m_enableBlur && m_blurKernelSize >= 3) {
        try {
            cv::GaussianBlur(grayFrame, grayFrame,
                             cv::Size(m_blurKernelSize, m_blurKernelSize),
                             m_blurSigmaX);
        } catch (const cv::Exception& ex) {
            YAWT_WARN(lcGuiVideoLoader) << "GaussianBlur Exception:" << ex.what();
        }
    }
    int type =
        m_assumeLightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    try {
        switch (m_thresholdAlgorithm) {
        case Thresholding::ThresholdAlgorithm::Global:
            cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255,
                          type);
            break;
        case Thresholding::ThresholdAlgorithm::Otsu:
            cv::threshold(grayFrame, m_thresholdedFrame_mono, 0, 255,
                          type | cv::THRESH_OTSU);
            break;
        case Thresholding::ThresholdAlgorithm::AdaptiveMean:
            if (m_adaptiveBlockSize >= 3)
                cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255,
                                      cv::ADAPTIVE_THRESH_MEAN_C, type,
                                      m_adaptiveBlockSize, m_adaptiveC);
            else
                m_thresholdedFrame_mono = cv::Mat();
            break;
        case Thresholding::ThresholdAlgorithm::AdaptiveGaussian:
            if (m_adaptiveBlockSize >= 3)
                cv::adaptiveThreshold(grayFrame, m_thresholdedFrame_mono, 255,
                                      cv::ADAPTIVE_THRESH_GAUSSIAN_C, type,
                                      m_adaptiveBlockSize, m_adaptiveC);
            else
                m_thresholdedFrame_mono = cv::Mat();
            break;
        default:
            cv::threshold(grayFrame, m_thresholdedFrame_mono, m_thresholdValue, 255,
                          type);
            break;
        }
    } catch (const cv::Exception& ex) {
        YAWT_WARN(lcGuiVideoLoader) << "Thresholding Exception:" << ex.what();
        m_thresholdedFrame_mono = cv::Mat();
    }
}
void VideoLoader::updateTimerInterval() {
    if (framesPerSecond > 0 && m_playbackSpeedMultiplier > 0)
        playbackTimer->setInterval(qMax(
            1, qRound(1000.0 / (framesPerSecond * m_playbackSpeedMultiplier))));
    else
        playbackTimer->setInterval(40);
}
void VideoLoader::emitThresholdParametersChanged() {
    emit thresholdParametersChanged(getCurrentThresholdSettings());
}

QColor VideoLoader::getTrackColor(int trackId) const {
    // First check if the item exists in storage
    if (m_storage) {
        const TableItems::ClickedItem* item = m_storage->getItem(trackId);
        if (item && item->color.isValid()) {
            return item->color;
        }
    }

    // Fall back to cached colors
    if (m_trackColors.contains(trackId)) {
        return m_trackColors.value(trackId);
    }

    // Generate a random color if not found
    quint32 seed = static_cast<quint32>(trackId + 0xABCDEF);
    QRandomGenerator generator(seed);
    int hue = generator.bounded(360);
    int saturation = 180 + generator.bounded(76);
    int value = 180 + generator.bounded(76);
    QColor color = QColor::fromHsv(hue, saturation, value);
    m_trackColors.insert(trackId, color);
    return color;
}

void VideoLoader::startFrameLoader() {
    if (m_frameLoaderThread) {
        stopFrameLoader();
    }

    m_frameLoader = new FrameLoader();
    m_frameLoaderThread = new QThread(this);
    m_frameLoader->moveToThread(m_frameLoaderThread);

    // Provide the loader with our shared frame cache (if available) so the loader
    // can populate the cache from the worker thread without involving the UI thread.
    if (m_frameCache) {
        m_frameLoader->setFrameCache(m_frameCache);
    }

    // Connect signals - force queued delivery for cross-thread signals to ensure
    // slots run in the receiver (GUI) thread and arguments are copied through Qt's meta system.
    connect(m_frameLoaderThread, &QThread::started, m_frameLoader, &FrameLoader::processRequests);
    connect(m_frameLoader, &FrameLoader::frameLoaded, this, &VideoLoader::onFrameLoaded, Qt::QueuedConnection);
    connect(m_frameLoader, &FrameLoader::frameLoadError, this, &VideoLoader::onFrameLoadError, Qt::QueuedConnection);
    connect(m_frameLoaderThread, &QThread::finished, m_frameLoader, &QObject::deleteLater);

    m_frameLoaderThread->start();
    YAWT_INFO(lcGuiVideoLoader) << "VideoLoader: Frame loader thread started";
}

void VideoLoader::stopFrameLoader() {
    if (m_frameLoader) {
        m_frameLoader->stop();
    }

    if (m_frameLoaderThread && m_frameLoaderThread->isRunning()) {
        m_frameLoaderThread->quit();
        if (!m_frameLoaderThread->wait(3000)) {
            YAWT_WARN(lcGuiVideoLoader) << "VideoLoader: Frame loader thread failed to stop gracefully, terminating";
            m_frameLoaderThread->terminate();
            m_frameLoaderThread->wait(1000);
        }
    }

    m_frameLoader = nullptr;
    m_frameLoaderThread = nullptr;
    YAWT_INFO(lcGuiVideoLoader) << "VideoLoader: Frame loader thread stopped";
}

bool VideoLoader::getCachedFrame(int frameNumber, cv::Mat& outFrame) {
    if (!m_frameCache) {
        return false;
    }
    return m_frameCache->getFrame(frameNumber, outFrame);
}

QImage VideoLoader::getQImageForFrame(int frameNumber) const {
    // Return a QImage for an arbitrary frame number if available in the cache.
    // This does not attempt to synchronously load from disk; it only consults the in-memory cache.
    if (frameNumber < 0 || (totalFramesCount > 0 && frameNumber >= totalFramesCount)) {
        return QImage();
    }

    cv::Mat mat;
    if (m_frameCache && m_frameCache->getFrame(frameNumber, mat)) {
        QImage qimg;
        // convertCvMatToQImage is non-const; safe to call via const_cast here.
        const_cast<VideoLoader*>(this)->convertCvMatToQImage(mat, qimg);
        return qimg;
    }

    // Not present in cache -> return null QImage
    return QImage();
}

QImage VideoLoader::getOrLoadQImageForFrame(int frameNumber) {
    // Return a QImage for an arbitrary frame number, loading from disk if needed.
    if (frameNumber < 0 || (totalFramesCount > 0 && frameNumber >= totalFramesCount)) {
        return QImage();
    }

    cv::Mat mat;
    if (m_frameCache && m_frameCache->getFrame(frameNumber, mat)) {
        QImage qimg;
        convertCvMatToQImage(mat, qimg);
        return qimg;
    }

    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) {
        return QImage();
    }

    int restoreFrame = currentFrameIdx;
    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    bool seekOk = true;
    if (currentPos != frameNumber) {
        seekOk = videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber));
    }
    if (!seekOk) {
        return QImage();
    }

    cv::Mat loaded;
    if (!videoCapture.read(loaded) || loaded.empty()) {
        // Attempt to restore position even on failure
        videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(restoreFrame));
        return QImage();
    }

    if (m_frameCache) {
        m_frameCache->insertFrame(frameNumber, loaded);
    }

    // Restore capture position to the current frame (best effort)
    videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(restoreFrame));

    QImage qimg;
    convertCvMatToQImage(loaded, qimg);
    return qimg;
}

void VideoLoader::cacheWindowAroundFrame(int centerFrame, int radius) {
    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) {
        return;
    }
    if (centerFrame < 0 || (totalFramesCount > 0 && centerFrame >= totalFramesCount)) {
        return;
    }
    if (radius < 0) {
        return;
    }

    int startFrame = qMax(0, centerFrame - radius);
    int endFrame = (totalFramesCount > 0) ? qMin(totalFramesCount - 1, centerFrame + radius)
                                          : (centerFrame + radius);

    if (m_frameCache) {
        bool allPresent = true;
        for (int f = startFrame; f <= endFrame; ++f) {
            if (!m_frameCache->hasFrame(f)) {
                allPresent = false;
                break;
            }
        }
        if (allPresent) {
            return;
        }
    }

    int restoreFrame = currentFrameIdx;
    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    if (currentPos != startFrame) {
        if (!videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(startFrame))) {
            return;
        }
    }

    for (int f = startFrame; f <= endFrame; ++f) {
        cv::Mat frame;
        if (!videoCapture.read(frame) || frame.empty()) {
            break;
        }
        if (m_frameCache) {
            m_frameCache->insertFrame(f, frame);
        }
    }

    m_cacheStreamNextFrame = endFrame + 1;
    if (m_isPlaying) {
        if (restoreFrame >= 0 && (totalFramesCount <= 0 || restoreFrame < totalFramesCount)) {
            videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(restoreFrame));
        }
    }
}

bool VideoLoader::prefetchNextSequentialFrame() {
    if (!videoCapture.isOpened() || originalFrameSize.isEmpty()) {
        return false;
    }
    if (m_cacheStreamNextFrame < 0) {
        return false;
    }
    if (totalFramesCount > 0 && m_cacheStreamNextFrame >= totalFramesCount) {
        return false;
    }

    int currentPos = static_cast<int>(videoCapture.get(cv::CAP_PROP_POS_FRAMES));
    if (currentPos != m_cacheStreamNextFrame) {
        return false;
    }

    cv::Mat frame;
    if (!videoCapture.read(frame) || frame.empty()) {
        return false;
    }
    if (m_frameCache) {
        m_frameCache->insertFrame(m_cacheStreamNextFrame, frame);
    }
    ++m_cacheStreamNextFrame;
    return true;
}

QMap<int, QImage> VideoLoader::getQImagesForFrames(const QList<int>& frameNumbers) const {
    QMap<int, QImage> result;
    for (int fn : frameNumbers) {
        result.insert(fn, getQImageForFrame(fn));
    }
    return result;
}

void VideoLoader::preloadAdjacentFrames(int centerFrame, int radius) {
    if (!m_frameLoader || !m_frameCache || centerFrame < 0) {
        return;
    }

    // Avoid redundant preloading
    if (centerFrame == m_lastPreloadCenter) {
        return;
    }

    m_lastPreloadCenter = centerFrame;

    // Create list of frames to preload
    QList<int> framesToLoad;

    // Add frames in priority order: closest to center first
    for (int distance = 1; distance <= radius; distance++) {
        // Add frame before center
        int prevFrame = centerFrame - distance;
        if (prevFrame >= 0 && prevFrame < totalFramesCount && !m_frameCache->hasFrame(prevFrame)) {
            framesToLoad.append(prevFrame);
        }

        // Add frame after center
        int nextFrame = centerFrame + distance;
        if (nextFrame >= 0 && nextFrame < totalFramesCount && !m_frameCache->hasFrame(nextFrame)) {
            framesToLoad.append(nextFrame);
        }
    }

    if (!framesToLoad.isEmpty()) {
        // Higher priority for closer frames
        int priority = 10 - qMin(radius, 9); // Priority 1-10
        m_frameLoader->requestFrames(framesToLoad, priority);
        YAWT_DEBUG(lcGuiVideoLoader) << "VideoLoader: Requested preload of" << framesToLoad.size() << "frames around" << centerFrame;
    }
}

void VideoLoader::onFrameLoaded(int frameNumber, cv::Mat frame) {
    // Frame has already been inserted into the cache by the worker thread (if a cache was provided).
    // Avoid redundant cache insertion and expensive cloning on the main thread.
    // If this was a pending seek, update the display using a shallow assignment.
    if (m_pendingSeekFrame == frameNumber) {
        m_pendingSeekFrame = -1;

        // Shallow-assign the frame (cheap). The cache owns the pixel buffer.
        currentCvFrame = frame;
        currentFrameIdx = frameNumber;
        applyThresholding();

        if (m_activeViewModes.testFlag(ViewModeOption::Threshold) && !m_thresholdedFrame_mono.empty()) {
            convertCvMatToQImage(m_thresholdedFrame_mono, currentQImageFrame);
        } else if (!currentCvFrame.empty()) {
            convertCvMatToQImage(currentCvFrame, currentQImageFrame);
        }

        emit frameChanged(currentFrameIdx, currentQImageFrame);
        update();
    }
}

void VideoLoader::onFrameLoadError(int frameNumber, QString error) {
    YAWT_WARN(lcGuiVideoLoader) << "VideoLoader: Failed to load frame" << frameNumber << ":" << error;

    // If this was the pending seek frame, clear it
    if (m_pendingSeekFrame == frameNumber) {
        m_pendingSeekFrame = -1;
    }
}

void VideoLoader::setCacheSize(int maxFrames) {
    if (m_frameCache) {
        m_frameCache->setMaxSize(maxFrames);
        qDebug() << "VideoLoader: Cache size set to" << maxFrames;
    }
}

int VideoLoader::getCacheSize() const {
    return m_frameCache ? m_frameCache->size() : 0;
}

double VideoLoader::getCacheHitRate() const {
    return m_frameCache ? m_frameCache->hitRate() : 0.0;
}

QString VideoLoader::createDataDirectory(const QString& videoFilePath) {
    QFileInfo videoInfo(videoFilePath);
    QString videoDirectory = videoInfo.absolutePath();
    QString yawtDirPath = QDir(videoDirectory).absoluteFilePath("yawt");

    // Try to create the directory in the same folder as the video
    QDir yawtDir(yawtDirPath);
    if (!yawtDir.exists()) {
        if (QDir().mkpath(yawtDirPath)) {
            qDebug() << "Created yawt data directory:" << yawtDirPath;
            return yawtDirPath;
        } else {
            qWarning() << "Failed to create yawt directory in video folder:" << yawtDirPath;
            qWarning() << "Falling back to user's home directory";

            // Fallback to user's home directory
            QString homeDirectory = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
            QString fallbackYawtDir = QDir(homeDirectory).absoluteFilePath("yawt");

            QDir fallbackDir(fallbackYawtDir);
            if (!fallbackDir.exists()) {
                if (QDir().mkpath(fallbackYawtDir)) {
                    qDebug() << "Created yawt data directory in home:" << fallbackYawtDir;
                    return fallbackYawtDir;
                } else {
                    qWarning() << "Failed to create yawt directory in home folder:" << fallbackYawtDir;
                    return QString(); // Return empty string if all attempts fail
                }
            } else {
                qDebug() << "Using existing yawt data directory in home:" << fallbackYawtDir;
                return fallbackYawtDir;
            }
        }
    } else {
        qDebug() << "Using existing yawt data directory:" << yawtDirPath;
        return yawtDirPath;
    }
}
