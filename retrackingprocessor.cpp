#include "retrackingprocessor.h"
#include "trackingdatastorage.h"
#include "thresholdingutils.h"
#include "wormtracker.h"
#include <QDebug>
#include <QMutexLocker>
#include <QThread>
#include <QApplication>
#include <cmath>

RetrackingProcessor::RetrackingProcessor(QObject* parent)
    : QObject(parent)
    , m_fixBlobId(-1)
    , m_startFrame(0)
    , m_endFrame(0)
    , m_replaceExisting(false)
    , m_extendTrack(true)
    , m_isRunning(false)
    , m_stopRequested(false)
    , m_dataStorage(nullptr)
    , m_wormTracker(nullptr)
    , m_trackerThread(nullptr)
{
    qDebug() << "RetrackingProcessor constructor called";
}

RetrackingProcessor::~RetrackingProcessor()
{
    qDebug() << "RetrackingProcessor destructor called";
    requestStop();
    
    // Clean up WormTracker and thread
    if (m_wormTracker) {
        m_wormTracker->stopTracking();
        m_wormTracker->deleteLater();
        m_wormTracker = nullptr;
    }
    
    if (m_trackerThread && m_trackerThread->isRunning()) {
        m_trackerThread->quit();
        m_trackerThread->wait(3000);
        m_trackerThread->deleteLater();
        m_trackerThread = nullptr;
    }
    
    if (m_videoCapture.isOpened()) {
        m_videoCapture.release();
    }
}

void RetrackingProcessor::startRetracking(const QString& originalVideoPath,
                                         const Thresholding::ThresholdSettings& thresholdSettings,
                                         int fixBlobId,
                                         const QRectF& initialROI,
                                         int startFrame,
                                         int endFrame,
                                         bool replaceExisting,
                                         bool extendTrack,
                                         TrackingDataStorage* dataStorage)
{
    QMutexLocker locker(&m_mutex);
    
    qDebug() << "RetrackingProcessor::startRetracking called";
    qDebug() << "  fixBlobId:" << fixBlobId;
    qDebug() << "  startFrame:" << startFrame << "endFrame:" << endFrame;
    qDebug() << "  videoPath:" << originalVideoPath;
    
    if (m_isRunning) {
        qWarning() << "RetrackingProcessor: Already running, cannot start new retracking";
        return;
    }
    
    // Store parameters
    m_videoPath = originalVideoPath;
    m_thresholdSettings = thresholdSettings;
    m_fixBlobId = fixBlobId;
    m_initialROI = initialROI;
    m_expandedROI = expandROI(initialROI, ROI_EXPANSION_FACTOR);
    m_startFrame = startFrame;
    m_endFrame = endFrame;
    m_replaceExisting = replaceExisting;
    m_extendTrack = extendTrack;
    m_dataStorage = dataStorage;
    
    // Initialize tracking state
    m_thresholdedFrames.clear();
    m_trackingResults.clear();
    m_stopRequested = false;
    
    qDebug() << "RetrackingProcessor: Starting retracking for Fix blob" << fixBlobId
             << "in range" << startFrame << "to" << endFrame
             << "with ROI" << initialROI
             << "expanded to" << m_expandedROI;
    
    // Start processing in a separate thread context
    QMetaObject::invokeMethod(this, "processRetracking", Qt::QueuedConnection);
}

void RetrackingProcessor::requestStop()
{
    QMutexLocker locker(&m_mutex);
    m_stopRequested = true;
}

bool RetrackingProcessor::isRunning() const
{
    QMutexLocker locker(&m_mutex);
    return m_isRunning;
}

void RetrackingProcessor::processRetracking()
{
    qDebug() << "RetrackingProcessor::processRetracking started";
    
    {
        QMutexLocker locker(&m_mutex);
        m_isRunning = true;
    }
    
    emit statusUpdate(QString("Starting retracking for Fix Blob ID %1").arg(m_fixBlobId));
    
    RetrackingResult result;
    result.fixBlobId = m_fixBlobId;
    
    do {
        // Load the original video
        if (!loadOriginalVideo(m_videoPath)) {
            result.errorMessage = "Failed to load original video";
            break;
        }
        
        emit statusUpdate("Loading and thresholding frame range...");
        
        // Load and threshold the frame range
        if (!loadAndThresholdFrameRange()) {
            result.errorMessage = "Failed to load and threshold frames";
            break;
        }
        
        emit statusUpdate("Setting up WormTracker for retracking...");
        
        // Use WormTracker to do the actual tracking
        if (!processFrameRangeWithWormTracker()) {
            result.errorMessage = "Failed to track with WormTracker";
            break;
        }
        
        // Check if we have enough successful tracking results
        int minSuccessfulFrames = (m_endFrame - m_startFrame + 1) * MIN_SUCCESSFUL_FRAMES_RATIO / 100;
        if (m_trackingResults.size() < minSuccessfulFrames) {
            result.errorMessage = QString("Insufficient tracking success rate: %1/%2 frames")
                                  .arg(m_trackingResults.size())
                                  .arg(m_endFrame - m_startFrame + 1);
            break;
        }
        
        result.processedFrames = m_endFrame - m_startFrame + 1;
        result.successfulFrames = m_trackingResults.size();
        result.completed = true;
        
        emit statusUpdate("Integrating retracking results...");
        
        // Integrate results into the main tracking data
        integrateResults();
        
    } while(false);
    
    // Clean up
    if (m_videoCapture.isOpened()) {
        m_videoCapture.release();
    }
    
    {
        QMutexLocker locker(&m_mutex);
        m_isRunning = false;
    }
    
    if (result.completed) {
        emit statusUpdate(QString("Retracking completed: %1/%2 frames successfully tracked")
                          .arg(result.successfulFrames)
                          .arg(result.processedFrames));
        emit retrackingCompleted(result);
    } else {
        emit statusUpdate(QString("Retracking failed: %1").arg(result.errorMessage));
        emit retrackingFailed(m_fixBlobId, result.errorMessage);
    }
}

bool RetrackingProcessor::loadOriginalVideo(const QString& videoPath)
{
    if (videoPath.isEmpty()) {
        qWarning() << "RetrackingProcessor: Empty video path";
        return false;
    }
    
    qDebug() << "RetrackingProcessor::loadOriginalVideo attempting to load:" << videoPath;
    
    if (m_videoCapture.isOpened()) {
        m_videoCapture.release();
    }
    
    if (!m_videoCapture.open(videoPath.toStdString())) {
        qWarning() << "RetrackingProcessor: Failed to open original video:" << videoPath;
        return false;
    }
    
    // Verify video properties
    int totalFrames = static_cast<int>(m_videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
    if (totalFrames <= 0) {
        qWarning() << "RetrackingProcessor: Invalid frame count in video:" << totalFrames;
        return false;
    }
    
    if (m_endFrame >= totalFrames) {
        qWarning() << "RetrackingProcessor: End frame" << m_endFrame 
                   << "exceeds video length" << totalFrames;
        return false;
    }
    
    qDebug() << "RetrackingProcessor: Successfully loaded original video with" 
             << totalFrames << "frames";
    
    return true;
}

bool RetrackingProcessor::loadAndThresholdFrameRange()
{
    int totalFrames = m_endFrame - m_startFrame + 1;
    m_thresholdedFrames.clear();
    m_thresholdedFrames.reserve(totalFrames);
    
    for (int frameNum = m_startFrame; frameNum <= m_endFrame; ++frameNum) {
        // Check for stop request
        {
            QMutexLocker locker(&m_mutex);
            if (m_stopRequested) {
                qDebug() << "RetrackingProcessor: Stopping due to user request";
                return false;
            }
        }
        
        // Load the frame
        if (!m_videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNum))) {
            qWarning() << "RetrackingProcessor: Failed to seek to frame" << frameNum;
            return false;
        }
        
        cv::Mat rawFrame;
        if (!m_videoCapture.read(rawFrame) || rawFrame.empty()) {
            qWarning() << "RetrackingProcessor: Failed to read frame" << frameNum;
            return false;
        }
        
        // Apply thresholding to the raw frame
        cv::Mat thresholdedFrame;
        ThresholdingUtils::applyThresholding(rawFrame, thresholdedFrame, m_thresholdSettings);
        
        if (thresholdedFrame.empty()) {
            qWarning() << "RetrackingProcessor: Failed to threshold frame" << frameNum;
            return false;
        }
        
        m_thresholdedFrames.push_back(thresholdedFrame);
        
        // Emit progress update
        emit retrackingProgress(m_fixBlobId, frameNum - m_startFrame + 1, totalFrames);
        
        // Allow GUI updates
        if ((frameNum - m_startFrame) % 10 == 0) {
            QApplication::processEvents();
        }
    }
    
    qDebug() << "RetrackingProcessor: Successfully loaded and thresholded" << m_thresholdedFrames.size() << "frames";
    return true;
}

bool RetrackingProcessor::processFrameRangeWithWormTracker()
{
    setupWormTracker();
    
    if (!m_wormTracker) {
        qWarning() << "RetrackingProcessor: Failed to create WormTracker";
        return false;
    }
    
    // Set the thresholded frames for the tracker
    m_wormTracker->setFrames(&m_thresholdedFrames);
    
    // Connect signals to collect results
    connect(m_wormTracker, &WormTracker::positionUpdated,
            this, &RetrackingProcessor::collectTrackingResults);
    
    connect(m_wormTracker, &WormTracker::finished,
            this, [this]() {
                qDebug() << "RetrackingProcessor: WormTracker finished";
            });
    
    // Start tracking
    qDebug() << "RetrackingProcessor: Starting WormTracker";
    m_wormTracker->startTracking();
    
    // Wait for tracking to complete (simplified for now)
    // In a full implementation, this would be event-driven
    return true;
}

void RetrackingProcessor::setupWormTracker()
{
    // Create WormTracker instance
    m_wormTracker = new WormTracker(
        m_fixBlobId,                               // wormId
        m_initialROI,                             // initialRoi
        WormTracker::TrackingDirection::Forward,  // direction
        m_startFrame,                             // videoKeyFrameNum
        this                                      // parent
    );
    
    qDebug() << "RetrackingProcessor: Created WormTracker with ID" << m_fixBlobId
             << "ROI" << m_initialROI << "starting from frame" << m_startFrame;
}

void RetrackingProcessor::collectTrackingResults(int wormId,
                                               int originalFrameNumber,
                                               const Tracking::DetectedBlob& primaryBlob,
                                               const Tracking::DetectedBlob& fullBlob,
                                               QRectF searchRoiUsed,
                                               Tracking::TrackerState currentState,
                                               const QList<Tracking::DetectedBlob>& splitCandidates)
{
    Q_UNUSED(fullBlob)
    Q_UNUSED(searchRoiUsed) 
    Q_UNUSED(currentState)
    Q_UNUSED(splitCandidates)
    
    qDebug() << "RetrackingProcessor: Collecting result for worm" << wormId << "frame" << originalFrameNumber;
    
    // Create track point from the primary blob
    Tracking::WormTrackPoint trackPoint;
    trackPoint.frameNumberOriginal = m_startFrame + originalFrameNumber; // Adjust frame number
    trackPoint.position = cv::Point2f(primaryBlob.centroid.x(), primaryBlob.centroid.y());
    trackPoint.roi = primaryBlob.boundingBox;
    trackPoint.quality = Tracking::TrackPointQuality::Confident;
    
    m_trackingResults.push_back(trackPoint);
    
    qDebug() << "RetrackingProcessor: Added track point at" << trackPoint.position.x << "," << trackPoint.position.y
             << "for frame" << trackPoint.frameNumberOriginal;
}

void RetrackingProcessor::integrateResults()
{
    if (!m_dataStorage || m_trackingResults.empty()) {
        qWarning() << "RetrackingProcessor: No data storage or no results to integrate";
        return;
    }
    
    if (m_replaceExisting) {
        replaceExistingTrack();
    } else if (m_extendTrack) {
        extendExistingTrack();
    }
    
    qDebug() << "RetrackingProcessor: Integrated" << m_trackingResults.size() 
             << "retracked points for Fix blob ID" << m_fixBlobId;
}

void RetrackingProcessor::replaceExistingTrack()
{
    // Remove existing track points in the frame range for this blob ID
    for (int frame = m_startFrame; frame <= m_endFrame; ++frame) {
        m_dataStorage->removeBlobsInFrame(frame, m_fixBlobId);
    }
    
    // Set the new track data
    m_dataStorage->setTrackForItem(m_fixBlobId, m_trackingResults);
    
    // Remove the original Fix blob since we've replaced it with tracked data
    m_dataStorage->removeBlobsOfType(TableItems::ItemType::Fix);
}

void RetrackingProcessor::extendExistingTrack()
{
    // Get existing tracks and merge with new results
    auto existingTracks = m_dataStorage->getAllTracks();
    auto& track = existingTracks[m_fixBlobId];
    
    // Add new tracking results to existing track
    track.insert(track.end(), m_trackingResults.begin(), m_trackingResults.end());
    
    // Set the updated track
    m_dataStorage->setTrackForItem(m_fixBlobId, track);
    
    // Remove the Fix blob marker since tracking is complete
    auto fixItems = m_dataStorage->getBlobsByType(TableItems::ItemType::Fix);
    for (const auto& fixItem : fixItems) {
        if (fixItem.id == m_fixBlobId) {
            m_dataStorage->removeBlob(fixItem.frameOfSelection, fixItem.id);
            break;
        }
    }
}

QRectF RetrackingProcessor::expandROI(const QRectF& baseROI, double expansionFactor) const
{
    if (expansionFactor <= 1.0) {
        return baseROI;
    }
    
    double centerX = baseROI.center().x();
    double centerY = baseROI.center().y();
    double newWidth = baseROI.width() * expansionFactor;
    double newHeight = baseROI.height() * expansionFactor;
    
    QRectF expandedROI(centerX - newWidth / 2.0,
                       centerY - newHeight / 2.0,
                       newWidth,
                       newHeight);
    
    return expandedROI;
}

void RetrackingProcessor::startRetrackingSlot(const QString& originalVideoPath,
                                            const Thresholding::ThresholdSettings& thresholdSettings,
                                            int fixBlobId,
                                            const QRectF& initialROI,
                                            int startFrame,
                                            int endFrame,
                                            bool replaceExisting,
                                            bool extendTrack,
                                            TrackingDataStorage* dataStorage)
{
    // Simple wrapper that calls the main startRetracking method
    startRetracking(originalVideoPath, thresholdSettings, fixBlobId, initialROI,
                   startFrame, endFrame, replaceExisting, extendTrack, dataStorage);
}