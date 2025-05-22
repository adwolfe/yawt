// trackingmanager.cpp
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <algorithm>  // For std::min, std::sort, std::transform etc.
#include <numeric>    // For std::iota if used for assignments
#include <QLineF>
#include <set>        // For QSet operations if needed beyond QSet directly

#include "trackingmanager.h"
#include "wormtracker.h" // For WormTracker class and its enums

// Helper function to calculate squared Euclidean distance
double squaredDistance(const QPointF& p1, const QPointF& p2) {
    QPointF diff = p1 - p2;
    return QPointF::dotProduct(diff, diff);
}


TrackingManager::TrackingManager(QObject* parent)
    : QObject(parent),
    m_keyFrameNum(-1),
    m_totalFramesInVideo(0),
    m_isTrackingRunning(false),
    m_cancelRequested(false),
    m_videoProcessor(nullptr),
    m_videoProcessorThread(nullptr),
    m_videoFps(0.0),
    m_expectedTrackersToFinish(0),
    m_finishedTrackersCount(0),
    m_videoProcessingProgress(0) {
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<WormObject::TrackingState>("WormObject::TrackingState");
    qRegisterMetaType<AllWormTracks>("AllWormTracks");
    qRegisterMetaType<QList<TrackingHelper::DetectedBlob>>(
        "QList<TrackingHelper::DetectedBlob>");
    qRegisterMetaType<TrackingHelper::DetectedBlob>("TrackingHelper::DetectedBlob"); // For Q_ARG
}

TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this
             << ") DESTRUCTOR - START cleaning up...";
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting "
                    "cancel.";
        cancelTracking(); // This sets m_cancelRequested = true
        // Give some time for threads to respond to cancellation.
        // The main cleanup of threads happens in cleanupThreadsAndObjects.
        QThread::msleep(200);
    }
    cleanupThreadsAndObjects(); // This will handle stopping and deleting threads.
    qDebug() << "TrackingManager (" << this
             << ") DESTRUCTOR - FINISHED cleaning up.";
}

void TrackingManager::startFullTrackingProcess(
    const QString& videoPath, int keyFrameNum,
    const std::vector<InitialWormInfo>& initialWorms,
    const ThresholdSettings& settings, int totalFramesInVideo) {
    qDebug() << "TrackingManager (" << this
             << "): startFullTrackingProcess called.";
    if (m_isTrackingRunning) {
        qWarning() << "TrackingManager: Attempted to start tracking while already "
                      "running.";
        emit trackingFailed("Another tracking process is already running.");
        return;
    }
    cleanupThreadsAndObjects(); // Ensure clean state before starting
    qDebug() << "TrackingManager (" << this
             << "): Post-cleanup in start, m_videoProcessor is"
             << m_videoProcessor << "m_videoProcessorThread is"
             << m_videoProcessorThread;

    m_videoPath = videoPath;
    m_keyFrameNum = keyFrameNum;
    m_initialWormInfos = initialWorms;
    m_thresholdSettings = settings;
    m_totalFramesInVideo = totalFramesInVideo;
    m_isTrackingRunning = true;
    m_cancelRequested = false; // Reset cancel flag for new run
    m_videoProcessingProgress = 0;
    m_finishedTrackersCount = 0;
    m_expectedTrackersToFinish = 0; // Will be set in launchWormTrackers
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameInfos.clear();
    m_mergedGroups.clear();
    m_wormToMergeGroupMap.clear();

    emit trackingStatusUpdate("Starting video processing...");
    emit overallTrackingProgress(0);

    m_videoProcessor = new VideoProcessor();
    m_videoProcessorThread = new QThread(this);
    m_videoProcessor->moveToThread(m_videoProcessorThread);
    qDebug() << "TrackingManager: Created new VideoProcessor ("
             << m_videoProcessor << ") and QThread (" << m_videoProcessorThread
             << ")";

    connect(
        m_videoProcessorThread, &QThread::started, m_videoProcessor, [this]() {
            if (m_videoProcessor) {
                m_videoProcessor->startInitialProcessing(m_videoPath, m_keyFrameNum,
                                                         m_thresholdSettings,
                                                         m_totalFramesInVideo);
            }
        });
    connect(m_videoProcessor, &VideoProcessor::initialProcessingComplete, this,
            &TrackingManager::handleInitialProcessingComplete);
    connect(m_videoProcessor, &VideoProcessor::processingError, this,
            &TrackingManager::handleVideoProcessingError);
    connect(m_videoProcessor, &VideoProcessor::initialProcessingProgress, this,
            &TrackingManager::handleVideoProcessingProgress);
    connect(m_videoProcessor, &VideoProcessor::processingStarted, this, [this]() {
        emit trackingStatusUpdate("Video processing started in thread.");
    });
    connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor,
            &QObject::deleteLater, Qt::UniqueConnection);
    connect(m_videoProcessorThread, &QThread::finished, this, [this]() {
        qDebug() << "VideoProcessorThread (" << QObject::sender()
        << ") finished. m_videoProcessor (" << m_videoProcessor
        << ") was scheduled for deleteLater.";
        m_videoProcessor = nullptr;
    }, Qt::DirectConnection);

    m_videoProcessorThread->start();
}

void TrackingManager::cancelTracking() {
    qDebug() << "TrackingManager (" << this
             << "): cancelTracking called. IsRunning:" << m_isTrackingRunning
             << "CancelRequested:" << m_cancelRequested;

    if (m_cancelRequested) { // Already cancelling
        qDebug() << "TM: Cancellation already in progress.";
        return;
    }
    m_cancelRequested = true; // Set the flag first
    emit trackingStatusUpdate("Cancellation requested...");
    qDebug() << "TM: Cancellation flag set to true.";

    if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
        qDebug() << "TM: Requesting video processor thread ("
                 << m_videoProcessorThread << ") interruption.";
        m_videoProcessorThread->requestInterruption();
    }

    // Use a copy of the list for iteration if trackers might be removed during this process
    QList<WormTracker*> trackersToStop = m_wormTrackers;
    for (WormTracker* tracker : trackersToStop) {
        if (tracker) {
            QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
        }
    }
    // Threads will be requested to interrupt by their WormTracker's stopTracking, or when they finish.
    // If a tracker is already finished, its thread might be gone.
    // The checkForAllTrackersFinished will handle emitting trackingCancelled.
}

void TrackingManager::cleanupThreadsAndObjects() {
    qDebug() << "TrackingManager (" << this
             << "): cleanupThreadsAndObjects - START";
    if (m_videoProcessorThread) {
        qDebug() << "  Cleaning up video processor thread:"
                 << m_videoProcessorThread;
        if (m_videoProcessorThread->isRunning()) {
            m_videoProcessorThread->requestInterruption();
            m_videoProcessorThread->quit();
            if (!m_videoProcessorThread->wait(1000)) {
                qWarning() << "  VideoProcessor thread (" << m_videoProcessorThread
                           << ") did not finish gracefully. Forcing termination.";
                m_videoProcessorThread->terminate(); // Last resort
                m_videoProcessorThread->wait();      // Wait for termination
            } else {
                qDebug() << "  Video processor thread (" << m_videoProcessorThread
                         << ") finished gracefully.";
            }
        }
        // m_videoProcessor is deleted via deleteLater connected to thread's finished signal
        // We just delete the thread object itself.
        delete m_videoProcessorThread;
        m_videoProcessorThread = nullptr;
        m_videoProcessor = nullptr; // Ensure it's null after thread is gone
    }


    // Make a copy for safe iteration as trackers/threads might be removed
    QList<QThread*> threadsToClean = m_trackerThreads;
    for (QThread* thread : threadsToClean) {
        if (thread) {
            qDebug() << "  Cleaning up tracker thread:" << thread;
            if (thread->isRunning()) {
                thread->requestInterruption();
                thread->quit();
                if (!thread->wait(500)) {
                    qWarning() << "  A WormTracker thread (" << thread
                               << ") did not finish gracefully. Forcing termination.";
                    thread->terminate();
                    thread->wait();
                } else {
                    qDebug() << "  Tracker thread (" << thread
                             << ") finished gracefully.";
                }
            }
            delete thread;
        }
    }
    m_trackerThreads.clear();
    // WormTracker objects are set to deleteLater when their threads finish.
    m_wormTrackers.clear(); // Clear the list of pointers.

    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    m_processedForwardFrames.clear();
    std::vector<cv::Mat>().swap(m_processedForwardFrames);
    m_processedReversedFrames.clear();
    std::vector<cv::Mat>().swap(m_processedReversedFrames);

    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameInfos.clear();
    m_mergedGroups.clear();
    m_wormToMergeGroupMap.clear();

    m_isTrackingRunning = false;
    // m_cancelRequested = false; // Reset for next run, handled in startFullTrackingProcess
    qDebug() << "TrackingManager (" << this
             << "): cleanupThreadsAndObjects - FINISHED";
}

QList<WormTracker*> TrackingManager::findTrackersForWorm(int conceptualWormId) {
    QList<WormTracker*> foundTrackers;
    for (WormTracker* tracker : qAsConst(m_wormTrackers)) { // Iterate over current live trackers
        if (tracker && tracker->getWormId() == conceptualWormId) {
            foundTrackers.append(tracker);
        }
    }
    return foundTrackers;
}


void TrackingManager::handleInitialProcessingComplete(
    const std::vector<cv::Mat>& forwardFrames,
    const std::vector<cv::Mat>& reversedFrames, double fps,
    cv::Size frameSize) {
    qDebug() << "TrackingManager (" << this
             << "): handleInitialProcessingComplete received.";

    if (m_cancelRequested) {
        qDebug() << "TM: Video processing complete but cancellation requested. Not launching trackers.";
        cleanupThreadsAndObjects();
        emit trackingCancelled();
        m_isTrackingRunning = false;
        return;
    }
    if (!m_isTrackingRunning) { // Could have been stopped by an error during video processing
        qWarning() << "TM: handleInitialProcessingComplete, but tracking is not marked as running. Aborting tracker launch.";
        cleanupThreadsAndObjects();
        return;
    }


    qDebug() << "TM: Initial video processing complete. Fwd:"
             << forwardFrames.size() << "Rev:" << reversedFrames.size();
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    m_videoProcessingProgress = 100;
    m_processedForwardFrames = forwardFrames;
    m_processedReversedFrames = reversedFrames;
    m_videoFps = fps;
    m_videoFrameSize = frameSize;

    for (const auto& info : m_initialWormInfos) {
        if (m_wormObjectsMap.contains(info.id)) continue;
        m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
    }
    launchWormTrackers();
    updateOverallProgress();
}

void TrackingManager::handleVideoProcessingError(const QString& errorMessage) {
    qDebug() << "TrackingManager (" << this
             << "): handleVideoProcessingError: " << errorMessage;
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing error (" << errorMessage << ") received during/after cancellation. Letting cancel flow proceed.";
        // Don't emit trackingFailed if a cancel was already in progress, let cancel flow dominate.
        // cleanupThreadsAndObjects will be called by the cancel flow or final check.
        return;
    }
    if (!m_isTrackingRunning) {
        qWarning() << "TM: Video processing error (" << errorMessage << ") but tracking not marked as running.";
        return; // Avoid double error handling
    }

    qWarning() << "TM: Video processing error:" << errorMessage;
    emit trackingFailed("Video processing failed: " + errorMessage);
    m_isTrackingRunning = false;
    cleanupThreadsAndObjects();
}

void TrackingManager::handleVideoProcessingProgress(int percentage) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    m_videoProcessingProgress = percentage;
    updateOverallProgress();
}

void TrackingManager::launchWormTrackers() {
    if (m_initialWormInfos.empty()) {
        emit trackingStatusUpdate("No worms selected for tracking.");
        emit trackingFinishedSuccessfully("");
        m_isTrackingRunning = false;
        return;
    }

    m_expectedTrackersToFinish = static_cast<int>(m_initialWormInfos.size() * 2);
    m_finishedTrackersCount = 0;
    m_individualTrackerProgress.clear();
    emit trackingStatusUpdate(
        QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));

    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        // Forward Tracker
        if (!m_processedForwardFrames.empty() || m_keyFrameNum == m_totalFramesInVideo -1) { // Launch if frames exist or keyframe is last
            WormTracker* fwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
            fwdTracker->setFrames(&m_processedForwardFrames);
            QThread* fwdThread = new QThread(this);
            fwdTracker->moveToThread(fwdThread);
            fwdTracker->setProperty("wormId", wormId);
            fwdTracker->setProperty("direction", "Forward");

            connect(fwdThread, &QThread::started, fwdTracker, &WormTracker::startTracking, Qt::QueuedConnection);
            connect(fwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
            connect(fwdThread, &QThread::finished, fwdTracker, &QObject::deleteLater, Qt::UniqueConnection);
            connect(fwdThread, &QThread::finished, this, [this, fwdThread, wormId](){ // Capture wormId for debug
                m_trackerThreads.removeOne(fwdThread);
                qDebug() << "Forward Tracker Thread for worm" << wormId << "finished and removed from list.";
            }, Qt::DirectConnection);

            connect(fwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
            connect(fwdTracker, &WormTracker::splitDetectedAndPaused, this, &TrackingManager::handleWormSplitDetectedAndPaused);
            connect(fwdTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
            connect(fwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(fwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
            m_wormTrackers.append(fwdTracker);
            m_trackerThreads.append(fwdThread);
            m_individualTrackerProgress[fwdTracker] = 0;
            fwdThread->start();
        } else {
            m_expectedTrackersToFinish--; // Decrement if not launched
            qDebug() << "TM: Skipping Forward Tracker for worm" << wormId << "due to no forward frames.";
        }


        // Backward Tracker
        if (!m_processedReversedFrames.empty() || m_keyFrameNum == 0) { // Launch if frames exist or keyframe is first
            WormTracker* bwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
            bwdTracker->setFrames(&m_processedReversedFrames);
            QThread* bwdThread = new QThread(this);
            bwdTracker->moveToThread(bwdThread);
            bwdTracker->setProperty("wormId", wormId);
            bwdTracker->setProperty("direction", "Backward");

            connect(bwdThread, &QThread::started, bwdTracker, &WormTracker::startTracking, Qt::QueuedConnection);
            connect(bwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
            connect(bwdThread, &QThread::finished, bwdTracker, &QObject::deleteLater, Qt::UniqueConnection);
            connect(bwdThread, &QThread::finished, this, [this, bwdThread, wormId](){ // Capture wormId
                m_trackerThreads.removeOne(bwdThread);
                qDebug() << "Backward Tracker Thread for worm" << wormId << "finished and removed from list.";
            }, Qt::DirectConnection);

            connect(bwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
            connect(bwdTracker, &WormTracker::splitDetectedAndPaused, this, &TrackingManager::handleWormSplitDetectedAndPaused);
            connect(bwdTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
            connect(bwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(bwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
            m_wormTrackers.append(bwdTracker);
            m_trackerThreads.append(bwdThread);
            m_individualTrackerProgress[bwdTracker] = 0;
            bwdThread->start();
        } else {
            m_expectedTrackersToFinish--; // Decrement if not launched
            qDebug() << "TM: Skipping Backward Tracker for worm" << wormId << "due to no backward frames.";
        }
    }
    if(m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()){
        qWarning() << "TM: No trackers were launched. Check keyframe position and frame processing.";
        emit trackingFailed("No trackers could be launched. Video might be too short or keyframe at an extreme end.");
        m_isTrackingRunning = false;
        cleanupThreadsAndObjects();
    } else if (m_expectedTrackersToFinish == 0 && m_initialWormInfos.empty()){
        // This case is handled at the start of the function.
    }
}

void TrackingManager::handleWormPositionUpdated(
    int reportingWormId,
    int originalFrameNumber, QPointF newPosition, QRectF newRoi, // newRoi is the fixed search ROI used by tracker
    int plausibleBlobsFoundInRoi, double primaryBlobArea) {
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormObject* wormObject = m_wormObjectsMap.value(reportingWormId, nullptr);
    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender());

    if (wormObject && reportingTracker) {
        if (reportingTracker->getCurrentTrackerState() != WormTracker::TrackerState::PausedForSplit &&
            !m_wormToMergeGroupMap.contains(reportingWormId)) {
            cv::Point2f cvPos(static_cast<float>(newPosition.x()), static_cast<float>(newPosition.y()));
            // The ROI stored in WormObject should be the actual blob's ROI if available,
            // or the tracker's fixed search ROI if that's all we have.
            // For now, WormTracker::positionUpdated sends its fixed search ROI as newRoi.
            // If WormTracker could send the actual detected blob's boundingBox, that would be better here.
            wormObject->updateTrackPoint(originalFrameNumber, cvPos, newRoi); // Storing the search ROI for now
            WormTrackPoint lastPt;
            lastPt.frameNumberOriginal = originalFrameNumber;
            lastPt.position = cvPos;
            lastPt.roi = newRoi; // This is the search ROI
            emit individualWormTrackUpdated(reportingWormId, lastPt);
        }

        WormFrameInfo info;
        info.position = newPosition; // Centroid of the primary blob
        info.roi = newRoi;           // The fixed search ROI used by the tracker for this frame
        info.plausibleBlobsInRoi = plausibleBlobsFoundInRoi;
        info.primaryBlobArea = primaryBlobArea;
        info.reportingTracker = reportingWormId;
        info.isValid = true;
        // It would be ideal if info also contained the actual boundingBox of the primaryBlob.
        // This would require WormTracker to emit it.

        m_frameInfos[originalFrameNumber][reportingWormId] = info;

        while (m_frameInfos.size() > m_frameInfoHistorySize && !m_frameInfos.isEmpty()) {
            m_frameInfos.remove(m_frameInfos.firstKey());
        }
        processFrameDataForMergesAndSplits(originalFrameNumber);
    } else {
        qWarning() << "TrackingManager: handleWormPositionUpdated from unknown tracker or for unknown worm ID:" << reportingWormId;
    }
}

void TrackingManager::handleWormSplitDetectedAndPaused(
    int reportingWormId,
    int originalFrameNumber,
    const QList<TrackingHelper::DetectedBlob>& detectedBlobs) {

    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* pausedTracker = qobject_cast<WormTracker*>(sender());
    if (!pausedTracker) {
        qWarning() << "TM: Split detected, but sender is not a WormTracker. WormID from signal:" << reportingWormId;
        return;
    }
    if (pausedTracker->getWormId() != reportingWormId) {
        qWarning() << "TM: Split detected, sender WormTracker ID" << pausedTracker->getWormId()
        << "does not match signal's reportingWormId" << reportingWormId << ". Ignoring.";
        return;
    }

    qDebug() << "TM: Worm ID" << reportingWormId
             << "(Tracker:" << pausedTracker << ", Dir:" << pausedTracker->getDirection() << ")"
             << "reported SPLIT and PAUSED at frame" << originalFrameNumber
             << "with" << detectedBlobs.count() << "new blobs.";
    emit trackingStatusUpdate(
        QString("Worm %1 (Dir: %2) paused for split resolution at frame %3.")
            .arg(reportingWormId)
            .arg(pausedTracker->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd")
            .arg(originalFrameNumber));

    QSet<int> idsInMergeGroup;
    int representativeId = m_wormToMergeGroupMap.value(reportingWormId, -1);

    if (representativeId != -1 && m_mergedGroups.contains(representativeId)) {
        idsInMergeGroup = m_mergedGroups.value(representativeId);
        qDebug() << "  Split involves merge group represented by" << representativeId << "with worms:" << idsInMergeGroup;
    } else {
        qWarning() << "TM: Paused tracker for worm" << reportingWormId << "was not in a formal merge group. Assuming it was tracking solo.";
        idsInMergeGroup.insert(reportingWormId); // The worm itself
    }

    if (detectedBlobs.isEmpty()) {
        qWarning() << "TM: Split detected for worms" << idsInMergeGroup << "but no blobs provided. Trackers may be lost.";
        // Mark worms as lost or allow trackers to finish without resuming.
        for (int idInGroup : idsInMergeGroup) {
            WormObject* wo = m_wormObjectsMap.value(idInGroup, nullptr);
            if (wo) wo->setState(WormObject::TrackingState::Lost);
            QList<WormTracker*> trackersToStop = findTrackersForWorm(idInGroup);
            for(WormTracker* tr : trackersToStop) {
                if(tr == pausedTracker || tr->getCurrentTrackerState() == WormTracker::TrackerState::TrackingMerged) { // Stop relevant trackers
                    QMetaObject::invokeMethod(tr, "stopTracking", Qt::QueuedConnection);
                }
            }
        }
        // Update merge state: dissolve the group
        if (representativeId != -1) m_mergedGroups.remove(representativeId);
        for (int idInGroup : idsInMergeGroup) m_wormToMergeGroupMap.remove(idInGroup);
        return;
    }

    QMap<int, QPointF> lastPositions;
    for (int idInGroup : idsInMergeGroup) {
        WormObject* wo = m_wormObjectsMap.value(idInGroup, nullptr);
        if (wo) {
            // Use the last known position from WormObject
            lastPositions[idInGroup] = QPointF(wo->getCurrentPosition().x, wo->getCurrentPosition().y);
        } else {
            qWarning() << "TM: WormObject not found for ID" << idInGroup << "during split. Cannot get last position.";
        }
    }
    // If for the pausedTracker's worm, its WormObject position isn't good, try tracker's last known blob.
    if (lastPositions.value(reportingWormId, QPointF(-1,-1)).x() < 0 && pausedTracker->property("lastPrimaryBlob").isValid()) {
        TrackingHelper::DetectedBlob lastBlob = pausedTracker->property("lastPrimaryBlob").value<TrackingHelper::DetectedBlob>();
        if (lastBlob.isValid) lastPositions[reportingWormId] = lastBlob.centroid;
    }


    if (lastPositions.isEmpty() && !idsInMergeGroup.isEmpty()) {
        qWarning() << "TM: Could not retrieve any last known positions for worms in the split group:" << idsInMergeGroup << ". Cannot assign blobs.";
        // Similar to no detected blobs, mark as lost.
        for (int idInGroup : idsInMergeGroup) {
            WormObject* wo = m_wormObjectsMap.value(idInGroup, nullptr);
            if (wo) wo->setState(WormObject::TrackingState::Lost);
            QList<WormTracker*> trackersToStop = findTrackersForWorm(idInGroup);
            for(WormTracker* tr : trackersToStop) {
                if(tr == pausedTracker || tr->getCurrentTrackerState() == WormTracker::TrackerState::TrackingMerged) {
                    QMetaObject::invokeMethod(tr, "stopTracking", Qt::QueuedConnection);
                }
            }
        }
        if (representativeId != -1) m_mergedGroups.remove(representativeId);
        for (int idInGroup : idsInMergeGroup) m_wormToMergeGroupMap.remove(idInGroup);
        return;
    }

    QList<TrackingHelper::DetectedBlob> availableBlobs = detectedBlobs;
    QMap<int, TrackingHelper::DetectedBlob> assignments;
    QSet<int> assignedBlobIndices;

    // Create a list of worm IDs that need assignment and have a last position
    QList<int> wormsToAssignSorted;
    for(int id : idsInMergeGroup) {
        if(lastPositions.contains(id)) wormsToAssignSorted.append(id);
    }
    // Sort by ID to make assignment deterministic if distances are equal (optional)
    std::sort(wormsToAssignSorted.begin(), wormsToAssignSorted.end());


    for (int wormIdToAssign : wormsToAssignSorted) {
        QPointF lastPos = lastPositions.value(wormIdToAssign);
        double minSqDist = std::numeric_limits<double>::max();
        int bestBlobIdx = -1;

        for (int i = 0; i < availableBlobs.size(); ++i) {
            if (assignedBlobIndices.contains(i)) continue;
            double sqDist = squaredDistance(lastPos, availableBlobs.at(i).centroid);
            if (sqDist < minSqDist) {
                minSqDist = sqDist;
                bestBlobIdx = i;
            }
        }

        if (bestBlobIdx != -1) {
            assignments[wormIdToAssign] = availableBlobs.at(bestBlobIdx);
            assignedBlobIndices.insert(bestBlobIdx);
            qDebug() << "  Assigning blob" << bestBlobIdx << "(Centroid:" << availableBlobs.at(bestBlobIdx).centroid << ") to worm" << wormIdToAssign;
        } else {
            qDebug() << "  Could not find an unassigned blob for worm" << wormIdToAssign << ". It might be lost.";
            WormObject* wo = m_wormObjectsMap.value(wormIdToAssign, nullptr);
            if (wo) wo->setState(WormObject::TrackingState::Lost);
            // Stop trackers for this worm if no blob assigned
            QList<WormTracker*> trackersForLostWorm = findTrackersForWorm(wormIdToAssign);
            for(WormTracker* tr : trackersForLostWorm) {
                QMetaObject::invokeMethod(tr, "stopTracking", Qt::QueuedConnection);
            }
        }
    }

    for (int assignedWormId : assignments.keys()) {
        TrackingHelper::DetectedBlob assignedBlob = assignments.value(assignedWormId);
        QList<WormTracker*> trackersForThisWorm = findTrackersForWorm(assignedWormId);

        for (WormTracker* trackerInstance : trackersForThisWorm) {
            // Only instruct trackers that were part of the merge or the one that paused.
            // A simple check: if it's the paused one, or if its current state implies it was merged.
            if (trackerInstance == pausedTracker || trackerInstance->getCurrentTrackerState() == WormTracker::TrackerState::TrackingMerged) {
                qDebug() << "  Instructing Tracker for worm" << trackerInstance->getWormId()
                << "(Dir:" << trackerInstance->getDirection()
                << ", Ptr:" << trackerInstance << ") to resume with blob at" << assignedBlob.centroid;
                QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithNewTarget", Qt::QueuedConnection,
                                          Q_ARG(TrackingHelper::DetectedBlob, assignedBlob));
            } else if (trackerInstance->getCurrentTrackerState() == WormTracker::TrackerState::PausedForSplit && trackerInstance != pausedTracker) {
                // Another tracker for the same conceptual worm also paused, instruct it too.
                qDebug() << "  Instructing other paused Tracker for worm" << trackerInstance->getWormId()
                         << "(Dir:" << trackerInstance->getDirection()
                         << ", Ptr:" << trackerInstance << ") to resume with blob at" << assignedBlob.centroid;
                QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithNewTarget", Qt::QueuedConnection,
                                          Q_ARG(TrackingHelper::DetectedBlob, assignedBlob));
            }
        }
        WormObject* wo = m_wormObjectsMap.value(assignedWormId, nullptr);
        if (wo) wo->setState(WormObject::TrackingState::Tracking);
    }

    if (representativeId != -1) {
        m_mergedGroups.remove(representativeId);
    }
    for (int idInGroup : idsInMergeGroup) {
        m_wormToMergeGroupMap.remove(idInGroup);
        qDebug() << "  Worm" << idInGroup << "removed from merge group tracking.";
    }
    emit trackingStatusUpdate(QString("Split resolved for worms. Resuming individual tracking."));
}


void TrackingManager::processFrameDataForMergesAndSplits(int frameNumber) {
/*    if (m_cancelRequested || !m_isTrackingRunning) return;
    if (!m_frameInfos.contains(frameNumber)) return;

    const QMap<int, WormFrameInfo>& currentFrameTrackerData = m_frameInfos.value(frameNumber);
    QList<int> activeTrackersInFrame = currentFrameTrackerData.keys();

    for (int i = 0; i < activeTrackersInFrame.size(); ++i) {
        WormTracker* trackerA = activeTrackersInFrame.at(i);
        if (!trackerA) continue;
        const WormFrameInfo& infoA = currentFrameTrackerData.value(trackerA);
        int idA = trackerA->getWormId();

        if (m_wormToMergeGroupMap.contains(idA) || trackerA->getCurrentTrackerState() == WormTracker::TrackerState::PausedForSplit) {
            continue; // Already in a merge or paused, skip for new merge detection
        }

        for (int j = i + 1; j < activeTrackersInFrame.size(); ++j) {
            WormTracker* trackerB = activeTrackersInFrame.at(j);
            if (!trackerB) continue;
            const WormFrameInfo& infoB = currentFrameTrackerData.value(trackerB);
            int idB = trackerB->getWormId();

            if (idA == idB) continue; // Same conceptual worm (fwd vs bwd tracker)
            if (m_wormToMergeGroupMap.contains(idB) || trackerB->getCurrentTrackerState() == WormTracker::TrackerState::PausedForSplit) {
                continue; // Already in a merge or paused
            }

            // Use actual blob bounding boxes for intersection if WormFrameInfo contains them.
            // For now, using search ROIs as a proxy.
            // Ideally: QRectF actualBboxA = infoA.primaryBlobBoundingBox;
            QRectF searchRoiA = infoA.roi;
            QRectF searchRoiB = infoB.roi;

            // A more robust check would be intersection of actual detected blob bounding boxes.
            // For now, if their fixed search ROIs intersect AND they both found substantial blobs.
            if (searchRoiA.intersects(searchRoiB)) {
                // Check if both trackers found something significant.
                // Need a way to get minBlobArea for each tracker, or use a global one.
                // Assuming WormTracker has a getMinBlobArea() or we use TrackingConstants
                double minAreaThresholdA = trackerA->property("minBlobArea").isValid() ? trackerA->property("minBlobArea").toDouble() : TrackingConstants::DEFAULT_MIN_WORM_AREA;
                double minAreaThresholdB = trackerB->property("minBlobArea").isValid() ? trackerB->property("minBlobArea").toDouble() : TrackingConstants::DEFAULT_MIN_WORM_AREA;

                if (infoA.primaryBlobArea > minAreaThresholdA * 0.8 && infoB.primaryBlobArea > minAreaThresholdB * 0.8) {
                    qDebug() << "TM: Potential MERGE detected at frame" << frameNumber << "between worm" << idA << "(tracker" << trackerA << ")"
                             << "and worm" << idB << "(tracker" << trackerB << ") based on ROI intersection and blob presence.";
                    emit trackingStatusUpdate(QString("Merge detected: %1 and %2").arg(idA).arg(idB));

                    int repA = m_wormToMergeGroupMap.value(idA, idA);
                    int repB = m_wormToMergeGroupMap.value(idB, idB);
                    int finalRep = qMin(repA, repB); // Choose one representative (e.g., smallest ID)

                    QSet<int> groupA_members = m_mergedGroups.value(repA, QSet<int>() << idA);
                    QSet<int> groupB_members = m_mergedGroups.value(repB, QSet<int>() << idB);
                    QSet<int> newMergedGroupMembers = groupA_members + groupB_members;

                    // Update maps
                    if (repA != finalRep && m_mergedGroups.contains(repA)) m_mergedGroups.remove(repA);
                    if (repB != finalRep && m_mergedGroups.contains(repB)) m_mergedGroups.remove(repB);
                    m_mergedGroups[finalRep] = newMergedGroupMembers;

                    for (int memberId : newMergedGroupMembers) {
                        m_wormToMergeGroupMap[memberId] = finalRep;
                        WormObject* wo = m_wormObjectsMap.value(memberId, nullptr);
                        if (wo) wo->setState(WormObject::TrackingState::Merged, finalRep);
                    }
                    qDebug() << "  Updated merge group. Representative:" << finalRep << "Members:" << newMergedGroupMembers;

                    // Calculate encompassing ROI for the new merged group
                    QRectF encompassingRoi;
                    for (int memberId : newMergedGroupMembers) {
                        // Find trackers for this member in the current frame
                        for(WormTracker* memberTracker : activeTrackersInFrame) {
                            if(memberTracker && memberTracker->getWormId() == memberId) {
                                const WormFrameInfo& memberInfo = currentFrameTrackerData.value(memberTracker);
                                // Ideally use actual blob bbox from memberInfo if it had it.
                                // For now, using the search ROI.
                                if (encompassingRoi.isNull()) {
                                    encompassingRoi = memberInfo.roi; // Or actual blob BBox
                                } else {
                                    encompassingRoi = encompassingRoi.united(memberInfo.roi); // Or actual blob BBox
                                }
                                break; // Found one tracker for this member
                            }
                        }
                    }
                    // Add some padding to the encompassing ROI
                    if (!encompassingRoi.isNull()) {
                        encompassingRoi.adjust(-5, -5, 5, 5); // Example padding
                        // Clamp to frame
                        encompassingRoi.setX(qMax(0.0, encompassingRoi.x()));
                        encompassingRoi.setY(qMax(0.0, encompassingRoi.y()));
                        if(encompassingRoi.right() > m_videoFrameSize.width) encompassingRoi.setRight(m_videoFrameSize.width);
                        if(encompassingRoi.bottom() > m_videoFrameSize.height) encompassingRoi.setBottom(m_videoFrameSize.height);
                    } else {
                        qWarning() << "TM: Could not determine encompassing ROI for merge group" << finalRep;
                        // Fallback: could use union of their last known search ROIs.
                        // For now, trackers will use their existing logic if ROI is null.
                    }


                    // Instruct all involved trackers
                    for (int memberId : newMergedGroupMembers) {
                        QList<WormTracker*> trackersToNotify = findTrackersForWorm(memberId);
                        for (WormTracker* trackerInstance : trackersToNotify) {
                            if (trackerInstance && trackerInstance->getCurrentTrackerState() != WormTracker::TrackerState::PausedForSplit) {
                                // Provide centroid of the whole merge (approximate for now)
                                // and the encompassing ROI.
                                QPointF mergedCentroid = encompassingRoi.isValid() ? encompassingRoi.center() : QPointF(infoA.position + infoB.position)/2.0; // very rough
                                QMetaObject::invokeMethod(trackerInstance, "confirmTargetIsMerged", Qt::QueuedConnection,
                                                          Q_ARG(int, finalRep),
                                                          Q_ARG(QPointF, mergedCentroid),
                                                          Q_ARG(QRectF, encompassingRoi.isValid() ? encompassingRoi : searchRoiA.united(searchRoiB) ));
                            }
                        }
                    }
                    // Important: after processing a pair and merging, we might need to restart or be careful,
                    // as trackerA and trackerB are now in a merged group.
                    // The 'continue' for already merged trackers at the start of the loops helps.
                    // This simple pairwise check might lead to cascading merges in one frame.
                }
            }
        }
    }*/
}


void TrackingManager::handleWormStateChanged(int wormId, WormTracker::TrackerState newState) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    WormObject* worm = m_wormObjectsMap.value(wormId, nullptr);
    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender());

    if (worm && reportingTracker) {
        qDebug() << "TM: Worm ID" << wormId << "(Tracker:" << reportingTracker << ", Dir:" << reportingTracker->getDirection()
        << ") State CHANGED to" << static_cast<int>(newState);

        // Potentially update WormObject's high-level state if needed,
        // though WormTracker::TrackerState is more detailed.
        // Example: if newState is TrackingLost, update wormObject.
    }
}

void TrackingManager::handleWormTrackerFinished() {
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if (!finishedTracker) {
        qWarning() << "TrackingManager: handleWormTrackerFinished called by non-WormTracker sender.";
        // Potentially decrement m_expectedTrackersToFinish if this is an unaccounted finish.
        // However, this path should ideally not be hit if connections are set up correctly.
        return;
    }

    int wormId = finishedTracker->getWormId();
    QString direction = finishedTracker->property("direction").toString();
    qDebug() << "TrackingManager: WormTracker for worm ID" << wormId
             << "(Dir:" << direction << ", Ptr: " << finishedTracker << ") reported finished.";

    // Remove from active lists. It will be deleteLater'd by its thread.
    m_wormTrackers.removeOne(finishedTracker);
    m_individualTrackerProgress.remove(finishedTracker);

    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: Total finished trackers:"
             << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;

    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerError(int wormId, QString errorMessage) {
    if (m_cancelRequested && !m_isTrackingRunning) { // If already cancelling or stopped, log but don't re-trigger failure
        qDebug() << "TrackingManager: Error from tracker for worm ID" << wormId << "during/after cancel/stop:" << errorMessage;
        // Tracker will finish, let normal flow handle it.
        return;
    }
    if (!m_isTrackingRunning) {
        qDebug() << "TrackingManager: Error from tracker for worm ID" << wormId << "but tracking not running:" << errorMessage;
        return;
    }


    qWarning() << "TrackingManager: Error from tracker for worm ID" << wormId
               << ":" << errorMessage;
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    if (errorTracker) {
        m_wormTrackers.removeOne(errorTracker);
        m_individualTrackerProgress.remove(errorTracker);
        // The tracker's thread will delete it.
    }
    m_finishedTrackersCount++; // Count it as "finished" to not stall the process
    emit trackingStatusUpdate(
        QString("Error with tracker for worm %1: %2. Trying to continue...")
            .arg(wormId).arg(errorMessage.left(50)));

    updateOverallProgress();
    checkForAllTrackersFinished(); // Check if this error means all are now done or failed
}

void TrackingManager::handleWormTrackerProgress(int wormId, int percentDone) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    WormTracker* tracker = qobject_cast<WormTracker*>(sender());
    if (tracker) {
        // Check if tracker is still in our active list (it might have finished right after sending progress)
        if (m_individualTrackerProgress.contains(tracker)) {
            m_individualTrackerProgress[tracker] = percentDone;
            updateOverallProgress();
        }
    }
}


void TrackingManager::checkForAllTrackersFinished() {
    qDebug() << "TM: Checking if all trackers finished. Finished:" << m_finishedTrackersCount << "Expected:" << m_expectedTrackersToFinish << "Running:" << m_isTrackingRunning;

    if (m_isTrackingRunning && (m_finishedTrackersCount >= m_expectedTrackersToFinish || m_wormTrackers.isEmpty()) ) {
        if (m_cancelRequested) {
            qDebug() << "TrackingManager: All trackers accounted for after CANCELLATION request.";
            // Consolidate whatever tracks were gathered
            m_finalTracks.clear();
            for (WormObject* worm : qAsConst(m_wormObjectsMap)) {
                if (worm) {
                    m_finalTracks[worm->getId()] = worm->getTrackHistory();
                }
            }
            if (!m_finalTracks.empty()) {
                qDebug() << "TrackingManager: Emitting partial tracks due to cancellation. Count:" << m_finalTracks.size();
                emit allTracksUpdated(m_finalTracks);
                // outputTracksToDebug(m_finalTracks); // Optional
            } else {
                qDebug() << "TrackingManager: No tracks to emit upon cancellation.";
            }
            emit trackingStatusUpdate("Tracking cancelled by user. Partial tracks (if any) processed.");
            emit trackingCancelled(); // Signal that cancellation is complete
        } else {
            qDebug() << "TrackingManager: All trackers accounted for NORMALLY.";
            emit trackingStatusUpdate("All worm trackers completed. Consolidating and saving tracks...");
            m_finalTracks.clear();
            for (WormObject* worm : qAsConst(m_wormObjectsMap)) {
                if (worm) {
                    m_finalTracks[worm->getId()] = worm->getTrackHistory();
                }
            }
            emit allTracksUpdated(m_finalTracks);
            // outputTracksToDebug(m_finalTracks); // Optional

            QString csvOutputPath;
            if (!m_videoPath.isEmpty()) {
                QFileInfo videoInfo(m_videoPath);
                QString baseName = videoInfo.completeBaseName();
                QString outputDir = videoInfo.absolutePath();
                csvOutputPath = QDir(outputDir).filePath(baseName + "_tracks.csv");
            } else {
                csvOutputPath = "worm_tracks.csv"; // Default if no video path
            }

            bool csvSaved = outputTracksToCsv(m_finalTracks, csvOutputPath);
            if (csvSaved) {
                emit trackingStatusUpdate("Tracking finished. Tracks saved to: " + csvOutputPath);
                emit trackingFinishedSuccessfully(csvOutputPath);
            } else {
                emit trackingStatusUpdate("Tracking finished, but failed to save CSV output.");
                emit trackingFailed("Failed to save CSV output."); // Use trackingFailed for this
            }
        }
        m_isTrackingRunning = false; // Mark tracking as no longer running
        // cleanupThreadsAndObjects(); // Perform final cleanup after all logic is done for this run
    } else if (!m_isTrackingRunning) {
        qDebug() << "TM: checkForAllTrackersFinished - Tracking was already stopped (e.g. by error or earlier cancel).";
        // If cancel was requested and we are here, it means it's the final part of cancel flow
        if(m_cancelRequested) {
            emit trackingCancelled(); // Ensure this is emitted if not already
        }
    }
    // If m_isTrackingRunning is true but not all trackers are finished, do nothing yet.
}

void TrackingManager::updateOverallProgress() {
    if (!m_isTrackingRunning && !m_cancelRequested) { // If not running and not in a cancel flow that needs final progress
        emit overallTrackingProgress(0);
        return;
    }

    double totalProgressValue = 0.0;
    // Weighting: e.g., video processing 10%, trackers 90%
    double videoProcWeight = 0.10;
    double trackersWeight = 0.90;

    totalProgressValue += static_cast<double>(m_videoProcessingProgress) * videoProcWeight / 100.0;

    if (m_expectedTrackersToFinish > 0) {
        double sumOfIndividualProgress = 0;
        for (int progress : m_individualTrackerProgress.values()) {
            sumOfIndividualProgress += progress;
        }
        // Average progress of trackers that are still in m_individualTrackerProgress (i.e. not yet fully finished and removed)
        // Plus contribution from already finished trackers
        double finishedContribution = static_cast<double>(m_finishedTrackersCount) * 100.0;
        double totalTrackerPoints = sumOfIndividualProgress + finishedContribution;

        double overallTrackerPercentage = (totalTrackerPoints / (static_cast<double>(m_expectedTrackersToFinish) * 100.0)) * 100.0;
        overallTrackerPercentage = qBound(0.0, overallTrackerPercentage, 100.0);

        totalProgressValue += overallTrackerPercentage * trackersWeight / 100.0;
    } else if (m_videoProcessingProgress == 100) { // No trackers expected, video processing is 100% of the task
        totalProgressValue = 1.0; // Full progress
    }


    int finalProgressPercentage = qBound(0, static_cast<int>(totalProgressValue * 100.0), 100);
    emit overallTrackingProgress(finalProgressPercentage);
}

void TrackingManager::outputTracksToDebug(const AllWormTracks& tracks) const {
    qDebug() << "--- Begin Track Output (" << this << ") ---";
    if (tracks.empty()) {
        qDebug() << "No tracks to output.";
    }
    for (auto const& [wormId, points] : tracks) {
        qDebug() << "Worm ID:" << wormId << "Number of points:" << points.size();
        for (const WormTrackPoint& point : points) {
            qDebug() << "  Frame:" << point.frameNumberOriginal
                     << "X:" << QString::number(point.position.x, 'f', 2)
                     << "Y:" << QString::number(point.position.y, 'f', 2)
                     << "ROI: [x:" << QString::number(point.roi.x(), 'f', 1)
                     << "y:" << QString::number(point.roi.y(), 'f', 1)
                     << "w:" << QString::number(point.roi.width(), 'f', 1)
                     << "h:" << QString::number(point.roi.height(), 'f', 1) << "]";
        }
    }
    qDebug() << "--- End Track Output ---";
}

bool TrackingManager::outputTracksToCsv(const AllWormTracks& tracks, const QString& outputFilePath) const {
    if (outputFilePath.isEmpty()) {
        qWarning() << "outputTracksToCsv: No output file path provided.";
        return false;
    }
    QFile csvFile(outputFilePath);
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        qWarning() << "Failed to open CSV file for writing:" << outputFilePath << csvFile.errorString();
        return false;
    }
    QTextStream outStream(&csvFile);
    outStream << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight\n";
    for (auto const& [wormId, points] : tracks) { // C++17 structured binding
        for (const WormTrackPoint& point : points) {
            outStream << wormId << ","
                      << point.frameNumberOriginal << ","
                      << QString::number(point.position.x, 'f', 4) << ","
                      << QString::number(point.position.y, 'f', 4) << ","
                      << QString::number(point.roi.x(), 'f', 2) << ","
                      << QString::number(point.roi.y(), 'f', 2) << ","
                      << QString::number(point.roi.width(), 'f', 2) << ","
                      << QString::number(point.roi.height(), 'f', 2) << "\n";
        }
    }
    csvFile.close();
    if (csvFile.error() != QFile::NoError) {
        qWarning() << "Error writing to CSV file:" << outputFilePath << csvFile.errorString();
        return false;
    }
    qDebug() << "Tracks successfully written to CSV:" << outputFilePath;
    return true;
}
