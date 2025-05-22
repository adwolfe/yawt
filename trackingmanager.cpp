// trackingmanager.cpp
#include "trackingmanager.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QPointer> // For QPointer in PausedTrackerInfo if we make that change later

#include <algorithm> // For std::min, std::sort, std::transform etc.
#include <numeric>   // For std::iota if used for assignments
#include <limits>    // For std::numeric_limits

// Helper function to calculate squared Euclidean distance
// (Consider moving to a common utility header if used elsewhere extensively)
static double squaredDistance(const QPointF& p1, const QPointF& p2) {
    QPointF diff = p1 - p2;
    return QPointF::dotProduct(diff, diff);
}

// Define a threshold for considering blobs from different trackers as potentially interacting/merged.
// This is a placeholder; actual value might depend on worm size, resolution, etc.
const double MERGE_INTERACTION_DISTANCE_THRESHOLD_SQ = 100.0; // e.g., (10 pixels)^2
const double MIN_OVERLAP_AREA_FOR_MERGE_RATIO = 0.1; // Minimum overlap relative to smaller blob's area


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
    m_videoProcessingProgress(0),
    m_frameInfoHistorySize(5) // Initialize history size (e.g., 5 frames)
{
    // Register meta types for signals/slots
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<WormObject::TrackingState>("WormObject::TrackingState");
    qRegisterMetaType<Tracking::AllWormTracks>("Tracking::AllWormTracks");
    qRegisterMetaType<QList<Tracking::DetectedBlob>>("QList<Tracking::DetectedBlob>");
    qRegisterMetaType<Tracking::DetectedBlob>("Tracking::DetectedBlob");
    qRegisterMetaType<WormTracker::TrackerState>("WormTracker::TrackerState");
    // qRegisterMetaType<WormTracker::TrackingDirection>("WormTracker::TrackingDirection"); // Already Q_ENUM

    qDebug() << "TrackingManager (" << this << ") created.";
}

TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking(); // This sets m_cancelRequested = true
        // Give some time for threads to respond to cancellation.
        QThread::msleep(200); // Consider if this is the best approach or if wait() in cleanup is enough
    }
    cleanupThreadsAndObjects();
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - FINISHED cleaning up.";
}

void TrackingManager::startFullTrackingProcess(
    const QString& videoPath, int keyFrameNum,
    const std::vector<Tracking::InitialWormInfo>& initialWorms,
    const Thresholding::ThresholdSettings& settings, int totalFramesInVideo) {
    qDebug() << "TrackingManager (" << this << "): startFullTrackingProcess called.";
    if (m_isTrackingRunning) {
        qWarning() << "TrackingManager: Attempted to start tracking while already running.";
        emit trackingFailed("Another tracking process is already running.");
        return;
    }

    cleanupThreadsAndObjects(); // Ensure clean state before starting

    m_videoPath = videoPath;
    m_keyFrameNum = keyFrameNum;
    m_initialWormInfos = initialWorms;
    m_thresholdSettings = settings;
    m_totalFramesInVideo = totalFramesInVideo;
    m_isTrackingRunning = true;
    m_cancelRequested = false;
    m_videoProcessingProgress = 0;
    m_finishedTrackersCount = 0;
    m_expectedTrackersToFinish = 0;

    // Clear all data structures for the new run
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameInfos.clear();

    m_forwardMergedGroups.clear();
    m_forwardWormToMergeGroupMap.clear();
    m_reverseMergedGroups.clear();
    m_reverseWormToMergeGroupMap.clear();
    m_pausedTrackerCache.clear();

    // Clear WormObjects from previous runs if any
    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    // Create WormObjects for this run
    for (const auto& info : m_initialWormInfos) {
        if (!m_wormObjectsMap.contains(info.id)) { // Should always be true after clear
            m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
        }
    }

    emit trackingStatusUpdate("Starting video processing...");
    emit overallTrackingProgress(0);

    m_videoProcessor = new VideoProcessor();
    m_videoProcessorThread = new QThread(this); // Parent to this TrackingManager
    m_videoProcessor->moveToThread(m_videoProcessorThread);

    connect(m_videoProcessorThread, &QThread::started, m_videoProcessor, [this]() {
        if (m_videoProcessor) { // Check if still valid
            m_videoProcessor->startInitialProcessing(m_videoPath, m_keyFrameNum,
                                                     m_thresholdSettings, m_totalFramesInVideo);
        }
    });
    connect(m_videoProcessor, &VideoProcessor::initialProcessingComplete, this, &TrackingManager::handleInitialProcessingComplete);
    connect(m_videoProcessor, &VideoProcessor::processingError, this, &TrackingManager::handleVideoProcessingError);
    connect(m_videoProcessor, &VideoProcessor::initialProcessingProgress, this, &TrackingManager::handleVideoProcessingProgress);
    connect(m_videoProcessor, &VideoProcessor::processingStarted, this, [this]() {
        emit trackingStatusUpdate("Video processing started in thread.");
    });

    // Ensure VideoProcessor is deleted when its thread finishes
    connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor, &QObject::deleteLater);
    // Ensure m_videoProcessor pointer is nulled after its thread finishes and it's deleted.
    connect(m_videoProcessorThread, &QThread::finished, this, [this]() {
        qDebug() << "VideoProcessorThread (" << QObject::sender() << ") finished. m_videoProcessor (" << m_videoProcessor << ") was scheduled for deleteLater.";
        m_videoProcessor = nullptr; // Nullify after deleteLater is scheduled
        // m_videoProcessorThread itself will be deleted in cleanupThreadsAndObjects
    });


    m_videoProcessorThread->start();
}

void TrackingManager::cancelTracking() {
    qDebug() << "TrackingManager (" << this << "): cancelTracking called. IsRunning:" << m_isTrackingRunning << "CancelRequested:" << m_cancelRequested;
    if (m_cancelRequested) {
        qDebug() << "TM: Cancellation already in progress.";
        return;
    }
    m_cancelRequested = true;
    emit trackingStatusUpdate("Cancellation requested...");

    if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
        qDebug() << "TM: Requesting video processor thread (" << m_videoProcessorThread << ") interruption.";
        m_videoProcessorThread->requestInterruption();
        // VideoProcessor should handle interruption and stop.
    }

    // Iterate over a copy for safety if m_wormTrackers can be modified during stopTracking
    QList<WormTracker*> trackersToStop = m_wormTrackers;
    for (WormTracker* tracker : trackersToStop) {
        if (tracker) {
            // stopTracking will eventually lead to the tracker's thread finishing
            QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
        }
    }
    // Actual thread cleanup happens in handleWormTrackerFinished or final cleanup.
    // checkForAllTrackersFinished will eventually be called, which handles emitting trackingCancelled.
}

void TrackingManager::cleanupThreadsAndObjects() {
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - START";

    if (m_videoProcessorThread) {
        qDebug() << "  Cleaning up video processor thread:" << m_videoProcessorThread;
        if (m_videoProcessorThread->isRunning()) {
            m_videoProcessorThread->requestInterruption();
            m_videoProcessorThread->quit();
            if (!m_videoProcessorThread->wait(1000)) { // Wait for graceful exit
                qWarning() << "  VideoProcessor thread (" << m_videoProcessorThread << ") did not finish gracefully. Forcing termination.";
                m_videoProcessorThread->terminate(); // Last resort
                m_videoProcessorThread->wait();      // Wait for termination to complete
            }
        }
        // m_videoProcessor is connected to deleteLater with thread's finished signal.
        // The thread object itself needs to be deleted.
        delete m_videoProcessorThread;
        m_videoProcessorThread = nullptr;
        m_videoProcessor = nullptr; // Ensure pointer is null
    }

    // Stop and delete tracker threads
    // Iterate over a copy because m_trackerThreads might be modified by finished signals
    QList<QThread*> threadsToClean = m_trackerThreads;
    for (QThread* thread : threadsToClean) {
        if (thread) {
            qDebug() << "  Cleaning up tracker thread:" << thread;
            if (thread->isRunning()) {
                thread->requestInterruption(); // WormTracker::stopTracking should handle this
                thread->quit();
                if (!thread->wait(500)) {
                    qWarning() << "  A WormTracker thread (" << thread << ") did not finish gracefully. Forcing termination.";
                    thread->terminate();
                    thread->wait();
                }
            }
            delete thread; // Delete the QThread object
        }
    }
    m_trackerThreads.clear();
    // WormTracker objects are set to deleteLater when their threads finish.
    // We just clear our list of pointers.
    m_wormTrackers.clear();

    // Delete WormObject instances
    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    // Clear frame data
    m_processedForwardFrames.clear();
    std::vector<cv::Mat>().swap(m_processedForwardFrames); // Ensure memory is released
    m_processedReversedFrames.clear();
    std::vector<cv::Mat>().swap(m_processedReversedFrames);

    // Clear tracking data structures
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameInfos.clear();
    m_forwardMergedGroups.clear();
    m_forwardWormToMergeGroupMap.clear();
    m_reverseMergedGroups.clear();
    m_reverseWormToMergeGroupMap.clear();
    m_pausedTrackerCache.clear();

    m_isTrackingRunning = false;
    // m_cancelRequested is reset in startFullTrackingProcess
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - FINISHED";
}


TrackingManager::MergeMapsContext TrackingManager::getMergeMapsForFrame(int originalFrameNumber) {
    if (originalFrameNumber >= m_keyFrameNum) { // Forward context
        return {m_forwardMergedGroups, m_forwardWormToMergeGroupMap};
    } else { // Reverse context
        return {m_reverseMergedGroups, m_reverseWormToMergeGroupMap};
    }
}

// --- VideoProcessor Signal Handlers ---
void TrackingManager::handleInitialProcessingComplete(
    const std::vector<cv::Mat>& forwardFrames,
    const std::vector<cv::Mat>& reversedFrames, double fps,
    cv::Size frameSize) {
    qDebug() << "TrackingManager (" << this << "): handleInitialProcessingComplete received.";
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing complete but cancellation requested. Not launching trackers.";
        // cleanupThreadsAndObjects(); // Already called by cancel or will be by checkForAllTrackersFinished
        emit trackingCancelled(); // Ensure this is emitted if VP finishes during cancel
        m_isTrackingRunning = false; // Ensure state is correct
        return;
    }
    if (!m_isTrackingRunning) {
        qWarning() << "TM: handleInitialProcessingComplete, but tracking is not marked as running. Aborting tracker launch.";
        // This might happen if an error occurred just before this signal.
        // cleanupThreadsAndObjects(); // Avoid redundant cleanup if error path already did it.
        return;
    }

    qDebug() << "TM: Initial video processing complete. Fwd:" << forwardFrames.size() << "Rev:" << reversedFrames.size() << "FPS:" << fps;
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    m_videoProcessingProgress = 100; // Mark video processing as fully complete
    m_processedForwardFrames = forwardFrames;
    m_processedReversedFrames = reversedFrames;
    m_videoFps = fps;
    m_videoFrameSize = frameSize;

    // WormObjects are already created in startFullTrackingProcess
    launchWormTrackers();
    updateOverallProgress(); // Reflects that video processing part is done
}

void TrackingManager::handleVideoProcessingError(const QString& errorMessage) {
    qDebug() << "TrackingManager (" << this << "): handleVideoProcessingError: " << errorMessage;
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing error (" << errorMessage << ") received during/after cancellation. Letting cancel flow proceed.";
        return; // Cancel flow will handle cleanup and final signals
    }
    if (!m_isTrackingRunning) {
        qWarning() << "TM: Video processing error (" << errorMessage << ") but tracking not marked as running.";
        return; // Avoid double error handling
    }

    qWarning() << "TM: Video processing error:" << errorMessage;
    emit trackingFailed("Video processing failed: " + errorMessage);
    m_isTrackingRunning = false; // Stop tracking
    // cleanupThreadsAndObjects(); // Let checkForAllTrackersFinished or destructor handle full cleanup
    // No trackers are running yet, so main cleanup is for VideoProcessor thread which is handled.
}

void TrackingManager::handleVideoProcessingProgress(int percentage) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    m_videoProcessingProgress = percentage;
    updateOverallProgress();
}


// --- WormTracker Signal Handlers ---
void TrackingManager::handleWormPositionUpdated(
    int reportingWormId,
    int originalFrameNumber,
    const Tracking::DetectedBlob& primaryBlob, // The actual blob tracked
    QRectF searchRoiUsed,                      // The search ROI used for detection
    int plausibleBlobsFoundInSearchRoi) {

    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormObject* wormObject = m_wormObjectsMap.value(reportingWormId, nullptr);
    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender()); // Useful for debug or getting tracker-specific props

    if (!wormObject) {
        qWarning() << "TrackingManager: handleWormPositionUpdated for unknown worm ID:" << reportingWormId;
        return;
    }
    if (!reportingTracker) {
        qWarning() << "TrackingManager: handleWormPositionUpdated from non-WormTracker sender for worm ID:" << reportingWormId;
        // We can still proceed with wormObject if conceptual ID is known.
    }

    // Update WormObject's track history if the blob is valid and not part of an unresolved merge by this tracker
    // (i.e., tracker is not PausedForSplit and not in a merge group that TM is actively managing for split)
    auto maps = getMergeMapsForFrame(originalFrameNumber);
    bool isWormInActiveMergeGroup = maps.wormToMergeGroupMap.contains(reportingWormId);

    if (primaryBlob.isValid && reportingTracker && reportingTracker->getCurrentTrackerState() != WormTracker::TrackerState::PausedForSplit && !isWormInActiveMergeGroup) {
        Tracking::WormTrackPoint point;
        point.frameNumberOriginal = originalFrameNumber;
        point.position = cv::Point2f(static_cast<float>(primaryBlob.centroid.x()), static_cast<float>(primaryBlob.centroid.y()));
        // Store the actual bounding box of the tracked blob as the ROI for this point, if valid.
        // Otherwise, could fall back to searchRoiUsed, but primaryBlob.boundingBox is preferred.
        point.roi = primaryBlob.boundingBox.isValid() ? primaryBlob.boundingBox : searchRoiUsed;
        // point.state = ...; // TODO: Determine annotation based on tracker state or other factors

        wormObject->updateTrackPoint(originalFrameNumber, point.position, point.roi);
        emit individualWormTrackUpdated(reportingWormId, point);
    } else if (!primaryBlob.isValid) {
        // If blob is invalid, we might still want to record a "miss" or "lost" point.
        // For now, WormObject is only updated with valid points.
        // We could emit an invalid point to UI if needed:
        // Tracking::WormTrackPoint lostPoint;
        // lostPoint.frameNumberOriginal = originalFrameNumber;
        // lostPoint.position = cv::Point2f(-1,-1); // Indicate invalid
        // lostPoint.roi = searchRoiUsed; // ROI where it was lost
        // emit individualWormTrackUpdated(reportingWormId, lostPoint);
    }


    // Store frame info for merge/split detection
    WormFrameInfo info;
    info.primaryBlob = primaryBlob;
    info.searchRoiUsed = searchRoiUsed;
    info.plausibleBlobsInSearchRoi = plausibleBlobsFoundInSearchRoi;
    info.reportingTrackerWormId = reportingWormId;

    m_frameInfos[originalFrameNumber][reportingWormId] = info;

    // Limit history size of m_frameInfos
    while (m_frameInfos.size() > m_frameInfoHistorySize && !m_frameInfos.isEmpty()) {
        m_frameInfos.remove(m_frameInfos.firstKey());
    }

    // Process this new frame's data for potential merges or to inform split resolutions
    // This needs to be intelligent about when it's called.
    // If multiple trackers report for the same frame, call it after all have reported,
    // or use a slight delay / queued call. For now, direct call.
    processFrameDataForMergesAndSplits(originalFrameNumber);

    // Potentially trigger resolution of paused trackers if this update is relevant
    if (m_pausedTrackerCache.contains(originalFrameNumber)) {
        // Using QMetaObject::invokeMethod to ensure it runs after current slot execution
        QMetaObject::invokeMethod(this, [this, originalFrameNumber](){
            resolvePausedTrackersForFrame(originalFrameNumber);
        }, Qt::QueuedConnection);
    }
}


void TrackingManager::handleWormSplitDetectedAndPaused(
    int reportingWormId,
    int originalFrameNumber,
    const QList<Tracking::DetectedBlob>& detectedBlobs) {

    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* pausedTracker = qobject_cast<WormTracker*>(sender());
    if (!pausedTracker) {
        qWarning() << "TM: Split detected, but sender is not a WormTracker. WormID from signal:" << reportingWormId;
        return;
    }
    // Basic validation
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

    PausedTrackerInfo info;
    info.wormId = reportingWormId;
    info.originalFrameNumber = originalFrameNumber;
    info.trackerInstance = pausedTracker; // Store raw pointer for now. QPointer is safer.
    info.reportedBlobs = detectedBlobs;

    m_pausedTrackerCache[originalFrameNumber].append(info);
    qDebug() << "TM: Added tracker" << pausedTracker << "for worm" << reportingWormId << "to paused cache for frame" << originalFrameNumber;

    // Schedule resolution for this frame. Using QueuedConnection to allow all signals for this frame
    // (e.g., from other trackers also pausing) to be processed before resolution.
    QMetaObject::invokeMethod(this, [this, originalFrameNumber](){
        resolvePausedTrackersForFrame(originalFrameNumber);
    }, Qt::QueuedConnection);
}

void TrackingManager::processFrameDataForMergesAndSplits(int frameNumber) {
    if (m_cancelRequested || !m_isTrackingRunning || !m_frameInfos.contains(frameNumber)) {
        return;
    }

    // Check if there's enough data to compare (at least two worms reported for this frame)
    if (m_frameInfos.value(frameNumber).size() < 2) {
        return;
    }

    qDebug() << "TM: Processing frame" << frameNumber << "for potential new merges.";

    const QMap<int, WormFrameInfo>& currentFrameData = m_frameInfos.value(frameNumber);
    auto maps = getMergeMapsForFrame(frameNumber); // Get context-specific merge maps

    QList<int> activeWormIdsInFrame = currentFrameData.keys();
    bool isForwardContext = (frameNumber >= m_keyFrameNum);

    for (int i = 0; i < activeWormIdsInFrame.size(); ++i) {
        int idA = activeWormIdsInFrame.at(i);
        const WormFrameInfo& infoA = currentFrameData.value(idA);

        // Ensure worm A's data is valid for merge consideration
        if (!infoA.primaryBlob.isValid || infoA.primaryBlob.boundingBox.isEmpty()) {
            continue;
        }

        // Find tracker A for state checking
        WormTracker* trackerA = nullptr;
        QList<WormTracker*> trackersForA = findTrackersForWorm(idA);
        for(WormTracker* tr : trackersForA) {
            if(tr && ((isForwardContext && tr->getDirection() == WormTracker::TrackingDirection::Forward) ||
                       (!isForwardContext && tr->getDirection() == WormTracker::TrackingDirection::Backward))) {
                trackerA = tr;
                break;
            }
        }
        // If trackerA is paused, it cannot initiate or be part of a new merge detection here.
        // Its situation will be handled by resolvePausedTrackersForFrame.
        if (trackerA && trackerA->getCurrentTrackerState() == WormTracker::TrackerState::PausedForSplit) {
            continue;
        }
        // If trackerA is already in a TrackingMerged state confirmed by the manager for this context,
        // it means its merge group is already defined. We shouldn't try to re-merge it here
        // unless we are refining groups, which is more complex. For now, if it's in a map, assume it's handled.
        if (maps.wormToMergeGroupMap.contains(idA) && trackerA && trackerA->getCurrentTrackerState() == WormTracker::TrackerState::TrackingMerged) {
            // It's already in a known merge group for this context.
            // We might still want to check if this group should merge with *another* group/worm.
            // This is handled by repA_current logic below.
        }


        for (int j = i + 1; j < activeWormIdsInFrame.size(); ++j) {
            int idB = activeWormIdsInFrame.at(j);
            const WormFrameInfo& infoB = currentFrameData.value(idB);

            // Ensure worm B's data is valid
            if (!infoB.primaryBlob.isValid || infoB.primaryBlob.boundingBox.isEmpty()) {
                continue;
            }
            // idA == idB check is implicitly handled by conceptual IDs if m_frameInfos is keyed correctly.

            // Find tracker B for state checking
            WormTracker* trackerB = nullptr;
            QList<WormTracker*> trackersForB = findTrackersForWorm(idB);
            for(WormTracker* tr : trackersForB) {
                if(tr && ((isForwardContext && tr->getDirection() == WormTracker::TrackingDirection::Forward) ||
                           (!isForwardContext && tr->getDirection() == WormTracker::TrackingDirection::Backward))) {
                    trackerB = tr;
                    break;
                }
            }
            if (trackerB && trackerB->getCurrentTrackerState() == WormTracker::TrackerState::PausedForSplit) {
                continue;
            }

            // --- Proactive Merge Detection Logic ---

            // Get current representative IDs for A and B. If not in a map, they represent themselves.
            int repA_current = maps.wormToMergeGroupMap.value(idA, idA);
            int repB_current = maps.wormToMergeGroupMap.value(idB, idB);

            // If they are already part of the *same* merge group, skip.
            if (repA_current == repB_current) {
                continue;
            }

            // Check for significant overlap of their primary blobs' bounding boxes
            QRectF bboxA = infoA.primaryBlob.boundingBox;
            QRectF bboxB = infoB.primaryBlob.boundingBox;
            QRectF intersection = bboxA.intersected(bboxB);

            if (intersection.isEmpty() || intersection.width() <= 0 || intersection.height() <=0 ) {
                continue; // No overlap
            }

            double overlapArea = intersection.width() * intersection.height();
            // Consider the smaller of the two primary blob areas for the ratio,
            // or ensure both blobs are substantial.
            double areaA = infoA.primaryBlob.area;
            double areaB = infoB.primaryBlob.area;
            double minRelevantBlobArea = qMin(areaA, areaB);

            // Ensure blobs are somewhat substantial before considering merge based on overlap
            // Using TrackingConstants::DEFAULT_MIN_WORM_AREA as a baseline.
            if (areaA < TrackingConstants::DEFAULT_MIN_WORM_AREA * 0.5 || areaB < TrackingConstants::DEFAULT_MIN_WORM_AREA * 0.5) {
                // One of the blobs is too small to be a reliable partner in a new merge, might be noise.
                // This threshold is arbitrary and might need tuning.
                continue;
            }


            if (minRelevantBlobArea > 0 && (overlapArea / minRelevantBlobArea) > MIN_OVERLAP_AREA_FOR_MERGE_RATIO) {
                // Significant overlap detected! This is a new merge or a merge of existing groups.
                qDebug() << "TM: Frame" << frameNumber << (isForwardContext ? "[Fwd]" : "[Rev]")
                         << "- MERGE DETECTED between conceptual worm/group" << repA_current << "(orig ID A:" << idA << ")"
                         << "and worm/group" << repB_current << "(orig ID B:" << idB << ")"
                         << "based on blob overlap. Overlap area:" << overlapArea;
                emit trackingStatusUpdate(QString("Merge detected: %1 & %2 at frame %3").arg(repA_current).arg(repB_current).arg(frameNumber));

                int finalRep = qMin(repA_current, repB_current); // New representative for the combined group

                // Get all members of the existing groups (or just the individuals if they weren't in groups)
                QSet<int> groupA_members = maps.mergedGroups.value(repA_current, QSet<int>() << idA);
                QSet<int> groupB_members = maps.mergedGroups.value(repB_current, QSet<int>() << idB);
                QSet<int> newMergedGroupMembers = groupA_members + groupB_members;

                // Update maps: remove old group entries if they are different from finalRep
                if (repA_current != finalRep && maps.mergedGroups.contains(repA_current)) {
                    maps.mergedGroups.remove(repA_current);
                }
                if (repB_current != finalRep && maps.mergedGroups.contains(repB_current)) {
                    maps.mergedGroups.remove(repB_current);
                }
                maps.mergedGroups[finalRep] = newMergedGroupMembers; // Store the new combined group

                // Calculate an encompassing ROI for the new merged group
                // Start with the union of the two interacting primary blobs
                QRectF encompassingRoi = bboxA.united(bboxB);
                // If the groups were larger, unite with their previous encompassing ROIs (not easily available here)
                // For now, just use the interacting blobs' union and add padding.
                encompassingRoi.adjust(-10, -10, 10, 10); // Add padding (e.g., 10 pixels)

                // Clamp to frame boundaries (m_videoFrameSize should be valid here)
                if(m_videoFrameSize.width > 0 && m_videoFrameSize.height > 0) {
                    encompassingRoi.setX(qMax(0.0, encompassingRoi.x()));
                    encompassingRoi.setY(qMax(0.0, encompassingRoi.y()));
                    if(encompassingRoi.right() > m_videoFrameSize.width) encompassingRoi.setRight(m_videoFrameSize.width);
                    if(encompassingRoi.bottom() > m_videoFrameSize.height) encompassingRoi.setBottom(m_videoFrameSize.height);
                    // Ensure width/height are also clamped if they were initially larger than the frame
                    encompassingRoi.setWidth(qMin(encompassingRoi.width(), static_cast<qreal>(m_videoFrameSize.width)));
                    encompassingRoi.setHeight(qMin(encompassingRoi.height(), static_cast<qreal>(m_videoFrameSize.height)));
                }


                for (int memberId : newMergedGroupMembers) {
                    maps.wormToMergeGroupMap[memberId] = finalRep; // Update each member's mapping to the new representative
                    WormObject* wo = m_wormObjectsMap.value(memberId, nullptr);
                    if (wo) {
                        wo->setState(WormObject::TrackingState::Merged, finalRep);
                    }

                    // Instruct the relevant tracker instances for this memberId
                    QList<WormTracker*> trackersToNotify = findTrackersForWorm(memberId);
                    for (WormTracker* trackerInstance : trackersToNotify) {
                        if (trackerInstance) {
                            // Check direction matches current frame context
                            if (((isForwardContext && trackerInstance->getDirection() == WormTracker::TrackingDirection::Forward) ||
                                 (!isForwardContext && trackerInstance->getDirection() == WormTracker::TrackingDirection::Backward))) {
                                // Only instruct if not already paused (paused trackers handle their state separately)
                                if (trackerInstance->getCurrentTrackerState() != WormTracker::TrackerState::PausedForSplit) {
                                    qDebug() << "  Instructing Tracker for worm" << memberId << "(Dir:" << trackerInstance->getDirection()
                                    << ", Ptr:" << trackerInstance << ") to CONFIRM MERGE with group" << finalRep << ", ROI:" << encompassingRoi;
                                    QMetaObject::invokeMethod(trackerInstance, "confirmTargetIsMerged", Qt::QueuedConnection,
                                                              Q_ARG(int, finalRep),
                                                              Q_ARG(QPointF, encompassingRoi.center()), // Approx centroid of the new merged ROI
                                                              Q_ARG(QRectF, encompassingRoi));
                                }
                            }
                        }
                    }
                }
                qDebug() << "  Frame" << frameNumber << (isForwardContext ? "[Fwd]" : "[Rev]")
                         << "- Updated merge group. Representative:" << finalRep << "Members:" << newMergedGroupMembers;
                // After merging A and B (or their groups), their new representative 'finalRep' is now set.
                // The outer loop for 'i' will continue. If 'finalRep' (now representing idA's group)
                // encounters another worm/group 'idC', the logic repA_current = maps.wormToMergeGroupMap.value(idA, idA)
                // will correctly pick up 'finalRep' if idA was part of that merge.
            }
        }
    }
}


void TrackingManager::resolvePausedTrackersForFrame(int frameNumber) {
    if (m_cancelRequested || !m_isTrackingRunning) {
        return;
    }
    if (!m_pausedTrackerCache.contains(frameNumber) || m_pausedTrackerCache.value(frameNumber).isEmpty()) {
        // qDebug() << "TM: No paused trackers in cache for frame" << frameNumber << "to resolve.";
        return;
    }

    qDebug() << "TM: Resolving paused trackers for frame" << frameNumber;
    QList<PausedTrackerInfo> pausedThisFrame = m_pausedTrackerCache.take(frameNumber); // Process and remove from cache

    auto maps = getMergeMapsForFrame(frameNumber); // Context-specific merge maps
    bool isForwardContext = (frameNumber >= m_keyFrameNum);

    // Group PausedTrackerInfo items by their original merge group representative ID.
    // Key: representativeId (or wormId if solo), Value: List of PausedTrackerInfo from that group.
    QMap<int, QList<PausedTrackerInfo>> groupedPausedTrackers;
    for (const PausedTrackerInfo& pInfo : pausedThisFrame) {
        if (!pInfo.trackerInstance) { // Check if tracker pointer is valid
            qWarning() << "TM: Stale PausedTrackerInfo for worm" << pInfo.wormId << "(tracker deleted). Skipping.";
            continue;
        }
        // Ensure the tracker is actually still paused. It might have been stopped by another mechanism.
        if (pInfo.trackerInstance->getCurrentTrackerState() != WormTracker::TrackerState::PausedForSplit) {
            qDebug() << "TM: Tracker for worm" << pInfo.wormId << "(Ptr:" << pInfo.trackerInstance
                     << ") is no longer in PausedForSplit state. Current state:"
                     << static_cast<int>(pInfo.trackerInstance->getCurrentTrackerState()) << ". Skipping its resolution here.";
            continue;
        }

        int representativeId = maps.wormToMergeGroupMap.value(pInfo.wormId, pInfo.wormId); // Default to self if not in a map
        groupedPausedTrackers[representativeId].append(pInfo);
    }


    for (int repId : groupedPausedTrackers.keys()) {
        const QList<PausedTrackerInfo>& groupPausedList = groupedPausedTrackers.value(repId);
        if (groupPausedList.isEmpty()) continue;

        qDebug() << "  Processing paused group with representative ID:" << repId << "for frame" << frameNumber;

        QSet<int> idsInOriginalMergeGroup;
        if (maps.mergedGroups.contains(repId)) {
            idsInOriginalMergeGroup = maps.mergedGroups.value(repId);
        } else {
            // This repId represents a single worm that was likely merged with an untracked entity.
            // The group only contains this one worm.
            for(const PausedTrackerInfo& pInfo : groupPausedList) { // Should typically be one pInfo if repId was pInfo.wormId
                idsInOriginalMergeGroup.insert(pInfo.wormId);
            }
        }
        if(idsInOriginalMergeGroup.isEmpty()){
            qWarning() << "  Rep ID" << repId << "has no members in mergedGroups map. This is unexpected if it's from wormToMergeGroupMap. Skipping.";
            continue;
        }


        qDebug() << "    Original merge group members for rep ID" << repId << ":" << idsInOriginalMergeGroup;

        // Collect all unique reported blobs from all trackers in this paused group
        QList<Tracking::DetectedBlob> allReportedBlobsForGroup;
        QSet<QPair<qreal, qreal>> seenBlobCentroids; // To crudely deduplicate blobs by centroid

        for (const PausedTrackerInfo& pInfo : groupPausedList) {
            for (const Tracking::DetectedBlob& blob : pInfo.reportedBlobs) {
                QPair<qreal, qreal> centroidPair(blob.centroid.x(), blob.centroid.y());
                if (blob.isValid && !seenBlobCentroids.contains(centroidPair)) {
                    allReportedBlobsForGroup.append(blob);
                    seenBlobCentroids.insert(centroidPair);
                }
            }
        }

        if (allReportedBlobsForGroup.isEmpty()) {
            qWarning() << "    Group" << repId << "paused but collectively reported no valid blobs. Stopping involved trackers.";
            for (int wormIdInGroup : idsInOriginalMergeGroup) {
                if (m_wormObjectsMap.contains(wormIdInGroup)) {
                    m_wormObjectsMap[wormIdInGroup]->setState(WormObject::TrackingState::Lost);
                }
                QList<WormTracker*> trackersToStop = findTrackersForWorm(wormIdInGroup);
                for (WormTracker* tr : trackersToStop) {
                    if (tr && ((isForwardContext && tr->getDirection() == WormTracker::TrackingDirection::Forward) ||
                               (!isForwardContext && tr->getDirection() == WormTracker::TrackingDirection::Backward))) {
                        // Stop if it's one of the paused trackers or was part of this merge context
                        bool partOfThisPausedEvent = false;
                        for(const PausedTrackerInfo& pInfoCheck : groupPausedList) { if(pInfoCheck.trackerInstance == tr) { partOfThisPausedEvent = true; break;} }

                        if(partOfThisPausedEvent || (maps.wormToMergeGroupMap.value(tr->getWormId(), -1) == repId) ) {
                            qDebug() << "      Stopping tracker" << tr << "for worm" << tr->getWormId() << "due to no blobs from split.";
                            QMetaObject::invokeMethod(tr, "stopTracking", Qt::QueuedConnection);
                        }
                    }
                }
            }
            // Dissolve the merge group from maps
            if (maps.mergedGroups.contains(repId)) maps.mergedGroups.remove(repId);
            for (int wormIdInGroup : idsInOriginalMergeGroup) maps.wormToMergeGroupMap.remove(wormIdInGroup);
            qDebug() << "    Dissolved merge group" << repId << " (no blobs from split).";
            continue; // Next group
        }

        qDebug() << "    Group" << repId << "collectively reported" << allReportedBlobsForGroup.size() << "unique blobs.";

        // --- Assignment Logic ---
        QMap<int, Tracking::DetectedBlob> assignments; // wormId -> assignedBlob
        QList<Tracking::DetectedBlob> availableBlobs = allReportedBlobsForGroup;
        QList<int> assignedBlobIndicesInAvailableList; // Indices into 'availableBlobs'

        // Create a list of (wormId, lastKnownPosition) for worms in this specific original merge group
        QList<QPair<int, QPointF>> wormsToAssign;
        for (int wormId : idsInOriginalMergeGroup) {
            WormObject* wo = m_wormObjectsMap.value(wormId, nullptr);
            if (wo) {
                wormsToAssign.append({wormId, QPointF(wo->getCurrentPosition().x, wo->getCurrentPosition().y)});
            } else {
                qWarning() << "    WormObject not found for ID" << wormId << "in group" << repId << ". Cannot get last position.";
            }
        }
        // Optional: Sort wormsToAssign (e.g., by ID, or by some priority if applicable)
        std::sort(wormsToAssign.begin(), wormsToAssign.end(), [](const QPair<int, QPointF>& a, const QPair<int, QPointF>& b){
            return a.first < b.first; // Sort by worm ID for deterministic assignment in ties
        });


        for (const auto& wormEntry : wormsToAssign) {
            int wormIdToAssign = wormEntry.first;
            QPointF lastPos = wormEntry.second;
            double minSqDist = std::numeric_limits<double>::max();
            int bestBlobListIdx = -1; // Index in 'availableBlobs'

            for (int i = 0; i < availableBlobs.size(); ++i) {
                if (assignedBlobIndicesInAvailableList.contains(i)) continue; // Blob already assigned

                const Tracking::DetectedBlob& currentBlob = availableBlobs.at(i);
                if (!currentBlob.isValid) continue;

                double sqDist = squaredDistance(lastPos, currentBlob.centroid);
                if (sqDist < minSqDist) {
                    minSqDist = sqDist;
                    bestBlobListIdx = i;
                }
            }

            if (bestBlobListIdx != -1) {
                // Check if this distance is reasonable (e.g. not excessively far)
                // double MAX_ASSIGNMENT_DISTANCE_SQ = 200.0 * 200.0; // Example: 200 pixels
                // if (minSqDist > MAX_ASSIGNMENT_DISTANCE_SQ) {
                //     qDebug() << "    Best blob for worm" << wormIdToAssign << "is too far (dist_sq:" << minSqDist << "). Marking as lost.";
                //     // Fall through to "not assigned" logic
                // } else {
                assignments[wormIdToAssign] = availableBlobs.at(bestBlobListIdx);
                assignedBlobIndicesInAvailableList.append(bestBlobListIdx); // Mark this index as used
                qDebug() << "      Assigned blob (Centroid:" << availableBlobs.at(bestBlobListIdx).centroid
                         << ", Area:" << availableBlobs.at(bestBlobListIdx).area
                         << ") to worm" << wormIdToAssign << "from group" << repId;
                // }
            }
            // If bestBlobListIdx remains -1, this worm won't get an assignment in this pass.
        }


        // --- Instruct Trackers and Update States ---
        bool allOriginalMembersAccountedFor = true;
        for (int wormIdInGroup : idsInOriginalMergeGroup) {
            WormObject* wo = m_wormObjectsMap.value(wormIdInGroup, nullptr);
            if (!wo) continue;

            if (assignments.contains(wormIdInGroup)) {
                Tracking::DetectedBlob assignedBlob = assignments.value(wormIdInGroup);
                wo->setState(WormObject::TrackingState::Tracking);

                // Find the specific tracker instances that were paused for this wormId in this group
                bool resumedRelevantTracker = false;
                for (const PausedTrackerInfo& pInfo : groupPausedList) {
                    if (pInfo.trackerInstance && pInfo.wormId == wormIdInGroup) {
                        qDebug() << "      Instructing Tracker" << pInfo.trackerInstance << "for worm" << wormIdInGroup
                                 << "(Dir:" << pInfo.trackerInstance->getDirection() << ") to RESUME with blob at" << assignedBlob.centroid;
                        QMetaObject::invokeMethod(pInfo.trackerInstance, "resumeTrackingWithNewTarget", Qt::QueuedConnection,
                                                  Q_ARG(Tracking::DetectedBlob, assignedBlob));
                        resumedRelevantTracker = true;
                        // break; // Assuming only one tracker per wormId per direction would be in groupPausedList
                    }
                }
                if (!resumedRelevantTracker) {
                    qDebug() << "      Worm" << wormIdInGroup << "was assigned a blob, but its paused tracker instance was not found in the groupPausedList to resume. This is unexpected.";
                    allOriginalMembersAccountedFor = false; // Or handle as an error
                }

            } else { // Worm from original group was not assigned a blob
                qDebug() << "    Worm" << wormIdInGroup << "from group" << repId << "was not assigned a blob from the split. Marking as lost.";
                wo->setState(WormObject::TrackingState::Lost);
                allOriginalMembersAccountedFor = false; // Not all original members got a new target

                // Stop any of its trackers that were part of this paused event or still associated with this merge group
                for (const PausedTrackerInfo& pInfo : groupPausedList) {
                    if (pInfo.trackerInstance && pInfo.wormId == wormIdInGroup) {
                        qDebug() << "      Stopping (unassigned) paused tracker" << pInfo.trackerInstance << "for worm" << wormIdInGroup;
                        QMetaObject::invokeMethod(pInfo.trackerInstance, "stopTracking", Qt::QueuedConnection);
                    }
                }
                // Also stop any other relevant trackers for this wormId that might still think they are in this merge
                QList<WormTracker*> otherTrackers = findTrackersForWorm(wormIdInGroup);
                for(WormTracker* tr : otherTrackers) {
                    bool wasThisPausedInstance = false;
                    for(const PausedTrackerInfo& pInfoCheck : groupPausedList) { if(pInfoCheck.trackerInstance == tr && pInfoCheck.wormId == wormIdInGroup) { wasThisPausedInstance = true; break;} }
                    if(tr && !wasThisPausedInstance && maps.wormToMergeGroupMap.value(tr->getWormId(), -1) == repId &&
                        ((isForwardContext && tr->getDirection() == WormTracker::TrackingDirection::Forward) ||
                         (!isForwardContext && tr->getDirection() == WormTracker::TrackingDirection::Backward))) {
                        qDebug() << "      Stopping (unassigned) associated tracker" << tr << "for worm" << wormIdInGroup;
                        QMetaObject::invokeMethod(tr, "stopTracking", Qt::QueuedConnection);
                    }
                }
            }
        }

        // Dissolve the merge group from maps if all original members were accounted for (resumed or lost explicitly by this process)
        // This means the specific merge event represented by 'repId' has been fully resolved.
        if (allOriginalMembersAccountedFor || idsInOriginalMergeGroup.size() == assignments.size()) { // Simpler check for now
            if (maps.mergedGroups.contains(repId)) {
                maps.mergedGroups.remove(repId);
            }
            for (int idInGroup : idsInOriginalMergeGroup) {
                maps.wormToMergeGroupMap.remove(idInGroup);
            }
            qDebug() << "    Dissolved merge group" << repId << "after processing split.";
        } else {
            qDebug() << "    Merge group" << repId << "not fully dissolved. Some members may need further attention or were not assigned.";
            // This case needs careful thought: if some worms from the original group were not assigned
            // and their trackers weren't in the pausedThisFrame list, the group might persist partially.
            // The current logic tries to stop trackers for unassigned worms from the original group.
        }
    } // End loop over groupedPausedTrackers

    if (!m_pausedTrackerCache.contains(frameNumber) || m_pausedTrackerCache.value(frameNumber).isEmpty()) {
        qDebug() << "TM: Paused tracker cache for frame" << frameNumber << "is now empty after resolution.";
    } else {
        // This should not happen if we .take() the list and process all groups.
        qWarning() << "TM: Paused tracker cache for frame" << frameNumber << "still has"
                   << m_pausedTrackerCache.value(frameNumber).count() << "items after resolution. This is unexpected.";
    }
}



void TrackingManager::handleWormStateChanged(int reportingWormId, WormTracker::TrackerState newState, int associatedEntityId) {
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormObject* wormObject = m_wormObjectsMap.value(reportingWormId, nullptr);
    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender());

    if (reportingTracker) {
        qDebug() << "TM: Worm ID" << reportingWormId
                 << "(Tracker:" << reportingTracker << ", Dir:" << reportingTracker->getDirection()
                 << ") State CHANGED to" << static_cast<int>(newState)
                 << (associatedEntityId != -1 ? QString(" (Assoc. ID: %1)").arg(associatedEntityId) : QString(""));

        if (wormObject) {
            // Update WormObject's high-level state based on tracker's more detailed state
            if (newState == WormTracker::TrackerState::Idle && wormObject->getCurrentState() != WormObject::TrackingState::Lost) { // e.g. tracker stopped before starting well
                // Could mark as Inactive or Lost depending on context
            } else if (newState == WormTracker::TrackerState::TrackingMerged) {
                // WormObject state is likely already Merged due to processFrameDataForMergesAndSplits
                // but ensure associatedEntityId is updated if provided.
                if (associatedEntityId != -1 && wormObject->getCurrentState() == WormObject::TrackingState::Merged) {
                    // wormObject->setMergedWithId(associatedEntityId); // Already done by processFrameData...
                }
            }
            // Add more mappings if necessary
        }
    } else {
        qWarning() << "TM: handleWormStateChanged from non-WormTracker sender for worm ID:" << reportingWormId;
    }
}

void TrackingManager::handleWormTrackerFinished() {
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if (!finishedTracker) {
        qWarning() << "TrackingManager: handleWormTrackerFinished called by non-WormTracker sender.";
        // This could potentially lead to m_finishedTrackersCount not matching m_expectedTrackersToFinish
        // if a tracker finishes without being in m_wormTrackers.
        // However, normal flow should have it in m_wormTrackers.
        m_finishedTrackersCount++; // Increment defensively
        checkForAllTrackersFinished();
        return;
    }

    int wormId = finishedTracker->getWormId();
    QString direction = (finishedTracker->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd");
    qDebug() << "TrackingManager: WormTracker for worm ID" << wormId
             << "(Dir:" << direction << ", Ptr: " << finishedTracker << ") reported finished.";

    // Remove from active lists. It will be deleteLater'd by its thread's finished signal.
    m_wormTrackers.removeOne(finishedTracker);
    m_individualTrackerProgress.remove(finishedTracker);

    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: Total finished trackers:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;

    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerError(int reportingWormId, QString errorMessage) {
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    QString trackerInfo = errorTracker ? QString("(Ptr: %1, Dir: %2)").arg(reinterpret_cast<quintptr>(errorTracker)).arg(errorTracker->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd") : "";

    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Error from tracker for worm ID" << reportingWormId << trackerInfo << "during/after cancel/stop:" << errorMessage;
        // Tracker will finish, let normal flow handle it. If it's not already counted, count it.
        if (errorTracker && m_wormTrackers.contains(errorTracker)) { // Only if it was an active tracker
            m_wormTrackers.removeOne(errorTracker);
            m_individualTrackerProgress.remove(errorTracker);
            m_finishedTrackersCount++;
        } else if (!errorTracker && !m_wormTrackers.isEmpty()){ // Sender unknown, but trackers might be running
            // This case is problematic.
        } else if (!errorTracker && m_wormTrackers.isEmpty() && m_finishedTrackersCount < m_expectedTrackersToFinish) {
            // If sender is null and no active trackers, but not all expected are finished, increment count.
            // m_finishedTrackersCount++; // This might be risky if errors come from other sources
        }
        updateOverallProgress();
        checkForAllTrackersFinished();
        return;
    }
    if (!m_isTrackingRunning) {
        qDebug() << "TrackingManager: Error from tracker for worm ID" << reportingWormId << trackerInfo << "but tracking not running:" << errorMessage;
        return;
    }

    qWarning() << "TrackingManager: Error from tracker for worm ID" << reportingWormId << trackerInfo << ":" << errorMessage;

    if (errorTracker && m_wormTrackers.contains(errorTracker)) {
        m_wormTrackers.removeOne(errorTracker);
        m_individualTrackerProgress.remove(errorTracker);
        m_finishedTrackersCount++; // Count it as "finished" (with error)
    } else if (!errorTracker && m_expectedTrackersToFinish > m_finishedTrackersCount) {
        // If sender is null but we expect more trackers, this is an issue.
        // We might increment finished count to avoid stalling, but it's a guess.
        // m_finishedTrackersCount++;
        qWarning() << "  Error from unknown sender, but trackers still expected. Process might stall.";
    }


    emit trackingStatusUpdate(QString("Error with tracker for worm %1: %2. Trying to continue...").arg(reportingWormId).arg(errorMessage.left(50)));
    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerProgress(int wormId, int percentDone) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    WormTracker* tracker = qobject_cast<WormTracker*>(sender());
    if (tracker) {
        if (m_individualTrackerProgress.contains(tracker)) { // Check if still active
            m_individualTrackerProgress[tracker] = percentDone;
            updateOverallProgress();
        }
    }
}

// --- Private Helper Methods ---
void TrackingManager::launchWormTrackers() {
    if (m_initialWormInfos.empty()) {
        emit trackingStatusUpdate("No worms selected for tracking.");
        // cleanupThreadsAndObjects(); // No threads to clean yet
        emit trackingFinishedSuccessfully(""); // Or trackingFailed if this is an error condition
        m_isTrackingRunning = false;
        return;
    }

    m_expectedTrackersToFinish = 0; // Recalculate based on actual launches
    m_finishedTrackersCount = 0;
    m_individualTrackerProgress.clear();


    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        // Forward Tracker
        // Launch if there are forward frames OR if the keyframe is the very last frame (no fwd frames, but tracker should "finish" immediately)
        if (!m_processedForwardFrames.empty() || m_keyFrameNum == m_totalFramesInVideo - 1) {
            WormTracker* fwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
            fwdTracker->setFrames(&m_processedForwardFrames); // Pass pointer to frames
            QThread* fwdThread = new QThread(this); // Parent to TrackingManager
            fwdTracker->moveToThread(fwdThread);
            // Set properties for easier debugging/identification if needed
            fwdTracker->setProperty("wormId", wormId);
            fwdTracker->setProperty("direction", "Forward");

            connect(fwdThread, &QThread::started, fwdTracker, &WormTracker::startTracking);
            connect(fwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(fwdThread, &QThread::finished, fwdTracker, &QObject::deleteLater); // Delete tracker when thread done
            connect(fwdThread, &QThread::finished, this, [this, fwdThread, wormId](){
                m_trackerThreads.removeOne(fwdThread); // Remove from our list
                // fwdThread itself is deleted in cleanupThreadsAndObjects or if this is the last one
                qDebug() << "Forward Tracker Thread for worm" << wormId << "finished and removed from list. Remaining tracker threads:" << m_trackerThreads.count();
            });

            connect(fwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
            connect(fwdTracker, &WormTracker::splitDetectedAndPaused, this, &TrackingManager::handleWormSplitDetectedAndPaused);
            connect(fwdTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
            connect(fwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(fwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);

            m_wormTrackers.append(fwdTracker);
            m_trackerThreads.append(fwdThread);
            m_individualTrackerProgress[fwdTracker] = 0;
            fwdThread->start();
            m_expectedTrackersToFinish++;
        } else {
            qDebug() << "TM: Skipping Forward Tracker for worm" << wormId << "due to no forward frames and keyframe not being the last frame.";
        }

        // Backward Tracker
        // Launch if there are reverse frames OR if the keyframe is the very first frame
        if (!m_processedReversedFrames.empty() || m_keyFrameNum == 0) {
            WormTracker* bwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
            bwdTracker->setFrames(&m_processedReversedFrames);
            QThread* bwdThread = new QThread(this);
            bwdTracker->moveToThread(bwdThread);
            bwdTracker->setProperty("wormId", wormId);
            bwdTracker->setProperty("direction", "Backward");

            connect(bwdThread, &QThread::started, bwdTracker, &WormTracker::startTracking);
            connect(bwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(bwdThread, &QThread::finished, bwdTracker, &QObject::deleteLater);
            connect(bwdThread, &QThread::finished, this, [this, bwdThread, wormId](){
                m_trackerThreads.removeOne(bwdThread);
                qDebug() << "Backward Tracker Thread for worm" << wormId << "finished and removed from list. Remaining tracker threads:" << m_trackerThreads.count();
            });

            connect(bwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
            connect(bwdTracker, &WormTracker::splitDetectedAndPaused, this, &TrackingManager::handleWormSplitDetectedAndPaused);
            connect(bwdTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
            connect(bwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(bwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);

            m_wormTrackers.append(bwdTracker);
            m_trackerThreads.append(bwdThread);
            m_individualTrackerProgress[bwdTracker] = 0;
            bwdThread->start();
            m_expectedTrackersToFinish++;
        } else {
            qDebug() << "TM: Skipping Backward Tracker for worm" << wormId << "due to no backward frames and keyframe not being the first frame.";
        }
    }

    if (m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) {
        qWarning() << "TM: No trackers were launched despite having initial worm infos. Check keyframe position and frame processing.";
        emit trackingFailed("No trackers could be launched. Video might be too short or keyframe at an extreme end where no tracking frames are available.");
        m_isTrackingRunning = false;
        // cleanupThreadsAndObjects(); // No threads to clean
    } else if (m_expectedTrackersToFinish > 0) {
        emit trackingStatusUpdate(QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));
    }
    // If m_initialWormInfos is empty, it's handled at the start of the function.
}


QList<WormTracker*> TrackingManager::findTrackersForWorm(int conceptualWormId) {
    QList<WormTracker*> foundTrackers;
    // Iterate over a const copy or use a mutex if m_wormTrackers can be modified concurrently
    // For now, assuming modifications happen on the main thread or are synchronized.
    for (WormTracker* tracker : std::as_const(m_wormTrackers)) {
        if (tracker && tracker->getWormId() == conceptualWormId) {
            foundTrackers.append(tracker);
        }
    }
    return foundTrackers;
}

void TrackingManager::updateOverallProgress() {
    if (!m_isTrackingRunning && !m_cancelRequested) {
        emit overallTrackingProgress(0); // Reset if not running and not in a final cancel stage
        return;
    }

    double totalProgressValue = 0.0;
    double videoProcWeight = 0.10; // Video processing is 10%
    double trackersWeight = 0.90;  // Trackers are 90%

    totalProgressValue += (static_cast<double>(m_videoProcessingProgress) / 100.0) * videoProcWeight;

    if (m_expectedTrackersToFinish > 0) {
        double sumOfIndividualProgress = 0;
        // Sum progress of trackers still in m_individualTrackerProgress (i.e., active)
        for (int progress : m_individualTrackerProgress.values()) {
            sumOfIndividualProgress += progress;
        }
        // Add 100% for each tracker that has already finished
        double finishedContribution = static_cast<double>(m_finishedTrackersCount) * 100.0;
        // Total points achieved by trackers so far
        double currentTrackerPoints = sumOfIndividualProgress + finishedContribution;
        // Maximum possible points from all expected trackers
        double maxTrackerPoints = static_cast<double>(m_expectedTrackersToFinish) * 100.0;

        double overallTrackerPercentage = (maxTrackerPoints > 0) ? (currentTrackerPoints / maxTrackerPoints) : 0.0;
        overallTrackerPercentage = qBound(0.0, overallTrackerPercentage, 1.0); // Normalize to 0-1 range

        totalProgressValue += overallTrackerPercentage * trackersWeight;
    } else if (m_videoProcessingProgress == 100) { // No trackers expected, video processing is 100% of task if it's done
        totalProgressValue = 1.0; // Full progress
    }

    int finalProgressPercentage = qBound(0, static_cast<int>(totalProgressValue * 100.0), 100);
    emit overallTrackingProgress(finalProgressPercentage);
}


void TrackingManager::checkForAllTrackersFinished() {
    qDebug() << "TM: Checking if all trackers finished. Finished:" << m_finishedTrackersCount
             << "Expected:" << m_expectedTrackersToFinish << "Running:" << m_isTrackingRunning
             << "Trackers in list:" << m_wormTrackers.count();

    // Condition: Tracking is marked as running AND (either all expected trackers have signaled finished OR the list of active trackers is empty)
    if (m_isTrackingRunning && (m_finishedTrackersCount >= m_expectedTrackersToFinish || (m_expectedTrackersToFinish > 0 && m_wormTrackers.isEmpty()))) {
        if (m_cancelRequested) {
            qDebug() << "TrackingManager: All trackers accounted for after CANCELLATION request.";
            // Consolidate whatever tracks were gathered
            m_finalTracks.clear();
            for (WormObject* worm : m_wormObjectsMap.values()) { // Iterate values
                if (worm) {
                    m_finalTracks[worm->getId()] = worm->getTrackHistory();
                }
            }
            if (!m_finalTracks.empty()) {
                qDebug() << "TrackingManager: Emitting partial tracks due to cancellation. Count:" << m_finalTracks.size();
                emit allTracksUpdated(m_finalTracks);
            } else {
                qDebug() << "TrackingManager: No tracks to emit upon cancellation.";
            }
            emit trackingStatusUpdate("Tracking cancelled by user. Partial tracks (if any) processed.");
            emit trackingCancelled();
        } else { // Normal finish
            qDebug() << "TrackingManager: All trackers accounted for NORMALLY.";
            emit trackingStatusUpdate("All worm trackers completed. Consolidating and saving tracks...");
            m_finalTracks.clear();
            for (WormObject* worm : m_wormObjectsMap.values()) {
                if (worm) {
                    m_finalTracks[worm->getId()] = worm->getTrackHistory();
                }
            }
            emit allTracksUpdated(m_finalTracks);

            QString csvOutputPath;
            if (!m_videoPath.isEmpty()) {
                QFileInfo videoInfo(m_videoPath);
                csvOutputPath = QDir(videoInfo.absolutePath()).filePath(videoInfo.completeBaseName() + "_tracks.csv");
            } else {
                csvOutputPath = "worm_tracks.csv"; // Default
            }

            if (outputTracksToCsv(m_finalTracks, csvOutputPath)) {
                emit trackingStatusUpdate("Tracking finished. Tracks saved to: " + csvOutputPath);
                emit trackingFinishedSuccessfully(csvOutputPath);
            } else {
                emit trackingStatusUpdate("Tracking finished, but failed to save CSV output to: " + csvOutputPath);
                emit trackingFailed("Failed to save CSV output to: " + csvOutputPath);
            }
        }
        m_isTrackingRunning = false; // Mark tracking as no longer running
        // cleanupThreadsAndObjects(); // Call cleanup explicitly here to ensure all resources are released promptly.
        // This is important if TM instance is long-lived.
        // However, if TM is destroyed soon after, destructor handles it.
        // For now, let destructor or next start handle it to avoid issues if signals are still pending.
    } else if (!m_isTrackingRunning) {
        qDebug() << "TM: checkForAllTrackersFinished - Tracking was already stopped (e.g., by error or earlier cancel).";
        if (m_cancelRequested) { // If a cancel was requested and tracking is now stopped
            emit trackingCancelled(); // Ensure this is emitted if not already
        }
    }
    // If m_isTrackingRunning is true but not all trackers are finished, do nothing yet.
}


bool TrackingManager::outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFilePath) const {
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
    outStream << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight\n"; // Header
    for (auto const& [wormId, trackPoints] : tracks) { // C++17 structured binding
        for (const Tracking::WormTrackPoint& point : trackPoints) {
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
    if (csvFile.error() != QFile::NoError) { // Check error after closing
        qWarning() << "Error writing to CSV file:" << outputFilePath << csvFile.errorString();
        return false;
    }
    qDebug() << "Tracks successfully written to CSV:" << outputFilePath;
    return true;
}
