// trackingmanager.cpp
#include "trackingmanager.h"
#include <QDebug>
#include <numeric>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QFileInfo>
#include <set>      // For std::set if needed, QSet is generally fine for Qt int IDs
#include <algorithm> // For std::min, std::sort etc.

TrackingManager::TrackingManager(QObject *parent)
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
    m_videoProcessingProgress(0)
{
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<WormObject::TrackingState>("WormObject::TrackingState");
    qRegisterMetaType<AllWormTracks>("AllWormTracks");
    qRegisterMetaType<QList<TrackingHelper::DetectedBlob>>("QList<TrackingHelper::DetectedBlob>"); // For the new signal
}

TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking();
        QThread::msleep(100); // Brief pause to allow signals to process
    }
    cleanupThreadsAndObjects();
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - FINISHED cleaning up.";
}

void TrackingManager::startFullTrackingProcess(const QString& videoPath,
                                               int keyFrameNum,
                                               const std::vector<InitialWormInfo>& initialWorms,
                                               const ThresholdSettings& settings,
                                               int totalFramesInVideo) {
    qDebug() << "TrackingManager (" << this << "): startFullTrackingProcess called.";
    if (m_isTrackingRunning) {
        qWarning() << "TrackingManager: Attempted to start tracking while already running.";
        emit trackingFailed("Another tracking process is already running.");
        return;
    }
    cleanupThreadsAndObjects(); // CRITICAL: Clean slate before starting
    qDebug() << "TrackingManager (" << this << "): Post-cleanup in start, m_videoProcessor is" << m_videoProcessor << "m_videoProcessorThread is" << m_videoProcessorThread;

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
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameInfos.clear();
    m_mergedGroups.clear();
    m_wormToMergeGroupMap.clear();

    emit trackingStatusUpdate("Starting video processing...");
    emit overallTrackingProgress(0);

    m_videoProcessor = new VideoProcessor();
    m_videoProcessorThread = new QThread(this); // Parent to this for potential auto-cleanup
    m_videoProcessor->moveToThread(m_videoProcessorThread);
    qDebug() << "TrackingManager: Created new VideoProcessor (" << m_videoProcessor << ") and QThread (" << m_videoProcessorThread << ")";

    connect(m_videoProcessorThread, &QThread::started, m_videoProcessor, [this](){
        if(m_videoProcessor) {
            m_videoProcessor->startInitialProcessing(m_videoPath, m_keyFrameNum, m_thresholdSettings, m_totalFramesInVideo);
        }
    });
    connect(m_videoProcessor, &VideoProcessor::initialProcessingComplete, this, &TrackingManager::handleInitialProcessingComplete);
    connect(m_videoProcessor, &VideoProcessor::processingError, this, &TrackingManager::handleVideoProcessingError);
    connect(m_videoProcessor, &VideoProcessor::initialProcessingProgress, this, &TrackingManager::handleVideoProcessingProgress);
    connect(m_videoProcessor, &VideoProcessor::processingStarted, this, [this](){ emit trackingStatusUpdate("Video processing started in thread."); });
    connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor, &QObject::deleteLater, Qt::UniqueConnection);
    connect(m_videoProcessorThread, &QThread::finished, [this](){
        qDebug() << "VideoProcessorThread (" << QObject::sender() << ") finished. m_videoProcessor (" << m_videoProcessor << ") was scheduled for deleteLater.";
        m_videoProcessor = nullptr;
    });
    m_videoProcessorThread->start();
}

void TrackingManager::cancelTracking() {
    // ... (Implementation as in previous version, ensuring m_cancelRequested is set) ...
    qDebug() << "TrackingManager (" << this << "): cancelTracking called. IsRunning:" << m_isTrackingRunning << "CancelRequested:" << m_cancelRequested;
    if (!m_isTrackingRunning && !m_cancelRequested) { qDebug() << "TM: Cancel requested but not running or already cancelled."; return; }
    if (m_cancelRequested) { qDebug() << "TM: Cancellation already in progress."; return; }
    m_cancelRequested = true; emit trackingStatusUpdate("Cancellation requested...");
    qDebug() << "TM: Cancellation flag set to true.";
    if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
        qDebug() << "TM: Requesting video processor thread (" << m_videoProcessorThread << ") interruption.";
        m_videoProcessorThread->requestInterruption();
    }
    for (WormTracker* tracker : std::as_const(m_wormTrackers)) {
        if (tracker) QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
    }
    for (QThread* thread : std::as_const(m_trackerThreads)) {
        if (thread && thread->isRunning()) thread->requestInterruption();
    }
}

void TrackingManager::cleanupThreadsAndObjects() {
    // ... (Implementation as in previous robust version, ensuring threads are quit, waited for, deleted, and pointers nulled) ...
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - START";
    if (m_videoProcessorThread) {
        qDebug() << "  Cleaning up video processor thread:" << m_videoProcessorThread;
        if (m_videoProcessorThread->isRunning()) {
            m_videoProcessorThread->requestInterruption(); m_videoProcessorThread->quit();
            if (!m_videoProcessorThread->wait(1000)) qWarning() << "  VideoProcessor thread didn't finish gracefully.";
            else qDebug() << "  Video processor thread finished gracefully.";
        } delete m_videoProcessorThread; m_videoProcessorThread = nullptr;
    }
    if (m_videoProcessor) { /* m_videoProcessor is deleteLater'd by thread */ m_videoProcessor = nullptr; }

    for (QThread* thread : m_trackerThreads) {
        if (thread) {
            if (thread->isRunning()) {
                thread->requestInterruption(); thread->quit();
                if (!thread->wait(500)) qWarning() << "  A WormTracker thread (" << thread << ") didn't finish gracefully.";
                else qDebug() << "  Tracker thread (" << thread << ") finished gracefully.";
            } delete thread;
        }
    }
    m_trackerThreads.clear();
    m_wormTrackers.clear(); // Trackers are deleteLater'd by their threads finishing

    qDeleteAll(m_wormObjectsMap); m_wormObjectsMap.clear();
    m_processedForwardFrames.clear(); std::vector<cv::Mat>().swap(m_processedForwardFrames);
    m_processedReversedFrames.clear(); std::vector<cv::Mat>().swap(m_processedReversedFrames);
    m_finalTracks.clear(); m_individualTrackerProgress.clear(); m_frameInfos.clear();
    m_mergedGroups.clear(); m_wormToMergeGroupMap.clear();
    m_isTrackingRunning = false; m_cancelRequested = false;
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - FINISHED";
}

void TrackingManager::handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                                      const std::vector<cv::Mat>& reversedFrames,
                                                      double fps,
                                                      cv::Size frameSize) {
    // ... (Implementation as in previous robust version, ensuring m_videoProcessor is nulled after its thread is done with it) ...
    qDebug() << "TrackingManager (" << this << "): handleInitialProcessingComplete received.";
    if (!m_videoProcessor && !m_cancelRequested) { // Check if processor got deleted early
        qWarning() << "TM: handleInitialProcessingComplete, but m_videoProcessor is null. Race condition or premature deletion?";
    }
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing complete but cancellation requested. Not launching trackers.";
        emit trackingCancelled(); m_isTrackingRunning = false; return;
    }
    qDebug() << "TM: Initial video processing complete. Fwd:" << forwardFrames.size() << "Rev:" << reversedFrames.size();
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    m_videoProcessingProgress = 100;
    m_processedForwardFrames = forwardFrames; m_processedReversedFrames = reversedFrames;
    m_videoFps = fps; m_videoFrameSize = frameSize;
    // m_videoProcessor is handled by its thread's finished signal (deleteLater and nulling our pointer)
    for (const auto& info : m_initialWormInfos) {
        if (m_wormObjectsMap.contains(info.id)) continue;
        m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
    }
    launchWormTrackers(); updateOverallProgress();
}

void TrackingManager::handleVideoProcessingError(const QString& errorMessage) { /* ... as before ... */
    qDebug() << "TrackingManager (" << this << "): handleVideoProcessingError: " << errorMessage;
    if (m_cancelRequested) { emit trackingCancelled(); m_isTrackingRunning = false; return; }
    qWarning() << "TM: Video processing error:" << errorMessage;
    emit trackingFailed("Video processing failed: " + errorMessage);
    m_isTrackingRunning = false; cleanupThreadsAndObjects();
}
void TrackingManager::handleVideoProcessingProgress(int percentage) { /* ... as before ... */
    if (m_cancelRequested) return; m_videoProcessingProgress = percentage; updateOverallProgress();
}

void TrackingManager::launchWormTrackers() {
    // ... (Implementation as in previous robust version, ensuring threads are parented to `this`
    //      and worker objects are connected to `deleteLater` on their thread's `finished` signal.
    //      Also, connect the new/updated signals from WormTracker.)
    if (m_initialWormInfos.empty()) { /* ... handle no worms ... */ return; }
    m_expectedTrackersToFinish = static_cast<int>(m_initialWormInfos.size() * 2);
    m_finishedTrackersCount = 0; m_individualTrackerProgress.clear();
    emit trackingStatusUpdate(QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));

    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id; QRectF initialRoi = info.initialRoi;
        // Forward
        WormTracker* fwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
        fwdTracker->setFrames(&m_processedForwardFrames);
        QThread* fwdThread = new QThread(this); fwdTracker->moveToThread(fwdThread); fwdTracker->setProperty("wormId", wormId);
        connect(fwdThread, &QThread::started, fwdTracker, &WormTracker::startTracking);
        connect(fwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
        connect(fwdThread, &QThread::finished, fwdTracker, &QObject::deleteLater);
        connect(fwdThread, &QThread::finished, [this, fwdTracker, fwdThread](){ m_wormTrackers.removeOne(fwdTracker); /* Thread deleted in cleanup */ });
        connect(fwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(fwdTracker, &WormTracker::splitDetectedAndPaused, this, &TrackingManager::handleWormSplitDetectedAndPaused); // New connection
        connect(fwdTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(fwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(fwdTracker, &WormTracker::progress, this, [this, fwdTracker](int,int p){ if(m_cancelRequested)return; m_individualTrackerProgress[fwdTracker]=p; updateOverallProgress(); });
        m_wormTrackers.append(fwdTracker); m_trackerThreads.append(fwdThread); m_individualTrackerProgress[fwdTracker]=0; fwdThread->start();

        // Backward
        WormTracker* bwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
        bwdTracker->setFrames(&m_processedReversedFrames);
        QThread* bwdThread = new QThread(this); bwdTracker->moveToThread(bwdThread); bwdTracker->setProperty("wormId", wormId);
        connect(bwdThread, &QThread::started, bwdTracker, &WormTracker::startTracking);
        connect(bwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
        connect(bwdThread, &QThread::finished, bwdTracker, &QObject::deleteLater);
        connect(bwdThread, &QThread::finished, [this, bwdTracker, bwdThread](){ m_wormTrackers.removeOne(bwdTracker); /* Thread deleted in cleanup */ });
        connect(bwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(bwdTracker, &WormTracker::splitDetectedAndPaused, this, &TrackingManager::handleWormSplitDetectedAndPaused); // New connection
        connect(bwdTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(bwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(bwdTracker, &WormTracker::progress, this, [this, bwdTracker](int,int p){ if(m_cancelRequested)return; m_individualTrackerProgress[bwdTracker]=p; updateOverallProgress(); });
        m_wormTrackers.append(bwdTracker); m_trackerThreads.append(bwdThread); m_individualTrackerProgress[bwdTracker]=0; bwdThread->start();
    }
}

// Updated Slot
void TrackingManager::handleWormPositionUpdated(int wormId,
                                                int originalFrameNumber,
                                                QPointF newPosition,
                                                QRectF newRoi,
                                                int plausibleBlobsFoundInRoi,
                                                double primaryBlobArea) {
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormObject* wormObject = m_wormObjectsMap.value(wormId, nullptr);
    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender());

    if (wormObject && reportingTracker) {
        // Always update the WormObject with the tracker's primary finding if not paused
        if (reportingTracker->property("currentState").value<WormTracker::TrackerState>() != WormTracker::TrackerState::PausedAwaitingSplitDecision) {
            // Only update if this worm is NOT currently part of a centrally managed merge group
            // OR if this update IS the centrally decided position for a merge.
            // For now, let's assume if a worm is in m_wormToMergeGroupMap, its position is dictated elsewhere.
            if (!m_wormToMergeGroupMap.contains(wormId)) {
                cv::Point2f cvPos(static_cast<float>(newPosition.x()), static_cast<float>(newPosition.y()));
                wormObject->updateTrackPoint(originalFrameNumber, cvPos, newRoi);
                WormTrackPoint lastPt;
                lastPt.frameNumberOriginal = originalFrameNumber;
                lastPt.position = cvPos;
                lastPt.roi = newRoi;
                emit individualWormTrackUpdated(wormId, lastPt);
            }
        }

        // Store info for merge/split detection
        WormFrameInfo info;
        info.position = newPosition;
        info.roi = newRoi;
        info.plausibleBlobsInRoi = plausibleBlobsFoundInRoi;
        info.primaryBlobArea = primaryBlobArea;
        info.reportingTracker = reportingTracker;
        info.isValid = true;
        m_frameInfos[originalFrameNumber][wormId] = info;

        // Prune old frame info
        while (m_frameInfos.size() > m_frameInfoHistorySize && !m_frameInfos.isEmpty()) {
            m_frameInfos.remove(m_frameInfos.firstKey());
        }

        // Process data for this frame (can be complex)
        processFrameDataForMergesAndSplits(originalFrameNumber);

    } else {
        qWarning() << "TrackingManager: handleWormPositionUpdated from unknown source. WormID:" << wormId;
    }
}

// New Slot
void TrackingManager::handleWormSplitDetectedAndPaused(int wormId,
                                                       int originalFrameNumber,
                                                       const QList<TrackingHelper::DetectedBlob>& detectedBlobs) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    qDebug() << "TrackingManager: Worm ID" << wormId << "reported split and PAUSED at frame" << originalFrameNumber
             << "with" << detectedBlobs.count() << "new blobs.";
    emit trackingStatusUpdate(QString("Worm %1 paused, potential split detected.").arg(wormId));

    // Store this event's information.
    // The actual processing might happen in processFrameDataForMergesAndSplits
    // or directly here if it's a simpler model.
    // For now, let processFrameDataForMergesAndSplits handle it based on stored frameInfos
    // and this new explicit signal.

    // We can directly call a refined split processing function here
    // This function needs to know which tracker reported it.
    WormTracker* reportingTracker = nullptr;
    for(WormTracker* t : std::as_const(m_wormTrackers)) {
        if(t && t->getWormId() == wormId) {
            reportingTracker = t;
            break;
        }
    }

    if (reportingTracker) {
        // This is a more direct way to handle the split reported by a specific tracker
        // It was part of a merge.
        int representativeId = m_wormToMergeGroupMap.value(wormId, -1); // Get merge group rep ID

        if (representativeId != -1 && m_mergedGroups.contains(representativeId)) {
            QSet<int> mergedWormIds = m_mergedGroups.value(representativeId);
            qDebug() << "  Split from merge group" << representativeId << "(worms:" << mergedWormIds << "). New blobs:" << detectedBlobs.count();

            if (detectedBlobs.count() == mergedWormIds.count() && mergedWormIds.count() > 0) {
                qDebug() << "  Perfect split: Assigning" << detectedBlobs.count() << "blobs to" << mergedWormIds.count() << "worms.";
                emit trackingStatusUpdate(QString("Split resolved for merge group %1").arg(representativeId));

                QList<int> idsToAssign = QList<int>(mergedWormIds.begin(), mergedWormIds.end()); // Convert QSet to QList for ordered access
                std::sort(idsToAssign.begin(), idsToAssign.end()); // Sort for consistent assignment if desired

                for (int i = 0; i < idsToAssign.count(); ++i) {
                    int wormIdToReassign = idsToAssign.at(i);
                    const TrackingHelper::DetectedBlob& targetBlob = detectedBlobs.at(i); // Simple ordered assignment

                    WormTracker* trackerToCommand = nullptr;
                    for(WormTracker* t : std::as_const(m_wormTrackers)) { if(t && t->getWormId() == wormIdToReassign) { trackerToCommand = t; break; }}

                    if (trackerToCommand) {
                        qDebug() << "  Reassigning Worm" << wormIdToReassign << " (tracker " << trackerToCommand << ") to new blob at" << targetBlob.centroid;
                        QMetaObject::invokeMethod(trackerToCommand, "resumeTrackingWithNewTarget", Qt::QueuedConnection, Q_ARG(TrackingHelper::DetectedBlob, targetBlob));
                        if (m_wormObjectsMap.contains(wormIdToReassign)) m_wormObjectsMap.value(wormIdToReassign)->setState(WormObject::TrackingState::Tracking);
                    } else {
                        qWarning() << "  Could not find tracker for worm ID" << wormIdToReassign << "during split reassignment.";
                    }
                    m_wormToMergeGroupMap.remove(wormIdToReassign);
                }
                m_mergedGroups.remove(representativeId);
            } else {
                qWarning() << "  Imperfect split for merge group" << representativeId << ". Blobs:" << detectedBlobs.count() << "Expected:" << mergedWormIds.count();
                // Fallback: tell the reporting tracker to just pick the largest of the new blobs
                if (!detectedBlobs.isEmpty()) {
                    TrackingHelper::DetectedBlob largestBlob = detectedBlobs.first(); // Simplistic: pick first, or find largest
                    double maxArea = 0;
                    for(const auto& b : detectedBlobs) if(b.area > maxArea) {maxArea = b.area; largestBlob = b;}
                    qDebug() << "  Imperfect split: Telling reporting tracker" << reportingWormId << "to follow largest new blob.";
                    QMetaObject::invokeMethod(reportingTracker, "resumeTrackingWithNewTarget", Qt::QueuedConnection, Q_ARG(TrackingHelper::DetectedBlob, largestBlob));
                    if (m_wormObjectsMap.contains(wormId)) m_wormObjectsMap.value(wormId)->setState(WormObject::TrackingState::Tracking);
                    // Remove all involved worms from the merge group as it's no longer valid
                    for(int idInOldMerge : mergedWormIds) m_wormToMergeGroupMap.remove(idInOldMerge);
                    m_mergedGroups.remove(representativeId);
                }
            }
        } else { // Split from a worm that wasn't in a known merge group (fragmentation)
            qDebug() << "  Worm" << reportingWormId << "fragmented (was not in a merge group). New blobs:" << detectedBlobs.count();
            if (!detectedBlobs.isEmpty()) {
                TrackingHelper::DetectedBlob largestBlob = detectedBlobs.first();
                double maxArea = 0;
                for(const auto& b : detectedBlobs) if(b.area > maxArea) {maxArea = b.area; largestBlob = b;}
                qDebug() << "  Fragmented: Telling tracker" << reportingWormId << "to follow largest fragment.";
                QMetaObject::invokeMethod(reportingTracker, "resumeTrackingWithNewTarget", Qt::QueuedConnection, Q_ARG(TrackingHelper::DetectedBlob, largestBlob));
                if (m_wormObjectsMap.contains(wormId)) m_wormObjectsMap.value(wormId)->setState(WormObject::TrackingState::Tracking);
            }
        }
    } else {
        qWarning() << "TrackingManager: Split reported by worm ID" << wormId << "but could not find its tracker instance.";
    }
}


void TrackingManager::processFrameDataForMergesAndSplits(int frameNumber) {
    if (!m_frameInfos.contains(frameNumber)) return;
    const QMap<int, WormFrameInfo>& currentWormInfos = m_frameInfos.value(frameNumber);
    QList<int> activeWormIds = currentWormInfos.keys(); // Worms that reported in this frame

    // --- Merge Detection ---
    std::set<int> alreadyProcessedForMerge; // Use std::set for efficient lookup

    for (int i = 0; i < activeWormIds.size(); ++i) {
        int id1 = activeWormIds[i];
        if (alreadyProcessedForMerge.count(id1) || m_wormToMergeGroupMap.contains(id1)) {
            continue; // Already part of a merge or processed in this iteration
        }
        const WormFrameInfo& info1 = currentWormInfos.value(id1);
        if (!info1.isValid || !info1.reportingTracker) continue;

        for (int j = i + 1; j < activeWormIds.size(); ++j) {
            int id2 = activeWormIds[j];
            if (alreadyProcessedForMerge.count(id2) || m_wormToMergeGroupMap.contains(id2)) {
                continue;
            }
            const WormFrameInfo& info2 = currentWormInfos.value(id2);
            if (!info2.isValid || !info2.reportingTracker) continue;

            // Merge Criteria (example, needs tuning):
            // 1. ROIs overlap
            // 2. Centroids are very close (e.g., < half a typical worm width)
            // 3. Both trackers are reporting only 1 plausible blob in their (now likely overlapping) ROI.
            // 4. The area of the blob they are reporting is large enough to be a merge.
            bool roisOverlap = info1.roi.intersects(info2.roi);
            double centroidDistance = QLineF(info1.position, info2.position).length();
            double typicalWormWidth = 20.0; // Example, should be configurable or estimated

            if (roisOverlap && centroidDistance < (typicalWormWidth * 0.75) &&
                info1.plausibleBlobsInRoi == 1 && info2.plausibleBlobsInRoi == 1 &&
                (info1.primaryBlobArea > DEFAULT_MAX_BLOB_AREA * 0.7 || info2.primaryBlobArea > DEFAULT_MAX_BLOB_AREA * 0.7) // Area check
                ) {

                qDebug() << "TrackingManager: MERGE detected between Worm" << id1 << "and Worm" << id2 << "at frame" << frameNumber;
                emit trackingStatusUpdate(QString("Merge detected: %1 & %2").arg(id1).arg(id2));

                int representativeId = qMin(id1, id2);
                m_mergedGroups[representativeId].insert(id1);
                m_mergedGroups[representativeId].insert(id2);
                m_wormToMergeGroupMap[id1] = representativeId;
                m_wormToMergeGroupMap[id2] = representativeId;
                alreadyProcessedForMerge.insert(id1);
                alreadyProcessedForMerge.insert(id2);

                WormObject* wo1 = m_wormObjectsMap.value(id1);
                WormObject* wo2 = m_wormObjectsMap.value(id2);
                QPointF mergedPosition = (info1.position + info2.position) / 2.0;
                QRectF mergedRoi = info1.roi.united(info2.roi).adjusted(-5,-5,5,5); // Expand a bit

                if (wo1) {
                    wo1->setState(WormObject::TrackingState::Merged, representativeId);
                    wo1->updateTrackPoint(frameNumber, cv::Point2f(mergedPosition.x(), mergedPosition.y()), mergedRoi);
                }
                if (wo2) {
                    wo2->setState(WormObject::TrackingState::Merged, representativeId);
                    wo2->updateTrackPoint(frameNumber, cv::Point2f(mergedPosition.x(), mergedPosition.y()), mergedRoi);
                }

                // Command trackers
                if(info1.reportingTracker) QMetaObject::invokeMethod(info1.reportingTracker, "confirmTargetIsMerged", Qt::QueuedConnection, Q_ARG(int, representativeId), Q_ARG(QPointF, mergedPosition), Q_ARG(QRectF, mergedRoi));
                if(info2.reportingTracker) QMetaObject::invokeMethod(info2.reportingTracker, "confirmTargetIsMerged", Qt::QueuedConnection, Q_ARG(int, representativeId), Q_ARG(QPointF, mergedPosition), Q_ARG(QRectF, mergedRoi));

                break; // id1 is now merged, move to next i
            }
        }
    }
    // Split detection is now primarily handled by handleWormSplitDetectedAndPaused
    // This function could also look for splits if a merged group suddenly has its trackers diverge
    // without an explicit split signal, but that's more complex.
}


// --- Other slots and private methods (handleWormStateChanged, handleWormTrackerFinished, etc.) ---
// ... (ensure these are implemented as in the robust cleanup version, adapted for new signal parameters if needed) ...
void TrackingManager::handleWormStateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId) {
    if (m_cancelRequested) return;
    WormObject* worm = m_wormObjectsMap.value(wormId, nullptr);
    if (worm) {
        // This signal from WormTracker is about the WormObject's state,
        // which TrackingManager might also update based on merges/splits.
        // For now, let's assume this is mostly for "Lost" state.
        if (newState == WormObject::TrackingState::Lost) {
            worm->setState(newState, associatedWormId);
            emit trackingStatusUpdate(QString("Worm %1 reported as Lost by its tracker.").arg(wormId));
        }
        // Other state changes (like to Merged) are now handled more directly by TrackingManager
        // when it detects a merge.
    }
}

void TrackingManager::handleWormTrackerFinished() {
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if(finishedTracker) {
        qDebug() << "TrackingManager: WormTracker for worm ID" << finishedTracker->getWormId() << " (ptr: " << finishedTracker << ") reported finished.";
        m_individualTrackerProgress.remove(finishedTracker);
        // The tracker object itself is scheduled for deleteLater by its thread's finished signal.
        // The QList m_wormTrackers will have this tracker removed by the lambda connected to thread finished.
    } else {
        qWarning() << "TrackingManager: handleWormTrackerFinished called by non-WormTracker sender.";
    }
    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: Total finished trackers:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;
    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerError(int wormId, QString errorMessage) {
    qDebug() << "TrackingManager (" << this << "): handleWormTrackerError for worm " << wormId << ":" << errorMessage;
    if (m_cancelRequested) { return; }
    qWarning() << "TrackingManager: Error from tracker for worm ID" << wormId << ":" << errorMessage;
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    if(errorTracker) { m_individualTrackerProgress.remove(errorTracker); }
    m_finishedTrackersCount++;
    updateOverallProgress();
    emit trackingStatusUpdate(QString("Error with tracker for worm %1.").arg(wormId));
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerProgress(int wormId, int percentDone){
    Q_UNUSED(wormId); // Progress is now mapped by tracker instance
    WormTracker* tracker = qobject_cast<WormTracker*>(sender());
    if(tracker && !m_cancelRequested){
        m_individualTrackerProgress[tracker] = percentDone;
        updateOverallProgress();
    }
}


void TrackingManager::checkForAllTrackersFinished() {
    // ... (Implementation as in previous robust version) ...
    qDebug() << "TrackingManager (" << this << "): checkForAllTrackersFinished. Cancelled:" << m_cancelRequested << "Running:" << m_isTrackingRunning << "Finished:" << m_finishedTrackersCount << "Expected:" << m_expectedTrackersToFinish;
    if (m_cancelRequested) {
        if (m_isTrackingRunning && m_finishedTrackersCount >= m_expectedTrackersToFinish) {
            qDebug() << "TM: All expected trackers have reported finished after cancellation request.";
            emit trackingCancelled(); m_isTrackingRunning = false;
        } return;
    }
    if (m_isTrackingRunning && m_finishedTrackersCount >= m_expectedTrackersToFinish) {
        emit trackingStatusUpdate("All worm trackers completed. Consolidating and saving tracks...");
        m_finalTracks.clear();
        for(WormObject* worm : m_wormObjectsMap.values()){ if(worm) m_finalTracks[worm->getId()] = worm->getTrackHistory(); }
        emit allTracksUpdated(m_finalTracks); outputTracksToDebug(m_finalTracks);
        QString csvOutputPath;
        if (!m_videoPath.isEmpty()) {
            QFileInfo videoInfo(m_videoPath); QString tmpDirName = "tmp_tracking_output"; QDir videoDir = videoInfo.dir();
            if (videoDir.mkpath(tmpDirName)) csvOutputPath = videoDir.filePath(tmpDirName + "/" + videoInfo.completeBaseName() + "_tracks.csv");
            else { csvOutputPath = videoDir.filePath(videoInfo.completeBaseName() + "_tracks.csv"); qWarning() << "Could not create tmp directory, saving CSV to video directory."; }
        } else { csvOutputPath = "worm_tracks.csv"; qWarning() << "Video path is empty, saving tracks to default file:" << csvOutputPath; }
        bool csvSaved = outputTracksToCsv(m_finalTracks, csvOutputPath);
        if (csvSaved) { emit trackingStatusUpdate("Tracks saved to: " + csvOutputPath); emit trackingFinishedSuccessfully(csvOutputPath); }
        else { emit trackingStatusUpdate("Tracking finished, but failed to save CSV."); emit trackingFinishedSuccessfully(""); }
        m_isTrackingRunning = false; qDebug() << "TM: All tracking tasks complete. Tracks consolidated.";
    }
}
void TrackingManager::updateOverallProgress() {
    // ... (Implementation as in previous robust version) ...
    if (m_cancelRequested && !m_isTrackingRunning) { emit overallTrackingProgress(0); return; }
    if (!m_isTrackingRunning && m_finishedTrackersCount < m_expectedTrackersToFinish) { emit overallTrackingProgress(0); return; }
    double totalProgress = 0; double videoProcWeight = 0.20; double trackersWeight = 0.80;
    totalProgress += static_cast<double>(m_videoProcessingProgress) * videoProcWeight / 100.0;
    if (m_expectedTrackersToFinish > 0) {
        double currentTrackersAggregatedProgressPts = 0; int validTrackerProgressEntries = 0;
        for (int progress : m_individualTrackerProgress.values()) { currentTrackersAggregatedProgressPts += static_cast<double>(progress); validTrackerProgressEntries++; }
        if (validTrackerProgressEntries > 0) { double avgTrackerProgress = currentTrackersAggregatedProgressPts / static_cast<double>(validTrackerProgressEntries); totalProgress += avgTrackerProgress * trackersWeight / 100.0; }
        else if (m_finishedTrackersCount > 0 && m_finishedTrackersCount == m_expectedTrackersToFinish) { totalProgress += 1.0 * trackersWeight; }
    } else if (m_videoProcessingProgress == 100) { totalProgress = 1.0; }
    emit overallTrackingProgress(qBound(0, static_cast<int>(totalProgress * 100.0), 100));
}
void TrackingManager::outputTracksToDebug(const AllWormTracks& tracks) const { /* ... as before ... */
    qDebug() << "--- Begin Track Output (" << this << ") ---"; if (tracks.empty()) { qDebug() << "No tracks to output."; }
    for (const auto& pair : tracks) { /* ... existing log logic ... */ } qDebug() << "--- End Track Output ---";
}
bool TrackingManager::outputTracksToCsv(const AllWormTracks& tracks, const QString& outputFilePath) const { /* ... as before ... */
    if (outputFilePath.isEmpty()) return false; QFile csvFile(outputFilePath);
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) { qWarning() << "Failed to open CSV:" << outputFilePath; return false; }
    QTextStream outStream(&csvFile); outStream << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight\n";
    for (const auto& pair : tracks) { /* ... existing CSV writing logic ... */ } csvFile.close();
    if(csvFile.error()!=QFile::NoError) {qWarning()<<"Error writing CSV:"<<csvFile.errorString(); return false;}
    qDebug() << "Tracks written to CSV:" << outputFilePath; return true;
}
