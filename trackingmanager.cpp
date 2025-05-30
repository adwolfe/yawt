// trackingmanager.cpp
#include "trackingmanager.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
// QPointer is included in trackingmanager.h
#include <QPair>
#include <QRectF>

#include <algorithm>
#include <numeric>
#include <limits>
#include <cmath>

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
    m_nextUniqueMergeId(0),
    m_pauseResolutionTimer(nullptr)
{
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<Tracking::AllWormTracks>("Tracking::AllWormTracks");
    qRegisterMetaType<QList<Tracking::DetectedBlob>>("QList<Tracking::DetectedBlob>");
    qRegisterMetaType<Tracking::DetectedBlob>("Tracking::DetectedBlob");
    qRegisterMetaType<Tracking::TrackerState>("Tracking::TrackerState");

    m_pauseResolutionTimer = new QTimer(this);
    connect(m_pauseResolutionTimer, &QTimer::timeout, this, &TrackingManager::checkPausedWormsAndResolveSplits);

    qDebug() << "TrackingManager (" << this << ") created.";
}

TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";
    if (m_pauseResolutionTimer) {
        m_pauseResolutionTimer->stop();
    }
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking(); // This will also stop the timer
        QThread::msleep(200); // Give cancel a moment
    }
    cleanupThreadsAndObjects(); // This will delete m_pauseResolutionTimer as it's a child
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

    cleanupThreadsAndObjects();

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
    m_nextUniqueMergeId = 1; // Start IDs from 1

    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_activeMergeGroups.clear();
    m_frameToActiveMergeGroupIds.clear();
    m_wormToCurrentMergeGroupId.clear();
    m_pausedWorms.clear();

    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    for (const auto& info : m_initialWormInfos) {
        if (!m_wormObjectsMap.contains(info.id)) {
            m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
        }
        m_wormToCurrentMergeGroupId[info.id] = -1;
    }

    emit trackingStatusUpdate("Starting video processing...");
    emit overallTrackingProgress(0);

    m_videoProcessor = new VideoProcessor();
    m_videoProcessorThread = new QThread(this);
    m_videoProcessor->moveToThread(m_videoProcessorThread);

    connect(m_videoProcessorThread, &QThread::started, m_videoProcessor, [this]() {
        if (m_videoProcessor) {
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

    connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor, &QObject::deleteLater);
    connect(m_videoProcessorThread, &QThread::finished, this, [this]() {
        qDebug() << "VideoProcessorThread (" << QObject::sender() << ") finished. m_videoProcessor (" << m_videoProcessor << ") was scheduled for deleteLater.";
        m_videoProcessor = nullptr;
    });

    m_videoProcessorThread->start();
    if (m_pauseResolutionTimer) { // Ensure timer exists
        m_pauseResolutionTimer->start(PAUSE_RESOLUTION_INTERVAL_MS);
    }
}

void TrackingManager::cancelTracking() {
    qDebug() << "TrackingManager (" << this << "): cancelTracking called. IsRunning:" << m_isTrackingRunning << "CancelRequested:" << m_cancelRequested;

    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested && !m_isTrackingRunning) { // Already cancelled and processed.
        qDebug() << "TM: Cancellation already fully processed or tracking not running.";
        return;
    }
    if (m_cancelRequested) { // Cancellation requested but perhaps not fully finished.
        qDebug() << "TM: Cancellation already in progress.";
        return;
    }
    m_cancelRequested = true;
    bool wasRunning = m_isTrackingRunning; // Capture state before potentially changing it
    locker.unlock();

    emit trackingStatusUpdate("Cancellation requested...");

    if (m_pauseResolutionTimer) {
        m_pauseResolutionTimer->stop();
    }

    if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
        qDebug() << "TM: Requesting video processor thread (" << m_videoProcessorThread << ") interruption.";
        m_videoProcessorThread->requestInterruption();
    }

    QList<WormTracker*> trackersToStop;
    locker.relock(); // Lock for accessing m_wormTrackersList
    trackersToStop = m_wormTrackersList;
    locker.unlock();

    for (WormTracker* tracker : trackersToStop) {
        if (tracker) {
            QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
        }
    }

    // If tracking wasn't even fully started (e.g. stuck in video processing)
    // or if no trackers were ever launched.
    locker.relock();
    if (wasRunning && m_expectedTrackersToFinish == 0 && m_wormTrackersList.isEmpty()) {
        m_isTrackingRunning = false; // Ensure tracking stops
        locker.unlock();
        qDebug() << "TM: Cancellation processed for a state with no active trackers.";
        emit trackingCancelled();
    } else if (!wasRunning) {
        // if it was not running, and cancel is called, just emit cancelled.
        locker.unlock();
        emit trackingCancelled();
    }
    // checkForAllTrackersFinished will handle emitting trackingCancelled if trackers were running.
}

void TrackingManager::cleanupThreadsAndObjects() {
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - START";
    if (m_pauseResolutionTimer && m_pauseResolutionTimer->isActive()) {
        m_pauseResolutionTimer->stop();
    }
    // delete m_pauseResolutionTimer; // QObject parent system handles this if it's a child
    // m_pauseResolutionTimer = nullptr;

    if (m_videoProcessorThread) {
        qDebug() << "  Cleaning up video processor thread:" << m_videoProcessorThread;
        if (m_videoProcessorThread->isRunning()) {
            m_videoProcessorThread->requestInterruption();
            m_videoProcessorThread->quit();
            if (!m_videoProcessorThread->wait(1000)) {
                qWarning() << "  VideoProcessor thread (" << m_videoProcessorThread << ") did not finish gracefully. Forcing termination.";
                m_videoProcessorThread->terminate();
                m_videoProcessorThread->wait();
            }
        }
        delete m_videoProcessorThread;
        m_videoProcessorThread = nullptr;
        m_videoProcessor = nullptr;
    }

    QList<QThread*> threadsToClean = m_trackerThreads;
    for (QThread* thread : threadsToClean) {
        if (thread) {
            qDebug() << "  Cleaning up tracker thread:" << thread;
            if (thread->isRunning()) {
                thread->requestInterruption();
                thread->quit();
                if (!thread->wait(500)) {
                    qWarning() << "  A WormTracker thread (" << thread << ") did not finish gracefully. Forcing termination.";
                    thread->terminate();
                    thread->wait();
                }
            }
            delete thread;
        }
    }
    m_trackerThreads.clear();
    m_wormTrackersList.clear();
    m_wormIdToForwardTrackerInstanceMap.clear();
    m_wormIdToBackwardTrackerInstanceMap.clear();

    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    m_processedForwardFrames.clear();
    std::vector<cv::Mat>().swap(m_processedForwardFrames);
    m_processedReversedFrames.clear();
    std::vector<cv::Mat>().swap(m_processedReversedFrames);

    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_activeMergeGroups.clear();
    m_frameToActiveMergeGroupIds.clear();
    m_wormToCurrentMergeGroupId.clear();
    m_pausedWorms.clear();

    m_isTrackingRunning = false;
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - FINISHED";
}


void TrackingManager::handleInitialProcessingComplete(
    const std::vector<cv::Mat>& forwardFrames,
    const std::vector<cv::Mat>& reversedFrames, double fps,
    cv::Size frameSize) {
    qDebug() << "TrackingManager (" << this << "): handleInitialProcessingComplete received.";
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing complete but cancellation requested. Not launching trackers.";
        m_isTrackingRunning = false; // Ensure state is correct
        locker.unlock();
        emit trackingCancelled();
        return;
    }
    if (!m_isTrackingRunning) {
        qWarning() << "TM: handleInitialProcessingComplete, but tracking is not marked as running. Aborting tracker launch.";
        return; // Mutex will unlock
    }
    locker.unlock();


    qDebug() << "TM: Initial video processing complete. Fwd:" << forwardFrames.size() << "Rev:" << reversedFrames.size() << "FPS:" << fps;
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    m_videoProcessingProgress = 100;
    m_processedForwardFrames = forwardFrames;
    m_processedReversedFrames = reversedFrames;
    m_videoFps = fps;
    m_videoFrameSize = frameSize;

    launchWormTrackers(); // This will set m_expectedTrackersToFinish
    updateOverallProgress();
}

void TrackingManager::handleVideoProcessingError(const QString& errorMessage) {
    qDebug() << "TrackingManager (" << this << "): handleVideoProcessingError: " << errorMessage;
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing error (" << errorMessage << ") received during/after cancellation. Letting cancel flow proceed.";
        // If cancellation is in progress, it will eventually emit trackingCancelled().
        // We might not need to emit trackingFailed() here to avoid double signals.
        return;
    }
    if (!m_isTrackingRunning) {
        qWarning() << "TM: Video processing error (" << errorMessage << ") but tracking not marked as running.";
        return;
    }
    m_isTrackingRunning = false;
    if (m_pauseResolutionTimer) {
        m_pauseResolutionTimer->stop();
    }
    locker.unlock(); // Unlock before emitting

    qWarning() << "TM: Video processing error:" << errorMessage;
    emit trackingFailed("Video processing failed: " + errorMessage);
    // No trackers launched, so no need to wait for them to finish.
}

void TrackingManager::handleVideoProcessingProgress(int percentage) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    m_videoProcessingProgress = percentage;
    updateOverallProgress();
}

void TrackingManager::handleFrameUpdate(int reportingConceptualWormId,
                                        int originalFrameNumber,
                                        const Tracking::DetectedBlob& primaryBlob,
                                        QRectF searchRoiUsed,
                                        Tracking::TrackerState currentState)
{
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* reportingTrackerInstance = qobject_cast<WormTracker*>(sender());
    if (!reportingTrackerInstance) {
        qWarning() << "TM: handleFrameUpdate from unknown sender for worm ID:" << reportingConceptualWormId << "Frame:" << originalFrameNumber;
        // Cannot reliably store trackerInstance if sender is null.
        // This could happen if the signal was emitted in a way that QObject::sender() is not preserved (e.g. through lambda disconnect/reconnect).
        // For now, we'll proceed but PausedForSplit might not work correctly if it relies on this.
    }


    WormObject* wormObject = m_wormObjectsMap.value(reportingConceptualWormId, nullptr);
    if (!wormObject) {
        qWarning() << "TM: handleFrameUpdate for unknown worm object ID:" << reportingConceptualWormId;
        return;
    }

    QString dirTag = reportingTrackerInstance ? (reportingTrackerInstance->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd") : "UnkDir";
    qDebug().noquote() << QString("TM: WT %1(%2) Frame %3 State %4")
                              .arg(reportingConceptualWormId)
                              .arg(dirTag)
                              .arg(originalFrameNumber)
                              .arg(static_cast<int>(currentState));

    if (primaryBlob.isValid) {
        Tracking::WormTrackPoint point;
        point.frameNumberOriginal = originalFrameNumber;
        point.position = cv::Point2f(static_cast<float>(primaryBlob.centroid.x()), static_cast<float>(primaryBlob.centroid.y()));
        point.roi = searchRoiUsed;
        point.quality = (currentState == Tracking::TrackerState::TrackingSingle) ? Tracking::TrackPointQuality::Confident : Tracking::TrackPointQuality::Ambiguous;
        wormObject->updateTrackPoint(point);
    }

    int currentMergeGroupId = m_wormToCurrentMergeGroupId.value(reportingConceptualWormId, -1);

    if (currentState == Tracking::TrackerState::TrackingSingle || currentState == Tracking::TrackerState::TrackingLost) {
        if (currentMergeGroupId != -1) {
            if (m_activeMergeGroups.contains(currentMergeGroupId)) {
                TrackedMergeGroup& group = m_activeMergeGroups[currentMergeGroupId];
                group.participatingWormIds.remove(reportingConceptualWormId);
                group.individualEstimatedCentroids.remove(reportingConceptualWormId);
                if (group.participatingWormIds.isEmpty()) {
                    group.isValid = false; // Mark for cleanup by cleanupStaleMergeGroups
                    qDebug() << "TM: Merge group" << currentMergeGroupId << "is now empty due to worm" << reportingConceptualWormId << "state" << static_cast<int>(currentState);
                }
            }
            m_wormToCurrentMergeGroupId[reportingConceptualWormId] = -1;
            qDebug() << "TM: Worm" << reportingConceptualWormId << "exited merge group" << currentMergeGroupId << "to state" << static_cast<int>(currentState);
        }
        if (m_pausedWorms.contains(reportingConceptualWormId)) {
            m_pausedWorms.remove(reportingConceptualWormId);
            qDebug() << "TM: Worm" << reportingConceptualWormId << "removed from pause queue due to state" << static_cast<int>(currentState);
        }
    } else if (currentState == Tracking::TrackerState::TrackingMerged) {
        if (primaryBlob.isValid) {
            processMergedState(reportingConceptualWormId, originalFrameNumber, primaryBlob, reportingTrackerInstance);
        }
    } else if (currentState == Tracking::TrackerState::PausedForSplit) {
        if (primaryBlob.isValid && reportingTrackerInstance) { // Need tracker instance to resume it
            processPausedForSplitState(reportingConceptualWormId, originalFrameNumber, primaryBlob, reportingTrackerInstance);
        } else {
            qWarning() << "TM: Worm" << reportingConceptualWormId
                       << "reported PausedForSplit without a valid primaryBlob or tracker instance. Forcing to lost.";
            if (m_pausedWorms.contains(reportingConceptualWormId)) { // Clean up if it was somehow added
                m_pausedWorms.remove(reportingConceptualWormId);
            }
            if (currentMergeGroupId != -1) { // Detach from merge group
                m_wormToCurrentMergeGroupId[reportingConceptualWormId] = -1;
                // Further cleanup of group if needed would happen in cleanupStaleMergeGroups or TrackingLost handler
            }
            if(reportingTrackerInstance){ // If we have the instance, tell it to go lost
                Tracking::DetectedBlob invalidBlob;
                QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, invalidBlob));
            }
        }
    }

    if (originalFrameNumber > 0 && originalFrameNumber % 20 == 0) { // Less frequent cleanup
        cleanupStaleMergeGroups(originalFrameNumber);
    }
}

void TrackingManager::processMergedState(int reportingConceptualWormId, int frameNumber, const Tracking::DetectedBlob& mergedBlobData, WormTracker* /*reportingTrackerInstance*/) {
    // m_dataMutex is already locked
    QRectF reportedBox = mergedBlobData.boundingBox;
    cv::Point2f reportedCentroid(static_cast<float>(mergedBlobData.centroid.x()), static_cast<float>(mergedBlobData.centroid.y()));
    double reportedArea = mergedBlobData.area;

    TrackedMergeGroup* matchedGroup = findMatchingMergeGroup(frameNumber, reportedBox, reportedCentroid, reportedArea);
    int assignedMergeId = -1;

    if (matchedGroup) {
        assignedMergeId = matchedGroup->uniqueMergeId;
        matchedGroup->participatingWormIds.insert(reportingConceptualWormId);
        matchedGroup->individualEstimatedCentroids[reportingConceptualWormId] = reportedCentroid;
        matchedGroup->lastFrameActive = frameNumber;
        matchedGroup->lastUpdateTime = QDateTime::currentDateTime();
        // Update overall group properties
        matchedGroup->currentBoundingBox = matchedGroup->currentBoundingBox.united(reportedBox);
        matchedGroup->currentArea = qMax(matchedGroup->currentArea, reportedArea);
        // Centroid could be an average, but for simplicity, let's keep the one from the first worm or the largest contribution
        // Or, if mergedBlobData.centroid is meant to be the overall centroid, we can average them if they are very close.
        // For now, we don't aggressively update the group's main centroid from subsequent joiners unless it's a new group.
        qDebug() << "TM: Worm" << reportingConceptualWormId << "joined/updated merge group" << assignedMergeId << "in frame" << frameNumber;
    } else {
        assignedMergeId = createNewMergeGroup(frameNumber, reportingConceptualWormId, mergedBlobData);
        qDebug() << "TM: Worm" << reportingConceptualWormId << "created new merge group" << assignedMergeId << "in frame" << frameNumber;
    }

    int previousMergeId = m_wormToCurrentMergeGroupId.value(reportingConceptualWormId, -1);
    if (previousMergeId != -1 && previousMergeId != assignedMergeId && m_activeMergeGroups.contains(previousMergeId)) {
        TrackedMergeGroup& oldGroup = m_activeMergeGroups[previousMergeId];
        oldGroup.participatingWormIds.remove(reportingConceptualWormId);
        oldGroup.individualEstimatedCentroids.remove(reportingConceptualWormId);
        if (oldGroup.participatingWormIds.isEmpty()) {
            oldGroup.isValid = false;
        }
    }
    m_wormToCurrentMergeGroupId[reportingConceptualWormId] = assignedMergeId;

    if (m_pausedWorms.contains(reportingConceptualWormId)) {
        m_pausedWorms.remove(reportingConceptualWormId);
        qDebug() << "TM: Worm" << reportingConceptualWormId << "exited PausedForSplit into TrackingMerged group" << assignedMergeId;
    }
}

TrackedMergeGroup* TrackingManager::findMatchingMergeGroup(int frameNumber, const QRectF& blobBox, const cv::Point2f& blobCentroid, double /*blobArea*/) {
    // m_dataMutex is already locked
    TrackedMergeGroup* bestMatch = nullptr;
    double bestMatchScore = -1.0; // Using a score, e.g., IoU

    // Check groups active in current frame or previous frame
    QSet<int> candidateGroupIds;
    if (m_frameToActiveMergeGroupIds.contains(frameNumber)) {
        candidateGroupIds.unite(m_frameToActiveMergeGroupIds.value(frameNumber));
    }
    if (m_frameToActiveMergeGroupIds.contains(frameNumber - 1)) {
        candidateGroupIds.unite(m_frameToActiveMergeGroupIds.value(frameNumber - 1));
    }

    for (int groupId : candidateGroupIds) {
        if (!m_activeMergeGroups.contains(groupId) || !m_activeMergeGroups[groupId].isValid) continue;

        TrackedMergeGroup& group = m_activeMergeGroups[groupId];
        // Ensure group was active recently enough if checking previous frame's groups
        if (group.lastFrameActive < frameNumber - 1 && frameNumber > 0) continue;


        double iou = calculateIoU(blobBox, group.currentBoundingBox);
        double distSq = Tracking::sqDistance(blobCentroid, group.currentCentroid);

        if (iou > MERGE_GROUP_IOU_THRESHOLD && distSq < MERGE_GROUP_CENTROID_MAX_DIST_SQ) {
            if (iou > bestMatchScore) { // Prioritize IoU for matching
                bestMatchScore = iou;
                bestMatch = &group;
            }
        }
    }
    return bestMatch;
}

int TrackingManager::createNewMergeGroup(int frameNumber, int initialConceptualWormId, const Tracking::DetectedBlob& mergedBlobData) {
    // m_dataMutex is already locked
    TrackedMergeGroup newGroup;
    newGroup.uniqueMergeId = m_nextUniqueMergeId++;
    newGroup.currentBoundingBox = mergedBlobData.boundingBox;
    newGroup.currentCentroid = cv::Point2f(static_cast<float>(mergedBlobData.centroid.x()), static_cast<float>(mergedBlobData.centroid.y()));
    newGroup.currentArea = mergedBlobData.area;
    newGroup.participatingWormIds.insert(initialConceptualWormId);
    newGroup.individualEstimatedCentroids[initialConceptualWormId] = newGroup.currentCentroid;
    newGroup.firstFrameSeen = frameNumber;
    newGroup.lastFrameActive = frameNumber;
    newGroup.lastUpdateTime = QDateTime::currentDateTime();
    newGroup.isValid = true;

    m_activeMergeGroups[newGroup.uniqueMergeId] = newGroup;
    m_frameToActiveMergeGroupIds[frameNumber].insert(newGroup.uniqueMergeId);
    return newGroup.uniqueMergeId;
}

void TrackingManager::processPausedForSplitState(int reportingConceptualWormId, int frameNumber,
                                                 const Tracking::DetectedBlob& candidateSplitBlob,
                                                 WormTracker* reportingTrackerInstance) {
    // m_dataMutex is already locked
    if (!reportingTrackerInstance) {
        qWarning() << "TM: processPausedForSplitState called with null tracker instance for worm" << reportingConceptualWormId << ". Cannot process pause.";
        return;
    }
    if (m_pausedWorms.contains(reportingConceptualWormId)) {
        qDebug() << "TM: Worm" << reportingConceptualWormId << "reported PausedForSplit again. Updating candidate and resetting timer.";
        m_pausedWorms[reportingConceptualWormId].candidateSplitBlob = candidateSplitBlob;
        m_pausedWorms[reportingConceptualWormId].timePaused = QDateTime::currentDateTime();
        m_pausedWorms[reportingConceptualWormId].trackerInstance = reportingTrackerInstance; // Ensure it's up-to-date
        return;
    }

    PausedWormInfo pwi;
    pwi.conceptualWormId = reportingConceptualWormId;
    pwi.trackerInstance = reportingTrackerInstance;
    pwi.framePausedOn = frameNumber;
    pwi.timePaused = QDateTime::currentDateTime();
    pwi.candidateSplitBlob = candidateSplitBlob;
    pwi.presumedMergeGroupId_F_minus_1 = m_wormToCurrentMergeGroupId.value(reportingConceptualWormId, -1);

    m_pausedWorms[reportingConceptualWormId] = pwi;
    qDebug() << "TM: Worm" << reportingConceptualWormId << " (Tracker:" << reportingTrackerInstance << ")"
             << "entered PausedForSplit in frame" << frameNumber
             << ". Candidate @ (" << candidateSplitBlob.centroid.x() << "," << candidateSplitBlob.centroid.y() << ")"
             << ". Came from merge group:" << pwi.presumedMergeGroupId_F_minus_1;

    if (pwi.presumedMergeGroupId_F_minus_1 != -1 && m_activeMergeGroups.contains(pwi.presumedMergeGroupId_F_minus_1)) {
        TrackedMergeGroup& group = m_activeMergeGroups[pwi.presumedMergeGroupId_F_minus_1];
        group.participatingWormIds.remove(reportingConceptualWormId);
        group.individualEstimatedCentroids.remove(reportingConceptualWormId);
        if (group.participatingWormIds.isEmpty()){
            group.isValid = false; // Will be cleaned up by cleanupStaleMergeGroups
        }
    }
    m_wormToCurrentMergeGroupId[reportingConceptualWormId] = -1;

    attemptAutomaticSplitResolution(reportingConceptualWormId, m_pausedWorms[reportingConceptualWormId]);
}

void TrackingManager::checkPausedWormsAndResolveSplits() {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning || m_pausedWorms.isEmpty()) {
        return;
    }

    QDateTime currentTime = QDateTime::currentDateTime();
    // Iterate over a copy of keys because resolution might remove items from m_pausedWorms
    QList<int> pausedConceptualWormIds = m_pausedWorms.keys();

    for (int conceptualId : pausedConceptualWormIds) {
        if (!m_pausedWorms.contains(conceptualId)) continue; // Already resolved in this iteration

        PausedWormInfo& pausedInfo = m_pausedWorms[conceptualId]; // Get reference

        if (pausedInfo.timePaused.msecsTo(currentTime) > MAX_PAUSED_DURATION_MS) {
            qDebug() << "TM: Worm" << conceptualId << "in PausedForSplit TIMED OUT. Forcing resolution.";
            forceResolvePausedWorm(conceptualId, pausedInfo);
        } else {
            attemptAutomaticSplitResolution(conceptualId, pausedInfo);
        }
    }
}

void TrackingManager::attemptAutomaticSplitResolution(int pausedConceptualWormId, PausedWormInfo& pausedInfo) {
    // m_dataMutex is already locked
    if (!pausedInfo.trackerInstance) {
        qWarning() << "TM: No tracker instance for paused worm" << pausedConceptualWormId << ". Cannot resolve split.";
        m_pausedWorms.remove(pausedConceptualWormId); // Remove to prevent repeated attempts
        return;
    }
    if (!pausedInfo.candidateSplitBlob.isValid) {
        qDebug() << "TM: Worm" << pausedConceptualWormId << "has invalid candidate blob in attemptAutomaticSplitResolution. Forcing lost.";
        Tracking::DetectedBlob invalidBlob;
        QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, invalidBlob));
        m_pausedWorms.remove(pausedConceptualWormId);
        return;
    }

    int presumedMergeId = pausedInfo.presumedMergeGroupId_F_minus_1;
    if (presumedMergeId == -1 || !m_activeMergeGroups.contains(presumedMergeId) || !m_activeMergeGroups.value(presumedMergeId).isValid) {
        qDebug() << "TM: Worm" << pausedConceptualWormId << "paused, but no valid prior merge group (" << presumedMergeId << "). Resolving independently via force.";
        forceResolvePausedWorm(pausedConceptualWormId, pausedInfo);
        return;
    }

    QList<int> potentialBuddyConceptualIds;
    for (auto it = m_pausedWorms.constBegin(); it != m_pausedWorms.constEnd(); ++it) {
        if (it.key() == pausedConceptualWormId) continue;
        const PausedWormInfo& otherInfo = it.value();
        if (otherInfo.framePausedOn == pausedInfo.framePausedOn &&
            otherInfo.presumedMergeGroupId_F_minus_1 == presumedMergeId &&
            otherInfo.candidateSplitBlob.isValid && otherInfo.trackerInstance) {
            potentialBuddyConceptualIds.append(it.key());
        }
    }

    if (potentialBuddyConceptualIds.isEmpty()) {
        // No buddies currently paused from the same group and frame. Wait for timeout or for a buddy to pause.
        return;
    }

    // Simple 2-worm split: if exactly one buddy.
    if (potentialBuddyConceptualIds.size() == 1) {
        int buddyConceptualId = potentialBuddyConceptualIds.first();
        // Ensure buddy is still in m_pausedWorms as this function can be re-entrant due to iteration over keys copy
        if (!m_pausedWorms.contains(buddyConceptualId)) return;
        PausedWormInfo& buddyInfo = m_pausedWorms[buddyConceptualId];

        double iou = calculateIoU(pausedInfo.candidateSplitBlob.boundingBox, buddyInfo.candidateSplitBlob.boundingBox);
        if (iou < 0.1) { // Blobs are distinct
            qDebug() << "TM: Resolving 2-way split for" << pausedConceptualWormId << "and" << buddyConceptualId << "from group" << presumedMergeId;

            if (pausedInfo.trackerInstance) {
                QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, pausedInfo.candidateSplitBlob));
            }
            m_pausedWorms.remove(pausedConceptualWormId);

            if (buddyInfo.trackerInstance) {
                QMetaObject::invokeMethod(buddyInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, buddyInfo.candidateSplitBlob));
            }
            m_pausedWorms.remove(buddyConceptualId);
            return;
        } else {
            qDebug() << "TM: Worm" << pausedConceptualWormId << "and buddy" << buddyConceptualId << "paused, candidates overlap (IoU:" << iou << "). Waiting.";
        }
    } else {
        qDebug() << "TM: Worm" << pausedConceptualWormId << "paused with" << potentialBuddyConceptualIds.size() << "buddies. Complex split, deferring to timeout or simpler state.";
    }
}

void TrackingManager::forceResolvePausedWorm(int conceptualWormId, PausedWormInfo& pausedInfo) {
    // m_dataMutex is already locked
    qDebug() << "TM: Forcing resolution for paused worm" << conceptualWormId;

    if (!pausedInfo.trackerInstance) {
        qWarning() << "TM: No tracker instance for force-resolving worm" << conceptualWormId << ". Removing from pause queue.";
        m_pausedWorms.remove(conceptualWormId);
        return;
    }

    Tracking::DetectedBlob targetBlob = pausedInfo.candidateSplitBlob; // Might be invalid
    if (!targetBlob.isValid) {
        qDebug() << "  Worm" << conceptualWormId << "had no valid candidate blob. Instructing to go lost.";
    } else {
        qDebug() << "  Worm" << conceptualWormId << "is taking its candidate blob at"
                 << targetBlob.centroid.x() << "," << targetBlob.centroid.y()
                 << "after timeout/force resolve.";
        // TODO: Add a check here if targetBlob is already actively tracked by a *non-paused* worm.
        // This is complex as it requires knowing current targets of all other active trackers.
        // For now, we let it try to take its candidate.
    }

    QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(),
                              "resumeTrackingWithAssignedTarget",
                              Qt::QueuedConnection,
                              Q_ARG(Tracking::DetectedBlob, targetBlob)); // Pass invalid blob if !targetBlob.isValid

    m_pausedWorms.remove(conceptualWormId);
}

void TrackingManager::cleanupStaleMergeGroups(int currentFrameNumber) {
    // m_dataMutex is already locked
    QList<int> groupIdsToRemove;
    for (auto it = m_activeMergeGroups.begin(); it != m_activeMergeGroups.end(); ++it) {
        TrackedMergeGroup& group = it.value();
        bool isStale = (currentFrameNumber - group.lastFrameActive > 100); // e.g., 100 frames with no activity
        bool isEmptyAndInvalid = (!group.isValid && group.participatingWormIds.isEmpty());
        bool isEmptyAndOld = (group.participatingWormIds.isEmpty() && (currentFrameNumber - group.lastFrameActive > 10));


        if (isEmptyAndInvalid || (isStale && group.participatingWormIds.isEmpty()) || isEmptyAndOld) {
            groupIdsToRemove.append(it.key());
        } else if (group.isValid && group.participatingWormIds.isEmpty() && !isEmptyAndOld) {
            // If it just became empty but isn't old yet, mark it invalid.
            // It will be cleaned up if it remains empty and invalid.
            group.isValid = false;
            group.lastUpdateTime = QDateTime::currentDateTime(); // Mark that we processed it
            qDebug() << "TM: Merge group" << group.uniqueMergeId << "became empty. Marked invalid.";
        }
    }

    if (!groupIdsToRemove.isEmpty()) {
        qDebug() << "TM: Cleaning up" << groupIdsToRemove.size() << "stale/invalid merge groups.";
        for (int groupId : groupIdsToRemove) {
            m_activeMergeGroups.remove(groupId);
            // Remove from m_frameToActiveMergeGroupIds for all frames it might have been in
            // This is a bit inefficient; better to track frames per group or manage lifecycle more tightly.
            // For now, iterate relevant recent frames.
            for (int i = 0; i < 150; ++i) { // Check recent frames
                int frameKey = currentFrameNumber - i;
                if (frameKey < 0) break;
                if (m_frameToActiveMergeGroupIds.contains(frameKey)) {
                    m_frameToActiveMergeGroupIds[frameKey].remove(groupId);
                    if (m_frameToActiveMergeGroupIds[frameKey].isEmpty()) {
                        m_frameToActiveMergeGroupIds.remove(frameKey);
                    }
                }
            }
            // Also ensure no worms point to this groupId anymore
            for(auto it_worm_group = m_wormToCurrentMergeGroupId.begin(); it_worm_group != m_wormToCurrentMergeGroupId.end(); ) {
                if(it_worm_group.value() == groupId) {
                    it_worm_group = m_wormToCurrentMergeGroupId.erase(it_worm_group);
                } else {
                    ++it_worm_group;
                }
            }
        }
    }
}


double TrackingManager::calculateIoU(const QRectF& r1, const QRectF& r2) const {
    // QRectF doesn't exist directly, assuming QRectF for internal logic
    // If r1, r2 are QRectF:
    QRectF qr1(r1.x(), r1.y(), r1.width(), r1.height());
    QRectF qr2(r2.x(), r2.y(), r2.width(), r2.height());

    QRectF intersection = qr1.intersected(qr2);
    double intersectionArea = intersection.width() * intersection.height();

    if (intersectionArea <= 0) return 0.0;

    double unionArea = (qr1.width() * qr1.height()) + (qr2.width() * qr2.height()) - intersectionArea;

    return unionArea > 0 ? (intersectionArea / unionArea) : 0.0;
}


void TrackingManager::handleWormTrackerFinished() {
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if (!finishedTracker) {
        qWarning() << "TrackingManager: handleWormTrackerFinished called by non-WormTracker sender.";
        QMutexLocker locker(&m_dataMutex);
        m_finishedTrackersCount++;
        locker.unlock();
        checkForAllTrackersFinished();
        return;
    }

    int conceptualWormId = finishedTracker->getWormId(); // Conceptual ID
    // Actual tracker instance might be forward or backward
    // We need to remove it from m_wormTrackersList and its specific map

    qDebug() << "TrackingManager: WormTracker for conceptual worm ID" << conceptualWormId
             << "(Dir:" << (finishedTracker->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd")
             << ", Ptr: " << finishedTracker << ") reported finished.";

    QMutexLocker locker(&m_dataMutex);
    m_wormTrackersList.removeOne(finishedTracker);
    if(finishedTracker->getDirection() == WormTracker::TrackingDirection::Forward) {
        m_wormIdToForwardTrackerInstanceMap.remove(conceptualWormId);
    } else {
        m_wormIdToBackwardTrackerInstanceMap.remove(conceptualWormId);
    }

    m_individualTrackerProgress.remove(finishedTracker);
    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: Total finished trackers:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;
    locker.unlock();

    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerError(int reportingWormId, QString errorMessage) {
    // reportingWormId is conceptual ID
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    QString trackerInfo = errorTracker ? QString("(Ptr: %1, Dir: %2)").arg(reinterpret_cast<quintptr>(errorTracker)).arg(errorTracker->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd") : "";

    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Error from tracker for worm ID" << reportingWormId << trackerInfo << "during/after cancel/stop:" << errorMessage;
        if (errorTracker && m_wormTrackersList.contains(errorTracker)) {
            m_wormTrackersList.removeOne(errorTracker);
            m_individualTrackerProgress.remove(errorTracker);
            m_finishedTrackersCount++;
        }
        // No need to update m_wormIdTo...Map here as it's part of cancel/cleanup flow
        locker.unlock();
        updateOverallProgress();
        checkForAllTrackersFinished();
        return;
    }
    if (!m_isTrackingRunning) {
        qDebug() << "TrackingManager: Error from tracker for worm ID" << reportingWormId << trackerInfo << "but tracking not running:" << errorMessage;
        return; // Mutex will unlock
    }
    locker.unlock(); // Unlock before emitting

    qWarning() << "TrackingManager: Error from tracker for worm ID" << reportingWormId << trackerInfo << ":" << errorMessage;

    locker.relock(); // Relock to modify shared members
    if (errorTracker && m_wormTrackersList.contains(errorTracker)) {
        m_wormTrackersList.removeOne(errorTracker);
        m_individualTrackerProgress.remove(errorTracker);
        if(errorTracker->getDirection() == WormTracker::TrackingDirection::Forward) {
            m_wormIdToForwardTrackerInstanceMap.remove(reportingWormId);
        } else {
            m_wormIdToBackwardTrackerInstanceMap.remove(reportingWormId);
        }
        m_finishedTrackersCount++;
    } else if (!errorTracker && m_expectedTrackersToFinish > m_finishedTrackersCount) {
        // Potentially an error from a tracker that was already removed or a logic issue
        qWarning() << "  Error from unknown sender for worm ID" << reportingWormId << ", but trackers still expected. Processing as if one less tracker expected.";
        // This case is tricky; if sender is null, we don't know which tracker to remove.
        // For now, we'll assume it contributes to finished count if we can't identify it.
        // This might lead to premature finish if not handled carefully.
        // A better approach would be to ensure errorTracker is always valid or have another way to identify.
    }
    locker.unlock();

    emit trackingStatusUpdate(QString("Error with tracker for worm %1: %2. Trying to continue...").arg(reportingWormId).arg(errorMessage.left(50)));
    updateOverallProgress();
    checkForAllTrackersFinished();
}


void TrackingManager::handleWormTrackerProgress(int /*wormId*/, int percentDone) {
    // wormId is conceptual, sender is the actual tracker instance
    if (m_cancelRequested || !m_isTrackingRunning) return;
    WormTracker* tracker = qobject_cast<WormTracker*>(sender());

    QMutexLocker locker(&m_dataMutex);
    if (tracker) {
        if (m_individualTrackerProgress.contains(tracker)) { // Check if tracker is still considered active
            m_individualTrackerProgress[tracker] = percentDone;
            locker.unlock(); // Unlock before calling updateOverallProgress if it emits signals
            updateOverallProgress();
            return;
        }
    }
    // If tracker is null or not in map, do nothing (it might have finished/errored out)
}

void TrackingManager::launchWormTrackers() {
    // ... (Your existing launchWormTrackers logic)
    // IMPORTANT: Update how trackers are stored if you need to signal them back.
    // For example, when creating trackers:
    // m_wormIdToTrackerInstanceMap[wormId] = fwdTracker;
    // m_wormIdToReverseTrackerInstanceMap[wormId] = bwdTracker;
    // And ensure m_wormTrackersList is populated with all tracker instances.
    if (m_initialWormInfos.empty()) {
        emit trackingStatusUpdate("No worms selected for tracking.");
        emit trackingFinishedSuccessfully("");
        m_isTrackingRunning = false;
        return;
    }

    m_expectedTrackersToFinish = 0;
    m_finishedTrackersCount = 0;
    m_individualTrackerProgress.clear();
    m_wormIdToForwardTrackerInstanceMap.clear();
    m_wormIdToBackwardTrackerInstanceMap.clear();


    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        if (!m_processedForwardFrames.empty() || m_keyFrameNum == m_totalFramesInVideo - 1) {
            WormTracker* fwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
            fwdTracker->setFrames(&m_processedForwardFrames);
            QThread* fwdThread = new QThread(this);
            fwdTracker->moveToThread(fwdThread);
            fwdTracker->setProperty("wormId", wormId); // For debug
            fwdTracker->setProperty("direction", "Forward"); // For debug

            connect(fwdThread, &QThread::started, fwdTracker, &WormTracker::startTracking);
            connect(fwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(fwdThread, &QThread::finished, fwdTracker, &QObject::deleteLater);
            connect(fwdThread, &QThread::finished, this, [this, fwdThread, wormId](){
                QMutexLocker locker(&m_dataMutex);
                m_trackerThreads.removeOne(fwdThread);
                qDebug() << "Forward Tracker Thread for worm" << wormId << "finished and removed from list. Remaining tracker threads:" << m_trackerThreads.count();
            });

            connect(fwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(fwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(fwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);

            m_wormTrackersList.append(fwdTracker);
            m_wormIdToForwardTrackerInstanceMap[wormId] = fwdTracker; // Store forward tracker instance
            m_trackerThreads.append(fwdThread);
            m_individualTrackerProgress[fwdTracker] = 0;
            fwdThread->start();
            m_expectedTrackersToFinish++;
        } else {
            qDebug() << "TM: Skipping Forward Tracker for worm" << wormId << "due to no forward frames and keyframe not being the last frame.";
        }

        if (!m_processedReversedFrames.empty() || m_keyFrameNum == 0) {
            WormTracker* bwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
            bwdTracker->setFrames(&m_processedReversedFrames);
            QThread* bwdThread = new QThread(this);
            bwdTracker->moveToThread(bwdThread);
            bwdTracker->setProperty("wormId", wormId); // For debug
            bwdTracker->setProperty("direction", "Backward"); // For debug

            connect(bwdThread, &QThread::started, bwdTracker, &WormTracker::startTracking);
            connect(bwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(bwdThread, &QThread::finished, bwdTracker, &QObject::deleteLater);
            connect(bwdThread, &QThread::finished, this, [this, bwdThread, wormId](){
                QMutexLocker locker(&m_dataMutex);
                m_trackerThreads.removeOne(bwdThread);
                qDebug() << "Backward Tracker Thread for worm" << wormId << "finished and removed from list. Remaining tracker threads:" << m_trackerThreads.count();
            });

            connect(bwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(bwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(bwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);

            m_wormTrackersList.append(bwdTracker);
            m_wormIdToBackwardTrackerInstanceMap[wormId] = bwdTracker; // Store reverse tracker instance
            m_trackerThreads.append(bwdThread);
            m_individualTrackerProgress[bwdTracker] = 0;
            bwdThread->start();
            m_expectedTrackersToFinish++;
        } else {
            qDebug() << "TM: Skipping Backward Tracker for worm" << wormId << "due to no backward frames and keyframe not being the first frame.";
        }
    }
    // ... (rest of your launchWormTrackers logic)
    if (m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) {
        qWarning() << "TM: No trackers were launched despite having initial worm infos. Check keyframe position and frame processing.";
        emit trackingFailed("No trackers could be launched. Video might be too short or keyframe at an extreme end where no tracking frames are available.");
        m_isTrackingRunning = false;
        if (m_pauseResolutionTimer && m_pauseResolutionTimer->isActive()) {
            m_pauseResolutionTimer->stop();
        }
    } else if (m_expectedTrackersToFinish > 0) {
        emit trackingStatusUpdate(QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));
    }
}

void TrackingManager::updateOverallProgress() {
    // ... (Your existing updateOverallProgress logic, ensure it's thread-safe if accessing shared members directly)
    // This function is typically called from slots that are already mutex-protected or from main thread.
    // If called from other contexts, ensure m_individualTrackerProgress access is safe.
    // For now, assuming calls are from contexts that don't conflict or are already locked.

    if (!m_isTrackingRunning && !m_cancelRequested) { // If not running and not cancelled, progress is 0
        emit overallTrackingProgress(0);
        return;
    }

    double totalProgressValue = 0.0;
    double videoProcWeight = 0.10; // Weight for video processing part
    double trackersWeight = 0.90;  // Weight for actual tracking part

    totalProgressValue += (static_cast<double>(m_videoProcessingProgress) / 100.0) * videoProcWeight;

    // Tracker progress calculation needs to be careful with shared data
    double overallTrackerPercentage = 0.0;
    { // Scope for mutex if direct access to m_individualTrackerProgress values
        QMutexLocker locker(&m_dataMutex); // Protect access to progress maps and counts
        if (m_expectedTrackersToFinish > 0) {
            double sumOfIndividualProgress = 0;
            // m_individualTrackerProgress stores WormTracker* -> int.
            // Iterate through its values.
            for (int progress : m_individualTrackerProgress.values()) {
                sumOfIndividualProgress += progress;
            }
            // Contribution from already finished trackers (they are at 100%)
            // This count (m_finishedTrackersCount) should be accurate.
            double finishedContribution = static_cast<double>(m_finishedTrackersCount) * 100.0;

            // Total points achieved by trackers still in m_individualTrackerProgress + finished trackers
            double currentTrackerPoints = sumOfIndividualProgress; // sumOfIndividualProgress is only for *active* trackers
                // It should be: sum of progress of *active* trackers
                // + 100 * num_finished_trackers

            // Re-evaluate: m_individualTrackerProgress only contains *active* trackers.
            // So, sumOfIndividualProgress is correct for those.
            // m_finishedTrackersCount is the number of trackers that have completed.
            // Total possible points = m_expectedTrackersToFinish * 100
            // Points achieved = (sum of progress of active trackers) + (m_finishedTrackersCount * 100)

            double totalActiveProgress = 0;
            for(int progress : m_individualTrackerProgress.values()){
                totalActiveProgress += progress;
            }
            double currentPointsFromTrackers = totalActiveProgress + (static_cast<double>(m_finishedTrackersCount) * 100.0);
            double maxPointsFromTrackers = static_cast<double>(m_expectedTrackersToFinish) * 100.0;


            if (maxPointsFromTrackers > 0) {
                overallTrackerPercentage = currentPointsFromTrackers / maxPointsFromTrackers;
            }
            overallTrackerPercentage = qBound(0.0, overallTrackerPercentage, 1.0); // Ensure it's between 0 and 1
        } else if (m_videoProcessingProgress == 100 && m_initialWormInfos.empty()) { // No worms to track, but video processed
            overallTrackerPercentage = 1.0; // Trackers part is "complete" as there's nothing to do
        } else if (m_videoProcessingProgress == 100 && m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()){
            // Video processed, worms were expected, but no trackers launched (e.g. keyframe issue)
            // Tracker part is effectively 0 or failed. Let's say 0 for progress.
            overallTrackerPercentage = 0.0;
        }

    } // Mutex unlocks

    totalProgressValue += overallTrackerPercentage * trackersWeight;
    int finalProgressPercentage = qBound(0, static_cast<int>(totalProgressValue * 100.0), 100);
    emit overallTrackingProgress(finalProgressPercentage);
}


void TrackingManager::checkForAllTrackersFinished() {
    // ... (Your existing checkForAllTrackersFinished logic)
    // Ensure m_isTrackingRunning and m_cancelRequested are accessed safely if needed,
    // though this is usually called when these states are already determined.
    QMutexLocker locker(&m_dataMutex); // Protect counts and m_isTrackingRunning
    qDebug() << "TM: Checking if all trackers finished. Finished:" << m_finishedTrackersCount
             << "Expected:" << m_expectedTrackersToFinish << "Running:" << m_isTrackingRunning
             << "Trackers in list:" << m_wormTrackersList.count();

    bool allDoneOrCancelled = false;
    if (m_isTrackingRunning && (m_finishedTrackersCount >= m_expectedTrackersToFinish)) {
        allDoneOrCancelled = true;
    } else if (m_cancelRequested && m_finishedTrackersCount >= m_expectedTrackersToFinish) {
        // If cancelled, we also consider it done when all expected trackers report back (even if fewer than original expected due to errors during cancel)
        allDoneOrCancelled = true;
    }


    if (allDoneOrCancelled) {
        bool wasCancelled = m_cancelRequested; // Capture before modifying m_isTrackingRunning
        m_isTrackingRunning = false; // Stop tracking state
        if (m_pauseResolutionTimer && m_pauseResolutionTimer->isActive()) {
            m_pauseResolutionTimer->stop();
        }
        locker.unlock(); // Unlock before emitting signals

        if (wasCancelled) {
            qDebug() << "TrackingManager: All trackers accounted for after CANCELLATION request.";
            // Consolidate whatever tracks were made
            m_finalTracks.clear();
            for (WormObject* worm : m_wormObjectsMap.values()) { // m_wormObjectsMap access should be fine if not modified elsewhere
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
        } else { // Finished normally
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
    } else if (!m_isTrackingRunning && m_cancelRequested) {
        // This case handles if tracking was stopped by an error, then cancel was called.
        locker.unlock();
        qDebug() << "TM: checkForAllTrackersFinished - Tracking was already stopped and cancel was requested.";
        emit trackingCancelled(); // Ensure cancel signal is emitted
    }
    // If not allDoneOrCancelled and m_isTrackingRunning is false (e.g. due to an error),
    // the trackingFailed signal should have already been emitted.
}

bool TrackingManager::outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFilePath) const {
    // ... (Your existing outputTracksToCsv logic - seems fine)
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
    outStream << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight,Quality\n"; // Added Quality
    for (auto const& [wormId, trackPoints] : tracks) {
        for (const Tracking::WormTrackPoint& point : trackPoints) {
            outStream << wormId << ","
                      << point.frameNumberOriginal << ","
                      << QString::number(point.position.x, 'f', 4) << ","
                      << QString::number(point.position.y, 'f', 4) << ","
                      << QString::number(point.roi.x(), 'f', 2) << ","
                      << QString::number(point.roi.y(), 'f', 2) << ","
                      << QString::number(point.roi.width(), 'f', 2) << ","
                      << QString::number(point.roi.height(), 'f', 2) << ","
                      << static_cast<int>(point.quality) // Output quality as int
                      << "\n";
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
