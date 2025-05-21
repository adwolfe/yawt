// trackingmanager.cpp
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <algorithm>  // For std::min, std::sort, std::transform etc.
#include <numeric>    // For std::iota if used for assignments
#include <QLineF>
#include <QSet>       // For QSet operations

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
    qRegisterMetaType<TrackingHelper::DetectedBlob>("TrackingHelper::DetectedBlob");
    qRegisterMetaType<WormTracker::TrackerState>("WormTracker::TrackerState");
    qRegisterMetaType<WormTracker::TrackingDirection>("WormTracker::TrackingDirection");
}

TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this
             << ") DESTRUCTOR - START cleaning up...";
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting "
                    "cancel.";
        cancelTracking();
        QThread::msleep(200); // Allow time for signals
    }
    cleanupThreadsAndObjects();
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
    cleanupThreadsAndObjects();
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
    m_cancelRequested = false;
    m_videoProcessingProgress = 0;
    m_finishedTrackersCount = 0;
    m_expectedTrackersToFinish = 0;
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameInfos.clear();
    m_mergedTrackerGroups.clear(); // Changed map
    m_trackerToMergeGroupMap.clear(); // Changed map

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

    if (m_cancelRequested) {
        qDebug() << "TM: Cancellation already in progress.";
        return;
    }
    m_cancelRequested = true;
    emit trackingStatusUpdate("Cancellation requested...");
    qDebug() << "TM: Cancellation flag set to true.";

    if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
        qDebug() << "TM: Requesting video processor thread ("
                 << m_videoProcessorThread << ") interruption.";
        m_videoProcessorThread->requestInterruption();
    }

    QList<WormTracker*> trackersToStop = m_wormTrackers; // Iterate over a copy
    for (WormTracker* tracker : trackersToStop) {
        if (tracker) {
            QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
        }
    }
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
                m_videoProcessorThread->terminate();
                m_videoProcessorThread->wait();
            } else {
                qDebug() << "  Video processor thread (" << m_videoProcessorThread
                         << ") finished gracefully.";
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
    m_wormTrackers.clear();

    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    m_processedForwardFrames.clear();
    std::vector<cv::Mat>().swap(m_processedForwardFrames);
    m_processedReversedFrames.clear();
    std::vector<cv::Mat>().swap(m_processedReversedFrames);

    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameInfos.clear();
    m_mergedTrackerGroups.clear(); // Changed map
    m_trackerToMergeGroupMap.clear(); // Changed map

    m_isTrackingRunning = false;
    qDebug() << "TrackingManager (" << this
             << "): cleanupThreadsAndObjects - FINISHED";
}

QList<WormTracker*> TrackingManager::findTrackersForWorm(int conceptualWormId) {
    QList<WormTracker*> foundTrackers;
    for (WormTracker* tracker : qAsConst(m_wormTrackers)) {
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
    if (!m_isTrackingRunning) {
        qWarning() << "TM: handleInitialProcessingComplete, but tracking is not marked as running. Aborting tracker launch.";
        cleanupThreadsAndObjects(); // Ensure cleanup if state is inconsistent
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
        return;
    }
    if (!m_isTrackingRunning) {
        qWarning() << "TM: Video processing error (" << errorMessage << ") but tracking not marked as running.";
        return;
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

    m_expectedTrackersToFinish = 0; // Will be incremented for each launched tracker
    m_finishedTrackersCount = 0;
    m_individualTrackerProgress.clear();


    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        if (!m_processedForwardFrames.empty() || m_keyFrameNum == m_totalFramesInVideo -1) {
            WormTracker* fwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
            fwdTracker->setFrames(&m_processedForwardFrames);
            QThread* fwdThread = new QThread(this);
            fwdTracker->moveToThread(fwdThread);
            fwdTracker->setProperty("conceptualWormId", wormId); // Store conceptual ID
            fwdTracker->setProperty("direction", static_cast<int>(WormTracker::TrackingDirection::Forward));


            connect(fwdThread, &QThread::started, fwdTracker, &WormTracker::startTracking, Qt::QueuedConnection);
            connect(fwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
            connect(fwdThread, &QThread::finished, fwdTracker, &QObject::deleteLater, Qt::UniqueConnection);
            connect(fwdThread, &QThread::finished, this, [this, fwdThread, wormId](){
                m_trackerThreads.removeOne(fwdThread);
                qDebug() << "Forward Tracker Thread for conceptual worm" << wormId << "finished and removed from list.";
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
            m_expectedTrackersToFinish++;
        } else {
            qDebug() << "TM: Skipping Forward Tracker for conceptual worm" << wormId << "due to no forward frames.";
        }

        if (!m_processedReversedFrames.empty() || m_keyFrameNum == 0) {
            WormTracker* bwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
            bwdTracker->setFrames(&m_processedReversedFrames);
            QThread* bwdThread = new QThread(this);
            bwdTracker->moveToThread(bwdThread);
            bwdTracker->setProperty("conceptualWormId", wormId);
            bwdTracker->setProperty("direction", static_cast<int>(WormTracker::TrackingDirection::Backward));


            connect(bwdThread, &QThread::started, bwdTracker, &WormTracker::startTracking, Qt::QueuedConnection);
            connect(bwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
            connect(bwdThread, &QThread::finished, bwdTracker, &QObject::deleteLater, Qt::UniqueConnection);
            connect(bwdThread, &QThread::finished, this, [this, bwdThread, wormId](){
                m_trackerThreads.removeOne(bwdThread);
                qDebug() << "Backward Tracker Thread for conceptual worm" << wormId << "finished and removed from list.";
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
            m_expectedTrackersToFinish++;
        } else {
            qDebug() << "TM: Skipping Backward Tracker for conceptual worm" << wormId << "due to no backward frames.";
        }
    }
    emit trackingStatusUpdate(QString("Launched %1 worm trackers...").arg(m_expectedTrackersToFinish));

    if(m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()){
        qWarning() << "TM: No trackers were launched. Check keyframe position and frame processing.";
        emit trackingFailed("No trackers could be launched. Video might be too short or keyframe at an extreme end.");
        m_isTrackingRunning = false; // Stop the process
        cleanupThreadsAndObjects();
    } else if (m_expectedTrackersToFinish == 0 && m_initialWormInfos.empty()){
        // This case is handled at the start of the function.
    }
}

void TrackingManager::handleWormPositionUpdated(
    int conceptualWormId, // This is the conceptualWormId from the tracker
    int originalFrameNumber, QPointF newPosition, QRectF newRoi,
    int plausibleBlobsFoundInRoi, double primaryBlobArea) {
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormObject* wormObject = m_wormObjectsMap.value(conceptualWormId, nullptr);
    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender());

    if (wormObject && reportingTracker) {
        // Update WormObject if the tracker is not paused and not part of a merge group handled by TM
        if (reportingTracker->getCurrentTrackerState() != WormTracker::TrackerState::PausedForSplit &&
            !m_trackerToMergeGroupMap.contains(reportingTracker)) { // Check if THIS TRACKER INSTANCE is in a merge
            cv::Point2f cvPos(static_cast<float>(newPosition.x()), static_cast<float>(newPosition.y()));
            wormObject->updateTrackPoint(originalFrameNumber, cvPos, newRoi);
            WormTrackPoint lastPt;
            lastPt.frameNumberOriginal = originalFrameNumber;
            lastPt.position = cvPos;
            lastPt.roi = newRoi;
            emit individualWormTrackUpdated(conceptualWormId, lastPt);
        }

        WormFrameInfo info;
        info.position = newPosition;
        info.roi = newRoi;
        info.plausibleBlobsInRoi = plausibleBlobsFoundInRoi;
        info.primaryBlobArea = primaryBlobArea;
        info.reportingTracker = reportingTracker;
        info.isValid = true;

        m_frameInfos[originalFrameNumber][reportingTracker] = info;

        while (m_frameInfos.size() > m_frameInfoHistorySize && !m_frameInfos.isEmpty()) {
            m_frameInfos.remove(m_frameInfos.firstKey());
        }
        processFrameDataForMergesAndSplits(originalFrameNumber);
    } else {
        qWarning() << "TrackingManager: handleWormPositionUpdated from unknown tracker or for unknown conceptualWormId:" << conceptualWormId;
    }
}

void TrackingManager::handleWormSplitDetectedAndPaused(
    int conceptualWormId, // Conceptual ID of the worm whose tracker paused
    int originalFrameNumber,
    const QList<TrackingHelper::DetectedBlob>& detectedBlobs) {

    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* pausedTrackerInstance = qobject_cast<WormTracker*>(sender());
    if (!pausedTrackerInstance) {
        qWarning() << "TM: Split detected, but sender is not a WormTracker. ConceptualWormID from signal:" << conceptualWormId;
        return;
    }
    if (pausedTrackerInstance->getWormId() != conceptualWormId) {
        qWarning() << "TM: Split detected, sender WormTracker ID" << pausedTrackerInstance->getWormId()
        << "does not match signal's conceptualWormId" << conceptualWormId << ". Ignoring.";
        return;
    }

    qDebug() << "TM: Conceptual Worm ID" << conceptualWormId
             << "(Paused Tracker:" << pausedTrackerInstance << ", Dir:" << pausedTrackerInstance->getDirection() << ")"
             << "reported SPLIT and PAUSED at frame" << originalFrameNumber
             << "with" << detectedBlobs.count() << "new blobs.";
    emit trackingStatusUpdate(
        QString("Conceptual Worm %1 (Tracker %2, Dir: %3) paused for split resolution at frame %4.")
            .arg(conceptualWormId)
            .arg(QString::number(reinterpret_cast<quintptr>(pausedTrackerInstance), 16)) // Unique ID for tracker instance
            .arg(pausedTrackerInstance->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd")
            .arg(originalFrameNumber));

    QSet<WormTracker*> trackersInMergeGroup;
    WormTracker* representativeTracker = m_trackerToMergeGroupMap.value(pausedTrackerInstance, nullptr);

    if (representativeTracker && m_mergedTrackerGroups.contains(representativeTracker)) {
        trackersInMergeGroup = m_mergedTrackerGroups.value(representativeTracker);
        qDebug() << "  Split involves merge group represented by Tracker" << representativeTracker << "with Trackers:" << trackersInMergeGroup;
    } else {
        qWarning() << "TM: Paused Tracker" << pausedTrackerInstance << "was not in a formal merge group. Assuming it was tracking solo or its partner.";
        trackersInMergeGroup.insert(pausedTrackerInstance);
        // Find its partner tracker (opposite direction, same conceptualWormId)
        QList<WormTracker*> partnerTrackers = findTrackersForWorm(conceptualWormId);
        for(WormTracker* partner : partnerTrackers) {
            if (partner != pausedTrackerInstance) {
                trackersInMergeGroup.insert(partner); // Add partner if it exists and is active
                qDebug() << "  Also considering partner tracker:" << partner;
            }
        }
    }

    if (detectedBlobs.isEmpty()) {
        qWarning() << "TM: Split detected for trackers" << trackersInMergeGroup << "but no blobs provided. Trackers may be lost.";
        for (WormTracker* trackerToStop : trackersInMergeGroup) {
            if (trackerToStop && (trackerToStop == pausedTrackerInstance || trackerToStop->getCurrentTrackerState() == WormTracker::TrackerState::TrackingMerged || trackerToStop->getCurrentTrackerState() == WormTracker::TrackerState::PausedForSplit) ) {
                WormObject* wo = m_wormObjectsMap.value(trackerToStop->getWormId(), nullptr);
                if (wo) wo->setState(WormObject::TrackingState::Lost);
                QMetaObject::invokeMethod(trackerToStop, "stopTracking", Qt::QueuedConnection);
            }
        }
        if (representativeTracker) m_mergedTrackerGroups.remove(representativeTracker);
        for (WormTracker* trackerInGroup : trackersInMergeGroup) m_trackerToMergeGroupMap.remove(trackerInGroup);
        return;
    }

    QMap<WormTracker*, QPointF> lastPositionsForTrackers;
    for (WormTracker* trackerInGroup : trackersInMergeGroup) {
        if (!trackerInGroup) continue;
        // Try to get the most recent position from m_frameInfos for this specific tracker
        bool foundPos = false;
        for (int frameOffset = 0; frameOffset <= m_frameInfoHistorySize; ++frameOffset) {
            int checkFrame = originalFrameNumber - frameOffset; // Check current and recent past frames
            if (m_frameInfos.contains(checkFrame) && m_frameInfos.value(checkFrame).contains(trackerInGroup)) {
                lastPositionsForTrackers[trackerInGroup] = m_frameInfos.value(checkFrame).value(trackerInGroup).position;
                foundPos = true;
                break;
            }
        }
        if (!foundPos) { // Fallback to WormObject for conceptual worm
            WormObject* wo = m_wormObjectsMap.value(trackerInGroup->getWormId(), nullptr);
            if (wo) {
                lastPositionsForTrackers[trackerInGroup] = QPointF(wo->getCurrentPosition().x, wo->getCurrentPosition().y);
            } else {
                qWarning() << "TM: Could not get last known position for Tracker" << trackerInGroup;
            }
        }
    }

    if (lastPositionsForTrackers.isEmpty() && !trackersInMergeGroup.isEmpty()) {
        qWarning() << "TM: Could not retrieve any last known positions for trackers in the split group:" << trackersInMergeGroup << ". Cannot assign blobs.";
        // Mark as lost, similar to above
        for (WormTracker* trackerToStop : trackersInMergeGroup) { /* ... stop logic ... */ }
        if (representativeTracker) m_mergedTrackerGroups.remove(representativeTracker);
        for (WormTracker* trackerInGroup : trackersInMergeGroup) m_trackerToMergeGroupMap.remove(trackerInGroup);
        return;
    }

    QList<TrackingHelper::DetectedBlob> availableBlobs = detectedBlobs;
    QMap<WormTracker*, TrackingHelper::DetectedBlob> assignments;
    QSet<int> assignedBlobIndices;

    QList<WormTracker*> trackersToAssign = lastPositionsForTrackers.keys();
    // Optional: Sort trackersToAssign for deterministic assignment (e.g., by pointer address or conceptual ID then direction)
    std::sort(trackersToAssign.begin(), trackersToAssign.end(), [](WormTracker* a, WormTracker* b){
        if (a->getWormId() != b->getWormId()) return a->getWormId() < b->getWormId();
        return a->getDirection() < b->getDirection();
    });


    for (WormTracker* trackerToAssign : trackersToAssign) {
        QPointF lastPos = lastPositionsForTrackers.value(trackerToAssign);
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
            assignments[trackerToAssign] = availableBlobs.at(bestBlobIdx);
            assignedBlobIndices.insert(bestBlobIdx);
            qDebug() << "  Assigning blob" << bestBlobIdx << "(Centroid:" << availableBlobs.at(bestBlobIdx).centroid
                     << ") to Tracker" << trackerToAssign << "(ConceptID:" << trackerToAssign->getWormId() << ", Dir:" << trackerToAssign->getDirection() << ")";
        } else {
            qDebug() << "  Could not find an unassigned blob for Tracker" << trackerToAssign << ". It might be lost.";
            WormObject* wo = m_wormObjectsMap.value(trackerToAssign->getWormId(), nullptr);
            if (wo) wo->setState(WormObject::TrackingState::Lost);
            QMetaObject::invokeMethod(trackerToAssign, "stopTracking", Qt::QueuedConnection);
        }
    }

    for (WormTracker* assignedTracker : assignments.keys()) {
        TrackingHelper::DetectedBlob assignedBlob = assignments.value(assignedTracker);
        qDebug() << "  Instructing Tracker" << assignedTracker << "(ConceptID:" << assignedTracker->getWormId()
                 << ", Dir:" << assignedTracker->getDirection() << ") to resume with blob at" << assignedBlob.centroid;
        QMetaObject::invokeMethod(assignedTracker, "resumeTrackingWithNewTarget", Qt::QueuedConnection,
                                  Q_ARG(TrackingHelper::DetectedBlob, assignedBlob));
        WormObject* wo = m_wormObjectsMap.value(assignedTracker->getWormId(), nullptr);
        if (wo) wo->setState(WormObject::TrackingState::Tracking); // Back to tracking
    }

    if (representativeTracker) {
        m_mergedTrackerGroups.remove(representativeTracker);
    }
    for (WormTracker* trackerInGroup : trackersInMergeGroup) {
        m_trackerToMergeGroupMap.remove(trackerInGroup);
        qDebug() << "  Tracker" << trackerInGroup << "(ConceptID:" << trackerInGroup->getWormId() << ") removed from merge group tracking.";
    }
    emit trackingStatusUpdate(QString("Split resolved. Resuming individual tracking."));
}


void TrackingManager::processFrameDataForMergesAndSplits(int frameNumber) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    if (!m_frameInfos.contains(frameNumber)) return;

    const QMap<WormTracker*, WormFrameInfo>& currentFrameTrackerData = m_frameInfos.value(frameNumber);
    QList<WormTracker*> activeTrackersInFrame = currentFrameTrackerData.keys();

    for (int i = 0; i < activeTrackersInFrame.size(); ++i) {
        WormTracker* trackerA = activeTrackersInFrame.at(i);
        if (!trackerA || !m_wormTrackers.contains(trackerA)) continue; // Check if tracker is still active
        const WormFrameInfo& infoA = currentFrameTrackerData.value(trackerA);

        if (m_trackerToMergeGroupMap.contains(trackerA) || infoA.reportingTracker->getCurrentTrackerState() == WormTracker::TrackerState::PausedForSplit) {
            continue;
        }

        for (int j = i + 1; j < activeTrackersInFrame.size(); ++j) {
            WormTracker* trackerB = activeTrackersInFrame.at(j);
            if (!trackerB || !m_wormTrackers.contains(trackerB)) continue;
            const WormFrameInfo& infoB = currentFrameTrackerData.value(trackerB);

            if (trackerA->getWormId() == trackerB->getWormId()) continue; // Same conceptual worm (fwd vs bwd)
            if (trackerA->getDirection() != trackerB->getDirection()) continue; // Different directions, don't merge
            if (m_trackerToMergeGroupMap.contains(trackerB) || infoB.reportingTracker->getCurrentTrackerState() == WormTracker::TrackerState::PausedForSplit) {
                continue;
            }

            QRectF searchRoiA = infoA.roi;
            QRectF searchRoiB = infoB.roi;
            // Ideally, use actual blob BBoxes from WormFrameInfo if available
            // QRectF actualBboxA = infoA.primaryBlobBoundingBox;
            // QRectF actualBboxB = infoB.primaryBlobBoundingBox;

            if (searchRoiA.intersects(searchRoiB)) { // Basic proximity check
                // More robust: check intersection of actual primary blob bounding boxes if available
                // For now, assume if search ROIs intersect and both found blobs, it's a potential merge.
                double minAreaA = trackerA->property("minBlobArea").isValid() ? trackerA->property("minBlobArea").toDouble() : TrackingConstants::DEFAULT_MIN_WORM_AREA;
                double minAreaB = trackerB->property("minBlobArea").isValid() ? trackerB->property("minBlobArea").toDouble() : TrackingConstants::DEFAULT_MIN_WORM_AREA;

                if (infoA.primaryBlobArea > minAreaA * 0.5 && infoB.primaryBlobArea > minAreaB * 0.5) {
                    qDebug() << "TM: Potential MERGE at frame" << frameNumber
                             << "between Tracker" << trackerA << "(ID:" << trackerA->getWormId() << ",Dir:" << trackerA->getDirection() << ")"
                             << "and Tracker" << trackerB << "(ID:" << trackerB->getWormId() << ",Dir:" << trackerB->getDirection() << ")";
                    emit trackingStatusUpdate(QString("Merge detected: Worms %1 and %2 (Dir: %3)")
                                                  .arg(trackerA->getWormId()).arg(trackerB->getWormId())
                                                  .arg(trackerA->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd"));

                    WormTracker* repA = m_trackerToMergeGroupMap.value(trackerA, trackerA);
                    WormTracker* repB = m_trackerToMergeGroupMap.value(trackerB, trackerB);
                    WormTracker* finalRep = (repA < repB) ? repA : repB; // Choose representative (e.g. by pointer address)

                    QSet<WormTracker*> groupA_members = m_mergedTrackerGroups.value(repA, QSet<WormTracker*>() << trackerA);
                    QSet<WormTracker*> groupB_members = m_mergedTrackerGroups.value(repB, QSet<WormTracker*>() << trackerB);
                    QSet<WormTracker*> newMergedGroupMembers = groupA_members + groupB_members;

                    if (repA != finalRep && m_mergedTrackerGroups.contains(repA)) m_mergedTrackerGroups.remove(repA);
                    if (repB != finalRep && m_mergedTrackerGroups.contains(repB)) m_mergedTrackerGroups.remove(repB);
                    m_mergedTrackerGroups[finalRep] = newMergedGroupMembers;

                    QRectF encompassingRoi;
                    for (WormTracker* memberTracker : newMergedGroupMembers) {
                        m_trackerToMergeGroupMap[memberTracker] = finalRep;
                        WormObject* wo = m_wormObjectsMap.value(memberTracker->getWormId(), nullptr);
                        if (wo) wo->setState(WormObject::TrackingState::Merged, finalRep->getWormId()); // Store conceptual ID of rep

                        // Update encompassing ROI based on this member's current info
                        if (currentFrameTrackerData.contains(memberTracker)) {
                            const WormFrameInfo& memberInfo = currentFrameTrackerData.value(memberTracker);
                            // Ideally use actual blob bbox from memberInfo. For now, search ROI.
                            QRectF blobEffectiveRoi = memberInfo.roi; // Fallback to search ROI
                            // If WormFrameInfo had primaryBlobBoundingBox:
                            // if (memberInfo.primaryBlobBoundingBox.isValid()) blobEffectiveRoi = memberInfo.primaryBlobBoundingBox;

                            if (encompassingRoi.isNull()) encompassingRoi = blobEffectiveRoi;
                            else encompassingRoi = encompassingRoi.united(blobEffectiveRoi);
                        }
                    }
                    qDebug() << "  Updated merge group. Representative Tracker:" << finalRep << "Members:" << newMergedGroupMembers;

                    if (!encompassingRoi.isNull()) {
                        encompassingRoi.adjust(-5, -5, 5, 5); // Padding
                        encompassingRoi.setX(qMax(0.0, encompassingRoi.x()));
                        encompassingRoi.setY(qMax(0.0, encompassingRoi.y()));
                        if(encompassingRoi.right() > m_videoFrameSize.width) encompassingRoi.setRight(m_videoFrameSize.width);
                        if(encompassingRoi.bottom() > m_videoFrameSize.height) encompassingRoi.setBottom(m_videoFrameSize.height);
                    } else {
                        qWarning() << "TM: Could not determine encompassing ROI for merge group" << finalRep;
                        encompassingRoi = searchRoiA.united(searchRoiB); // Fallback
                    }

                    QPointF mergedCentroid = encompassingRoi.center(); // Approximate centroid

                    for (WormTracker* memberTracker : newMergedGroupMembers) {
                        if (memberTracker && memberTracker->getCurrentTrackerState() != WormTracker::TrackerState::PausedForSplit) {
                            QMetaObject::invokeMethod(memberTracker, "confirmTargetIsMerged", Qt::QueuedConnection,
                                                      Q_ARG(int, finalRep->getWormId()), // Conceptual ID of representative
                                                      Q_ARG(QPointF, mergedCentroid),
                                                      Q_ARG(QRectF, encompassingRoi));
                        }
                    }
                    // Break from inner loop (j) as trackerB is now handled
                    // The outer loop (i) will continue, and trackerA will be skipped due to m_trackerToMergeGroupMap check
                    break;
                }
            }
        }
    }
}


void TrackingManager::handleWormStateChanged(int conceptualWormId, WormTracker::TrackerState newState) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender());

    if (reportingTracker) {
        qDebug() << "TM: Conceptual Worm ID" << conceptualWormId
                 << "(Tracker:" << reportingTracker << ", Dir:" << reportingTracker->getDirection()
                 << ") State CHANGED to" << static_cast<int>(newState);
        // Further logic based on state changes can be added here if TM needs to react directly
    } else {
        qWarning() << "TM: handleWormStateChanged from non-WormTracker sender for conceptual ID" << conceptualWormId;
    }
}

void TrackingManager::handleWormTrackerFinished() {
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if (!finishedTracker) {
        qWarning() << "TrackingManager: handleWormTrackerFinished called by non-WormTracker sender. This is unexpected.";
        // Decrement expected count if we can't identify the tracker, to avoid stall
        // This is risky, means something is wrong with signal/slot or object lifetimes
        // m_finishedTrackersCount++;
        // checkForAllTrackersFinished();
        return;
    }

    int conceptualWormId = finishedTracker->getWormId();
    WormTracker::TrackingDirection direction = finishedTracker->getDirection();
    qDebug() << "TrackingManager: WormTracker for conceptual ID" << conceptualWormId
             << "(Dir:" << direction << ", Ptr: " << finishedTracker << ") reported finished.";

    m_wormTrackers.removeOne(finishedTracker); // Remove from active list
    m_individualTrackerProgress.remove(finishedTracker);
    // The tracker's QThread will deleteLater the finishedTracker object.

    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: Total finished trackers:"
             << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;

    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerError(int conceptualWormId, QString errorMessage) {
    if (!m_isTrackingRunning && !m_cancelRequested) { // If tracking already stopped (e.g. previous error) or not cancelling
        qDebug() << "TrackingManager: Error from tracker for conceptual ID" << conceptualWormId << "but tracking not running/not cancelling:" << errorMessage;
        return;
    }
    // If cancelling, errors are expected, log them but don't fail the whole process.
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Error from tracker for conceptual ID" << conceptualWormId << "during cancellation:" << errorMessage;
    } else {
        qWarning() << "TrackingManager: Error from tracker for conceptual ID" << conceptualWormId
                   << ":" << errorMessage;
        emit trackingStatusUpdate(
            QString("Error with tracker for worm %1: %2. Trying to continue...")
                .arg(conceptualWormId).arg(errorMessage.left(50)));
    }

    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    if (errorTracker) {
        m_wormTrackers.removeOne(errorTracker);
        m_individualTrackerProgress.remove(errorTracker);
    }
    m_finishedTrackersCount++; // Count as "finished" to not stall
    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerProgress(int conceptualWormId, int percentDone) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    WormTracker* tracker = qobject_cast<WormTracker*>(sender());
    if (tracker) {
        if (m_individualTrackerProgress.contains(tracker)) { // Check if still active
            m_individualTrackerProgress[tracker] = percentDone;
            updateOverallProgress();
        }
    }
}


void TrackingManager::checkForAllTrackersFinished() {
    qDebug() << "TM: Checking if all trackers finished. Finished:" << m_finishedTrackersCount << "Expected:" << m_expectedTrackersToFinish << "Running:" << m_isTrackingRunning;

    if (m_isTrackingRunning && (m_finishedTrackersCount >= m_expectedTrackersToFinish || (m_expectedTrackersToFinish > 0 && m_wormTrackers.isEmpty())) ) {
        // Condition: Tracking was running, AND (all expected reported finished OR no trackers are left in active list but some were expected)
        if (m_cancelRequested) {
            qDebug() << "TrackingManager: All trackers accounted for after CANCELLATION request.";
            m_finalTracks.clear();
            for (WormObject* worm : qAsConst(m_wormObjectsMap)) {
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

            QString csvOutputPath;
            if (!m_videoPath.isEmpty()) {
                QFileInfo videoInfo(m_videoPath);
                QString baseName = videoInfo.completeBaseName();
                QString outputDir = videoInfo.absolutePath();
                csvOutputPath = QDir(outputDir).filePath(baseName + "_tracks.csv");
            } else {
                csvOutputPath = "worm_tracks.csv";
            }

            bool csvSaved = outputTracksToCsv(m_finalTracks, csvOutputPath);
            if (csvSaved) {
                emit trackingStatusUpdate("Tracking finished. Tracks saved to: " + csvOutputPath);
                emit trackingFinishedSuccessfully(csvOutputPath);
            } else {
                emit trackingStatusUpdate("Tracking finished, but failed to save CSV output.");
                emit trackingFailed("Failed to save CSV output.");
            }
        }
        m_isTrackingRunning = false;
        // cleanupThreadsAndObjects(); // Call cleanup after all processing for this run is done
    } else if (!m_isTrackingRunning) {
        qDebug() << "TM: checkForAllTrackersFinished - Tracking was already stopped (e.g. by error or earlier cancel).";
        if(m_cancelRequested) {
            emit trackingCancelled();
        }
    }
    // If m_isTrackingRunning is true but not all trackers are finished, do nothing yet.
    // This function will be called again when another tracker finishes.
    // If it's the very end of the application, destructor will call cleanup.
}

void TrackingManager::updateOverallProgress() {
    if (!m_isTrackingRunning && !m_cancelRequested) {
        emit overallTrackingProgress(0);
        return;
    }

    double totalProgressValue = 0.0;
    double videoProcWeight = 0.10;
    double trackersWeight = 0.90;

    totalProgressValue += static_cast<double>(m_videoProcessingProgress) * videoProcWeight / 100.0;

    if (m_expectedTrackersToFinish > 0) {
        double sumOfIndividualProgress = 0;
        // Iterate over a copy of keys if map might change, but here it's just reading values
        for (WormTracker* trackerKey : m_individualTrackerProgress.keys()) {
            sumOfIndividualProgress += m_individualTrackerProgress.value(trackerKey, 0);
        }
        double finishedContribution = static_cast<double>(m_finishedTrackersCount) * 100.0;
        double totalTrackerPoints = sumOfIndividualProgress + finishedContribution;
        // Total possible points from trackers = m_expectedTrackersToFinish * 100
        // But sumOfIndividualProgress is only for *active* trackers.
        // So, totalTrackerPoints should be sum of progress of active ones + 100 for each finished one.

        double overallTrackerPercentage = (totalTrackerPoints / (static_cast<double>(m_expectedTrackersToFinish) * 100.0)) * 100.0;
        overallTrackerPercentage = qBound(0.0, overallTrackerPercentage, 100.0);
        totalProgressValue += overallTrackerPercentage * trackersWeight / 100.0;

    } else if (m_videoProcessingProgress == 100) {
        totalProgressValue = 1.0;
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
    for (auto const& [wormId, points] : tracks) {
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

