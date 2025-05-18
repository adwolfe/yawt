// trackingmanager.cpp
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <algorithm>  // For std::min, std::sort etc.
#include <numeric>
#include <QLineF>
#include <set>  // For std::set if needed, QSet is generally fine for Qt int IDs

#include "trackingmanager.h"
#include "wormtracker.h" // For WormTracker class and its enums

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
        cancelTracking();
        QThread::msleep(100); // Give a moment for signals to process
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
    connect(m_videoProcessorThread, &QThread::finished, [this]() {
        qDebug() << "VideoProcessorThread (" << QObject::sender()
        << ") finished. m_videoProcessor (" << m_videoProcessor
        << ") was scheduled for deleteLater.";
        m_videoProcessor = nullptr; // Important: Null out the pointer
    });
    m_videoProcessorThread->start();
}

void TrackingManager::cancelTracking() {
    qDebug() << "TrackingManager (" << this
             << "): cancelTracking called. IsRunning:" << m_isTrackingRunning
             << "CancelRequested:" << m_cancelRequested;
    if (!m_isTrackingRunning && !m_cancelRequested) {
        qDebug() << "TM: Cancel requested but not running or already cancelled.";
        return;
    }
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
    for (WormTracker* tracker : std::as_const(m_wormTrackers)) { // Use std::as_const for C++17
        if (tracker)
            QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
    }
    for (QThread* thread : std::as_const(m_trackerThreads)) { // Use std::as_const for C++17
        if (thread && thread->isRunning()) thread->requestInterruption();
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
            if (!m_videoProcessorThread->wait(1000)) { // Wait for thread to finish
                qWarning() << "  VideoProcessor thread (" << m_videoProcessorThread
                           << ") did not finish gracefully.";
            } else {
                qDebug() << "  Video processor thread (" << m_videoProcessorThread
                         << ") finished gracefully.";
            }
        }
        delete m_videoProcessorThread; // Delete the QThread object
        m_videoProcessorThread = nullptr;
    }
    if (m_videoProcessor) {
        qWarning() << "  TrackingManager: m_videoProcessor was not nulled out by thread finished, "
                   << "or thread did not finish. Setting to nullptr now.";
        m_videoProcessor = nullptr;
    }

    for (QThread* thread : m_trackerThreads) {
        if (thread) {
            if (thread->isRunning()) {
                thread->requestInterruption();
                thread->quit();
                if (!thread->wait(500)) {
                    qWarning() << "  A WormTracker thread (" << thread
                               << ") did not finish gracefully.";
                } else {
                    qDebug() << "  Tracker thread (" << thread
                             << ") finished gracefully.";
                }
            }
            delete thread;
        }
    }
    m_trackerThreads.clear();

    if (!m_wormTrackers.isEmpty()) {
        qWarning() << "  TrackingManager: m_wormTrackers list is not empty after thread cleanup. Count:"
                   << m_wormTrackers.size() << ". This indicates some trackers might not have signaled finished correctly.";
        m_wormTrackers.clear();
    }

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
    m_cancelRequested = false;
    qDebug() << "TrackingManager (" << this
             << "): cleanupThreadsAndObjects - FINISHED";
}

void TrackingManager::handleInitialProcessingComplete(
    const std::vector<cv::Mat>& forwardFrames,
    const std::vector<cv::Mat>& reversedFrames, double fps,
    cv::Size frameSize) {
    qDebug() << "TrackingManager (" << this
             << "): handleInitialProcessingComplete received.";
    if (!m_videoProcessor && !m_cancelRequested) {
        qWarning() << "TM: handleInitialProcessingComplete, but m_videoProcessor "
                      "is null. Race condition or premature deletion?";
    }
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing complete but cancellation requested. Not "
                    "launching trackers.";
        emit trackingCancelled();
        m_isTrackingRunning = false;
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
        emit trackingCancelled();
        m_isTrackingRunning = false;
        return;
    }
    qWarning() << "TM: Video processing error:" << errorMessage;
    emit trackingFailed("Video processing failed: " + errorMessage);
    m_isTrackingRunning = false;
    cleanupThreadsAndObjects();
}

void TrackingManager::handleVideoProcessingProgress(int percentage) {
    if (m_cancelRequested) return;
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
    // May need code here to handle using first or last frame...
    // e.g. if size Reverse or size FWD == 0
    m_expectedTrackersToFinish = static_cast<int>(m_initialWormInfos.size() * 2);
    m_finishedTrackersCount = 0;
    m_individualTrackerProgress.clear();
    emit trackingStatusUpdate(
        QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));

    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        WormTracker* fwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
        fwdTracker->setFrames(&m_processedForwardFrames);
        QThread* fwdThread = new QThread(this);
        fwdTracker->moveToThread(fwdThread);
        fwdTracker->setProperty("wormId", wormId);
        fwdTracker->setProperty("direction", "Forward");

        connect(fwdThread, &QThread::started, fwdTracker, &WormTracker::startTracking, Qt::QueuedConnection);
        connect(fwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
        connect(fwdThread, &QThread::finished, fwdTracker, &QObject::deleteLater, Qt::UniqueConnection);

        connect(fwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(fwdTracker, &WormTracker::splitDetectedAndPaused, this, &TrackingManager::handleWormSplitDetectedAndPaused);
        connect(fwdTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(fwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(fwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
        m_wormTrackers.append(fwdTracker);
        m_trackerThreads.append(fwdThread);
        m_individualTrackerProgress[fwdTracker] = 0;
        fwdThread->start();

        WormTracker* bwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
        bwdTracker->setFrames(&m_processedReversedFrames);
        QThread* bwdThread = new QThread(this);
        bwdTracker->moveToThread(bwdThread);
        bwdTracker->setProperty("wormId", wormId);
        bwdTracker->setProperty("direction", "Backward");

        connect(bwdThread, &QThread::started, bwdTracker, &WormTracker::startTracking, Qt::QueuedConnection);
        connect(bwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
        connect(bwdThread, &QThread::finished, bwdTracker, &QObject::deleteLater, Qt::UniqueConnection);

        connect(bwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(bwdTracker, &WormTracker::splitDetectedAndPaused, this, &TrackingManager::handleWormSplitDetectedAndPaused);
        connect(bwdTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(bwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(bwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
        m_wormTrackers.append(bwdTracker);
        m_trackerThreads.append(bwdThread);
        m_individualTrackerProgress[bwdTracker] = 0;
        bwdThread->start();
    }
}

void TrackingManager::handleWormPositionUpdated(
    int wormId, int originalFrameNumber, QPointF newPosition, QRectF newRoi,
    int plausibleBlobsFoundInRoi, double primaryBlobArea) {
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormObject* wormObject = m_wormObjectsMap.value(wormId, nullptr);
    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender());

    if (wormObject && reportingTracker) {
        if (reportingTracker->getCurrentTrackerState() != WormTracker::TrackerState::PausedAwaitingSplitDecision) {
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

        WormFrameInfo info;
        info.position = newPosition;
        info.roi = newRoi;
        info.plausibleBlobsInRoi = plausibleBlobsFoundInRoi;
        info.primaryBlobArea = primaryBlobArea;
        info.reportingTracker = reportingTracker;
        info.isValid = true;
        m_frameInfos[originalFrameNumber][wormId] = info;

        while (m_frameInfos.size() > m_frameInfoHistorySize && !m_frameInfos.isEmpty()) {
            m_frameInfos.remove(m_frameInfos.firstKey());
        }
        // ***** Temporarily COMMENTED OUT for diagnostics *****
        // processFrameDataForMergesAndSplits(originalFrameNumber);
    } else {
        qWarning() << "TrackingManager: handleWormPositionUpdated from unknown source or null object. WormID:" << wormId;
    }
}

void TrackingManager::handleWormSplitDetectedAndPaused(
    int wormId, int originalFrameNumber,
    const QList<TrackingHelper::DetectedBlob>& detectedBlobs) {
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* reportingTracker = qobject_cast<WormTracker*>(sender());

    if (!reportingTracker) {
        qWarning() << "TrackingManager: handleWormSplitDetectedAndPaused received, but sender is not a WormTracker. WormID:" << wormId;
        return;
    }

    if (reportingTracker->getWormId() != wormId || reportingTracker->getCurrentTrackerState() != WormTracker::TrackerState::PausedAwaitingSplitDecision) {
        qWarning() << "TrackingManager: Split/Paused signal from WormTracker (ID" << reportingTracker->getWormId()
        << ", State" << static_cast<int>(reportingTracker->getCurrentTrackerState())
        << ", Dir:" << reportingTracker->property("direction").toString()
        << ") does not match expected parameters for wormId" << wormId << "or is not actually paused. Signal ignored.";
        return;
    }

    qDebug() << "TrackingManager: Worm ID" << wormId << "(Tracker:" << reportingTracker << ", Dir:" << reportingTracker->property("direction").toString() << ")"
             << "reported split and PAUSED at frame" << originalFrameNumber
             << "with" << detectedBlobs.count() << "new blobs.";
    emit trackingStatusUpdate(
        QString("Worm %1 (Dir: %2) paused, potential split.").arg(wormId).arg(reportingTracker->property("direction").toString()));

    // ***** SIMPLIFIED LOGIC FOR TESTING THE STALL *****
    if (!detectedBlobs.isEmpty()) {
        TrackingHelper::DetectedBlob largestBlob = detectedBlobs.first();
        double maxArea = 0;
        for (const auto& b : detectedBlobs) {
            if (b.area > maxArea) {
                maxArea = b.area;
                largestBlob = b;
            }
        }
        qDebug() << "  TrackingManager (Simplified Logic): Telling tracker" << reportingTracker->getWormId()
                 << "(Instance:" << reportingTracker << ", Dir:" << reportingTracker->property("direction").toString()
                 << ") to follow largest fragment at" << largestBlob.centroid;

        if (m_wormObjectsMap.contains(wormId)) {
            m_wormObjectsMap.value(wormId)->setState(WormObject::TrackingState::Tracking);
        } else {
            qWarning() << "  TrackingManager: WormObject not found for ID" << wormId << "when trying to update state on resume.";
        }

        QMetaObject::invokeMethod(
            reportingTracker, "resumeTrackingWithNewTarget",
            Qt::QueuedConnection,
            Q_ARG(TrackingHelper::DetectedBlob, largestBlob));

    } else {
        qWarning() << "  TrackingManager: Split detected for worm" << wormId << "(Tracker:" << reportingTracker
                   << ") but no blobs provided to resume with. Tracker will remain paused.";
    }
    // ***** END OF SIMPLIFIED LOGIC *****
}


void TrackingManager::processFrameDataForMergesAndSplits(int frameNumber) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    if (!m_frameInfos.contains(frameNumber)) return;
    qDebug() << "TrackingManager: processFrameDataForMergesAndSplits for frame" << frameNumber << "(Currently a stub if original logic is commented out)";
}


void TrackingManager::handleWormStateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId) {
    if (m_cancelRequested) return;
    WormObject* worm = m_wormObjectsMap.value(wormId, nullptr);
    if (worm) {
        if (newState == WormObject::TrackingState::Lost) {
            emit trackingStatusUpdate(
                QString("Worm %1 may be Lost (reported by tracker).").arg(wormId));
        }
    }
}

void TrackingManager::handleWormTrackerFinished() {
    qDebug() << "TM: handleWormTrackerFinished - START"; // DIAGNOSTIC
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if (finishedTracker) {
        qDebug() << "TrackingManager: WormTracker for worm ID"
                 << finishedTracker->getWormId() << "(Dir:" << finishedTracker->property("direction").toString()
                 << ", Ptr: " << finishedTracker << ") reported finished.";

        m_wormTrackers.removeOne(finishedTracker);
        m_individualTrackerProgress.remove(finishedTracker);

    } else {
        qWarning() << "TrackingManager: handleWormTrackerFinished called by non-WormTracker sender.";
    }
    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: Total finished trackers:"
             << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;

    updateOverallProgress(); // Call before checkForAllTrackersFinished
    checkForAllTrackersFinished();
    qDebug() << "TM: handleWormTrackerFinished - END"; // DIAGNOSTIC
}

void TrackingManager::handleWormTrackerError(int wormId, QString errorMessage) {
    qDebug() << "TrackingManager (" << this
             << "): handleWormTrackerError for worm " << wormId << ":"
             << errorMessage;
    if (m_cancelRequested) {
        return;
    }
    qWarning() << "TrackingManager: Error from tracker for worm ID" << wormId
               << ":" << errorMessage;
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    if (errorTracker) {
        m_wormTrackers.removeOne(errorTracker);
        m_individualTrackerProgress.remove(errorTracker);
    }
    m_finishedTrackersCount++;
    updateOverallProgress();
    emit trackingStatusUpdate(
        QString("Error with tracker for worm %1.").arg(wormId));
    checkForAllTrackersFinished();
}

void TrackingManager::handleWormTrackerProgress(int wormId, int percentDone) {
    if (m_cancelRequested) return;
    WormTracker* tracker = qobject_cast<WormTracker*>(sender());
    if (tracker) {
        if (tracker->getWormId() == wormId) {
            m_individualTrackerProgress[tracker] = percentDone;
            updateOverallProgress(); // Update overall progress when individual progress comes in
        } else {
            qWarning() << "TrackingManager: Progress signal for wormId" << wormId << "but sender is tracker for ID" << tracker->getWormId();
        }
    }
}


void TrackingManager::checkForAllTrackersFinished() {
    qDebug() << "TM: checkForAllTrackersFinished - START. Finished:" << m_finishedTrackersCount << "Expected:" << m_expectedTrackersToFinish; // DIAGNOSTIC
    // ... (rest of the function is the same as the one in the context, including the fix for emitting tracks on cancel)
    if (m_isTrackingRunning && m_finishedTrackersCount >= m_expectedTrackersToFinish) {
        if (m_cancelRequested) {
            qDebug() << "TrackingManager: All expected trackers have reported finished after CANCELLATION request.";
            m_finalTracks.clear();
            for (WormObject* worm : m_wormObjectsMap.values()) {
                if (worm) {
                    m_finalTracks[worm->getId()] = worm->getTrackHistory();
                }
            }
            if (!m_finalTracks.empty()) {
                qDebug() << "TrackingManager: Emitting partial tracks due to cancellation. Count:" << m_finalTracks.size();
                emit allTracksUpdated(m_finalTracks);
                outputTracksToDebug(m_finalTracks);
            } else {
                qDebug() << "TrackingManager: No tracks to emit upon cancellation.";
            }
            emit trackingStatusUpdate("Tracking cancelled by user. Partial tracks (if any) processed.");
            emit trackingCancelled();
        } else {
            qDebug() << "TrackingManager: All expected trackers have reported finished NORMALLY.";
            emit trackingStatusUpdate("All worm trackers completed. Consolidating and saving tracks...");
            m_finalTracks.clear();
            for (WormObject* worm : m_wormObjectsMap.values()) {
                if (worm) {
                    m_finalTracks[worm->getId()] = worm->getTrackHistory();
                }
            }
            emit allTracksUpdated(m_finalTracks);
            outputTracksToDebug(m_finalTracks);

            QString csvOutputPath;
            if (!m_videoPath.isEmpty()) {
                QFileInfo videoInfo(m_videoPath);
                QString baseName = videoInfo.completeBaseName();
                QString outputDir = videoInfo.absolutePath();
                csvOutputPath = outputDir + "/" + baseName + "_tracks.csv";
            } else {
                csvOutputPath = "worm_tracks.csv";
                qWarning() << "Video path is empty, saving tracks to default file:" << csvOutputPath;
            }
            bool csvSaved = outputTracksToCsv(m_finalTracks, csvOutputPath);
            if (csvSaved) {
                emit trackingStatusUpdate("Tracking finished. Tracks saved to: " + csvOutputPath);
                emit trackingFinishedSuccessfully(csvOutputPath);
            } else {
                emit trackingStatusUpdate("Tracking finished, but failed to save CSV output.");
                emit trackingFinishedSuccessfully("");
            }
        }
        m_isTrackingRunning = false;
    } else if (!m_isTrackingRunning && m_cancelRequested) {
        qDebug() << "TrackingManager: checkForAllTrackersFinished - Tracking was already stopped and cancel was requested.";
    } else if (!m_isTrackingRunning) {
        qDebug() << "TrackingManager: checkForAllTrackersFinished - Tracking is not marked as running (possibly due to earlier error or premature stop).";
    }
    qDebug() << "TM: checkForAllTrackersFinished - END"; // DIAGNOSTIC
}

void TrackingManager::updateOverallProgress() {
    qDebug() << "TM: updateOverallProgress - START"; // DIAGNOSTIC
    if (m_cancelRequested && !m_isTrackingRunning) {
        emit overallTrackingProgress(0);
        qDebug() << "TM: updateOverallProgress - END (cancelled and not running)"; // DIAGNOSTIC
        return;
    }
    if (!m_isTrackingRunning && m_finishedTrackersCount < m_expectedTrackersToFinish) {
        // This case can happen if called after cleanup but before tracking fully starts,
        // or if tracking failed very early.
        emit overallTrackingProgress(0);
        qDebug() << "TM: updateOverallProgress - END (not running, trackers not all finished)"; // DIAGNOSTIC
        return;
    }

    double totalProgress = 0;
    double videoProcWeight = 0.10;
    double trackersWeight = 0.90;

    totalProgress += static_cast<double>(m_videoProcessingProgress) * videoProcWeight / 100.0;

    if (m_expectedTrackersToFinish > 0) {
        double finishedTrackersContribution = static_cast<double>(m_finishedTrackersCount) * 100.0;
        double runningTrackersContribution = 0;

        // Iterate over trackers that are still in the progress map (i.e., not yet finished)
        for(WormTracker* tracker : m_individualTrackerProgress.keys()){
            if(m_individualTrackerProgress.contains(tracker)){
                runningTrackersContribution += m_individualTrackerProgress.value(tracker,0);
            }
        }

        double totalPointsFromTrackers = runningTrackersContribution + finishedTrackersContribution;
        double overallTrackerPercentage = (m_expectedTrackersToFinish > 0) ?
                                              (totalPointsFromTrackers / (static_cast<double>(m_expectedTrackersToFinish) * 100.0)) * 100.0
                                                                           : 0.0;
        overallTrackerPercentage = qMin(overallTrackerPercentage, 100.0); // Cap at 100%

        totalProgress += overallTrackerPercentage * trackersWeight / 100.0;

    } else if (m_videoProcessingProgress == 100) { // No trackers to run, video processing is everything
        totalProgress = 1.0; // 100%
    }

    int finalProgressPercentage = qBound(0, static_cast<int>(totalProgress * 100.0), 100);
    emit overallTrackingProgress(finalProgressPercentage);
    qDebug() << "TM: updateOverallProgress - END. Emitted:" << finalProgressPercentage; // DIAGNOSTIC
}

// ... (outputTracksToDebug and outputTracksToCsv remain the same)
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
