
// TrackingManager.cpp
#include "trackingmanager.h" // Lowercase include
#include <QDebug>
#include <numeric> // for std::accumulate
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QFileInfo>

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
}

TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this << ") destroyed. Cleaning up...";
    // Ensure any ongoing tracking is stopped and threads are cleaned.
    // cancelTracking() should ideally lead to cleanupThreadsAndObjects() being effectively called
    // or making it safe.
    if (m_isTrackingRunning) {
        cancelTracking(); // Request stop
        // It might be necessary to wait here briefly if cancel is fully async
        // QThread::msleep(100); // Small delay to allow signals to process, use with caution
    }
    cleanupThreadsAndObjects(); // Perform final cleanup
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

    // *** Crucial: Clean up any remnants from a previous run ***
    cleanupThreadsAndObjects();

    m_videoPath = videoPath;
    m_keyFrameNum = keyFrameNum;
    m_initialWormInfos = initialWorms;
    m_thresholdSettings = settings;
    m_totalFramesInVideo = totalFramesInVideo;
    m_isTrackingRunning = true; // Set this *after* cleanup and *before* starting new work
    m_cancelRequested = false;
    m_videoProcessingProgress = 0;
    m_finishedTrackersCount = 0;
    m_expectedTrackersToFinish = 0;
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();

    emit trackingStatusUpdate("Starting video processing...");
    emit overallTrackingProgress(0);

    // 1. Setup and start VideoProcessor
    m_videoProcessor = new VideoProcessor(); // Worker object
    m_videoProcessorThread = new QThread();  // Thread for the worker
    m_videoProcessor->moveToThread(m_videoProcessorThread);

    // Connections for VideoProcessor
    connect(m_videoProcessorThread, &QThread::started, m_videoProcessor, [this](){
        m_videoProcessor->startInitialProcessing(m_videoPath, m_keyFrameNum, m_thresholdSettings, m_totalFramesInVideo);
    });
    connect(m_videoProcessor, &VideoProcessor::initialProcessingComplete, this, &TrackingManager::handleInitialProcessingComplete);
    connect(m_videoProcessor, &VideoProcessor::processingError, this, &TrackingManager::handleVideoProcessingError);
    connect(m_videoProcessor, &VideoProcessor::initialProcessingProgress, this, &TrackingManager::handleVideoProcessingProgress);
    connect(m_videoProcessor, &VideoProcessor::processingStarted, this, [this](){ emit trackingStatusUpdate("Video processing started in thread."); });

    // Worker object deletion when its thread finishes
    connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor, &QObject::deleteLater);
    // The m_videoProcessorThread itself will be deleted in cleanupThreadsAndObjects

    m_videoProcessorThread->start();
}

void TrackingManager::cancelTracking() {
    qDebug() << "TrackingManager (" << this << "): cancelTracking called.";
    if (!m_isTrackingRunning && !m_cancelRequested) {
        qDebug() << "TrackingManager: Cancel requested but not running or already cancelled.";
        return;
    }
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Cancellation already in progress.";
        return;
    }

    m_cancelRequested = true; // Set flag immediately
    emit trackingStatusUpdate("Cancellation requested...");
    qDebug() << "TrackingManager: Cancellation flag set to true.";

    // Request VideoProcessor thread to stop
    if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
        qDebug() << "TrackingManager: Requesting video processor thread (" << m_videoProcessorThread << ") interruption.";
        m_videoProcessorThread->requestInterruption();
        // VideoProcessor's loop should check for QThread::isInterruptionRequested()
    }

    // Request WormTracker threads to stop
    for (WormTracker* tracker : qAsConst(m_wormTrackers)) { // Use qAsConst for const-correct iteration
        if (tracker) {
            // The tracker itself might be in a different thread, so use invokeMethod or signal.
            // stopTracking() slot in WormTracker should set its internal active flag to false.
            qDebug() << "TrackingManager: Invoking stopTracking() for tracker related to worm ID" << tracker->property("wormId").toInt();
            QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
        }
    }
    for (QThread* thread : qAsConst(m_trackerThreads)) {
        if (thread && thread->isRunning()) {
            qDebug() << "TrackingManager: Requesting tracker thread (" << thread << ") interruption.";
            thread->requestInterruption();
        }
    }
    // Note: Actual thread stopping and cleanup is handled when threads finish or in cleanupThreadsAndObjects.
    // This function primarily sets the m_cancelRequested flag and signals workers.
    // The `checkForAllTrackersFinished` will eventually see that `m_cancelRequested` is true.
}


void TrackingManager::cleanupThreadsAndObjects() {
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - START";

    // Video Processor Thread and Object
    if (m_videoProcessorThread) {
        qDebug() << "Cleaning up video processor thread:" << m_videoProcessorThread;
        if (m_videoProcessorThread->isRunning()) {
            qDebug() << "Video processor thread is running. Requesting interruption and quit.";
            m_videoProcessorThread->requestInterruption();
            m_videoProcessorThread->quit();
            if (!m_videoProcessorThread->wait(3000)) {
                qWarning() << "VideoProcessor thread (" << m_videoProcessorThread << ") did not finish gracefully after quit().";
                // Terminating is risky, but if it's stuck...
                // m_videoProcessorThread->terminate();
                // m_videoProcessorThread->wait();
            } else {
                qDebug() << "Video processor thread (" << m_videoProcessorThread << ") finished gracefully.";
            }
        }
        // VideoProcessor object (m_videoProcessor) is connected to deleteLater on thread finished.
        // Now delete the QThread object itself.
        delete m_videoProcessorThread;
        m_videoProcessorThread = nullptr;
    }
    // If m_videoProcessor wasn't nulled out and thread didn't finish to deleteLater it:
    if (m_videoProcessor) {
        qWarning() << "TrackingManager: m_videoProcessor was not null during cleanup. Deleting manually.";
        delete m_videoProcessor; // Should have been deleteLater'd
        m_videoProcessor = nullptr;
    }


    // Worm Tracker Threads and Objects
    // Iterate carefully as WormTracker objects are deleteLater'd when their thread finishes
    for (QThread* thread : m_trackerThreads) { // No qAsConst if we might modify by deleting
        if (thread) {
            qDebug() << "Cleaning up tracker thread:" << thread;
            if (thread->isRunning()) {
                qDebug() << "Tracker thread (" << thread << ") is running. Requesting interruption and quit.";
                thread->requestInterruption();
                thread->quit();
                if (!thread->wait(1000)) {
                    qWarning() << "A WormTracker thread (" << thread << ") did not finish gracefully after quit().";
                    // thread->terminate();
                    // thread->wait();
                } else {
                    qDebug() << "Tracker thread (" << thread << ") finished gracefully.";
                }
            }
            // WormTracker objects are connected to deleteLater on thread finished.
            // Now delete the QThread object itself.
            delete thread;
        }
    }
    m_trackerThreads.clear(); // Clear list of (now deleted) QThread pointers
    m_wormTrackers.clear();   // Clear list of WormTracker pointers (they are deleteLater'd)


    // Clear data structures
    qDeleteAll(m_wormObjectsMap); // WormObject are not QObjects, so direct delete
    m_wormObjectsMap.clear();

    m_processedForwardFrames.clear(); std::vector<cv::Mat>().swap(m_processedForwardFrames);
    m_processedReversedFrames.clear(); std::vector<cv::Mat>().swap(m_processedReversedFrames);
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    // m_wormTrackerProgress was removed as m_individualTrackerProgress is used with WormTracker* as key

    m_isTrackingRunning = false; // Ensure state is reset
    m_cancelRequested = false;   // Ensure state is reset
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - FINISHED";
}


void TrackingManager::handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                                      const std::vector<cv::Mat>& reversedFrames,
                                                      double fps,
                                                      cv::Size frameSize) {
    qDebug() << "TrackingManager (" << this << "): handleInitialProcessingComplete received.";
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Processing complete but cancellation was requested during video processing. Cleaning up.";
        // VideoProcessor object and its thread will be cleaned up via their finished signals
        // (deleteLater connected in startFullTrackingProcess for m_videoProcessor).
        // We still need to ensure this instance of TrackingManager is clean.
        // cleanupThreadsAndObjects(); // This might be too aggressive if called from here directly
        emit trackingCancelled(); // Signal that the overall process was cancelled
        m_isTrackingRunning = false; // Mark as not running
        // Consider if m_videoProcessor and m_videoProcessorThread should be nulled here
        // if their deleteLater is guaranteed by the thread finishing.
        return;
    }

    qDebug() << "TrackingManager: Initial video processing complete. Forward frames:" << forwardFrames.size()
             << "Reversed frames:" << reversedFrames.size();
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    m_videoProcessingProgress = 100;

    m_processedForwardFrames = forwardFrames;
    m_processedReversedFrames = reversedFrames;
    m_videoFps = fps;
    m_videoFrameSize = frameSize;

    // The m_videoProcessor is already connected to deleteLater when m_videoProcessorThread finishes.
    // The m_videoProcessorThread itself is now managed by cleanupThreadsAndObjects.
    // We don't need to delete m_videoProcessor here.
    // m_videoProcessor = nullptr; // It will be deleted later.

    for (const auto& info : m_initialWormInfos) {
        if (m_wormObjectsMap.contains(info.id)) {
            qWarning() << "Duplicate worm ID in initialWormInfos:" << info.id << "Skipping recreation.";
            continue;
        }
        m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
    }

    launchWormTrackers();
    updateOverallProgress();
}

void TrackingManager::handleVideoProcessingError(const QString& errorMessage) {
    qDebug() << "TrackingManager (" << this << "): handleVideoProcessingError: " << errorMessage;
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Video processing error, but cancellation already requested.";
        // cleanupThreadsAndObjects(); // Let cancel flow handle cleanup
        emit trackingCancelled();
        m_isTrackingRunning = false;
        return;
    }
    qWarning() << "TrackingManager: Video processing error:" << errorMessage;
    emit trackingFailed("Video processing failed: " + errorMessage);
    m_isTrackingRunning = false;
    cleanupThreadsAndObjects(); // Clean up on error
}

void TrackingManager::handleVideoProcessingProgress(int percentage) {
    if (m_cancelRequested) return;
    m_videoProcessingProgress = percentage;
    updateOverallProgress();
    // emit trackingStatusUpdate(QString("Video processing: %1%").arg(percentage)); // Can be too noisy
}


void TrackingManager::launchWormTrackers() {
    if (m_initialWormInfos.empty()) {
        emit trackingStatusUpdate("No worms selected for tracking.");
        emit trackingFinishedSuccessfully(""); // No tracks, but process finished as "successful"
        m_isTrackingRunning = false;
        return;
    }

    m_expectedTrackersToFinish = static_cast<int>(m_initialWormInfos.size() * 2);
    m_finishedTrackersCount = 0;
    m_individualTrackerProgress.clear();

    emit trackingStatusUpdate(QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));

    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        // Forward Tracker
        WormTracker* forwardTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
        forwardTracker->setFrames(&m_processedForwardFrames);
        QThread* fwdThread = new QThread(); // Create new thread
        forwardTracker->moveToThread(fwdThread);
        forwardTracker->setProperty("wormId", wormId); // For easier identification

        connect(fwdThread, &QThread::started, forwardTracker, &WormTracker::startTracking, Qt::QueuedConnection);
        connect(forwardTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
        // Worker object deletion when its thread finishes
        connect(fwdThread, &QThread::finished, forwardTracker, &WormTracker::deleteLater);
        // QThread object (fwdThread) will be deleted in cleanupThreadsAndObjects

        connect(forwardTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(forwardTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(forwardTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(forwardTracker, &WormTracker::progress, this, [this, forwardTracker](int /*id*/, int p){
            if (m_cancelRequested) return;
            m_individualTrackerProgress[forwardTracker] = p;
            updateOverallProgress();
        });
        m_wormTrackers.append(forwardTracker); // Store worker pointer
        m_trackerThreads.append(fwdThread);    // Store thread pointer
        m_individualTrackerProgress[forwardTracker] = 0;
        fwdThread->start();

        // Backward Tracker
        WormTracker* backwardTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
        backwardTracker->setFrames(&m_processedReversedFrames);
        QThread* bwdThread = new QThread(); // Create new thread
        backwardTracker->moveToThread(bwdThread);
        backwardTracker->setProperty("wormId", wormId);

        connect(bwdThread, &QThread::started, backwardTracker, &WormTracker::startTracking, Qt::QueuedConnection);
        connect(backwardTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
        // Worker object deletion
        connect(bwdThread, &QThread::finished, backwardTracker, &WormTracker::deleteLater);
        // QThread object (bwdThread) will be deleted in cleanupThreadsAndObjects

        connect(backwardTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(backwardTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(backwardTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(backwardTracker, &WormTracker::progress, this, [this, backwardTracker](int /*id*/, int p){
            if (m_cancelRequested) return;
            m_individualTrackerProgress[backwardTracker] = p;
            updateOverallProgress();
        });
        m_wormTrackers.append(backwardTracker); // Store worker pointer
        m_trackerThreads.append(bwdThread);    // Store thread pointer
        m_individualTrackerProgress[backwardTracker] = 0;
        bwdThread->start();
    }
}

void TrackingManager::handleWormPositionUpdated(int wormId, int originalFrameNumber, QPointF newPosition, QRectF newRoi) {
    if (m_cancelRequested) return;
    WormObject* worm = m_wormObjectsMap.value(wormId, nullptr);
    if (worm) {
        cv::Point2f cvPos(static_cast<float>(newPosition.x()), static_cast<float>(newPosition.y()));
        worm->updateTrackPoint(originalFrameNumber, cvPos, newRoi);
        WormTrackPoint lastPt; lastPt.frameNumberOriginal = originalFrameNumber; lastPt.position = cvPos; lastPt.roi = newRoi;
        emit individualWormTrackUpdated(wormId, lastPt);
    }
}

void TrackingManager::handleWormStateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId) {
    if (m_cancelRequested) return;
    WormObject* worm = m_wormObjectsMap.value(wormId, nullptr);
    if (worm) {
        worm->setState(newState, associatedWormId);
        emit trackingStatusUpdate(QString("Worm %1 state changed.").arg(wormId));
    }
}

void TrackingManager::handleWormTrackerFinished() {
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if(finishedTracker) {
        m_individualTrackerProgress[finishedTracker] = 100;
        // The tracker object itself is scheduled for deleteLater by its thread's finished signal.
        // The thread object is managed by cleanupThreadsAndObjects.
        // We don't remove from m_wormTrackers here as qDeleteAll will handle it in cleanup,
        // or they'll be gone by deleteLater. For progress map, it's fine.
    }

    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: A worm tracker finished. Total finished:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;
    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::checkForAllTrackersFinished() {
    qDebug() << "TrackingManager (" << this << "): checkForAllTrackersFinished. Cancelled:" << m_cancelRequested << "Running:" << m_isTrackingRunning << "Finished:" << m_finishedTrackersCount << "Expected:" << m_expectedTrackersToFinish;
    if (m_cancelRequested) {
        // If cancel was requested, the main goal is to ensure everything stops and cleans up.
        // The actual emission of trackingCancelled and cleanup might happen here
        // or after all threads have confirmed they've stopped.
        // For now, let's assume cancelTracking initiated the stop, and cleanup will be called.
        // If all trackers have reported finished (even if due to stopping early), proceed to cleanup.
        if (m_finishedTrackersCount >= m_expectedTrackersToFinish) {
            qDebug() << "TrackingManager: All trackers finished after cancellation request.";
            emit trackingCancelled(); // Emit signal
            m_isTrackingRunning = false; // Ensure state is updated
            // cleanupThreadsAndObjects(); // Cleanup will be called by destructor or next start
        }
        return;
    }

    if (m_isTrackingRunning && m_finishedTrackersCount >= m_expectedTrackersToFinish) {
        emit trackingStatusUpdate("All worm trackers completed. Consolidating and saving tracks...");
        m_finalTracks.clear();
        for(WormObject* worm : m_wormObjectsMap.values()){ // Corrected iteration
            if(worm) {
                m_finalTracks[worm->getId()] = worm->getTrackHistory();
            }
        }
        emit allTracksUpdated(m_finalTracks);
        //outputTracksToDebug(m_finalTracks);
        QString csvOutputPath;
        if (!m_videoPath.isEmpty()) {
            QFileInfo videoInfo(m_videoPath);
            QString tmpDirName = "tmp_tracking_output";
            QDir videoDir = videoInfo.dir();
            if (videoDir.mkpath(tmpDirName)) {
                csvOutputPath = videoDir.filePath(tmpDirName + "/" + videoInfo.completeBaseName() + "_tracks.csv");
            } else {
                csvOutputPath = videoDir.filePath(videoInfo.completeBaseName() + "_tracks.csv");
                qWarning() << "Could not create tmp directory:" << videoDir.filePath(tmpDirName) << "Saving CSV to video directory.";
            }
        } else {
            csvOutputPath = "worm_tracks.csv";
            qWarning() << "Video path is empty, saving tracks to default file:" << csvOutputPath;
        }
        bool csvSaved = outputTracksToCsv(m_finalTracks, csvOutputPath);
        if (csvSaved) {
            emit trackingStatusUpdate("Tracks saved to: " + csvOutputPath);
            emit trackingFinishedSuccessfully(csvOutputPath);
        } else {
            emit trackingStatusUpdate("Tracking finished, but failed to save CSV.");
            emit trackingFinishedSuccessfully("");
        }
        m_isTrackingRunning = false; // Tracking is now complete
        qDebug() << "TrackingManager: All tracking tasks complete. Tracks consolidated.";
        // Threads and workers are scheduled for deleteLater or will be handled by next cleanup.
    }
}

void TrackingManager::handleWormTrackerError(int wormId, QString errorMessage) {
    qDebug() << "TrackingManager (" << this << "): handleWormTrackerError for worm " << wormId << ":" << errorMessage;
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Worm tracker error, but cancellation already requested.";
        // Let cancel flow handle cleanup
        return;
    }
    qWarning() << "TrackingManager: Error from tracker for worm ID" << wormId << ":" << errorMessage;
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    if(errorTracker) {
        m_individualTrackerProgress[errorTracker] = 100; // Mark as "done" for progress calculation
    }
    m_finishedTrackersCount++; // Count it as finished to avoid hanging
    updateOverallProgress();
    emit trackingStatusUpdate(QString("Error with tracker for worm %1. See logs.").arg(wormId));
    // Potentially emit trackingFailed if one error should stop everything
    // emit trackingFailed(QString("Tracker error for worm %1: %2").arg(wormId).arg(errorMessage));
    // m_isTrackingRunning = false;
    // cleanupThreadsAndObjects();
    checkForAllTrackersFinished();
}

void TrackingManager::updateOverallProgress() {
    if (m_cancelRequested || !m_isTrackingRunning) {
        // If cancelled, progress should ideally reflect that or be reset.
        // If not running (e.g. before start or after finish/fail), don't emit misleading progress.
        if (m_cancelRequested) emit overallTrackingProgress(0); // Or a specific "cancelled" value if desired
        return;
    }

    double totalProgress = 0;
    double videoProcWeight = 0.20;
    double trackersWeight = 0.80;

    totalProgress += static_cast<double>(m_videoProcessingProgress) * videoProcWeight / 100.0;

    if (m_expectedTrackersToFinish > 0 && m_individualTrackerProgress.size() > 0) { // Check size > 0 for division
        double currentTrackersAggregatedProgressPts = 0;
        for (int progress : m_individualTrackerProgress.values()) { // Corrected: iterate directly
            currentTrackersAggregatedProgressPts += static_cast<double>(progress);
        }
        double avgTrackerProgress = currentTrackersAggregatedProgressPts / static_cast<double>(m_individualTrackerProgress.size());
        totalProgress += avgTrackerProgress * trackersWeight / 100.0;
    } else if (m_videoProcessingProgress == 100 && m_expectedTrackersToFinish == 0) {
        totalProgress = 1.0; // 100%
    }
    emit overallTrackingProgress(qBound(0, static_cast<int>(totalProgress * 100.0), 100)); // Ensure progress is 0-100
}

// --- Debugging and Output Methods ---
void TrackingManager::outputTracksToDebug(const AllWormTracks& tracks) const {
    qDebug() << "--- Begin Track Output (" << this << ") ---";
    if (tracks.empty()) {
        qDebug() << "No tracks to output.";
    }
    for (const auto& pair : tracks) {
        int wormId = pair.first;
        const std::vector<WormTrackPoint>& points = pair.second;
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
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) { // Truncate to overwrite
        qWarning() << "Failed to open CSV file for writing:" << outputFilePath << csvFile.errorString();
        return false;
    }
    QTextStream outStream(&csvFile);
    outStream << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight\n";
    for (const auto& pair : tracks) {
        int wormId = pair.first;
        const std::vector<WormTrackPoint>& points = pair.second;
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

