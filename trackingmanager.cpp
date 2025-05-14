// TrackingManager.cpp
#include "trackingmanager.h"
#include <QDebug>
#include <numeric>
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
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking(); // Request stop
        // Give a very brief moment for threads to acknowledge stop request if they are busy
        // This is a bit of a hack; robust designs use QEventLoop or wait conditions.
        // For destructor, best effort to signal and then proceed to cleanup.
        QThread::msleep(50);
    }
    cleanupThreadsAndObjects(); // Perform final cleanup
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

    // *** Crucial: Clean up any remnants from a previous run ***
    // This ensures all old threads are gone and pointers are null before re-creating.
    cleanupThreadsAndObjects();
    qDebug() << "TrackingManager (" << this << "): Post-cleanup, m_videoProcessor is" << m_videoProcessor << "m_videoProcessorThread is" << m_videoProcessorThread;


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
    m_videoProcessorThread = new QThread(this);  // Thread for the worker, parented to TrackingManager for auto-cleanup if TM is deleted first
    m_videoProcessor->moveToThread(m_videoProcessorThread);
    qDebug() << "TrackingManager: Created new VideoProcessor (" << m_videoProcessor << ") and QThread (" << m_videoProcessorThread << ")";


    // Connections for VideoProcessor
    connect(m_videoProcessorThread, &QThread::started, m_videoProcessor, [this](){
        if(m_videoProcessor) { // Check if m_videoProcessor is still valid
            m_videoProcessor->startInitialProcessing(m_videoPath, m_keyFrameNum, m_thresholdSettings, m_totalFramesInVideo);
        } else {
            qWarning() << "TrackingManager: m_videoProcessor is null when QThread::started lambda was called.";
        }
    });
    connect(m_videoProcessor, &VideoProcessor::initialProcessingComplete, this, &TrackingManager::handleInitialProcessingComplete);
    connect(m_videoProcessor, &VideoProcessor::processingError, this, &TrackingManager::handleVideoProcessingError);
    connect(m_videoProcessor, &VideoProcessor::initialProcessingProgress, this, &TrackingManager::handleVideoProcessingProgress);
    connect(m_videoProcessor, &VideoProcessor::processingStarted, this, [this](){ emit trackingStatusUpdate("Video processing started in thread."); });

    // Worker object deletion when its thread finishes
    connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor, &QObject::deleteLater);
    // Also null out our pointer to it when it's confirmed deleted by the thread finishing
    connect(m_videoProcessorThread, &QThread::finished, [this](){
        qDebug() << "VideoProcessorThread (" << QObject::sender() << ") finished. m_videoProcessor (" << m_videoProcessor << ") was scheduled for deleteLater.";
        m_videoProcessor = nullptr; // Object is gone or will be soon, don't hold stale pointer
        // The m_videoProcessorThread itself is deleted in cleanupThreadsAndObjects
    });

    m_videoProcessorThread->start();
}

void TrackingManager::cancelTracking() {
    qDebug() << "TrackingManager (" << this << "): cancelTracking called. IsRunning:" << m_isTrackingRunning << "CancelRequested:" << m_cancelRequested;
    if (!m_isTrackingRunning && !m_cancelRequested) {
        qDebug() << "TrackingManager: Cancel requested but not running or already cancelled.";
        return;
    }
    if (m_cancelRequested) { // Already processing cancel
        qDebug() << "TrackingManager: Cancellation already in progress.";
        return;
    }

    m_cancelRequested = true; // Set flag immediately
    emit trackingStatusUpdate("Cancellation requested...");
    qDebug() << "TrackingManager: Cancellation flag set to true.";

    if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
        qDebug() << "TrackingManager: Requesting video processor thread (" << m_videoProcessorThread << ") interruption.";
        m_videoProcessorThread->requestInterruption();
        // VideoProcessor's loop should check for QThread::isInterruptionRequested()
    }

    for (WormTracker* tracker : qAsConst(m_wormTrackers)) {
        if (tracker) {
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
    // After requesting interruption, the threads should eventually finish.
    // checkForAllTrackersFinished will be called by their 'finished' signals.
    // If m_cancelRequested is true, checkForAllTrackersFinished will emit trackingCancelled
    // and potentially trigger cleanup.
}


void TrackingManager::cleanupThreadsAndObjects() {
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - START";
    qDebug() << "  Before VP cleanup: m_videoProcessor=" << m_videoProcessor << "m_videoProcessorThread=" << m_videoProcessorThread;

    if (m_videoProcessorThread) {
        qDebug() << "  Cleaning up video processor thread:" << m_videoProcessorThread;
        if (m_videoProcessorThread->isRunning()) {
            qDebug() << "  Video processor thread is running. Requesting interruption, quit, and wait.";
            m_videoProcessorThread->requestInterruption();
            m_videoProcessorThread->quit();
            if (!m_videoProcessorThread->wait(2000)) { // Reduced wait time
                qWarning() << "  VideoProcessor thread (" << m_videoProcessorThread << ") did not finish gracefully after quit(). Forcing termination (risky).";
                // m_videoProcessorThread->terminate(); // Last resort, can lead to resource leaks
                // m_videoProcessorThread->wait();
            } else {
                qDebug() << "  Video processor thread (" << m_videoProcessorThread << ") finished gracefully.";
            }
        }
        // m_videoProcessor is connected to deleteLater on m_videoProcessorThread->finished()
        // AND m_videoProcessor is set to nullptr in that lambda.
        // So, m_videoProcessor should be nullptr here if the thread finished correctly.
        // We only delete the QThread object itself.
        delete m_videoProcessorThread;
        m_videoProcessorThread = nullptr;
    }
    // If m_videoProcessor is somehow still not null (e.g., thread didn't finish, lambda didn't run)
    // this indicates a problem, but deleting it here might be a double delete if deleteLater is still pending.
    // The lambda setting it to nullptr is safer.
    if (m_videoProcessor) {
        qWarning() << "  TrackingManager: m_videoProcessor (" << m_videoProcessor << ") was not nulled out by thread finished signal. This may indicate an issue or pending deleteLater.";
        // delete m_videoProcessor; // Avoid manual deletion if deleteLater is connected
        m_videoProcessor = nullptr; // At least null it out now
    }


    qDebug() << "  Cleaning up tracker threads. Count:" << m_trackerThreads.size();
    for (QThread* thread : m_trackerThreads) { // Iterate by value for safety as list might change if we remove
        if (thread) {
            qDebug() << "  Cleaning up tracker thread:" << thread;
            if (thread->isRunning()) {
                qDebug() << "  Tracker thread (" << thread << ") is running. Requesting interruption, quit, and wait.";
                thread->requestInterruption();
                thread->quit();
                if (!thread->wait(500)) { // Reduced wait time
                    qWarning() << "  A WormTracker thread (" << thread << ") did not finish gracefully after quit().";
                    // thread->terminate();
                    // thread->wait();
                } else {
                    qDebug() << "  Tracker thread (" << thread << ") finished gracefully.";
                }
            }
            // WormTracker objects are connected to deleteLater on thread finished.
            // And the lambda connected to thread->finished should remove the tracker from m_wormTrackers.
            delete thread; // Delete the QThread object itself.
        }
    }
    m_trackerThreads.clear();
    // m_wormTrackers list should be getting modified by the lambdas connected to thread->finished.
    // If any remain, they are likely for threads that didn't finish cleanly.
    // qDeleteAll is for QObject pointers, WormTracker are QObjects.
    if (!m_wormTrackers.isEmpty()) {
        qWarning() << "  TrackingManager: m_wormTrackers list is not empty after thread cleanup. Count:" << m_wormTrackers.size() << "This might indicate some trackers were not properly cleaned by deleteLater.";
        // qDeleteAll(m_wormTrackers); // Risky if deleteLater is still pending for some
        m_wormTrackers.clear(); // Clear the list of pointers.
    }


    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    m_processedForwardFrames.clear(); std::vector<cv::Mat>().swap(m_processedForwardFrames);
    m_processedReversedFrames.clear(); std::vector<cv::Mat>().swap(m_processedReversedFrames);
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();

    m_isTrackingRunning = false;
    m_cancelRequested = false;
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - FINISHED";
}


void TrackingManager::handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                                      const std::vector<cv::Mat>& reversedFrames,
                                                      double fps,
                                                      cv::Size frameSize) {
    qDebug() << "TrackingManager (" << this << "): handleInitialProcessingComplete received.";
    // Check if m_videoProcessor is null, which would happen if the thread finished and deleteLater occurred
    // before this slot was processed (unlikely with direct connection but good to be aware of).
    if (!m_videoProcessor) {
        qWarning() << "TrackingManager: handleInitialProcessingComplete called, but m_videoProcessor is already null. Cancellation or race condition?";
        if (m_cancelRequested) emit trackingCancelled();
        // else emit trackingFailed("Video processor became unavailable unexpectedly.");
        m_isTrackingRunning = false;
        return;
    }

    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Video processing complete but cancellation was requested. Not proceeding to launch trackers.";
        // m_videoProcessor and m_videoProcessorThread will be cleaned up by their finished signals / cleanup.
        emit trackingCancelled();
        m_isTrackingRunning = false;
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

    // The VideoProcessor object (m_videoProcessor) is connected to deleteLater
    // when m_videoProcessorThread finishes. The lambda also sets m_videoProcessor to nullptr.
    // The QThread (m_videoProcessorThread) itself will be deleted in the *next* call to
    // cleanupThreadsAndObjects (e.g., at the start of another tracking run, or in destructor).

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
        emit trackingCancelled();
        m_isTrackingRunning = false;
        return;
    }
    qWarning() << "TrackingManager: Video processing error:" << errorMessage;
    emit trackingFailed("Video processing failed: " + errorMessage);
    m_isTrackingRunning = false;
    cleanupThreadsAndObjects();
}

void TrackingManager::handleVideoProcessingProgress(int percentage) {
    if (m_cancelRequested) return;
    m_videoProcessingProgress = percentage;
    updateOverallProgress();
    emit trackingStatusUpdate(QString("Video processing: %1%").arg(percentage));
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
    m_individualTrackerProgress.clear(); // Clear for new set of trackers

    emit trackingStatusUpdate(QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));

    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        // Forward Tracker
        WormTracker* forwardTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
        forwardTracker->setFrames(&m_processedForwardFrames);
        QThread* fwdThread = new QThread(this); // Parent thread to TrackingManager
        forwardTracker->moveToThread(fwdThread);
        forwardTracker->setProperty("wormId", wormId);

        connect(fwdThread, &QThread::started, forwardTracker, &WormTracker::startTracking, Qt::QueuedConnection);
        connect(forwardTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
        connect(fwdThread, &QThread::finished, forwardTracker, &QObject::deleteLater); // Delete worker when thread finishes
        connect(fwdThread, &QThread::finished, [this, forwardTracker, fwdThread](){ // Null out pointers and remove from lists
            qDebug() << "Forward tracker thread (" << fwdThread << ") finished. Tracker (" << forwardTracker << ") was deleteLater'd.";
            m_wormTrackers.removeOne(forwardTracker);
            // m_trackerThreads.removeOne(fwdThread); // Thread is deleted in cleanup
        });


        connect(forwardTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(forwardTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(forwardTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(forwardTracker, &WormTracker::progress, this, [this, forwardTracker](int /*id*/, int p){
            if (m_cancelRequested) return;
            m_individualTrackerProgress[forwardTracker] = p;
            updateOverallProgress();
        });
        m_wormTrackers.append(forwardTracker);
        m_trackerThreads.append(fwdThread);
        m_individualTrackerProgress[forwardTracker] = 0;
        fwdThread->start();

        // Backward Tracker
        WormTracker* backwardTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
        backwardTracker->setFrames(&m_processedReversedFrames);
        QThread* bwdThread = new QThread(this); // Parent thread to TrackingManager
        backwardTracker->moveToThread(bwdThread);
        backwardTracker->setProperty("wormId", wormId);

        connect(bwdThread, &QThread::started, backwardTracker, &WormTracker::startTracking, Qt::QueuedConnection);
        connect(backwardTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
        connect(bwdThread, &QThread::finished, backwardTracker, &QObject::deleteLater); // Delete worker when thread finishes
        connect(bwdThread, &QThread::finished, [this, backwardTracker, bwdThread](){
            qDebug() << "Backward tracker thread (" << bwdThread << ") finished. Tracker (" << backwardTracker << ") was deleteLater'd.";
            m_wormTrackers.removeOne(backwardTracker);
            // m_trackerThreads.removeOne(bwdThread); // Thread is deleted in cleanup
        });

        connect(backwardTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(backwardTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(backwardTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(backwardTracker, &WormTracker::progress, this, [this, backwardTracker](int /*id*/, int p){
            if (m_cancelRequested) return;
            m_individualTrackerProgress[backwardTracker] = p;
            updateOverallProgress();
        });
        m_wormTrackers.append(backwardTracker);
        m_trackerThreads.append(bwdThread);
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
        qDebug() << "TrackingManager: WormTracker for worm ID" << finishedTracker->property("wormId").toInt() << "reported finished.";
        m_individualTrackerProgress.remove(finishedTracker); // Remove from progress map as it's done
            // It will be deleteLater'd by its thread.
    } else {
        qWarning() << "TrackingManager: handleWormTrackerFinished called by non-WormTracker sender.";
    }

    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: Total finished trackers:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;
    updateOverallProgress(); // Update progress based on remaining trackers
    checkForAllTrackersFinished();
}

void TrackingManager::checkForAllTrackersFinished() {
    qDebug() << "TrackingManager (" << this << "): checkForAllTrackersFinished. Cancelled:" << m_cancelRequested << "Running:" << m_isTrackingRunning << "Finished:" << m_finishedTrackersCount << "Expected:" << m_expectedTrackersToFinish;

    if (m_cancelRequested) {
        if (m_isTrackingRunning) { // Only emit cancelled and cleanup if it was running and then cancelled
            qDebug() << "TrackingManager: Cancellation was requested and now checking if all trackers have stopped.";
            // Check if all threads in m_trackerThreads are finished.
            // This is tricky because threads might be removed from m_trackerThreads by cleanup.
            // The m_finishedTrackersCount >= m_expectedTrackersToFinish check is more reliable here
            // as trackers should emit finished() even when stopped early.
            if (m_finishedTrackersCount >= m_expectedTrackersToFinish) {
                qDebug() << "TrackingManager: All expected trackers have reported finished after cancellation request.";
                emit trackingCancelled();
                m_isTrackingRunning = false;
                // cleanupThreadsAndObjects(); // Let destructor or next start handle full cleanup.
                // Or call it if immediate cleanup is desired.
            }
        }
        return;
    }

    if (m_isTrackingRunning && m_finishedTrackersCount >= m_expectedTrackersToFinish) {
        emit trackingStatusUpdate("All worm trackers completed. Consolidating and saving tracks...");
        m_finalTracks.clear();
        for(WormObject* worm : m_wormObjectsMap.values()){
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
        m_isTrackingRunning = false;
        qDebug() << "TrackingManager: All tracking tasks complete. Tracks consolidated.";
        // Not calling cleanupThreadsAndObjects() here; let them be cleaned at next start or destruction.
    }
}

void TrackingManager::handleWormTrackerError(int wormId, QString errorMessage) {
    qDebug() << "TrackingManager (" << this << "): handleWormTrackerError for worm " << wormId << ":" << errorMessage;
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Worm tracker error, but cancellation already requested.";
        return;
    }
    qWarning() << "TrackingManager: Error from tracker for worm ID" << wormId << ":" << errorMessage;

    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    if(errorTracker) {
        m_individualTrackerProgress.remove(errorTracker); // Remove from progress calculation
    }

    m_finishedTrackersCount++; // Count it as finished to avoid hanging
    updateOverallProgress();
    emit trackingStatusUpdate(QString("Error with tracker for worm %1. See logs.").arg(wormId));
    checkForAllTrackersFinished();
}

void TrackingManager::updateOverallProgress() {
    if (m_cancelRequested && !m_isTrackingRunning) { // If cancelled and already marked not running
        emit overallTrackingProgress(0);
        return;
    }
    if (!m_isTrackingRunning && m_finishedTrackersCount < m_expectedTrackersToFinish) { // Not started yet
        emit overallTrackingProgress(0);
        return;
    }


    double totalProgress = 0;
    double videoProcWeight = 0.20;
    double trackersWeight = 0.80;

    totalProgress += static_cast<double>(m_videoProcessingProgress) * videoProcWeight / 100.0;

    if (m_expectedTrackersToFinish > 0) {
        double currentTrackersAggregatedProgressPts = 0;
        int validTrackerProgressEntries = 0;
        for (int progress : m_individualTrackerProgress.values()) {
            currentTrackersAggregatedProgressPts += static_cast<double>(progress);
            validTrackerProgressEntries++;
        }
        if (validTrackerProgressEntries > 0) {
            double avgTrackerProgress = currentTrackersAggregatedProgressPts / static_cast<double>(validTrackerProgressEntries);
            totalProgress += avgTrackerProgress * trackersWeight / 100.0;
        } else if (m_finishedTrackersCount > 0 && m_finishedTrackersCount == m_expectedTrackersToFinish) {
            // All trackers finished, but map might be empty if they finished very fast
            totalProgress += 1.0 * trackersWeight; // Assume 100% for trackers part
        }
    } else if (m_videoProcessingProgress == 100) { // No trackers to run
        totalProgress = 1.0; // 100%
    }

    emit overallTrackingProgress(qBound(0, static_cast<int>(totalProgress * 100.0), 100));
}

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
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
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

