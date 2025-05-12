
// TrackingManager.cpp
#include "trackingmanager.h"
#include <QDebug>
#include <numeric> // for std::accumulate

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
}

TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager destroyed. Cleaning up...";
    cancelTracking(); // Ensure threads are stopped
    cleanupThreadsAndObjects(); // Explicit cleanup
}

void TrackingManager::startFullTrackingProcess(const QString& videoPath,
                                               int keyFrameNum,
                                               const std::vector<InitialWormInfo>& initialWorms,
                                               const ThresholdSettings& settings,
                                               int totalFramesInVideo) {
    if (m_isTrackingRunning) {
        emit trackingFailed("Another tracking process is already running.");
        return;
    }

    cleanupThreadsAndObjects(); // Clean up from any previous run

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


    emit trackingStatusUpdate("Starting video processing...");
    emit overallTrackingProgress(0);


    // 1. Setup and start VideoProcessor
    m_videoProcessor = new VideoProcessor();
    m_videoProcessorThread = new QThread();
    m_videoProcessor->moveToThread(m_videoProcessorThread);

    connect(m_videoProcessorThread, &QThread::started, m_videoProcessor, [this](){
        m_videoProcessor->startInitialProcessing(m_videoPath, m_keyFrameNum, m_thresholdSettings, m_totalFramesInVideo);
    });
    connect(m_videoProcessor, &VideoProcessor::initialProcessingComplete, this, &TrackingManager::handleInitialProcessingComplete);
    connect(m_videoProcessor, &VideoProcessor::processingError, this, &TrackingManager::handleVideoProcessingError);
    connect(m_videoProcessor, &VideoProcessor::initialProcessingProgress, this, &TrackingManager::handleVideoProcessingProgress);
    connect(m_videoProcessor, &VideoProcessor::processingStarted, this, [this](){ emit trackingStatusUpdate("Video processing started in thread."); });

    // Ensure thread quits and object is deleted when done or error
    connect(m_videoProcessor, &VideoProcessor::initialProcessingComplete, m_videoProcessorThread, &QThread::quit);
    connect(m_videoProcessor, &VideoProcessor::processingError, m_videoProcessorThread, &QThread::quit);
    // connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor, &VideoProcessor::deleteLater); // Defer deletion
    // connect(m_videoProcessorThread, &QThread::finished, m_videoProcessorThread, &QThread::deleteLater);

    m_videoProcessorThread->start();
}

void TrackingManager::cancelTracking() {
    if (!m_isTrackingRunning && !m_cancelRequested) return;

    m_cancelRequested = true;
    emit trackingStatusUpdate("Cancellation requested...");

    // Stop VideoProcessor thread
    if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
        m_videoProcessorThread->requestInterruption(); // VideoProcessor should check for this
        // m_videoProcessorThread->quit(); // Ask to quit
        // m_videoProcessorThread->wait(5000); // Wait for it to finish
    }

    // Stop WormTracker threads
    for (QThread* thread : m_trackerThreads) {
        if (thread && thread->isRunning()) {
            thread->requestInterruption(); // WormTracker should check this
            // thread->quit();
            // thread->wait(1000);
        }
    }
    // The actual cleanup of threads and objects will happen in their finished slots or explicitly in cleanup.
    // For now, just set flags and request interruption.
    // A more robust cancel would involve signals to the worker objects.
    // WormTracker has stopTracking(), VideoProcessor needs similar.

    // cleanupThreadsAndObjects(); // Do this once threads have confirmed stopped.
    // For now, we assume interruption is enough.
    m_isTrackingRunning = false;
    emit trackingCancelled();
    emit trackingStatusUpdate("Tracking cancelled.");
    updateOverallProgress(); // Reset progress
}


void TrackingManager::cleanupThreadsAndObjects() {
    qDebug() << "TrackingManager: Cleaning up threads and objects.";

    // Video Processor
    if (m_videoProcessorThread) {
        if (m_videoProcessorThread->isRunning()) {
            m_videoProcessorThread->quit();
            if (!m_videoProcessorThread->wait(3000)) {
                qWarning() << "VideoProcessor thread did not quit gracefully, terminating.";
                m_videoProcessorThread->terminate();
                m_videoProcessorThread->wait();
            }
        }
        delete m_videoProcessorThread;
        m_videoProcessorThread = nullptr;
    }
    if (m_videoProcessor) {
        delete m_videoProcessor; // Assumes it's safe to delete (e.g., thread finished)
        m_videoProcessor = nullptr;
    }
    m_processedForwardFrames.clear();
    m_processedReversedFrames.clear();

    // Worm Trackers and their threads
    for (QThread* thread : m_trackerThreads) {
        if (thread) {
            if (thread->isRunning()) {
                thread->quit();
                if (!thread->wait(1000)) {
                    qWarning() << "A WormTracker thread did not quit gracefully, terminating.";
                    thread->terminate();
                    thread->wait();
                }
            }
            delete thread;
        }
    }
    m_trackerThreads.clear();

    qDeleteAll(m_wormTrackers); // Deletes all WormTracker objects
    m_wormTrackers.clear();

    qDeleteAll(m_wormObjectsMap); // Deletes all WormObject instances
    m_wormObjectsMap.clear();

    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_wormTrackerProgress.clear();

    qDebug() << "TrackingManager: Cleanup complete.";
}


void TrackingManager::handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                                      const std::vector<cv::Mat>& reversedFrames,
                                                      double fps,
                                                      cv::Size frameSize) {
    if (m_cancelRequested) { cleanupThreadsAndObjects(); return; }

    qDebug() << "TrackingManager: Initial video processing complete.";
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    m_videoProcessingProgress = 100; // Mark video processing as fully complete for progress calculation

    m_processedForwardFrames = forwardFrames;
    m_processedReversedFrames = reversedFrames;
    m_videoFps = fps;
    m_videoFrameSize = frameSize;

    // Clean up video processor thread and object as they are done.
    if (m_videoProcessorThread) {
        // m_videoProcessorThread->quit(); // Already connected to quit on completion signal
        // No need to wait here, just delete when finished signal is processed by Qt event loop
        connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor, &VideoProcessor::deleteLater);
        connect(m_videoProcessorThread, &QThread::finished, m_videoProcessorThread, &QThread::deleteLater);
        m_videoProcessor = nullptr; // Ownership passed to deleteLater
        m_videoProcessorThread = nullptr;
    }


    // 2. Create WormObjects
    for (const auto& info : m_initialWormInfos) {
        if (m_wormObjectsMap.contains(info.id)) {
            qWarning() << "Duplicate worm ID in initialWormInfos:" << info.id;
            continue;
        }
        m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
    }

    // 3. Launch WormTrackers
    launchWormTrackers();
    updateOverallProgress();
}

void TrackingManager::handleVideoProcessingError(const QString& errorMessage) {
    if (m_cancelRequested) { cleanupThreadsAndObjects(); return; }
    qWarning() << "TrackingManager: Video processing error:" << errorMessage;
    emit trackingFailed("Video processing failed: " + errorMessage);
    m_isTrackingRunning = false;
    cleanupThreadsAndObjects(); // Clean up on error
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
        trackingFinishedSuccessfully(); // Or failed, depending on expectation
        m_isTrackingRunning = false;
        return;
    }

    m_expectedTrackersToFinish = static_cast<int>(m_initialWormInfos.size() * 2); // Each worm has 2 trackers
    m_finishedTrackersCount = 0;

    emit trackingStatusUpdate(QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));

    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        // Forward Tracker
        WormTracker* forwardTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
        forwardTracker->setFrames(&m_processedForwardFrames);
        QThread* fwdThread = new QThread();
        forwardTracker->moveToThread(fwdThread);

        connect(fwdThread, &QThread::started, forwardTracker, &WormTracker::startTracking);
        connect(forwardTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
        connect(forwardTracker, &WormTracker::finished, fwdThread, &QThread::quit);
        connect(forwardTracker, &WormTracker::finished, forwardTracker, &WormTracker::deleteLater); // Auto-delete tracker
        connect(fwdThread, &QThread::finished, fwdThread, &QThread::deleteLater); // Auto-delete thread

        connect(forwardTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(forwardTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(forwardTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(forwardTracker, &WormTracker::progress, this, [this, forwardTracker](int /*id*/, int p){
            m_individualTrackerProgress[forwardTracker] = p;
            updateOverallProgress();
        });


        m_wormTrackers.append(forwardTracker);
        m_trackerThreads.append(fwdThread);
        m_individualTrackerProgress[forwardTracker] = 0;
        fwdThread->start();

        // Backward Tracker
        // Note: The keyframe itself is the first frame in m_processedForwardFrames.
        // m_processedReversedFrames starts with the frame *before* the keyframe.
        WormTracker* backwardTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
        backwardTracker->setFrames(&m_processedReversedFrames);
        QThread* bwdThread = new QThread();
        backwardTracker->moveToThread(bwdThread);

        connect(bwdThread, &QThread::started, backwardTracker, &WormTracker::startTracking);
        connect(backwardTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
        connect(backwardTracker, &WormTracker::finished, bwdThread, &QThread::quit);
        connect(backwardTracker, &WormTracker::finished, backwardTracker, &WormTracker::deleteLater);
        connect(bwdThread, &QThread::finished, bwdThread, &QThread::deleteLater);

        connect(backwardTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(backwardTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(backwardTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(backwardTracker, &WormTracker::progress, this, [this, backwardTracker](int /*id*/, int p){
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

        WormTrackPoint lastPt;
        lastPt.frameNumberOriginal = originalFrameNumber;
        lastPt.position = cvPos;
        lastPt.roi = newRoi;
        emit individualWormTrackUpdated(wormId, lastPt);

        // For simplicity, update m_finalTracks here directly.
        // A more robust way might be to collect all points and build m_finalTracks at the end or periodically.
        m_finalTracks[wormId] = worm->getTrackHistory(); // This gets the sorted vector
        // emit allTracksUpdated(m_finalTracks); // Can be too frequent, maybe emit periodically
    } else {
        qWarning() << "TrackingManager: Received position update for unknown worm ID" << wormId;
    }
}

void TrackingManager::handleWormStateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId) {
    if (m_cancelRequested) return;

    WormObject* worm = m_wormObjectsMap.value(wormId, nullptr);
    if (worm) {
        worm->setState(newState, associatedWormId);
        emit trackingStatusUpdate(QString("Worm %1 state changed to %2").arg(wormId).arg(static_cast<int>(newState)));

        // Handle merge logic:
        // "if two blobs merge, treat them as a single entity for as long as necessary.
        // both tracks will end up having the same position values."
        // This part needs more sophisticated logic. The WormTracker would detect a merge candidate.
        // TrackingManager would then need to decide how to link these.
        // For now, it's just a state change.
        if (newState == WormObject::TrackingState::Merged) {
            qDebug() << "Worm ID" << wormId << "merged. Associated ID (if any from tracker):" << associatedWormId;
            // Potentially pause one of the trackers or link their ROIs.
        } else if (newState == WormObject::TrackingState::Lost) {
            qDebug() << "Worm ID" << wormId << "lost by one of its trackers.";
        }

    } else {
        qWarning() << "TrackingManager: Received state change for unknown worm ID" << wormId;
    }
}

void TrackingManager::handleWormTrackerFinished() {
    if (m_cancelRequested && m_finishedTrackersCount < m_expectedTrackersToFinish) {
        // If cancellation was requested, and this is one of the trackers finishing up.
        // We don't want to immediately declare success if other trackers were aborted.
    }

    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: A worm tracker finished. Total finished:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;

    // Update progress for the tracker that just finished (mark as 100%)
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if(finishedTracker) {
        m_individualTrackerProgress[finishedTracker] = 100;
    }
    updateOverallProgress();


    checkForAllTrackersFinished();
}

void TrackingManager::checkForAllTrackersFinished() {
    if (m_finishedTrackersCount >= m_expectedTrackersToFinish && m_isTrackingRunning && !m_cancelRequested) {
        emit trackingStatusUpdate("All worm trackers have completed.");

        // Consolidate final tracks one last time
        m_finalTracks.clear();
        for(WormObject* worm : m_wormObjectsMap.values()){
            if(worm) {
                m_finalTracks[worm->getId()] = worm->getTrackHistory();
            }
        }
        emit allTracksUpdated(m_finalTracks); // Emit final consolidated tracks
        emit trackingFinishedSuccessfully();
        m_isTrackingRunning = false;
        // cleanupThreadsAndObjects(); // Objects are set to deleteLater, threads to quit/deleteLater
        qDebug() << "TrackingManager: All tracking tasks nominally complete.";
    } else if (m_cancelRequested && m_finishedTrackersCount >= m_expectedTrackersToFinish) {
        // All trackers finished, but because of cancellation.
        // The trackingCancelled signal would have already been emitted.
        qDebug() << "TrackingManager: All trackers finished after cancellation request.";
        m_isTrackingRunning = false; // Ensure state is updated
        cleanupThreadsAndObjects(); // Perform full cleanup now
    }
}


void TrackingManager::handleWormTrackerError(int wormId, QString errorMessage) {
    if (m_cancelRequested) return; // Ignore errors if we are already cancelling

    qWarning() << "TrackingManager: Error from tracker for worm ID" << wormId << ":" << errorMessage;
    // Decide if one tracker error fails the whole process
    // For now, let it continue with other trackers, but log it.
    // emit trackingFailed(QString("Error in tracker for worm %1: %2").arg(wormId).arg(errorMessage));
    // m_isTrackingRunning = false;
    // cleanupThreadsAndObjects();

    // We still count it as "finished" to not hang the process, but it's an erroneous finish.
    m_finishedTrackersCount++;
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    if(errorTracker) {
        m_individualTrackerProgress[errorTracker] = 100; // Mark as done for progress calculation
    }
    updateOverallProgress();
    checkForAllTrackersFinished(); // Check if this error means all are done
}


void TrackingManager::updateOverallProgress() {
    if (m_cancelRequested) {
        emit overallTrackingProgress(0); // Or some indication of cancellation
        return;
    }
    if (!m_isTrackingRunning && m_finishedTrackersCount < m_expectedTrackersToFinish) { // Not started or already finished/failed
        emit overallTrackingProgress(0); // Or 100 if successfully finished
        return;
    }


    double totalProgress = 0;
    double videoProcWeight = 0.2; // Video processing is 20% of the work
    double trackersWeight = 0.8;  // Trackers are 80%

    totalProgress += m_videoProcessingProgress * videoProcWeight;

    if (m_expectedTrackersToFinish > 0) {
        double currentTrackersAggregatedProgress = 0;
        if (!m_individualTrackerProgress.empty()) {
            for (int progress : m_individualTrackerProgress.values()) {
                currentTrackersAggregatedProgress += progress;
            }
            currentTrackersAggregatedProgress /= m_individualTrackerProgress.size(); // Average progress of all trackers
        }
        totalProgress += currentTrackersAggregatedProgress * trackersWeight;
    } else if (m_videoProcessingProgress == 100) { // No trackers, video processing is everything
        totalProgress = m_videoProcessingProgress;
    }


    emit overallTrackingProgress(static_cast<int>(totalProgress));
    // qDebug() << "Overall Progress:" << static_cast<int>(totalProgress);
}


