// trackingmanager.cpp
#include "trackingmanager.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QPair> // For QPair
#include <QRectF> // For QRectF, though cv::Rect might be more common with OpenCV

#include <algorithm> // For std::reverse, std::min, std::max
#include <numeric>   // For std::accumulate
#include <limits>    // For std::numeric_limits
#include <cmath>     // For std::ceil

// Constructor
TrackingManager::TrackingManager(QObject* parent)
    : QObject(parent),
    m_keyFrameNum(-1),
    m_totalFramesInVideoHint(0),
    m_isTrackingRunning(false),
    m_cancelRequested(false),
    // m_videoProcessor(nullptr), // Deprecated
    // m_videoProcessorThread(nullptr), // Deprecated
    m_videoProcessorsFinishedCount(0),
    m_totalVideoChunksToProcess(0),
    m_videoFps(0.0),
    m_expectedTrackersToFinish(0),
    m_finishedTrackersCount(0),
    m_videoProcessingOverallProgress(0),
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

// Destructor
TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";
    if (m_pauseResolutionTimer) {
        m_pauseResolutionTimer->stop();
        // delete m_pauseResolutionTimer; // Parent QObject handles this
        // m_pauseResolutionTimer = nullptr;
    }
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking();
        QThread::msleep(300); // Give cancel a moment to propagate
    }
    cleanupThreadsAndObjects();
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - FINISHED cleaning up.";
}

// Main entry point for tracking
void TrackingManager::startFullTrackingProcess(
    const QString& videoPath, int keyFrameNum,
    const std::vector<Tracking::InitialWormInfo>& initialWorms,
    const Thresholding::ThresholdSettings& settings, int totalFramesInVideoHint) {
    qDebug() << "TrackingManager (" << this << "): startFullTrackingProcess called.";
    if (m_isTrackingRunning) {
        qWarning() << "TrackingManager: Attempted to start tracking while already running.";
        emit trackingFailed("Another tracking process is already running.");
        return;
    }

    cleanupThreadsAndObjects(); // Clean up from any previous run

    // --- Basic setup ---
    m_videoPath = videoPath;
    m_keyFrameNum = keyFrameNum;
    m_initialWormInfos = initialWorms;
    m_thresholdSettings = settings;
    m_totalFramesInVideoHint = totalFramesInVideoHint; // Store the hint
    m_isTrackingRunning = true;
    m_cancelRequested = false;
    m_videoProcessingOverallProgress = 0;
    m_finishedTrackersCount = 0;
    m_expectedTrackersToFinish = 0;
    m_nextUniqueMergeId = 1;

    // Clear data structures
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_activeMergeGroups.clear();
    m_frameToActiveMergeGroupIds.clear();
    m_wormToCurrentMergeGroupId.clear();
    m_pausedWorms.clear();
    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();
    m_videoChunkProgressMap.clear();
    m_videoProcessorsFinishedCount = 0;
    m_totalVideoChunksToProcess = 0;
    m_finalProcessedForwardFrames.clear();
    m_finalProcessedReversedFrames.clear();


    for (const auto& info : m_initialWormInfos) {
        if (!m_wormObjectsMap.contains(info.id)) {
            m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
        }
        m_wormToCurrentMergeGroupId[info.id] = -1;
    }

    emit trackingStatusUpdate("Initializing video processing...");
    emit overallTrackingProgress(0);

    // --- Determine actual total frames, FPS, and frame size ONCE ---
    cv::VideoCapture preliminaryCap;
    int actualTotalFrames = 0;
    try {
        if (!preliminaryCap.open(m_videoPath.toStdString())) {
            emit trackingFailed("Failed to open video file for preliminary checks: " + m_videoPath);
            m_isTrackingRunning = false;
            return;
        }
        actualTotalFrames = static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_COUNT));
        m_videoFps = preliminaryCap.get(cv::CAP_PROP_FPS);
        m_videoFrameSize = cv::Size(
            static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_WIDTH)),
            static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_HEIGHT))
            );
        preliminaryCap.release();
    } catch (const cv::Exception& ex) {
        preliminaryCap.release();
        emit trackingFailed("OpenCV exception during preliminary video checks: " + QString(ex.what()));
        m_isTrackingRunning = false;
        return;
    }

    if (actualTotalFrames <= 0 && m_totalFramesInVideoHint > 0) {
        actualTotalFrames = m_totalFramesInVideoHint;
        qWarning() << "Could not get actual total frames, using hint:" << actualTotalFrames;
    } else if (actualTotalFrames <= 0) {
        emit trackingFailed("Could not determine total number of frames for video: " + m_videoPath);
        m_isTrackingRunning = false;
        return;
    }
    if (m_videoFps <= 0) m_videoFps = 25.0; // Default FPS if not available
    if (m_videoFrameSize.width <= 0 || m_videoFrameSize.height <= 0) {
        emit trackingFailed("Could not determine frame size for video: " + m_videoPath);
        m_isTrackingRunning = false;
        return;
    }
    if (m_keyFrameNum < 0 || m_keyFrameNum >= actualTotalFrames) {
        emit trackingFailed(QString("Keyframe index %1 is out of bounds for video with %2 frames.").arg(m_keyFrameNum).arg(actualTotalFrames));
        m_isTrackingRunning = false;
        return;
    }


    // --- Parallel Video Processing Logic ---
    int numThreads = QThread::idealThreadCount();
    numThreads = qMax(1, qMin(numThreads, 8)); // Use 1 to 8 threads

    emit trackingStatusUpdate(QString("Processing video in chunks across %1 threads...").arg(numThreads));

    QList<QPair<int, int>> forwardFrameRanges; // For frames from keyframe to end
    QList<QPair<int, int>> backwardFrameRanges; // For frames from 0 to keyframe

    // 1. Calculate chunks for FORWARD processing (keyframe to end)
    int forwardSegmentStart = m_keyFrameNum;
    int forwardSegmentEnd = actualTotalFrames;
    int totalForwardSegmentFrames = forwardSegmentEnd - forwardSegmentStart;

    if (totalForwardSegmentFrames > 0) {
        int framesPerThread = std::max(1, static_cast<int>(std::ceil(static_cast<double>(totalForwardSegmentFrames) / numThreads)));
        for (int currentStart = forwardSegmentStart; currentStart < forwardSegmentEnd; currentStart += framesPerThread) {
            int currentEnd = std::min(currentStart + framesPerThread, forwardSegmentEnd);
            if (currentStart < currentEnd) {
                forwardFrameRanges.append({currentStart, currentEnd});
            }
        }
    }

    // 2. Calculate chunks for BACKWARD processing (start to keyframe)
    // These frames will eventually be reversed for the backward trackers.
    int backwardSegmentStart = 0;
    int backwardSegmentEnd = m_keyFrameNum; // Exclusive, so up to keyFrameNum - 1
    int totalBackwardSegmentFrames = backwardSegmentEnd - backwardSegmentStart;

    if (totalBackwardSegmentFrames > 0) {
        int framesPerThread = std::max(1, static_cast<int>(std::ceil(static_cast<double>(totalBackwardSegmentFrames) / numThreads)));
        for (int currentStart = backwardSegmentStart; currentStart < backwardSegmentEnd; currentStart += framesPerThread) {
            int currentEnd = std::min(currentStart + framesPerThread, backwardSegmentEnd);
            if(currentStart < currentEnd) {
                backwardFrameRanges.append({currentStart, currentEnd});
            }
        }
    }

    m_totalVideoChunksToProcess = forwardFrameRanges.size() + backwardFrameRanges.size();
    qDebug() << "Total video chunks to process:" << m_totalVideoChunksToProcess
             << "(Fwd:" << forwardFrameRanges.size() << ", Bwd:" << backwardFrameRanges.size() << ")";


    if (m_totalVideoChunksToProcess == 0) {
        qDebug() << "No video frames to process based on keyframe and total frames. Finishing early.";
        // This will call launchWormTrackers with empty frame vectors
        handleInitialProcessingComplete({}, {}, m_videoFps, m_videoFrameSize);
        return;
    }

    // 3. Launch threads for all chunks
    auto launch_chunk_processor = [this](int startFrame, int endFrame, bool isForward) {
        int chunkId = startFrame; // Use start frame as a unique chunk ID for this segment
        m_videoChunkProgressMap[chunkId] = 0;

        VideoProcessor* processor = new VideoProcessor(); // No parent, will be deleted with thread
        QThread* thread = new QThread(); // No parent initially, TM will own via QPointer list
        processor->moveToThread(thread);

        // Ensure thread is added to list before connecting its finished signal for cleanup
        m_videoProcessorThreads.append(thread);


        connect(thread, &QThread::started, processor, [=]() {
            // Pass copies of settings and path
            processor->processFrameRange(m_videoPath, m_thresholdSettings, startFrame, endFrame, chunkId, isForward);
        });

        connect(processor, &VideoProcessor::rangeProcessingComplete, this, &TrackingManager::handleRangeProcessingComplete);
        connect(processor, &VideoProcessor::processingError, this, &TrackingManager::handleVideoChunkProcessingError);
        connect(processor, &VideoProcessor::rangeProcessingProgress, this, &TrackingManager::handleRangeProcessingProgress);

        // Cleanup: When thread finishes, delete the processor. The thread itself is managed by m_videoProcessorThreads.
        connect(thread, &QThread::finished, processor, &QObject::deleteLater);
        connect(thread, &QThread::finished, this, [this, threadPtr = QPointer<QThread>(thread)](){
            if (threadPtr && m_videoProcessorThreads.contains(threadPtr)) {
                // No need to remove here, cleanupThreadsAndObjects will handle it.
                // Or, if we want to remove dynamically:
                // m_videoProcessorThreads.removeOne(threadPtr);
                // delete threadPtr; // If TM owns it directly
                qDebug() << "Video processor thread" << threadPtr << "finished.";
            }
        });
        thread->start();
    };

    for (const auto& range : forwardFrameRanges) {
        launch_chunk_processor(range.first, range.second, true);
    }
    for (const auto& range : backwardFrameRanges) {
        launch_chunk_processor(range.first, range.second, false);
    }

    if (m_pauseResolutionTimer) {
        m_pauseResolutionTimer->start(PAUSE_RESOLUTION_INTERVAL_MS);
    }
}

// Slot to handle progress from individual video processing chunks
void TrackingManager::handleRangeProcessingProgress(int chunkId, int percentage) {
    if (m_cancelRequested || !m_isTrackingRunning) return;

    QMutexLocker locker(&m_dataMutex);
    if (m_videoChunkProgressMap.contains(chunkId)) {
        m_videoChunkProgressMap[chunkId] = percentage;
    } else {
        qWarning() << "Received progress for unknown chunkId:" << chunkId;
        return;
    }

    double totalProgressSum = 0;
    for (int p : m_videoChunkProgressMap.values()) {
        totalProgressSum += p;
    }

    if (m_totalVideoChunksToProcess > 0) {
        m_videoProcessingOverallProgress = static_cast<int>(totalProgressSum / m_totalVideoChunksToProcess);
    } else {
        m_videoProcessingOverallProgress = 100; // No chunks means processing is "done"
    }
    locker.unlock();

    updateOverallProgress(); // This will emit overallTrackingProgress
}

// Slot to handle completion of a single video processing chunk
void TrackingManager::handleRangeProcessingComplete(int chunkId,
                                                    const std::vector<cv::Mat>& processedFrames,
                                                    bool wasForwardChunk) {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) {
        qDebug() << "Video chunk" << chunkId << "finished, but cancellation was requested. Discarding.";
        // Decrement count to ensure cleanup logic works if some threads finish after cancel
        m_totalVideoChunksToProcess = qMax(0, m_totalVideoChunksToProcess -1);
        if (m_videoProcessorsFinishedCount >= m_totalVideoChunksToProcess && m_totalVideoChunksToProcess == 0) {
            qDebug() << "All remaining video chunks accounted for after cancellation.";
            // Potentially call a cleanup or final cancel step if needed
        }
        return;
    }

    qDebug() << "Video processing chunk" << chunkId << (wasForwardChunk ? "(Forward)" : "(Backward)")
             << "finished with" << processedFrames.size() << "frames.";

    if (wasForwardChunk) {
        m_assembledForwardFrameChunks[chunkId] = processedFrames;
    } else {
        m_assembledBackwardFrameChunks[chunkId] = processedFrames;
    }
    m_videoChunkProgressMap[chunkId] = 100; // Mark this chunk as 100% complete

    m_videoProcessorsFinishedCount++;
    qDebug() << "Finished video chunks:" << m_videoProcessorsFinishedCount << "/" << m_totalVideoChunksToProcess;


    // Check if all chunks are done
    if (m_videoProcessorsFinishedCount >= m_totalVideoChunksToProcess) {
        locker.unlock();
        qDebug() << "All video processing chunks are complete. Assembling final frames.";
        assembleProcessedFrames(); // This will eventually call handleInitialProcessingComplete
    } else {
        // Update overall video processing progress based on completed chunks and ongoing ones
        double totalProgressSum = 0;
        for (int p : m_videoChunkProgressMap.values()) {
            totalProgressSum += p;
        }
        if (m_totalVideoChunksToProcess > 0) {
            m_videoProcessingOverallProgress = static_cast<int>(totalProgressSum / m_totalVideoChunksToProcess);
        } else {
            m_videoProcessingOverallProgress = 100;
        }
        locker.unlock();
        updateOverallProgress();
    }
}

// New method to assemble frames once all chunks are processed
void TrackingManager::assembleProcessedFrames() {
    QMutexLocker locker(&m_dataMutex); // Protect access to chunk maps
    if (m_cancelRequested) {
        qDebug() << "AssembleProcessedFrames called during cancellation. Aborting assembly.";
        // If cancellation happened right before this, ensure we signal cancellation.
        // The checkForAllTrackersFinished or cancelTracking itself should handle emitting trackingCancelled.
        // We might need to ensure that if video processing is cancelled, tracking doesn't start.
        m_isTrackingRunning = false; // Ensure tracking stops if it was in video processing phase
        locker.unlock();
        emit trackingCancelled(); // Or rely on cancelTracking to do this.
        return;
    }

    m_finalProcessedForwardFrames.clear();
    m_finalProcessedReversedFrames.clear();

    // QMap iterates keys in ascending order, which is what we want (chunkId = startFrame)
    for (const auto& chunk : m_assembledForwardFrameChunks.values()) {
        m_finalProcessedForwardFrames.insert(m_finalProcessedForwardFrames.end(), chunk.begin(), chunk.end());
    }
    qDebug() << "Assembled" << m_finalProcessedForwardFrames.size() << "forward frames from" << m_assembledForwardFrameChunks.size() << "chunks.";

    // For backward frames, assemble them in their natural chunk order first
    std::vector<cv::Mat> tempBackwardFrames;
    for (const auto& chunk : m_assembledBackwardFrameChunks.values()) {
        tempBackwardFrames.insert(tempBackwardFrames.end(), chunk.begin(), chunk.end());
    }
    // Now, globally reverse the assembled backward frames.
    // These frames were processed, e.g., [0-49], [50-99]. After concat: [0..49, 50..99].
    // For backward tracking (from keyframe-1 down to 0), this whole sequence needs to be reversed.
    m_finalProcessedReversedFrames = tempBackwardFrames;
    std::reverse(m_finalProcessedReversedFrames.begin(), m_finalProcessedReversedFrames.end());

    qDebug() << "Assembled" << tempBackwardFrames.size() << "frames for backward segment from"
             << m_assembledBackwardFrameChunks.size() << "chunks. After reversing, size is"
             << m_finalProcessedReversedFrames.size();

    // Clear the temporary chunk storage
    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();
    m_videoChunkProgressMap.clear(); // Reset for any potential next run

    locker.unlock();

    // Now call the original handler that proceeds to launch trackers
    // m_videoFps and m_videoFrameSize were already determined in startFullTrackingProcess
    handleInitialProcessingComplete(m_finalProcessedForwardFrames, m_finalProcessedReversedFrames, m_videoFps, m_videoFrameSize);
}


// This is the original slot, now called by assembleProcessedFrames
void TrackingManager::handleInitialProcessingComplete(
    const std::vector<cv::Mat>& finalForwardFrames,
    const std::vector<cv::Mat>& finalReversedFrames,
    double fps,
    cv::Size frameSize) {

    qDebug() << "TrackingManager (" << this << "): All video processing assembled and complete.";
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing fully complete but cancellation requested. Not launching trackers.";
        m_isTrackingRunning = false;
        locker.unlock();
        emit trackingCancelled();
        return;
    }
    if (!m_isTrackingRunning) {
        qWarning() << "TM: handleInitialProcessingComplete, but tracking is not marked as running. Aborting tracker launch.";
        return;
    }

    // Store the final processed frames (already done if called from assembleProcessedFrames)
    // m_finalProcessedForwardFrames = finalForwardFrames;
    // m_finalProcessedReversedFrames = finalReversedFrames;
    // m_videoFps = fps; // Already set
    // m_videoFrameSize = frameSize; // Already set

    m_videoProcessingOverallProgress = 100; // Mark video processing as fully complete
    locker.unlock();

    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    updateOverallProgress(); // Update progress to reflect video part is 100%

    // Pass references to the final assembled frames to the trackers
    launchWormTrackers();
}

// Slot to handle errors from any video processing chunk
void TrackingManager::handleVideoChunkProcessingError(int chunkId, const QString& errorMessage) {
    qDebug() << "TrackingManager (" << this << "): handleVideoChunkProcessingError from chunk" << chunkId << ":" << errorMessage;
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) {
        qDebug() << "TM: Video processing error (chunk " << chunkId << ":" << errorMessage << ") received during/after cancellation. Letting cancel flow proceed.";
        return;
    }
    if (!m_isTrackingRunning) {
        qWarning() << "TM: Video processing error (chunk " << chunkId << ":" << errorMessage << ") but tracking not marked as running.";
        return;
    }

    // An error in one chunk means the whole video processing fails.
    // Request cancellation for other running video processor threads.
    m_cancelRequested = true; // Set flag to prevent further actions and signal other parts
    m_isTrackingRunning = false; // Stop the overall tracking process

    if (m_pauseResolutionTimer && m_pauseResolutionTimer->isActive()) {
        m_pauseResolutionTimer->stop();
    }
    locker.unlock();

    // Request interruption for all video processor threads
    for (QPointer<QThread> thread : m_videoProcessorThreads) {
        if (thread && thread->isRunning()) {
            qDebug() << "Requesting interruption for video processor thread:" << thread;
            thread->requestInterruption();
        }
    }
    // Note: Threads will be joined/deleted in cleanupThreadsAndObjects or when cancelTracking is fully processed.

    qWarning() << "TM: Video processing failed due to error in chunk" << chunkId << ":" << errorMessage;
    emit trackingFailed("Video processing failed (chunk " + QString::number(chunkId) + "): " + errorMessage);
}


// Cancel Tracking
void TrackingManager::cancelTracking() {
    qDebug() << "TrackingManager (" << this << "): cancelTracking called. IsRunning:" << m_isTrackingRunning << "CancelRequested:" << m_cancelRequested;

    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested && !m_isTrackingRunning) {
        qDebug() << "TM: Cancellation already fully processed or tracking not running.";
        return;
    }
    if (m_cancelRequested) {
        qDebug() << "TM: Cancellation already in progress.";
        return;
    }
    m_cancelRequested = true; // Set the flag
    bool wasRunning = m_isTrackingRunning;
    // m_isTrackingRunning will be set to false once all parts acknowledge cancellation or finish.
    locker.unlock();

    emit trackingStatusUpdate("Cancellation requested...");

    if (m_pauseResolutionTimer) {
        m_pauseResolutionTimer->stop();
    }

    // Request interruption for all video processor threads
    for (QPointer<QThread> thread : m_videoProcessorThreads) {
        if (thread && thread->isRunning()) {
            qDebug() << "Cancelling: Requesting interruption for video processor thread:" << thread;
            thread->requestInterruption();
        }
    }

    // Request stop for all worm tracker threads
    QList<WormTracker*> trackersToStop;
    locker.relock();
    trackersToStop = m_wormTrackersList; // m_wormTrackersList contains active trackers
    locker.unlock();

    for (WormTracker* tracker : trackersToStop) {
        if (tracker) {
            // Use invokeMethod to ensure it's called on the tracker's thread
            QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
        }
    }

    locker.relock();
    // If tracking was cancelled during video processing (no trackers launched yet)
    // or if it was running but all trackers somehow finished before cancel fully propagated.
    if ((wasRunning && m_expectedTrackersToFinish == 0 && m_wormTrackersList.isEmpty()) || !wasRunning) {
        m_isTrackingRunning = false; // Ensure tracking stops
        locker.unlock();
        qDebug() << "TM: Cancellation processed. Emitting trackingCancelled.";
        emit trackingCancelled(); // This will trigger cleanup via checkForAllTrackersFinished if needed
    } else if (wasRunning && m_videoProcessorsFinishedCount < m_totalVideoChunksToProcess && m_totalVideoChunksToProcess > 0) {
        // If cancelled during video processing, and some chunks might still be "running"
        // The interruption request should make them finish.
        // We need to ensure that assembleProcessedFrames doesn't run or bails out.
        // The m_cancelRequested flag should handle this.
        // The trackingCancelled signal will be emitted by checkForAllTrackersFinished
        // once all (potentially fewer) trackers report back, or if video processing error occurred.
        // If video processing was active, we need to ensure that flow leads to trackingCancelled.
        // If no trackers were launched, and video processing is cancelled, this path is tricky.
        // Let's assume if video processing is active, the error/complete path for it will eventually lead to
        // a state where checkForAllTrackersFinished can emit trackingCancelled.
        // For now, if cancel is called, and video processors are running, they will be interrupted.
        // Their completion (even if partial or errored due to interrupt) should lead to assembleProcessedFrames,
        // which will see m_cancelRequested and bail, then emit trackingCancelled.
        qDebug() << "TM: Cancellation requested during active video processing. Video threads interrupted.";
        // No direct emit trackingCancelled() here; let the processing pipeline wind down.
    }
    // checkForAllTrackersFinished will be called by trackers finishing/erroring,
    // or if video processing finishes/errors and no trackers were launched.
    // It will see m_cancelRequested and emit trackingCancelled.
}

// Cleanup
void TrackingManager::cleanupThreadsAndObjects() {
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - START";

    // Stop and wait for video processor threads
    // Use a copy for iteration as threads might be removed from m_videoProcessorThreads upon finishing
    QList<QPointer<QThread>> videoThreadsToClean = m_videoProcessorThreads;
    m_videoProcessorThreads.clear(); // Clear the main list

    for (QPointer<QThread> thread : videoThreadsToClean) {
        if (thread) { // QPointer checks if object still exists
            qDebug() << "  Cleaning up video processor thread:" << thread;
            if (thread->isRunning()) {
                thread->requestInterruption(); // Ensure it's asked to stop
                thread->quit();
                if (!thread->wait(1500)) { // Increased wait time
                    qWarning() << "  VideoProcessor thread (" << thread << ") did not finish gracefully. Forcing termination.";
                    thread->terminate();
                    thread->wait(); // Wait for termination
                }
            }
            delete thread; // Delete the QThread object itself
        }
    }

    // Stop and wait for worm tracker threads
    QList<QPointer<QThread>> trackerThreadsToClean = m_trackerThreads;
    m_trackerThreads.clear();

    for (QPointer<QThread> thread : trackerThreadsToClean) {
        if (thread) {
            qDebug() << "  Cleaning up tracker thread:" << thread;
            if (thread->isRunning()) {
                thread->requestInterruption();
                thread->quit();
                if (!thread->wait(1000)) { // Increased wait time
                    qWarning() << "  A WormTracker thread (" << thread << ") did not finish gracefully. Forcing termination.";
                    thread->terminate();
                    thread->wait();
                }
            }
            delete thread;
        }
    }

    // Clear lists of trackers (WormTracker objects themselves are deleted when their QThread finishes)
    m_wormTrackersList.clear();
    m_wormIdToForwardTrackerInstanceMap.clear();
    m_wormIdToBackwardTrackerInstanceMap.clear();

    // Delete WormObjects
    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();

    // Clear frame data
    m_finalProcessedForwardFrames.clear();
    std::vector<cv::Mat>().swap(m_finalProcessedForwardFrames); // Ensure memory is released
    m_finalProcessedReversedFrames.clear();
    std::vector<cv::Mat>().swap(m_finalProcessedReversedFrames);

    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();
    m_videoChunkProgressMap.clear();


    // Clear other state
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_activeMergeGroups.clear();
    m_frameToActiveMergeGroupIds.clear();
    m_wormToCurrentMergeGroupId.clear();
    m_pausedWorms.clear();

    m_isTrackingRunning = false; // Ensure state is reset
    m_cancelRequested = false;   // Reset cancel flag for next run
    qDebug() << "TrackingManager (" << this << "): cleanupThreadsAndObjects - FINISHED";
}

// Launch Worm Trackers (uses m_finalProcessedForwardFrames and m_finalProcessedReversedFrames)
void TrackingManager::launchWormTrackers() {
    if (m_cancelRequested) {
        qDebug() << "LaunchWormTrackers: Cancelled before launching. Aborting.";
        m_isTrackingRunning = false;
        emit trackingCancelled();
        return;
    }

    if (m_initialWormInfos.empty()) {
        emit trackingStatusUpdate("No worms selected for tracking.");
        emit trackingFinishedSuccessfully(""); // No tracks to save, but process is "successful"
        m_isTrackingRunning = false;
        return;
    }

    m_expectedTrackersToFinish = 0;
    m_finishedTrackersCount = 0;
    m_individualTrackerProgress.clear();
    m_wormIdToForwardTrackerInstanceMap.clear();
    m_wormIdToBackwardTrackerInstanceMap.clear();

    // Ensure tracker threads list is clear before populating
    // cleanupThreadsAndObjects should have handled this, but as a safeguard:
    for(QPointer<QThread> t : m_trackerThreads) { if(t) delete t; }
    m_trackerThreads.clear();


    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        // Launch Forward Tracker
        // Condition: there are forward frames OR keyframe is the very last frame (only backward tracking possible, but this condition might be for symmetry or edge cases)
        if (!m_finalProcessedForwardFrames.empty() || m_keyFrameNum == (m_videoFrameSize.width > 0 ? static_cast<int>(m_videoFps * (m_totalFramesInVideoHint > 0 ? m_totalFramesInVideoHint : 1) / m_videoFps) -1 : 0) ) { // Approx total frames if needed
            WormTracker* fwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
            // Pass a pointer to the member variable vector. This is safe as long as TrackingManager outlives the trackers.
            fwdTracker->setFrames(&m_finalProcessedForwardFrames);
            QThread* fwdThread = new QThread();
            fwdTracker->moveToThread(fwdThread);
            fwdTracker->setProperty("wormId", wormId);
            fwdTracker->setProperty("direction", "Forward");

            m_trackerThreads.append(fwdThread); // Add before connecting finished for cleanup

            connect(fwdThread, &QThread::started, fwdTracker, &WormTracker::startTracking);
            connect(fwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(fwdThread, &QThread::finished, fwdTracker, &QObject::deleteLater);
            connect(fwdThread, &QThread::finished, this, [this, fwdThreadPtr = QPointer<QThread>(fwdThread), wormId](){
                if (fwdThreadPtr && m_trackerThreads.contains(fwdThreadPtr)) {
                    // m_trackerThreads.removeOne(fwdThreadPtr); // cleanupThreadsAndObjects handles full clear
                    qDebug() << "Forward Tracker Thread for worm" << wormId << "finished.";
                }
            });

            connect(fwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(fwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(fwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);

            m_wormTrackersList.append(fwdTracker);
            m_wormIdToForwardTrackerInstanceMap[wormId] = fwdTracker;
            m_individualTrackerProgress[fwdTracker] = 0;
            fwdThread->start();
            m_expectedTrackersToFinish++;
        } else {
            qDebug() << "TM: Skipping Forward Tracker for worm" << wormId << "due to no forward frames or keyframe position.";
        }

        // Launch Backward Tracker
        // Condition: there are reversed frames OR keyframe is the very first frame
        if (!m_finalProcessedReversedFrames.empty() || m_keyFrameNum == 0) {
            WormTracker* bwdTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
            // Pass a pointer to the member variable vector.
            bwdTracker->setFrames(&m_finalProcessedReversedFrames);
            QThread* bwdThread = new QThread();
            bwdTracker->moveToThread(bwdThread);
            bwdTracker->setProperty("wormId", wormId);
            bwdTracker->setProperty("direction", "Backward");

            m_trackerThreads.append(bwdThread);

            connect(bwdThread, &QThread::started, bwdTracker, &WormTracker::startTracking);
            connect(bwdTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(bwdThread, &QThread::finished, bwdTracker, &QObject::deleteLater);
            connect(bwdThread, &QThread::finished, this, [this, bwdThreadPtr = QPointer<QThread>(bwdThread), wormId](){
                if (bwdThreadPtr && m_trackerThreads.contains(bwdThreadPtr)) {
                    qDebug() << "Backward Tracker Thread for worm" << wormId << "finished.";
                }
            });

            connect(bwdTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(bwdTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(bwdTracker, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);

            m_wormTrackersList.append(bwdTracker);
            m_wormIdToBackwardTrackerInstanceMap[wormId] = bwdTracker;
            m_individualTrackerProgress[bwdTracker] = 0;
            bwdThread->start();
            m_expectedTrackersToFinish++;
        } else {
            qDebug() << "TM: Skipping Backward Tracker for worm" << wormId << "due to no backward frames or keyframe position.";
        }
    }

    if (m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) {
        qWarning() << "TM: No trackers were launched despite having initial worm infos. Check keyframe, video length, and frame processing results.";
        emit trackingFailed("No trackers could be launched. Video might be too short, keyframe at an extreme, or video processing yielded no frames.");
        m_isTrackingRunning = false;
        if (m_pauseResolutionTimer && m_pauseResolutionTimer->isActive()) {
            m_pauseResolutionTimer->stop();
        }
    } else if (m_expectedTrackersToFinish > 0) {
        emit trackingStatusUpdate(QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));
    } else if (m_initialWormInfos.empty()) { // Already handled, but for completeness
        emit trackingStatusUpdate("No worms to track.");
        emit trackingFinishedSuccessfully("");
        m_isTrackingRunning = false;
    }
    updateOverallProgress(); // Initialize progress display
}

// Update Overall Progress
void TrackingManager::updateOverallProgress() {
    if (m_cancelRequested && !m_isTrackingRunning) { // If fully cancelled
        emit overallTrackingProgress(m_videoProcessingOverallProgress); // Show final video progress
        return;
    }
    if (!m_isTrackingRunning && !m_cancelRequested) { // If not running and not cancelled (e.g. before start or after error)
        emit overallTrackingProgress(0);
        return;
    }

    double totalProgressValue = 0.0;
    // Weight for video processing part (e.g., 10-30% of total time)
    // Let's make it dynamic: if no trackers, video is 100% of the task.
    double videoProcWeight = (m_expectedTrackersToFinish > 0 || m_initialWormInfos.empty()) ? 0.20 : 1.0;
    double trackersWeight = 1.0 - videoProcWeight;

    // Video processing progress (m_videoProcessingOverallProgress is already 0-100)
    totalProgressValue += (static_cast<double>(m_videoProcessingOverallProgress) / 100.0) * videoProcWeight;

    double overallTrackerPercentage = 0.0;
    {
        QMutexLocker locker(&m_dataMutex);
        if (m_expectedTrackersToFinish > 0) {
            double currentPointsFromTrackers = 0;
            // Sum progress of active trackers
            for(int progress : m_individualTrackerProgress.values()){
                currentPointsFromTrackers += progress;
            }
            // Add points from finished trackers
            currentPointsFromTrackers += (static_cast<double>(m_finishedTrackersCount) * 100.0);

            double maxPointsFromTrackers = static_cast<double>(m_expectedTrackersToFinish) * 100.0;

            if (maxPointsFromTrackers > 0) {
                overallTrackerPercentage = currentPointsFromTrackers / maxPointsFromTrackers;
            }
            overallTrackerPercentage = qBound(0.0, overallTrackerPercentage, 1.0);
        } else if (m_videoProcessingOverallProgress == 100 && m_initialWormInfos.empty()) {
            // No worms to track, video processed, so tracker part is "complete"
            overallTrackerPercentage = 1.0;
        } else if (m_videoProcessingOverallProgress == 100 && m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()){
            // Video processed, worms were expected, but no trackers launched (e.g. keyframe issue)
            overallTrackerPercentage = 0.0; // Tracker part is 0 or failed.
        }
    }

    if (m_expectedTrackersToFinish > 0 || (m_initialWormInfos.empty() && videoProcWeight < 1.0) ) { // Only add tracker progress if trackers are expected or part of weighted progress
        totalProgressValue += overallTrackerPercentage * trackersWeight;
    }


    int finalProgressPercentage = qBound(0, static_cast<int>(totalProgressValue * 100.0), 100);
    emit overallTrackingProgress(finalProgressPercentage);
}


// Check For All Trackers Finished
void TrackingManager::checkForAllTrackersFinished() {
    QMutexLocker locker(&m_dataMutex);
    qDebug() << "TM: Checking if all trackers finished. Finished:" << m_finishedTrackersCount
             << "Expected:" << m_expectedTrackersToFinish << "Running:" << m_isTrackingRunning
             << "CancelReq:" << m_cancelRequested << "Trackers in list:" << m_wormTrackersList.count();

    bool allDoneOrCancelled = false;
    if (m_isTrackingRunning && (m_finishedTrackersCount >= m_expectedTrackersToFinish)) {
        allDoneOrCancelled = true;
        qDebug() << "TM: All expected trackers finished normally.";
    } else if (m_cancelRequested) {
        // If cancelled, we consider it "done" when all *currently active or expected* trackers report back.
        // This means m_finishedTrackersCount should eventually equal m_expectedTrackersToFinish,
        // even if m_expectedTrackersToFinish was reduced due to errors during cancellation.
        // Or, if no trackers were ever launched but cancel was called during video processing.
        if (m_finishedTrackersCount >= m_expectedTrackersToFinish || m_expectedTrackersToFinish == 0) {
            allDoneOrCancelled = true;
            qDebug() << "TM: All trackers accounted for after CANCELLATION request, or no trackers were active.";
        }
    }


    if (allDoneOrCancelled) {
        bool wasCancelled = m_cancelRequested;
        m_isTrackingRunning = false; // Stop tracking state
        if (m_pauseResolutionTimer && m_pauseResolutionTimer->isActive()) {
            m_pauseResolutionTimer->stop();
        }
        // At this point, all worker threads (video & tracker) should have been signaled to stop or have finished.
        // We can perform a final cleanup of thread objects if not already handled.
        // cleanupThreadsAndObjects(); // Call this to ensure QThread objects are deleted.
        // Be careful of calling this if it's already in destructor flow.

        locker.unlock();

        if (wasCancelled) {
            qDebug() << "TrackingManager: Processing CANCELLATION completion.";
            m_finalTracks.clear(); // Usually, partial tracks are not saved on cancel, but can be configured.
            // For now, let's assume we consolidate any tracks made before cancellation.
            for (WormObject* worm : m_wormObjectsMap.values()) {
                if (worm) m_finalTracks[worm->getId()] = worm->getTrackHistory();
            }
            if (!m_finalTracks.empty()) emit allTracksUpdated(m_finalTracks);

            emit trackingStatusUpdate("Tracking cancelled by user.");
            emit trackingCancelled();
        } else { // Finished normally
            qDebug() << "TrackingManager: Processing NORMAL completion.";
            emit trackingStatusUpdate("All worm trackers completed. Consolidating tracks...");
            m_finalTracks.clear();
            for (WormObject* worm : m_wormObjectsMap.values()) {
                if (worm) m_finalTracks[worm->getId()] = worm->getTrackHistory();
            }
            emit allTracksUpdated(m_finalTracks);

            QString csvOutputPath;
            if (!m_videoPath.isEmpty()) {
                QFileInfo videoInfo(m_videoPath);
                csvOutputPath = QDir(videoInfo.absolutePath()).filePath(videoInfo.completeBaseName() + "_tracks.csv");
            } else {
                csvOutputPath = "worm_tracks.csv";
            }

            if (outputTracksToCsv(m_finalTracks, csvOutputPath)) {
                emit trackingStatusUpdate("Tracking finished. Tracks saved to: " + csvOutputPath);
                emit trackingFinishedSuccessfully(csvOutputPath);
            } else {
                emit trackingStatusUpdate("Tracking finished, but failed to save CSV output to: " + csvOutputPath);
                emit trackingFailed("Failed to save CSV output to: " + csvOutputPath);
            }
        }
        // Perform a final cleanup after signals are emitted and state is stable.
        // This ensures QThread objects are deleted.
        QMetaObject::invokeMethod(this, "cleanupThreadsAndObjects", Qt::QueuedConnection);


    } else if (!m_isTrackingRunning && m_cancelRequested) {
        // This case handles if tracking was stopped by an error, then cancel was called.
        // Or if cancel was called, and we are waiting for threads to finish.
        locker.unlock();
        qDebug() << "TM: checkForAllTrackersFinished - Tracking not running, cancel requested. Waiting for threads or emitting cancel.";
        // If all threads are truly done, the above (allDoneOrCancelled) would be true.
        // If we reach here, it implies something is still pending or state is inconsistent.
        // However, if m_isTrackingRunning is false and m_cancelRequested is true,
        // it's safe to assume the process should end with trackingCancelled.
        // emit trackingCancelled(); // This might be redundant if the above block handles it.
    }
}


// --- Handler for WormTracker signals (FrameUpdate, Finished, Error, Progress) ---
// These remain largely the same as your original implementation.
// Ensure m_dataMutex is used appropriately.

void TrackingManager::handleFrameUpdate(int reportingConceptualWormId,
                                        int originalFrameNumber,
                                        const Tracking::DetectedBlob& primaryBlob,
                                        QRectF searchRoiUsed,
                                        Tracking::TrackerState currentState)
{
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* reportingTrackerInstance = qobject_cast<WormTracker*>(sender());
    // ... (rest of your existing logic for handleFrameUpdate) ...
    // Ensure it uses m_wormObjectsMap, m_wormToCurrentMergeGroupId, m_activeMergeGroups, m_pausedWorms correctly.
    // The merge/split logic (processMergedState, processPausedForSplitState) should be fine.
    // Make sure any access to shared data within those functions is also mindful of the mutex if they call out.
    // For brevity, I'm not reproducing the entire merge/split logic here.
    // Just ensure it's compatible with the overall multithreaded design.

    WormObject* wormObject = m_wormObjectsMap.value(reportingConceptualWormId, nullptr);
    if (!wormObject) {
        qWarning() << "TM: handleFrameUpdate for unknown worm object ID:" << reportingConceptualWormId;
        return;
    }

    QString dirTag = reportingTrackerInstance ? (reportingTrackerInstance->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd") : "UnkDir";
    // qDebug().noquote() << QString("TM: WT %1(%2) Frame %3 State %4") // Too verbose for normal operation
    //                           .arg(reportingConceptualWormId)
    //                           .arg(dirTag)
    //                           .arg(originalFrameNumber)
    //                           .arg(static_cast<int>(currentState));

    if (primaryBlob.isValid) {
        Tracking::WormTrackPoint point;
        point.frameNumberOriginal = originalFrameNumber;
        point.position = cv::Point2f(static_cast<float>(primaryBlob.centroid.x()), static_cast<float>(primaryBlob.centroid.y()));
        point.roi = searchRoiUsed; // This is QRectF
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
                    group.isValid = false;
                    qDebug() << "TM: Merge group" << currentMergeGroupId << "is now empty due to worm" << reportingConceptualWormId << "state" << static_cast<int>(currentState);
                }
            }
            m_wormToCurrentMergeGroupId[reportingConceptualWormId] = -1;
            // qDebug() << "TM: Worm" << reportingConceptualWormId << "exited merge group" << currentMergeGroupId << "to state" << static_cast<int>(currentState);
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
        if (primaryBlob.isValid && reportingTrackerInstance) {
            processPausedForSplitState(reportingConceptualWormId, originalFrameNumber, primaryBlob, reportingTrackerInstance);
        } else {
            qWarning() << "TM: Worm" << reportingConceptualWormId
                       << "reported PausedForSplit without a valid primaryBlob or tracker instance. Forcing to lost.";
            if (m_pausedWorms.contains(reportingConceptualWormId)) {
                m_pausedWorms.remove(reportingConceptualWormId);
            }
            if (currentMergeGroupId != -1) {
                m_wormToCurrentMergeGroupId[reportingConceptualWormId] = -1;
            }
            if(reportingTrackerInstance){
                Tracking::DetectedBlob invalidBlob; // Default constructor makes it invalid
                QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, invalidBlob));
            }
        }
    }

    if (originalFrameNumber > 0 && originalFrameNumber % 30 == 0) {
        cleanupStaleMergeGroups(originalFrameNumber);
    }
}


void TrackingManager::handleWormTrackerFinished() {
    WormTracker* finishedTracker = qobject_cast<WormTracker*>(sender());
    if (!finishedTracker) {
        qWarning() << "TrackingManager: handleWormTrackerFinished called by non-WormTracker sender.";
        QMutexLocker locker(&m_dataMutex);
        // This case is problematic if we don't know which tracker finished.
        // For now, assume it's one of the expected ones if counts don't match.
        if (m_expectedTrackersToFinish > m_finishedTrackersCount) {
            m_finishedTrackersCount++;
        }
        locker.unlock();
        checkForAllTrackersFinished();
        return;
    }

    int conceptualWormId = finishedTracker->getWormId();
    WormTracker::TrackingDirection direction = finishedTracker->getDirection();

    qDebug() << "TrackingManager: WormTracker for conceptual worm ID" << conceptualWormId
             << "(Dir:" << (direction == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd")
             << ", Ptr: " << finishedTracker << ") reported finished.";

    QMutexLocker locker(&m_dataMutex);
    bool removed = m_wormTrackersList.removeOne(finishedTracker);
    if (!removed) {
        qWarning() << "  Tracker" << finishedTracker << "was not in m_wormTrackersList upon finishing.";
    }

    if(direction == WormTracker::TrackingDirection::Forward) {
        m_wormIdToForwardTrackerInstanceMap.remove(conceptualWormId);
    } else {
        m_wormIdToBackwardTrackerInstanceMap.remove(conceptualWormId);
    }

    m_individualTrackerProgress.remove(finishedTracker); // Remove its progress entry
    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: Total finished trackers:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;
    locker.unlock();

    updateOverallProgress();
    checkForAllTrackersFinished(); // This will check if all are done
}

void TrackingManager::handleWormTrackerError(int reportingWormId, QString errorMessage) {
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    QString trackerInfo = errorTracker ? QString("(Ptr: %1, Dir: %2)").arg(reinterpret_cast<quintptr>(errorTracker)).arg(errorTracker->getDirection() == WormTracker::TrackingDirection::Forward ? "Fwd" : "Bwd") : "";

    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Error from tracker ID" << reportingWormId << trackerInfo << "during/after cancel:" << errorMessage;
        // If cancelled, we still count it as "finished" to let the cancellation process complete.
        if (errorTracker && m_wormTrackersList.removeOne(errorTracker)) {
            m_individualTrackerProgress.remove(errorTracker);
        }
        // Don't increment m_finishedTrackersCount here if it's already counted by finished signal,
        // but ensure it's accounted for if error is the *only* signal.
        // For simplicity, let's assume error means it's "done" one way or another.
        // If not already in finished count, add it.
        // This needs careful thought: if error comes *then* finished, double count.
        // Let's assume an errored tracker will also emit 'finished'. If not, this needs adjustment.
        // For now, we'll rely on 'finished' to increment m_finishedTrackersCount.
        // We just remove it from active lists.
        locker.unlock();
        // updateOverallProgress(); // Progress might be misleading if error
        checkForAllTrackersFinished(); // Check if cancellation can complete
        return;
    }
    if (!m_isTrackingRunning) {
        qDebug() << "TrackingManager: Error from tracker ID" << reportingWormId << trackerInfo << "but tracking not running:" << errorMessage;
        return;
    }
    locker.unlock();

    qWarning() << "TrackingManager: Error from tracker for worm ID" << reportingWormId << trackerInfo << ":" << errorMessage;

    locker.relock();
    if (errorTracker && m_wormTrackersList.removeOne(errorTracker)) {
        m_individualTrackerProgress.remove(errorTracker);
        if(errorTracker->getDirection() == WormTracker::TrackingDirection::Forward) {
            m_wormIdToForwardTrackerInstanceMap.remove(reportingWormId);
        } else {
            m_wormIdToBackwardTrackerInstanceMap.remove(reportingWormId);
        }
        // Consider this tracker "finished" due to error for progress accounting.
        // This assumes 'finished' signal might not come after an error.
        m_finishedTrackersCount++;
        qDebug() << "  Incremented finished trackers due to error. Now:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;
    } else if (!errorTracker) {
        qWarning() << "  Error from unknown sender for worm ID" << reportingWormId << ". Cannot update tracker lists accurately.";
        // Potentially increment finished count if we are short, but this is risky.
        if (m_expectedTrackersToFinish > m_finishedTrackersCount) {
            // m_finishedTrackersCount++;
        }
    }
    locker.unlock();

    emit trackingStatusUpdate(QString("Error with tracker for worm %1. Trying to continue...").arg(reportingWormId));
    updateOverallProgress();
    checkForAllTrackersFinished(); // Check if this error means all (remaining) trackers are done
}


void TrackingManager::handleWormTrackerProgress(int /*reportingWormId*/, int percentDone) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    WormTracker* tracker = qobject_cast<WormTracker*>(sender());

    QMutexLocker locker(&m_dataMutex);
    if (tracker && m_wormTrackersList.contains(tracker)) { // Check if tracker is still considered active
        m_individualTrackerProgress[tracker] = percentDone;
        locker.unlock();
        updateOverallProgress();
    }
    // If tracker is null or not in map, do nothing (it might have finished/errored out)
}


// --- Merge/Split Logic (processMergedState, processPausedForSplitState, etc.) ---
// These methods (calculateIoU, findMatchingMergeGroup, createNewMergeGroup,
// attemptAutomaticSplitResolution, forceResolvePausedWorm, cleanupStaleMergeGroups,
// checkPausedWormsAndResolveSplits) are complex and highly specific to your tracking algorithm.
// For brevity, I'm not reproducing them here, but ensure they correctly use m_dataMutex
// when accessing shared state like m_activeMergeGroups, m_pausedWorms, m_wormToCurrentMergeGroupId.
// The logic provided in your original `trackingmanager.cpp` seems to handle mutexes correctly
// within these functions.

// Placeholder for your existing merge/split logic functions:
void TrackingManager::processMergedState(int reportingWormId, int frameNumber, const Tracking::DetectedBlob& mergedBlobData, WormTracker* reportingTrackerInstance) {
    // (Your existing logic - ensure m_dataMutex is locked if called from a context where it isn't already)
    // This is called from handleFrameUpdate, which already locks m_dataMutex.
    // ...
    QRectF reportedBox = mergedBlobData.boundingBox;
    cv::Point2f reportedCentroid(static_cast<float>(mergedBlobData.centroid.x()), static_cast<float>(mergedBlobData.centroid.y()));
    double reportedArea = mergedBlobData.area;

    TrackedMergeGroup* matchedGroup = findMatchingMergeGroup(frameNumber, reportedBox, reportedCentroid, reportedArea);
    int assignedMergeId = -1;

    if (matchedGroup) {
        assignedMergeId = matchedGroup->uniqueMergeId;
        matchedGroup->participatingWormIds.insert(reportingWormId);
        matchedGroup->individualEstimatedCentroids[reportingWormId] = reportedCentroid; // Store individual estimate
        matchedGroup->lastFrameActive = frameNumber;
        matchedGroup->lastUpdateTime = QDateTime::currentDateTime();
        matchedGroup->currentBoundingBox = matchedGroup->currentBoundingBox.united(reportedBox); // Expand group box
        matchedGroup->currentArea = qMax(matchedGroup->currentArea, reportedArea); // Take max area
        // Group centroid update can be tricky; averaging might be good if blobs are similar size/confidence
        // For now, let's keep it simple: the first worm's centroid or a weighted average if complex.
        // Or, if mergedBlobData.centroid is the "true" merged centroid, use that, but it might jump.
        // Let's assume individual centroids are more useful for potential splits.
        // The group's main centroid could be an average of its participants.
        cv::Point2f sumCentroids(0,0);
        if (!matchedGroup->participatingWormIds.isEmpty()) {
            for(int id : matchedGroup->participatingWormIds) {
                if(matchedGroup->individualEstimatedCentroids.contains(id)){
                    sumCentroids.x += matchedGroup->individualEstimatedCentroids[id].x;
                    sumCentroids.y += matchedGroup->individualEstimatedCentroids[id].y;
                } else { // If a new worm joined and its individual centroid isn't there yet
                    sumCentroids.x += reportedCentroid.x; // Use its current estimate
                    sumCentroids.y += reportedCentroid.y;
                }
            }
            matchedGroup->currentCentroid.x = sumCentroids.x / matchedGroup->participatingWormIds.size();
            matchedGroup->currentCentroid.y = sumCentroids.y / matchedGroup->participatingWormIds.size();
        }


        //qDebug() << "TM: Worm" << reportingWormId << "joined/updated merge group" << assignedMergeId << "in frame" << frameNumber;
    } else {
        assignedMergeId = createNewMergeGroup(frameNumber, reportingWormId, mergedBlobData);
        //qDebug() << "TM: Worm" << reportingWormId << "created new merge group" << assignedMergeId << "in frame" << frameNumber;
    }

    int previousMergeId = m_wormToCurrentMergeGroupId.value(reportingWormId, -1);
    if (previousMergeId != -1 && previousMergeId != assignedMergeId && m_activeMergeGroups.contains(previousMergeId)) {
        TrackedMergeGroup& oldGroup = m_activeMergeGroups[previousMergeId];
        oldGroup.participatingWormIds.remove(reportingWormId);
        oldGroup.individualEstimatedCentroids.remove(reportingWormId);
        if (oldGroup.participatingWormIds.isEmpty()) {
            oldGroup.isValid = false; // Mark for cleanup
        }
    }
    m_wormToCurrentMergeGroupId[reportingWormId] = assignedMergeId;

    if (m_pausedWorms.contains(reportingWormId)) {
        m_pausedWorms.remove(reportingWormId); // Exited pause into merge
        qDebug() << "TM: Worm" << reportingWormId << "exited PausedForSplit into TrackingMerged group" << assignedMergeId;
    }
}

TrackedMergeGroup* TrackingManager::findMatchingMergeGroup(int frameNumber, const QRectF& blobBox, const cv::Point2f& blobCentroid, double /*blobArea*/) {
    // (Your existing logic - m_dataMutex is locked by caller)
    // ...
    TrackedMergeGroup* bestMatch = nullptr;
    double bestMatchScore = -1.0;

    QSet<int> candidateGroupIds;
    if (m_frameToActiveMergeGroupIds.contains(frameNumber)) {
        candidateGroupIds.unite(m_frameToActiveMergeGroupIds.value(frameNumber));
    }
    if (m_frameToActiveMergeGroupIds.contains(frameNumber - 1)) { // Check previous frame too
        candidateGroupIds.unite(m_frameToActiveMergeGroupIds.value(frameNumber - 1));
    }


    for (int groupId : candidateGroupIds) {
        if (!m_activeMergeGroups.contains(groupId) || !m_activeMergeGroups[groupId].isValid) continue;

        TrackedMergeGroup& group = m_activeMergeGroups[groupId];
        // Ensure group was active recently enough if checking previous frame's groups
        if (group.lastFrameActive < frameNumber - 2 && frameNumber > 1) continue; // Allow a small gap

        double iou = calculateIoU(blobBox, group.currentBoundingBox);
        // Calculate distance between blobCentroid and group.currentCentroid
        double dx = blobCentroid.x - group.currentCentroid.x;
        double dy = blobCentroid.y - group.currentCentroid.y;
        double distSq = dx * dx + dy * dy;


        if (iou > MERGE_GROUP_IOU_THRESHOLD && distSq < MERGE_GROUP_CENTROID_MAX_DIST_SQ) {
            // A more sophisticated score could combine IoU and distance.
            // For now, prioritize IoU.
            if (iou > bestMatchScore) {
                bestMatchScore = iou;
                bestMatch = &group;
            }
        }
    }
    return bestMatch;
}

int TrackingManager::createNewMergeGroup(int frameNumber, int initialConceptualWormId, const Tracking::DetectedBlob& mergedBlobData) {
    // (Your existing logic - m_dataMutex is locked by caller)
    // ...
    TrackedMergeGroup newGroup;
    newGroup.uniqueMergeId = m_nextUniqueMergeId++;
    newGroup.currentBoundingBox = mergedBlobData.boundingBox;
    newGroup.currentCentroid = cv::Point2f(static_cast<float>(mergedBlobData.centroid.x()), static_cast<float>(mergedBlobData.centroid.y()));
    newGroup.currentArea = mergedBlobData.area;
    newGroup.participatingWormIds.insert(initialConceptualWormId);
    newGroup.individualEstimatedCentroids[initialConceptualWormId] = newGroup.currentCentroid; // Store its own centroid
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
    // (Your existing logic - m_dataMutex is locked by caller)
    // ...
    if (!reportingTrackerInstance) {
        qWarning() << "TM: processPausedForSplitState called with null tracker instance for worm" << reportingConceptualWormId;
        return;
    }
    if (m_pausedWorms.contains(reportingConceptualWormId)) {
        qDebug() << "TM: Worm" << reportingConceptualWormId << "reported PausedForSplit again. Updating candidate.";
        m_pausedWorms[reportingConceptualWormId].candidateSplitBlob = candidateSplitBlob;
        m_pausedWorms[reportingConceptualWormId].timePaused = QDateTime::currentDateTime(); // Reset timer
        m_pausedWorms[reportingConceptualWormId].trackerInstance = reportingTrackerInstance; // Update instance just in case
        return;
    }

    PausedWormInfo pwi;
    pwi.conceptualWormId = reportingConceptualWormId;
    pwi.trackerInstance = reportingTrackerInstance; // QPointer handles this
    pwi.framePausedOn = frameNumber;
    pwi.timePaused = QDateTime::currentDateTime();
    pwi.candidateSplitBlob = candidateSplitBlob;
    pwi.presumedMergeGroupId_F_minus_1 = m_wormToCurrentMergeGroupId.value(reportingConceptualWormId, -1);

    m_pausedWorms[reportingConceptualWormId] = pwi;
    qDebug() << "TM: Worm" << reportingConceptualWormId << "(Tracker:" << reportingTrackerInstance << ")"
             << "entered PausedForSplit in frame" << frameNumber
             << ". Candidate @ (" << candidateSplitBlob.centroid.x() << "," << candidateSplitBlob.centroid.y() << ")"
             << ". Came from merge group:" << pwi.presumedMergeGroupId_F_minus_1;

    // Detach from its previous merge group
    if (pwi.presumedMergeGroupId_F_minus_1 != -1 && m_activeMergeGroups.contains(pwi.presumedMergeGroupId_F_minus_1)) {
        TrackedMergeGroup& group = m_activeMergeGroups[pwi.presumedMergeGroupId_F_minus_1];
        group.participatingWormIds.remove(reportingConceptualWormId);
        group.individualEstimatedCentroids.remove(reportingConceptualWormId);
        if (group.participatingWormIds.isEmpty()){
            group.isValid = false; // Mark for cleanup
        }
    }
    m_wormToCurrentMergeGroupId[reportingConceptualWormId] = -1; // No longer in a merge group

    // Attempt to resolve immediately if possible (e.g., if buddies are already paused)
    attemptAutomaticSplitResolution(reportingConceptualWormId, m_pausedWorms[reportingConceptualWormId]);
}

void TrackingManager::checkPausedWormsAndResolveSplits() {
    // (Your existing logic - ensure m_dataMutex is locked)
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning || m_pausedWorms.isEmpty()) {
        return;
    }
    // ... (rest of your existing checkPausedWormsAndResolveSplits logic) ...
    QDateTime currentTime = QDateTime::currentDateTime();
    QList<int> pausedConceptualWormIds = m_pausedWorms.keys(); // Iterate over a copy

    for (int conceptualId : pausedConceptualWormIds) {
        if (!m_pausedWorms.contains(conceptualId)) continue; // Already resolved

        PausedWormInfo& pausedInfo = m_pausedWorms[conceptualId];

        if (pausedInfo.timePaused.msecsTo(currentTime) > MAX_PAUSED_DURATION_MS) {
            qDebug() << "TM: Worm" << conceptualId << "in PausedForSplit TIMED OUT. Forcing resolution.";
            forceResolvePausedWorm(conceptualId, pausedInfo); // This will remove from m_pausedWorms
        } else {
            // Try to resolve with buddies if any are available
            attemptAutomaticSplitResolution(conceptualId, pausedInfo); // This might remove from m_pausedWorms
        }
    }
}

void TrackingManager::attemptAutomaticSplitResolution(int pausedConceptualWormId, PausedWormInfo& pausedInfo) {
    // (Your existing logic - m_dataMutex is locked by caller)
    // ...
    if (!pausedInfo.trackerInstance) { // QPointer check
        qWarning() << "TM: No tracker instance for paused worm" << pausedConceptualWormId << ". Cannot resolve split.";
        m_pausedWorms.remove(pausedConceptualWormId);
        return;
    }
    if (!pausedInfo.candidateSplitBlob.isValid) {
        qDebug() << "TM: Worm" << pausedConceptualWormId << "has invalid candidate blob. Forcing lost.";
        Tracking::DetectedBlob invalidBlob;
        QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, invalidBlob));
        m_pausedWorms.remove(pausedConceptualWormId);
        return;
    }

    int presumedMergeId = pausedInfo.presumedMergeGroupId_F_minus_1;
    // If it wasn't part of a merge group, or its merge group is gone, it resolves independently (usually by timeout -> forceResolve)
    if (presumedMergeId == -1 || !m_activeMergeGroups.contains(presumedMergeId) || !m_activeMergeGroups.value(presumedMergeId).isValid) {
        // qDebug() << "TM: Worm" << pausedConceptualWormId << "paused, but no valid prior merge group (" << presumedMergeId << "). Waiting for timeout or independent resolution.";
        // Don't force resolve here, let timeout handle it unless other logic dictates.
        return;
    }

    // Find "buddies": other worms that were in the SAME merge group and paused in the SAME frame.
    QList<int> potentialBuddyConceptualIds;
    const TrackedMergeGroup& sourceGroup = m_activeMergeGroups.value(presumedMergeId); // Should be valid based on check above

    for (auto it = m_pausedWorms.constBegin(); it != m_pausedWorms.constEnd(); ++it) {
        if (it.key() == pausedConceptualWormId) continue; // Don't compare to self
        const PausedWormInfo& otherInfo = it.value();
        if (otherInfo.framePausedOn == pausedInfo.framePausedOn && // Same frame
            otherInfo.presumedMergeGroupId_F_minus_1 == presumedMergeId && // Same source group
            otherInfo.candidateSplitBlob.isValid && otherInfo.trackerInstance) {
            potentialBuddyConceptualIds.append(it.key());
        }
    }

    // Simple 2-worm split scenario:
    // If the original group had 2 worms, and now both are paused with distinct candidates.
    // The original group size check is tricky if worms can join/leave groups dynamically.
    // Let's focus on: if this worm and exactly one buddy are paused from the same group/frame.
    if (potentialBuddyConceptualIds.size() == 1) {
        int buddyConceptualId = potentialBuddyConceptualIds.first();
        if (!m_pausedWorms.contains(buddyConceptualId)) return; // Buddy might have been resolved in same timer tick

        PausedWormInfo& buddyInfo = m_pausedWorms[buddyConceptualId];

        // Check if their candidate blobs are reasonably distinct (low IoU)
        double iou = calculateIoU(pausedInfo.candidateSplitBlob.boundingBox, buddyInfo.candidateSplitBlob.boundingBox);
        if (iou < 0.15) { // Threshold for distinctness
            qDebug() << "TM: Resolving 2-way split for" << pausedConceptualWormId << "and" << buddyConceptualId
                     << "from group" << presumedMergeId << ". IoU:" << iou;

            // Assign targets and remove from paused list
            QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, pausedInfo.candidateSplitBlob));
            m_pausedWorms.remove(pausedConceptualWormId);

            QMetaObject::invokeMethod(buddyInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, buddyInfo.candidateSplitBlob));
            m_pausedWorms.remove(buddyConceptualId);
            return;
        } else {
            // qDebug() << "TM: Worm" << pausedConceptualWormId << "and buddy" << buddyConceptualId << "paused, candidates overlap (IoU:" << iou << "). Waiting.";
        }
    } else if (potentialBuddyConceptualIds.isEmpty()) {
        // No buddies currently paused from the same merge event.
        // This worm might be splitting off alone. Timeout will handle this via forceResolve.
        // qDebug() << "TM: Worm" << pausedConceptualWormId << "paused alone from group" << presumedMergeId << ". Waiting for timeout or buddies.";
    } else {
        // More than 1 buddy paused from the same event. Complex N-way split.
        // Current logic relies on timeout and forceResolve for these complex cases.
        // qDebug() << "TM: Worm" << pausedConceptualWormId << "paused with" << potentialBuddyConceptualIds.size() << "buddies. Complex split, deferring.";
    }
}

void TrackingManager::forceResolvePausedWorm(int conceptualWormId, PausedWormInfo& pausedInfo) {
    // (Your existing logic - m_dataMutex is locked by caller)
    // ...
    qDebug() << "TM: Forcing resolution for paused worm" << conceptualWormId;

    if (!pausedInfo.trackerInstance) { // QPointer check
        qWarning() << "TM: No tracker instance for force-resolving worm" << conceptualWormId << ". Removing from pause queue.";
        m_pausedWorms.remove(conceptualWormId);
        return;
    }

    Tracking::DetectedBlob targetBlob = pausedInfo.candidateSplitBlob;
    if (!targetBlob.isValid) {
        qDebug() << "  Worm" << conceptualWormId << "had no valid candidate blob. Instructing to go lost.";
    } else {
        qDebug() << "  Worm" << conceptualWormId << "is taking its candidate blob at ("
                 << targetBlob.centroid.x() << "," << targetBlob.centroid.y()
                 << ") after timeout/force resolve.";
        // TODO: Could add a check here if targetBlob is already actively tracked by another *non-paused* worm.
        // This is complex. For now, let it try. Tracker might go into merged state if it's the same.
    }

    QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(),
                              "resumeTrackingWithAssignedTarget",
                              Qt::QueuedConnection,
                              Q_ARG(Tracking::DetectedBlob, targetBlob)); // Pass original blob (valid or invalid)

    m_pausedWorms.remove(conceptualWormId);
}

void TrackingManager::cleanupStaleMergeGroups(int currentFrameNumber) {
    // (Your existing logic - m_dataMutex is locked by caller)
    // ...
    QList<int> groupIdsToRemove;
    for (auto it = m_activeMergeGroups.begin(); it != m_activeMergeGroups.end(); ++it) {
        TrackedMergeGroup& group = it.value();
        // Condition for staleness: not updated for a while AND no participants
        bool isStaleAndEmpty = (!group.isValid && group.participatingWormIds.isEmpty() &&
                                (currentFrameNumber - group.lastFrameActive > 20)); // Stale if invalid, empty, and old
        bool veryOld = (currentFrameNumber - group.lastFrameActive > 200); // Very old groups get removed regardless of validity if empty

        if (isStaleAndEmpty || (veryOld && group.participatingWormIds.isEmpty())) {
            groupIdsToRemove.append(it.key());
        } else if (group.isValid && group.participatingWormIds.isEmpty() && !isStaleAndEmpty) {
            // If it just became empty but isn't old yet, mark it invalid.
            // It will be cleaned up later if it remains empty and invalid.
            group.isValid = false;
            group.lastUpdateTime = QDateTime::currentDateTime(); // Mark that we processed it
            // qDebug() << "TM: Merge group" << group.uniqueMergeId << "became empty. Marked invalid.";
        }
    }

    if (!groupIdsToRemove.isEmpty()) {
        qDebug() << "TM: Cleaning up" << groupIdsToRemove.size() << "stale/invalid merge groups.";
        for (int groupId : groupIdsToRemove) {
            m_activeMergeGroups.remove(groupId);
            // Efficiently remove from m_frameToActiveMergeGroupIds only for frames where it was active
            // This requires iterating relevant recent frames or a more complex structure.
            // For now, a simpler approach: iterate recent frames.
            for (int f = qMax(0, currentFrameNumber - 250); f <= currentFrameNumber; ++f) {
                if (m_frameToActiveMergeGroupIds.contains(f)) {
                    m_frameToActiveMergeGroupIds[f].remove(groupId);
                    if (m_frameToActiveMergeGroupIds[f].isEmpty()) {
                        m_frameToActiveMergeGroupIds.remove(f);
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
    // (Your existing logic)
    QRectF intersection = r1.intersected(r2);
    double intersectionArea = intersection.width() * intersection.height();

    if (intersectionArea <= 0) return 0.0;

    double unionArea = (r1.width() * r1.height()) + (r2.width() * r2.height()) - intersectionArea;
    return unionArea > 0 ? (intersectionArea / unionArea) : 0.0;
}

// Output Tracks To CSV
bool TrackingManager::outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFilePath) const {
    // (Your existing logic - seems fine)
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
    outStream << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight,Quality\n";
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
                      << static_cast<int>(point.quality)
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
