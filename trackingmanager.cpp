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

void TrackingManager::handleFrameUpdate(int reportingConceptualWormId,
                                        int originalFrameNumber,
                                        const Tracking::DetectedBlob& primaryBlob,
                                        const Tracking::DetectedBlob& fullBlob,
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
        if (fullBlob.isValid) {
            processMergedState(reportingConceptualWormId, originalFrameNumber, fullBlob, reportingTrackerInstance);
        }
    } else if (currentState == Tracking::TrackerState::PausedForSplit) {
        if (fullBlob.isValid && reportingTrackerInstance) { // Need tracker instance to resume it
            processPausedForSplitState(reportingConceptualWormId, originalFrameNumber, fullBlob, reportingTrackerInstance);
        } else {
            qWarning() << "TM: Worm" << reportingConceptualWormId
                       << "reported PausedForSplit without a valid fullBlob or tracker instance. Forcing to lost.";
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
