// trackingmanager.cpp
#include "trackingmanager.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QPair>
#include <QRectF>

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
    m_videoProcessorsFinishedCount(0),
    m_totalVideoChunksToProcess(0),
    m_videoFps(0.0),
    m_expectedTrackersToFinish(0),
    m_finishedTrackersCount(0),
    m_videoProcessingOverallProgress(0),
    m_nextPhysicalBlobId(1), // Start IDs from 1
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
    qDebug() << "TrackingManager (" << this << ") created with frame-atomic logic.";
}

// Destructor
TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";
    if (m_pauseResolutionTimer) {
        m_pauseResolutionTimer->stop();
    }
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking();
        QThread::msleep(300);
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

    m_videoPath = videoPath;
    m_keyFrameNum = keyFrameNum;
    m_initialWormInfos = initialWorms;
    m_thresholdSettings = settings;
    m_totalFramesInVideoHint = totalFramesInVideoHint;
    m_isTrackingRunning = true;
    m_cancelRequested = false;
    m_videoProcessingOverallProgress = 0;
    m_finishedTrackersCount = 0;
    m_expectedTrackersToFinish = 0;
    m_nextPhysicalBlobId = 1; // Reset for new tracking session

    // Clear data structures
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameMergeRecords.clear();
    m_splitResolutionMap.clear();
    m_pausedWormsRecords.clear();
    m_wormToPhysicalBlobIdMap.clear();

    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();
    for (const auto& info : m_initialWormInfos) {
        if (!m_wormObjectsMap.contains(info.id)) {
            m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
        }
        m_wormToPhysicalBlobIdMap[info.id] = -1; // Initially not part of any physical blob
    }

    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();
    m_videoChunkProgressMap.clear();
    m_videoProcessorsFinishedCount = 0;
    m_totalVideoChunksToProcess = 0;
    m_finalProcessedForwardFrames.clear();
    m_finalProcessedReversedFrames.clear();

    emit trackingStatusUpdate("Initializing video processing...");
    emit overallTrackingProgress(0);

    // --- Determine actual total frames, FPS, and frame size ONCE ---
    // (This logic remains the same as your version with parallel video processing)
    cv::VideoCapture preliminaryCap;
    int actualTotalFrames = 0;
    try {
        if (!preliminaryCap.open(m_videoPath.toStdString())) {
            emit trackingFailed("Failed to open video file for preliminary checks: " + m_videoPath);
            m_isTrackingRunning = false; return;
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
        m_isTrackingRunning = false; return;
    }
    if (actualTotalFrames <= 0 && m_totalFramesInVideoHint > 0) actualTotalFrames = m_totalFramesInVideoHint;
    else if (actualTotalFrames <= 0) { emit trackingFailed("Could not determine total frames."); m_isTrackingRunning = false; return; }
    if (m_videoFps <= 0) m_videoFps = 25.0;
    if (m_videoFrameSize.width <= 0 || m_videoFrameSize.height <= 0) { emit trackingFailed("Could not determine frame size."); m_isTrackingRunning = false; return; }
    if (m_keyFrameNum < 0 || m_keyFrameNum >= actualTotalFrames) { emit trackingFailed("Keyframe out of bounds."); m_isTrackingRunning = false; return; }

    // --- Parallel Video Processing Logic ---
    // (This logic remains the same as your version with parallel video processing)
    int numThreads = QThread::idealThreadCount();
    numThreads = qMax(1, qMin(numThreads, 8));
    emit trackingStatusUpdate(QString("Processing video in chunks across %1 threads...").arg(numThreads));
    QList<QPair<int, int>> forwardFrameRanges, backwardFrameRanges;
    int forwardSegmentStart = m_keyFrameNum, forwardSegmentEnd = actualTotalFrames;
    int totalForwardSegmentFrames = forwardSegmentEnd - forwardSegmentStart;
    if (totalForwardSegmentFrames > 0) {
        int framesPerThread = std::max(1, static_cast<int>(std::ceil(static_cast<double>(totalForwardSegmentFrames) / numThreads)));
        for (int cs = forwardSegmentStart; cs < forwardSegmentEnd; cs += framesPerThread) {
            forwardFrameRanges.append({cs, std::min(cs + framesPerThread, forwardSegmentEnd)});
        }
    }
    int backwardSegmentStart = 0, backwardSegmentEnd = m_keyFrameNum;
    int totalBackwardSegmentFrames = backwardSegmentEnd - backwardSegmentStart;
    if (totalBackwardSegmentFrames > 0) {
        int framesPerThread = std::max(1, static_cast<int>(std::ceil(static_cast<double>(totalBackwardSegmentFrames) / numThreads)));
        for (int cs = backwardSegmentStart; cs < backwardSegmentEnd; cs += framesPerThread) {
            backwardFrameRanges.append({cs, std::min(cs + framesPerThread, backwardSegmentEnd)});
        }
    }
    m_totalVideoChunksToProcess = forwardFrameRanges.size() + backwardFrameRanges.size();
    if (m_totalVideoChunksToProcess == 0) {
        handleInitialProcessingComplete({}, {}, m_videoFps, m_videoFrameSize); return;
    }
    auto launch_chunk_processor = [this](int sF, int eF, bool isFwd) { /* ... same as your version ... */
                                                                       int chunkId = sF; m_videoChunkProgressMap[chunkId] = 0;
                                                                       VideoProcessor* proc = new VideoProcessor(); QThread* thr = new QThread(); proc->moveToThread(thr);
                                                                       m_videoProcessorThreads.append(thr);
                                                                       connect(thr, &QThread::started, proc, [=](){ proc->processFrameRange(m_videoPath, m_thresholdSettings, sF, eF, chunkId, isFwd); });
                                                                       connect(proc, &VideoProcessor::rangeProcessingComplete, this, &TrackingManager::handleRangeProcessingComplete);
                                                                       connect(proc, &VideoProcessor::processingError, this, &TrackingManager::handleVideoChunkProcessingError);
                                                                       connect(proc, &VideoProcessor::rangeProcessingProgress, this, &TrackingManager::handleRangeProcessingProgress);
                                                                       connect(thr, &QThread::finished, proc, &QObject::deleteLater);
                                                                       connect(thr, &QThread::finished, this, [this, tPtr = QPointer<QThread>(thr)](){ if(tPtr){/*qDebug() << "VidProcThr fin"*/;}});
                                                                       thr->start();
    };
    for (const auto& r : forwardFrameRanges) launch_chunk_processor(r.first, r.second, true);
    for (const auto& r : backwardFrameRanges) launch_chunk_processor(r.first, r.second, false);

    if (m_pauseResolutionTimer) {
        m_pauseResolutionTimer->start(PAUSE_RESOLUTION_INTERVAL_MS);
    }
}

void TrackingManager::cancelTracking() {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested && !m_isTrackingRunning) return;
    if (m_cancelRequested) return;
    m_cancelRequested = true;
    bool wasRunning = m_isTrackingRunning;
    locker.unlock();
    emit trackingStatusUpdate("Cancellation requested...");
    if (m_pauseResolutionTimer) m_pauseResolutionTimer->stop();
    for (QPointer<QThread> thread : m_videoProcessorThreads) { if (thread && thread->isRunning()) thread->requestInterruption(); }
    QList<WormTracker*> trackersToStop;
    locker.relock(); trackersToStop = m_wormTrackersList; locker.unlock();
    for (WormTracker* tracker : trackersToStop) { if (tracker) QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection); }
    locker.relock();
    if ((wasRunning && m_expectedTrackersToFinish == 0 && m_wormTrackersList.isEmpty()) || !wasRunning) {
        m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled();
    }
}

void TrackingManager::cleanupThreadsAndObjects() {
    // (Largely same as your version, ensuring QPointer safety and clearing new maps)
    QList<QPointer<QThread>> videoThreadsToClean = m_videoProcessorThreads;
    m_videoProcessorThreads.clear();
    for (QPointer<QThread> thread : videoThreadsToClean) { /* ... quit, wait, delete ... */
        if (thread) { if (thread->isRunning()) { thread->requestInterruption(); thread->quit(); if (!thread->wait(1500)) { thread->terminate(); thread->wait();}} delete thread;}
    }
    QList<QPointer<QThread>> trackerThreadsToClean = m_trackerThreads;
    m_trackerThreads.clear();
    for (QPointer<QThread> thread : trackerThreadsToClean) { /* ... quit, wait, delete ... */
        if (thread) { if (thread->isRunning()) { thread->requestInterruption(); thread->quit(); if (!thread->wait(1000)) { thread->terminate(); thread->wait();}} delete thread;}
    }
    m_wormTrackersList.clear(); m_wormIdToForwardTrackerInstanceMap.clear(); m_wormIdToBackwardTrackerInstanceMap.clear();
    qDeleteAll(m_wormObjectsMap); m_wormObjectsMap.clear();
    m_finalProcessedForwardFrames.clear(); std::vector<cv::Mat>().swap(m_finalProcessedForwardFrames);
    m_finalProcessedReversedFrames.clear(); std::vector<cv::Mat>().swap(m_finalProcessedReversedFrames);
    m_assembledForwardFrameChunks.clear(); m_assembledBackwardFrameChunks.clear(); m_videoChunkProgressMap.clear();
    m_finalTracks.clear(); m_individualTrackerProgress.clear();
    // Clear new data structures
    m_frameMergeRecords.clear();
    m_splitResolutionMap.clear();
    m_pausedWormsRecords.clear();
    m_wormToPhysicalBlobIdMap.clear();
    m_isTrackingRunning = false; m_cancelRequested = false;
}

// --- Video Processing Callbacks ---
void TrackingManager::handleRangeProcessingProgress(int chunkId, int percentage) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    QMutexLocker locker(&m_dataMutex);
    if (m_videoChunkProgressMap.contains(chunkId)) m_videoChunkProgressMap[chunkId] = percentage; else return;
    double totalProgressSum = 0; for (int p : m_videoChunkProgressMap.values()) totalProgressSum += p;
    m_videoProcessingOverallProgress = (m_totalVideoChunksToProcess > 0) ? static_cast<int>(totalProgressSum / m_totalVideoChunksToProcess) : 100;
    locker.unlock(); updateOverallProgress();
}
void TrackingManager::handleRangeProcessingComplete(int chunkId, const std::vector<cv::Mat>& processedFrames, bool wasForwardChunk) {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) return;
    if (wasForwardChunk) m_assembledForwardFrameChunks[chunkId] = processedFrames; else m_assembledBackwardFrameChunks[chunkId] = processedFrames;
    m_videoChunkProgressMap[chunkId] = 100; m_videoProcessorsFinishedCount++;
    if (m_videoProcessorsFinishedCount >= m_totalVideoChunksToProcess) {
        locker.unlock(); assembleProcessedFrames();
    } else {
        double totalProgressSum = 0; for (int p : m_videoChunkProgressMap.values()) totalProgressSum += p;
        m_videoProcessingOverallProgress = (m_totalVideoChunksToProcess > 0) ? static_cast<int>(totalProgressSum / m_totalVideoChunksToProcess) : 100;
        locker.unlock(); updateOverallProgress();
    }
}
void TrackingManager::assembleProcessedFrames() {
    QMutexLocker locker(&m_dataMutex); if (m_cancelRequested) { m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled(); return; }
    m_finalProcessedForwardFrames.clear(); m_finalProcessedReversedFrames.clear();
    for (const auto& chunk : m_assembledForwardFrameChunks.values()) m_finalProcessedForwardFrames.insert(m_finalProcessedForwardFrames.end(), chunk.begin(), chunk.end());
    std::vector<cv::Mat> tempBackwardFrames;
    for (const auto& chunk : m_assembledBackwardFrameChunks.values()) tempBackwardFrames.insert(tempBackwardFrames.end(), chunk.begin(), chunk.end());
    m_finalProcessedReversedFrames = tempBackwardFrames; std::reverse(m_finalProcessedReversedFrames.begin(), m_finalProcessedReversedFrames.end());
    m_assembledForwardFrameChunks.clear(); m_assembledBackwardFrameChunks.clear(); m_videoChunkProgressMap.clear();
    locker.unlock();
    handleInitialProcessingComplete(m_finalProcessedForwardFrames, m_finalProcessedReversedFrames, m_videoFps, m_videoFrameSize);
}
void TrackingManager::handleInitialProcessingComplete(const std::vector<cv::Mat>&, const std::vector<cv::Mat>&, double, cv::Size) {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) { m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled(); return; }
    if (!m_isTrackingRunning) return;
    m_videoProcessingOverallProgress = 100;
    locker.unlock();
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    updateOverallProgress();
    launchWormTrackers();
}
void TrackingManager::handleVideoChunkProcessingError(int chunkId, const QString& errorMessage) {
    QMutexLocker locker(&m_dataMutex); if (m_cancelRequested || !m_isTrackingRunning) return;
    m_cancelRequested = true; m_isTrackingRunning = false;
    if (m_pauseResolutionTimer && m_pauseResolutionTimer->isActive()) m_pauseResolutionTimer->stop();
    locker.unlock();
    for (QPointer<QThread> thread : m_videoProcessorThreads) { if (thread && thread->isRunning()) thread->requestInterruption(); }
    emit trackingFailed("Video processing failed (chunk " + QString::number(chunkId) + "): " + errorMessage);
}

// --- Core Frame Update Logic ---
void TrackingManager::handleFrameUpdate(int reportingConceptualWormId,
                                        int originalFrameNumber,
                                        const Tracking::DetectedBlob& primaryBlob, // For WormObject track
                                        const Tracking::DetectedBlob& fullBlob,    // For merge processing
                                        QRectF searchRoiUsed,
                                        Tracking::TrackerState currentState,
                                        const QList<Tracking::DetectedBlob>& splitCandidates)
{
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* reportingTrackerInstance = qobject_cast<WormTracker*>(sender());
    // Note: WormObject update uses primaryBlob
    WormObject* wormObject = m_wormObjectsMap.value(reportingConceptualWormId, nullptr);
    if (wormObject && primaryBlob.isValid) {
        Tracking::WormTrackPoint point;
        point.frameNumberOriginal = originalFrameNumber;
        point.position = cv::Point2f(static_cast<float>(primaryBlob.centroid.x()), static_cast<float>(primaryBlob.centroid.y()));
        point.roi = searchRoiUsed;
        point.quality = (currentState == Tracking::TrackerState::TrackingSingle) ? Tracking::TrackPointQuality::Confident : Tracking::TrackPointQuality::Ambiguous;
        wormObject->updateTrackPoint(point);
    }

    QString dmsg = QString("TM: WT %1 FN%2 | ").arg(reportingConceptualWormId).arg(originalFrameNumber);
     qDebug().noquote() << dmsg << "State" << static_cast<int>(currentState) << "FullBlobValid:" << fullBlob.isValid;

    if (currentState == Tracking::TrackerState::TrackingSingle || currentState == Tracking::TrackerState::TrackingLost) {
        m_wormToPhysicalBlobIdMap[reportingConceptualWormId] = -1; // No longer part of a specific physical blob
        if (m_pausedWormsRecords.contains(reportingConceptualWormId)) {
             qDebug().noquote() << dmsg << "Worm was paused, now single/lost. Removing from pause records.";
            m_pausedWormsRecords.remove(reportingConceptualWormId);
        }
    } else if (currentState == Tracking::TrackerState::TrackingMerged) {
        if (fullBlob.isValid) {
            processFrameSpecificMerge(reportingConceptualWormId, originalFrameNumber, fullBlob, reportingTrackerInstance);
        } else {
             qDebug().noquote() << dmsg << "State Merged but fullBlob invalid. Treating as lost for merge logic.";
            m_wormToPhysicalBlobIdMap[reportingConceptualWormId] = -1;
        }
    } else if (currentState == Tracking::TrackerState::PausedForSplit) {
        if (!splitCandidates.isEmpty() && reportingTrackerInstance) {
            // Find the chosen candidate (should be the primaryBlob if valid, otherwise first candidate)
            Tracking::DetectedBlob chosenCandidate = primaryBlob.isValid ? primaryBlob :
                                                    (!splitCandidates.isEmpty() ? splitCandidates.first() : Tracking::DetectedBlob());
            processFrameSpecificPause(reportingConceptualWormId, originalFrameNumber, splitCandidates, chosenCandidate, reportingTrackerInstance);
        } else {
             qDebug().noquote() << dmsg << "State PausedForSplit but no candidates/instance. Forcing lost.";
            if (m_pausedWormsRecords.contains(reportingConceptualWormId)) m_pausedWormsRecords.remove(reportingConceptualWormId);
            m_wormToPhysicalBlobIdMap[reportingConceptualWormId] = -1;
            if(reportingTrackerInstance) QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        }
    }
}


void TrackingManager::processFrameSpecificMerge(int conceptualWormId, int frameNumber,
                                                const Tracking::DetectedBlob& reportedFullBlob,
                                                WormTracker* reportingTrackerInstance)
{
    QString dmsg = QString("TM: WT %1 FN%2 | procFrameSpecMerge | ").arg(conceptualWormId).arg(frameNumber);
     qDebug().noquote() << dmsg << "Blob @ " << reportedFullBlob.centroid.x() << "," << reportedFullBlob.centroid.y() << " Area: " << reportedFullBlob.area;

    QList<FrameSpecificPhysicalBlob>& blobsOnThisFrame = m_frameMergeRecords[frameNumber];
    FrameSpecificPhysicalBlob* matchedPhysicalBlob = nullptr;

    for (FrameSpecificPhysicalBlob& existingBlobRecord : blobsOnThisFrame) {
        double iou = calculateIoU(reportedFullBlob.boundingBox, existingBlobRecord.currentBoundingBox);
        bool isContained = existingBlobRecord.currentBoundingBox.contains(reportedFullBlob.centroid);
        //cv::Point2f reportedCentroidCv(static_cast<float>(reportedFullBlob.centroid.x()), static_cast<float>(reportedFullBlob.centroid.y()));
        //double distSq = Tracking::sqDistance(reportedCentroidCv, existingBlobRecord.currentCentroid);

        if (iou > PHYSICAL_BLOB_IOU_THRESHOLD || (isContained && iou > 0.01) /*|| distSq < PHYSICAL_BLOB_CENTROID_MAX_DIST_SQ*/) {
            matchedPhysicalBlob = &existingBlobRecord;
             qDebug().noquote() << dmsg << "Matched existing PhysicalBlobID:" << matchedPhysicalBlob->uniqueId << "IoU:" << iou << "Contained:" << isContained;
            break;
        }
    }

    if (matchedPhysicalBlob) {
        matchedPhysicalBlob->participatingWormTrackerIDs.insert(conceptualWormId);
        // Update the physical blob's representation with this new information
        matchedPhysicalBlob->currentBoundingBox = matchedPhysicalBlob->currentBoundingBox.united(reportedFullBlob.boundingBox);
        matchedPhysicalBlob->currentArea = qMax(matchedPhysicalBlob->currentArea, reportedFullBlob.area); // Or sum, or more complex logic
        if(matchedPhysicalBlob->currentBoundingBox.isValid()){
            QPointF newCenter = matchedPhysicalBlob->currentBoundingBox.center();
            matchedPhysicalBlob->currentCentroid = cv::Point2f(static_cast<float>(newCenter.x()), static_cast<float>(newCenter.y()));
        }
        m_wormToPhysicalBlobIdMap[conceptualWormId] = matchedPhysicalBlob->uniqueId;
         qDebug().noquote() << dmsg << "Added to existing PhysicalBlobID:" << matchedPhysicalBlob->uniqueId << ". Participants:" << matchedPhysicalBlob->participatingWormTrackerIDs;
    } else {
        FrameSpecificPhysicalBlob newPhysicalBlob;
        newPhysicalBlob.uniqueId = m_nextPhysicalBlobId++;
        newPhysicalBlob.frameNumber = frameNumber;
        newPhysicalBlob.currentBoundingBox = reportedFullBlob.boundingBox;
        if(newPhysicalBlob.currentBoundingBox.isValid()){
            QPointF center = newPhysicalBlob.currentBoundingBox.center();
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(center.x()), static_cast<float>(center.y()));
        } else {
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(reportedFullBlob.centroid.x()), static_cast<float>(reportedFullBlob.centroid.y()));
        }
        newPhysicalBlob.currentArea = reportedFullBlob.area;
        newPhysicalBlob.participatingWormTrackerIDs.insert(conceptualWormId);
        // newPhysicalBlob.timeFirstReported = QDateTime::currentDateTime();

        blobsOnThisFrame.append(newPhysicalBlob);
        m_wormToPhysicalBlobIdMap[conceptualWormId] = newPhysicalBlob.uniqueId;
         qDebug().noquote() << dmsg << "Created new PhysicalBlobID:" << newPhysicalBlob.uniqueId << ". Participants:" << newPhysicalBlob.participatingWormTrackerIDs;
    }

    // If this worm was paused, it's no longer paused because it's now merged.
    if (m_pausedWormsRecords.contains(conceptualWormId)) {
         qDebug().noquote() << dmsg << "Worm was paused, now merged. Removing from pause records.";
        m_pausedWormsRecords.remove(conceptualWormId);
    }
}

void TrackingManager::processFrameSpecificPause(int conceptualWormId, int frameNumber,
                                                const QList<Tracking::DetectedBlob>& allSplitCandidates,
                                                const Tracking::DetectedBlob& chosenCandidate,
                                                WormTracker* reportingTrackerInstance)
{
    QString dmsg = QString("TM: WT %1 FN%2 | procFrameSpecPause | ").arg(conceptualWormId).arg(frameNumber);
     qDebug().noquote() << dmsg << "Candidates:" << allSplitCandidates.size() << "Chosen @ " << chosenCandidate.centroid.x() << "," << chosenCandidate.centroid.y();

    if (m_pausedWormsRecords.contains(conceptualWormId)) {
         qDebug().noquote() << dmsg << "Already paused. Updating candidates and timer.";
        m_pausedWormsRecords[conceptualWormId].allSplitCandidates = allSplitCandidates;
        m_pausedWormsRecords[conceptualWormId].chosenCandidate = chosenCandidate;
        m_pausedWormsRecords[conceptualWormId].timePaused = QDateTime::currentDateTime();   // Reset timer
        m_pausedWormsRecords[conceptualWormId].trackerInstance = reportingTrackerInstance; // Ensure instance is current
        return;
    }

    PausedWormInfoFrameSpecific pwi;
    pwi.conceptualWormId = conceptualWormId;
    pwi.trackerInstance = reportingTrackerInstance;
    pwi.framePausedOn = frameNumber;
    pwi.timePaused = QDateTime::currentDateTime();
    pwi.allSplitCandidates = allSplitCandidates;
    pwi.chosenCandidate = chosenCandidate;

    // Determine the presumedPreviousPhysicalBlobId
    pwi.presumedPreviousPhysicalBlobId = m_wormToPhysicalBlobIdMap.value(conceptualWormId, -1);

     qDebug().noquote() << dmsg << "Recorded pause. PrevPhysBlobID:" << pwi.presumedPreviousPhysicalBlobId;
    m_pausedWormsRecords[conceptualWormId] = pwi;
    m_wormToPhysicalBlobIdMap[conceptualWormId] = -1; // No longer part of a specific physical blob *yet*

    attemptAutomaticSplitResolutionFrameSpecific(conceptualWormId, m_pausedWormsRecords[conceptualWormId]);
}


void TrackingManager::attemptAutomaticSplitResolutionFrameSpecific(int conceptualWormIdToResolve, PausedWormInfoFrameSpecific& pausedInfo) {
    QString dmsg = QString("TM: WT %1 FN%2 | attemptAutoSplit | ").arg(conceptualWormIdToResolve).arg(pausedInfo.framePausedOn);
    qDebug().noquote() << dmsg << "PrevPhysBlobID:" << pausedInfo.presumedPreviousPhysicalBlobId;

    if (!pausedInfo.trackerInstance) { /*qWarning() << dmsg << "No tracker instance.";*/ m_pausedWormsRecords.remove(conceptualWormIdToResolve); return; }
    if (!pausedInfo.chosenCandidate.isValid) { /*qDebug() << dmsg << "Invalid candidate. Forcing lost.";*/ QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob())); m_pausedWormsRecords.remove(conceptualWormIdToResolve); return; }
    if (pausedInfo.presumedPreviousPhysicalBlobId == -1) { /*qDebug() << dmsg << "No previous physical blob ID. Cannot find buddies. Will timeout or resolve singly.";*/ return; }

    QList<int> buddyConceptualIds;
    for (const PausedWormInfoFrameSpecific& otherPausedInfo : m_pausedWormsRecords.values()) {
        if (otherPausedInfo.conceptualWormId == conceptualWormIdToResolve) continue;
        if (otherPausedInfo.framePausedOn == pausedInfo.framePausedOn &&
            otherPausedInfo.presumedPreviousPhysicalBlobId == pausedInfo.presumedPreviousPhysicalBlobId &&
            otherPausedInfo.chosenCandidate.isValid && otherPausedInfo.trackerInstance) {
            buddyConceptualIds.append(otherPausedInfo.conceptualWormId);
        }
    }

     qDebug().noquote() << dmsg << "Found" << buddyConceptualIds.size() << "buddies.";

    // Simple 2-worm split scenario (1 original worm + 1 buddy = 2 worms from same previous physical blob)
    if (buddyConceptualIds.size() == 1) {
        int buddyId = buddyConceptualIds.first();
        if (!m_pausedWormsRecords.contains(buddyId)) { /*qDebug() << dmsg << "Buddy" << buddyId << "no longer in pause records.";*/ return; } // Buddy might have resolved

        PausedWormInfoFrameSpecific& buddyInfo = m_pausedWormsRecords[buddyId];

        // Check if this worm (conceptualWormIdToResolve) has already had its split resolved by the buddy
        if (m_splitResolutionMap.value(pausedInfo.framePausedOn).contains(conceptualWormIdToResolve)) {
             qDebug().noquote() << dmsg << "This worm's split already resolved (likely by buddy). Removing from pause.";
            m_pausedWormsRecords.remove(conceptualWormIdToResolve); // Already handled
            return;
        }
        // Check if buddy has already had its split resolved (e.g. by timeout)
        if (m_splitResolutionMap.value(pausedInfo.framePausedOn).contains(buddyId)) {
             qDebug().noquote() << dmsg << "Buddy" << buddyId << "already resolved. This worm (" << conceptualWormIdToResolve << ") must take a different blob.";
            Tracking::DetectedBlob buddyChosenBlob = m_splitResolutionMap.value(pausedInfo.framePausedOn).value(buddyId);
            // TODO: Requires WormTracker to send all candidates. For now, if own candidate is different enough, take it.
            if (calculateIoU(pausedInfo.chosenCandidate.boundingBox, buddyChosenBlob.boundingBox) < 0.1) {
                 qDebug().noquote() << dmsg << "Assigning own candidate as it's different from buddy's choice.";
                QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, pausedInfo.chosenCandidate));
                m_splitResolutionMap[pausedInfo.framePausedOn][conceptualWormIdToResolve] = pausedInfo.chosenCandidate;
                m_pausedWormsRecords.remove(conceptualWormIdToResolve);
            } else {
                 qDebug().noquote() << dmsg << "Own candidate overlaps buddy's choice. Forcing lost for now (TODO: need list of candidates).";
                QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
                m_splitResolutionMap[pausedInfo.framePausedOn][conceptualWormIdToResolve] = Tracking::DetectedBlob(); // Record lost
                m_pausedWormsRecords.remove(conceptualWormIdToResolve);
            }
            return;
        }


        // Standard 2-way split resolution
        double iou = calculateIoU(pausedInfo.chosenCandidate.boundingBox, buddyInfo.chosenCandidate.boundingBox);
        if (iou < 0.1) { // Blobs are distinct
             qDebug().noquote() << dmsg << "Resolving 2-way split with buddy" << buddyId << ". IoU:" << iou;
            if (pausedInfo.trackerInstance) {
                QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, pausedInfo.chosenCandidate));
                m_splitResolutionMap[pausedInfo.framePausedOn][conceptualWormIdToResolve] = pausedInfo.chosenCandidate;
            }
            m_pausedWormsRecords.remove(conceptualWormIdToResolve);

            if (buddyInfo.trackerInstance) {
                QMetaObject::invokeMethod(buddyInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, buddyInfo.chosenCandidate));
                m_splitResolutionMap[buddyInfo.framePausedOn][buddyId] = buddyInfo.chosenCandidate; // framePausedOn should be same
            }
            m_pausedWormsRecords.remove(buddyId);
        } else {
             qDebug().noquote() << dmsg << "Candidates overlap with buddy" << buddyId << "(IoU:" << iou << "). Waiting for timeout or clearer separation.";
        }
    } else if (buddyConceptualIds.size() > 1) {
         qDebug().noquote() << dmsg << "N-way split (" << buddyConceptualIds.size() + 1 << " worms). Deferring to timeout (TODO: implement N-M logic).";
        // For now, N-way splits will resolve via timeout individually.
    }
    // If no buddies, it will also resolve via timeout.
}

void TrackingManager::checkPausedWormsAndResolveSplits() {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning || m_pausedWormsRecords.isEmpty()) {
        return;
    }
    QDateTime currentTime = QDateTime::currentDateTime();
    QList<int> pausedConceptualWormIds = m_pausedWormsRecords.keys(); // Iterate over a copy

    for (int conceptualId : pausedConceptualWormIds) {
        if (!m_pausedWormsRecords.contains(conceptualId)) continue; // Already resolved

        PausedWormInfoFrameSpecific& pausedInfo = m_pausedWormsRecords[conceptualId];
        if (m_splitResolutionMap.value(pausedInfo.framePausedOn).contains(conceptualId)) {
             qDebug() << "TM: Worm" << conceptualId << "on frame" << pausedInfo.framePausedOn << "already has a split resolution. Removing from pause.";
            m_pausedWormsRecords.remove(conceptualId);
            continue;
        }

        if (pausedInfo.timePaused.msecsTo(currentTime) > MAX_PAUSED_DURATION_MS) {
             qDebug() << "TM: Worm" << conceptualId << "FN" << pausedInfo.framePausedOn << " PausedForSplit TIMED OUT. Forcing resolution.";
            forceResolvePausedWormFrameSpecific(conceptualId, pausedInfo);
        } else {
            // Try resolution again, in case a buddy has since paused.
            attemptAutomaticSplitResolutionFrameSpecific(conceptualId, pausedInfo);
        }
    }
}

void TrackingManager::forceResolvePausedWormFrameSpecific(int conceptualWormIdToResolve, PausedWormInfoFrameSpecific& pausedInfo) {
    QString dmsg = QString("TM: WT %1 FN%2 | forceResolve | ").arg(conceptualWormIdToResolve).arg(pausedInfo.framePausedOn);
     qDebug().noquote() << dmsg;

    if (!pausedInfo.trackerInstance) { /*qWarning() << dmsg << "No tracker instance.";*/ m_pausedWormsRecords.remove(conceptualWormIdToResolve); return; }

    // Before assigning, check if any buddy from the same original merge has already claimed a blob.
    Tracking::DetectedBlob blobToAssign = pausedInfo.chosenCandidate; // Default to its chosen candidate

    if (pausedInfo.presumedPreviousPhysicalBlobId != -1) {
        for (int buddyId : m_splitResolutionMap.value(pausedInfo.framePausedOn).keys()) {
            if (buddyId == conceptualWormIdToResolve) continue;

            const Tracking::DetectedBlob& buddyChosenBlob = m_splitResolutionMap.value(pausedInfo.framePausedOn).value(buddyId);
            if (blobToAssign.isValid && buddyChosenBlob.isValid &&
                calculateIoU(blobToAssign.boundingBox, buddyChosenBlob.boundingBox) > 0.5) { // Significant overlap
                 qDebug().noquote() << dmsg << "Chosen candidate overlaps with buddy. Trying alternatives.";

                // Try to find an alternative from allSplitCandidates
                bool foundAlternative = false;
                for (const Tracking::DetectedBlob& candidate : pausedInfo.allSplitCandidates) {
                    if (candidate.isValid && calculateIoU(candidate.boundingBox, buddyChosenBlob.boundingBox) < 0.1) {
                        blobToAssign = candidate;
                        foundAlternative = true;
                        break;
                    }
                }

                if (!foundAlternative) {
                     qDebug().noquote() << dmsg << "No alternative candidate found. Forcing lost.";
                    blobToAssign.isValid = false; // Force lost
                }
                break;
            }
        }
    }

    if (!blobToAssign.isValid) {
         qDebug().noquote() << dmsg << "No valid candidate after considering conflicts. Going lost.";
    } else {
         qDebug().noquote() << dmsg << "Assigning candidate " << blobToAssign.boundingBox;
    }

    QMetaObject::invokeMethod(pausedInfo.trackerInstance.data(), "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, blobToAssign));
    m_splitResolutionMap[pausedInfo.framePausedOn][conceptualWormIdToResolve] = blobToAssign;
    m_pausedWormsRecords.remove(conceptualWormIdToResolve);
}



// --- Helper and Utility functions (calculateIoU, launchWormTrackers, progress, finish handlers etc.) ---
// These remain largely the same as your version with parallel video processing.
// calculateIoU is used by processFrameSpecificMerge.
// launchWormTrackers, updateOverallProgress, checkForAllTrackersFinished, outputTracksToCsv,
// handleWormTrackerFinished, handleWormTrackerError, handleWormTrackerProgress are mostly independent
// of the merge group internal logic, dealing with tracker lifecycle and overall progress.

double TrackingManager::calculateIoU(const QRectF& r1, const QRectF& r2) const {
    QRectF intersection = r1.intersected(r2);
    double intersectionArea = intersection.width() * intersection.height();
    if (intersectionArea <= 0) return 0.0;
    double unionArea = (r1.width() * r1.height()) + (r2.width() * r2.height()) - intersectionArea;
    return unionArea > 0 ? (intersectionArea / unionArea) : 0.0;
}

void TrackingManager::launchWormTrackers() { /* ... same as your version ... */
    if (m_cancelRequested) { m_isTrackingRunning = false; emit trackingCancelled(); return; }
    if (m_initialWormInfos.empty()) { emit trackingStatusUpdate("No worms selected."); emit trackingFinishedSuccessfully(""); m_isTrackingRunning = false; return; }
    m_expectedTrackersToFinish = 0; m_finishedTrackersCount = 0; m_individualTrackerProgress.clear();
    m_wormIdToForwardTrackerInstanceMap.clear(); m_wormIdToBackwardTrackerInstanceMap.clear();
    for(QPointer<QThread> t : m_trackerThreads) { if(t) delete t; } m_trackerThreads.clear();

    for (const auto& info : m_initialWormInfos) {
        int wId = info.id; QRectF iRoi = info.initialRoi;
        if (!m_finalProcessedForwardFrames.empty() || m_keyFrameNum == (m_videoFrameSize.width > 0 ? static_cast<int>(m_videoFps * (m_totalFramesInVideoHint > 0 ? m_totalFramesInVideoHint : 1) / m_videoFps) -1 : 0) ) {
            WormTracker* trk = new WormTracker(wId, iRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
            trk->setFrames(&m_finalProcessedForwardFrames); QThread* thr = new QThread(); trk->moveToThread(thr);
            trk->setProperty("wormId", wId); trk->setProperty("direction", "Forward"); m_trackerThreads.append(thr);
            connect(thr, &QThread::started, trk, &WormTracker::startTracking);
            connect(trk, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(thr, &QThread::finished, trk, &QObject::deleteLater);
            connect(thr, &QThread::finished, this, [this, tPtr=QPointer<QThread>(thr), wId](){if(tPtr){/*qDebug()<<"FwdTrkThr fin" << wId;*/}});
            connect(trk, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(trk, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(trk, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
            m_wormTrackersList.append(trk); m_wormIdToForwardTrackerInstanceMap[wId] = trk; m_individualTrackerProgress[trk] = 0;
            thr->start(); m_expectedTrackersToFinish++;
        }
        if (!m_finalProcessedReversedFrames.empty() || m_keyFrameNum == 0) {
            WormTracker* trk = new WormTracker(wId, iRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
            trk->setFrames(&m_finalProcessedReversedFrames); QThread* thr = new QThread(); trk->moveToThread(thr);
            trk->setProperty("wormId", wId); trk->setProperty("direction", "Backward"); m_trackerThreads.append(thr);
            connect(thr, &QThread::started, trk, &WormTracker::startTracking);
            connect(trk, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(thr, &QThread::finished, trk, &QObject::deleteLater);
            connect(thr, &QThread::finished, this, [this, tPtr=QPointer<QThread>(thr), wId](){if(tPtr){/*qDebug()<<"BwdTrkThr fin"<< wId;*/}});
            connect(trk, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(trk, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(trk, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
            m_wormTrackersList.append(trk); m_wormIdToBackwardTrackerInstanceMap[wId] = trk; m_individualTrackerProgress[trk] = 0;
            thr->start(); m_expectedTrackersToFinish++;
        }
    }
    if (m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) { emit trackingFailed("No trackers launched."); m_isTrackingRunning = false; if(m_pauseResolutionTimer) m_pauseResolutionTimer->stop(); }
    else if (m_expectedTrackersToFinish > 0) { emit trackingStatusUpdate(QString("Launching %1 trackers...").arg(m_expectedTrackersToFinish)); }
    updateOverallProgress();
}
void TrackingManager::updateOverallProgress() { /* ... same as your version ... */
    if (m_cancelRequested && !m_isTrackingRunning) { emit overallTrackingProgress(m_videoProcessingOverallProgress); return; }
    if (!m_isTrackingRunning && !m_cancelRequested) { emit overallTrackingProgress(0); return; }
    double totalProgressValue = 0.0; double videoProcWeight = (m_expectedTrackersToFinish > 0 || m_initialWormInfos.empty()) ? 0.20 : 1.0;
    double trackersWeight = 1.0 - videoProcWeight;
    totalProgressValue += (static_cast<double>(m_videoProcessingOverallProgress) / 100.0) * videoProcWeight;
    double overallTrackerPercentage = 0.0;
    { QMutexLocker locker(&m_dataMutex);
        if (m_expectedTrackersToFinish > 0) {
            double currentPoints = 0; for(int p : m_individualTrackerProgress.values()) currentPoints += p;
            currentPoints += (static_cast<double>(m_finishedTrackersCount) * 100.0);
            double maxPoints = static_cast<double>(m_expectedTrackersToFinish) * 100.0;
            if (maxPoints > 0) overallTrackerPercentage = currentPoints / maxPoints;
            overallTrackerPercentage = qBound(0.0, overallTrackerPercentage, 1.0);
        } else if (m_videoProcessingOverallProgress == 100 && m_initialWormInfos.empty()) overallTrackerPercentage = 1.0;
        else if (m_videoProcessingOverallProgress == 100 && m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) overallTrackerPercentage = 0.0;
    }
    if (m_expectedTrackersToFinish > 0 || (m_initialWormInfos.empty() && videoProcWeight < 1.0) ) totalProgressValue += overallTrackerPercentage * trackersWeight;
    emit overallTrackingProgress(qBound(0, static_cast<int>(totalProgressValue * 100.0), 100));
}
void TrackingManager::checkForAllTrackersFinished() { /* ... same as your version ... */
    QMutexLocker locker(&m_dataMutex);
    bool allDoneOrCancelled = false;
    if (m_isTrackingRunning && (m_finishedTrackersCount >= m_expectedTrackersToFinish)) allDoneOrCancelled = true;
    else if (m_cancelRequested && (m_finishedTrackersCount >= m_expectedTrackersToFinish || m_expectedTrackersToFinish == 0) ) allDoneOrCancelled = true;

    if (allDoneOrCancelled) {
        bool wasCancelled = m_cancelRequested; m_isTrackingRunning = false;
        if (m_pauseResolutionTimer && m_pauseResolutionTimer->isActive()) m_pauseResolutionTimer->stop();
        locker.unlock();
        if (wasCancelled) {
            m_finalTracks.clear(); for (WormObject* w : m_wormObjectsMap.values()) { if(w) m_finalTracks[w->getId()] = w->getTrackHistory(); }
            if (!m_finalTracks.empty()) emit allTracksUpdated(m_finalTracks);
            emit trackingStatusUpdate("Tracking cancelled."); emit trackingCancelled();
        } else {
            m_finalTracks.clear(); for (WormObject* w : m_wormObjectsMap.values()) { if(w) m_finalTracks[w->getId()] = w->getTrackHistory(); }
            emit allTracksUpdated(m_finalTracks); QString csvPath = m_videoPath.isEmpty() ? "tracks.csv" : QDir(QFileInfo(m_videoPath).absolutePath()).filePath(QFileInfo(m_videoPath).completeBaseName() + "_tracks.csv");
            if (outputTracksToCsv(m_finalTracks, csvPath)) { emit trackingStatusUpdate("Tracks saved: " + csvPath); emit trackingFinishedSuccessfully(csvPath); }
            else { emit trackingStatusUpdate("Failed to save CSV: " + csvPath); emit trackingFailed("Failed to save CSV."); }
        }
        QMetaObject::invokeMethod(this, "cleanupThreadsAndObjects", Qt::QueuedConnection);
    } else if (!m_isTrackingRunning && m_cancelRequested) {
        locker.unlock(); emit trackingCancelled();
    }
}
void TrackingManager::handleWormTrackerFinished() { /* ... same as your version ... */
    WormTracker* ft = qobject_cast<WormTracker*>(sender()); if (!ft) { QMutexLocker l(&m_dataMutex); if(m_expectedTrackersToFinish > m_finishedTrackersCount) m_finishedTrackersCount++; l.unlock(); checkForAllTrackersFinished(); return; }
    int cId = ft->getWormId(); WormTracker::TrackingDirection dir = ft->getDirection();
    QMutexLocker locker(&m_dataMutex); m_wormTrackersList.removeOne(ft);
    if(dir == WormTracker::TrackingDirection::Forward) m_wormIdToForwardTrackerInstanceMap.remove(cId); else m_wormIdToBackwardTrackerInstanceMap.remove(cId);
    m_individualTrackerProgress.remove(ft); m_finishedTrackersCount++; locker.unlock();
    updateOverallProgress(); checkForAllTrackersFinished();
}
void TrackingManager::handleWormTrackerError(int reportingWormId, QString errorMessage) { /* ... same as your version ... */
    WormTracker* et = qobject_cast<WormTracker*>(sender());
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) { /* Handle error during cancel/stop */ if(et && m_wormTrackersList.removeOne(et)){ m_individualTrackerProgress.remove(et); if(m_expectedTrackersToFinish > m_finishedTrackersCount) m_finishedTrackersCount++;} locker.unlock(); updateOverallProgress(); checkForAllTrackersFinished(); return; }
    locker.unlock();
    //qWarning() << "Error from tracker" << reportingWormId << ":" << errorMessage;
    locker.relock();
    if (et && m_wormTrackersList.removeOne(et)) {
        m_individualTrackerProgress.remove(et);
        if(et->getDirection() == WormTracker::TrackingDirection::Forward) m_wormIdToForwardTrackerInstanceMap.remove(reportingWormId); else m_wormIdToBackwardTrackerInstanceMap.remove(reportingWormId);
        m_finishedTrackersCount++;
    } else if (!et && m_expectedTrackersToFinish > m_finishedTrackersCount) { m_finishedTrackersCount++; }
    locker.unlock();
    emit trackingStatusUpdate(QString("Error tracker %1.").arg(reportingWormId));
    updateOverallProgress(); checkForAllTrackersFinished();
}
void TrackingManager::handleWormTrackerProgress(int, int percentDone) { /* ... same as your version ... */
    if (m_cancelRequested || !m_isTrackingRunning) return; WormTracker* trk = qobject_cast<WormTracker*>(sender());
    QMutexLocker locker(&m_dataMutex); if (trk && m_wormTrackersList.contains(trk)) { m_individualTrackerProgress[trk] = percentDone; locker.unlock(); updateOverallProgress(); }
}
bool TrackingManager::outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFilePath) const { /* ... same as your version ... */
    if (outputFilePath.isEmpty()) return false; QFile f(outputFilePath); if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return false;
    QTextStream o(&f); o << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight,Quality\n";
    for (auto const& [wId, tps] : tracks) { for (const Tracking::WormTrackPoint& p : tps) {
            o << wId << "," << p.frameNumberOriginal << "," << QString::number(p.position.x, 'f', 4) << "," << QString::number(p.position.y, 'f', 4) << ","
              << QString::number(p.roi.x(), 'f', 2) << "," << QString::number(p.roi.y(), 'f', 2) << ","
              << QString::number(p.roi.width(), 'f', 2) << "," << QString::number(p.roi.height(), 'f', 2) << ","
              << static_cast<int>(p.quality) << "\n"; }}
    f.close(); return f.error() == QFile::NoError;
}
