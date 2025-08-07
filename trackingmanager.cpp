// trackingmanager.cpp
#include "trackingmanager.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QPair>
#include <QRectF>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

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
    m_storage(nullptr)
{
    registerMetaTypes();
}

TrackingManager::TrackingManager(TrackingDataStorage* storage, QObject* parent)
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
    m_storage(storage)
{
    registerMetaTypes();
}

void TrackingManager::registerMetaTypes()
{

    qRegisterMetaType<QMap<int, std::vector<Tracking::WormTrackPoint>>>("QMap<int, std::vector<Tracking::WormTrackPoint>>");
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<Tracking::AllWormTracks>("Tracking::AllWormTracks");
    qRegisterMetaType<QList<Tracking::DetectedBlob>>("QList<Tracking::DetectedBlob>");
    qRegisterMetaType<Tracking::DetectedBlob>("Tracking::DetectedBlob");
    qRegisterMetaType<Tracking::TrackerState>("Tracking::TrackerState");
    qDebug() << "TrackingManager (" << this << ") created with frame-atomic logic. Timer eliminated for direct resolution.";
}

// Destructor
TrackingManager::~TrackingManager() {
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";

    // Ensure tracking is cancelled and resources are released
    if (m_isTrackingRunning) {
        qDebug() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking();
        QThread::msleep(300);
    }

    // Force a final cleanup to ensure all resources are released
    cleanupThreadsAndObjects();
    qDebug() << "TrackingManager (" << this << ") DESTRUCTOR - FINISHED cleaning up.";
}

// Main entry point for tracking
void TrackingManager::startFullTrackingProcess(
    const QString& videoPath, const QString& dataDirectory, int keyFrameNum,
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
    m_dataDirectory = dataDirectory;
    m_keyFrameNum = keyFrameNum;
    m_initialWormInfos = initialWorms;
    m_thresholdSettings = settings;
    m_totalFramesInVideoHint = totalFramesInVideoHint;
    
    // Create video-specific directory and save JSON files
    m_videoSpecificDirectory = createVideoSpecificDirectory(dataDirectory, videoPath);
    if (!m_videoSpecificDirectory.isEmpty()) {
        saveThresholdSettings(m_videoSpecificDirectory, settings);
        saveInputBlobs(m_videoSpecificDirectory, initialWorms);
        
        // Check if threshold settings differ from existing ones
        QString thresholdFilePath = QDir(m_videoSpecificDirectory).absoluteFilePath("thresh_settings.json");
        if (QFile::exists(thresholdFilePath)) {
            bool settingsMatch = compareThresholdSettings(thresholdFilePath, settings);
            if (!settingsMatch) {
                qDebug() << "Current threshold settings differ from stored settings in" << thresholdFilePath;
                emit trackingStatusUpdate("Threshold settings differ from previous run");
            }
        }
    }
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

    // Timer-based pause resolution has been eliminated in favor of direct resolution
    qDebug() << "TM: Using direct split resolution instead of timer-based approach";
}

void TrackingManager::cancelTracking() {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested && !m_isTrackingRunning) return;
    if (m_cancelRequested) return;
    m_cancelRequested = true;
    bool wasRunning = m_isTrackingRunning;

    // Clear the split resolution map
    m_splitResolutionMap.clear();

    locker.unlock();
    emit trackingStatusUpdate("Cancellation requested...");
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
    // Clear all data structures related to split resolution
    m_frameMergeRecords.clear();
    m_splitResolutionMap.clear();
    m_wormToPhysicalBlobIdMap.clear();

    // Reset state flags
    m_isTrackingRunning = false;
    m_cancelRequested = false;

    qDebug() << "TrackingManager: All data structures cleared in cleanupThreadsAndObjects - split states eliminated";
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
    locker.unlock();
    for (QPointer<QThread> thread : m_videoProcessorThreads) { if (thread && thread->isRunning()) thread->requestInterruption(); }
    emit trackingFailed("Video processing failed (chunk " + QString::number(chunkId) + "): " + errorMessage);
}

// --- Helper function for signed worm IDs ---
int TrackingManager::getSignedWormId(int conceptualWormId, WormTracker::TrackingDirection direction) {
    return (direction == WormTracker::TrackingDirection::Forward) ? conceptualWormId : -conceptualWormId;
}

int TrackingManager::getUnsignedWormId(int signedWormId) {
    return qAbs(signedWormId);
}

WormTracker::TrackingDirection TrackingManager::getDirectionFromSignedId(int signedWormId) {
    return (signedWormId >= 0) ? WormTracker::TrackingDirection::Forward : WormTracker::TrackingDirection::Backward;
}

// --- Core Frame Update Logic ---
void TrackingManager::handleFrameUpdate(int reportingConceptualWormId,
                                           int originalFrameNumber,
                                           const Tracking::DetectedBlob& primaryBlob, // Anchor blob for track history
                                           const Tracking::DetectedBlob& fullBlob,    // Full blob for merge/state processing
                                           QRectF searchRoiUsed,
                                           Tracking::TrackerState currentState,
                                           const QList<Tracking::DetectedBlob>& splitCandidates)
{
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* reportingTrackerInstance = qobject_cast<WormTracker*>(sender());

    // Get direction and calculate signed worm ID for pause handling
    WormTracker::TrackingDirection direction = reportingTrackerInstance ? reportingTrackerInstance->getDirection() : WormTracker::TrackingDirection::Forward;
    int signedWormId = getSignedWormId(reportingConceptualWormId, direction);

    QString dmsg = QString("TM: WT %1 FN%2 | ").arg(signedWormId).arg(originalFrameNumber);
    qDebug().noquote() << dmsg << " State " << static_cast<int>(currentState) << " FullBlobValid: " << fullBlob.isValid;
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

    //QString dmsg = QString("TM: WT %1 FN%2 | ").arg(reportingConceptualWormId).arg(originalFrameNumber);
    // qDebug().noquote() << dmsg << "State" << static_cast<int>(currentState) << "FullBlobValid:" << fullBlob.isValid;

    if (currentState == Tracking::TrackerState::TrackingSingle || currentState == Tracking::TrackerState::TrackingLost) {
        m_wormToPhysicalBlobIdMap[signedWormId] = -1; // No longer part of a specific physical blob
    } else if (currentState == Tracking::TrackerState::TrackingMerged) {
        if (fullBlob.isValid) {
            processFrameSpecificMerge(signedWormId, originalFrameNumber, fullBlob, reportingTrackerInstance);
        } else {
             qDebug().noquote() << dmsg << "State Merged but fullBlob invalid. Treating as lost for merge logic.";
            m_wormToPhysicalBlobIdMap[signedWormId] = -1;
        }
    } else if (currentState == Tracking::TrackerState::PausedForSplit) {
        if (!splitCandidates.isEmpty() && reportingTrackerInstance) {
            // Find the chosen candidate (should be the primaryBlob if valid, otherwise first candidate)
            Tracking::DetectedBlob chosenCandidate = primaryBlob.isValid ? primaryBlob :
                                                    (!splitCandidates.isEmpty() ? splitCandidates.first() : Tracking::DetectedBlob());
            processFrameSpecificSplit(signedWormId, originalFrameNumber, splitCandidates, chosenCandidate, reportingTrackerInstance);
        } else {
             qDebug().noquote() << dmsg << "State PausedForSplit but no candidates/instance. Forcing lost.";
            m_wormToPhysicalBlobIdMap[signedWormId] = -1;
            if(reportingTrackerInstance) QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        }
    }

    // All split resolution is now immediate without any paused state
}


void TrackingManager::processFrameSpecificMerge(int signedWormId, int frameNumber,
                                                const Tracking::DetectedBlob& reportedFullBlob,
                                                WormTracker* reportingTrackerInstance)
{
    int conceptualWormId = getUnsignedWormId(signedWormId);
    QString dmsg = QString("TM: WT %1 FN%2 | procFrameSpecMerge | ").arg(signedWormId).arg(frameNumber);
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
        matchedPhysicalBlob->participatingWormTrackerIDs.insert(signedWormId);
        // Update the physical blob's representation with this new information
        matchedPhysicalBlob->currentBoundingBox = matchedPhysicalBlob->currentBoundingBox.united(reportedFullBlob.boundingBox);
        matchedPhysicalBlob->currentArea = qMax(matchedPhysicalBlob->currentArea, reportedFullBlob.area); // ideally should not change
        if(matchedPhysicalBlob->currentBoundingBox.isValid()){
            QPointF newCenter = matchedPhysicalBlob->currentBoundingBox.center();
            matchedPhysicalBlob->currentCentroid = cv::Point2f(static_cast<float>(newCenter.x()), static_cast<float>(newCenter.y()));
        }
        m_wormToPhysicalBlobIdMap[signedWormId] = matchedPhysicalBlob->uniqueId;
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
        newPhysicalBlob.participatingWormTrackerIDs.insert(signedWormId);
        // newPhysicalBlob.timeFirstReported = QDateTime::currentDateTime();

        blobsOnThisFrame.append(newPhysicalBlob);
        m_wormToPhysicalBlobIdMap[signedWormId] = newPhysicalBlob.uniqueId;
         qDebug().noquote() << dmsg << "Created new PhysicalBlobID:" << newPhysicalBlob.uniqueId << ". Participants:" << newPhysicalBlob.participatingWormTrackerIDs;
    }

    // Merge state is fully handled with the frame-specific physical blob representation
}

void TrackingManager::processFrameSpecificSplit(int signedWormId, int frameNumber,
                                               const QList<Tracking::DetectedBlob>& allSplitCandidates,
                                               const Tracking::DetectedBlob& chosenCandidate,
                                               WormTracker* reportingTrackerInstance)
{
    int conceptualWormId = getUnsignedWormId(signedWormId);
    QString dmsg = QString("TM: WT %1 FN%2 | procFrameSpecSplit | ").arg(signedWormId).arg(frameNumber);
    qDebug().noquote() << dmsg << "Candidates:" << allSplitCandidates.size() << "Chosen @ " << chosenCandidate.centroid.x() << "," << chosenCandidate.centroid.y() << " Area: " << chosenCandidate.area;

    // Get reference to current frame's physical blobs
    QList<FrameSpecificPhysicalBlob>& blobsOnThisFrame = m_frameMergeRecords[frameNumber];

    // Step 1: Create PhysicalBlobIds for all split candidates if they don't exist
    QList<int> candidatePhysicalBlobIds;
    int chosenCandidatePhysicalBlobId = -1; // Track which physical blob ID corresponds to the chosen candidate

    for (const Tracking::DetectedBlob& candidate : allSplitCandidates) {
        if (!candidate.isValid) continue;

        // Check if this candidate matches an existing physical blob
        FrameSpecificPhysicalBlob* existingBlob = nullptr;
        for (FrameSpecificPhysicalBlob& blob : blobsOnThisFrame) {
            double iou = calculateIoU(candidate.boundingBox, blob.currentBoundingBox);
            bool isContained = blob.currentBoundingBox.contains(candidate.centroid);

            if (iou > PHYSICAL_BLOB_IOU_THRESHOLD || (isContained && iou > 0.01)) {
                existingBlob = &blob;
                break;
            }
        }

        int currentBlobId = -1;
        if (existingBlob) {
            // Add this WT to the existing blob's participants
            existingBlob->participatingWormTrackerIDs.insert(signedWormId);
            currentBlobId = existingBlob->uniqueId;
            candidatePhysicalBlobIds.append(currentBlobId);
            qDebug().noquote() << dmsg << "Matched existing PhysicalBlobID:" << currentBlobId;
        } else {
            // Create new physical blob for this candidate
            FrameSpecificPhysicalBlob newPhysicalBlob;
            newPhysicalBlob.uniqueId = m_nextPhysicalBlobId++;
            newPhysicalBlob.frameNumber = frameNumber;
            newPhysicalBlob.currentBoundingBox = candidate.boundingBox;
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(candidate.centroid.x()), static_cast<float>(candidate.centroid.y()));
            newPhysicalBlob.currentArea = candidate.area;
            newPhysicalBlob.participatingWormTrackerIDs.insert(signedWormId);
            newPhysicalBlob.selectedByWormTrackerId = 0; // Initially unselected

            blobsOnThisFrame.append(newPhysicalBlob);
            currentBlobId = newPhysicalBlob.uniqueId;
            candidatePhysicalBlobIds.append(currentBlobId);
            qDebug().noquote() << dmsg << "Created new PhysicalBlobID:" << currentBlobId << "for split candidate";
        }

        // Check if this is the chosen candidate (by comparing centroids)
        if (qFuzzyCompare(candidate.centroid.x(), chosenCandidate.centroid.x()) &&
            qFuzzyCompare(candidate.centroid.y(), chosenCandidate.centroid.y())) {
            chosenCandidatePhysicalBlobId = currentBlobId;
            qDebug().noquote() << dmsg << "Identified chosen candidate as PhysicalBlobID:" << currentBlobId;
        }
    }

    // Set the preferred blob for this worm to be the chosen candidate
    if (chosenCandidatePhysicalBlobId != -1) {
        m_wormToPhysicalBlobIdMap[conceptualWormId] = chosenCandidatePhysicalBlobId;
        qDebug().noquote() << dmsg << "Setting preferred blob for worm" << conceptualWormId << "to PhysicalBlobID:" << chosenCandidatePhysicalBlobId;
    }

    // Try immediate resolution using our new method
    if (attemptImmediateSplitResolution(signedWormId, frameNumber, allSplitCandidates, chosenCandidate, reportingTrackerInstance)) {
        qDebug().noquote() << dmsg << "Split successfully resolved immediately";
    } else {
        // Fallback - assign the first valid blob or go lost
        qDebug().noquote() << dmsg << "Immediate resolution failed, using fallback (go lost)";
        QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][signedWormId] = Tracking::DetectedBlob();
    }
}


bool TrackingManager::attemptImmediateSplitResolution(int conceptualWormId, int frameNumber,
                                                    const QList<Tracking::DetectedBlob>& allCandidates,
                                                    const Tracking::DetectedBlob& chosenCandidate,
                                                    WormTracker* trackerInstance) {
    QString dmsg = QString("TM: WT %1 FN%2 | attemptImmediateSplit | ").arg(conceptualWormId).arg(frameNumber);
    qDebug().noquote() << dmsg << "Starting immediate split resolution";

    if (!trackerInstance) { qWarning() << dmsg << "No tracker instance."; return false; }
    if (!chosenCandidate.isValid) { qDebug() << dmsg << "Invalid candidate. Forcing lost."; QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob())); return true; }

    // Find physical blob IDs for this worm
    QList<int> thisWormPhysicalBlobIds;
    if (m_frameMergeRecords.contains(frameNumber)) {
        for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
            if (blob.participatingWormTrackerIDs.contains(conceptualWormId)) {
                thisWormPhysicalBlobIds.append(blob.uniqueId);
            }
        }
    }

    if (thisWormPhysicalBlobIds.isEmpty()) {
        qDebug() << dmsg << "No PhysicalBlobIds found for this worm. Cannot proceed with resolution.";
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection,
                                 Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][conceptualWormId] = Tracking::DetectedBlob();
        return true;
    }

    qDebug().noquote() << dmsg << "This worm's PhysicalBlobIds:" << thisWormPhysicalBlobIds;

    // Check if a preferred blob is already assigned to this worm
    int preferredBlobId = m_wormToPhysicalBlobIdMap.value(conceptualWormId, -1);
    Tracking::DetectedBlob blobToAssign;
    blobToAssign.isValid = false;
    bool resolutionSuccess = false;

    // Look for already resolved worms on this frame that might have taken blobs we're interested in
    QMap<int, QList<int>> otherWormBlobAssignments; // wormId -> list of blobIds

    if (m_splitResolutionMap.contains(frameNumber)) {
        QMap<int, Tracking::DetectedBlob> resolvedBlobs = m_splitResolutionMap[frameNumber];
        for (auto it = resolvedBlobs.begin(); it != resolvedBlobs.end(); ++it) {
            int otherWormId = it.key();
            if (otherWormId == conceptualWormId) continue;

            QList<int> otherWormBlobIds;
            for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.selectedByWormTrackerId == otherWormId) {
                    otherWormBlobIds.append(blob.uniqueId);
                }
            }

            if (!otherWormBlobIds.isEmpty()) {
                otherWormBlobAssignments[otherWormId] = otherWormBlobIds;
                qDebug().noquote() << dmsg << "Worm" << otherWormId << "already has blobs assigned:" << otherWormBlobIds;
            }
        }
    }

    // First, try to find our preferred blob if it's available
    if (preferredBlobId != -1) {
        bool blobTaken = false;

        // Check if any other worm has already been assigned this blob
        for (auto it = otherWormBlobAssignments.begin(); it != otherWormBlobAssignments.end(); ++it) {
            if (it.value().contains(preferredBlobId)) {
                blobTaken = true;
                qDebug().noquote() << dmsg << "Preferred blob" << preferredBlobId << "already taken by worm" << it.key();
                break;
            }
        }

        if (!blobTaken) {
            // Try to find and claim this blob
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.uniqueId == preferredBlobId && blob.selectedByWormTrackerId == 0) {
                    // Claim the blob
                    blob.selectedByWormTrackerId = conceptualWormId;

                    // Create the blob to assign
                    blobToAssign.isValid = true;
                    blobToAssign.centroid = QPointF(blob.currentCentroid.x, blob.currentCentroid.y);
                    blobToAssign.boundingBox = blob.currentBoundingBox;
                    blobToAssign.area = blob.currentArea;

                    qDebug().noquote() << dmsg << "Successfully claimed preferred blob:" << preferredBlobId;
                    resolutionSuccess = true;
                    break;
                }
            }
        }
    }

    // If we couldn't claim our preferred blob, look for any available alternative
    if (!blobToAssign.isValid) {
        qDebug().noquote() << dmsg << "Preferred blob unavailable or not set. Looking for alternatives.";

        // Get the worm tracker instance to find its last known position
        //WormTracker* tracker = nullptr;
        //for (WormTracker* trackerPtr : m_wormTrackersList) {
        //    if (trackerPtr && getSignedWormId(trackerPtr->getWormId(), trackerPtr->getDirection()) == conceptualWormId) {
        //        tracker = trackerPtr;
        //        break;
        //    }
        // }

        if (trackerInstance) {
            // Find all unassigned blobs that this worm is participating in
            struct BlobWithDistance {
                FrameSpecificPhysicalBlob* blob;
                double distance;
            };
            QList<BlobWithDistance> availableBlobs;

            // Get the last known position
            QPointF lastKnownPos;
            if (chosenCandidate.isValid) {
                // Use the chosen candidate as reference if available
                lastKnownPos = chosenCandidate.centroid;
            } else {
                // Try to use the tracker's last known position
                cv::Point2f trackerPos = trackerInstance->getLastKnownPosition();
                lastKnownPos = QPointF(trackerPos.x, trackerPos.y);
            }

            // Find all available blobs and calculate distances
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.participatingWormTrackerIDs.contains(conceptualWormId) && blob.selectedByWormTrackerId == 0) {
                    QPointF blobPos(blob.currentCentroid.x, blob.currentCentroid.y);
                    double dx = blobPos.x() - lastKnownPos.x();
                    double dy = blobPos.y() - lastKnownPos.y();
                    double distSq = dx*dx + dy*dy;

                    BlobWithDistance bwd;
                    bwd.blob = &blob;
                    bwd.distance = distSq;
                    availableBlobs.append(bwd);
                }
            }

            // Sort blobs by distance (closest first)
            std::sort(availableBlobs.begin(), availableBlobs.end(),
                     [](const BlobWithDistance& a, const BlobWithDistance& b) {
                         return a.distance < b.distance;
                     });

            // Use the closest blob if available
            if (!availableBlobs.isEmpty()) {
                FrameSpecificPhysicalBlob* closestBlob = availableBlobs.first().blob;

                // Claim this blob
                closestBlob->selectedByWormTrackerId = conceptualWormId;

                // Update the mapping
                m_wormToPhysicalBlobIdMap[conceptualWormId] = closestBlob->uniqueId;

                // Create the blob to assign
                blobToAssign.isValid = true;
                blobToAssign.centroid = QPointF(closestBlob->currentCentroid.x, closestBlob->currentCentroid.y);
                blobToAssign.boundingBox = closestBlob->currentBoundingBox;
                blobToAssign.area = closestBlob->currentArea;

                qDebug().noquote() << dmsg << "Found and claimed alternative blob:" << closestBlob->uniqueId
                                  << "at distance:" << std::sqrt(availableBlobs.first().distance);
                resolutionSuccess = true;
            } else {
                qDebug().noquote() << dmsg << "No available alternative blobs found.";
            }
        } else {
            // Fallback to original method if we can't find the tracker
            qDebug().noquote() << dmsg << "Couldn't find tracker, using original method.";

            // Find any unassigned blob that this worm is participating in
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.participatingWormTrackerIDs.contains(conceptualWormId) && blob.selectedByWormTrackerId == 0) {
                    // Claim this blob
                    blob.selectedByWormTrackerId = conceptualWormId;

                    // Update the mapping
                    m_wormToPhysicalBlobIdMap[conceptualWormId] = blob.uniqueId;

                    // Create the blob to assign
                    blobToAssign.isValid = true;
                    blobToAssign.centroid = QPointF(blob.currentCentroid.x, blob.currentCentroid.y);
                    blobToAssign.boundingBox = blob.currentBoundingBox;
                    blobToAssign.area = blob.currentArea;

                    qDebug().noquote() << dmsg << "Found and claimed alternative blob:" << blob.uniqueId;
                    resolutionSuccess = true;
                    break;
                }
            }
        }
    }

    // Assign blob or go lost
    if (blobToAssign.isValid) {
        qDebug().noquote() << dmsg << "Assigning blob to worm" << conceptualWormId;
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, blobToAssign));
        m_splitResolutionMap[frameNumber][conceptualWormId] = blobToAssign;
    } else {
        qDebug().noquote() << dmsg << "No valid blob available. Going lost.";
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][conceptualWormId] = Tracking::DetectedBlob();
    }

    return true;
}

// This function has been replaced by immediate resolution in processFrameSpecificSplit

// This function has been replaced by immediate resolution in processFrameSpecificSplit

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
    if (m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) { emit trackingFailed("No trackers launched."); m_isTrackingRunning = false; }
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

        // Clear any remaining split resolutions
        m_splitResolutionMap.clear();

        locker.unlock();
        if (wasCancelled) {
            m_finalTracks.clear(); for (WormObject* w : m_wormObjectsMap.values()) { if(w) m_finalTracks[w->getId()] = w->getTrackHistory(); }
            if (!m_finalTracks.empty()) emit allTracksUpdated(m_finalTracks);
            emit trackingStatusUpdate("Tracking cancelled."); emit trackingCancelled();
        } else {
            m_finalTracks.clear(); for (WormObject* w : m_wormObjectsMap.values()) { if(w) m_finalTracks[w->getId()] = w->getTrackHistory(); }
            emit allTracksUpdated(m_finalTracks); 
            QString csvPath;
            if (m_videoPath.isEmpty()) {
                csvPath = "tracks.csv";
            } else if (!m_videoSpecificDirectory.isEmpty() && QDir(m_videoSpecificDirectory).exists()) {
                csvPath = QDir(m_videoSpecificDirectory).filePath(QFileInfo(m_videoPath).completeBaseName() + "_tracks.csv");
            } else {
                // Fallback to video directory if video-specific directory is not available
                csvPath = QDir(QFileInfo(m_videoPath).absolutePath()).filePath(QFileInfo(m_videoPath).completeBaseName() + "_tracks.csv");
            }
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

QString TrackingManager::createVideoSpecificDirectory(const QString& dataDirectory, const QString& videoPath) {
    if (dataDirectory.isEmpty() || videoPath.isEmpty()) {
        qWarning() << "TrackingManager: Invalid data directory or video path";
        return QString();
    }
    
    QFileInfo videoInfo(videoPath);
    QString videoBaseName = videoInfo.completeBaseName(); // Gets filename without extension
    QString videoSpecificPath = QDir(dataDirectory).absoluteFilePath(videoBaseName);
    
    QDir videoSpecificDir(videoSpecificPath);
    if (!videoSpecificDir.exists()) {
        if (QDir().mkpath(videoSpecificPath)) {
            qDebug() << "TrackingManager: Created video-specific directory:" << videoSpecificPath;
        } else {
            qWarning() << "TrackingManager: Failed to create video-specific directory:" << videoSpecificPath;
            return QString();
        }
    } else {
        qDebug() << "TrackingManager: Using existing video-specific directory:" << videoSpecificPath;
    }
    
    return videoSpecificPath;
}

void TrackingManager::saveThresholdSettings(const QString& directoryPath, const Thresholding::ThresholdSettings& settings) {
    if (directoryPath.isEmpty()) {
        qWarning() << "TrackingManager: Cannot save threshold settings - empty directory path";
        return;
    }
    
    QJsonObject jsonObj = thresholdSettingsToJson(settings);
    QJsonDocument doc(jsonObj);
    
    QString filePath = QDir(directoryPath).absoluteFilePath("thresh_settings.json");
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        qDebug() << "TrackingManager: Saved threshold settings to:" << filePath;
    } else {
        qWarning() << "TrackingManager: Failed to save threshold settings to:" << filePath;
    }
}

void TrackingManager::saveInputBlobs(const QString& directoryPath, const std::vector<Tracking::InitialWormInfo>& worms) {
    if (directoryPath.isEmpty()) {
        qWarning() << "TrackingManager: Cannot save input blobs - empty directory path";
        return;
    }
    
    QJsonArray wormsArray;
    for (const auto& worm : worms) {
        wormsArray.append(initialWormInfoToJson(worm));
    }
    
    QJsonObject rootObj;
    rootObj["worms"] = wormsArray;
    rootObj["count"] = static_cast<int>(worms.size());
    
    QJsonDocument doc(rootObj);
    QString filePath = QDir(directoryPath).absoluteFilePath("input_blobs.json");
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        qDebug() << "TrackingManager: Saved input blobs to:" << filePath;
    } else {
        qWarning() << "TrackingManager: Failed to save input blobs to:" << filePath;
    }
}

bool TrackingManager::compareThresholdSettings(const QString& filePath, const Thresholding::ThresholdSettings& currentSettings) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "TrackingManager: Cannot read threshold settings file:" << filePath;
        return false;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "TrackingManager: JSON parse error in threshold settings:" << parseError.errorString();
        return false;
    }
    
    QJsonObject stored = doc.object();
    QJsonObject current = thresholdSettingsToJson(currentSettings);
    
    // Compare key settings and log differences
    bool match = true;
    QStringList differences;
    
    if (stored["algorithm"].toInt() != current["algorithm"].toInt()) {
        differences << QString("algorithm: %1 vs %2").arg(stored["algorithm"].toInt()).arg(current["algorithm"].toInt());
        match = false;
    }
    if (stored["globalThresholdValue"].toInt() != current["globalThresholdValue"].toInt()) {
        differences << QString("globalThresholdValue: %1 vs %2").arg(stored["globalThresholdValue"].toInt()).arg(current["globalThresholdValue"].toInt());
        match = false;
    }
    if (stored["assumeLightBackground"].toBool() != current["assumeLightBackground"].toBool()) {
        differences << QString("assumeLightBackground: %1 vs %2").arg(stored["assumeLightBackground"].toBool()).arg(current["assumeLightBackground"].toBool());
        match = false;
    }
    if (stored["adaptiveBlockSize"].toInt() != current["adaptiveBlockSize"].toInt()) {
        differences << QString("adaptiveBlockSize: %1 vs %2").arg(stored["adaptiveBlockSize"].toInt()).arg(current["adaptiveBlockSize"].toInt());
        match = false;
    }
    if (qAbs(stored["adaptiveCValue"].toDouble() - current["adaptiveCValue"].toDouble()) >= 0.001) {
        differences << QString("adaptiveCValue: %1 vs %2").arg(stored["adaptiveCValue"].toDouble()).arg(current["adaptiveCValue"].toDouble());
        match = false;
    }
    if (stored["enableBlur"].toBool() != current["enableBlur"].toBool()) {
        differences << QString("enableBlur: %1 vs %2").arg(stored["enableBlur"].toBool()).arg(current["enableBlur"].toBool());
        match = false;
    }
    if (stored["blurKernelSize"].toInt() != current["blurKernelSize"].toInt()) {
        differences << QString("blurKernelSize: %1 vs %2").arg(stored["blurKernelSize"].toInt()).arg(current["blurKernelSize"].toInt());
        match = false;
    }
    if (qAbs(stored["blurSigmaX"].toDouble() - current["blurSigmaX"].toDouble()) >= 0.001) {
        differences << QString("blurSigmaX: %1 vs %2").arg(stored["blurSigmaX"].toDouble()).arg(current["blurSigmaX"].toDouble());
        match = false;
    }
    
    if (!match) {
        qDebug() << "TrackingManager: Threshold settings differ:";
        for (const QString& diff : differences) {
            qDebug() << "  " << diff;
        }
    } else {
        qDebug() << "TrackingManager: Threshold settings match stored values";
    }
    
    return match;
}

QJsonObject TrackingManager::thresholdSettingsToJson(const Thresholding::ThresholdSettings& settings) const {
    QJsonObject obj;
    obj["algorithm"] = static_cast<int>(settings.algorithm);
    obj["globalThresholdValue"] = settings.globalThresholdValue;
    obj["assumeLightBackground"] = settings.assumeLightBackground;
    obj["adaptiveBlockSize"] = settings.adaptiveBlockSize;
    obj["adaptiveCValue"] = settings.adaptiveCValue;
    obj["enableBlur"] = settings.enableBlur;
    obj["blurKernelSize"] = settings.blurKernelSize;
    obj["blurSigmaX"] = settings.blurSigmaX;
    return obj;
}

QJsonObject TrackingManager::initialWormInfoToJson(const Tracking::InitialWormInfo& worm) const {
    QJsonObject obj;
    obj["id"] = worm.id;
    
    QJsonObject roiObj;
    roiObj["x"] = worm.initialRoi.x();
    roiObj["y"] = worm.initialRoi.y();
    roiObj["width"] = worm.initialRoi.width();
    roiObj["height"] = worm.initialRoi.height();
    obj["initialRoi"] = roiObj;
    
    return obj;
}
