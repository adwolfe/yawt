// trackingmanager.h
// ... (header remains the same as in the artifact "TrackingManager_h_cpp" with id "TrackingManager_h_cpp") ...
// No changes needed in the header file for this specific fix.
// The AllWormTracks typedef is:
// typedef std::map<int, std::vector<WormTrackPoint>> AllWormTracks;
// ...
#ifndef TRACKINGMANAGER_H
#define TRACKINGMANAGER_H

#include <QObject>
#include <QVector> // Should be QList or QMap if used for m_wormTrackers, m_trackerThreads, m_wormObjectsMap
#include <QMap>
#include <QString>
#include <QRectF>
#include <QThread>
#include <opencv2/core.hpp>

#include "trackingcommon.h" // For ThresholdSettings
#include "trackeditemdata.h" // For InitialWormInfo, AllWormTracks, WormTrackPoint
#include "wormobject.h"      // For WormObject and its state enum
#include "videoprocessor.h"  // Lowercase include
#include "wormtracker.h"     // Lowercase include

class TrackingManager : public QObject {
    Q_OBJECT

public:
    explicit TrackingManager(QObject *parent = nullptr);
    ~TrackingManager();

public slots:
    void startFullTrackingProcess(const QString& videoPath,
                                  int keyFrameNum, // 0-indexed
                                  const std::vector<InitialWormInfo>& initialWorms,
                                  const ThresholdSettings& settings,
                                  int totalFramesInVideo);
    void cancelTracking();

private slots:
    // Slots for VideoProcessor signals
    void handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                         const std::vector<cv::Mat>& reversedFrames,
                                         double fps,
                                         cv::Size frameSize);
    void handleVideoProcessingError(const QString& errorMessage);
    void handleVideoProcessingProgress(int percentage);

    // Slots for WormTracker signals
    void handleWormPositionUpdated(int wormId, int originalFrameNumber, QPointF newPosition, QRectF newRoi);
    void handleWormStateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId);
    void handleWormTrackerFinished();
    void handleWormTrackerError(int wormId, QString errorMessage);
    void handleWormTrackerProgress(int wormId, int percentDone);

signals:
    void overallTrackingProgress(int percentage);
    void trackingStatusUpdate(const QString& statusMessage);
    void individualWormTrackUpdated(int wormId, const WormTrackPoint& lastPoint);
    void allTracksUpdated(const AllWormTracks& tracks);
    void trackingFinishedSuccessfully(const QString& outputPath); // Added outputPath
    void trackingFailed(const QString& reason);
    void trackingCancelled();

private:
    void cleanupThreadsAndObjects();
    void launchWormTrackers();
    void updateOverallProgress();
    void checkForAllTrackersFinished();

    // --- Debugging and Output ---
    void outputTracksToDebug(const AllWormTracks& tracks) const;
    bool outputTracksToCsv(const AllWormTracks& tracks, const QString& outputFileName = "worm_tracks.csv") const;


    // --- Configuration & State ---
    QString m_videoPath; // Store the full path to the original video
    int m_keyFrameNum;
    std::vector<InitialWormInfo> m_initialWormInfos;
    ThresholdSettings m_thresholdSettings;
    int m_totalFramesInVideo;
    bool m_isTrackingRunning;
    bool m_cancelRequested;

    // --- Video Processing ---
    VideoProcessor* m_videoProcessor;
    QThread* m_videoProcessorThread;
    std::vector<cv::Mat> m_processedForwardFrames;
    std::vector<cv::Mat> m_processedReversedFrames;
    double m_videoFps;
    cv::Size m_videoFrameSize;

    // --- Worm Tracking ---
    QMap<int, WormObject*> m_wormObjectsMap;
    QList<WormTracker*> m_wormTrackers; // QList for ordered append and iteration
    QList<QThread*> m_trackerThreads;   // QList for ordered append and iteration
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    // --- Progress Tracking ---
    int m_videoProcessingProgress;
    QMap<WormTracker*, int> m_individualTrackerProgress; // Tracker instance -> percentage

    AllWormTracks m_finalTracks; // This is std::map
};

#endif // TRACKINGMANAGER_H

// trackingmanager.cpp
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
    qDebug() << "TrackingManager destroyed. Cleaning up...";
    if(m_isTrackingRunning) {
        cancelTracking();
    }
    cleanupThreadsAndObjects();
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
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();

    emit trackingStatusUpdate("Starting video processing...");
    emit overallTrackingProgress(0);

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
    connect(m_videoProcessor, &VideoProcessor::initialProcessingComplete, m_videoProcessorThread, &QThread::quit);
    connect(m_videoProcessor, &VideoProcessor::processingError, m_videoProcessorThread, &QThread::quit);
    m_videoProcessorThread->start();
}

void TrackingManager::cancelTracking() {
    if (!m_isTrackingRunning && !m_cancelRequested) {
        qDebug() << "TrackingManager: Cancel requested but not running or already cancelled.";
        return;
    }
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Cancellation already in progress.";
        return;
    }
    m_cancelRequested = true;
    emit trackingStatusUpdate("Cancellation requested...");
    qDebug() << "TrackingManager: Cancellation flag set.";
    if (m_videoProcessor) {
        qDebug() << "TrackingManager: Requesting video processor thread interruption.";
        if (m_videoProcessorThread && m_videoProcessorThread->isRunning()) {
            m_videoProcessorThread->requestInterruption();
        }
    }
    for (WormTracker* tracker : qAsConst(m_wormTrackers)) {
        if (tracker) {
            qDebug() << "TrackingManager: Calling stopTracking() for tracker ID" << tracker->property("wormId").toInt();
            QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection);
        }
    }
    for (QThread* thread : qAsConst(m_trackerThreads)) {
        if (thread && thread->isRunning()) {
            qDebug() << "TrackingManager: Requesting tracker thread interruption.";
            thread->requestInterruption();
        }
    }
}

void TrackingManager::cleanupThreadsAndObjects() {
    qDebug() << "TrackingManager: Cleaning up threads and objects.";
    if (m_videoProcessorThread) {
        if (m_videoProcessorThread->isRunning()) {
            m_videoProcessorThread->requestInterruption();
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
        delete m_videoProcessor;
        m_videoProcessor = nullptr;
    }
    m_processedForwardFrames.clear(); std::vector<cv::Mat>().swap(m_processedForwardFrames);
    m_processedReversedFrames.clear(); std::vector<cv::Mat>().swap(m_processedReversedFrames);

    for (QThread* thread : qAsConst(m_trackerThreads)) {
        if (thread) {
            if (thread->isRunning()) {
                thread->requestInterruption();
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
    qDeleteAll(m_wormTrackers); m_wormTrackers.clear();
    qDeleteAll(m_wormObjectsMap); m_wormObjectsMap.clear();
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_isTrackingRunning = false;
    m_cancelRequested = false;
    qDebug() << "TrackingManager: Cleanup complete.";
}

void TrackingManager::handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                                      const std::vector<cv::Mat>& reversedFrames,
                                                      double fps,
                                                      cv::Size frameSize) {
    if (m_cancelRequested) {
        qDebug() << "TrackingManager: Processing complete but cancellation was requested. Cleaning up.";
        emit trackingCancelled();
        cleanupThreadsAndObjects();
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

    if (m_videoProcessorThread) { // Ensure connections for deleteLater are made
        connect(m_videoProcessorThread, &QThread::finished, m_videoProcessor, &QObject::deleteLater, Qt::UniqueConnection);
        connect(m_videoProcessorThread, &QThread::finished, m_videoProcessorThread, &QObject::deleteLater, Qt::UniqueConnection);
        // Thread should quit on its own via the initialProcessingComplete signal connection
    }
    // m_videoProcessor will be deleted when its thread finishes due to deleteLater.
    // m_videoProcessorThread will be deleted when it finishes due to deleteLater.
    // No need to explicitly delete them here if those connections are solid.
    // Setting to nullptr to avoid dangling pointers if cleanup is called again before they are deleted.
    m_videoProcessor = nullptr;
    // m_videoProcessorThread = nullptr; // Be careful with this if cleanup might re-check it


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
    if (m_cancelRequested) { cleanupThreadsAndObjects(); return; }
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
    m_individualTrackerProgress.clear();
    emit trackingStatusUpdate(QString("Launching %1 worm trackers...").arg(m_expectedTrackersToFinish));

    for (const auto& info : m_initialWormInfos) {
        int wormId = info.id;
        QRectF initialRoi = info.initialRoi;

        WormTracker* forwardTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
        forwardTracker->setFrames(&m_processedForwardFrames);
        QThread* fwdThread = new QThread();
        forwardTracker->moveToThread(fwdThread);
        forwardTracker->setProperty("wormId", wormId);
        connect(fwdThread, &QThread::started, forwardTracker, &WormTracker::startTracking, Qt::QueuedConnection);
        connect(forwardTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
        connect(forwardTracker, &WormTracker::finished, fwdThread, &QThread::quit, Qt::QueuedConnection);
        connect(forwardTracker, &WormTracker::finished, forwardTracker, &WormTracker::deleteLater);
        connect(fwdThread, &QThread::finished, fwdThread, &QThread::deleteLater);
        connect(forwardTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(forwardTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(forwardTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(forwardTracker, &WormTracker::progress, this, [this, forwardTracker](int, int p){
            if (m_cancelRequested) return;
            m_individualTrackerProgress[forwardTracker] = p;
            updateOverallProgress();
        });
        m_wormTrackers.append(forwardTracker); m_trackerThreads.append(fwdThread);
        m_individualTrackerProgress[forwardTracker] = 0; fwdThread->start();

        WormTracker* backwardTracker = new WormTracker(wormId, initialRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
        backwardTracker->setFrames(&m_processedReversedFrames);
        QThread* bwdThread = new QThread();
        backwardTracker->moveToThread(bwdThread);
        backwardTracker->setProperty("wormId", wormId);
        connect(bwdThread, &QThread::started, backwardTracker, &WormTracker::startTracking, Qt::QueuedConnection);
        connect(backwardTracker, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished, Qt::QueuedConnection);
        connect(backwardTracker, &WormTracker::finished, bwdThread, &QThread::quit, Qt::QueuedConnection);
        connect(backwardTracker, &WormTracker::finished, backwardTracker, &WormTracker::deleteLater);
        connect(bwdThread, &QThread::finished, bwdThread, &QThread::deleteLater);
        connect(backwardTracker, &WormTracker::positionUpdated, this, &TrackingManager::handleWormPositionUpdated);
        connect(backwardTracker, &WormTracker::stateChanged, this, &TrackingManager::handleWormStateChanged);
        connect(backwardTracker, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
        connect(backwardTracker, &WormTracker::progress, this, [this, backwardTracker](int, int p){
            if (m_cancelRequested) return;
            m_individualTrackerProgress[backwardTracker] = p;
            updateOverallProgress();
        });
        m_wormTrackers.append(backwardTracker); m_trackerThreads.append(bwdThread);
        m_individualTrackerProgress[backwardTracker] = 0; bwdThread->start();
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
    }
    m_finishedTrackersCount++;
    qDebug() << "TrackingManager: A worm tracker finished. Total finished:" << m_finishedTrackersCount << "/" << m_expectedTrackersToFinish;
    updateOverallProgress();
    checkForAllTrackersFinished();
}

void TrackingManager::checkForAllTrackersFinished() {
    if (m_cancelRequested) {
        bool allEffectivelyStopped = true;
        // Iterate over a copy if threads might be removed during iteration, or use index-based loop.
        // For now, assuming m_trackerThreads is stable during this check or qAsConst is sufficient.
        for(QThread* thread : qAsConst(m_trackerThreads)){
            if(thread && thread->isRunning() && !thread->isFinished()){ // isFinished() is usually more reliable after quit()
                allEffectivelyStopped = false;
                break;
            }
        }
        // Also check if all trackers have reported finished if that's a separate mechanism
        if(allEffectivelyStopped || m_finishedTrackersCount >= m_expectedTrackersToFinish) {
            qDebug() << "TrackingManager: All trackers appear stopped after cancellation request.";
            emit trackingCancelled();
            cleanupThreadsAndObjects(); // Perform full cleanup
        }
        return; // Don't proceed to success if cancelled
    }

    if (m_finishedTrackersCount >= m_expectedTrackersToFinish && m_isTrackingRunning) {
        emit trackingStatusUpdate("All worm trackers completed. Consolidating and saving tracks...");
        m_finalTracks.clear();
        // Corrected iteration for QMap values:
        for(WormObject* worm : m_wormObjectsMap.values()){ // Iterate directly over the QList<WormObject*>
            if(worm) {
                m_finalTracks[worm->getId()] = worm->getTrackHistory(); // This is the stitched track
            }
        }
        emit allTracksUpdated(m_finalTracks);

        outputTracksToDebug(m_finalTracks);
        QString csvOutputPath;
        if (!m_videoPath.isEmpty()) {
            QFileInfo videoInfo(m_videoPath);
            QString tmpDirName = "tmp_tracking_output"; // Desired subdirectory name
            QDir videoDir = videoInfo.dir(); // Directory of the video file
            if (videoDir.mkpath(tmpDirName)) { // Create subdir if it doesn't exist
                csvOutputPath = videoDir.filePath(tmpDirName + "/" + videoInfo.completeBaseName() + "_tracks.csv");
            } else {
                // Fallback if subdir creation fails
                csvOutputPath = videoDir.filePath(videoInfo.completeBaseName() + "_tracks.csv");
                qWarning() << "Could not create tmp directory:" << videoDir.filePath(tmpDirName) << "Saving CSV to video directory.";
            }
        } else {
            csvOutputPath = "worm_tracks.csv"; // Default if no video path available
            qWarning() << "Video path is empty, saving tracks to default file:" << csvOutputPath;
        }

        bool csvSaved = outputTracksToCsv(m_finalTracks, csvOutputPath);
        if (csvSaved) {
            emit trackingStatusUpdate("Tracks saved to: " + csvOutputPath);
            emit trackingFinishedSuccessfully(csvOutputPath);
        } else {
            emit trackingStatusUpdate("Tracking finished, but failed to save CSV.");
            emit trackingFinishedSuccessfully(""); // Indicate success but no file path
        }

        m_isTrackingRunning = false;
        qDebug() << "TrackingManager: All tracking tasks complete. Tracks consolidated.";
        // Objects and threads are set to deleteLater, major cleanup not needed here unless forced.
        // cleanupThreadsAndObjects(); // Call this if you want immediate aggressive cleanup
    }
}

void TrackingManager::handleWormTrackerError(int wormId, QString errorMessage) {
    if (m_cancelRequested) return;
    qWarning() << "TrackingManager: Error from tracker for worm ID" << wormId << ":" << errorMessage;
    WormTracker* errorTracker = qobject_cast<WormTracker*>(sender());
    if(errorTracker) {
        m_individualTrackerProgress[errorTracker] = 100; // Mark as done for progress
    }
    m_finishedTrackersCount++;
    updateOverallProgress();
    emit trackingStatusUpdate(QString("Error with tracker for worm %1.").arg(wormId));
    checkForAllTrackersFinished(); // Check if this error means all are effectively done
}

void TrackingManager::updateOverallProgress() {
    if (m_cancelRequested || !m_isTrackingRunning) {
        // If cancelled or not running, progress might be misleading or reset
        return;
    }

    double totalProgress = 0;
    double videoProcWeight = 0.20; // Video processing is 20%
    double trackersWeight = 0.80;  // All trackers combined are 80%

    totalProgress += static_cast<double>(m_videoProcessingProgress) * videoProcWeight / 100.0;

    if (m_expectedTrackersToFinish > 0 && !m_individualTrackerProgress.empty()) {
        double currentTrackersAggregatedProgressPts = 0;
        // Ensure m_individualTrackerProgress.size() is not zero to prevent division by zero
        if (m_individualTrackerProgress.size() > 0) {
            // Corrected iteration:
            for (int progress : m_individualTrackerProgress.values()) { // Iterate directly over the QList<int> returned by values()
                currentTrackersAggregatedProgressPts += static_cast<double>(progress);
            }
            double avgTrackerProgress = currentTrackersAggregatedProgressPts / static_cast<double>(m_individualTrackerProgress.size());
            totalProgress += avgTrackerProgress * trackersWeight / 100.0;
        }
    } else if (m_videoProcessingProgress == 100 && m_expectedTrackersToFinish == 0) { // No trackers, video processing is everything
        totalProgress = 1.0; // 100%
    }
    emit overallTrackingProgress(static_cast<int>(totalProgress * 100.0));
}

// --- Debugging and Output Methods ---
void TrackingManager::outputTracksToDebug(const AllWormTracks& tracks) const {
    qDebug() << "--- Begin Track Output ---";
    if (tracks.empty()) { // Use .empty() for std::map
        qDebug() << "No tracks to output.";
    }
    // Use range-based for loop for std::map (C++11 and later)
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
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "Failed to open CSV file for writing:" << outputFilePath << csvFile.errorString();
        return false;
    }
    QTextStream outStream(&csvFile);
    outStream << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight\n";

    // Use range-based for loop for std::map (C++11 and later)
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

