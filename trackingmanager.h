// trackingmanager.h
#ifndef TRACKINGMANAGER_H
#define TRACKINGMANAGER_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>
#include <QRectF>
#include <QThread>
#include <QSet>
#include <QMutex>
#include <QTimer>
#include <QDateTime>
#include <QPointer> // For QPointer

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp> // For VideoCapture to get FPS and frame size

#include "trackingcommon.h"
#include "wormobject.h"
#include "videoprocessor.h" // Will be used for its type, not direct instantiation here
#include "wormtracker.h"

// Constants for merge/split logic (consider moving to a config or tuning)
const double MERGE_GROUP_IOU_THRESHOLD = 0.3;
const double MERGE_GROUP_CENTROID_MAX_DIST_SQ = 225.0; // 15*15 pixels
const int MAX_PAUSED_DURATION_MS = 5000;
const int PAUSE_RESOLUTION_INTERVAL_MS = 500;

struct TrackedMergeGroup {
    int uniqueMergeId;
    QRectF currentBoundingBox;
    cv::Point2f currentCentroid;
    double currentArea;
    QSet<int> participatingWormIds;
    QMap<int, cv::Point2f> individualEstimatedCentroids;
    int firstFrameSeen;
    int lastFrameActive;
    QDateTime lastUpdateTime;
    bool isValid;
    TrackedMergeGroup() : uniqueMergeId(-1), currentArea(0.0), firstFrameSeen(-1), lastFrameActive(-1), isValid(false) {}
};

struct PausedWormInfo {
    int conceptualWormId;
    QPointer<WormTracker> trackerInstance;
    int framePausedOn;
    QDateTime timePaused;
    Tracking::DetectedBlob candidateSplitBlob;
    int presumedMergeGroupId_F_minus_1;
    PausedWormInfo() : conceptualWormId(-1), trackerInstance(nullptr), framePausedOn(-1), presumedMergeGroupId_F_minus_1(-1) {}
};


class TrackingManager : public QObject {
    Q_OBJECT

public:
    explicit TrackingManager(QObject *parent = nullptr);
    ~TrackingManager();

public slots:
    void startFullTrackingProcess(const QString& videoPath,
                                  int keyFrameNum,
                                  const std::vector<Tracking::InitialWormInfo>& initialWorms,
                                  const Thresholding::ThresholdSettings& settings,
                                  int totalFramesInVideo); // totalFramesInVideo is a hint
    void cancelTracking();

private slots:
    // This is now called internally by assembleProcessedFrames
    void handleInitialProcessingComplete(const std::vector<cv::Mat>& finalForwardFrames,
                                         const std::vector<cv::Mat>& finalReversedFrames,
                                         double fps, // FPS and FrameSize are determined once by TrackingManager
                                         cv::Size frameSize);

    // Renamed to reflect it handles errors from any chunk processor
    void handleVideoChunkProcessingError(int chunkId, const QString& errorMessage);

    // New slots for parallel video processing
    void handleRangeProcessingProgress(int chunkId, int percentage);
    void handleRangeProcessingComplete(int chunkId,
                                       const std::vector<cv::Mat>& processedFrames,
                                       bool wasForwardChunk);


    void handleFrameUpdate(int reportingWormId,
                           int originalFrameNumber,
                           const Tracking::DetectedBlob& primaryBlob,
                           const Tracking::DetectedBlob& fullBlob,
                           QRectF searchRoiUsed,
                           Tracking::TrackerState currentState);

    void handleWormTrackerFinished();
    void handleWormTrackerError(int reportingWormId, QString errorMessage);
    void handleWormTrackerProgress(int reportingWormId, int percentDone);

    void checkPausedWormsAndResolveSplits();


signals:
    void overallTrackingProgress(int percentage);
    void trackingStatusUpdate(const QString& statusMessage);
    void allTracksUpdated(const Tracking::AllWormTracks& tracks);
    void trackingFinishedSuccessfully(const QString& outputPath);
    void trackingFailed(const QString& reason);
    void trackingCancelled();


private:
    void cleanupThreadsAndObjects();
    void launchWormTrackers();
    void updateOverallProgress();
    void checkForAllTrackersFinished();
    bool outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFileName) const;

    void processMergedState(int reportingWormId, int frameNumber, const Tracking::DetectedBlob& mergedBlobData, WormTracker* reportingTrackerInstance);
    void processPausedForSplitState(int reportingWormId, int frameNumber, const Tracking::DetectedBlob& candidateSplitBlob, WormTracker* reportingTrackerInstance);
    TrackedMergeGroup* findMatchingMergeGroup(int frameNumber, const QRectF& blobBox, const cv::Point2f& blobCentroid, double blobArea, WormTracker::TrackingDirection direction);int createNewMergeGroup(int frameNumber, int initialWormId, const Tracking::DetectedBlob& mergedBlobData);
    void attemptAutomaticSplitResolution(int pausedConceptualWormId, PausedWormInfo& pausedInfo);
    void forceResolvePausedWorm(int conceptualWormId, PausedWormInfo& pausedInfo);
    void cleanupStaleMergeGroups(int currentFrameNumber);
    double calculateIoU(const QRectF& r1, const QRectF& r2) const;

    // New private method for parallel processing
    void assembleProcessedFrames();


    QString m_videoPath;
    int m_keyFrameNum;
    std::vector<Tracking::InitialWormInfo> m_initialWormInfos;
    Thresholding::ThresholdSettings m_thresholdSettings;
    int m_totalFramesInVideoHint; // Store the hint
    bool m_isTrackingRunning;
    bool m_cancelRequested;

    // --- DEPRECATED ---
    // VideoProcessor* m_videoProcessor;
    // QThread* m_videoProcessorThread;

    // --- NEW MEMBERS for Parallel Video Processing ---
    QList<QPointer<QThread>> m_videoProcessorThreads; // Use QPointer for safety
    int m_videoProcessorsFinishedCount;
    int m_totalVideoChunksToProcess;

    // Maps to store processed chunks before assembly
    // Key: chunkId (startFrame of the chunk), Value: processed frames
    QMap<int, std::vector<cv::Mat>> m_assembledForwardFrameChunks;
    QMap<int, std::vector<cv::Mat>> m_assembledBackwardFrameChunks;
    QMap<int, int> m_videoChunkProgressMap;


    // These are populated once all chunks are processed and assembled
    std::vector<cv::Mat> m_finalProcessedForwardFrames;
    std::vector<cv::Mat> m_finalProcessedReversedFrames; // after assembly and global reverse
    double m_videoFps;     // Determined once at the start
    cv::Size m_videoFrameSize; // Determined once at the start

    QMap<int, WormObject*> m_wormObjectsMap;
    QList<WormTracker*> m_wormTrackersList;
    QMap<int, WormTracker*> m_wormIdToForwardTrackerInstanceMap;
    QMap<int, WormTracker*> m_wormIdToBackwardTrackerInstanceMap;

    QList<QPointer<QThread>> m_trackerThreads; // Use QPointer
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    int m_videoProcessingOverallProgress; // Aggregated from m_videoChunkProgressMap
    QMap<WormTracker*, int> m_individualTrackerProgress;

    Tracking::AllWormTracks m_finalTracks;

    QMutex m_dataMutex;
    int m_nextUniqueMergeId;
    QMap<int, TrackedMergeGroup> m_activeMergeGroups;
    QMap<int, QSet<int>> m_frameToActiveMergeGroupIds;
    QMap<int, int> m_wormToCurrentMergeGroupId;
    QMap<int, PausedWormInfo> m_pausedWorms;
    QTimer* m_pauseResolutionTimer;
};

#endif // TRACKINGMANAGER_H
