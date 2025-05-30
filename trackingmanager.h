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

#include "trackingcommon.h"
#include "wormobject.h"
#include "videoprocessor.h"
#include "wormtracker.h"

// Constants for merge/split logic (consider moving to a config or tuning)
const double MERGE_GROUP_IOU_THRESHOLD = 0.3;
const double MERGE_GROUP_CENTROID_MAX_DIST_SQ = 225.0;
const int MAX_PAUSED_DURATION_MS = 5000;
const int PAUSE_RESOLUTION_INTERVAL_MS = 500;

struct TrackedMergeGroup {
    int uniqueMergeId;
    QRectF currentBoundingBox;
    cv::Point2f currentCentroid;
    double currentArea;

    QSet<int> participatingWormIds; // Conceptual Worm IDs
    QMap<int, cv::Point2f> individualEstimatedCentroids;

    int firstFrameSeen;
    int lastFrameActive;
    QDateTime lastUpdateTime;
    bool isValid;

    TrackedMergeGroup() : uniqueMergeId(-1), currentArea(0.0), firstFrameSeen(-1), lastFrameActive(-1), isValid(false) {}
};

struct PausedWormInfo {
    int conceptualWormId;
    QPointer<WormTracker> trackerInstance; // Pointer to the specific WormTracker instance
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
                                  int totalFramesInVideo);
    void cancelTracking();

private slots:
    void handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                         const std::vector<cv::Mat>& reversedFrames,
                                         double fps,
                                         cv::Size frameSize);
    void handleVideoProcessingError(const QString& errorMessage);
    void handleVideoProcessingProgress(int percentage);

    void handleFrameUpdate(int reportingWormId, // Conceptual ID
                           int originalFrameNumber,
                           const Tracking::DetectedBlob& primaryBlob,
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
    TrackedMergeGroup* findMatchingMergeGroup(int frameNumber, const QRectF& blobBox, const cv::Point2f& blobCentroid, double blobArea);
    int createNewMergeGroup(int frameNumber, int initialWormId, const Tracking::DetectedBlob& mergedBlobData);
    void attemptAutomaticSplitResolution(int pausedConceptualWormId, PausedWormInfo& pausedInfo);
    void forceResolvePausedWorm(int conceptualWormId, PausedWormInfo& pausedInfo);
    void cleanupStaleMergeGroups(int currentFrameNumber);

    double calculateIoU(const QRectF& r1, const QRectF& r2) const;

    QString m_videoPath;
    int m_keyFrameNum;
    std::vector<Tracking::InitialWormInfo> m_initialWormInfos;
    Thresholding::ThresholdSettings m_thresholdSettings;
    int m_totalFramesInVideo;
    bool m_isTrackingRunning;
    bool m_cancelRequested;

    VideoProcessor* m_videoProcessor;
    QThread* m_videoProcessorThread;
    std::vector<cv::Mat> m_processedForwardFrames;
    std::vector<cv::Mat> m_processedReversedFrames;
    double m_videoFps;
    cv::Size m_videoFrameSize;

    QMap<int, WormObject*> m_wormObjectsMap;
    QList<WormTracker*> m_wormTrackersList;
    // Maps to get specific FWD/BWD tracker instances for a conceptual worm ID
    QMap<int, WormTracker*> m_wormIdToForwardTrackerInstanceMap;
    QMap<int, WormTracker*> m_wormIdToBackwardTrackerInstanceMap;

    QList<QThread*> m_trackerThreads;
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    int m_videoProcessingProgress;
    QMap<WormTracker*, int> m_individualTrackerProgress;

    Tracking::AllWormTracks m_finalTracks;

    QMutex m_dataMutex;
    int m_nextUniqueMergeId;

    QMap<int, TrackedMergeGroup> m_activeMergeGroups;
    QMap<int, QSet<int>> m_frameToActiveMergeGroupIds;
    QMap<int, int> m_wormToCurrentMergeGroupId; // conceptualWormId -> uniqueMergeId

    // Key is conceptualWormId. PausedWormInfo stores the specific QPointer<WormTracker> instance.
    QMap<int, PausedWormInfo> m_pausedWorms;

    QTimer* m_pauseResolutionTimer;
};

#endif // TRACKINGMANAGER_H
