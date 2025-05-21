#ifndef TRACKINGMANAGER_H
#define TRACKINGMANAGER_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>
#include <QRectF>
#include <QThread>
#include <opencv2/core.hpp>
#include <QSet> // Changed from <set> for QSet<WormTracker*>

#include "trackingcommon.h" // For ThresholdSettings
#include "wormobject.h"      // For WormObject
#include "videoprocessor.h"
#include "wormtracker.h"     // For WormTracker class and its signals

// Forward declare WormFrameInfo if its definition remains in .cpp
struct WormFrameInfo;

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
    // VideoProcessor signals
    void handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                         const std::vector<cv::Mat>& reversedFrames,
                                         double fps,
                                         cv::Size frameSize);
    void handleVideoProcessingError(const QString& errorMessage);
    void handleVideoProcessingProgress(int percentage);

    // WormTracker signals
    void handleWormPositionUpdated(WormTracker* reportingTrackerInstance, // Changed: Pass tracker instance
                                   int conceptualWormId,
                                   int originalFrameNumber,
                                   QPointF newPosition,
                                   QRectF newRoi,
                                   int plausibleBlobsFoundInRoi,
                                   double primaryBlobArea);

    void handleWormSplitDetectedAndPaused(WormTracker* reportingTrackerInstance, // Changed: Pass tracker instance
                                          int conceptualWormId,
                                          int originalFrameNumber,
                                          const QList<TrackingHelper::DetectedBlob>& detectedBlobs);

    // Slot to handle state changes, sender() can be used to get WormTracker*
    void handleWormStateChanged(int conceptualWormId, WormTracker::TrackerState newState);
    void handleWormTrackerFinished(); // Sender is the WormTracker*
    void handleWormTrackerError(int conceptualWormId, QString errorMessage); // Sender is the WormTracker*
    void handleWormTrackerProgress(int conceptualWormId, int percentDone); // Sender is the WormTracker*


signals:
    void overallTrackingProgress(int percentage);
    void trackingStatusUpdate(const QString& statusMessage);
    void individualWormTrackUpdated(int wormId, const WormTrackPoint& lastPoint); // wormId is conceptualWormId
    void allTracksUpdated(const AllWormTracks& tracks); // Key is conceptualWormId
    void trackingFinishedSuccessfully(const QString& outputPath);
    void trackingFailed(const QString& reason);
    void trackingCancelled();


private:
    void cleanupThreadsAndObjects();
    void launchWormTrackers();
    void updateOverallProgress();
    void checkForAllTrackersFinished();
    void outputTracksToDebug(const AllWormTracks& tracks) const;
    bool outputTracksToCsv(const AllWormTracks& tracks, const QString& outputFileName) const;
    QList<WormTracker*> findTrackersForWorm(int conceptualWormId);


    // --- Merge/Split Management ---
    // Key: frame number, Value: Map of specific WormTracker instance to its reported info in that frame
    QMap<int, QMap<WormTracker*, WormFrameInfo>> m_frameInfos;
    int m_frameInfoHistorySize = 3;

    QMap<WormTracker*, QSet<WormTracker*>> m_mergedTrackerGroups;
    QMap<WormTracker*, WormTracker*> m_trackerToMergeGroupMap;

    void processFrameDataForMergesAndSplits(int frameNumber);


    // --- Configuration & State ---
    QString m_videoPath;
    int m_keyFrameNum;
    std::vector<InitialWormInfo> m_initialWormInfos;
    ThresholdSettings m_thresholdSettings;
    int m_totalFramesInVideo;
    bool m_isTrackingRunning;
    bool m_cancelRequested;

    VideoProcessor* m_videoProcessor;
    QThread* m_videoProcessorThread;
    std::vector<cv::Mat> m_processedForwardFrames;
    std::vector<cv::Mat> m_processedReversedFrames;
    double m_videoFps;
    cv::Size m_videoFrameSize;

    QMap<int, WormObject*> m_wormObjectsMap; // Key is conceptualWormId
    QList<WormTracker*> m_wormTrackers;      // List of all *active* tracker instances
    QList<QThread*> m_trackerThreads;
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    int m_videoProcessingProgress;
    QMap<WormTracker*, int> m_individualTrackerProgress; // Progress per tracker instance

    AllWormTracks m_finalTracks; // Key is conceptualWormId
};

#endif // TRACKINGMANAGER_H
