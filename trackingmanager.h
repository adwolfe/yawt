#ifndef TRACKINGMANAGER_H
#define TRACKINGMANAGER_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>
#include <QRectF>
#include <QThread>
#include <opencv2/core.hpp>
#include <set> // For std::set in merge logic

#include "trackingcommon.h" // For Thresholding::ThresholdSettings (ensure lowercase if filename is)
#include "wormobject.h"      // For WormObject (ensure lowercase)
#include "videoprocessor.h"  // Lowercase include
#include "wormtracker.h"     // Lowercase include, for WormTracker class and its signals

class TrackingManager : public QObject {
    Q_OBJECT

public:
    explicit TrackingManager(QObject *parent = nullptr);
    ~TrackingManager();

public slots:
    void startFullTrackingProcess(const QString& videoPath,
                                  int keyFrameNum, // 0-indexed
                                  const std::vector<Tracking::InitialWormInfo>& initialWorms,
                                  const Thresholding::ThresholdSettings& settings,
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

    // WormTracker signals (Updated and New)
    void handleWormPositionUpdated(int wormId,
                                   int originalFrameNumber,
                                   QPointF newPosition,
                                   QRectF newRoi,
                                   int plausibleBlobsFoundInRoi, // New param
                                   double primaryBlobArea);      // New param

    void handleWormSplitDetectedAndPaused(int wormId,
                                          int originalFrameNumber,
                                          const QList<Tracking::DetectedBlob>& detectedBlobs); // New slot

    void handleWormStateChanged(int wormId, WormTracker::TrackerState newState);
    void handleWormTrackerFinished();
    void handleWormTrackerError(int wormId, QString errorMessage);
    void handleWormTrackerProgress(int wormId, int percentDone);


signals:
    void overallTrackingProgress(int percentage);
    void trackingStatusUpdate(const QString& statusMessage);
    void individualWormTrackUpdated(int wormId, const Tracking::WormTrackPoint& lastPoint);
    void allTracksUpdated(const Tracking::AllWormTracks& tracks);
    void trackingFinishedSuccessfully(const QString& outputPath);
    void trackingFailed(const QString& reason);
    void trackingCancelled();


private:
    void cleanupThreadsAndObjects();
    void launchWormTrackers();
    void updateOverallProgress();
    void checkForAllTrackersFinished();
    void outputTracksToDebug(const Tracking::AllWormTracks& tracks) const;
    bool outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFileName) const;
    QList<WormTracker*> findTrackersForWorm(int conceptualWormId);

    // --- Merge/Split Management ---
    struct WormFrameInfo { // Renamed from WormFrameState for clarity
        QPointF position;
        QRectF roi;
        int plausibleBlobsInRoi; // Number of blobs tracker saw in its ROI
        double primaryBlobArea;   // Area of the blob it decided to follow
        int reportingTracker = 0;
        bool isValid = true; // Was this info successfully reported?
    };
    // Key: frame number, Value: Map of wormID to its reported info in that frame
    QMap<int, QMap<int, WormFrameInfo>> m_frameInfos;
    int m_frameInfoHistorySize = 3; // How many recent frames of info to keep for context

    // Represents a group of merged worms. Key: a representative worm ID from the group (e.g., lowest ID).
    // Value: the set of all worm IDs currently considered part of that merge.
    QMap<int, QSet<int>> m_mergedGroups;
    // Tracks which merged group a worm currently belongs to. Key: wormId, Value: representative ID of its merge group.
    QMap<int, int> m_wormToMergeGroupMap;

    void processFrameDataForMergesAndSplits(int frameNumber);


    // --- Configuration & State (as before) ---
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
    QList<WormTracker*> m_wormTrackers;
    QList<QThread*> m_trackerThreads;
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    int m_videoProcessingProgress;
    QMap<WormTracker*, int> m_individualTrackerProgress;

    Tracking::AllWormTracks m_finalTracks; // std::map
};

#endif // TRACKINGMANAGER_H
