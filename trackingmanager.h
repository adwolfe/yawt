// TrackingManager.h
#ifndef TRACKINGMANAGER_H
#define TRACKINGMANAGER_H

#include <QObject>
#include <QVector>
#include <QMap>
#include <QString>
#include <QRectF>
#include <QThread>
#include <opencv2/core.hpp>

#include "trackdata.h"    // For ThresholdSettings, InitialWormInfo, AllWormTracks
#include "wormobject.h"   // For WormObject and its state enum
#include "videoprocessor.h"
#include "wormtracker.h"

class TrackingManager : public QObject {
    Q_OBJECT

public:
    explicit TrackingManager(QObject *parent = nullptr);
    ~TrackingManager();

    // Call this to start the whole process
public slots:
    void startFullTrackingProcess(const QString& videoPath,
                                  int keyFrameNum, // 0-indexed
                                  const std::vector<InitialWormInfo>& initialWorms,
                                  const ThresholdSettings& settings,
                                  int totalFramesInVideo); // Hint for VideoProcessor progress

    void cancelTracking(); // To stop all processing and tracking

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
    //void handleWormTrackerProgress(int wormId, int percentDone);


signals:
    void overallTrackingProgress(int percentage); // Aggregated progress
    void trackingStatusUpdate(const QString& statusMessage);
    void individualWormTrackUpdated(int wormId, const WormTrackPoint& lastPoint); // For live UI updates
    void allTracksUpdated(const AllWormTracks& tracks); // Periodically or on significant change
    void trackingFinishedSuccessfully();
    void trackingFailed(const QString& reason);
    void trackingCancelled();


private:
    void cleanupThreadsAndObjects();
    void launchWormTrackers();
    void updateOverallProgress();
    void checkForAllTrackersFinished();

    // --- Configuration & State ---
    QString m_videoPath;
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
    QMap<int, WormObject*> m_wormObjectsMap; // Maps worm ID to WormObject
    // Each WormObject will have two trackers (forward and backward)
    // We can map worm ID to a pair of trackers, or have a flat list and identify by wormId in tracker.
    QList<WormTracker*> m_wormTrackers;
    QList<QThread*> m_trackerThreads;
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    // --- Progress Tracking ---
    int m_videoProcessingProgress; // 0-100
    QMap<int, int> m_wormTrackerProgress; // wormId -> percentage for its combined trackers
        // or just average progress of all trackers.
        // For simplicity, let's track individual tracker progress.
    QMap<WormTracker*, int> m_individualTrackerProgress; // Tracker instance -> percentage

    AllWormTracks m_finalTracks; // Consolidated tracks
};

#endif // TRACKINGMANAGER_H
