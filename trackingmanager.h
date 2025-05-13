// trackingmanager.h
#ifndef TRACKINGMANAGER_H
#define TRACKINGMANAGER_H

#include <QObject>
#include <QVector>
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
    //void handleWormTrackerProgress(int wormId, int percentDone); // Note: wormId here is from tracker, not used for m_wormTrackerProgress key

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
    std::vector<cv::Mat> m_processedForwardFrames; // Store by value
    std::vector<cv::Mat> m_processedReversedFrames; // Store by value
    double m_videoFps;
    cv::Size m_videoFrameSize;

    // --- Worm Tracking ---
    QMap<int, WormObject*> m_wormObjectsMap;
    QList<WormTracker*> m_wormTrackers;
    QList<QThread*> m_trackerThreads;
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    // --- Progress Tracking ---
    int m_videoProcessingProgress;
    QMap<WormTracker*, int> m_individualTrackerProgress; // Tracker instance -> percentage

    AllWormTracks m_finalTracks;
};

#endif // TRACKINGMANAGER_H
