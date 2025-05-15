// TrackingManager.h
#ifndef TRACKINGMANAGER_H
#define TRACKINGMANAGER_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>
#include <QRectF>
#include <QThread>
#include <opencv2/core.hpp>

#include "trackingcommon.h" // For ThresholdSettings
#include "wormobject.h"      // For WormObject
#include "videoprocessor.h"
#include "wormtracker.h"

class TrackingManager : public QObject {
    Q_OBJECT

public:
    explicit TrackingManager(QObject *parent = nullptr);
    ~TrackingManager();

public slots:
    void startFullTrackingProcess(const QString& videoPath,
                                  int keyFrameNum,
                                  const std::vector<InitialWormInfo>& initialWorms,
                                  const ThresholdSettings& settings,
                                  int totalFramesInVideo);
    void cancelTracking();

private slots:
    void handleInitialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                         const std::vector<cv::Mat>& reversedFrames,
                                         double fps,
                                         cv::Size frameSize);
    void handleVideoProcessingError(const QString& errorMessage);
    void handleVideoProcessingProgress(int percentage);
    void handleWormPositionUpdated(int wormId, int originalFrameNumber, QPointF newPosition, QRectF newRoi);
    void handleWormStateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId);
    void handleWormTrackerFinished();
    void handleWormTrackerError(int wormId, QString errorMessage);
    //void handleWormTrackerProgress(int wormId, int percentDone);

signals:
    void overallTrackingProgress(int percentage);
    void trackingStatusUpdate(const QString& statusMessage);
    void individualWormTrackUpdated(int wormId, const WormTrackPoint& lastPoint);
    void allTracksUpdated(const AllWormTracks& tracks);
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

    QString m_videoPath;
    int m_keyFrameNum;
    std::vector<InitialWormInfo> m_initialWormInfos;
    ThresholdSettings m_thresholdSettings;
    int m_totalFramesInVideo;
    bool m_isTrackingRunning;
    bool m_cancelRequested;

    VideoProcessor* m_videoProcessor;       // Worker object
    QThread* m_videoProcessorThread;        // Thread for VideoProcessor
    std::vector<cv::Mat> m_processedForwardFrames;
    std::vector<cv::Mat> m_processedReversedFrames;
    double m_videoFps;
    cv::Size m_videoFrameSize;

    QMap<int, WormObject*> m_wormObjectsMap;
    QList<WormTracker*> m_wormTrackers;     // Pointers to worker objects
    QList<QThread*> m_trackerThreads;       // Pointers to QThreads for WormTrackers
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    int m_videoProcessingProgress;
    QMap<WormTracker*, int> m_individualTrackerProgress;

    AllWormTracks m_finalTracks;
};

#endif // TRACKINGMANAGER_H
