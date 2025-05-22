// trackingmanager.h
#ifndef TRACKINGMANAGER_H
#define TRACKINGMANAGER_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>
#include <QRectF>
#include <QThread>
#include <QSet> // QSet is used instead of std::set for Qt consistency

#include <opencv2/core.hpp>

#include "trackingcommon.h" // Defines Tracking::DetectedBlob, InitialWormInfo, AllWormTracks
#include "wormobject.h"
#include "videoprocessor.h"
#include "wormtracker.h"    // Defines WormTracker::TrackerState, WormTracker::TrackingDirection

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

    // WormTracker signals
    /**
     * @brief Handles position updates from WormTrackers.
     * @param reportingWormId The conceptual ID of the worm.
     * @param originalFrameNumber The frame number in the original video.
     * @param primaryBlob The main blob identified by the tracker.
     * @param searchRoiUsed The search ROI used by the tracker in this frame.
     * @param plausibleBlobsFoundInSearchRoi Count of plausible blobs in the search ROI.
     */
    void handleWormPositionUpdated(int reportingWormId,
                                   int originalFrameNumber,
                                   const Tracking::DetectedBlob& primaryBlob,
                                   QRectF searchRoiUsed,
                                   int plausibleBlobsFoundInSearchRoi);

    void handleWormSplitDetectedAndPaused(int reportingWormId,
                                          int originalFrameNumber,
                                          const QList<Tracking::DetectedBlob>& detectedBlobs);

    void handleWormStateChanged(int reportingWormId, WormTracker::TrackerState newState, int associatedEntityId);
    void handleWormTrackerFinished();
    void handleWormTrackerError(int reportingWormId, QString errorMessage);
    void handleWormTrackerProgress(int reportingWormId, int percentDone);


signals:
    void overallTrackingProgress(int percentage);
    void trackingStatusUpdate(const QString& statusMessage);
    void individualWormTrackUpdated(int wormId, const Tracking::WormTrackPoint& lastPoint); // For UI updates
    void allTracksUpdated(const Tracking::AllWormTracks& tracks); // For final full data update
    void trackingFinishedSuccessfully(const QString& outputPath);
    void trackingFailed(const QString& reason);
    void trackingCancelled();


private:
    void cleanupThreadsAndObjects();
    void launchWormTrackers();
    void updateOverallProgress();
    void checkForAllTrackersFinished();
    bool outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFileName) const;
    QList<WormTracker*> findTrackersForWorm(int conceptualWormId); // Finds live tracker instances

    // --- Per-Frame Information from Trackers ---
    struct WormFrameInfo {
        Tracking::DetectedBlob primaryBlob; // Contains centroid, bbox, area, contour of the main blob
        QRectF searchRoiUsed;               // The search ROI the tracker used for this frame's detection
        int plausibleBlobsInSearchRoi;      // Number of blobs the tracker saw in its searchRoiUsed
        int reportingTrackerWormId;         // Conceptual ID of the worm this info pertains to
        // WormTracker::TrackingDirection reportingTrackerDirection; // Implicit from originalFrameNumber vs m_keyFrameNum
        // bool isValid;                    // Can be inferred from primaryBlob.isValid

        WormFrameInfo() : plausibleBlobsInSearchRoi(0), reportingTrackerWormId(-1) {
            primaryBlob.isValid = false; // Ensure blob starts invalid
        }
    };
    // Key: originalFrameNumber, Value: Map of (conceptual wormID to its WormFrameInfo for that frame)
    QMap<int, QMap<int, WormFrameInfo>> m_frameInfos;
    int m_frameInfoHistorySize; // How many recent frames of info to keep (e.g., for m_frameInfos)

    // --- Merge/Split Management (Direction-Specific) ---
    struct MergeMapsContext {
        QMap<int, QSet<int>>& mergedGroups;
        QMap<int, int>& wormToMergeGroupMap;
    };
    MergeMapsContext getMergeMapsForFrame(int originalFrameNumber);

    // Forward tracking context merge state
    // Key: representative conceptual worm ID from the group. Value: set of all conceptual worm IDs in that merge.
    QMap<int, QSet<int>> m_forwardMergedGroups;
    // Key: conceptual wormId. Value: representative conceptual ID of its current merge group (forward context).
    QMap<int, int> m_forwardWormToMergeGroupMap;

    // Reverse tracking context merge state
    QMap<int, QSet<int>> m_reverseMergedGroups;
    QMap<int, int> m_reverseWormToMergeGroupMap;

    // Proactive merge detection and classification of merges (with tracked or untracked entities)
    void processFrameDataForMergesAndSplits(int frameNumber);

    // --- Paused Tracker Cache for Coordinated Split Resolution ---
    struct PausedTrackerInfo {
        int wormId;
        int originalFrameNumber;
        WormTracker* trackerInstance; // QPointer might be safer if lifetime is an issue
        QList<Tracking::DetectedBlob> reportedBlobs;
        // Could add timestamp or frame count for timeout
    };
    QMap<int, QList<PausedTrackerInfo>> m_pausedTrackerCache; // Key: originalFrameNumber
    void resolvePausedTrackersForFrame(int frameNumber); // Processes the cache

    // --- Configuration & State ---
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

    QMap<int, WormObject*> m_wormObjectsMap; // Key: conceptual wormId
    QList<WormTracker*> m_wormTrackers;      // List of active tracker instances
    QList<QThread*> m_trackerThreads;
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;

    int m_videoProcessingProgress;
    QMap<WormTracker*, int> m_individualTrackerProgress; // Progress per active tracker instance

    Tracking::AllWormTracks m_finalTracks; // Stores consolidated tracks from WormObjects
};

#endif // TRACKINGMANAGER_H
