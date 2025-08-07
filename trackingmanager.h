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
#include <QDateTime>
#include <QPointer>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "trackingcommon.h"
#include "wormobject.h"
#include "videoprocessor.h" // For VideoProcessor type, not direct instantiation here
#include "wormtracker.h"    // For WormTracker type and enums
#include "trackingdatastorage.h" // Central data storage

// Constants for matching physical blobs on a frame
const double PHYSICAL_BLOB_IOU_THRESHOLD = 0.5; // Higher threshold for matching same physical blob
const double PHYSICAL_BLOB_CENTROID_MAX_DIST_SQ = 100.0; // Max squared distance (e.g., 10 pixels)

// No longer using paused worm resolution

// Represents a distinct physical (merged) blob observed on a specific frame.
struct FrameSpecificPhysicalBlob {
    int uniqueId;                           // Globally unique ID for this physical blob instance
    QRectF currentBoundingBox;
    cv::Point2f currentCentroid;            // Centroid of currentBoundingBox
    double currentArea;
    QSet<int> participatingWormTrackerIDs;  // Conceptual IDs of WormTrackers in this blob on this frame
    int frameNumber;                        // The frame this blob exists on
    int selectedByWormTrackerId;            // Which WT has selected this blob as their target (-1 = unselected)
    // QDateTime timeFirstReported;         // When this physical blob was first noted on this frame

    FrameSpecificPhysicalBlob() : uniqueId(-1), currentArea(0.0), frameNumber(-1), selectedByWormTrackerId(-1) {}
};

// No longer using pause mechanism - split resolution is immediate


// Forward declaration
class TrackingDataStorage;

class TrackingManager : public QObject {
    Q_OBJECT

public:
    explicit TrackingManager(QObject* parent = nullptr);
    explicit TrackingManager(TrackingDataStorage* storage, QObject* parent = nullptr);
    ~TrackingManager();

public slots:
    void startFullTrackingProcess(const QString& videoPath,
                                  const QString& dataDirectory,
                                  int keyFrameNum,
                                  const std::vector<Tracking::InitialWormInfo>& initialWorms,
                                  const Thresholding::ThresholdSettings& settings,
                                  int totalFramesInVideoHint);
    void cancelTracking();

private slots:
    // Video processing slots
    void handleInitialProcessingComplete(const std::vector<cv::Mat>& finalForwardFrames,
                                         const std::vector<cv::Mat>& finalReversedFrames,
                                         double fps,
                                         cv::Size frameSize);
    void handleVideoChunkProcessingError(int chunkId, const QString& errorMessage);
    void handleRangeProcessingProgress(int chunkId, int percentage);
    void handleRangeProcessingComplete(int chunkId,
                                       const std::vector<cv::Mat>& processedFrames,
                                       bool wasForwardChunk);

    // WormTracker slots
    void handleFrameUpdate(int reportingConceptualWormId,
                           int originalFrameNumber,
                           const Tracking::DetectedBlob& primaryBlob, // Anchor blob for track history
                           const Tracking::DetectedBlob& fullBlob,    // Full blob for merge/state processing
                           QRectF searchRoiUsed,
                           Tracking::TrackerState currentState,
                           const QList<Tracking::DetectedBlob>& splitCandidates = QList<Tracking::DetectedBlob>());
    void handleWormTrackerFinished();
    void handleWormTrackerError(int reportingConceptualWormId, QString errorMessage);
    void handleWormTrackerProgress(int reportingConceptualWormId, int percentDone);



signals:
    void overallTrackingProgress(int percentage);
    void trackingStatusUpdate(const QString& statusMessage);
    void allTracksUpdated(const Tracking::AllWormTracks& tracks); // For visualization/output
    void trackingFinishedSuccessfully(const QString& outputPath);
    void trackingFailed(const QString& reason);
    void trackingCancelled();


private:
    // Core logic for frame-atomic merge/split handling
    void processFrameSpecificMerge(int signedWormId, int frameNumber,
                                   const Tracking::DetectedBlob& reportedFullBlob,
                                   WormTracker* reportingTrackerInstance);
    void processFrameSpecificSplit(int signedWormId, int frameNumber,
                                   const QList<Tracking::DetectedBlob>& allSplitCandidates,
                                   const Tracking::DetectedBlob& chosenCandidate,
                                   WormTracker* reportingTrackerInstance);
    bool attemptImmediateSplitResolution(int conceptualWormId, int frameNumber, 
                                     const QList<Tracking::DetectedBlob>& allCandidates,
                                     const Tracking::DetectedBlob& chosenCandidate,
                                     WormTracker* trackerInstance);

    // Helper functions for signed worm IDs (direction-aware)
    int getSignedWormId(int conceptualWormId, WormTracker::TrackingDirection direction);
    int getUnsignedWormId(int signedWormId);
    WormTracker::TrackingDirection getDirectionFromSignedId(int signedWormId);

    // Helper and utility functions
    void cleanupThreadsAndObjects();
    void launchWormTrackers();
    void updateOverallProgress();
    void checkForAllTrackersFinished();
    bool outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFileName) const;
    double calculateIoU(const QRectF& r1, const QRectF& r2) const;
    void assembleProcessedFrames(); // For parallel video processing
    
    // JSON storage methods
    QString createVideoSpecificDirectory(const QString& dataDirectory, const QString& videoPath);
    void saveThresholdSettings(const QString& directoryPath, const Thresholding::ThresholdSettings& settings);
    void saveInputBlobs(const QString& directoryPath, const std::vector<Tracking::InitialWormInfo>& worms);
    bool compareThresholdSettings(const QString& filePath, const Thresholding::ThresholdSettings& currentSettings);
    QJsonObject thresholdSettingsToJson(const Thresholding::ThresholdSettings& settings) const;
    QJsonObject initialWormInfoToJson(const Tracking::InitialWormInfo& worm) const;

    // Basic tracking parameters
    QString m_videoPath;
    QString m_dataDirectory;
    QString m_videoSpecificDirectory;
    int m_keyFrameNum;
    std::vector<Tracking::InitialWormInfo> m_initialWormInfos;
    Thresholding::ThresholdSettings m_thresholdSettings;
    int m_totalFramesInVideoHint;
    bool m_isTrackingRunning;
    bool m_cancelRequested;

    // Video processing members (for parallel processing)
    QList<QPointer<QThread>> m_videoProcessorThreads;
    int m_videoProcessorsFinishedCount;
    int m_totalVideoChunksToProcess;
    QMap<int, std::vector<cv::Mat>> m_assembledForwardFrameChunks;
    QMap<int, std::vector<cv::Mat>> m_assembledBackwardFrameChunks;
    QMap<int, int> m_videoChunkProgressMap;
    std::vector<cv::Mat> m_finalProcessedForwardFrames;
    std::vector<cv::Mat> m_finalProcessedReversedFrames;
    double m_videoFps;
    cv::Size m_videoFrameSize;

    // Worm object and tracker management
    QMap<int, WormObject*> m_wormObjectsMap; // Stores track history for each conceptual worm
    QList<WormTracker*> m_wormTrackersList;  // List of active tracker instances
    QMap<int, WormTracker*> m_wormIdToForwardTrackerInstanceMap;
    QMap<int, WormTracker*> m_wormIdToBackwardTrackerInstanceMap;
    QList<QPointer<QThread>> m_trackerThreads;
    int m_expectedTrackersToFinish;
    int m_finishedTrackersCount;
    int m_videoProcessingOverallProgress;
    QMap<WormTracker*, int> m_individualTrackerProgress; // WormTracker instance -> progress %
    Tracking::AllWormTracks m_finalTracks; // Final consolidated tracks
    TrackingDataStorage* m_storage;        // Pointer to central data storage

    // --- NEW Data Structures for Frame-Atomic Merge Logic ---
    // Maps frame number to a list of distinct physical merged blobs observed on that frame.
    QMap<int, QList<FrameSpecificPhysicalBlob>> m_frameMergeRecords;

    // Maps frame number to another map, which maps conceptual Worm ID to its chosen blob after a split decision.
    QMap<int, QMap<int, Tracking::DetectedBlob>> m_splitResolutionMap;

    // To generate unique IDs for FrameSpecificPhysicalBlob instances
    int m_nextPhysicalBlobId;

    // Associates a conceptual worm ID with the uniqueId of the FrameSpecificPhysicalBlob it's part of in the current frame.
    // This helps find which physical blob a worm belonged to in the previous frame when it pauses.
    QMap<int, int> m_wormToPhysicalBlobIdMap;


    // General utilities
    QMutex m_dataMutex; // Protects shared data structures
    
    // Helper method to register meta types
    void registerMetaTypes();
};

#endif // TRACKINGMANAGER_H
