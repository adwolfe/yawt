#ifndef RETRACKINGPROCESSOR_H
#define RETRACKINGPROCESSOR_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QString>
#include <QRectF>
#include <QPointF>
#include <opencv2/opencv.hpp>
#include <vector>
#include "trackingcommon.h"
#include "thresholdingutils.h"

class TrackingDataStorage;
class WormTracker;

/**
 * @brief Specialized processor for retracking Fix blobs using saved thresholded video
 * 
 * This class handles the retracking process by:
 * 1. Loading frames from the saved thresholded video
 * 2. Processing the specified frame range around the Fix blob location
 * 3. Running blob detection and tracking within the ROI
 * 4. Integrating results back into the main tracking data
 */
class RetrackingProcessor : public QObject
{
    Q_OBJECT

public:
    struct RetrackingResult {
        int fixBlobId;
        int processedFrames;
        int successfulFrames;
        bool completed;
        QString errorMessage;
        std::vector<Tracking::DetectedBlob> newBlobs;
        
        RetrackingResult() 
            : fixBlobId(-1), processedFrames(0), successfulFrames(0), completed(false) {}
    };

    explicit RetrackingProcessor(QObject* parent = nullptr);
    ~RetrackingProcessor();

    // Main retracking function
    void startRetracking(const QString& originalVideoPath,
                        const Thresholding::ThresholdSettings& thresholdSettings,
                        int fixBlobId,
                        const QRectF& initialROI,
                        int startFrame,
                        int endFrame,
                        bool replaceExisting,
                        bool extendTrack,
                        TrackingDataStorage* dataStorage);

    // Control functions
    void requestStop();
    bool isRunning() const;

public slots:
    void processRetracking();
    void startRetrackingSlot(const QString& originalVideoPath,
                           const Thresholding::ThresholdSettings& thresholdSettings,
                           int fixBlobId,
                           const QRectF& initialROI,
                           int startFrame,
                           int endFrame,
                           bool replaceExisting,
                           bool extendTrack,
                           TrackingDataStorage* dataStorage);

signals:
    void retrackingProgress(int fixBlobId, int currentFrame, int totalFrames);
    void retrackingCompleted(const RetrackingResult& result);
    void retrackingFailed(int fixBlobId, const QString& errorMessage);
    void statusUpdate(const QString& message);

private:
    // Core processing functions
    bool loadOriginalVideo(const QString& videoPath);
    bool processFrameRangeWithWormTracker();
    bool loadAndThresholdFrameRange();
    
    // WormTracker integration
    void setupWormTracker();
    void collectTrackingResults(int wormId,
                               int originalFrameNumber,
                               const Tracking::DetectedBlob& primaryBlob,
                               const Tracking::DetectedBlob& fullBlob,
                               QRectF searchRoiUsed,
                               Tracking::TrackerState currentState,
                               const QList<Tracking::DetectedBlob>& splitCandidates = QList<Tracking::DetectedBlob>());
    
    // Result integration
    void integrateResults();
    void replaceExistingTrack();
    void extendExistingTrack();
    
    // Utility functions
    QRectF expandROI(const QRectF& baseROI, double expansionFactor = 1.5) const;
    
    // Member variables
    QString m_videoPath;
    Thresholding::ThresholdSettings m_thresholdSettings;
    int m_fixBlobId;
    QRectF m_initialROI;
    QRectF m_expandedROI;  // Expanded search area
    int m_startFrame;
    int m_endFrame;
    bool m_replaceExisting;
    bool m_extendTrack;
    
    // Processing state
    cv::VideoCapture m_videoCapture;
    std::vector<cv::Mat> m_thresholdedFrames; // Frames for WormTracker
    WormTracker* m_wormTracker;
    QThread* m_trackerThread;
    std::vector<Tracking::WormTrackPoint> m_trackingResults;
    bool m_isRunning;
    bool m_stopRequested;
    
    // Data storage reference
    TrackingDataStorage* m_dataStorage;
    
    // Thread safety
    mutable QMutex m_mutex;
    
    // Constants for tracking parameters
    static constexpr double ROI_EXPANSION_FACTOR = 1.8;
    static constexpr int MIN_SUCCESSFUL_FRAMES_RATIO = 60; // Percentage
};

#endif // RETRACKINGPROCESSOR_H