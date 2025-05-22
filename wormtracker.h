// wormtracker.h
#ifndef WORMTRACKER_H
#define WORMTRACKER_H

#include <QObject>
#include <QThread>
#include <QRectF>
#include <QPointF>
#include <QList>
#include <QSizeF>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

//#include "wormobject.h"
#include "trackingcommon.h"

// static const double EXPANSION_FACTOR = 1.25; // Defunct if not used for temporary analysis ROI
// static const int MAX_EXPANSION_ITERATIONS = 3; // Defunct if not used

// Forward declaration
//class WormObject;

class WormTracker : public QObject {
    Q_OBJECT

public:
    enum class TrackingDirection {
        Forward,
        Backward
    };
    Q_ENUM(TrackingDirection)

    // Internal state for the tracker
    enum class TrackerState {
        Idle,                           // Not yet started or stopped
        TrackingSingle,                 // Confidently tracking one target
        AmbiguouslySingle,              // A new blob has entered the mix
        TrackingMerged,                 // Knows it's tracking multiple entities that combined; working hard to track our target but it's a guess
        AmbiguouslyMerged,              // The pile keeps growing
        PausedForSplit,                 // Detected a split and is waiting for TrackingManager to tell us which to follow
        //TrackingLost                    // Happens sometimes on poor videos based on threshold settings
    };
    Q_ENUM(TrackerState)

    explicit WormTracker(int wormId,
                         QRectF initialRoi, // This is the fixed-size ROI
                         TrackingDirection direction,
                         int videoKeyFrameNum, // Original video keyframe number
                         QObject *parent = nullptr);
    ~WormTracker();

    void setFrames(const std::vector<cv::Mat>* frames); // Pointer to avoid copying large data
    int getWormId() const { return m_wormId; }
    TrackingDirection getDirection() const { return m_direction; } // Getter for direction
    TrackerState getCurrentTrackerState() const { return m_currentState; }


public slots:
    void startTracking();
    void continueTracking();
    void stopTracking();
    void resumeTrackingWithNewTarget(const Tracking::DetectedBlob& targetBlob);
    void confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoi);


signals:
    void finished();
    void progress(int wormId, int percentDone);
    void positionUpdated(int wormId,
                         int originalFrameNumber,
                         QPointF newPosition,
                         QRectF newRoi, // This should be the fixed-size tracking ROI
                         int plausibleBlobsFoundInRoi,
                         double primaryBlobArea);
    void splitDetectedAndPaused(int wormId,
                                int originalFrameNumber,
                                const QList<Tracking::DetectedBlob>& detectedBlobs);
    void stateChanged(int wormId, TrackerState newState, int associatedWormId = -1);
    void errorOccurred(int wormId, QString errorMessage);


private:
    // Main processing logic for a frame
    bool processFrameAsSingleWorm(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiInOut);
    bool processFrameAsMergedWorms(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiInOut);

    // Helper functions
    QRectF adjustRoiPos(const cv::Point2f& wormCenter, const cv::Size& frameSize); // Adjusts fixed-size ROI position
    QList<Tracking::DetectedBlob> findPlausibleBlobsInRoi(const cv::Mat& fullFrame, const QRectF& roi);
    Tracking::DetectedBlob findLargestBlobComponentInMask(const cv::Mat& mask, const QString& debugContextName);

    /**
     * @brief Analyzes the overlap and new growth between a blob from the previous frame
     * and a (potentially larger/merged) blob in the current frame.
     * @param previousFrameBlob The DetectedBlob of the worm from the previous frame.
     * @param currentFrameBlob The DetectedBlob from the current frame (could be a merged entity).
     * @param frameSize The size of the current video frame (for creating masks).
     * @param originalFrameNumberForDebug Optional: Frame number for debug logging.
     * @return A DetectedBlob representing the most likely continuation of previousFrameBlob
     * within currentFrameBlob. Returns an invalid DetectedBlob if analysis is inconclusive.
     */
    Tracking::DetectedBlob findPersistingComponent(
        const Tracking::DetectedBlob& previousFrameBlob,
        const Tracking::DetectedBlob& currentFrameBlob,
        const cv::Size& frameSize,
        int originalFrameNumberForDebug = -1);


    // Permanent values
    int m_wormId;
    TrackingDirection m_direction;
    const std::vector<cv::Mat>* m_framesToProcess;
    qreal m_initialRoiEdge; // The edge length of the fixed square ROI

    double m_minBlobArea; // Plausibility parameters
    double m_maxBlobArea;

    // Transient values
    QRectF m_currentSearchRoi; // Current fixed-size ROI for searching in the frame
    cv::Point2f m_lastKnownPosition;
    int m_videoKeyFrameNum;
    int m_currFrameNum = 0;
    bool m_trackingActive;
    TrackerState m_currentState;
    Tracking::DetectedBlob m_lastPrimaryBlob; // Last successfully tracked blob characteristics

    // Defunct or to be re-evaluated:
    // double m_minAspectRatio; // Not currently used by findPlausibleBlobsInRoi directly
    // double m_maxAspectRatio; // Not currently used
    // QSizeF m_estimatedWormSize; // The fixed ROI m_initialRoiEdge replaces this for ROI definition
};

#endif // WORMTRACKER_H
