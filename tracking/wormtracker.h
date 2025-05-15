// WormTracker.h
#ifndef WORMTRACKER_H
#define WORMTRACKER_H

#include <QObject>
#include <QThread>
#include <QRectF>
#include <QPointF>
#include <QList> // For QList of DetectedBlob
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "wormobject.h"        // For WormObject::TrackingState
#include "trackingcommon.h"    // For TrackingHelper::DetectedBlob (ensure lowercase include if filename is)

// Forward declaration
class WormObject;

class WormTracker : public QObject {
    Q_OBJECT

public:
    enum class TrackingDirection {
        Forward,
        Backward
    };

    // Internal state for the tracker's understanding of its current situation
    enum class TrackerState {
        Idle,
        TrackingSingle,         // Confidently tracking one target
        PotentialMergeOrSplit,  // Seeing multiple blobs, trying to resolve or follow primary
        TrackingMerged,         // Knows it's tracking an entity declared as merged by manager
        PausedAwaitingSplitDecision // Detected a split and is waiting for TrackingManager
    };
    Q_ENUM(TrackerState) // Make state accessible via meta-object system if needed for debugging

    explicit WormTracker(int wormId,
                         QRectF initialRoi,
                         TrackingDirection direction,
                         int videoKeyFrameNum,
                         QObject *parent = nullptr);
    ~WormTracker();

    void setFrames(const std::vector<cv::Mat>* frames);
    int getWormId() const { return m_wormId; } // Getter for ID

public slots:
    void startTracking();
    void stopTracking(); // For external stop request (e.g., cancellation)
    /**
     * @brief Instructs the tracker to resume tracking a specific new target.
     * Called by TrackingManager after a split has been resolved.
     * @param targetBlob The specific blob (with centroid, ROI) to start tracking.
     */
    void resumeTrackingWithNewTarget(const TrackingHelper::DetectedBlob& targetBlob);

    /**
     * @brief Instructs the tracker that its current target is now considered part of a merged entity.
     * @param mergedEntityID A representative ID for the merge (optional, for logging).
     * @param mergedBlobCentroid The centroid of the merged blob it should now follow.
     * @param mergedBlobRoi The ROI encompassing the merged blob.
     */
    void confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoi);


signals:
    void finished(); // Emitted when tracking for this instance is done (normally or due to stop)
    void progress(int wormId, int percentDone); // Percent of its assigned frames processed

    // Modified: Includes count of plausible blobs and area of the primary target
    void positionUpdated(int wormId,
                         int originalFrameNumber,
                         QPointF newPosition,    // Centroid of the primary blob it's tracking
                         QRectF newRoi,          // Its new adjusted ROI
                         int plausibleBlobsFoundInRoi, // How many "worm-like" blobs it saw
                         double primaryBlobArea);     // Area of the blob at newPosition

    // New: Emitted when a tracker, previously tracking one entity, now sees multiple distinct entities
    // and is pausing to await instructions from TrackingManager.
    void splitDetectedAndPaused(int wormId,
                                int originalFrameNumber,
                                const QList<TrackingHelper::DetectedBlob>& detectedBlobs);

    void stateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId = -1); // e.g. mergedWithId (from WormObject perspective)
    void errorOccurred(int wormId, QString errorMessage);


private:
    bool processSingleFrame(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentRoiInOut, cv::Point2f& foundPositionOut);
    QRectF adjustRoi(const cv::Point2f& wormCenter, const cv::Size& frameSize, const QSizeF& wormSizeGuess);
    QList<TrackingHelper::DetectedBlob> findPlausibleBlobsInRoi(const cv::Mat& fullFrame, const QRectF& roi, double minArea, double maxArea, double minAspectRatio, double maxAspectRatio);


    int m_wormId;
    QRectF m_currentRoi;
    cv::Point2f m_lastKnownPosition;
    TrackingDirection m_direction;
    int m_videoKeyFrameNum;

    const std::vector<cv::Mat>* m_framesToProcess;
    bool m_trackingActive;      // Overall active state (can be set false by stopTracking)
    TrackerState m_currentState; // Internal state of the tracker

    QSizeF m_estimatedWormSize; // Could be based on initial selection or adaptive
    // Parameters for what constitutes a "plausible worm blob"
    double m_minBlobArea;
    double m_maxBlobArea;
    double m_minAspectRatio;
    double m_maxAspectRatio;

    // Temporary storage if needed between frames for complex logic
    TrackingHelper::DetectedBlob m_lastPrimaryBlob;
};

#endif // WORMTRACKER_H
