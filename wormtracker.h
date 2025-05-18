// wormtracker.h
#ifndef WORMTRACKER_H
#define WORMTRACKER_H

#include <QObject>
#include <QThread> // For Q_INVOKABLE if run in thread, and QThread::msleep
#include <QRectF>
#include <QPointF>
#include <QList>   // For QList of DetectedBlob
#include <QSizeF>  // For QSizeF
#include <vector>  // For std::vector<cv::Mat>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> // For findContours, moments etc.

#include "wormobject.h"        // For WormObject::TrackingState (ensure lowercase if filename is)
#include "trackingcommon.h"    // For TrackingHelper::DetectedBlob (ensure lowercase if filename is)


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
        Idle,                   // Not yet started or stopped
        TrackingSingle,         // Confidently tracking one target
        PotentialMergeOrSplit,  // Seeing multiple blobs, trying to resolve or follow primary (DEFUNCT?)
        TrackingMerged,         // Knows it's tracking an entity declared as merged by manager
        PausedAwaitingSplitDecision // Detected a split and is waiting for TrackingManager
    };
    Q_ENUM(TrackerState) // Make state accessible via meta-object system if needed for debugging

    explicit WormTracker(int wormId,
                         QRectF initialRoi,
                         TrackingDirection direction,
                         int videoKeyFrameNum, // Original video keyframe number
                         QObject *parent = nullptr);
    ~WormTracker();

    void setFrames(const std::vector<cv::Mat>* frames); // Pointer to avoid copying large data
    int getWormId() const { return m_wormId; }          // Getter for ID
    TrackerState getCurrentTrackerState() const { return m_currentState; } // Getter for state


public slots:
    void startTracking(); // Main slot to begin the tracking loop
    void continueTracking();
    void stopTracking();  // Slot to gracefully stop tracking (e.g., on cancellation)

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

    // This signal reports changes to the WormObject's state, which might be influenced by TrackingManager
    void stateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId = -1);
    void errorOccurred(int wormId, QString errorMessage);


private:

    // Main processing logic for a single frame
    bool processSingleFrame(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentRoiInOut, cv::Point2f& foundPositionOut);
    // Adjusts ROI based on found position and estimated size
    QRectF adjustRoi(const cv::Point2f& wormCenter, const cv::Size& frameSize, const QSizeF& wormSizeGuess);
    // Finds all plausible blobs within the current ROI based on defined criteria
    QList<TrackingHelper::DetectedBlob> findPlausibleBlobsInRoi(const cv::Mat& fullFrame, const QRectF& roi);


    int m_wormId;
    QRectF m_currentRoi; // Current ROI in video coordinates
    cv::Point2f m_lastKnownPosition; // Last confirmed position of the primary target
    TrackingDirection m_direction;
    int m_videoKeyFrameNum; // The original frame number of the keyframe
    int m_currFrameNum = 0; // relative to list of frames delivered (frame 0 is keyframe or keyframe-1)

    const std::vector<cv::Mat>* m_framesToProcess; // Pointer to the sequence of frames
    bool m_trackingActive;      // Overall active state (true if tracking loop should run)
    TrackerState m_currentState; // Internal state of the tracker

    QSizeF m_estimatedWormSize; // Estimated size of the worm, used for ROI adjustment

    // Parameters for what constitutes a "plausible worm blob" - could be made configurable
    double m_minBlobArea;
    double m_maxBlobArea;
    double m_minAspectRatio; // width/height or height/width, always >= 1
    double m_maxAspectRatio;

    // Stores the last primary blob this tracker was focused on. Useful for split detection.
    TrackingHelper::DetectedBlob m_lastPrimaryBlob;
};

#endif // WORMTRACKER_H
