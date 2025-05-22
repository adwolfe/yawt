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

#include "trackingcommon.h" // Defines Tracking::DetectedBlob

// Forward declaration
// class WormObject; // Not directly used in this header's public interface

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
        AmbiguouslySingle,              // More than one plausible blob in ROI, but one is primary
        TrackingMerged,                 // Believed to be tracking our worm as part of a merged entity
        AmbiguouslyMerged,              // Multiple blobs in ROI, and primary blob is large/complex, potentially merged
        PausedForSplit,                 // Detected a split from a merged state and is waiting for TrackingManager
        // TrackingLost                 // Optional: If tracking is definitively lost and cannot recover
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
    TrackingDirection getDirection() const { return m_direction; }
    TrackerState getCurrentTrackerState() const { return m_currentState; }
    Tracking::DetectedBlob getLastPrimaryBlob() const { return m_lastPrimaryBlob; } // Getter for the last tracked blob


public slots:
    void startTracking();
    void continueTracking(); // Internal slot for loop progression
    void stopTracking();     // Request to stop the tracking loop
    void resumeTrackingWithNewTarget(const Tracking::DetectedBlob& targetBlob); // Called by TrackingManager after a split
    void confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoi); // Called by TrackingManager


signals:
    void finished(); // Emitted when the tracker has finished its work (completed, stopped, or errored)
    void progress(int wormId, int percentDone); // Reports percentage of frames processed

    /**
     * @brief Emitted each frame a worm's position is updated.
     * @param wormId The conceptual ID of the worm being tracked.
     * @param originalFrameNumber The frame number in the original video sequence.
     * @param primaryBlob The characteristics of the blob chosen as the primary target for this frame.
     * @param searchRoiUsed The fixed-size search ROI that was used to find blobs in this frame.
     * @param plausibleBlobsFoundInSearchRoi The total number of plausible blobs found within searchRoiUsed.
     */
    void positionUpdated(int wormId,
                         int originalFrameNumber,
                         const Tracking::DetectedBlob& primaryBlob,
                         QRectF searchRoiUsed,
                         int plausibleBlobsFoundInSearchRoi);

    /**
     * @brief Emitted when the tracker was in a merged state and detects that the merged entity has split.
     * The tracker will pause after emitting this, awaiting instruction from TrackingManager.
     * @param wormId The ID of the worm this tracker is responsible for.
     * @param originalFrameNumber The frame number where the split was detected.
     * @param detectedBlobs A list of all distinct blobs found after the split.
     */
    void splitDetectedAndPaused(int wormId,
                                int originalFrameNumber,
                                const QList<Tracking::DetectedBlob>& detectedBlobs);

    /**
     * @brief Emitted when the tracker's internal state changes.
     * @param wormId The ID of the worm.
     * @param newState The new TrackerState.
     * @param associatedEntityId Optional: If merged, the ID of the entity it's merged with (e.g. representative ID from TrackingManager).
     * If split, could be an ID related to the split event.
     */
    void stateChanged(int wormId, TrackerState newState, int associatedEntityId = -1);

    void errorOccurred(int wormId, QString errorMessage); // If an unrecoverable error happens


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


    // --- Configuration & State (Permanent or set at init) ---
    int m_wormId;                           // Conceptual ID of the worm this tracker instance is for
    TrackingDirection m_direction;          // Forward or Backward from keyframe
    const std::vector<cv::Mat>* m_framesToProcess; // Pointer to frame data (not owned)
    qreal m_initialRoiEdge;                 // Edge length of the fixed square search ROI
    int m_videoKeyFrameNum;                 // Original video keyframe number where tracking starts
    double m_minBlobArea;                   // Plausibility parameter: minimum blob area
    double m_maxBlobArea;                   // Plausibility parameter: maximum blob area (for a single worm)

    // --- Transient State (Changes during tracking) ---
    QRectF m_currentSearchRoi;              // Current fixed-size ROI used for searching in the current frame
    cv::Point2f m_lastKnownPosition;        // Last known centroid of the tracked blob (used as fallback)
    int m_currFrameNum;                     // Index in m_framesToProcess (0 to N-1)
    bool m_trackingActive;                  // Flag to control the tracking loop
    TrackerState m_currentState;            // Current operational state of this tracker
    Tracking::DetectedBlob m_lastPrimaryBlob; // Characteristics of the blob successfully tracked in the *previous* frame
        // This is updated *after* processing a frame and choosing a target.
};

#endif // WORMTRACKER_H
