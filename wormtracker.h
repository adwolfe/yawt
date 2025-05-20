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

static const double EXPANSION_FACTOR = 1.25;               //
static const int MAX_EXPANSION_ITERATIONS = 3;

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
                         QRectF initialRoi,
                         TrackingDirection direction,
                         int videoKeyFrameNum, // Original video keyframe number
                         QObject *parent = nullptr);
    ~WormTracker();

    void setFrames(const std::vector<cv::Mat>* frames); // Pointer to avoid copying large data
    int getWormId() const { return m_wormId; }          // Getter for ID
    TrackerState getCurrentTrackerState() const { return m_currentState; } // Getter for state


public slots:
    void startTracking();           // Main slot to begin the tracking loop
    void continueTracking();        // Recursive, called per frame
    void stopTracking();            // Slot to gracefully stop tracking (e.g., on cancellation)

    /**
     * @brief Instructs the tracker to resume tracking a specific new target.
     * Called by TrackingManager after a split has been resolved.
     * @param targetBlob The specific blob (with centroid, ROI) to start tracking.
     */
    void resumeTrackingWithNewTarget(const TrackingHelper::DetectedBlob& targetBlob);


    // MARKED FOR DELETION
    /**
     * @brief Instructs the tracker that its current target is now considered part of a merged entity.
     * @param mergedEntityID A representative ID for the merge (optional, for logging).
     * @param mergedBlobCentroid The centroid of the merged blob it should now follow.
     * @param mergedBlobRoi The ROI encompassing the merged blob.
     */
    void confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoi);


signals:
    void finished();                                    // Emitted when tracking for this instance is done (normally or due to stop)
    void progress(int wormId, int percentDone);         // Percent of its assigned frames processed
    void positionUpdated(int wormId,                    // Worm ID
                         int originalFrameNumber,       // Absolute frame ID
                         QPointF newPosition,           // Centroid of the primary blob it's tracking
                         QRectF newRoi,                 // Its new adjusted ROI
                         int plausibleBlobsFoundInRoi,  // How many "worm-like" blobs it saw
                         double primaryBlobArea);       // Area of the blob at newPosition

    // New: Emitted when a tracker, previously tracking one merged entity, now sees multiple distinct entities
    // and is pausing to await instructions from TrackingManager. (I.e., is it a split? or a NEW worm?)
    void splitDetectedAndPaused(int wormId,
                                int originalFrameNumber,
                                const QList<TrackingHelper::DetectedBlob>& detectedBlobs);

    // This signal reports changes to the WormObject's state, which might be influenced by TrackingManager
    void stateChanged(int wormId, TrackerState newState, int associatedWormId = -1);
    void errorOccurred(int wormId, QString errorMessage);


private:
    // Main processing logic for a frame
    bool processFrameAsSingleWorm(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentRoiInOut);    // Single worm, though other worms may be near
    bool processFrameAsMergedWorms(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentRoiInOut, cv::Point2f& foundPositionOut);   // When merged, may split or have a NEW worm join the party

    // Helper functions
    QRectF adjustRoiSize(const QRectF roiInOut, const cv::Size& frameSize);                                                                 // For ROI expansion
    QRectF adjustRoiPos(const cv::Point2f& wormCenter, const cv::Size& frameSize);                                                          // For ROI position adjustment over time. Fixed size.
    //QRectF adjustRoi(const cv::Point2f& wormCenter, const cv::Size& frameSize, const QSizeF& wormSizeGuess);                                // Adjusts ROI based on found position
    QList<TrackingHelper::DetectedBlob> findPlausibleBlobsInRoi(const cv::Mat& fullFrame, const QRectF& roi);                               // Finds all plausible blobs within the current ROI based on defined criteria
    TrackingHelper::DetectedBlob findLargestBlobComponentInMask(const cv::Mat& mask, const QString& debugContextName);                      // For merge events: ignore small bits and pieces of the mask

    // Permanent values
    int m_wormId;                                       // Doesn't change. Identifies the worm we are tracking
    TrackingDirection m_direction;                      // Doesn't change. Identifies the direction we go.
    const std::vector<cv::Mat>* m_framesToProcess;      // Pointer to the sequence of frames
    qreal m_initialRoiEdge;                             // The fixed ROI size provided by the user
                //

    double m_minBlobArea;                               // Parameters for what constitutes a "plausible worm blob"
    double m_maxBlobArea;                               // Will be made configurable

    // Transient values
    QRectF m_currentRoi;                                // Current ROI in video coordinates for the frame
    cv::Point2f m_lastKnownPosition;                    // Last confirmed position of the primary target
    int m_videoKeyFrameNum;                             // The original frame number of the keyframe
    int m_currFrameNum = 0;                             // relative to list of frames delivered (frame 0 is keyframe or keyframe-1)
    bool m_trackingActive;                              // Overall active state (true if tracking loop should run)
    TrackerState m_currentState;                        // Internal state of the tracker
    TrackingHelper::DetectedBlob m_lastPrimaryBlob;     // Stores the last primary blob this tracker was focused on. Useful for split detection.

    // [DEFUNCT]
    double m_minAspectRatio; // width/height or height/width, always >= 1
    double m_maxAspectRatio;
    QSizeF m_estimatedWormSize;                         // Estimated size of the worm, used for ROI adjustment


};

#endif // WORMTRACKER_H
