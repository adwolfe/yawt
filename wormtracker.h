// WormTracker.h
#ifndef WORMTRACKER_H
#define WORMTRACKER_H

#include <QObject>
#include <QThread> // For Q_INVOKABLE if run in thread
#include <QRectF>
#include <QPointF>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> // For findContours, moments etc.
#include "wormobject.h"        // For WormObject::TrackingState

// Forward declaration
class WormObject;

class WormTracker : public QObject {
    Q_OBJECT

public:
    enum class TrackingDirection {
        Forward,
        Backward
    };

    explicit WormTracker(int wormId,
                         QRectF initialRoi,
                         TrackingDirection direction,
                         int videoKeyFrameNum, // Original video keyframe number
                         QObject *parent = nullptr);
    ~WormTracker();

    void setFrames(const std::vector<cv::Mat>* frames); // Pointer to avoid copying large data

public slots:
    void startTracking(); // Main slot to begin the tracking loop
    void stopTracking();  // Slot to gracefully stop tracking

signals:
    void finished(); // Emitted when tracking for this instance is done
    void progress(int wormId, int percentDone);
    void positionUpdated(int wormId, int originalFrameNumber, QPointF newPosition, QRectF newRoi);
    void stateChanged(int wormId, WormObject::TrackingState newState, int associatedWormId = -1); // e.g. mergedWithId
    void errorOccurred(int wormId, QString errorMessage);

private:
    // Tracking logic for a single frame
    bool processSingleFrame(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentRoi, cv::Point2f& foundPosition);
    QRectF adjustRoi(const cv::Point2f& wormCenter, const cv::Size& frameSize, const QSizeF& wormSizeGuess);


    int m_wormId;
    QRectF m_currentRoi; // Current ROI in video coordinates
    cv::Point2f m_lastKnownPosition;
    TrackingDirection m_direction;
    int m_videoKeyFrameNum; // The original frame number of the keyframe

    const std::vector<cv::Mat>* m_framesToProcess; // Pointer to the sequence of frames (forward or reversed)
    bool m_trackingActive;
    QSizeF m_estimatedWormSize; // Could be initialized or adaptively learned
};

#endif // WORMTRACKER_H
