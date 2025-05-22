// WormObject.h
#ifndef WORMWOBJECT_H
#define WORMWOBJECT_H

#include <opencv2/core.hpp>
#include <QRectF>
#include <QPointF>
#include <vector>
#include "trackingcommon.h"

class WormObject {
public:
    enum class TrackingState {
        Inactive,       // Not yet tracked or lost
        Tracking,       // Actively being tracked
        Merged,         // Merged with another worm/object
        Lost            // Tracking was lost
    };

    WormObject(int id, const QRectF& initialRoi);

    int getId() const;
    cv::Point2f getCurrentPosition() const;
    QRectF getCurrentRoi() const;
    TrackingState getCurrentState() const;
    const std::vector<Tracking::WormTrackPoint>& getTrackHistory() const;

    void updateTrackPoint(int originalFrameNum, const cv::Point2f& position, const QRectF& roi);
    void setState(TrackingState state, int mergedWithId = -1);
    void setMergedWithId(int id);
    int getMergedWithId() const;


private:
    int m_id;
    cv::Point2f m_currentPosition; // Current position in video coordinates
    QRectF m_currentRoi;           // Current ROI in video coordinates
    TrackingState m_currentState;
    int m_mergedWithId; // ID of the worm it's merged with, if any (-1 otherwise)

    // Stores the history of positions.
    // The key (int) will be the original frame number to allow sparse updates
    // from forward and backward tracking and easier merging.
    std::map<int, Tracking::WormTrackPoint> m_trackHistoryMap;
    std::vector<Tracking::WormTrackPoint> m_trackHistoryVector; // For ordered access if needed, can be rebuilt from map

    void rebuildTrackHistoryVector(); // Helper to keep vector sorted by frame number
};

#endif // WORMWOBJECT_H
