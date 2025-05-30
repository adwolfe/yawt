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
    WormObject(int id, const QRectF& initialRoi);

    int getId() const;
    const std::vector<Tracking::WormTrackPoint>& getTrackHistory() const;
    void updateTrackPoint(Tracking::WormTrackPoint point); //const cv::Point2f& position, const QRectF& roi);
    //cv::Point2f getCurrentPosition() const;
    //QRectF getCurrentRoi() const;
    //Tracking::TrackerState getCurrentState() const;

    //void setState(Tracking::TrackerState state, int mergedWithId = -1);
    //void setMergedWithId(int id);
    //int getMergedWithId() const;


private:
    int m_id;
    cv::Point2f m_currentPosition; // Current position in video coordinates
    QRectF m_currentRoi;           // Current ROI in video coordinates

    // Stores the history of positions.
    // The key (int) will be the original frame number to allow sparse updates
    // from forward and backward tracking and easier merging.
    std::map<int, Tracking::WormTrackPoint> m_trackHistoryMap;
    std::vector<Tracking::WormTrackPoint> m_trackHistoryVector; // For ordered access if needed, can be rebuilt from map

    void rebuildTrackHistoryVector(); // Helper to keep vector sorted by frame number
};

#endif // WORMWOBJECT_H
