#include "wormobject.h"
#include <algorithm> // For std::sort

WormObject::WormObject(int id, const QRectF& initialRoi)
    : m_id(id),
    m_currentPosition(initialRoi.center().x(), initialRoi.center().y()), // Initial position is center of ROI
    m_currentRoi(initialRoi),
    m_currentState(TrackingState::Inactive),
    m_mergedWithId(-1)
{
}

int WormObject::getId() const {
    return m_id;
}

cv::Point2f WormObject::getCurrentPosition() const {
    return m_currentPosition;
}

QRectF WormObject::getCurrentRoi() const {
    return m_currentRoi;
}

WormObject::TrackingState WormObject::getCurrentState() const {
    return m_currentState;
}

const std::vector<Tracking::WormTrackPoint>& WormObject::getTrackHistory() const {
    // Ensure vector is up-to-date if it's used frequently.
    // For now, it's rebuilt on demand or after updates.
    // If direct access to the vector is primary, update it in updateTrackPoint.
    return m_trackHistoryVector;
}

void WormObject::updateTrackPoint(int originalFrameNum, const cv::Point2f& position, const QRectF& roi) {
    m_currentPosition = position;
    m_currentRoi = roi;
    m_currentState = TrackingState::Tracking; // Assume tracking if position is updated

    Tracking::WormTrackPoint point;
    point.frameNumberOriginal = originalFrameNum;
    point.position = position;
    point.roi = roi;

    m_trackHistoryMap[originalFrameNum] = point;
    rebuildTrackHistoryVector(); // Rebuild the sorted vector
}

void WormObject::setState(TrackingState state, int mergedWithId) {
    m_currentState = state;
    if (state == TrackingState::Merged) {
        m_mergedWithId = mergedWithId;
    } else {
        m_mergedWithId = -1;
    }
}

void WormObject::setMergedWithId(int id) {
    m_mergedWithId = id;
}

int WormObject::getMergedWithId() const {
    return m_mergedWithId;
}


void WormObject::rebuildTrackHistoryVector() {
    m_trackHistoryVector.clear();
    for (const auto& pair : m_trackHistoryMap) {
        m_trackHistoryVector.push_back(pair.second);
    }
    // Sort by frame number
    std::sort(m_trackHistoryVector.begin(), m_trackHistoryVector.end(),
              [](const Tracking::WormTrackPoint& a, const Tracking::WormTrackPoint& b) {
                  return a.frameNumberOriginal < b.frameNumberOriginal;
              });
}
