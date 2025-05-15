#ifndef TRACKDATA_H
#define TRACKDATA_H

#include <opencv2/core.hpp>
#include <QPointF>
#include <QRectF>
#include <vector>
#include <map>

// Forward declaration
class WormObject; // If WormObject needs to be referenced here, though likely not for these structs


/**
 * @brief Represents a single point in a worm's track.
 */
struct WormTrackPoint {
    int frameNumberOriginal; // Frame number in the original video
    cv::Point2f position;    // Position (centroid) in video coordinates
    QRectF roi;              // ROI used for this worm at this frame (in video coordinates)
    // Potentially add confidence, state (e.g. merged) if needed per point
};

/**
 * @brief Typedef for storing all tracks.
 * Maps a unique worm ID to its sequence of track points.
 */
typedef std::map<int, std::vector<WormTrackPoint>> AllWormTracks;


/**
 * @brief Structure to pass initial information about a worm to be tracked.
 */
struct InitialWormInfo {
    int id;
    QRectF initialRoi; // ROI on the keyframe in video coordinates
};


#endif // TRACKDATA_H
