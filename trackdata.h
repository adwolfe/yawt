#ifndef TRACKDATA_H
#define TRACKDATA_H

#include <opencv2/core.hpp>
#include <QPointF>
#include <QRectF>
#include <vector>
#include <map>

// Include VideoLoader.h for the ThresholdAlgorithm enum.
// Make sure VideoLoader.h is accessible in your include paths.
// If VideoLoader.h is in a different directory, you might need to adjust the path.
#include "videoloader.h" // Assuming videoloader.h is in the same directory or in include path

// Forward declaration
class WormObject; // If WormObject needs to be referenced here, though likely not for these structs

/**
 * @brief Structure to hold parameters for thresholding.
 * This mirrors the relevant settings from VideoLoader.
 */
struct ThresholdSettings {
    ThresholdAlgorithm algorithm = ThresholdAlgorithm::Global;
    int thresholdValue = 127;
    bool assumeLightBackground = true;
    int adaptiveBlockSize = 11;
    double adaptiveCValue = 2.0;
    // Add any other relevant parameters from VideoLoader, e.g., blur settings if you make them configurable
};

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
