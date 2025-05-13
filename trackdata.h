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
 * @brief Structure to hold parameters for thresholding and pre-processing.
 * This mirrors the relevant settings from VideoLoader and adds pre-processing options.
 */
struct ThresholdSettings {
    // General setting for interpreting pixel values (background vs. foreground)
    // If true, assumes a light background and darker objects of interest.
    // This typically maps to cv::THRESH_BINARY_INV for dark objects on light bg,
    // or cv::THRESH_BINARY for light objects on dark bg.
    bool assumeLightBackground = true;

    // Main algorithm choice for thresholding.
    // This determines which of the subsequent parameter groups are relevant.
    ThresholdAlgorithm algorithm = ThresholdAlgorithm::Global;

    // --- Parameters for Global Thresholding ---
    // (Used if algorithm is ThresholdAlgorithm::Global)
    // (Ignored if algorithm is ThresholdAlgorithm::Otsu, as Otsu calculates it)
    int globalThresholdValue = 127;

    // --- Parameters for Adaptive Thresholding ---
    // (Used if algorithm is ThresholdAlgorithm::AdaptiveMean or ThresholdAlgorithm::AdaptiveGaussian)
    int adaptiveBlockSize = 11;    // Size of the pixel neighborhood (must be odd, >=3).
    double adaptiveCValue = 2.0;   // Constant subtracted from the mean or weighted mean. Can be negative.

    // --- Pre-processing: Gaussian Blur ---
    bool enableBlur = false;        // Whether to apply Gaussian blur before thresholding.
    int blurKernelSize = 5;        // Kernel size for Gaussian blur (must be odd, >=3, e.g., 3, 5, 7).
        // Used if enableBlur is true.
    double blurSigmaX = 0.0;       // Gaussian kernel standard deviation in X direction.
        // If 0, it's calculated from blurKernelSize.
        // (sigmaY will also be 0 or calculated similarly).
        // Used if enableBlur is true.
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
