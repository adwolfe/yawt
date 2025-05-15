#ifndef TRACKINGCOMMON_H
#define TRACKINGCOMMON_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <QPointF>
#include <QRectF>
#include <QMetaEnum>
#include <vector> // For std::vector
#include <limits> // For std::numeric_limits
#include <QColor>
#include <QString> // For typeToString and stringToType
#include <vector>
#include <map>


// This header defines types and structures common to video loading,
// processing, and tracking to avoid circular dependencies.

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


// Enum for the type of tracked item
enum class ItemType {
    Worm,
    StartPoint,
    EndPoint,
    ControlPoint,
    Undefined // Default or unassigned
};

// Helper functions to convert ItemType to/from QString for display and editing
inline QString itemTypeToString(ItemType type) {
    switch (type) {
    case ItemType::Worm: return "Worm";
    case ItemType::StartPoint: return "Start Point";
    case ItemType::EndPoint: return "End Point";
    case ItemType::ControlPoint: return "Control Point";
    case ItemType::Undefined: return "Undefined";
    default: return "Unknown";
    }
}

inline ItemType stringToItemType(const QString& typeStr) {
    if (typeStr == "Worm") return ItemType::Worm;
    if (typeStr == "Start Point") return ItemType::StartPoint;
    if (typeStr == "End Point") return ItemType::EndPoint;
    if (typeStr == "Control Point") return ItemType::ControlPoint;
    return ItemType::Undefined;
}

// Structure to hold data for each item in the table
struct TrackedItem {
    int id;                         // Unique auto-generated ID
    //QColor color;                   // Color for worm ROI
    ItemType type;                  // Type of the item
    QPointF initialCentroid;        // Centroid in video coordinates at selection
    QRectF initialBoundingBox;      // Bounding box in video coordinates at selection
    int frameOfSelection;           // Frame number where this item was selected
    // Add other relevant data as needed, e.g., color for display
};


/**
 * @brief Defines available thresholding algorithms.
 * (Moved from videoloader.h)
 */

enum class ThresholdAlgorithm {
    Global,         // Simple global threshold
    Otsu,           // Otsu's binarization (auto global threshold)
    AdaptiveMean,   // Adaptive threshold using mean of neighborhood
    AdaptiveGaussian // Adaptive threshold using Gaussian weighted sum of neighborhood
};


/**
 * @brief Structure to hold parameters for thresholding and pre-processing.
 * (Moved from TrackData.h, uses ThresholdAlgorithm defined above)
 */
struct ThresholdSettings {
    // General setting for interpreting pixel values (background vs. foreground)
    bool assumeLightBackground = true;

    // Main algorithm choice for thresholding.
    ThresholdAlgorithm algorithm = ThresholdAlgorithm::Global;

    // --- Parameters for Global Thresholding ---
    int globalThresholdValue = 127;

    // --- Parameters for Adaptive Thresholding ---
    int adaptiveBlockSize = 3;    // Must be odd, >=3.
    double adaptiveCValue = 0.0;   // Constant subtracted from the mean/weighted mean.

    // --- Pre-processing: Gaussian Blur ---
    bool enableBlur = false;        // Whether to apply Gaussian blur before thresholding.
    int blurKernelSize = 3;        // Must be odd, >=3.
    double blurSigmaX = 0.0;       // 0 for auto calculation from kernel size.
};



namespace TrackingHelper {

struct DetectedBlob {
    QPointF centroid;
    QRectF boundingBox;
    double area = 0.0;
    std::vector<cv::Point> contourPoints; // Store the actual contour points
    bool isValid = false;
};

/**
     * @brief Finds the blob in a binary image closest to a click point.
     * @param binaryImage The input 8-bit single-channel binary image (CV_8UC1). Non-zero pixels are foreground.
     * @param clickPointVideoCoords The click coordinates in the same space as the binaryImage.
     * @param minArea Minimum contour area to be considered a valid blob.
     * @param maxDistanceForSelection Max distance from click to a blob's centroid if click is not inside any bounding box.
     * @return DetectedBlob structure. Check DetectedBlob::isValid.
     */
DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea = 5.0,
                             double maxDistanceForSelection = 30.0); // Pixels

} // namespace TrackingHelper


#endif // TRACKINGCOMMON_H
