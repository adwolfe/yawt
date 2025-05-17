#ifndef TRACKINGCOMMON_H
#define TRACKINGCOMMON_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <QPointF>
#include <QRectF>
#include <QMetaEnum>
#include <vector> // For std::vector
#include <limits> // For std::numeric_limits
#include <QColor> // Added for TrackedItem color
#include <QString> // For typeToString and stringToType
#include <map>     // For AllWormTracks


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
    QColor color;      // Color associated with this worm
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
    QColor color;                   // Color for worm ROI and track
    ItemType type;                  // Type of the item
    QPointF initialCentroid;        // Centroid in video coordinates at selection
    QRectF initialBoundingBox;      // Bounding box in video coordinates at selection
    int frameOfSelection;           // Frame number where this item was selected
    // Add other relevant data as needed
};


/**
 * @brief Defines available thresholding algorithms.
 */
enum class ThresholdAlgorithm {
    Global,         // Simple global threshold
    Otsu,           // Otsu's binarization (auto global threshold)
    AdaptiveMean,   // Adaptive threshold using mean of neighborhood
    AdaptiveGaussian // Adaptive threshold using Gaussian weighted sum of neighborhood
};


/**
 * @brief Structure to hold parameters for thresholding and pre-processing.
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

/**
     * @brief Structure to hold information about a detected blob.
     */
struct DetectedBlob {
    QPointF centroid;                     // Centroid of the blob in video coordinates
    QRectF boundingBox;                   // Bounding box of the blob in video coordinates
    double area = 0.0;                    // Area of the blob
    std::vector<cv::Point> contourPoints; // Raw contour points (in video coordinates)
    bool isValid = false;                 // Flag indicating if this blob data is valid

    // Default constructor
    DetectedBlob() : area(0.0), isValid(false) {}
};

/**
     * @brief Finds the blob in a binary image closest to a click point, or the one containing the click.
     * This function first looks for blobs whose bounding box contains the click.
     * If none are found, it then looks for the blob whose centroid is closest to the click,
     * within a specified maximum distance.
     * @param binaryImage The input 8-bit single-channel binary image (CV_8UC1).
     * Non-zero pixels are considered foreground.
     * @param clickPointVideoCoords The click coordinates in the same coordinate system as the binaryImage.
     * @param minArea Minimum contour area to be considered a valid blob.
     * @param maxArea Maximum contour area to be considered a valid blob.
     * @param maxDistanceForSelection Max distance (in pixels) from click to a blob's centroid
     * if the click is not inside any blob's bounding box.
     * @return DetectedBlob structure. Check DetectedBlob::isValid to see if a suitable blob was found.
     */
DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea = 5.0,
                             double maxArea = 10000.0, // Added maxArea
                             double maxDistanceForSelection = 30.0);

/**
     * @brief Finds all plausible blobs within a given ROI of a binary image.
     * @param binaryImage The input 8-bit single-channel binary image (CV_8UC1).
     * @param roiToSearch The QRectF defining the region of interest in video coordinates.
     * @param minArea Minimum area for a blob to be considered.
     * @param maxArea Maximum area for a blob to be considered.
     * @param minAspectRatio Minimum aspect ratio (width/height or height/width, always >= 1).
     * @param maxAspectRatio Maximum aspect ratio.
     * @return QList of DetectedBlob structs for all plausible blobs found.
     */
QList<DetectedBlob> findAllPlausibleBlobsInRoi(const cv::Mat& binaryImage,
                                               const QRectF& roiToSearch,
                                               double minArea,
                                               double maxArea,
                                               double minAspectRatio,
                                               double maxAspectRatio);

} // namespace TrackingHelper

namespace TrackingConstants {
constexpr double DEFAULT_MIN_WORM_AREA = 10.0;
constexpr double DEFAULT_MAX_WORM_AREA = 1000.0; // Adjust as needed
constexpr double MERGE_AREA_FACTOR = 1.5; // Factor to determine if a blob is likely merged
const double DEFAULT_MIN_ASPECT_RATIO = 0.1; // e.g. long thin objects
const double DEFAULT_MAX_ASPECT_RATIO = 10.0;
}


#endif // TRACKINGCOMMON_H
