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
//class WormObject; // If WormObject needs to be referenced here, though likely not for these structs


namespace TableItems {
// These are used within the BlobTableModel, for user interaction with blobs prior to tracking.

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
struct ClickedItem {
    int id;                         // Unique auto-generated ID
    QColor color;                   // Color for worm ROI and track
    ItemType type;                  // Type of the item
    QPointF initialCentroid;        // Centroid in video coordinates at selection
    QRectF initialBoundingBox;      // Bounding box in video coordinates at selection. THIS WILL BECOME THE STANDARDIZED ROI.
    QRectF originalClickedBoundingBox; // The actual bounding box of the blob when it was clicked. Used for metrics.
    int frameOfSelection;           // Frame number where this item was selected
    bool visible = true;            // Whether this item's track/ROI should be displayed
    // Add other relevant data as needed
};

} // namespace TableItems

namespace Thresholding {

/**
 * @brief Defines available thresholding algorithms.
 */
enum class ThresholdAlgorithm {
    Global,             // Simple global threshold
    Otsu,               // Otsu's binarization (auto global threshold)
    AdaptiveMean,       // Adaptive threshold using mean of neighborhood
    AdaptiveGaussian     // Adaptive threshold using Gaussian weighted sum of neighborhood
};

inline QString algoToString(ThresholdAlgorithm algo) {
    switch (algo) {
    case ThresholdAlgorithm::Global: return "Global threshold [static]";
    case ThresholdAlgorithm::Otsu: return "Global threshold [automatic]";
    case ThresholdAlgorithm::AdaptiveMean: return "Adaptive threshold [Mean-weighted]";
    case ThresholdAlgorithm::AdaptiveGaussian: return "Adaptive threshold [Gaussian-weighted]";
    default: return "Unknown";
    }
}

/**
 * @brief Structure to hold parameters for thresholding and pre-processing.
 */
struct ThresholdSettings {
    // General setting for interpreting pixel values (background vs. foreground)
    bool assumeLightBackground = true;

    // Main algorithm choice for thresholding.
    ThresholdAlgorithm algorithm = ThresholdAlgorithm::Global;

    // --- Parameters for Global Thresholding ---
    int globalThresholdValue = 90;

    // --- Parameters for Adaptive Thresholding ---
    int adaptiveBlockSize = 3;    // Must be odd, >=3.
    double adaptiveCValue = 0.0;   // Constant subtracted from the mean/weighted mean.

    // --- Pre-processing: Gaussian Blur ---
    bool enableBlur = false;        // Whether to apply Gaussian blur before thresholding.
    int blurKernelSize = 3;        // Must be odd, >=3.
    double blurSigmaX = 0.0;       // 0 for auto calculation from kernel size.
};

} // namespace Thresholding

namespace Tracking {

Q_NAMESPACE

// Helper function to calculate squared Euclidean distance
static double sqDistance(const QPointF& p1, const QPointF& p2) {
    QPointF diff = p1 - p2;
    return QPointF::dotProduct(diff, diff);
}

// Overload for cv::Point2f
static double sqDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    cv::Point2f diff = p1 - p2;
    return diff.dot(diff); // cv::Point2f::dot returns float, implicitly convertible to double
}

// You could add one for cv::Point2d as well if needed
static double sqDistance(const cv::Point2d& p1, const cv::Point2d& p2) {
    cv::Point2d diff = p1 - p2;
    return diff.dot(diff); // cv::Point2d::dot returns double
}

enum TrackerState {
    Idle,                           // Not yet started or stopped
    TrackingSingle,                 // Confidently tracking one target
    TrackingMerged,                 // Believed to be tracking our worm as part of a merged entity
    PausedForSplit,                 // Detected a split from a merged state and is waiting for TrackingManager
    TrackingLost                    // Optional: If tracking is definitively lost and cannot recover
};
Q_ENUM_NS(TrackerState)



/**
     * @brief Structure to hold information about a detected blob during tracking.
     */

struct DetectedBlob {
    QPointF centroid;                     // Centroid of the blob in video coordinates
    QRectF boundingBox;                   // Bounding box of the blob in video coordinates
    double area = 0.0;                    // Area of the blob
    double convexHullArea = 0.0;          // Area of the convex hull (blob area without holes)
    std::vector<cv::Point> contourPoints; // Raw contour points (in video coordinates)
    bool isValid = false;                 // Flag indicating if this blob data is valid
    bool touchesROIboundary = false;      // Flag indicating if the ROI extends beyond the cropped region (suggests it is merged).

    // Default constructor
    DetectedBlob() : area(0.0), convexHullArea(0.0), isValid(false), touchesROIboundary(false) {}
};

enum class TrackPointQuality {
    Confident,
    Ambiguous

};
Q_ENUM_NS(TrackPointQuality)

/**
 * @brief Represents a single point in a worm's track.
 */
struct WormTrackPoint {
    int frameNumberOriginal;        // Frame number in the original video
    cv::Point2f position;           // Position (centroid) in video coordinates
    QRectF roi;                     // ROI used for this worm at this frame (in video coordinates)
    TrackPointQuality quality;      // Single is confident, merged is ambiguous. For visualization later.
    // Can add behavior annotations later.
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
