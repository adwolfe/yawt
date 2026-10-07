#ifndef TRACKINGCOMMON_H
#define TRACKINGCOMMON_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/geometry.hpp>
#include <QPointF>
#include <QRectF>
#include <QList>
#include <QMetaEnum>
#include <vector> // For std::vector
#include <limits> // For std::numeric_limits
#include <cmath>  // For std::sqrt (TipFeatureBaseline accessors)
#include <QColor> // For AnnotationItem color
#include <QString> // For typeToString and stringToType
#include <QJsonObject>
#include <map>     // For AllWormTracks


// This header defines types and structures common to video loading,
// processing, and tracking to avoid circular dependencies.
//
// Coordinate frames used throughout YAWT
// --------------------------------------
//  video coordinates   Pixels of the source video frame: origin top-left, x right, y down.
//                      Every position stored on a DetectedBlob, TrackPoint, AnnotationItem
//                      or reference point is in this frame. This is the only frame that is
//                      persisted.
//  local coordinates   Pixels relative to a blob's padded bounding box
//                      (Centerline::EndpointResult::localBounds). Used only inside the
//                      skeleton and distance-transform buffers; add the box origin to
//                      convert back to video coordinates.
//  widget coordinates  Pixels of a Qt widget (VideoLoader, MiniLoader). Transient.
//  crop coordinates    Pixels of MiniLoader's cropped image, offset from video coordinates
//                      by the crop origin. Transient.

// Forward declaration


namespace TableItems {
// Annotations: the items the user marks on the video (worms to track, ROI rectangles,
// Start/End/Center reference points). Shown in the annotation tables and persisted in
// worms.json / roi_points.json.

// Enum for the type of tracked item
enum class ItemType {
    Worm,
    Region,      // a user-drawn rectangle; not tracked (called "ROI" in files written before this rename)
    StartPoint,
    EndPoint,
    CenterPoint,
    Undefined // Default or unassigned
};

// Helper functions to convert ItemType to/from QString for display and editing
inline QString itemTypeToString(ItemType type) {
    switch (type) {
    case ItemType::Worm: return "Worm";
    case ItemType::Region: return "Region";
    case ItemType::StartPoint: return "Start Point";
    case ItemType::EndPoint: return "End Point";
    case ItemType::CenterPoint: return "Center";
    case ItemType::Undefined: return "Undefined";
    default: return "Unknown";
    }
}

inline ItemType stringToItemType(const QString& typeStr) {
    if (typeStr == "Worm") return ItemType::Worm;
    if (typeStr == "Region" || typeStr == "ROI") return ItemType::Region;   // "ROI" is the legacy name
    if (typeStr == "Start Point") return ItemType::StartPoint;
    if (typeStr == "End Point") return ItemType::EndPoint;
    if (typeStr == "Control Point") return ItemType::CenterPoint; // legacy name for the center reference point
    if (typeStr == "Center") return ItemType::CenterPoint;
    if (typeStr == "Fix") return ItemType::Worm;                // legacy: Fix items were tracked worms
    return ItemType::Undefined;
}

// One user annotation. Called a "blob" in older code because worms are picked by clicking a blob.
struct AnnotationItem {
    int id;                         // Unique auto-generated ID
    QColor color;                   // Color for worm ROI and track
    ItemType type;                  // Type of the item
    QPointF initialCentroid;        // Centroid in video coordinates at selection
    QRectF initialBoundingBox;      // Bounding box in video coordinates at selection. THIS WILL BECOME THE STANDARDIZED ROI.
    QRectF originalClickedBoundingBox; // The actual bounding box of the blob when it was clicked. Used for metrics.
    int frameOfSelection;           // Frame the item was placed on. For a Worm this is its KEYFRAME: tracking
                                    // runs outward from it, and every worm in one run must share it
                                    // (AppController::validateAndGetSharedKeyframe). Worms added later can be
                                    // tracked from a different frame in a later run. For reference items it is
                                    // informational only.
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
    // The model is transient, immutable, and shared with processing workers.
    bool enableBackgroundSubtraction = false;
    cv::Mat medianBackground;

    bool enableBlur = false;        // Whether to apply Gaussian blur before thresholding.
    int blurKernelSize = 3;        // Must be odd, >=3.
    double blurSigmaX = 0.0;       // 0 for auto calculation from kernel size.
};

} // namespace Thresholding

namespace Tracking {

Q_NAMESPACE

// Helper function to calculate squared Euclidean distance
inline double sqDistance(const QPointF& p1, const QPointF& p2) {
    QPointF diff = p1 - p2;
    return QPointF::dotProduct(diff, diff);
}

// Overload for cv::Point2f
inline double sqDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    cv::Point2f diff = p1 - p2;
    return diff.dot(diff); // cv::Point2f::dot returns float, implicitly convertible to double
}

// Overload for cv::Point2d
inline double sqDistance(const cv::Point2d& p1, const cv::Point2d& p2) {
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
 * @brief A candidate "tip" point (probable head or tail) on a blob's geometry.
 *
 * Produced as a pure preprocessing pass over each blob (no temporal state, no
 * scoring against previous frames). The downstream head/tail assignment step
 * consumes these candidates plus per-worm baselines + motion history.
 *
 * Two complementary sources:
 *   - SkeletonEndpoint: a degree-1 node of the Guo-Hall skeleton, mapped to
 *     the nearest outer-contour point. Reliable on clean topology (typically
 *     yields exactly 2 endpoints); often yields 1 on coiled-with-protrusion
 *     blobs and 0 on closed-ring blobs.
 *   - CurvaturePeak: a local maximum of |signed curvature| along the outer
 *     contour. Surfaces real tips even when the skeleton is degenerate (rings,
 *     merged blobs), at the cost of more false positives (tight body kinks).
 *
 * Candidates from both sources are merged and deduplicated by planar distance.
 */
struct TipCandidate {
    cv::Point2f point;                 // Selected tip position, in video coordinates
    float       curvature = 0.f;       // Signed local curvature (1/px); +ve = bulging outward
    float       width     = 0.f;       // Local perpendicular mask thickness (px), ~5px inward from tip
    enum class Source : uint8_t {
        SkeletonEndpoint,
        CurvaturePeak,
        HypothesizedHidden,  // S-1: inferred end position when that end is not visible
        BilateralCap,       // legacy value retained for compatibility
        AxisBoundary       // terminal body-axis intersection with contour
    };
    Source source = Source::SkeletonEndpoint;
};

/**
 * @brief Per-frame topology classification for a worm's blob.
 *
 * Independent of (but adjacent to) TrackPointQuality, which captures the
 * tracker's *confidence*. TopologyState captures the *geometric* state of
 * the worm's blob: clean topology with both tips visible, self-crossed
 * (ring or hidden tip), shared with another worm (merged), or unavailable.
 *
 * Set by detectEndpoints() once per blob per pass; consumed by the
 * centerline dispatch to pick D-1, S-1, or D-4.
 */
enum class TopologyState : uint8_t {
    Unknown,        // Default, before classification.
    Clean,          // No ring, sole occupant of its blob, both tips skeleton-confirmed.
    SelfCrossed,    // Ring topology OR fewer than two skeleton tips visible.
    Merged,         // Shares its blob with one or more other worms.
    Lost            // No valid blob this frame.
};

inline QString topologyStateToString(TopologyState s) {
    switch (s) {
    case TopologyState::Clean:       return QStringLiteral("Clean");
    case TopologyState::SelfCrossed: return QStringLiteral("SelfCrossed");
    case TopologyState::Merged:      return QStringLiteral("Merged");
    case TopologyState::Lost:        return QStringLiteral("Lost");
    case TopologyState::Unknown:
    default:                         return QStringLiteral("Unknown");
    }
}

/**
 * @brief Results of the post-tracking centerline pass for one blob.
 *
 * Empty until CenterlineWorker has run (see docs/centerline_pipeline.md). A
 * rerun of the pass replaces this wholesale without touching the detection-time
 * geometry on DetectedBlob, and it is persisted as its own "centerline" object
 * per track point.
 */
struct BlobCenterline {
    std::vector<cv::Point2f> points;          // Ordered head -> tail, video coordinates
    cv::Point2f cutPoint{0.f, 0.f};           // Debug/overlay: where a ring mask was cut open
    bool hasCutPoint = false;                 // True when cutPoint is meaningful
    std::vector<TipCandidate> tipCandidates;  // Per-frame head/tail candidates (Phase B)
    int headTipIdx = -1;                      // Index into tipCandidates of the assigned head, or -1
    int tailTipIdx = -1;                      // Index into tipCandidates of the assigned tail, or -1
    TopologyState topology = TopologyState::Unknown;  // Per-frame geometric classification (Phase C.2)
    bool needsReview = false;                 // Flagged by the centerline pass for human assessment
    QString reviewReason;                     // Why it was flagged; empty when not flagged

    bool isEmpty() const {
        return points.empty() && tipCandidates.empty() && topology == TopologyState::Unknown;
    }
};

/**
 * @brief One connected component on one thresholded frame.
 *
 * Everything except `centerline` is detection-time geometry, known as soon as
 * the tracker finds the blob. `centerline` is filled later by the centerline
 * pass and is serialised separately.
 */
struct DetectedBlob {
    QPointF centroid;                     // Centroid of the blob in video coordinates
    QRectF boundingBox;                   // Bounding box of the blob in video coordinates
    double area = 0.0;                    // Area of the blob
    double convexHullArea = 0.0;          // Area of the convex hull (blob area without holes)
    std::vector<cv::Point> contourPoints;                   // Outer contour points (in video coordinates)
    std::vector<std::vector<cv::Point>> holeContourPoints;  // Inner hole contours (ring topology from coiled worm)
    bool isValid = false;                 // Flag indicating if this blob data is valid
    bool touchesSearchWindow = false;     // Blob touches the edge of the search window (suggests it continues outside, i.e. merged or partially visible).

    BlobCenterline centerline;            // Centerline-pass results; empty until the pass has run
};

// Detection-time geometry only (everything but `centerline`).
QJsonObject blobGeometryToJson(const DetectedBlob& blob);
void        blobGeometryFromJson(const QJsonObject& obj, DetectedBlob& blob);
// Centerline-pass results only.
QJsonObject    blobCenterlineToJson(const BlobCenterline& cl);
BlobCenterline blobCenterlineFromJson(const QJsonObject& obj);
// Combined single-object layout (geometry + centerline keys). Still written for
// TrackingManager's mergeState section and read for pre-split worms.json files.
QJsonObject detectedBlobToJson(const DetectedBlob& blob);
DetectedBlob detectedBlobFromJson(const QJsonObject& obj);

/**
 * @brief Persisted per-track-point label describing how confidently the point was tracked.
 *
 * THE INTEGER VALUES ARE A PUBLIC CONTRACT. They are written to worms.json as
 * "quality" and exposed to analysis plugins as the `quality` variable and the
 * constants Single = 0, Merged = 1, Split = 2, Lost = 3 (see
 * docs/plugin_reference.md and PluginEngine). Never reorder or renumber these
 * members; append new ones at the end.
 *
 * A point gets its label from qualityForFrame() below, which is the only
 * place the tracker's live state is mapped to a persisted label:
 *   - Single: a valid blob was found while the tracker state was TrackingSingle.
 *   - Merged: a valid blob was found in any other tracker state (TrackingMerged,
 *             PausedForSplit). The position is the shared blob's centroid, so it
 *             is ambiguous.
 *   - Split:  the frame on which a split was resolved and this worm was assigned
 *             one of the resulting blobs. Functionally like Single for display.
 *   - Lost:   no valid blob. `position` is a (0,0) placeholder and must not be
 *             used; consumers should filter on quality != Lost.
 */
enum class TrackPointQuality {
    Single = 0,
    Merged = 1,
    Split  = 2,
    Lost   = 3
};

/**
 * @brief Map a tracker's per-frame report to the persisted TrackPointQuality.
 *
 * This is the single definition of the relationship between the three state
 * vocabularies: TrackerState (live tracker belief), TrackPointQuality (persisted
 * per-point label) and, indirectly, the blob's TopologyState, which the
 * centerline pass derives later and which does not feed back into quality.
 *
 * @param state                  Tracker state when the frame was reported.
 * @param hasValidBlob           Whether the tracker found a blob to anchor the point.
 * @param splitResolvedThisFrame True on the frame where TrackingManager resolved a
 *                               split and assigned this worm one of the pieces.
 */
inline TrackPointQuality qualityForFrame(TrackerState state,
                                         bool hasValidBlob,
                                         bool splitResolvedThisFrame = false)
{
    if (!hasValidBlob) return TrackPointQuality::Lost;
    if (splitResolvedThisFrame) return TrackPointQuality::Split;
    return (state == TrackerState::TrackingSingle) ? TrackPointQuality::Single
                                                   : TrackPointQuality::Merged;
}
Q_ENUM_NS(TrackPointQuality)

/**
 * @brief Represents a single point in a worm's track.
 */
struct TrackPoint {
    int frameNumber;              // Absolute frame index in the source video
    cv::Point2f position;           // Position (centroid) in video coordinates
    QRectF searchWindow;   // Fixed-size search window the tracker used on this frame (video coordinates)
    TrackPointQuality quality;      // Single is confident, merged is ambiguous. For visualization later.

    // Blob-derived morphology — populated post-centerline; 0/invalid if unavailable.
    float area        = 0.f;        // Blob area in pixels²
    float aspectRatio = 0.f;        // Bounding-box aspect ratio (long/short side, >= 1)
    float bodyLength  = 0.f;        // Arc length of centerline in pixels
    cv::Point2f headTip{};          // Assigned head tip position in video coordinates
    cv::Point2f tailTip{};          // Assigned tail tip position in video coordinates
    bool hasTips      = false;      // True when headTip/tailTip are valid
};

/**
 * @brief The worm IDs that share one blob on one frame.
 * Per-frame merge history is QList<MergeGroup>; persisted as mergeGroupsByFrame.
 */
using MergeGroup = QList<int>;

/** @brief One worm's track: its points in ascending frame order. */
using Track = std::vector<TrackPoint>;

/**
 * @brief All tracks of a run, keyed by worm ID.
 */
typedef std::map<int, Track> AllWormTracks;


/**
 * @brief Structure to pass initial information about a worm to be tracked.
 */
struct InitialWormInfo {
    int id;
    QRectF initialSearchWindow; // Fixed-size search window centred on the worm at the keyframe (video coordinates)
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

} // namespace Tracking



namespace TrackingConstants {
constexpr double DEFAULT_MIN_WORM_AREA = 10.0;
constexpr double DEFAULT_MAX_WORM_AREA = 1000.0; // Adjust as needed
constexpr double MERGE_AREA_FACTOR = 1.5; // Factor to determine if a blob is likely merged
const double DEFAULT_MIN_ASPECT_RATIO = 0.1; // e.g. long thin objects
const double DEFAULT_MAX_ASPECT_RATIO = 10.0;
}


#endif // TRACKINGCOMMON_H
