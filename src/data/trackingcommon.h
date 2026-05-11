#ifndef TRACKINGCOMMON_H
#define TRACKINGCOMMON_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <QPointF>
#include <QRectF>
#include <QMetaEnum>
#include <vector> // For std::vector
#include <limits> // For std::numeric_limits
#include <cmath>  // For std::sqrt (TipFeatureBaseline accessors)
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
    ROI,
    StartPoint,
    EndPoint,
    ControlPoint,
    CenterPoint,
    Fix, // For manual retracking of lost worms
    Undefined // Default or unassigned
};

// Helper functions to convert ItemType to/from QString for display and editing
inline QString itemTypeToString(ItemType type) {
    switch (type) {
    case ItemType::Worm: return "Worm";
    case ItemType::ROI: return "ROI";
    case ItemType::StartPoint: return "Start Point";
    case ItemType::EndPoint: return "End Point";
    case ItemType::ControlPoint: return "Control Point";
    case ItemType::CenterPoint: return "Center";
    case ItemType::Fix: return "Fix";
    case ItemType::Undefined: return "Undefined";
    default: return "Unknown";
    }
}

inline ItemType stringToItemType(const QString& typeStr) {
    if (typeStr == "Worm") return ItemType::Worm;
    if (typeStr == "ROI") return ItemType::ROI;
    if (typeStr == "Start Point") return ItemType::StartPoint;
    if (typeStr == "End Point") return ItemType::EndPoint;
    if (typeStr == "Control Point") return ItemType::ControlPoint;
    if (typeStr == "Center") return ItemType::CenterPoint;
    if (typeStr == "Fix") return ItemType::Fix;
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
 * @brief A candidate "tip" point (probable nose or tail) on a blob's geometry.
 *
 * Produced as a pure preprocessing pass over each blob (no temporal state, no
 * scoring against previous frames). The downstream head/tail assignment step
 * consumes these candidates plus per-worm baselines + motion history.
 *
 * Two complementary sources:
 *   - SkeletonEndpoint: a degree-1 node of the Zhang-Suen skeleton, mapped to
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
    cv::Point2f point;                 // Position on outer contour, in video coordinates
    float       curvature = 0.f;       // Signed local curvature (1/px); +ve = bulging outward
    float       width     = 0.f;       // Local perpendicular mask thickness (px), ~5px inward from tip
    enum class Source : uint8_t {
        SkeletonEndpoint,
        CurvaturePeak,
        HypothesizedHidden   // D-3: inferred tip when the other end is occluded
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
 * Set by classifyTopology() once per blob per pass; consumed by the
 * centerline-refinement dispatch (Phase D) to pick D-1/D-2/D-3/D-4 branches.
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
 * @brief Running per-worm distribution of tip features, accumulated online.
 *
 * Fed only from frames where the worm's blob has clean topology — defined as:
 *   • no ring (blob.holeContourPoints empty),
 *   • the worm is not part of a multi-worm merge group at this frame,
 *   • findTipCandidates() returned exactly two SkeletonEndpoint candidates.
 *
 * Under those conditions both candidates are bona-fide tips of this individual,
 * so their (|curvature|, width) features sample the "what does a tip look like
 * on this worm" distribution. The centerline arc length on the same frame
 * samples a body-length distribution that downstream code (Phase D-3) uses as
 * a geodesic-length prior when only one tip is visible.
 *
 * Statistics use Welford's online algorithm: numerically stable across many
 * samples, single-pass, no need to retain individual sample values.
 *
 * Curvature is stored as |signed curvature| since outer-contour traversal
 * direction can vary between blobs; tip-vs-kink discrimination depends on
 * magnitude, not sign.
 */
struct TipFeatureBaseline {
    // Welford running stats: mean + sum-of-squared-deviations-from-mean (M2).
    float meanAbsCurvature = 0.f;
    float m2AbsCurvature   = 0.f;
    int   curvatureSamples = 0;

    float meanWidth   = 0.f;
    float m2Width     = 0.f;
    int   widthSamples = 0;

    float meanBodyLength = 0.f;
    float m2BodyLength   = 0.f;
    int   lengthSamples  = 0;

    void addCurvatureSample(float curvatureMagnitude) {
        ++curvatureSamples;
        const float delta  = curvatureMagnitude - meanAbsCurvature;
        meanAbsCurvature  += delta / static_cast<float>(curvatureSamples);
        const float delta2 = curvatureMagnitude - meanAbsCurvature;
        m2AbsCurvature    += delta * delta2;
    }
    void addWidthSample(float width) {
        ++widthSamples;
        const float delta  = width - meanWidth;
        meanWidth         += delta / static_cast<float>(widthSamples);
        const float delta2 = width - meanWidth;
        m2Width           += delta * delta2;
    }
    void addLengthSample(float length) {
        ++lengthSamples;
        const float delta  = length - meanBodyLength;
        meanBodyLength    += delta / static_cast<float>(lengthSamples);
        const float delta2 = length - meanBodyLength;
        m2BodyLength      += delta * delta2;
    }

    float curvatureStdDev() const {
        return (curvatureSamples > 1)
            ? std::sqrt(m2AbsCurvature / static_cast<float>(curvatureSamples - 1)) : 0.f;
    }
    float widthStdDev() const {
        return (widthSamples > 1)
            ? std::sqrt(m2Width / static_cast<float>(widthSamples - 1)) : 0.f;
    }
    float bodyLengthStdDev() const {
        return (lengthSamples > 1)
            ? std::sqrt(m2BodyLength / static_cast<float>(lengthSamples - 1)) : 0.f;
    }

    /** Considered usable as a discriminator when both feature dists have ≥30 samples. */
    bool isReliable(int minSamples = 30) const {
        return curvatureSamples >= minSamples && widthSamples >= minSamples;
    }
};

/**
     * @brief Structure to hold information about a detected blob during tracking.
     */

struct DetectedBlob {
    QPointF centroid;                     // Centroid of the blob in video coordinates
    QRectF boundingBox;                   // Bounding box of the blob in video coordinates
    double area = 0.0;                    // Area of the blob
    double convexHullArea = 0.0;          // Area of the convex hull (blob area without holes)
    std::vector<cv::Point> contourPoints;                   // Outer contour points (in video coordinates)
    std::vector<std::vector<cv::Point>> holeContourPoints;  // Inner hole contours (ring topology from coiled worm)
    std::vector<cv::Point2f> centerlinePoints;              // Ordered centerline points from one body end to the other
    cv::Point2f centerlineCutPoint;                         // Debug/overlay point where a ring mask was cut open
    bool hasCenterlineCutPoint = false;                     // True when centerlineCutPoint is meaningful
    std::vector<TipCandidate> tipCandidates;                // Per-frame nose/tail candidates (Phase B preprocessing)
    int assignedHeadTipIdx = -1;                            // Index into tipCandidates of the assigned head, or -1
    int assignedTailTipIdx = -1;                            // Index into tipCandidates of the assigned tail, or -1
    TopologyState topologyState = TopologyState::Unknown;   // Per-frame geometric classification (Phase C.2)
    bool isValid = false;                 // Flag indicating if this blob data is valid
    bool touchesROIboundary = false;      // Flag indicating if the ROI extends beyond the cropped region (suggests it is merged).

    // Default constructor
    DetectedBlob()
        : area(0.0),
          convexHullArea(0.0),
          centerlineCutPoint(0.f, 0.f),
          hasCenterlineCutPoint(false),
          assignedHeadTipIdx(-1),
          assignedTailTipIdx(-1),
          topologyState(TopologyState::Unknown),
          isValid(false),
          touchesROIboundary(false) {}
};

enum class TrackPointQuality {
    Single,     // Single (confident) tracking
    Merged,     // Part of a merged blob (ambiguous)
    Split,      // Frame where a split was detected/resolved (functionally like Single for display)
    Lost        // Tracking was lost - position data is invalid
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

/**
 * @brief Tunable parameters for the active-contour ("snake") centerline refinement
 *        used on ring/coiled and (eventually) merged frames.
 *
 * The snake initializes from the previous-frame centerline translated by
 * blob-centroid motion, then evolves under three competing forces:
 *   - tension  (alpha) — first-derivative penalty; resists stretching, smooths spacing.
 *   - rigidity (beta)  — second-derivative penalty; resists kinking/bending.
 *   - image    (lambda)— attraction toward the medial axis (ridge of the
 *                        distance transform of the current blob mask).
 *
 * Endpoints are pinned to the nearest outer-contour point of the predicted
 * nose/tail. Because the curve is a parametric polyline (not a skeleton), it
 * is allowed to self-intersect — which is exactly what's needed when a worm
 * physically crosses over itself in 2D.
 */
struct CenterlineSnakeParams {
    bool   enabled    = true;   // Master toggle. When false, fall back to the legacy cut-skeleton path.
    double alpha      = 0.10;   // Tension (1st-derivative weight).
    double beta       = 0.05;   // Rigidity (2nd-derivative weight).
    double lambda     = 1.00;   // Image attraction (toward distance-transform ridge).
    int    iterations = 60;     // Number of explicit-Euler update steps.
    double stepSize   = 0.50;   // Per-iteration step size (tau).
    int    nPoints    = 20;     // Number of resampled points along the centerline.
                                // More points reduce kinking on highly curved bodies
                                // at the cost of slightly more computation.

    // ── Orientation consistency (right-hand rule) ─────────────────────────
    // The total signed turning angle of a polyline (∫κ ds, discrete sum of
    // signed inter-segment angles) is negated when traversal direction is
    // reversed. We track this quantity from frame to frame and veto any
    // candidate whose sign disagrees with the previous frame's sign — i.e.
    // we enforce that the centerline is always traversed nose→tail in the
    // same rotational sense (CCW or CW).
    //
    // When the worm is nearly straight the turning angle is close to zero
    // and the orientation is geometrically ambiguous; the veto is skipped
    // for frames whose |angle| < orientationAngleThreshold (radians).
    // 0.5 rad ≈ 29° is a reasonable starting point: it activates once the
    // worm bends noticeably but stays quiet on straight-body frames.
    double orientationAngleThreshold = 0.50;  // radians
};

/**
 * @brief Carrier for the per-worm temporal state assignHeadTail() needs.
 *
 * Maintained by the caller across frames; updated after each successful
 * assignment. Encodes "where was the head last frame, how fast is it
 * moving, what's the spatial scale of this body" — none of which lives
 * on DetectedBlob (per-frame) or on TipFeatureBaseline (no spatial info).
 */
struct HeadTailPredictor {
    bool        hasPrev      = false;     // Set once a successful assignment has been made.
    cv::Point2f lastHeadPos  = {0.f, 0.f};
    cv::Point2f lastTailPos  = {0.f, 0.f};
    bool        hasVelocity  = false;     // True after two consecutive successful assignments.
    cv::Point2f velHead      = {0.f, 0.f};
    cv::Point2f velTail      = {0.f, 0.f};
    float       refDistance  = 30.f;      // Normalising scale for the distance term (~½ body length).
};

/**
 * @brief Classify a blob's geometric topology for downstream dispatch.
 *
 * Pure function; depends only on the blob's contents and the caller-supplied
 * merge-group flag. The classification rules are:
 *   • !blob.isValid              → Lost
 *   • inMergeGroup                → Merged
 *   • blob.holeContourPoints non-empty
 *     OR  <2 skeleton-source tip candidates → SelfCrossed
 *   • otherwise                   → Clean
 *
 * `inMergeGroup` is the worm-vs-other multi-occupant flag the caller derives
 * by querying TrackingDataStorage::getMergeGroupsForFrame(). This function
 * doesn't depend on TrackingDataStorage so it stays usable from anywhere.
 *
 * The result is written into `blob.topologyState` and returned.
 */
TopologyState classifyTopology(DetectedBlob& blob, bool inMergeGroup);

/**
 * @brief Assign head and tail roles to tip candidates on a blob.
 *
 * Pure function: depends only on `blob.tipCandidates`, the temporal predictor
 * supplied by the caller, and the worm's accumulated tip-feature baseline.
 * Writes its result into `blob.assignedHeadTipIdx` and `blob.assignedTailTipIdx`
 * (indices into `blob.tipCandidates`, or -1 if no candidate was assigned to
 * that role).
 *
 * Only skeleton-endpoint candidates are eligible for visible head/tail
 * assignment. Curvature peaks remain available for diagnostics and geometry
 * cues, but they are not treated as free endpoints.
 *
 * The cost minimised per candidate × role is a weighted sum of three terms:
 *   • distance from this candidate to the role's predicted position;
 *   • Mahalanobis-style |curvature| distance from the baseline;
 *   • Mahalanobis-style width distance from the baseline.
 * On self-crossed frames with exactly one skeleton endpoint, velocity is used
 * as a visibility gate instead of an angular score: if a role's predicted
 * position has moved into the current blob and is no longer close to a free
 * endpoint, that role is left unassigned for the D-3 hidden-tip pass.
 * The feature terms are disabled when the baseline isn't yet reliable
 * (n < 30 samples in either dist), so on the very first frames of a worm
 * the assignment falls through to distance alone.
 *
 * The keyframe (no `prev`) is bootstrapped separately by the caller —
 * `assignHeadTail` requires `predictor.hasPrev == true`.
 *
 * @return True when at least one role was assigned. False on degenerate input
 *         (no candidates, or no prev state available).
 */
bool assignHeadTail(DetectedBlob& blob,
                    const HeadTailPredictor& predictor,
                    const TipFeatureBaseline& baseline);

/**
 * @brief Detect candidate nose/tail (tip) points on a blob.
 *
 * Pure function: depends only on `blob.contourPoints` and `blob.holeContourPoints`.
 * Populates `blob.tipCandidates` (replacing any prior contents) and returns the
 * same vector for convenience.
 *
 * The two sources (skeleton degree-1 endpoints, outer-contour curvature peaks)
 * are merged; candidates within `mergePlanarDistancePx` of each other in the
 * image plane are deduplicated, with SkeletonEndpoint taking precedence over
 * CurvaturePeak when both surface the same physical tip.
 *
 * Per-candidate features:
 *   - `curvature`: signed local curvature along the outer contour, in a window
 *     of ±`curvatureWindow` contour points. Sign convention matches the
 *     contour's traversal direction; magnitude is the meaningful quantity for
 *     scoring against a per-worm baseline.
 *   - `width`: 2× the distance-transform value sampled ~5px inward from the
 *     candidate along the inward direction (toward the blob centroid). Gives
 *     a robust local body-thickness estimate without needing an explicit
 *     normal direction.
 *
 * @param blob                     Blob to analyse; `tipCandidates` is overwritten.
 * @param curvatureWindow          Half-window (in contour points) for curvature.
 * @param minCurvatureMagnitude    Threshold on |curvature| for a peak to qualify.
 * @param mergePlanarDistancePx    Two candidates within this distance are merged.
 * @return Const reference to `blob.tipCandidates`.
 */
const std::vector<TipCandidate>& findTipCandidates(DetectedBlob& blob,
                                                   int   curvatureWindow      = 5,
                                                   float minCurvatureMagnitude = 0.08f,
                                                   float mergePlanarDistancePx = 4.0f);

/**
 * @brief Compute and store an ordered centerline for a detected blob from its contour.
 * The resulting polyline runs from one skeleton endpoint to the other and is stored in
 * DetectedBlob::centerlinePoints. Returns true when a usable centerline was found.
 */
bool populateCenterlineFromContour(DetectedBlob& blob);

/**
 * @brief Compute a centerline after cutting a ring blob mask open.
 * @param blob Detected blob with ring topology. Updated with the best ordered centerline.
 * @param cutStart First endpoint of the cut line in video coordinates.
 * @param cutEnd Second endpoint of the cut line in video coordinates.
 * @param cutThickness Thickness of the erased cut line in pixels.
 * @return True when a usable centerline was found after applying the cut.
 */
bool populateCenterlineFromContourWithCut(DetectedBlob& blob,
                                          const cv::Point2f& cutStart,
                                          const cv::Point2f& cutEnd,
                                          int cutThickness = 3);

/**
 * @brief Extract an ordered skeleton centerline from a detected worm blob.
 * @param blob Detected blob with contour points in video coordinates.
 * @return Ordered centerline points in video coordinates. Empty if no valid skeleton can be extracted.
 */
QList<QPointF> extractOrderedCenterlinePoints(const DetectedBlob& blob);

/**
 * @brief Resample an ordered centerline to a fixed number of evenly spaced points.
 * @param points Ordered source centerline points.
 * @param pointCount Number of points to return.
 * @return Exactly pointCount points when input is non-empty; empty if input is empty or pointCount <= 0.
 */
QList<QPointF> resampleCenterlinePoints(const QList<QPointF>& points, int pointCount);

/**
 * @brief Resample an ordered centerline to a fixed number of evenly spaced points.
 * @param points Ordered source centerline points.
 * @param pointCount Number of points to return.
 * @return Exactly pointCount points when input is non-empty; empty if input is empty or pointCount <= 0.
 */
QList<QPointF> resampleCenterlinePoints(const std::vector<cv::Point2f>& points, int pointCount);

/**
 * @brief Extract and resample a detected blob centerline.
 * @param blob Detected blob with contour points in video coordinates.
 * @param pointCount Number of centerline points to return.
 * @return Fixed-count centerline points when extraction succeeds; otherwise empty.
 */
QList<QPointF> extractResampledCenterlinePoints(const DetectedBlob& blob, int pointCount = 10);

} // namespace TrackingHelper



namespace TrackingConstants {
constexpr double DEFAULT_MIN_WORM_AREA = 10.0;
constexpr double DEFAULT_MAX_WORM_AREA = 1000.0; // Adjust as needed
constexpr double MERGE_AREA_FACTOR = 1.5; // Factor to determine if a blob is likely merged
const double DEFAULT_MIN_ASPECT_RATIO = 0.1; // e.g. long thin objects
const double DEFAULT_MAX_ASPECT_RATIO = 10.0;
}


#endif // TRACKINGCOMMON_H
