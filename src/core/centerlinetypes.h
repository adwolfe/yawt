#ifndef CENTERLINETYPES_H
#define CENTERLINETYPES_H

#include "../data/trackingcommon.h"

#include <QMap>
#include <cmath>
#include <vector>

namespace Centerline {

/**
 * @brief Skeleton graph extracted from a binary worm mask.
 *
 * The skeleton is the Zhang-Suen 1-pixel-wide medial-axis representation;
 * each foreground pixel is a graph node, connected to its 8-neighbour
 * skeleton pixels (orthogonal weight 1, diagonal sqrt(2)). Endpoint indices
 * are the degree-1 nodes — body tips when the topology is unobstructed.
 *
 * `indexImage` maps mask coords to graph index (-1 outside skeleton).
 * Coordinates inside `points` and `indexImage` are LOCAL to the bounding
 * box used to build the source mask; the caller adds the box origin when
 * converting back to video coordinates.
 */
struct SkeletonGraph {
    std::vector<cv::Point>        points;          // mask-local coords
    std::vector<std::vector<int>> adjacency;
    std::vector<int>              endpointIndices; // degree-1 nodes (post-prune)
    cv::Mat                       indexImage;      // CV_32SC1: pixel -> index
    cv::Mat                       skeleton;        // CV_8UC1, 0 or 255
};

/**
 * @brief A high-confidence body tip — skeleton endpoint optionally extended
 *        to the matching outer-contour curvature peak.
 *
 * `point` is the best-estimate world position used for downstream geodesic
 * paths, snake pinning, and head/tail assignment. `skelPoint` is the raw
 * skeleton degree-1 node mapped to the nearest outer contour point, in
 * world coordinates (handy when the extension is disputed).
 *
 * `extended == true` iff `point` came from a curvature peak rather than the
 * skeleton-endpoint contour snap.
 */
struct TrueTip {
    cv::Point2f point;          // world coords; either curvature peak or skel pt
    cv::Point2f skelPoint;      // world coords; raw skeleton degree-1 node
    cv::Point2f bilateralTip;   // world coords; midpoint of left/right cap-wall apexes
    float       curvature    = 0.f;
    float       width        = 0.f;
    bool        extended     = false;
    bool        hasBilateral = false; // true when bilateralTip is valid
};

/**
 * @brief Per-tip debug snapshot of the bilateral cap midpoint computation.
 *
 * Populated inside detectEndpoints() for every skeleton endpoint and carried
 * on EndpointResult::tipCapDebug (parallel to EndpointResult::tips).
 * Consumed only by the debug exporter — zero cost in release when unused.
 *
 * All point coordinates are in WORLD (video) space.
 */
struct TipCapDebug {
    bool valid = false;

    // Search window parameters (mirroring what detectEndpoints used).
    cv::Point2f skelEndpoint;   // skeleton degree-1 node projected to contour snap
    cv::Point2f outwardDir;     // normalised direction away from body interior
    float dtAtEp    = 0.f;     // DT value at skeleton endpoint
    float maxForward = 0.f;    // forward search limit (pixels)
    float maxSide    = 0.f;    // lateral search limit (pixels)

    // Contour points classified by which half-plane they fall in.
    std::vector<cv::Point2f> leftCapPoints;    // fwd in window, left of outwardDir
    std::vector<cv::Point2f> rightCapPoints;   // fwd in window, right of outwardDir
    // (points outside the window are not stored — they are uncoloured on the canvas)

    // Apex weighted centroids from the top-forward fraction of each side.
    cv::Point2f leftApex;    // weighted centroid of top-forward left points
    cv::Point2f rightApex;   // weighted centroid of top-forward right points
    bool  hasLeft  = false;
    bool  hasRight = false;
    float leftPeakFwd  = 0.f;  // forward depth of the leftmost apex contour point
    float rightPeakFwd = 0.f;  // forward depth of the rightmost apex contour point
    bool  sanityPassed = false; // did the left/right apex depths agree well enough?

    // Bilateral result.
    cv::Point2f bilateralTip;
    bool hasBilateral = false;

    // Old-approach comparison points (for overlay in exporter).
    cv::Point2f snapPoint;       // projectedEndpointContourIdx snap (= t.skelPoint)
    cv::Point2f peakOrSnapPoint; // curvature peak if found, else same as snapPoint
    bool hadPeak = false;        // true when a curvature peak was accepted
};

/**
 * @brief Combined output of `detectEndpoints()`.
 *
 * Bundles the skeleton graph, distance transform, true tips, topology
 * classification, and head/tail role assignment. Designed to be computed
 * once per frame in the new centerline pipeline and reused for every
 * downstream step (centerline build, snake refinement, predictor update).
 *
 * Coordinates: `tips`, `headIdx`, `tailIdx` index into `tips`; the `point`
 * and `skelPoint` fields inside each TrueTip are in WORLD (video) coords.
 * `skeleton` and `distTransform` are LOCAL to `localBounds`.
 */
struct EndpointResult {
    SkeletonGraph        skeleton;
    cv::Mat              distTransform;        // CV_32F, local coords
    cv::Rect             localBounds;          // origin offset (LOCAL->WORLD)
    std::vector<TrueTip>     tips;             // 0-2 entries (post-prune)
    std::vector<TipCapDebug> tipCapDebug;      // parallel to tips[], bilateral debug
    std::vector<cv::Point2f> contourPoints;    // WORLD coords, for curvature debug
    std::vector<float> contourCurvatures;      // Signed curvature, aligned with contourPoints
    std::vector<int> contourCurvaturePeaks;    // Indices into contourPoints
    Tracking::TopologyState topology = Tracking::TopologyState::Unknown;
    int                  headIdx = -1;         // index into `tips`
    int                  tailIdx = -1;         // index into `tips`

    bool valid() const {
        return !skeleton.points.empty() && !skeleton.skeleton.empty();
    }
};

/**
 * @brief Running per-worm distribution of tip features, accumulated online.
 *
 * Fed only from frames where the worm's blob has clean topology — defined as:
 *   • no ring (blob.holeContourPoints empty),
 *   • the worm is not part of a multi-worm merge group at this frame,
 *   • detectEndpoints() returned exactly two SkeletonEndpoint candidates.
 *
 * Under those conditions both candidates are bona-fide tips of this individual,
 * so their (|curvature|, width) features sample the "what does a tip look like
 * on this worm" distribution. The centerline arc length on the same frame
 * samples a body-length distribution that downstream code uses as a length
 * prior when only one tip is visible.
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
 * @brief Carrier for the per-worm temporal state detectEndpoints() needs.
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
    cv::Point2f lastCenterPos = {0.f, 0.f};
    bool        hasVelocity  = false;     // True after two consecutive successful assignments.
    cv::Point2f velHead      = {0.f, 0.f};
    cv::Point2f velTail      = {0.f, 0.f};
    cv::Point2f velCenter    = {0.f, 0.f};
    float       refDistance  = 30.f;      // Normalising scale for the distance term (~½ body length).
};

/**
 * @brief Single-pass endpoint detector for the centerline pipeline.
 *
 * Pure function: depends only on `blob.contourPoints`, `blob.holeContourPoints`,
 * the per-worm `predictor` (head/tail tie-breaking), the per-worm `baseline`
 * (curvature-peak threshold), and the caller-supplied `inMergeGroup` flag.
 *
 * Steps performed (see implementation in trackingcommon.cpp for detail):
 *   (a) Build padded local mask + distance transform.
 *   (b) Skeletonize -> assemble SkeletonGraph (points, adjacency, indexImage).
 *   (c) Find degree-1 nodes; if more than 2, prune to the longest-path pair.
 *   (d) Compute outer-contour signed curvature; find local maxima.
 *   (e) For each surviving skeleton endpoint, search for a curvature peak
 *       within DT(endpoint) * 1.5 pixels. If found, set `point` to the
 *       strongest peak in range and mark `extended = true`. Otherwise
 *       `point = skelPoint` (nearest contour point).
 *   (f) Classify topology: holes present OR fewer than 2 tips -> SelfCrossed;
 *       otherwise Clean. `inMergeGroup=true` overrides to Merged.
 *   (g) Assign head/tail by minimum total predicted-distance, with velocity
 *       extrapolation as tiebreak when the cost spread is < 10%.
 *
 * The renderer at videoloader.cpp keys on `tc.source`. The caller is
 * expected to write each TrueTip back into `blob.tipCandidates` with
 * `tc.source = SkeletonEndpoint`, regardless of `extended`, so the dots
 * keep their familiar green-filled appearance.
 */
EndpointResult detectEndpoints(const Tracking::DetectedBlob& blob,
                               const HeadTailPredictor& predictor,
                               const TipFeatureBaseline& baseline,
                               bool inMergeGroup);


} // namespace Centerline

#endif // CENTERLINETYPES_H
