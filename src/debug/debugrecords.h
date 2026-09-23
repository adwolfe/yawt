#ifndef DEBUGRECORDS_H
#define DEBUGRECORDS_H

#include <QString>
#include <QStringList>
#include <vector>

#include "../core/centerlinetypes.h"

namespace Debug {

/**
 * @brief Per-tip debug snapshot of the terminal-axis boundary selection.
 *
 * Populated inside detectEndpoints() for every skeleton endpoint and carried
 * on Debug::EndpointDebug::tipCapDebug (parallel to EndpointResult::tips).
 * Consumed only by the debug exporter — zero cost in release when unused.
 *
 * All point coordinates are in video space.
 */
struct TipCapDebug {
    bool valid = false;

    // Raw endpoint and fitted terminal-axis geometry.
    cv::Point2f skelEndpoint;   // raw skeleton degree-1 node; fit samples extend inward
    cv::Point2f outwardDir;     // normalised direction away from body interior
    float dtAtEp    = 0.f;     // DT value at skeleton endpoint
    cv::Point2f axisOrigin;    // interior fit origin; ray starts here
    std::vector<cv::Point2f> axisSamples;
    bool hasAxis = false;

    // Independent comparison estimates (for overlay in exporter).
    cv::Point2f snapPoint;       // projectedEndpointContourIdx snap (= t.skelPoint)
    cv::Point2f peakOrSnapPoint; // curvature peak if found, else same as snapPoint
    bool hadPeak = false;        // true when a curvature peak was accepted

    cv::Point2f selectedPoint;
    QString selectedEstimator;
    QString selectionReason;
};

/**
 * @brief Per-skeleton-endpoint audit trail for detectEndpoints().
 *
 * This records the exact handoff from a degree-1 skeleton node to contour snap,
 * curvature-peak search, optional peak rejection, and final TrueTip output.
 * It is debug/export data only; centerline decisions still consume TrueTip.
 */
struct EndpointCandidateDebug {
    int rawEndpointOrder = -1;
    int prunedEndpointOrder = -1;
    int graphIndex = -1;
    int graphDegree = 0;

    cv::Point2f skeletonLocal = {0.f, 0.f};
    cv::Point2f skeletonVideo = {0.f, 0.f};
    cv::Point2f outwardDir = {0.f, 0.f};
    float dtAtEndpoint = 0.f;
    float maxForward = 0.f;
    float maxSide = 0.f;

    int snapContourIdx = -1;
    cv::Point2f snapVideo = {0.f, 0.f};
    float snapCurvature = 0.f;

    int reachablePeakCount = 0;
    int bestPeakContourIdx = -1;
    float bestPeakScore = 0.f;
    cv::Point2f bestPeakVideo = {-1.f, -1.f};
    float bestPeakCurvature = 0.f;
    float bestPeakDistanceFromSnap = 0.f;
    float maxPeakShift = 0.f;
    bool peakAccepted = false;
    QString peakRejectReason;

    int finalTipIdx = -1;
    cv::Point2f finalTipVideo = {-1.f, -1.f};
    bool finalExtended = false;
    float finalCurvature = 0.f;
    float finalWidth = 0.f;
    bool finalHasAxis = false;
};

/**
 * @brief Everything detectEndpoints() records purely for the debug exporter.
 *
 * Passed to detectEndpoints() as an optional sink. When the caller passes
 * nullptr (every non-debug path) none of this is computed or copied.
 */
struct EndpointDebug {
    std::vector<int> rawSkeletonEndpointIndices;      // degree-1 nodes before pruning (graph indices)
    std::vector<cv::Point2f> contourPoints;           // video coords, aligned with contourCurvatures
    std::vector<float> contourCurvatures;             // signed curvature per contour point
    std::vector<int> contourCurvaturePeaks;           // indices into contourPoints
    std::vector<TipCapDebug> tipCapDebug;             // parallel to EndpointResult::tips
    std::vector<EndpointCandidateDebug> endpointCandidateDebug;
};

enum class Pipeline {
    Tracking,
    Centerline
};

enum class CenterlineBranch {
    Unknown,
    D1CleanGraphPath,
    D1SyntheticHoleRetry,
    D2TwoKnownTips,
    D3OneKnownTipHiddenPrediction,
    D4FallbackContourSkeleton,
    ZeroTipRingCut
};

struct DistanceTransformDebug {
    cv::Rect localBounds;
    int rows = 0;
    int cols = 0;
    std::vector<float> values;

    bool empty() const {
        return rows <= 0 || cols <= 0 || values.empty();
    }

    float at(int y, int x) const {
        return values[static_cast<size_t>(y * cols + x)];
    }
};

inline QString centerlineBranchToString(CenterlineBranch branch)
{
    switch (branch) {
    case CenterlineBranch::D1CleanGraphPath:
        return QStringLiteral("D-1 clean graph path");
    case CenterlineBranch::D1SyntheticHoleRetry:
        return QStringLiteral("D-1 synthetic-hole retry");
    case CenterlineBranch::D2TwoKnownTips:
        return QStringLiteral("D-2 two known tips");
    case CenterlineBranch::D3OneKnownTipHiddenPrediction:
        return QStringLiteral("D-3 one known tip / hidden prediction");
    case CenterlineBranch::D4FallbackContourSkeleton:
        return QStringLiteral("D-4 fallback contour skeleton");
    case CenterlineBranch::ZeroTipRingCut:
        return QStringLiteral("0-tip ring cut");
    case CenterlineBranch::Unknown:
    default:
        return QStringLiteral("Unknown");
    }
}

struct CenterlineFrameDebug {
    int wormId = -1;
    int frameNumber = -1;
    int sweepStep = 0;
    bool keyframeBootstrap = false;

    Tracking::TopologyState topology = Tracking::TopologyState::Unknown;
    bool inMergeGroup = false;
    CenterlineBranch branch = CenterlineBranch::Unknown;

    Centerline::HeadTailPredictor predictorBefore;
    Centerline::TipFeatureBaseline baselineBefore;

    float refLength = 0.f;
    float previousTurningAngle = 0.f;
    float initialArcLength = 0.f;
    float finalArcLength = 0.f;
    float finalTurningAngle = 0.f;

    bool rhrFlipped = false;
    bool snakeRan = false;
    bool fallbackUsed = false;
    bool syntheticHoleUsed = false;
    bool hiddenTipHypothesized = false;

    cv::Point2f predictedHead = {0.f, 0.f};
    cv::Point2f predictedTail = {0.f, 0.f};
    cv::Point2f predictedCenter = {0.f, 0.f};
    cv::Point2f hiddenTipTarget = {-1.f, -1.f};
    cv::Point2f hiddenTipFinal = {-1.f, -1.f};
    cv::Point2f hiddenTipJunction = {-1.f, -1.f};
    int hiddenTipMaskDiffArea = 0;
    int hiddenTipMaskDiffSelectedArea = 0;

    std::vector<Tracking::TipCandidate> tipCandidates;
    int assignedHeadTipIdx = -1;
    int assignedTailTipIdx = -1;

    cv::Rect endpointLocalBounds;
    std::vector<cv::Point2f> skeletonPixels;
    std::vector<cv::Point2f> rawSkeletonEndpointPoints;
    std::vector<int> rawSkeletonEndpointGraphIndices;
    std::vector<int> prunedSkeletonEndpointGraphIndices;
    std::vector<cv::Point2f> skeletonEndpointPoints;
    DistanceTransformDebug distanceTransform;
    std::vector<cv::Point2f> contourCurvaturePoints;
    std::vector<float> contourCurvatures;
    std::vector<int> contourCurvaturePeaks;
    std::vector<EndpointCandidateDebug> endpointCandidateDebug;

    // Terminal-axis debug — one entry per detected tip.
    // tipCapRoles[i] is "head" / "tail" / "" matching tipCandidates[i].
    std::vector<TipCapDebug> tipCapDebug;
    std::vector<QString>               tipCapRoles;

    std::vector<cv::Point2f> initialCenterline;
    std::vector<cv::Point2f> resampledCenterline;
    std::vector<cv::Point2f> finalCenterline;

    bool d3RouteDebugAvailable = false;
    bool d3RouteStartIsHead = false;
    int d3SelectedCandidate = -1;
    int d3JunctionClusterCount = 0;
    int d3SelectedJunctionCluster = -1;
    bool d3JunctionFallbackUsed = false;
    cv::Point2f d3RouteStart = {-1.f, -1.f};
    cv::Point2f d3RouteJunction = {-1.f, -1.f};
    // Video-coordinate node positions for every junction cluster (index = cluster id).
    std::vector<std::vector<cv::Point2f>> d3JunctionClusterNodes;
    cv::Point2f d3RouteCenter = {-1.f, -1.f};
    cv::Point2f d3RouteEnd = {-1.f, -1.f};
    std::vector<std::vector<cv::Point2f>> d3CandidatePaths;
    QStringList d3JunctionDiagnostics;

    QStringList decisions;
};

} // namespace Debug

#endif // DEBUGRECORDS_H
