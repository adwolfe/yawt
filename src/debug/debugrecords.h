#ifndef DEBUGRECORDS_H
#define DEBUGRECORDS_H

#include <QString>
#include <QStringList>
#include <vector>

#include "../core/centerlinetypes.h"

namespace Debug {

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
    int directionStep = 0;
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
    std::vector<Centerline::EndpointCandidateDebug> endpointCandidateDebug;

    // Bilateral cap-midpoint debug — one entry per tip (parallel to tipCandidates).
    // Populated whenever the new bilateral path ran (Clean frames primarily).
    // tipCapRoles[i] is "head" / "tail" / "" matching tipCandidates[i].
    std::vector<Centerline::TipCapDebug> tipCapDebug;
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
    // World-coord node positions for every junction cluster (index = cluster id).
    std::vector<std::vector<cv::Point2f>> d3JunctionClusterNodes;
    cv::Point2f d3RouteCenter = {-1.f, -1.f};
    cv::Point2f d3RouteEnd = {-1.f, -1.f};
    std::vector<std::vector<cv::Point2f>> d3CandidatePaths;
    QStringList d3JunctionDiagnostics;

    QStringList decisions;
};

} // namespace Debug

#endif // DEBUGRECORDS_H
