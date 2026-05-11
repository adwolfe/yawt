
#include "trackingcommon.h" // Lowercase include
#include <QtMath>       // For qSqrt, qPow
#include <QDebug>       // For qWarning/qDebug
#include "../utils/loggingcategories.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>


namespace Tracking {

namespace {

constexpr int kCenterlinePaddingPixels = 2;

void zhangSuenThinningIteration(cv::Mat& image, int iteration)
{
    cv::Mat marker = cv::Mat::zeros(image.size(), CV_8UC1);

    for (int row = 1; row < image.rows - 1; ++row) {
        const uchar* previousRow = image.ptr<uchar>(row - 1);
        const uchar* currentRow = image.ptr<uchar>(row);
        const uchar* nextRow = image.ptr<uchar>(row + 1);
        uchar* markerRow = marker.ptr<uchar>(row);

        for (int col = 1; col < image.cols - 1; ++col) {
            if (currentRow[col] == 0) {
                continue;
            }

            const int p2 = previousRow[col];
            const int p3 = previousRow[col + 1];
            const int p4 = currentRow[col + 1];
            const int p5 = nextRow[col + 1];
            const int p6 = nextRow[col];
            const int p7 = nextRow[col - 1];
            const int p8 = currentRow[col - 1];
            const int p9 = previousRow[col - 1];

            const int neighborCount = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            if (neighborCount < 2 || neighborCount > 6) {
                continue;
            }

            const int transitions =
                (p2 == 0 && p3 == 1) +
                (p3 == 0 && p4 == 1) +
                (p4 == 0 && p5 == 1) +
                (p5 == 0 && p6 == 1) +
                (p6 == 0 && p7 == 1) +
                (p7 == 0 && p8 == 1) +
                (p8 == 0 && p9 == 1) +
                (p9 == 0 && p2 == 1);
            if (transitions != 1) {
                continue;
            }

            const bool shouldRemove =
                iteration == 0
                    ? (p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                    : (p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0);
            if (shouldRemove) {
                markerRow[col] = 1;
            }
        }
    }

    image &= ~marker;
}

cv::Mat skeletonizeBinaryMask(const cv::Mat& binaryMask)
{
    cv::Mat img;
    if (binaryMask.type() != CV_8UC1) {
        binaryMask.convertTo(img, CV_8UC1);
    } else {
        img = binaryMask.clone();
    }

    cv::threshold(img, img, 0, 255, cv::THRESH_BINARY);

    // Zhang-Suen thinning preserves connected curved bodies much better than
    // iterative erosion/dilation skeletons for thick, bent worm masks.
    img /= 255;
    cv::Mat previous = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;

    do {
        zhangSuenThinningIteration(img, 0);
        zhangSuenThinningIteration(img, 1);
        cv::absdiff(img, previous, diff);
        img.copyTo(previous);
    } while (cv::countNonZero(diff) > 0);

    img *= 255;
    return img;
}

struct GraphSearchResult {
    std::vector<double> distances;
    std::vector<int> parents;
};

GraphSearchResult dijkstraSkeleton(const std::vector<cv::Point>& points,
                                   const std::vector<std::vector<int>>& adjacency,
                                   int startIndex)
{
    GraphSearchResult result;
    result.distances.assign(points.size(), std::numeric_limits<double>::infinity());
    result.parents.assign(points.size(), -1);

    if (startIndex < 0 || startIndex >= static_cast<int>(points.size())) {
        return result;
    }

    using QueueEntry = std::pair<double, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    result.distances[startIndex] = 0.0;
    queue.push({0.0, startIndex});

    while (!queue.empty()) {
        const auto [dist, current] = queue.top();
        queue.pop();
        if (dist > result.distances[current]) {
            continue;
        }

        const cv::Point& currentPoint = points[current];
        for (int neighbor : adjacency[current]) {
            const cv::Point& neighborPoint = points[neighbor];
            const int dx = neighborPoint.x - currentPoint.x;
            const int dy = neighborPoint.y - currentPoint.y;
            const double weight = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            const double candidate = dist + weight;
            if (candidate + 1e-9 < result.distances[neighbor]) {
                result.distances[neighbor] = candidate;
                result.parents[neighbor] = current;
                queue.push({candidate, neighbor});
            }
        }
    }

    return result;
}

std::vector<cv::Point2f> reconstructCenterlinePath(const std::vector<cv::Point>& points,
                                                   const std::vector<int>& parents,
                                                   int startIndex,
                                                   int endIndex,
                                                   const cv::Point2f& offset)
{
    std::vector<cv::Point2f> path;
    if (startIndex < 0 || endIndex < 0 || startIndex >= static_cast<int>(points.size())
        || endIndex >= static_cast<int>(points.size())) {
        return path;
    }

    int current = endIndex;
    while (current != -1) {
        const cv::Point& point = points[current];
        path.emplace_back(offset.x + static_cast<float>(point.x),
                          offset.y + static_cast<float>(point.y));
        if (current == startIndex) {
            break;
        }
        current = parents[current];
    }

    if (path.empty() || current == -1) {
        path.clear();
        return path;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

cv::Rect buildCenterlineMask(const DetectedBlob& blob, cv::Mat& mask)
{
    cv::Rect localBounds = cv::boundingRect(blob.contourPoints);
    localBounds.x -= kCenterlinePaddingPixels;
    localBounds.y -= kCenterlinePaddingPixels;
    localBounds.width += 2 * kCenterlinePaddingPixels;
    localBounds.height += 2 * kCenterlinePaddingPixels;

    if (localBounds.width <= 1 || localBounds.height <= 1) {
        mask.release();
        return localBounds;
    }

    mask = cv::Mat::zeros(localBounds.height, localBounds.width, CV_8UC1);

    std::vector<std::vector<cv::Point>> outerContours(1);
    outerContours.front().reserve(blob.contourPoints.size());
    for (const cv::Point& pt : blob.contourPoints) {
        outerContours.front().push_back(cv::Point(pt.x - localBounds.x, pt.y - localBounds.y));
    }
    cv::fillPoly(mask, outerContours, cv::Scalar(255));

    for (const std::vector<cv::Point>& hole : blob.holeContourPoints) {
        std::vector<std::vector<cv::Point>> holeContour(1);
        holeContour.front().reserve(hole.size());
        for (const cv::Point& pt : hole) {
            holeContour.front().push_back(cv::Point(pt.x - localBounds.x, pt.y - localBounds.y));
        }
        cv::fillPoly(mask, holeContour, cv::Scalar(0));
    }

    if (!blob.holeContourPoints.empty()) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    }

    return localBounds;
}

std::vector<cv::Point2f> extractCenterlineFromMask(const cv::Mat& mask,
                                                   const cv::Point2f& offset)
{
    if (mask.empty()) {
        return {};
    }

    cv::Mat skeleton = skeletonizeBinaryMask(mask);
    if (cv::countNonZero(skeleton) < 2) {
        return {};
    }

    cv::Mat indexImage(skeleton.size(), CV_32SC1, cv::Scalar(-1));
    std::vector<cv::Point> skeletonPoints;
    skeletonPoints.reserve(static_cast<size_t>(cv::countNonZero(skeleton)));

    for (int row = 0; row < skeleton.rows; ++row) {
        const uchar* rowPtr = skeleton.ptr<uchar>(row);
        int* indexRow = indexImage.ptr<int>(row);
        for (int col = 0; col < skeleton.cols; ++col) {
            if (rowPtr[col] == 0) {
                continue;
            }
            indexRow[col] = static_cast<int>(skeletonPoints.size());
            skeletonPoints.emplace_back(col, row);
        }
    }

    if (skeletonPoints.size() < 2) {
        return {};
    }

    std::vector<std::vector<int>> adjacency(skeletonPoints.size());
    std::vector<int> endpoints;
    endpoints.reserve(skeletonPoints.size());

    for (int idx = 0; idx < static_cast<int>(skeletonPoints.size()); ++idx) {
        const cv::Point& point = skeletonPoints[idx];
        int degree = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = point.x + dx;
                const int ny = point.y + dy;
                if (nx < 0 || ny < 0 || nx >= indexImage.cols || ny >= indexImage.rows) {
                    continue;
                }
                const int neighborIndex = indexImage.at<int>(ny, nx);
                if (neighborIndex < 0) {
                    continue;
                }
                adjacency[idx].push_back(neighborIndex);
                ++degree;
            }
        }
        if (degree == 1) {
            endpoints.push_back(idx);
        }
    }

    int bestStart = -1;
    int bestEnd = -1;
    double bestDistance = -1.0;
    std::vector<int> bestParents;

    if (endpoints.size() >= 2) {
        for (int endpoint : endpoints) {
            GraphSearchResult search = dijkstraSkeleton(skeletonPoints, adjacency, endpoint);
            for (int candidate : endpoints) {
                if (candidate == endpoint) {
                    continue;
                }
                const double distance = search.distances[candidate];
                if (!std::isfinite(distance) || distance <= bestDistance) {
                    continue;
                }
                bestDistance = distance;
                bestStart = endpoint;
                bestEnd = candidate;
                bestParents = search.parents;
            }
        }
    }

    if (bestStart == -1 || bestEnd == -1) {
        GraphSearchResult firstSweep = dijkstraSkeleton(skeletonPoints, adjacency, 0);
        int farthestA = 0;
        for (int idx = 1; idx < static_cast<int>(skeletonPoints.size()); ++idx) {
            if (std::isfinite(firstSweep.distances[idx]) && firstSweep.distances[idx] > firstSweep.distances[farthestA]) {
                farthestA = idx;
            }
        }

        GraphSearchResult secondSweep = dijkstraSkeleton(skeletonPoints, adjacency, farthestA);
        int farthestB = farthestA;
        for (int idx = 0; idx < static_cast<int>(skeletonPoints.size()); ++idx) {
            if (std::isfinite(secondSweep.distances[idx]) && secondSweep.distances[idx] > secondSweep.distances[farthestB]) {
                farthestB = idx;
            }
        }

        bestStart = farthestA;
        bestEnd = farthestB;
        bestParents = std::move(secondSweep.parents);
    }

    return reconstructCenterlinePath(skeletonPoints, bestParents, bestStart, bestEnd, offset);
}

} // namespace

// ── Tip-candidate detection ─────────────────────────────────────────────────
//
// Pure preprocessing pass: from a blob's outer contour + holes, surface the
// points on the outer contour that are most likely to be physical tips
// (nose or tail) of the worm. Two complementary detectors are merged:
//
//   (a) Zhang-Suen skeleton degree-1 endpoints, mapped to nearest outer
//       contour point. Clean unobstructed worm → always 2. Coiled worm with
//       a protruding free end → typically 1 (the free end; the closure loop
//       contributes no degree-1 nodes). Closed-ring blob → 0.
//
//   (b) Local maxima of |signed curvature| along the outer contour. Surfaces
//       real tips even when the skeleton is degenerate (rings, merges), at
//       the cost of false positives on tight body kinks; downstream scoring
//       against the worm's curvature baseline filters those.
//
// For each surviving candidate we compute the local contour curvature and a
// local mask thickness (≈ body width at the tip). These two features are the
// inputs the head/tail-assignment step uses to score against per-worm
// baselines built up on clean frames.
const std::vector<TipCandidate>& findTipCandidates(DetectedBlob& blob,
                                                   int   curvatureWindow,
                                                   float minCurvatureMagnitude,
                                                   float mergePlanarDistancePx)
{
    blob.tipCandidates.clear();

    if (!blob.isValid || blob.contourPoints.size() < 8) {
        return blob.tipCandidates;
    }

    // ── Build local mask + bounds (reuses existing helper) ──────────────────
    cv::Mat mask;
    cv::Rect localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) return blob.tipCandidates;

    const cv::Point2f originOffset(static_cast<float>(localBounds.x),
                                   static_cast<float>(localBounds.y));

    // ── Distance transform of the mask (for width estimation) ───────────────
    // The DT value at any interior pixel = distance to nearest boundary, so
    // twice the DT value sampled a few pixels inward from a contour point is
    // a robust local body-thickness estimate.
    cv::Mat dt;
    cv::distanceTransform(mask, dt, cv::DIST_L2, 3);

    // ── Outer contour in local coordinates (for curvature + neighbour ops) ──
    const std::vector<cv::Point>& contour = blob.contourPoints;
    const int nContour = static_cast<int>(contour.size());
    std::vector<cv::Point2f> contourLocal;
    contourLocal.reserve(nContour);
    for (const cv::Point& p : contour) {
        contourLocal.emplace_back(static_cast<float>(p.x - localBounds.x),
                                  static_cast<float>(p.y - localBounds.y));
    }

    // Contour centroid (for inward-direction sampling of DT).
    cv::Point2f contourCentroidLocal(0.f, 0.f);
    for (const cv::Point2f& p : contourLocal) contourCentroidLocal += p;
    contourCentroidLocal *= 1.f / static_cast<float>(nContour);

    // ── (a) Skeleton degree-1 endpoints ─────────────────────────────────────
    std::vector<cv::Point2f> skeletonEndpoints;       // in local coords
    {
        cv::Mat skeleton = skeletonizeBinaryMask(mask);
        const int skeletonPixelCount = cv::countNonZero(skeleton);
        if (skeletonPixelCount >= 2) {
            // Mark each skeleton pixel; degree-1 = exactly one foreground 8-neighbour.
            for (int row = 1; row < skeleton.rows - 1; ++row) {
                const uchar* up   = skeleton.ptr<uchar>(row - 1);
                const uchar* mid  = skeleton.ptr<uchar>(row);
                const uchar* down = skeleton.ptr<uchar>(row + 1);
                for (int col = 1; col < skeleton.cols - 1; ++col) {
                    if (mid[col] == 0) continue;
                    int neighbours = 0;
                    neighbours += (up[col - 1]   > 0) ? 1 : 0;
                    neighbours += (up[col]       > 0) ? 1 : 0;
                    neighbours += (up[col + 1]   > 0) ? 1 : 0;
                    neighbours += (mid[col - 1]  > 0) ? 1 : 0;
                    neighbours += (mid[col + 1]  > 0) ? 1 : 0;
                    neighbours += (down[col - 1] > 0) ? 1 : 0;
                    neighbours += (down[col]     > 0) ? 1 : 0;
                    neighbours += (down[col + 1] > 0) ? 1 : 0;
                    if (neighbours == 1) {
                        skeletonEndpoints.emplace_back(static_cast<float>(col),
                                                      static_cast<float>(row));
                    }
                }
            }
        }
    }

    // ── Per-contour-point signed curvature (window = curvatureWindow) ───────
    // Signed turning-angle / arc-length, computed in a ±k window that skips
    // over local sampling noise from cv::findContours.
    const int k = std::max(2, curvatureWindow);
    std::vector<float> curvature(nContour, 0.f);
    for (int i = 0; i < nContour; ++i) {
        const cv::Point2f& a = contourLocal[(i - k + nContour) % nContour];
        const cv::Point2f& b = contourLocal[i];
        const cv::Point2f& c = contourLocal[(i + k) % nContour];
        const cv::Point2f v1 = b - a;
        const cv::Point2f v2 = c - b;
        const float cross = v1.x * v2.y - v1.y * v2.x;
        const float dot   = v1.x * v2.x + v1.y * v2.y;
        const float angle = std::atan2(cross, dot);
        const float arc   = 0.5f * (std::hypot(v1.x, v1.y) + std::hypot(v2.x, v2.y));
        curvature[i] = (arc > 1e-3f) ? (angle / arc) : 0.f;
    }

    // OpenCV's outer contours are traversed clockwise on image coordinates
    // (y-down), which flips the curvature sign relative to the textbook
    // mathematical convention. We don't care about absolute sign here —
    // downstream scoring uses |curvature| or per-worm-baseline sign.

    // ── (b) Local maxima of |curvature| ─────────────────────────────────────
    // Use a sliding window of size 2k+1; a point qualifies if its |curvature|
    // is the strict maximum in the window AND exceeds the threshold.
    std::vector<int> curvaturePeakIdx;
    curvaturePeakIdx.reserve(8);
    for (int i = 0; i < nContour; ++i) {
        const float mag = std::abs(curvature[i]);
        if (mag < minCurvatureMagnitude) continue;
        bool isLocalMax = true;
        for (int d = -k; d <= k; ++d) {
            if (d == 0) continue;
            const int j = (i + d + nContour) % nContour;
            if (std::abs(curvature[j]) > mag) { isLocalMax = false; break; }
        }
        if (isLocalMax) curvaturePeakIdx.push_back(i);
    }

    // ── Helper: width = 2 × DT sampled perpendicular-inward from a tip ──────
    // The "inward" direction is taken from the local contour tangent rotated
    // 90°, with the side that probes higher DT chosen as the mask interior.
    //
    // Previous implementation walked toward the contour centroid, which works
    // on straight bodies but fails on coiled/curved bodies whose centroid
    // lies in the empty hole of the curl — there the probe walks AWAY from
    // body material and returns near-zero. This version derives "into the
    // mask" purely from local geometry, so curvature of the overall body
    // doesn't affect it.
    auto probeDT = [&](const cv::Point2f& origin, const cv::Point2f& dir, float depth) -> float {
        const int px = std::clamp(static_cast<int>(std::round(origin.x + dir.x * depth)),
                                  0, dt.cols - 1);
        const int py = std::clamp(static_cast<int>(std::round(origin.y + dir.y * depth)),
                                  0, dt.rows - 1);
        return dt.at<float>(py, px);
    };

    auto widthAt = [&](const cv::Point2f& tipLocal, int contourIdx) -> float {
        const int nC = static_cast<int>(contourLocal.size());
        if (nC < 4) return 0.f;

        // Local tangent from ±tangentK contour neighbours. tangentK = 3 gives
        // a smooth tangent that ignores per-pixel sampling noise from
        // cv::findContours without overshooting on tightly curved bodies.
        const int tangentK = 3;
        const cv::Point2f& a = contourLocal[(contourIdx - tangentK + nC) % nC];
        const cv::Point2f& c = contourLocal[(contourIdx + tangentK) % nC];
        cv::Point2f tangent = c - a;
        const float tNorm = std::hypot(tangent.x, tangent.y);
        if (tNorm < 1e-3f) return 0.f;
        tangent *= 1.f / tNorm;

        // Two candidate inward normals: rotate the tangent ±90°. Pick the
        // one whose 1.5 px probe lands at higher DT (deeper into the mask).
        const cv::Point2f perpA(-tangent.y,  tangent.x);
        const cv::Point2f perpB( tangent.y, -tangent.x);
        const float dtA = probeDT(tipLocal, perpA, 1.5f);
        const float dtB = probeDT(tipLocal, perpB, 1.5f);
        if (dtA < 0.5f && dtB < 0.5f) {
            // Tip sits at a very sharp taper where both perpendicular probes
            // exit the mask. Degenerate; return 0 so the feature is treated
            // as missing rather than misleading.
            return 0.f;
        }
        const cv::Point2f inward = (dtA >= dtB) ? perpA : perpB;

        // Final sample at 3 px depth. At a real tapered tip this lands near
        // the body axis with DT ≈ local half-width (modest). At a body kink
        // it lands well past the axis, often near the opposite contour, so
        // DT is small. The 2× factor converts half-width to full thickness.
        return 2.f * probeDT(tipLocal, inward, 3.f);
    };

    // ── Helper: nearest contour index to a local-coords point ──────────────
    auto nearestContourIdx = [&](const cv::Point2f& q) -> int {
        int bestIdx = -1;
        float bestDistSq = std::numeric_limits<float>::max();
        for (int i = 0; i < nContour; ++i) {
            const cv::Point2f d = contourLocal[i] - q;
            const float dsq = d.x * d.x + d.y * d.y;
            if (dsq < bestDistSq) { bestDistSq = dsq; bestIdx = i; }
        }
        return bestIdx;
    };

    // ── Merge: skeleton endpoints first (priority), then curvature peaks ───
    const float mergeSq = mergePlanarDistancePx * mergePlanarDistancePx;
    auto pushIfNew = [&](int contourIdx, TipCandidate::Source source) {
        const cv::Point2f& contourPtLocal = contourLocal[contourIdx];
        for (const TipCandidate& existing : blob.tipCandidates) {
            const cv::Point2f existingLocal(existing.point.x - localBounds.x,
                                            existing.point.y - localBounds.y);
            const cv::Point2f d = existingLocal - contourPtLocal;
            if (d.x * d.x + d.y * d.y <= mergeSq) return;  // duplicate
        }
        TipCandidate tc;
        tc.point     = cv::Point2f(contourPtLocal.x + originOffset.x,
                                   contourPtLocal.y + originOffset.y);
        tc.curvature = curvature[contourIdx];
        tc.width     = widthAt(contourPtLocal, contourIdx);
        tc.source    = source;
        blob.tipCandidates.push_back(tc);
    };

    for (const cv::Point2f& skelPt : skeletonEndpoints) {
        const int idx = nearestContourIdx(skelPt);
        if (idx < 0) continue;
        pushIfNew(idx, TipCandidate::Source::SkeletonEndpoint);
    }
    for (int idx : curvaturePeakIdx) {
        pushIfNew(idx, TipCandidate::Source::CurvaturePeak);
    }

    // Debug log — fires at verbosity ≥2 (Normal) via QT_LOGGING_RULES.
    // Caller (e.g. centerlineworker Pass 4) emits frame/worm context just
    // beforehand so log lines stay correlatable across hundreds of frames.
    if (lcDataCommon().isDebugEnabled()) {
        int skel = 0, curv = 0;
        for (const TipCandidate& tc : blob.tipCandidates) {
            (tc.source == TipCandidate::Source::SkeletonEndpoint ? skel : curv) += 1;
        }
        YAWT_DEBUG(lcDataCommon) << QString::asprintf(
            "findTipCandidates: %d skel + %d curv  (centroid=(%.1f, %.1f))",
            skel, curv,
            static_cast<double>(contourCentroidLocal.x + originOffset.x),
            static_cast<double>(contourCentroidLocal.y + originOffset.y));
        for (size_t i = 0; i < blob.tipCandidates.size(); ++i) {
            const TipCandidate& tc = blob.tipCandidates[i];
            YAWT_DEBUG(lcDataCommon) << QString::asprintf(
                "  [%zu] %s  at (%.1f, %.1f)  k=%+.4f  w=%.2f",
                i,
                tc.source == TipCandidate::Source::SkeletonEndpoint ? "skel" : "curv",
                static_cast<double>(tc.point.x),
                static_cast<double>(tc.point.y),
                static_cast<double>(tc.curvature),
                static_cast<double>(tc.width));
        }
    }

    return blob.tipCandidates;
}

// ── Topology classification (Phase C.2) ─────────────────────────────────────
//
// Per-frame geometric classification of a worm's blob. Independent of the
// tracker's confidence (TrackPointQuality); these flags describe the *shape*
// the centerline algorithm has to deal with on this frame:
//
//   Clean       — standard, both tips visible: D-1 skeleton path is fine.
//   SelfCrossed — ring topology, or only one skeleton tip surfaced.
//                 Centerline dispatch will use D-2 (geodesic head→tail) when
//                 both tips are assigned, else D-3 (geodesic-of-length).
//   Merged      — blob is shared with another worm. Centerline isn't useful
//                 on a merged blob; D-4 (legacy cut-skeleton) is the fallback.
//   Lost        — no valid blob this frame.
TopologyState classifyTopology(DetectedBlob& blob, bool inMergeGroup)
{
    TopologyState s;
    if (!blob.isValid) {
        s = TopologyState::Lost;
    } else if (inMergeGroup) {
        s = TopologyState::Merged;
    } else {
        int skeletonTips = 0;
        for (const TipCandidate& tc : blob.tipCandidates) {
            if (tc.source == TipCandidate::Source::SkeletonEndpoint) ++skeletonTips;
        }
        const bool hasRing = !blob.holeContourPoints.empty();
        s = (hasRing || skeletonTips < 2)
            ? TopologyState::SelfCrossed
            : TopologyState::Clean;
    }
    blob.topologyState = s;
    return s;
}

// ── Head/tail assignment (Phase C.1) ────────────────────────────────────────
//
// Given a per-frame set of tip candidates plus the per-worm baseline + a
// short temporal prediction, decide which candidate is the head and which is
// the tail.
//
// The optimal assignment over only two roles is trivial: enumerate all ordered
// pairs (i, j) of candidate indices with i ≠ j, pick the pair minimising
// cost(head=i) + cost(tail=j). With N tip candidates that's N*(N-1) pairs —
// always tiny in practice (clean worm = 2 candidates → 2 pairs).
//
// Single-candidate fallback: when only one tip is detected, assign it to
// whichever role has lower individual cost. The opposite role stays
// unassigned (-1) and the caller's predictor keeps the stale position prior
// for the missing tip — Phase D-3 (geodesic-of-body-length from the known
// tip) is what will eventually fill in the hidden tip's spatial estimate.
//
// Cost weights are hard-coded here for v1; expose to the Debug tab if tuning
// against real footage turns up an obvious bias.
bool assignHeadTail(DetectedBlob& blob,
                    const HeadTailPredictor& predictor,
                    const TipFeatureBaseline& baseline)
{
    blob.assignedHeadTipIdx = -1;
    blob.assignedTailTipIdx = -1;

    if (!predictor.hasPrev || blob.tipCandidates.empty()) {
        return false;
    }

    // ── Cost weights ────────────────────────────────────────────────────────
    constexpr float kWeightDistance   = 1.0f;
    constexpr float kWeightVelocity   = 0.5f;
    constexpr float kWeightCurvature  = 0.4f;
    constexpr float kWeightWidth      = 0.4f;

    // Coefficient-of-variation gate. A baseline feature whose standard
    // deviation rivals or exceeds its mean has no discriminative signal — it
    // would inject noise into the cost. We require σ/μ < kMaxFeatureCV
    // before that feature contributes to the cost. The gate is re-evaluated
    // every call, so a feature that starts as noise (early-baseline, very
    // few samples) silently becomes usable later as the baseline stabilises.
    constexpr float kMaxFeatureCV = 0.5f;

    auto featureCV = [](float mean, float stddev) -> float {
        return (mean > 1e-3f) ? (stddev / mean) : std::numeric_limits<float>::max();
    };

    const bool baselineReliable = baseline.isReliable();
    const float curvatureCV = featureCV(baseline.meanAbsCurvature, baseline.curvatureStdDev());
    const float widthCV     = featureCV(baseline.meanWidth,        baseline.widthStdDev());
    const bool useCurvatureFeature = baselineReliable && curvatureCV < kMaxFeatureCV;
    const bool useWidthFeature     = baselineReliable && widthCV     < kMaxFeatureCV;
    const float refDist = std::max(1.0f, predictor.refDistance);

    // ── Predicted positions: prev + velocity (if available) ────────────────
    const cv::Point2f predictedHead = predictor.hasVelocity
        ? predictor.lastHeadPos + predictor.velHead
        : predictor.lastHeadPos;
    const cv::Point2f predictedTail = predictor.hasVelocity
        ? predictor.lastTailPos + predictor.velTail
        : predictor.lastTailPos;

    // ── Per-candidate cost breakdown ────────────────────────────────────────
    // Returns the four cost components separately so the debug logger can
    // attribute the chosen pair to a specific term. Curvature/width terms
    // use the unlabelled-baseline distribution (Phase A), so they don't
    // discriminate head from tail — both terms are the same for a given
    // candidate regardless of which role we're evaluating it for.
    struct CostBreakdown {
        float distance = 0.f;
        float velocity = 0.f;
        float curvature = 0.f;
        float width = 0.f;
        float total() const { return distance + velocity + curvature + width; }
    };

    auto roleCostBreakdown = [&](const TipCandidate& tc,
                                 const cv::Point2f& predictedPos,
                                 const cv::Point2f& lastPos,
                                 const cv::Point2f& predictedVel) -> CostBreakdown {
        CostBreakdown b;
        const cv::Point2f delta = tc.point - predictedPos;
        const float dist = std::hypot(delta.x, delta.y);
        b.distance = kWeightDistance * (dist / refDist);

        if (predictor.hasVelocity) {
            const cv::Point2f move = tc.point - lastPos;
            const float moveMag = std::hypot(move.x, move.y);
            const float velMag  = std::hypot(predictedVel.x, predictedVel.y);
            // Skip the velocity term when either vector is degenerate — a
            // stationary worm has no meaningful direction.
            if (moveMag > 0.5f && velMag > 0.5f) {
                const float cosTheta = (move.x * predictedVel.x + move.y * predictedVel.y)
                                       / (moveMag * velMag);
                b.velocity = kWeightVelocity * (1.f - cosTheta);
            }
        }

        if (useCurvatureFeature) {
            const float cSd = std::max(1e-3f, baseline.curvatureStdDev());
            const float cZ  = std::abs(std::abs(tc.curvature) - baseline.meanAbsCurvature) / cSd;
            b.curvature = kWeightCurvature * cZ;
        }
        if (useWidthFeature) {
            const float wSd = std::max(1e-3f, baseline.widthStdDev());
            const float wZ  = std::abs(tc.width - baseline.meanWidth) / wSd;
            b.width = kWeightWidth * wZ;
        }
        return b;
    };

    auto roleCost = [&](const TipCandidate& tc,
                        const cv::Point2f& predictedPos,
                        const cv::Point2f& lastPos,
                        const cv::Point2f& predictedVel) -> float {
        return roleCostBreakdown(tc, predictedPos, lastPos, predictedVel).total();
    };

    // ── Debug log: predictor state going INTO this frame ───────────────────
    const bool logEnabled = lcDataCommon().isDebugEnabled();
    if (logEnabled) {
        YAWT_DEBUG(lcDataCommon) << QString::asprintf(
            "assignHeadTail: nCands=%d  refDist=%.1f  useCurv=%d  useWidth=%d  hasVel=%d",
            static_cast<int>(blob.tipCandidates.size()),
            static_cast<double>(refDist),
            useCurvatureFeature ? 1 : 0,
            useWidthFeature     ? 1 : 0,
            predictor.hasVelocity ? 1 : 0);
        YAWT_DEBUG(lcDataCommon) << QString::asprintf(
            "  lastHead=(%.1f, %.1f)  velHead=(%+.2f, %+.2f)  -> predHead=(%.1f, %.1f)",
            static_cast<double>(predictor.lastHeadPos.x),
            static_cast<double>(predictor.lastHeadPos.y),
            static_cast<double>(predictor.velHead.x),
            static_cast<double>(predictor.velHead.y),
            static_cast<double>(predictedHead.x),
            static_cast<double>(predictedHead.y));
        YAWT_DEBUG(lcDataCommon) << QString::asprintf(
            "  lastTail=(%.1f, %.1f)  velTail=(%+.2f, %+.2f)  -> predTail=(%.1f, %.1f)",
            static_cast<double>(predictor.lastTailPos.x),
            static_cast<double>(predictor.lastTailPos.y),
            static_cast<double>(predictor.velTail.x),
            static_cast<double>(predictor.velTail.y),
            static_cast<double>(predictedTail.x),
            static_cast<double>(predictedTail.y));
        if (baselineReliable) {
            YAWT_DEBUG(lcDataCommon) << QString::asprintf(
                "  baseline: |k|=%.4f±%.4f (n=%d cv=%.2f%s)  w=%.2f±%.2f (n=%d cv=%.2f%s)",
                static_cast<double>(baseline.meanAbsCurvature),
                static_cast<double>(baseline.curvatureStdDev()),
                baseline.curvatureSamples,
                static_cast<double>(curvatureCV),
                useCurvatureFeature ? "" : " GATED",
                static_cast<double>(baseline.meanWidth),
                static_cast<double>(baseline.widthStdDev()),
                baseline.widthSamples,
                static_cast<double>(widthCV),
                useWidthFeature ? "" : " GATED");
        }
    }

    const int nCands = static_cast<int>(blob.tipCandidates.size());

    // ── Per-candidate cost dump (for both roles) ────────────────────────────
    // Computed once per candidate (head + tail breakdowns) so the logger and
    // the assignment loop share the same numbers and don't double-compute.
    std::vector<CostBreakdown> headCosts(nCands);
    std::vector<CostBreakdown> tailCosts(nCands);
    for (int i = 0; i < nCands; ++i) {
        headCosts[i] = roleCostBreakdown(blob.tipCandidates[i], predictedHead,
                                         predictor.lastHeadPos, predictor.velHead);
        tailCosts[i] = roleCostBreakdown(blob.tipCandidates[i], predictedTail,
                                         predictor.lastTailPos, predictor.velTail);
        if (logEnabled) {
            const TipCandidate& tc = blob.tipCandidates[i];
            YAWT_DEBUG(lcDataCommon) << QString::asprintf(
                "  cand[%d] %s (%.1f,%.1f)  H=%.2f [d=%.2f v=%.2f k=%.2f w=%.2f]  "
                "T=%.2f [d=%.2f v=%.2f k=%.2f w=%.2f]",
                i,
                tc.source == TipCandidate::Source::SkeletonEndpoint ? "skel" : "curv",
                static_cast<double>(tc.point.x),
                static_cast<double>(tc.point.y),
                static_cast<double>(headCosts[i].total()),
                static_cast<double>(headCosts[i].distance),
                static_cast<double>(headCosts[i].velocity),
                static_cast<double>(headCosts[i].curvature),
                static_cast<double>(headCosts[i].width),
                static_cast<double>(tailCosts[i].total()),
                static_cast<double>(tailCosts[i].distance),
                static_cast<double>(tailCosts[i].velocity),
                static_cast<double>(tailCosts[i].curvature),
                static_cast<double>(tailCosts[i].width));
        }
    }

    // ── Single-candidate branch: assign to lower-cost role ─────────────────
    if (nCands == 1) {
        const float costH = headCosts[0].total();
        const float costT = tailCosts[0].total();
        if (costH <= costT) blob.assignedHeadTipIdx = 0;
        else                blob.assignedTailTipIdx = 0;
        if (logEnabled) {
            YAWT_DEBUG(lcDataCommon) << QString::asprintf(
                "  -> single candidate; assigned to %s (H=%.2f T=%.2f)",
                (costH <= costT) ? "head" : "tail",
                static_cast<double>(costH), static_cast<double>(costT));
        }
        return true;
    }

    // ── ≥2 candidates: enumerate ordered pairs, pick min total cost ────────
    float bestTotal = std::numeric_limits<float>::max();
    int bestHead = -1, bestTail = -1;
    for (int i = 0; i < nCands; ++i) {
        for (int j = 0; j < nCands; ++j) {
            if (i == j) continue;
            const float total = headCosts[i].total() + tailCosts[j].total();
            if (total < bestTotal) {
                bestTotal = total;
                bestHead = i;
                bestTail = j;
            }
        }
    }

    blob.assignedHeadTipIdx = bestHead;
    blob.assignedTailTipIdx = bestTail;

    if (logEnabled) {
        YAWT_DEBUG(lcDataCommon) << QString::asprintf(
            "  -> chosen pair head=cand[%d] tail=cand[%d]  total=%.2f",
            bestHead, bestTail, static_cast<double>(bestTotal));
    }

    return bestHead >= 0;
}

bool populateCenterlineFromContour(DetectedBlob& blob)
{
    blob.centerlinePoints.clear();

    if (!blob.isValid || blob.contourPoints.size() < 3) {
        return false;
    }

    cv::Mat mask;
    cv::Rect localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) return false;

    blob.centerlinePoints = extractCenterlineFromMask(
        mask,
        cv::Point2f(static_cast<float>(localBounds.x), static_cast<float>(localBounds.y)));

    return blob.centerlinePoints.size() >= 2;
}

bool populateCenterlineFromContourWithCut(DetectedBlob& blob,
                                          const cv::Point2f& cutStart,
                                          const cv::Point2f& cutEnd,
                                          int cutThickness)
{
    blob.centerlinePoints.clear();

    if (!blob.isValid || blob.contourPoints.size() < 3 || blob.holeContourPoints.empty()) {
        return false;
    }

    cv::Mat mask;
    cv::Rect localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) return false;

    const cv::Point localStart(qRound(cutStart.x - localBounds.x),
                               qRound(cutStart.y - localBounds.y));
    const cv::Point localEnd(qRound(cutEnd.x - localBounds.x),
                             qRound(cutEnd.y - localBounds.y));
    const int thickness = std::max(1, cutThickness);
    cv::line(mask, localStart, localEnd, cv::Scalar(0), thickness, cv::LINE_8);

    blob.centerlinePoints = extractCenterlineFromMask(
        mask,
        cv::Point2f(static_cast<float>(localBounds.x), static_cast<float>(localBounds.y)));

    return blob.centerlinePoints.size() >= 2;
}

QList<QPointF> extractOrderedCenterlinePoints(const DetectedBlob& blob)
{
    QList<QPointF> centerlinePoints;
    if (!blob.isValid) {
        return centerlinePoints;
    }

    DetectedBlob centerlineBlob = blob;
    if (centerlineBlob.centerlinePoints.empty() && !centerlineBlob.contourPoints.empty()) {
        populateCenterlineFromContour(centerlineBlob);
    }

    if (centerlineBlob.centerlinePoints.empty()) {
        return centerlinePoints;
    }

    centerlinePoints.reserve(static_cast<qsizetype>(centerlineBlob.centerlinePoints.size()));
    for (const cv::Point2f& point : centerlineBlob.centerlinePoints) {
        centerlinePoints.append(QPointF(point.x, point.y));
    }

    return centerlinePoints;
}

QList<QPointF> resampleCenterlinePoints(const QList<QPointF>& points, int pointCount)
{
    QList<QPointF> sampledPoints;
    if (points.isEmpty() || pointCount <= 0) {
        return sampledPoints;
    }

    sampledPoints.reserve(pointCount);
    if (points.size() == 1 || pointCount == 1) {
        for (int i = 0; i < pointCount; ++i) {
            sampledPoints.append(points.first());
        }
        return sampledPoints;
    }

    std::vector<double> cumulativeDistance(static_cast<size_t>(points.size()), 0.0);
    for (int i = 1; i < points.size(); ++i) {
        const QPointF delta = points.at(i) - points.at(i - 1);
        cumulativeDistance[static_cast<size_t>(i)] =
            cumulativeDistance[static_cast<size_t>(i - 1)] + std::hypot(delta.x(), delta.y());
    }

    const double totalLength = cumulativeDistance.back();
    if (qFuzzyIsNull(totalLength)) {
        for (int i = 0; i < pointCount; ++i) {
            sampledPoints.append(points.first());
        }
        return sampledPoints;
    }

    int segmentIndex = 1;
    for (int sampleIndex = 0; sampleIndex < pointCount; ++sampleIndex) {
        const double targetDistance =
            totalLength * static_cast<double>(sampleIndex) / static_cast<double>(pointCount - 1);

        while (segmentIndex < points.size() - 1 &&
               cumulativeDistance[static_cast<size_t>(segmentIndex)] < targetDistance) {
            ++segmentIndex;
        }

        const double previousDistance = cumulativeDistance[static_cast<size_t>(segmentIndex - 1)];
        const double nextDistance = cumulativeDistance[static_cast<size_t>(segmentIndex)];
        const double segmentLength = nextDistance - previousDistance;
        const double t = qFuzzyIsNull(segmentLength)
                             ? 0.0
                             : (targetDistance - previousDistance) / segmentLength;

        const QPointF a = points.at(segmentIndex - 1);
        const QPointF b = points.at(segmentIndex);
        sampledPoints.append(a + (b - a) * t);
    }

    return sampledPoints;
}

QList<QPointF> resampleCenterlinePoints(const std::vector<cv::Point2f>& points, int pointCount)
{
    QList<QPointF> convertedPoints;
    convertedPoints.reserve(static_cast<qsizetype>(points.size()));
    for (const cv::Point2f& point : points) {
        convertedPoints.append(QPointF(point.x, point.y));
    }
    return resampleCenterlinePoints(convertedPoints, pointCount);
}

QList<QPointF> extractResampledCenterlinePoints(const DetectedBlob& blob, int pointCount)
{
    return resampleCenterlinePoints(extractOrderedCenterlinePoints(blob), pointCount);
}

// Uses thresholded mat to find the nearest blob to a click and selects it.
DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea,
                             double maxArea,
                             double maxDistanceForSelection) {
    DetectedBlob result;
    result.isValid = false;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1) {
        YAWT_WARN(lcDataCommon) << "findClickedBlob: Invalid input image (empty or not CV_8UC1).";
        return result;
    }

    cv::Point clickCvPoint(qRound(clickPointVideoCoords.x()), qRound(clickPointVideoCoords.y()));
    if (clickCvPoint.x < 0 || clickCvPoint.y < 0 ||
        clickCvPoint.x >= binaryImage.cols || clickCvPoint.y >= binaryImage.rows) {
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Click out of bounds. Click:"
                                 << clickPointVideoCoords << "Image size:"
                                 << QSize(binaryImage.cols, binaryImage.rows);
        return result;
    }

    const int clickPixel = static_cast<int>(binaryImage.at<uchar>(clickCvPoint));
    YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Click:"
                             << clickPointVideoCoords
                             << "Pixel:" << clickPixel
                             << "Image size:" << QSize(binaryImage.cols, binaryImage.rows)
                             << "Min/Max area:" << minArea << "/" << maxArea
                             << "Max dist:" << maxDistanceForSelection;

    std::vector<std::vector<cv::Point>> contours;
    // Use a copy of binaryImage for findContours if it modifies the input
    cv::findContours(binaryImage.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: No contours found.";
        return result;
    }

    int bestContourIdx = -1;
    double minDistanceSqToCentroid = std::numeric_limits<double>::max();
    bool clickInsideABlob = false;
    int areaPassedCount = 0;
    double minContourArea = std::numeric_limits<double>::max();
    double maxContourArea = 0.0;

    // Pass 1: Check for contours whose bounding box *contains* the click point.
    // Prioritize these. If multiple, could pick smallest area or closest centroid.
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < minContourArea) minContourArea = area;
        if (area > maxContourArea) maxContourArea = area;
        if (area < minArea || area > maxArea) { // Apply area filter
            continue;
        }
        areaPassedCount++;
        cv::Rect br = cv::boundingRect(contours[i]);
        if (br.contains(clickCvPoint)) {
            // This contour is a strong candidate.
            // If we find one, we can potentially stop and use this one.
            // For now, let's take the first valid one we find that contains the click.
            // A more refined approach might be to find the one with the smallest area that contains the click.
            cv::Moments mu = cv::moments(contours[i]);
            if (mu.m00 > 0) { // Check for valid moments
                double distSq = qPow( (mu.m10 / mu.m00) - clickPointVideoCoords.x(), 2) +
                                qPow( (mu.m01 / mu.m00) - clickPointVideoCoords.y(), 2);
                if (distSq < minDistanceSqToCentroid) { // Prefer the one whose centroid is closer if multiple contain click
                    minDistanceSqToCentroid = distSq;
                    bestContourIdx = static_cast<int>(i);
                    clickInsideABlob = true;
                }
            }
        }
    }

    // Pass 2: If click was not inside any blob's bounding box, find the blob with the closest centroid.
    if (!clickInsideABlob) {
        minDistanceSqToCentroid = std::numeric_limits<double>::max(); // Reset for this pass
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < minArea || area > maxArea) { // Apply area filter
                continue;
            }
            cv::Moments mu = cv::moments(contours[i]);
            if (mu.m00 > 0) { // Check for valid moments (non-zero area)
                cv::Point2f centroid(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
                double distSq = qPow(centroid.x - clickPointVideoCoords.x(), 2) +
                                qPow(centroid.y - clickPointVideoCoords.y(), 2);

                if (distSq < minDistanceSqToCentroid) {
                    minDistanceSqToCentroid = distSq;
                    bestContourIdx = static_cast<int>(i);
                }
            }
        }
        // Check if the closest one found is within the maxDistanceForSelection
        if (qSqrt(minDistanceSqToCentroid) > maxDistanceForSelection) {
            bestContourIdx = -1; // Too far, invalidate selection
        }
    }


    // If a suitable contour was found by either method
    if (bestContourIdx != -1) {
        const auto& bestContour = contours[bestContourIdx];
        cv::Moments mu = cv::moments(bestContour);
        // Double check mu.m00 > 0, though area filter should imply this
        if (mu.m00 > 0) {
            result.centroid = QPointF(static_cast<double>(mu.m10 / mu.m00), static_cast<double>(mu.m01 / mu.m00));
            cv::Rect brCv = cv::boundingRect(bestContour);
            result.boundingBox = QRectF(brCv.x, brCv.y, brCv.width, brCv.height);
            result.area = cv::contourArea(bestContour); // Already calculated, but store it
            result.contourPoints = bestContour; // These points are relative to binaryImage origin
            result.isValid = true;
            populateCenterlineFromContour(result);
            // touchesROIboundary is not relevant for findClickedBlob as it operates on the whole image or a pre-defined mask.
        }
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Selected contour idx:"
                                 << bestContourIdx
                                 << "Centroid:" << result.centroid
                                 << "Area:" << result.area
                                 << "BBox:" << result.boundingBox;
    } else {
        const double minAreaForLog = (minContourArea == std::numeric_limits<double>::max()) ? 0.0 : minContourArea;
        const double dist = (minDistanceSqToCentroid == std::numeric_limits<double>::max())
                                ? -1.0
                                : qSqrt(minDistanceSqToCentroid);
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: No valid blob."
                                 << "Contours:" << contours.size()
                                 << "Area-passing:" << areaPassedCount
                                 << "Area min/max:" << minAreaForLog << "/" << maxContourArea
                                 << "Click inside bbox:" << clickInsideABlob
                                 << "Nearest centroid dist:" << dist;
    }

    return result;
}


QList<DetectedBlob> findAllPlausibleBlobsInRoi(const cv::Mat& binaryImage,
                                               const QRectF& roiToSearch, // This is in full image coordinates
                                               double minArea,
                                               double maxArea,
                                               double minAspectRatio,
                                               double maxAspectRatio) {
    QList<DetectedBlob> plausibleBlobs;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1 || roiToSearch.isEmpty() || roiToSearch.width() <=0 || roiToSearch.height() <=0) {
        YAWT_WARN(lcDataCommon) << "findAllPlausibleBlobsInRoi: Invalid input image or ROI.";
        return plausibleBlobs;
    }

    // Define the OpenCV ROI from QRectF (roiToSearch is in full image coordinates)
    cv::Rect roiCv(static_cast<int>(qRound(roiToSearch.x())),
                   static_cast<int>(qRound(roiToSearch.y())),
                   static_cast<int>(qRound(roiToSearch.width())),
                   static_cast<int>(qRound(roiToSearch.height())));

    // Ensure ROI is within the image boundaries
    // This creates the actual ROI that will be used on binaryImage
    cv::Rect actualRoiCv = roiCv & cv::Rect(0, 0, binaryImage.cols, binaryImage.rows);

    if (actualRoiCv.width <= 0 || actualRoiCv.height <= 0) {
        // qDebug() << "findAllPlausibleBlobsInRoi: ROI after clamping is invalid or outside image.";
        return plausibleBlobs; // ROI is outside image or has no area
    }

    cv::Mat roiImage = binaryImage(actualRoiCv); // Extract the sub-image for contour finding
    std::vector<std::vector<cv::Point>> contoursInSubImage;
    std::vector<cv::Vec4i> hierarchy;
    // RETR_CCOMP gives a 2-level hierarchy (outer contours + their holes).
    // This lets us subtract hole areas from outer contour areas, so a coiled worm
    // whose thresholded shape is a ring is measured by actual pixel area rather than
    // the much-larger disk area that RETR_EXTERNAL + contourArea would produce.
    cv::findContours(roiImage.clone(), contoursInSubImage, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contoursInSubImage.size(); ++i) {
        // Only process outer contours (parent index == -1 in RETR_CCOMP)
        if (hierarchy[i][3] != -1) continue;

        const auto& contourInSub = contoursInSubImage[i];
        double outerArea = cv::contourArea(contourInSub);

        // Subtract areas of direct-child hole contours to get true foreground pixel area.
        // This handles the ring topology produced by a self-touching coiled worm.
        double holeArea = 0.0;
        for (int childIdx = hierarchy[i][2]; childIdx != -1; childIdx = hierarchy[childIdx][0]) {
            holeArea += cv::contourArea(contoursInSubImage[childIdx]);
        }
        double area = outerArea - holeArea;

        // Calculate convex hull area using the outer boundary
        std::vector<cv::Point> hull;
        cv::convexHull(contourInSub, hull);
        double hullArea = cv::contourArea(hull);

        if (area < minArea || area > maxArea) {
            continue; // Filter by area
        }

        // Bounding box of the contour, relative to roiImage (the sub-image)
        cv::Rect brInSub = cv::boundingRect(contourInSub);
        if (brInSub.width == 0 || brInSub.height == 0) {
            continue; // Skip zero-dimension bounding boxes
        }

        // Aspect ratio (using dimensions from brInSub)
        double currentAspectRatio = static_cast<double>(brInSub.width) / static_cast<double>(brInSub.height);
        if (currentAspectRatio < 1.0) {
            currentAspectRatio = 1.0 / currentAspectRatio; // Ensure aspect ratio is >= 1
        }

        // Aspect ratio filter (currently commented out in your provided code)
        // if (currentAspectRatio < minAspectRatio || currentAspectRatio > maxAspectRatio) {
        //     continue;
        // }

        cv::Moments mu = cv::moments(contourInSub);
        if (mu.m00 > 0) { // Check for valid moments (non-zero area)
            DetectedBlob blob;
            blob.isValid = true;
            blob.area = area;
            blob.convexHullArea = hullArea;

            // Convert centroid and bounding box to full image coordinates
            // Centroid in sub-image: (mu.m10 / mu.m00), (mu.m01 / mu.m00)
            // Add actualRoiCv.x and actualRoiCv.y to convert to full image coordinates
            blob.centroid = QPointF(actualRoiCv.x + (mu.m10 / mu.m00),
                                    actualRoiCv.y + (mu.m01 / mu.m00));

            // Bounding box in sub-image: brInSub
            // Add actualRoiCv.x and actualRoiCv.y to convert to full image coordinates
            blob.boundingBox = QRectF(actualRoiCv.x + brInSub.x,
                                      actualRoiCv.y + brInSub.y,
                                      brInSub.width,
                                      brInSub.height);

            // Offset outer contour points to full frame coordinates
            blob.contourPoints.reserve(contourInSub.size());
            for(const cv::Point& ptInSub : contourInSub) {
                blob.contourPoints.push_back(cv::Point(ptInSub.x + actualRoiCv.x, ptInSub.y + actualRoiCv.y));
            }

            // Offset hole contour points to full frame coordinates
            for (int childIdx = hierarchy[i][2]; childIdx != -1; childIdx = hierarchy[childIdx][0]) {
                std::vector<cv::Point> holeInFullFrame;
                holeInFullFrame.reserve(contoursInSubImage[childIdx].size());
                for (const cv::Point& ptInSub : contoursInSubImage[childIdx]) {
                    holeInFullFrame.push_back(cv::Point(ptInSub.x + actualRoiCv.x, ptInSub.y + actualRoiCv.y));
                }
                blob.holeContourPoints.push_back(std::move(holeInFullFrame));
            }

            // --- Set touchesROIboundary flag ---
            // Check if the bounding box of the contour (brInSub, which is relative to roiImage)
            // touches the edges of roiImage.
            // roiImage has dimensions actualRoiCv.width and actualRoiCv.height.
            // Note: actualRoiCv.width and actualRoiCv.height are the dimensions of roiImage.
            if (brInSub.x <= 0 ||
                brInSub.y <= 0 ||
                (brInSub.x + brInSub.width) >= actualRoiCv.width ||
                (brInSub.y + brInSub.height) >= actualRoiCv.height) {
                blob.touchesROIboundary = true;
            } else {
                blob.touchesROIboundary = false;
            }
            // A more precise check could iterate over contour points if needed, but bounding box is usually sufficient.
            // For example, if any point in contourInSub has x=0, y=0, x=actualRoiCv.width-1, or y=actualRoiCv.height-1.
            // However, the bounding box check is simpler and often what's implied.

            populateCenterlineFromContour(blob);
            plausibleBlobs.append(blob);
        }
    }
    std::sort(plausibleBlobs.begin(), plausibleBlobs.end(), [](const DetectedBlob& a, const DetectedBlob& b) {
        return a.area > b.area; // For descending order; largest blob first
    });
    return plausibleBlobs;
}


} // namespace Tracking
