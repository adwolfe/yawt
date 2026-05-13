
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

// ── Single-pass endpoint detector ───────────────────────────────────────────
//
// Used by the centerline pipeline (centerlineworker doWork). Builds the
// skeleton once, then derives every downstream quantity from it.
//
// Design notes:
//
// * Curvature math + width probe are kept stable so baseline samples stay
//   commensurable across frames.
//
// * Mask construction reuses buildCenterlineMask, including the MORPH_CLOSE
//   applied for ring blobs.
//
// * Endpoint pruning preserves the longest-path pair when more than two
//   degree-1 nodes are present (skeleton noise spurs). Never drops below two
//   if two existed; never drops the only one if only one exists.
//
// * Each surviving skeleton endpoint is "extended" to the strongest curvature
//   peak within DT(endpoint) * 1.5 pixels (search radius scales with local
//   body thickness). Strongest, not nearest — picking the nearest peak snaps
//   onto body kinks for tightly-coiled worms.
EndpointResult detectEndpoints(const DetectedBlob& blob,
                               const HeadTailPredictor& predictor,
                               const TipFeatureBaseline& baseline,
                               bool inMergeGroup)
{
    EndpointResult r;

    if (!blob.isValid) {
        r.topology = TopologyState::Lost;
        return r;
    }
    // Merged blobs still get their skeleton + tips harvested when possible —
    // the topology label is set to Merged at the end. centerlineworker treats
    // Merged as a no-op for centerline computation but the tip data is still
    // useful for the renderer.
    if (blob.contourPoints.size() < 8) {
        r.topology = inMergeGroup ? TopologyState::Merged
                                  : TopologyState::SelfCrossed;
        return r;
    }

    // (a) ── Mask + DT ──────────────────────────────────────────────────────
    cv::Mat mask;
    r.localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) {
        r.topology = inMergeGroup ? TopologyState::Merged
                                  : TopologyState::SelfCrossed;
        return r;
    }
    cv::distanceTransform(mask, r.distTransform, cv::DIST_L2, 3);

    const cv::Point2f originOffset(static_cast<float>(r.localBounds.x),
                                   static_cast<float>(r.localBounds.y));

    // (b) ── Skeleton + adjacency + indexImage ──────────────────────────────
    // Identical procedure to extractCenterlineFromMask so the resulting graph
    // matches what the legacy centerline path produces.
    cv::Mat skel = skeletonizeBinaryMask(mask);
    if (cv::countNonZero(skel) < 2) {
        r.topology = inMergeGroup ? TopologyState::Merged
                                  : TopologyState::SelfCrossed;
        return r;
    }
    r.skeleton.skeleton   = skel;
    r.skeleton.indexImage = cv::Mat(skel.size(), CV_32SC1, cv::Scalar(-1));

    for (int row = 0; row < skel.rows; ++row) {
        const uchar* rowPtr = skel.ptr<uchar>(row);
        int* indexRow = r.skeleton.indexImage.ptr<int>(row);
        for (int col = 0; col < skel.cols; ++col) {
            if (rowPtr[col] == 0) continue;
            indexRow[col] = static_cast<int>(r.skeleton.points.size());
            r.skeleton.points.emplace_back(col, row);
        }
    }
    if (r.skeleton.points.size() < 2) {
        r.topology = inMergeGroup ? TopologyState::Merged
                                  : TopologyState::SelfCrossed;
        return r;
    }

    r.skeleton.adjacency.assign(r.skeleton.points.size(), {});
    std::vector<int> rawEndpoints;
    rawEndpoints.reserve(r.skeleton.points.size());
    for (int idx = 0; idx < static_cast<int>(r.skeleton.points.size()); ++idx) {
        const cv::Point& p = r.skeleton.points[idx];
        int degree = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const int nx = p.x + dx, ny = p.y + dy;
                if (nx < 0 || ny < 0 ||
                    nx >= r.skeleton.indexImage.cols ||
                    ny >= r.skeleton.indexImage.rows) continue;
                const int neigh = r.skeleton.indexImage.at<int>(ny, nx);
                if (neigh < 0) continue;
                r.skeleton.adjacency[idx].push_back(neigh);
                ++degree;
            }
        }
        if (degree == 1) rawEndpoints.push_back(idx);
    }

    // (c) ── Prune to ≤ 2 endpoints (longest-path pair) ─────────────────────
    int rawEndpointCount = static_cast<int>(rawEndpoints.size());
    if (rawEndpoints.size() <= 2) {
        r.skeleton.endpointIndices = rawEndpoints;
    } else {
        int bestA = rawEndpoints[0];
        int bestB = rawEndpoints[1];
        double bestDist = -1.0;
        for (int ep : rawEndpoints) {
            GraphSearchResult sr = dijkstraSkeleton(
                r.skeleton.points, r.skeleton.adjacency, ep);
            for (int other : rawEndpoints) {
                if (other == ep) continue;
                const double d = sr.distances[other];
                if (std::isfinite(d) && d > bestDist) {
                    bestDist = d;
                    bestA = ep;
                    bestB = other;
                }
            }
        }
        r.skeleton.endpointIndices = {bestA, bestB};
    }

    // (d) ── Outer-contour signed curvature + local maxima ─────────────────
    const std::vector<cv::Point>& contour = blob.contourPoints;
    const int nContour = static_cast<int>(contour.size());
    std::vector<cv::Point2f> contourLocal;
    contourLocal.reserve(nContour);
    for (const cv::Point& p : contour) {
        contourLocal.emplace_back(static_cast<float>(p.x - r.localBounds.x),
                                  static_cast<float>(p.y - r.localBounds.y));
    }

    constexpr int kCurvatureWindow = 5;
    const int k = std::max(2, kCurvatureWindow);
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

    // Curvature peak threshold: scaled by baseline if reliable, else default.
    const float curvFloor = baseline.isReliable()
        ? std::max(0.5f * baseline.meanAbsCurvature, 0.04f)
        : 0.08f;

    std::vector<int> curvaturePeakIdx;
    curvaturePeakIdx.reserve(8);
    for (int i = 0; i < nContour; ++i) {
        const float mag = std::abs(curvature[i]);
        if (mag < curvFloor) continue;
        bool isLocalMax = true;
        for (int d = -k; d <= k; ++d) {
            if (d == 0) continue;
            const int j = (i + d + nContour) % nContour;
            if (std::abs(curvature[j]) > mag) { isLocalMax = false; break; }
        }
        if (isLocalMax) curvaturePeakIdx.push_back(i);
    }

    // ── Width probe ────────────────────────────────────────────────────────
    auto probeDT = [&](const cv::Point2f& origin, const cv::Point2f& dir, float depth) -> float {
        const int px = std::clamp(static_cast<int>(std::round(origin.x + dir.x * depth)),
                                  0, r.distTransform.cols - 1);
        const int py = std::clamp(static_cast<int>(std::round(origin.y + dir.y * depth)),
                                  0, r.distTransform.rows - 1);
        return r.distTransform.at<float>(py, px);
    };

    auto widthAt = [&](const cv::Point2f& tipLocal, int contourIdx) -> float {
        const int nC = static_cast<int>(contourLocal.size());
        if (nC < 4) return 0.f;
        const int tangentK = 3;
        const cv::Point2f& a = contourLocal[(contourIdx - tangentK + nC) % nC];
        const cv::Point2f& c = contourLocal[(contourIdx + tangentK) % nC];
        cv::Point2f tangent = c - a;
        const float tNorm = std::hypot(tangent.x, tangent.y);
        if (tNorm < 1e-3f) return 0.f;
        tangent *= 1.f / tNorm;
        const cv::Point2f perpA(-tangent.y,  tangent.x);
        const cv::Point2f perpB( tangent.y, -tangent.x);
        const float dtA = probeDT(tipLocal, perpA, 1.5f);
        const float dtB = probeDT(tipLocal, perpB, 1.5f);
        if (dtA < 0.5f && dtB < 0.5f) return 0.f;
        const cv::Point2f inward = (dtA >= dtB) ? perpA : perpB;
        return 2.f * probeDT(tipLocal, inward, 3.f);
    };

    // ── nearest contour idx to a local-coords point ────────────────────────
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

    // (e) ── Extend each skeleton endpoint to the strongest reachable peak ──
    for (int epIdx : r.skeleton.endpointIndices) {
        const cv::Point& epLocal = r.skeleton.points[epIdx];
        const cv::Point2f epLocalF(static_cast<float>(epLocal.x),
                                   static_cast<float>(epLocal.y));
        const cv::Point2f epWorld(epLocalF.x + originOffset.x,
                                  epLocalF.y + originOffset.y);

        // Skeleton endpoint snapped to nearest outer-contour point.
        const int snapIdx = nearestContourIdx(epLocalF);
        cv::Point2f snapLocal(0.f, 0.f);
        cv::Point2f snapWorld = epWorld;
        if (snapIdx >= 0) {
            snapLocal = contourLocal[snapIdx];
            snapWorld = cv::Point2f(snapLocal.x + originOffset.x,
                                    snapLocal.y + originOffset.y);
        }

        // Search radius scales with local body half-width at the endpoint.
        const float dtAtEp = r.distTransform.at<float>(epLocal.y, epLocal.x);
        const float searchR = std::max(dtAtEp * 1.5f, 4.0f);
        const float searchRSq = searchR * searchR;

        // Find the strongest curvature peak within `searchR` of the
        // skeleton-endpoint contour-snap point. "Strongest" = max |curvature|.
        int bestPeak = -1;
        float bestMag = -1.f;
        for (int peakIdx : curvaturePeakIdx) {
            const cv::Point2f& peakLocal = contourLocal[peakIdx];
            const cv::Point2f d = peakLocal - snapLocal;
            const float dsq = d.x * d.x + d.y * d.y;
            if (dsq > searchRSq) continue;
            const float mag = std::abs(curvature[peakIdx]);
            if (mag > bestMag) {
                bestMag = mag;
                bestPeak = peakIdx;
            }
        }

        TrueTip t;
        t.skelPoint = snapWorld;
        if (bestPeak >= 0) {
            const cv::Point2f peakLocal = contourLocal[bestPeak];
            t.point     = cv::Point2f(peakLocal.x + originOffset.x,
                                      peakLocal.y + originOffset.y);
            t.curvature = curvature[bestPeak];
            t.width     = widthAt(peakLocal, bestPeak);
            t.extended  = true;
        } else {
            t.point     = snapWorld;
            t.curvature = (snapIdx >= 0) ? curvature[snapIdx] : 0.f;
            t.width     = (snapIdx >= 0) ? widthAt(snapLocal, snapIdx) : 0.f;
            t.extended  = false;
        }
        r.tips.push_back(t);
    }

    // (f) ── Topology classification ───────────────────────────────────────
    if (inMergeGroup) {
        r.topology = TopologyState::Merged;
    } else {
        const bool hasRing = !blob.holeContourPoints.empty();
        r.topology = (hasRing || r.tips.size() < 2)
                         ? TopologyState::SelfCrossed
                         : TopologyState::Clean;
    }

    // (g) ── Head/tail assignment ──────────────────────────────────────────
    // Distance-only assignment, with velocity-extrapolated tiebreak when the
    // 2-tip cost spread is < 10%. Predictor-less (keyframe) calls return
    // (-1, -1); the caller bootstraps from the centerline orientation.
    auto distSq = [](const cv::Point2f& a, const cv::Point2f& b) -> float {
        const cv::Point2f d = a - b;
        return d.x * d.x + d.y * d.y;
    };

    if (r.tips.empty() || !predictor.hasPrev) {
        // Nothing to do — caller handles bootstrap.
    } else {
        const cv::Point2f predHead = predictor.hasVelocity
            ? predictor.lastHeadPos + predictor.velHead
            : predictor.lastHeadPos;
        const cv::Point2f predTail = predictor.hasVelocity
            ? predictor.lastTailPos + predictor.velTail
            : predictor.lastTailPos;

        if (r.tips.size() == 1) {
            // One visible tip — assign to whichever role's predicted point
            // is closer. The other role stays -1 so D-3 can fill it.
            const float dH = distSq(r.tips[0].point, predHead);
            const float dT = distSq(r.tips[0].point, predTail);
            if (dH <= dT) r.headIdx = 0;
            else          r.tailIdx = 0;
        } else {
            // Two tips — minimum-cost ordered assignment.
            const float c00 = distSq(r.tips[0].point, predHead) +
                              distSq(r.tips[1].point, predTail);
            const float c01 = distSq(r.tips[1].point, predHead) +
                              distSq(r.tips[0].point, predTail);
            const float winner = std::min(c00, c01);
            const float loser  = std::max(c00, c01);
            const bool spreadIsTight = (winner > 0.f) &&
                                       ((loser - winner) / winner < 0.10f);

            int headPick = (c00 <= c01) ? 0 : 1;
            int tailPick = (c00 <= c01) ? 1 : 0;

            // Velocity-extrapolated tiebreak for tight spreads.
            if (spreadIsTight && predictor.hasVelocity) {
                const cv::Point2f extrapHead = predictor.lastHeadPos +
                                               2.f * predictor.velHead;
                const cv::Point2f extrapTail = predictor.lastTailPos +
                                               2.f * predictor.velTail;
                const float c00x = distSq(r.tips[0].point, extrapHead) +
                                   distSq(r.tips[1].point, extrapTail);
                const float c01x = distSq(r.tips[1].point, extrapHead) +
                                   distSq(r.tips[0].point, extrapTail);
                if (c00x <= c01x) { headPick = 0; tailPick = 1; }
                else              { headPick = 1; tailPick = 0; }
            }

            r.headIdx = headPick;
            r.tailIdx = tailPick;
        }
    }

    // ── Debug log ───────────────────────────────────────────────────────────
    if (lcDataCommon().isDebugEnabled()) {
        YAWT_DEBUG(lcDataCommon) << QString::asprintf(
            "detectEndpoints: rawEnds=%d  prunedEnds=%d  tips=%d  peaks=%d  "
            "topo=%s  head=%d  tail=%d",
            rawEndpointCount,
            static_cast<int>(r.skeleton.endpointIndices.size()),
            static_cast<int>(r.tips.size()),
            static_cast<int>(curvaturePeakIdx.size()),
            qUtf8Printable(topologyStateToString(r.topology)),
            r.headIdx, r.tailIdx);
        for (size_t i = 0; i < r.tips.size(); ++i) {
            const TrueTip& t = r.tips[i];
            YAWT_DEBUG(lcDataCommon) << QString::asprintf(
                "  tip[%zu] %s  point=(%.1f,%.1f)  skel=(%.1f,%.1f)  "
                "k=%+.4f  w=%.2f",
                i, t.extended ? "ext " : "skel",
                static_cast<double>(t.point.x),
                static_cast<double>(t.point.y),
                static_cast<double>(t.skelPoint.x),
                static_cast<double>(t.skelPoint.y),
                static_cast<double>(t.curvature),
                static_cast<double>(t.width));
        }
    }

    return r;
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
