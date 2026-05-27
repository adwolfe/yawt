#include "centerlineprocessor.h"
#include "../utils/loggingcategories.h"

#include <QDebug>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <opencv2/imgproc.hpp>

namespace Centerline {

constexpr int kCenterlinePaddingPixels = 2;

// Run one Zhang-Suen thinning sub-iteration over a normalized 0/1 mask.
static void zhangSuenThinningIteration(cv::Mat& image, int iteration)
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

Centerline::SkeletonGraph buildSkeletonGraph(const cv::Mat& mask)
{
    Centerline::SkeletonGraph graph;
    if (mask.empty()) {
        return graph;
    }

    cv::Mat skeleton = skeletonizeBinaryMask(mask);
    if (cv::countNonZero(skeleton) < 2) {
        return graph;
    }

    graph.skeleton = skeleton;
    graph.indexImage = cv::Mat(skeleton.size(), CV_32SC1, cv::Scalar(-1));
    graph.points.reserve(static_cast<size_t>(cv::countNonZero(skeleton)));

    for (int row = 0; row < skeleton.rows; ++row) {
        const uchar* rowPtr = skeleton.ptr<uchar>(row);
        int* indexRow = graph.indexImage.ptr<int>(row);
        for (int col = 0; col < skeleton.cols; ++col) {
            if (rowPtr[col] == 0) {
                continue;
            }
            indexRow[col] = static_cast<int>(graph.points.size());
            graph.points.emplace_back(col, row);
        }
    }

    if (graph.points.size() < 2) {
        graph = Centerline::SkeletonGraph{};
        return graph;
    }

    graph.adjacency.assign(graph.points.size(), {});
    graph.endpointIndices.reserve(graph.points.size());
    for (int idx = 0; idx < static_cast<int>(graph.points.size()); ++idx) {
        const cv::Point& point = graph.points[idx];
        int degree = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = point.x + dx;
                const int ny = point.y + dy;
                if (nx < 0 || ny < 0 || nx >= graph.indexImage.cols || ny >= graph.indexImage.rows) {
                    continue;
                }
                const int neighborIndex = graph.indexImage.at<int>(ny, nx);
                if (neighborIndex < 0) {
                    continue;
                }
                graph.adjacency[idx].push_back(neighborIndex);
                ++degree;
            }
        }
        if (degree == 1) {
            graph.endpointIndices.push_back(idx);
        }
    }

    return graph;
}

std::vector<cv::Point2f> extractCenterlineFromMask(const cv::Mat& mask,
                                                   const cv::Point2f& offset)
{
    Centerline::SkeletonGraph graph = buildSkeletonGraph(mask);
    if (graph.points.size() < 2) {
        return {};
    }

    int bestStart = -1;
    int bestEnd = -1;
    double bestDistance = -1.0;
    std::vector<int> bestParents;

    if (graph.endpointIndices.size() >= 2) {
        for (int endpoint : graph.endpointIndices) {
            GraphSearchResult search = dijkstraSkeleton(graph.points, graph.adjacency, endpoint);
            for (int candidate : graph.endpointIndices) {
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
        GraphSearchResult firstSweep = dijkstraSkeleton(graph.points, graph.adjacency, 0);
        int farthestA = 0;
        for (int idx = 1; idx < static_cast<int>(graph.points.size()); ++idx) {
            if (std::isfinite(firstSweep.distances[idx]) &&
                firstSweep.distances[idx] > firstSweep.distances[farthestA]) {
                farthestA = idx;
            }
        }

        GraphSearchResult secondSweep = dijkstraSkeleton(graph.points, graph.adjacency, farthestA);
        int farthestB = farthestA;
        for (int idx = 0; idx < static_cast<int>(graph.points.size()); ++idx) {
            if (std::isfinite(secondSweep.distances[idx]) &&
                secondSweep.distances[idx] > secondSweep.distances[farthestB]) {
                farthestB = idx;
            }
        }

        bestStart = farthestA;
        bestEnd = farthestB;
        bestParents = std::move(secondSweep.parents);
    }

    return reconstructCenterlinePath(graph.points, bestParents, bestStart, bestEnd, offset);
}


cv::Rect buildCenterlineMask(const Tracking::DetectedBlob& blob, cv::Mat& mask)
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


EndpointResult detectEndpoints(const Tracking::DetectedBlob& blob,
                               const HeadTailPredictor& predictor,
                               const TipFeatureBaseline& baseline,
                               bool inMergeGroup)
{
    EndpointResult r;

    if (!blob.isValid) {
        r.topology = Tracking::TopologyState::Lost;
        return r;
    }
    // Merged blobs still get their skeleton + tips harvested when possible —
    // the topology label is set to Merged at the end. centerlineworker treats
    // Merged as a no-op for centerline computation but the tip data is still
    // useful for the renderer.
    if (blob.contourPoints.size() < 8) {
        r.topology = inMergeGroup ? Tracking::TopologyState::Merged
                                  : Tracking::TopologyState::SelfCrossed;
        return r;
    }

    // (a) ── Mask + DT ──────────────────────────────────────────────────────
    cv::Mat mask;
    r.localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) {
        r.topology = inMergeGroup ? Tracking::TopologyState::Merged
                                  : Tracking::TopologyState::SelfCrossed;
        return r;
    }
    cv::distanceTransform(mask, r.distTransform, cv::DIST_L2, 3);

    const cv::Point2f originOffset(static_cast<float>(r.localBounds.x),
                                   static_cast<float>(r.localBounds.y));

    // (b) ── Skeleton + adjacency + indexImage ──────────────────────────────
    // Shared with the centerline path so endpoint detection and extraction
    // reason over exactly the same graph representation.
    r.skeleton = Centerline::buildSkeletonGraph(mask);
    if (r.skeleton.points.size() < 2) {
        r.topology = inMergeGroup ? Tracking::TopologyState::Merged
                                  : Tracking::TopologyState::SelfCrossed;
        return r;
    }
    std::vector<int> rawEndpoints = r.skeleton.endpointIndices;

    // (c) ── Prune to ≤ 2 endpoints (longest-path pair) ─────────────────────
    int rawEndpointCount = static_cast<int>(rawEndpoints.size());
    if (rawEndpoints.size() <= 2) {
        r.skeleton.endpointIndices = rawEndpoints;
    } else {
        int bestA = rawEndpoints[0];
        int bestB = rawEndpoints[1];
        double bestDist = -1.0;
        for (int ep : rawEndpoints) {
            Centerline::GraphSearchResult sr = Centerline::dijkstraSkeleton(
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
    r.contourPoints.reserve(nContour);
    r.contourCurvatures = curvature;
    r.contourCurvaturePeaks = curvaturePeakIdx;
    for (const cv::Point& p : contour) {
        r.contourPoints.emplace_back(static_cast<float>(p.x),
                                     static_cast<float>(p.y));
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

    auto endpointOutwardDirection = [&](int epIdx) -> cv::Point2f {
        if (epIdx < 0 || epIdx >= static_cast<int>(r.skeleton.points.size()) ||
            r.skeleton.adjacency[epIdx].empty()) {
            return cv::Point2f(0.f, 0.f);
        }

        int prev = epIdx;
        int cur = r.skeleton.adjacency[epIdx].front();
        cv::Point inner = r.skeleton.points[cur];
        constexpr int kEndpointDirectionSteps = 6;
        for (int step = 1; step < kEndpointDirectionSteps; ++step) {
            int next = -1;
            for (int candidate : r.skeleton.adjacency[cur]) {
                if (candidate != prev) {
                    next = candidate;
                    break;
                }
            }
            if (next < 0) {
                break;
            }
            prev = cur;
            cur = next;
            inner = r.skeleton.points[cur];
        }

        const cv::Point& ep = r.skeleton.points[epIdx];
        cv::Point2f dir(static_cast<float>(ep.x - inner.x),
                        static_cast<float>(ep.y - inner.y));
        const float norm = std::hypot(dir.x, dir.y);
        if (norm < 1e-3f) {
            return cv::Point2f(0.f, 0.f);
        }
        return dir * (1.f / norm);
    };

    auto projectedEndpointContourIdx = [&](const cv::Point2f& epLocalF,
                                           const cv::Point2f& outwardDir,
                                           float dtAtEp) -> int {
        if (std::hypot(outwardDir.x, outwardDir.y) < 1e-3f) {
            return nearestContourIdx(epLocalF);
        }

        const float maxForward = std::clamp(6.f + 4.f * dtAtEp, 8.f, 18.f);
        const float maxSide = std::clamp(2.f + 2.5f * dtAtEp, 4.f, 10.f);
        int bestIdx = -1;
        float bestScore = -std::numeric_limits<float>::max();
        for (int i = 0; i < nContour; ++i) {
            const cv::Point2f rel = contourLocal[i] - epLocalF;
            const float forward = rel.x * outwardDir.x + rel.y * outwardDir.y;
            if (forward < -1.f || forward > maxForward) {
                continue;
            }
            const float cross = rel.x * outwardDir.y - rel.y * outwardDir.x;
            const float side = std::abs(cross);
            if (side > maxSide) {
                continue;
            }

            const float curvatureBonus = 2.f * std::abs(curvature[i]);
            const float score = forward - 0.25f * side + curvatureBonus;
            if (score > bestScore) {
                bestScore = score;
                bestIdx = i;
            }
        }

        return bestIdx >= 0 ? bestIdx : nearestContourIdx(epLocalF);
    };

    // (e) ── Extend each skeleton endpoint to the strongest reachable peak ──
    for (int epIdx : r.skeleton.endpointIndices) {
        const cv::Point& epLocal = r.skeleton.points[epIdx];
        const cv::Point2f epLocalF(static_cast<float>(epLocal.x),
                                   static_cast<float>(epLocal.y));
        const cv::Point2f epWorld(epLocalF.x + originOffset.x,
                                  epLocalF.y + originOffset.y);

        const float dtAtEp = r.distTransform.at<float>(epLocal.y, epLocal.x);
        const cv::Point2f outwardDir = endpointOutwardDirection(epIdx);

        // Skeleton endpoint projected outward to the cap contour. A nearest
        // contour snap often lands on the side of a rounded tip instead of at
        // the end-cap apex.
        const int snapIdx = projectedEndpointContourIdx(epLocalF, outwardDir, dtAtEp);
        cv::Point2f snapLocal(0.f, 0.f);
        cv::Point2f snapWorld = epWorld;
        if (snapIdx >= 0) {
            snapLocal = contourLocal[snapIdx];
            snapWorld = cv::Point2f(snapLocal.x + originOffset.x,
                                    snapLocal.y + originOffset.y);
        }

        // Find a strong curvature peak in the local end-cap region defined by
        // the skeleton endpoint and its outward direction. This avoids making
        // curvature depend on a possibly side-biased contour snap.
        const float maxForward = std::clamp(6.f + 4.f * dtAtEp, 8.f, 18.f);
        const float maxSide = std::clamp(2.f + 2.5f * dtAtEp, 4.f, 10.f);
        int bestPeak = -1;
        float bestScore = -std::numeric_limits<float>::max();
        for (int peakIdx : curvaturePeakIdx) {
            const cv::Point2f& peakLocal = contourLocal[peakIdx];
            const cv::Point2f rel = peakLocal - epLocalF;
            float forward = 0.f;
            float side = std::sqrt(rel.x * rel.x + rel.y * rel.y);
            if (std::hypot(outwardDir.x, outwardDir.y) >= 1e-3f) {
                forward = rel.x * outwardDir.x + rel.y * outwardDir.y;
                const float cross = rel.x * outwardDir.y - rel.y * outwardDir.x;
                side = std::abs(cross);
            }
            if (forward < -1.f || forward > maxForward || side > maxSide) {
                continue;
            }

            const float score = forward - 0.25f * side + 4.f * std::abs(curvature[peakIdx]);
            if (score > bestScore) {
                bestScore = score;
                bestPeak = peakIdx;
            }
        }

        if (bestPeak >= 0) {
            const cv::Point2f peakDelta = contourLocal[bestPeak] - snapLocal;
            const float peakDist = std::hypot(peakDelta.x, peakDelta.y);
            const float maxPeakShift = std::clamp(1.0f + 0.75f * dtAtEp, 2.0f, 4.0f);
            if (peakDist > maxPeakShift) {
                bestPeak = -1;
            }
        }

        // ── (e2) Bilateral cap midpoint ───────────────────────────────────────
        //
        // Instead of relying on a single best-scored contour point (which can
        // jump 1–2 px between frames when the mask is imperfect), we split the
        // end-cap contour points into left-of-axis and right-of-axis halves,
        // then independently find the "apex" on each side — the point with the
        // greatest forward component along the outward direction. Taking the
        // midpoint of these two apexes gives a tip estimate that is robust to
        // one-sided mask noise because both sides' errors partially cancel.
        //
        // To further suppress single-pixel outliers we use a weighted centroid
        // of the top-forward fraction of cap points on each side rather than the
        // single furthest point. Points in the forward quartile (within
        // kFwdFraction of the side's own peak) contribute with weight
        // proportional to their forward depth, so genuine tip pixels dominate.
        //
        // A TipCapDebug is populated in parallel for the debug exporter.
        {
            constexpr float kFwdFraction = 0.25f; // include points within 25% of apex fwd

            const cv::Point2f perp(-outwardDir.y, outwardDir.x); // left of D

            // Debug snapshot — always populated so the exporter can show the
            // search window even when the bilateral computation falls through.
            TipCapDebug capDbg;
            capDbg.valid       = true;
            capDbg.skelEndpoint = snapWorld;
            capDbg.outwardDir  = outwardDir;
            capDbg.dtAtEp      = dtAtEp;
            capDbg.maxForward  = maxForward;
            capDbg.maxSide     = maxSide;
            capDbg.snapPoint       = snapWorld;
            capDbg.peakOrSnapPoint = snapWorld; // updated below if a peak is found
            capDbg.hadPeak         = false;

            float leftPeakFwd  = -std::numeric_limits<float>::max();
            float rightPeakFwd = -std::numeric_limits<float>::max();

            // First pass: find peak forward depth on each side and collect
            // all cap contour points for the debug snapshot.
            for (int i = 0; i < nContour; ++i) {
                const cv::Point2f rel = contourLocal[i] - epLocalF;
                const float fwd = rel.x * outwardDir.x + rel.y * outwardDir.y;
                if (fwd < -1.f || fwd > maxForward) continue;
                const float lat = rel.x * perp.x + rel.y * perp.y;
                if (std::abs(lat) > maxSide) continue;

                const cv::Point2f worldPt(contourLocal[i].x + originOffset.x,
                                          contourLocal[i].y + originOffset.y);
                if (lat >= 0.f) {
                    capDbg.leftCapPoints.push_back(worldPt);
                    if (fwd > leftPeakFwd) leftPeakFwd = fwd;
                }
                if (lat <= 0.f) {
                    capDbg.rightCapPoints.push_back(worldPt);
                    if (fwd > rightPeakFwd) rightPeakFwd = fwd;
                }
            }

            const bool hasLeft  = leftPeakFwd  > -std::numeric_limits<float>::max();
            const bool hasRight = rightPeakFwd > -std::numeric_limits<float>::max();
            capDbg.hasLeft       = hasLeft;
            capDbg.hasRight      = hasRight;
            capDbg.leftPeakFwd   = hasLeft  ? leftPeakFwd  : 0.f;
            capDbg.rightPeakFwd  = hasRight ? rightPeakFwd : 0.f;

            if (hasLeft && hasRight) {
                // Sanity: both sides' apexes should be at similar forward depths.
                // If one side is dramatically deeper the mask is degenerate; skip.
                const float fwdSpan = std::max(leftPeakFwd, rightPeakFwd) -
                                      std::min(leftPeakFwd, rightPeakFwd);
                const float fwdMean = 0.5f * (leftPeakFwd + rightPeakFwd);

                if (fwdSpan <= 0.6f * fwdMean + 2.f) {
                    capDbg.sanityPassed = true;

                    // Second pass: weighted centroid of points near each apex.
                    const float leftThresh  = leftPeakFwd  - kFwdFraction * leftPeakFwd;
                    const float rightThresh = rightPeakFwd - kFwdFraction * rightPeakFwd;

                    cv::Point2f leftSum(0.f, 0.f),  rightSum(0.f, 0.f);
                    float       leftWt = 0.f,        rightWt = 0.f;

                    for (int i = 0; i < nContour; ++i) {
                        const cv::Point2f rel = contourLocal[i] - epLocalF;
                        const float fwd = rel.x * outwardDir.x + rel.y * outwardDir.y;
                        if (fwd < -1.f || fwd > maxForward) continue;
                        const float lat = rel.x * perp.x + rel.y * perp.y;
                        if (std::abs(lat) > maxSide) continue;

                        if (lat >= 0.f && fwd >= leftThresh) {
                            const float w = fwd + 1.f; // weight by forward depth
                            leftSum += w * contourLocal[i];
                            leftWt  += w;
                        }
                        if (lat <= 0.f && fwd >= rightThresh) {
                            const float w = fwd + 1.f;
                            rightSum += w * contourLocal[i];
                            rightWt  += w;
                        }
                    }

                    if (leftWt > 0.f && rightWt > 0.f) {
                        const cv::Point2f leftCentroid  = leftSum  * (1.f / leftWt);
                        const cv::Point2f rightCentroid = rightSum * (1.f / rightWt);
                        const cv::Point2f midLocal = (leftCentroid + rightCentroid) * 0.5f;
                        const cv::Point2f bilateralWorld(midLocal.x + originOffset.x,
                                                         midLocal.y + originOffset.y);
                        const cv::Point2f leftApexWorld(leftCentroid.x + originOffset.x,
                                                        leftCentroid.y + originOffset.y);
                        const cv::Point2f rightApexWorld(rightCentroid.x + originOffset.x,
                                                         rightCentroid.y + originOffset.y);
                        capDbg.leftApex      = leftApexWorld;
                        capDbg.rightApex     = rightApexWorld;
                        capDbg.bilateralTip  = bilateralWorld;
                        capDbg.hasBilateral  = true;

                        TrueTip t;
                        t.skelPoint     = snapWorld;
                        t.bilateralTip  = bilateralWorld;
                        t.hasBilateral  = true;
                        if (bestPeak >= 0) {
                            const cv::Point2f peakLocal = contourLocal[bestPeak];
                            const cv::Point2f peakWorld(peakLocal.x + originOffset.x,
                                                        peakLocal.y + originOffset.y);
                            t.point     = peakWorld;
                            t.curvature = curvature[bestPeak];
                            t.width     = widthAt(peakLocal, bestPeak);
                            t.extended  = true;
                            capDbg.peakOrSnapPoint = peakWorld;
                            capDbg.hadPeak         = true;
                        } else {
                            t.point     = snapWorld;
                            t.curvature = (snapIdx >= 0) ? curvature[snapIdx] : 0.f;
                            t.width     = (snapIdx >= 0) ? widthAt(snapLocal, snapIdx) : 0.f;
                            t.extended  = false;
                        }
                        r.tips.push_back(t);
                        r.tipCapDebug.push_back(capDbg);
                        continue; // skip the fallback TrueTip construction below
                    }
                }
            }

            // Fallback path: bilateral not available. Fill in comparison fields.
            if (bestPeak >= 0) {
                const cv::Point2f peakWorld(contourLocal[bestPeak].x + originOffset.x,
                                            contourLocal[bestPeak].y + originOffset.y);
                capDbg.peakOrSnapPoint = peakWorld;
                capDbg.hadPeak         = true;
            }
            r.tipCapDebug.push_back(capDbg);
        }

        // Fallback (no valid bilateral): construct TrueTip with snap/peak only.
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
        r.topology = Tracking::TopologyState::Merged;
    } else {
        const bool hasRing = !blob.holeContourPoints.empty();
        r.topology = (hasRing || r.tips.size() < 2)
                         ? Tracking::TopologyState::SelfCrossed
                         : Tracking::TopologyState::Clean;
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
            qUtf8Printable(Tracking::topologyStateToString(r.topology)),
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

} // namespace Centerline

// ── geometry helpers ────────────────────────────────────────────────────────

// Return Euclidean distance between two image-space points.
static float ptDist(const cv::Point2f& a, const cv::Point2f& b)
{
    float dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Measure the total length of an ordered polyline.
static float arcLen(const std::vector<cv::Point2f>& pts)
{
    float len = 0.f;
    for (size_t i = 1; i < pts.size(); ++i)
        len += ptDist(pts[i - 1], pts[i]);
    return len;
}

// Find the contour vertex closest to a target point.
static int nearestContourIdx(const std::vector<cv::Point>& contour,
                             const cv::Point2f& target)
{
    int best = 0;
    float bestD = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(contour.size()); ++i) {
        float dx = contour[i].x - target.x;
        float dy = contour[i].y - target.y;
        float d  = dx * dx + dy * dy;
        if (d < bestD) { bestD = d; best = i; }
    }
    return best;
}

// Return the contour point closest to a target point.
static cv::Point2f nearestContourPoint(const std::vector<cv::Point>& contour,
                                       const cv::Point2f& target)
{
    const int idx = nearestContourIdx(contour, target);
    return cv::Point2f(static_cast<float>(contour[idx].x),
                       static_cast<float>(contour[idx].y));
}

// Resample an ordered polyline to a fixed number of evenly spaced points.
static std::vector<cv::Point2f> resample(const std::vector<cv::Point2f>& pts, int nPoints)
{
    if (static_cast<int>(pts.size()) <= 1 || nPoints < 2) return pts;
    std::vector<float> cum(pts.size(), 0.f);
    for (size_t i = 1; i < pts.size(); ++i)
        cum[i] = cum[i - 1] + ptDist(pts[i - 1], pts[i]);
    float total = cum.back();
    if (total < 1e-6f) return pts;

    std::vector<cv::Point2f> out(nPoints);
    out.front() = pts.front();
    out.back()  = pts.back();
    for (int k = 1; k < nPoints - 1; ++k) {
        float target = total * k / (nPoints - 1);
        auto it  = std::lower_bound(cum.begin(), cum.end(), target);
        size_t j = std::min<size_t>(std::distance(cum.begin(), it), pts.size() - 1);
        if (j == 0) { out[k] = pts.front(); continue; }
        float t = (target - cum[j - 1]) / (cum[j] - cum[j - 1] + 1e-9f);
        out[k]  = pts[j - 1] + t * (pts[j] - pts[j - 1]);
    }
    return out;
}

// Sum of consecutive 2D segment cross products along a polyline:
//   vi = p[i+1] - p[i]
//   sum += vi.x * v(i+1).y - v(i+1).x * vi.y
// The sign captures clockwise/counterclockwise bend sense and flips when the
// traversal direction is reversed. Returns 0 for fewer than 3 points.
static float centerlineCrossSum(const std::vector<cv::Point2f>& pts)
{
    float total = 0.f;
    const int n = static_cast<int>(pts.size());
    for (int i = 1; i < n - 1; ++i) {
        const cv::Point2f v1 = pts[i]     - pts[i - 1];
        const cv::Point2f v2 = pts[i + 1] - pts[i];
        total += v1.x * v2.y - v2.x * v1.y;
    }
    return total;
}

// Convert a blob's QPointF centroid into OpenCV point coordinates.
static cv::Point2f blobCentroid(const Tracking::DetectedBlob& blob)
{
    return cv::Point2f(static_cast<float>(blob.centroid.x()),
                       static_cast<float>(blob.centroid.y()));
}

// Choose the most reliable endpoint coordinate for a clean two-tip centerline.
static cv::Point2f trustedTipPointForCleanD1(const Centerline::TrueTip& tip)
{
    // Bilateral cap midpoint is the most stable estimate: it averages the
    // furthest-forward cap contour points on both sides of the body axis, so
    // single-pixel mask noise on one side is partially cancelled by the other.
    // Use it whenever it was successfully computed.
    if (tip.hasBilateral)
        return tip.bilateralTip;

    // Fallback: use the curvature-peak position only when it sits close to
    // the raw contour snap (large shifts indicate the peak landed on a body
    // kink rather than the genuine end-cap).
    constexpr float kMaxCleanD1Extension = 6.f;
    if (!tip.extended || ptDist(tip.point, tip.skelPoint) > kMaxCleanD1Extension)
        return tip.skelPoint;
    return tip.point;
}

// Rasterize a blob's outer contour and holes into a local binary mask.
static void fillBlobMask(cv::Mat& mask,
                         const Tracking::DetectedBlob& blob,
                         const cv::Rect& bounds,
                         const cv::Point2f& offset)
{
    std::vector<std::vector<cv::Point>> outerContours(1);
    outerContours.front().reserve(blob.contourPoints.size());
    for (const cv::Point& pt : blob.contourPoints) {
        outerContours.front().push_back(cv::Point(
            static_cast<int>(std::lround(static_cast<float>(pt.x) + offset.x - bounds.x)),
            static_cast<int>(std::lround(static_cast<float>(pt.y) + offset.y - bounds.y))));
    }
    cv::fillPoly(mask, outerContours, cv::Scalar(255));

    for (const std::vector<cv::Point>& hole : blob.holeContourPoints) {
        std::vector<std::vector<cv::Point>> holeContour(1);
        holeContour.front().reserve(hole.size());
        for (const cv::Point& pt : hole) {
            holeContour.front().push_back(cv::Point(
                static_cast<int>(std::lround(static_cast<float>(pt.x) + offset.x - bounds.x)),
                static_cast<int>(std::lround(static_cast<float>(pt.y) + offset.y - bounds.y))));
        }
        cv::fillPoly(mask, holeContour, cv::Scalar(0));
    }
}

// Snap a target point to the nearest foreground pixel in a blob mask.
static bool nearestMaskPoint(const Tracking::DetectedBlob& blob,
                             const cv::Point2f& target,
                             cv::Point2f& outPoint)
{
    if (blob.contourPoints.empty()) return false;

    cv::Rect bounds = cv::boundingRect(blob.contourPoints);
    constexpr int kPad = 8;
    bounds.x -= kPad;
    bounds.y -= kPad;
    bounds.width += 2 * kPad;
    bounds.height += 2 * kPad;
    if (bounds.width <= 1 || bounds.height <= 1) return false;

    cv::Mat mask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    fillBlobMask(mask, blob, bounds, cv::Point2f(0.f, 0.f));

    const int tx = static_cast<int>(std::lround(target.x - bounds.x));
    const int ty = static_cast<int>(std::lround(target.y - bounds.y));
    if (tx >= 0 && ty >= 0 && tx < mask.cols && ty < mask.rows &&
        mask.at<uchar>(ty, tx) != 0) {
        outPoint = cv::Point2f(static_cast<float>(tx + bounds.x),
                               static_cast<float>(ty + bounds.y));
        return true;
    }

    float bestD2 = std::numeric_limits<float>::max();
    bool found = false;
    for (int y = 0; y < mask.rows; ++y) {
        const uchar* row = mask.ptr<uchar>(y);
        for (int x = 0; x < mask.cols; ++x) {
            if (row[x] == 0) continue;
            const float wx = static_cast<float>(x + bounds.x);
            const float wy = static_cast<float>(y + bounds.y);
            const float dx = wx - target.x;
            const float dy = wy - target.y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < bestD2) {
                bestD2 = d2;
                outPoint = cv::Point2f(wx, wy);
                found = true;
            }
        }
    }
    return found;
}

// Reassign two self-crossed tip candidates using predictor position and velocity.
static bool enforceSelfCrossedTwoTipPredictorRoles(
    Tracking::DetectedBlob& blob,
    const Centerline::HeadTailPredictor& predictor,
    QStringList* diagnostics)
{
    if (blob.topologyState != Tracking::TopologyState::SelfCrossed ||
        blob.tipCandidates.size() != 2 ||
        !predictor.hasPrev) {
        return false;
    }

    auto distSq = [](const cv::Point2f& a, const cv::Point2f& b) -> float {
        const cv::Point2f d = a - b;
        return d.x * d.x + d.y * d.y;
    };

    const cv::Point2f predHead = predictor.hasVelocity
        ? predictor.lastHeadPos + predictor.velHead
        : predictor.lastHeadPos;
    const cv::Point2f predTail = predictor.hasVelocity
        ? predictor.lastTailPos + predictor.velTail
        : predictor.lastTailPos;

    auto assignmentCost = [&](int headIdx, int tailIdx) -> float {
        const cv::Point2f& h = blob.tipCandidates[headIdx].point;
        const cv::Point2f& t = blob.tipCandidates[tailIdx].point;

        float cost = distSq(h, predHead) + distSq(t, predTail);
        cost += 0.5f * (distSq(h, predictor.lastHeadPos) +
                        distSq(t, predictor.lastTailPos));
        if (predictor.hasVelocity) {
            cost += 0.25f * (distSq(h - predictor.lastHeadPos, predictor.velHead) +
                             distSq(t - predictor.lastTailPos, predictor.velTail));
        }
        return cost;
    };

    const float cost01 = assignmentCost(0, 1);
    const float cost10 = assignmentCost(1, 0);
    const int oldHead = blob.assignedHeadTipIdx;
    const int oldTail = blob.assignedTailTipIdx;

    if (cost01 <= cost10) {
        blob.assignedHeadTipIdx = 0;
        blob.assignedTailTipIdx = 1;
    } else {
        blob.assignedHeadTipIdx = 1;
        blob.assignedTailTipIdx = 0;
    }

    const bool changed =
        oldHead != blob.assignedHeadTipIdx ||
        oldTail != blob.assignedTailTipIdx;
    if (diagnostics) {
        diagnostics->append(
            QStringLiteral("SelfCrossed two-tip predictor role check cost01=%1 cost10=%2 selected headIdx=%3 tailIdx=%4%5")
                .arg(cost01, 0, 'f', 2)
                .arg(cost10, 0, 'f', 2)
                .arg(blob.assignedHeadTipIdx)
                .arg(blob.assignedTailTipIdx)
                .arg(changed ? QStringLiteral(" reassigned") : QString()));
    }
    return changed;
}

// Reassign two tip candidates by comparing them with previous centerline order.
static bool enforceTwoTipCenterlineOrderRoles(
    Tracking::DetectedBlob& blob,
    const std::vector<cv::Point2f>& previousCenterline,
    const cv::Point2f& previousCentroid,
    QStringList* diagnostics)
{
    if (blob.tipCandidates.size() != 2 || previousCenterline.size() < 2) {
        return false;
    }

    const cv::Point2f delta = blobCentroid(blob) - previousCentroid;
    std::vector<cv::Point2f> translatedPrev;
    translatedPrev.reserve(previousCenterline.size());
    for (const cv::Point2f& p : previousCenterline) {
        translatedPrev.push_back(p + delta);
    }

    std::vector<float> cumulative(translatedPrev.size(), 0.f);
    for (int i = 1; i < static_cast<int>(translatedPrev.size()); ++i) {
        cumulative[i] = cumulative[i - 1] + ptDist(translatedPrev[i - 1], translatedPrev[i]);
    }
    const float total = cumulative.back();
    if (total <= 1e-6f) {
        return false;
    }

    struct Projection {
        float fraction = 0.f;
        float distSq = std::numeric_limits<float>::max();
    };

    auto projectOntoPreviousOrder = [&](const cv::Point2f& q) -> Projection {
        Projection best;
        for (int i = 1; i < static_cast<int>(translatedPrev.size()); ++i) {
            const cv::Point2f a = translatedPrev[i - 1];
            const cv::Point2f b = translatedPrev[i];
            const cv::Point2f ab = b - a;
            const float lenSq = ab.x * ab.x + ab.y * ab.y;
            float t = 0.f;
            if (lenSq > 1e-6f) {
                const cv::Point2f aq = q - a;
                t = std::clamp((aq.x * ab.x + aq.y * ab.y) / lenSq, 0.f, 1.f);
            }
            const cv::Point2f proj = a + t * ab;
            const cv::Point2f d = q - proj;
            const float distSq = d.x * d.x + d.y * d.y;
            if (distSq < best.distSq) {
                best.distSq = distSq;
                const float along = cumulative[i - 1] +
                                    t * (cumulative[i] - cumulative[i - 1]);
                best.fraction = along / total;
            }
        }
        return best;
    };

    const Projection p0 = projectOntoPreviousOrder(blob.tipCandidates[0].point);
    const Projection p1 = projectOntoPreviousOrder(blob.tipCandidates[1].point);
    const float cost01 = p0.fraction * p0.fraction +
                         (1.f - p1.fraction) * (1.f - p1.fraction);
    const float cost10 = p1.fraction * p1.fraction +
                         (1.f - p0.fraction) * (1.f - p0.fraction);
    const int oldHead = blob.assignedHeadTipIdx;
    const int oldTail = blob.assignedTailTipIdx;

    if (cost01 <= cost10) {
        blob.assignedHeadTipIdx = 0;
        blob.assignedTailTipIdx = 1;
    } else {
        blob.assignedHeadTipIdx = 1;
        blob.assignedTailTipIdx = 0;
    }

    const bool changed =
        oldHead != blob.assignedHeadTipIdx ||
        oldTail != blob.assignedTailTipIdx;
    if (diagnostics) {
        diagnostics->append(
            QStringLiteral("two-tip centerline-order role check s0=%1 d0=%2 s1=%3 d1=%4 cost01=%5 cost10=%6 selected headIdx=%7 tailIdx=%8%9")
                .arg(p0.fraction, 0, 'f', 3)
                .arg(std::sqrt(p0.distSq), 0, 'f', 2)
                .arg(p1.fraction, 0, 'f', 3)
                .arg(std::sqrt(p1.distSq), 0, 'f', 2)
                .arg(cost01, 0, 'f', 4)
                .arg(cost10, 0, 'f', 4)
                .arg(blob.assignedHeadTipIdx)
                .arg(blob.assignedTailTipIdx)
                .arg(changed ? QStringLiteral(" reassigned") : QString()));
    }
    return changed;
}

// Pick a zero-tip ring centerline by trying plausible cut lines through the hole.
static bool selectZeroTipRingCutCenterline(
    const Tracking::DetectedBlob& blob,
    const cv::Point2f& predictedHead,
    const cv::Point2f& predictedTail,
    bool hasPredictedTips,
    const cv::Point2f& predictedCenter,
    bool hasPredictedCenter,
    float previousCrossSum,
    float refLength,
    QStringList* diagnostics,
    std::vector<cv::Point2f>& outCenterline,
    cv::Point2f& outCutPoint)
{
    outCenterline.clear();
    outCutPoint = cv::Point2f(-1.f, -1.f);
    if (blob.holeContourPoints.empty()) {
        return false;
    }

    struct CutCandidate {
        QString label;
        cv::Point2f a;
        cv::Point2f b;
    };

    std::vector<CutCandidate> cuts;
    if (hasPredictedTips) {
        cuts.push_back({QStringLiteral("predicted tips"), predictedHead, predictedTail});
    }

    const std::vector<cv::Point>& hole = blob.holeContourPoints.front();
    if (hole.size() >= 5) {
        const cv::RotatedRect rr = cv::fitEllipse(hole);
        const float ang = static_cast<float>(rr.angle * CV_PI / 180.0);
        const float halfMajor = std::max(rr.size.width, rr.size.height) * 0.5f + 6.f;
        const cv::Point2f dir(std::cos(ang), std::sin(ang));
        cuts.push_back({QStringLiteral("hole ellipse"),
                        rr.center - dir * halfMajor,
                        rr.center + dir * halfMajor});
    }

    struct ScoredCandidate {
        std::vector<cv::Point2f> points;
        cv::Point2f cutPoint = {-1.f, -1.f};
        QString label;
        float len = 0.f;
        float endpointDist = 0.f;
        float centerDist = 0.f;
        float crossSum = 0.f;
        float crossPenalty = 0.f;
        float score = std::numeric_limits<float>::max();
    };

    std::vector<ScoredCandidate> scored;
    for (const CutCandidate& cut : cuts) {
        Tracking::DetectedBlob cutBlob = blob;
        if (!Tracking::populateCenterlineFromContourWithCut(cutBlob, cut.a, cut.b, 3) ||
            cutBlob.centerlinePoints.size() < 2) {
            if (diagnostics) {
                diagnostics->append(QStringLiteral("0-tip ring cut candidate %1 failed")
                                        .arg(cut.label));
            }
            continue;
        }

        ScoredCandidate c;
        c.label = cut.label;
        c.cutPoint = (cut.a + cut.b) * 0.5f;
        c.points.assign(cutBlob.centerlinePoints.begin(), cutBlob.centerlinePoints.end());
        if (hasPredictedTips) {
            const float forward = ptDist(c.points.front(), predictedHead) +
                                  ptDist(c.points.back(), predictedTail);
            const float reversed = ptDist(c.points.front(), predictedTail) +
                                   ptDist(c.points.back(), predictedHead);
            if (reversed < forward) {
                std::reverse(c.points.begin(), c.points.end());
            }
            c.endpointDist = std::min(forward, reversed);
        }
        c.len = arcLen(c.points);
        if (hasPredictedCenter && c.points.size() >= 2) {
            c.centerDist = ptDist(c.points[c.points.size() / 2], predictedCenter);
        }
        c.crossSum = centerlineCrossSum(c.points);
        constexpr float kCrossSumEpsilon = 1e-4f;
        if (std::abs(previousCrossSum) > kCrossSumEpsilon &&
            std::abs(c.crossSum) > kCrossSumEpsilon) {
            c.crossPenalty = (c.crossSum * previousCrossSum > 0.f) ? 0.f : 1.f;
        }

        constexpr float kEndpointWeight = 3.0f;
        constexpr float kCenterWeight = 2.0f;
        constexpr float kCrossMismatchWeight = 100.0f;
        constexpr float kLengthWeight = 1.0f;
        c.score = kEndpointWeight * c.endpointDist +
                  kCenterWeight * c.centerDist +
                  kCrossMismatchWeight * c.crossPenalty;
        if (refLength > 0.f) {
            c.score += kLengthWeight * std::abs(c.len - refLength);
        }
        scored.push_back(std::move(c));
    }

    if (scored.empty()) {
        return false;
    }

    int bestIdx = 0;
    for (int i = 1; i < static_cast<int>(scored.size()); ++i) {
        if (scored[i].score < scored[bestIdx].score) {
            bestIdx = i;
        }
    }

    if (diagnostics) {
        diagnostics->append(QStringLiteral("0-tip ring cut candidates=%1 selected=%2")
                                .arg(static_cast<int>(scored.size()))
                                .arg(bestIdx));
        for (int i = 0; i < static_cast<int>(scored.size()); ++i) {
            const ScoredCandidate& c = scored[i];
            diagnostics->append(
                QStringLiteral("0-tip candidate %1 %2: len=%3 endpointDist=%4 centerDist=%5 crossSum=%6 crossPenalty=%7 score=%8%9")
                    .arg(i)
                    .arg(c.label)
                    .arg(c.len, 0, 'f', 2)
                    .arg(c.endpointDist, 0, 'f', 2)
                    .arg(c.centerDist, 0, 'f', 2)
                    .arg(c.crossSum, 0, 'f', 4)
                    .arg(c.crossPenalty, 0, 'f', 1)
                    .arg(c.score, 0, 'f', 2)
                    .arg(i == bestIdx ? QStringLiteral(" SELECTED") : QString()));
        }
    }

    outCenterline = scored[bestIdx].points;
    outCutPoint = scored[bestIdx].cutPoint;
    return outCenterline.size() >= 2;
}

// Estimate where a hidden tip emerged by differencing current and previous masks.
static bool hiddenTipMaskDifferenceCue(const Tracking::DetectedBlob& current,
                                       const Tracking::DetectedBlob& previous,
                                       const cv::Point2f& hiddenLast,
                                       const cv::Point2f& velocityPrediction,
                                       cv::Point2f& outCue,
                                       int* outTotalArea = nullptr,
                                       int* outSelectedArea = nullptr)
{
    if (outTotalArea) *outTotalArea = 0;
    if (outSelectedArea) *outSelectedArea = 0;
    if (current.contourPoints.empty() || previous.contourPoints.empty() ||
        hiddenLast.x < 0.f) {
        return false;
    }

    cv::Rect bounds = cv::boundingRect(current.contourPoints) |
                      cv::boundingRect(previous.contourPoints);
    constexpr int kPad = 8;
    bounds.x -= kPad;
    bounds.y -= kPad;
    bounds.width += 2 * kPad;
    bounds.height += 2 * kPad;
    if (bounds.width <= 1 || bounds.height <= 1) {
        return false;
    }

    cv::Mat currentMask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    cv::Mat previousMask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    fillBlobMask(currentMask, current, bounds, cv::Point2f(0.f, 0.f));
    fillBlobMask(previousMask, previous, bounds, cv::Point2f(0.f, 0.f));

    cv::Mat inversePrevious;
    cv::bitwise_not(previousMask, inversePrevious);
    cv::Mat enteredMask;
    cv::bitwise_and(currentMask, inversePrevious, enteredMask);
    if (outTotalArea) {
        *outTotalArea = cv::countNonZero(enteredMask);
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int componentCount =
        cv::connectedComponentsWithStats(enteredMask, labels, stats, centroids, 8, CV_32S);
    if (componentCount <= 1) {
        return false;
    }

    int bestLabel = -1;
    float bestScore = std::numeric_limits<float>::max();
    for (int label = 1; label < componentCount; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < 2) {
            continue;
        }

        const cv::Point2f c(
            static_cast<float>(centroids.at<double>(label, 0) + bounds.x),
            static_cast<float>(centroids.at<double>(label, 1) + bounds.y));
        const float dLast = ptDist(c, hiddenLast);
        const float dVel = (velocityPrediction.x >= 0.f)
            ? ptDist(c, velocityPrediction)
            : dLast;
        const float score = dLast + 0.35f * dVel - 0.20f * std::sqrt(static_cast<float>(area));
        if (score < bestScore) {
            bestScore = score;
            bestLabel = label;
        }
    }
    if (bestLabel < 0) {
        return false;
    }
    if (outSelectedArea) {
        *outSelectedArea = stats.at<int>(bestLabel, cv::CC_STAT_AREA);
    }

    cv::Point2f weightedCenter(0.f, 0.f);
    float totalWeight = 0.f;
    for (int y = 0; y < labels.rows; ++y) {
        const int* row = labels.ptr<int>(y);
        for (int x = 0; x < labels.cols; ++x) {
            if (row[x] != bestLabel) {
                continue;
            }
            const cv::Point2f p(static_cast<float>(x + bounds.x),
                                static_cast<float>(y + bounds.y));
            const float d = ptDist(p, hiddenLast);
            const float w = 1.f / std::max(1.f, d);
            weightedCenter += w * p;
            totalWeight += w;
        }
    }
    if (totalWeight <= 0.f) {
        return false;
    }
    weightedCenter *= 1.f / totalWeight;

    return nearestMaskPoint(current, weightedCenter, outCue);
}

struct HiddenTipTarget {
    cv::Point2f target = {-1.f, -1.f};
    cv::Point2f velocityTarget = {-1.f, -1.f};
    cv::Point2f maskCue = {-1.f, -1.f};
    int maskDiffArea = 0;
    int selectedMaskDiffArea = 0;
    bool hasTarget = false;
    bool hasMaskCue = false;
};

// Predict the target point for a hidden tip from velocity and mask-difference cues.
static HiddenTipTarget predictHiddenTipTarget(const Tracking::DetectedBlob& current,
                                              const Tracking::DetectedBlob* previous,
                                              const cv::Point2f& last,
                                              const cv::Point2f& velocity,
                                              bool hasVelocity)
{
    HiddenTipTarget result;
    if (last.x == 0.f && last.y == 0.f) {
        return result;
    }

    result.velocityTarget = hasVelocity ? (last + velocity) : last;
    result.target = result.velocityTarget;
    result.hasTarget = true;

    if (previous && previous->isValid &&
        hiddenTipMaskDifferenceCue(current,
                                   *previous,
                                   last,
                                   result.velocityTarget,
                                   result.maskCue,
                                   &result.maskDiffArea,
                                   &result.selectedMaskDiffArea)) {
        constexpr float kMaskCueWeight = 0.75f;
        result.target = kMaskCueWeight * result.maskCue +
                        (1.f - kMaskCueWeight) * result.velocityTarget;
        result.hasMaskCue = true;
    }

    cv::Point2f snappedTarget;
    if (nearestMaskPoint(current, result.target, snappedTarget)) {
        result.target = snappedTarget;
    }

    return result;
}

// ── Skeleton-graph shortest path (used by the new Clean centerline branch) ─
//
// Run Dijkstra on a prepared skeleton graph and return a world-coordinate
// start-to-goal path for the clean centerline branch.
static bool skeletonGraphPath(const Centerline::SkeletonGraph& graph,
                              int startIdx, int goalIdx,
                              const cv::Point2f& originOffset,
                              std::vector<cv::Point2f>& outPath)
{
    outPath.clear();
    if (startIdx < 0 || goalIdx < 0 ||
        startIdx >= static_cast<int>(graph.points.size()) ||
        goalIdx  >= static_cast<int>(graph.points.size())) return false;
    if (startIdx == goalIdx) return false;

    const Centerline::GraphSearchResult search =
        Centerline::dijkstraSkeleton(graph.points, graph.adjacency, startIdx);
    if (!std::isfinite(search.distances[goalIdx])) return false;

    outPath = Centerline::reconstructCenterlinePath(graph.points, search.parents,
                                                    startIdx, goalIdx, originOffset);
    return outPath.size() >= 2;
}

// ── Skeleton-arc helpers for SelfCrossed dispatch (D-2 / D-3) ──────────────
//
// For a ring or coiled worm the shortest skeleton path goes the wrong way —
// it takes the short arc across the ring rather than tracing the body axis
// all the way around. The three helpers below let the caller enumerate BOTH
// arcs of the skeleton between a source and a goal node, then pick the arc
// whose consecutive-segment cross-sum sign matches the previous frame's
// right-hand-rule state, falling back to arc-length proximity to refLength
// when no previous state is available.

// Nearest skeleton graph node (by squared Euclidean distance) to a world point.
// Find the skeleton graph node nearest to a world-coordinate point.
static int nearestSkeletonNode(const Centerline::SkeletonGraph& graph,
                                const cv::Point2f& worldPt,
                                const cv::Point2f& originOffset)
{
    int best = -1;
    float bestD2 = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(graph.points.size()); ++i) {
        const cv::Point& p = graph.points[i];
        const float dx = (p.x + originOffset.x) - worldPt.x;
        const float dy = (p.y + originOffset.y) - worldPt.y;
        const float d2 = dx * dx + dy * dy;
        if (d2 < bestD2) { bestD2 = d2; best = i; }
    }
    return best;
}

// Skeleton node that is geodesically farthest from srcIdx (Dijkstra on graph).
// Useful as a D-3 fallback when the predictor has no previous position.
// Find the reachable skeleton node farthest from a source node.
static int farthestSkeletonNode(const Centerline::SkeletonGraph& graph, int srcIdx)
{
    const int n = static_cast<int>(graph.points.size());
    if (srcIdx < 0 || srcIdx >= n) return -1;
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    dist[srcIdx] = 0.0;
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    pq.push({0.0, srcIdx});
    while (!pq.empty()) {
        const auto [d, cur] = pq.top();
        pq.pop();
        if (d > dist[cur]) continue;
        const cv::Point& cp = graph.points[cur];
        for (int nb : graph.adjacency[cur]) {
            const cv::Point& np = graph.points[nb];
            const int dx = np.x - cp.x, dy = np.y - cp.y;
            const double w = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            const double nd = d + w;
            if (nd < dist[nb]) { dist[nb] = nd; pq.push({nd, nb}); }
        }
    }
    int best = srcIdx;
    double bestD = 0.0;
    for (int i = 0; i < n; ++i) {
        if (std::isfinite(dist[i]) && dist[i] > bestD) { bestD = dist[i]; best = i; }
    }
    return best;
}

// Enumerate both skeleton arcs between srcIdx and goalIdx.
//   outA = Dijkstra shortest arc.
//   outB = complementary arc (Dijkstra with the first hop of A forbidden).
// Returns true when both arcs exist (≥2 points each).  Returns false for
// open-curve skeletons where srcIdx has only one neighbour — the caller
// should then just use the single path from skeletonGraphPath().
// Split a cyclic skeleton into the two arcs connecting a source and goal node.
static bool skeletonBothArcs(const Centerline::SkeletonGraph& graph,
                              int srcIdx, int goalIdx,
                              const cv::Point2f& originOffset,
                              std::vector<cv::Point2f>& outA,
                              std::vector<cv::Point2f>& outB)
{
    outA.clear();
    outB.clear();
    const int N = static_cast<int>(graph.points.size());
    if (srcIdx < 0 || goalIdx < 0 || srcIdx == goalIdx ||
        srcIdx >= N || goalIdx >= N) return false;

    using Entry = std::pair<double, int>;

    // ── Arc A: standard Dijkstra, keeping parent indices so we can
    //    reconstruct the node-index sequence (not just world coords).
    std::vector<double> distA(N, std::numeric_limits<double>::infinity());
    std::vector<int>    parentA(N, -1);
    distA[srcIdx] = 0.0;
    {
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
        pq.push({0.0, srcIdx});
        while (!pq.empty()) {
            const auto [d, cur] = pq.top();
            pq.pop();
            if (d > distA[cur]) continue;
            if (cur == goalIdx) break;
            const cv::Point& cp = graph.points[cur];
            for (int nb : graph.adjacency[cur]) {
                const cv::Point& np = graph.points[nb];
                const int dx = np.x - cp.x, dy = np.y - cp.y;
                const double w = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
                const double nd = d + w;
                if (nd < distA[nb]) { distA[nb] = nd; parentA[nb] = cur; pq.push({nd, nb}); }
            }
        }
    }
    if (!std::isfinite(distA[goalIdx])) return false;

    // Reconstruct arcA as a sequence of graph node indices.
    std::vector<int> arcAIdx;
    for (int cur = goalIdx; cur != -1; ) {
        arcAIdx.push_back(cur);
        if (cur == srcIdx) break;
        cur = parentA[cur];
    }
    std::reverse(arcAIdx.begin(), arcAIdx.end());
    if (arcAIdx.empty() || arcAIdx.front() != srcIdx) return false;

    outA.clear();
    for (int idx : arcAIdx) {
        const cv::Point& p = graph.points[idx];
        outA.emplace_back(p.x + originOffset.x, p.y + originOffset.y);
    }

    // ── Find the blocking edge for arc B ─────────────────────────────────
    //
    // We want arc B to take the alternative path through the ring.  Blocking
    // the edge from srcIdx only works when srcIdx itself has ≥2 neighbours
    // (pure ring).  For a ring-with-protrusion the tip (srcIdx) is a degree-1
    // node: it has one neighbour, the protrusion base, so there is no choice
    // at srcIdx.  The choice happens at the JUNCTION — the first node on
    // arcA that has degree ≥ 3 (where the protrusion meets the ring loop).
    //
    // Blocking the arc A edge at the junction forces Dijkstra to go around
    // the other side of the ring for arcB, which is what we want.
    //
    // Fallback: if arcA has no junction (open curve or pure ring), block at
    // srcIdx as before.  A pure ring has every node at degree 2, so the
    // junction search fails and we fall back to blocking at srcIdx — which
    // IS valid for a pure ring because srcIdx has degree 2.
    int blockFrom = -1, blockTo = -1;
    for (size_t k = 0; k + 1 < arcAIdx.size(); ++k) {
        if (static_cast<int>(graph.adjacency[arcAIdx[k]].size()) >= 3) {
            blockFrom = arcAIdx[k];
            blockTo   = arcAIdx[k + 1];
            break;
        }
    }
    if (blockFrom < 0) {
        // No junction found: pure ring or open curve.
        // For a pure ring every node has degree 2 so blocking at srcIdx works.
        // For an open curve (degree-1 src, no ring) blockTo stays -1 and the
        // second Dijkstra will fail, correctly returning false.
        blockFrom = srcIdx;
        blockTo   = (arcAIdx.size() >= 2) ? arcAIdx[1] : -1;
    }
    if (blockTo < 0) return false;

    // ── Arc B: Dijkstra with blockFrom→blockTo forbidden ─────────────────
    std::vector<double> distB(N, std::numeric_limits<double>::infinity());
    std::vector<int>    parentB(N, -1);
    distB[srcIdx] = 0.0;
    {
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
        pq.push({0.0, srcIdx});
        while (!pq.empty()) {
            const auto [d, cur] = pq.top();
            pq.pop();
            if (d > distB[cur]) continue;
            if (cur == goalIdx) break;
            const cv::Point& cp = graph.points[cur];
            for (int nb : graph.adjacency[cur]) {
                if (cur == blockFrom && nb == blockTo) continue; // forbidden
                const cv::Point& np = graph.points[nb];
                const int ddx = np.x - cp.x, ddy = np.y - cp.y;
                const double w = (ddx != 0 && ddy != 0) ? std::sqrt(2.0) : 1.0;
                const double nd = d + w;
                if (nd < distB[nb]) { distB[nb] = nd; parentB[nb] = cur; pq.push({nd, nb}); }
            }
        }
    }
    if (!std::isfinite(distB[goalIdx])) return false;

    std::vector<cv::Point2f> revB;
    for (int cur = goalIdx; cur != -1; ) {
        const cv::Point& p = graph.points[cur];
        revB.emplace_back(p.x + originOffset.x, p.y + originOffset.y);
        if (cur == srcIdx) break;
        cur = parentB[cur];
        if (cur == -1) return false;
    }
    outB.assign(revB.rbegin(), revB.rend());
    return outB.size() >= 2;
}

// Pick one of two skeleton arcs.
// Primary selector: the arc whose consecutive-segment cross-sum matches the
// sign of the previous frame's cross-sum. Fallback when the previous cross-sum
// is too close to zero or both/neither match: pick the arc whose arcLen is
// closer to refLength.
// Last resort: return arcA.
// Choose between two candidate arcs using turn direction and length heuristics.
static std::vector<cv::Point2f> pickArcByRHR(
    const std::vector<cv::Point2f>& arcA,
    const std::vector<cv::Point2f>& arcB,
    float prevTurningAngle,
    float angleThreshold,
    float refLength)
{
    const float turA = centerlineCrossSum(arcA);
    const float turB = centerlineCrossSum(arcB);
    constexpr float kCrossSumEpsilon = 1e-4f;
    (void)angleThreshold;

    if (std::abs(prevTurningAngle) > kCrossSumEpsilon) {
        const bool aOk = (turA * prevTurningAngle > 0.f);
        const bool bOk = (turB * prevTurningAngle > 0.f);
        if (aOk && !bOk) return arcA;
        if (bOk && !aOk) return arcB;
        // Both or neither match sign — fall through to length.
    }
    if (refLength > 0.f) {
        const float lenA = arcLen(arcA);
        const float lenB = arcLen(arcB);
        return (std::abs(lenA - refLength) <= std::abs(lenB - refLength)) ? arcA : arcB;
    }
    return arcA;
}

// Convert a skeleton node-index path into world-coordinate points.
static std::vector<cv::Point2f> nodesToWorldPath(const Centerline::SkeletonGraph& graph,
                                                 const std::vector<int>& nodePath,
                                                 const cv::Point2f& originOffset)
{
    std::vector<cv::Point2f> path;
    path.reserve(nodePath.size());
    for (int idx : nodePath) {
        const cv::Point& p = graph.points[idx];
        path.emplace_back(static_cast<float>(p.x) + originOffset.x,
                          static_cast<float>(p.y) + originOffset.y);
    }
    return path;
}

// Measure the weighted path length for a skeleton node-index path.
static float nodePathLength(const Centerline::SkeletonGraph& graph,
                            const std::vector<int>& nodePath,
                            int endExclusive = -1)
{
    if (nodePath.size() < 2) return 0.f;
    const int limit = endExclusive < 0
        ? static_cast<int>(nodePath.size())
        : std::min(endExclusive, static_cast<int>(nodePath.size()));
    float len = 0.f;
    for (int i = 1; i < limit; ++i) {
        const cv::Point& a = graph.points[nodePath[i - 1]];
        const cv::Point& b = graph.points[nodePath[i]];
        const int dx = b.x - a.x;
        const int dy = b.y - a.y;
        len += (dx != 0 && dy != 0) ? std::sqrt(2.f) : 1.f;
    }
    return len;
}

// Find the shortest skeleton node path between two graph nodes.
static bool shortestNodePath(const Centerline::SkeletonGraph& graph,
                             int startIdx,
                             int goalIdx,
                             std::vector<int>& outPath)
{
    outPath.clear();
    const int n = static_cast<int>(graph.points.size());
    if (startIdx < 0 || goalIdx < 0 || startIdx >= n || goalIdx >= n) return false;

    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    dist[startIdx] = 0.0;

    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    pq.push({0.0, startIdx});

    while (!pq.empty()) {
        const auto [d, cur] = pq.top();
        pq.pop();
        if (d > dist[cur]) continue;
        if (cur == goalIdx) break;
        const cv::Point& cp = graph.points[cur];
        for (int nb : graph.adjacency[cur]) {
            const cv::Point& np = graph.points[nb];
            const int dx = np.x - cp.x;
            const int dy = np.y - cp.y;
            const double w = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            const double nd = d + w;
            if (nd < dist[nb]) {
                dist[nb] = nd;
                parent[nb] = cur;
                pq.push({nd, nb});
            }
        }
    }

    if (!std::isfinite(dist[goalIdx])) return false;
    for (int cur = goalIdx; cur != -1; cur = parent[cur]) {
        outPath.push_back(cur);
        if (cur == startIdx) break;
    }
    if (outPath.empty() || outPath.back() != startIdx) {
        outPath.clear();
        return false;
    }
    std::reverse(outPath.begin(), outPath.end());
    return true;
}

// Find the nearest junction reachable from a source node.
static int nearestReachableJunction(const Centerline::SkeletonGraph& graph,
                                    int srcIdx)
{
    const int n = static_cast<int>(graph.points.size());
    if (srcIdx < 0 || srcIdx >= n) return -1;

    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    dist[srcIdx] = 0.0;
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    pq.push({0.0, srcIdx});

    while (!pq.empty()) {
        const auto [d, cur] = pq.top();
        pq.pop();
        if (d > dist[cur]) continue;
        if (cur != srcIdx && static_cast<int>(graph.adjacency[cur].size()) >= 3) {
            return cur;
        }
        const cv::Point& cp = graph.points[cur];
        for (int nb : graph.adjacency[cur]) {
            const cv::Point& np = graph.points[nb];
            const int dx = np.x - cp.x;
            const int dy = np.y - cp.y;
            const double w = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            const double nd = d + w;
            if (nd < dist[nb]) {
                dist[nb] = nd;
                pq.push({nd, nb});
            }
        }
    }
    return -1;
}

// Return whether a skeleton node has junction-like graph degree.
static bool isJunctionNode(const Centerline::SkeletonGraph& graph, int idx)
{
    return idx >= 0 && idx < static_cast<int>(graph.adjacency.size()) &&
           static_cast<int>(graph.adjacency[idx].size()) >= 3;
}

struct JunctionPort {
    int clusterNode = -1;
    int outsideNode = -1;
};

struct JunctionCluster {
    int id = -1;
    std::vector<int> nodes;
    std::vector<char> contains;
};

struct LoopReturnPath {
    std::vector<int> nodes; // cluster port node -> outside path -> return cluster node
    float length = 0.f;
};

struct JunctionSelection {
    bool valid = false;
    bool fallbackUsed = false;
    int clusterCount = 0;
    int selectedCluster = -1;
    int junctionIdx = -1;
    int incomingIdx = -1;
    std::vector<int> trunk;
    JunctionCluster cluster;
    QStringList diagnostics;
};

// Group adjacent junction nodes into connected junction clusters.
static std::vector<JunctionCluster> findJunctionClusters(const Centerline::SkeletonGraph& graph)
{
    const int n = static_cast<int>(graph.points.size());
    std::vector<JunctionCluster> clusters;
    std::vector<char> visited(static_cast<size_t>(n), 0);

    for (int i = 0; i < n; ++i) {
        if (visited[static_cast<size_t>(i)] || !isJunctionNode(graph, i)) {
            continue;
        }

        JunctionCluster cluster;
        cluster.id = static_cast<int>(clusters.size());
        cluster.contains.assign(static_cast<size_t>(n), 0);

        std::queue<int> q;
        q.push(i);
        visited[static_cast<size_t>(i)] = 1;
        while (!q.empty()) {
            const int cur = q.front();
            q.pop();
            cluster.nodes.push_back(cur);
            cluster.contains[static_cast<size_t>(cur)] = 1;

            for (int nb : graph.adjacency[cur]) {
                if (visited[static_cast<size_t>(nb)] || !isJunctionNode(graph, nb)) {
                    continue;
                }
                visited[static_cast<size_t>(nb)] = 1;
                q.push(nb);
            }
        }
        clusters.push_back(std::move(cluster));
    }

    return clusters;
}

// Find graph ports where a junction cluster connects to non-cluster branches.
static std::vector<JunctionPort> clusterPorts(const Centerline::SkeletonGraph& graph,
                                              const JunctionCluster& cluster)
{
    std::vector<JunctionPort> ports;
    for (int node : cluster.nodes) {
        for (int nb : graph.adjacency[node]) {
            if (cluster.contains[static_cast<size_t>(nb)]) {
                continue;
            }
            const auto duplicate = std::find_if(
                ports.begin(), ports.end(),
                [&](const JunctionPort& p) {
                    return p.clusterNode == node && p.outsideNode == nb;
                });
            if (duplicate == ports.end()) {
                ports.push_back({node, nb});
            }
        }
    }
    return ports;
}

// Find the shortest path from a source node to any node in a junction cluster.
static bool shortestPathToCluster(const Centerline::SkeletonGraph& graph,
                                  int srcIdx,
                                  const JunctionCluster& cluster,
                                  std::vector<int>& outPath)
{
    outPath.clear();
    const int n = static_cast<int>(graph.points.size());
    if (srcIdx < 0 || srcIdx >= n || cluster.nodes.empty()) {
        return false;
    }

    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    dist[srcIdx] = 0.0;
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    pq.push({0.0, srcIdx});

    int goal = -1;
    while (!pq.empty()) {
        const auto [d, cur] = pq.top();
        pq.pop();
        if (d > dist[cur]) continue;
        if (cur != srcIdx && cluster.contains[static_cast<size_t>(cur)]) {
            goal = cur;
            break;
        }
        const cv::Point& cp = graph.points[cur];
        for (int nb : graph.adjacency[cur]) {
            const cv::Point& np = graph.points[nb];
            const int dx = np.x - cp.x;
            const int dy = np.y - cp.y;
            const double w = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            const double nd = d + w;
            if (nd < dist[nb]) {
                dist[nb] = nd;
                parent[nb] = cur;
                pq.push({nd, nb});
            }
        }
    }
    if (goal < 0) {
        return false;
    }

    for (int cur = goal; cur != -1; cur = parent[cur]) {
        outPath.push_back(cur);
        if (cur == srcIdx) break;
    }
    if (outPath.empty() || outPath.back() != srcIdx) {
        outPath.clear();
        return false;
    }
    std::reverse(outPath.begin(), outPath.end());
    return outPath.size() >= 2;
}

// Find a path through the interior of a junction cluster between two ports.
static std::vector<int> clusterInternalPath(const Centerline::SkeletonGraph& graph,
                                            const JunctionCluster& cluster,
                                            int startNode,
                                            int goalNode)
{
    if (startNode == goalNode) {
        return {startNode};
    }
    const int n = static_cast<int>(graph.points.size());
    if (startNode < 0 || goalNode < 0 || startNode >= n || goalNode >= n ||
        !cluster.contains[static_cast<size_t>(startNode)] ||
        !cluster.contains[static_cast<size_t>(goalNode)]) {
        return {};
    }

    std::vector<int> parent(static_cast<size_t>(n), -1);
    std::queue<int> q;
    q.push(startNode);
    parent[static_cast<size_t>(startNode)] = startNode;
    while (!q.empty()) {
        const int cur = q.front();
        q.pop();
        if (cur == goalNode) break;
        for (int nb : graph.adjacency[cur]) {
            if (!cluster.contains[static_cast<size_t>(nb)] ||
                parent[static_cast<size_t>(nb)] >= 0) {
                continue;
            }
            parent[static_cast<size_t>(nb)] = cur;
            q.push(nb);
        }
    }
    if (parent[static_cast<size_t>(goalNode)] < 0) {
        return {};
    }

    std::vector<int> path;
    for (int cur = goalNode; cur != startNode; cur = parent[static_cast<size_t>(cur)]) {
        path.push_back(cur);
    }
    path.push_back(startNode);
    std::reverse(path.begin(), path.end());
    return path;
}

// Find the loop branch that leaves and returns to a junction through different ports.
static bool findLoopReturnPath(const Centerline::SkeletonGraph& graph,
                               const JunctionCluster& cluster,
                               const JunctionPort& startPort,
                               LoopReturnPath& out)
{
    out = LoopReturnPath{};
    const int n = static_cast<int>(graph.points.size());
    if (startPort.clusterNode < 0 || startPort.outsideNode < 0 ||
        startPort.clusterNode >= n || startPort.outsideNode >= n) {
        return false;
    }

    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    dist[startPort.outsideNode] = 0.0;
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    pq.push({0.0, startPort.outsideNode});

    int returnNode = -1;
    while (!pq.empty()) {
        const auto [d, cur] = pq.top();
        pq.pop();
        if (d > dist[cur]) continue;

        const cv::Point& cp = graph.points[cur];
        for (int nb : graph.adjacency[cur]) {
            const bool immediateBacktrack =
                cur == startPort.outsideNode && nb == startPort.clusterNode;
            if (immediateBacktrack) {
                continue;
            }
            if (cluster.contains[static_cast<size_t>(nb)]) {
                returnNode = nb;
                parent[static_cast<size_t>(nb)] = cur;
                dist[static_cast<size_t>(nb)] = d + ptDist(
                    cv::Point2f(static_cast<float>(cp.x), static_cast<float>(cp.y)),
                    cv::Point2f(static_cast<float>(graph.points[nb].x),
                                static_cast<float>(graph.points[nb].y)));
                pq = {};
                break;
            }
            if (cluster.contains[static_cast<size_t>(cur)]) {
                continue;
            }
            const cv::Point& np = graph.points[nb];
            const int dx = np.x - cp.x;
            const int dy = np.y - cp.y;
            const double w = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            const double nd = d + w;
            if (nd < dist[nb]) {
                dist[nb] = nd;
                parent[nb] = cur;
                pq.push({nd, nb});
            }
        }
    }
    if (returnNode < 0) {
        return false;
    }

    std::vector<int> rev;
    for (int cur = returnNode; cur != -1; cur = parent[static_cast<size_t>(cur)]) {
        rev.push_back(cur);
        if (cur == startPort.outsideNode) break;
    }
    if (rev.empty() || rev.back() != startPort.outsideNode) {
        return false;
    }
    std::reverse(rev.begin(), rev.end());

    out.nodes.clear();
    out.nodes.push_back(startPort.clusterNode);
    out.nodes.insert(out.nodes.end(), rev.begin(), rev.end());
    out.length = nodePathLength(graph, out.nodes);
    return out.nodes.size() >= 3;
}

// Compute the minimum distance from a skeleton node path to a target point.
static float minPathDistanceToPoint(const Centerline::SkeletonGraph& graph,
                                    const std::vector<int>& nodes,
                                    const cv::Point2f& target,
                                    const cv::Point2f& originOffset)
{
    float best = std::numeric_limits<float>::max();
    for (int idx : nodes) {
        const cv::Point& p = graph.points[idx];
        const cv::Point2f world(static_cast<float>(p.x) + originOffset.x,
                                static_cast<float>(p.y) + originOffset.y);
        best = std::min(best, ptDist(world, target));
    }
    return best;
}

// Select a zero-tip ring centerline directly from ring skeleton graph routes.
static bool selectZeroTipRingGraphCenterline(
    const Centerline::SkeletonGraph& graph,
    const cv::Point2f& originOffset,
    const cv::Point2f& predictedHead,
    const cv::Point2f& predictedTail,
    const cv::Point2f& predictedCenter,
    bool hasPredictedCenter,
    float previousCrossSum,
    float refLength,
    QStringList* diagnostics,
    std::vector<cv::Point2f>& outCenterline)
{
    outCenterline.clear();
    const int headNode = nearestSkeletonNode(graph, predictedHead, originOffset);
    const int tailNode = nearestSkeletonNode(graph, predictedTail, originOffset);
    if (headNode < 0 || tailNode < 0 || headNode == tailNode) {
        if (diagnostics) {
            diagnostics->append(QStringLiteral("0-tip graph route failed: invalid predicted endpoint nodes head=%1 tail=%2")
                                    .arg(headNode)
                                    .arg(tailNode));
        }
        return false;
    }

    std::vector<std::vector<cv::Point2f>> paths;
    const float minUsableLen = refLength > 0.f ? std::max(6.f, 0.5f * refLength) : 6.f;

    const std::vector<JunctionCluster> clusters = findJunctionClusters(graph);
    for (const JunctionCluster& cluster : clusters) {
        if (cluster.nodes.empty()) {
            continue;
        }
        std::vector<int> hTrunk;
        if (!shortestPathToCluster(graph, headNode, cluster, hTrunk) || hTrunk.size() < 2) {
            continue;
        }
        const int hClusterNode = hTrunk.back();
        const int hIncoming = hTrunk[hTrunk.size() - 2];

        std::vector<int> tTrunk;
        if (!shortestPathToCluster(graph, tailNode, cluster, tTrunk) || tTrunk.size() < 2) {
            continue;
        }
        const int tClusterNode = tTrunk.back();
        const int tIncoming = tTrunk[tTrunk.size() - 2];
        if (hIncoming == tIncoming) {
            continue;
        }

        std::vector<int> internal = clusterInternalPath(graph, cluster, hClusterNode, tClusterNode);
        if (internal.empty()) {
            continue;
        }
        std::vector<int> full = hTrunk;
        full.insert(full.end(), internal.begin() + 1, internal.end());
        std::reverse(tTrunk.begin(), tTrunk.end());
        full.insert(full.end(), tTrunk.begin() + 1, tTrunk.end());
        if (full.size() >= 2 && nodePathLength(graph, full) >= minUsableLen) {
            paths.push_back(nodesToWorldPath(graph, full, originOffset));
        }

        const std::vector<JunctionPort> ports = clusterPorts(graph, cluster);
        for (const JunctionPort& port : ports) {
            if (port.outsideNode == hIncoming || port.outsideNode == tIncoming) {
                continue;
            }
            LoopReturnPath loop;
            if (!findLoopReturnPath(graph, cluster, port, loop) ||
                loop.length < minUsableLen) {
                continue;
            }
            std::vector<int> loopToTail =
                clusterInternalPath(graph, cluster, loop.nodes.back(), tClusterNode);
            if (loopToTail.empty()) {
                continue;
            }
            std::vector<int> loopFull = hTrunk;
            loopFull.insert(loopFull.end(), internal.begin() + 1, internal.end());
            loopFull.insert(loopFull.end(), loop.nodes.begin() + 1, loop.nodes.end());
            loopFull.insert(loopFull.end(), loopToTail.begin() + 1, loopToTail.end());
            std::vector<int> tailToCluster = tTrunk;
            std::reverse(tailToCluster.begin(), tailToCluster.end());
            loopFull.insert(loopFull.end(), tailToCluster.begin() + 1, tailToCluster.end());
            if (loopFull.size() >= 2 && nodePathLength(graph, loopFull) >= minUsableLen) {
                paths.push_back(nodesToWorldPath(graph, loopFull, originOffset));
            }
        }
    }

    if (paths.empty()) {
        if (diagnostics) {
            diagnostics->append(QStringLiteral("0-tip graph route failed: no skeleton path between predicted endpoint nodes"));
        }
        return false;
    }

    struct Candidate {
        std::vector<cv::Point2f> points;
        float len = 0.f;
        float endpointDist = 0.f;
        float centerDist = 0.f;
        float crossSum = 0.f;
        float crossPenalty = 0.f;
        float score = std::numeric_limits<float>::max();
    };

    std::vector<Candidate> candidates;
    for (std::vector<cv::Point2f> path : paths) {
        if (path.size() < 2) {
            continue;
        }
        const float forward = ptDist(path.front(), predictedHead) +
                              ptDist(path.back(), predictedTail);
        const float reversed = ptDist(path.front(), predictedTail) +
                               ptDist(path.back(), predictedHead);
        if (reversed < forward) {
            std::reverse(path.begin(), path.end());
        }

        Candidate c;
        c.points = std::move(path);
        c.endpointDist = std::min(forward, reversed);
        c.len = arcLen(c.points);
        if (hasPredictedCenter && c.points.size() >= 2) {
            c.centerDist = ptDist(c.points[c.points.size() / 2], predictedCenter);
        }
        c.crossSum = centerlineCrossSum(c.points);
        constexpr float kCrossSumEpsilon = 1e-4f;
        if (std::abs(previousCrossSum) > kCrossSumEpsilon &&
            std::abs(c.crossSum) > kCrossSumEpsilon) {
            c.crossPenalty = (c.crossSum * previousCrossSum > 0.f) ? 0.f : 1.f;
        }

        constexpr float kEndpointWeight = 3.0f;
        constexpr float kCenterWeight = 2.0f;
        constexpr float kCrossMismatchWeight = 100.0f;
        constexpr float kLengthWeight = 1.0f;
        c.score = kEndpointWeight * c.endpointDist +
                  kCenterWeight * c.centerDist +
                  kCrossMismatchWeight * c.crossPenalty;
        if (refLength > 0.f) {
            c.score += kLengthWeight * std::abs(c.len - refLength);
        }
        candidates.push_back(std::move(c));
    }

    if (candidates.empty()) {
        return false;
    }

    int bestIdx = 0;
    for (int i = 1; i < static_cast<int>(candidates.size()); ++i) {
        if (candidates[i].score < candidates[bestIdx].score) {
            bestIdx = i;
        }
    }

    if (diagnostics) {
        diagnostics->append(QStringLiteral("0-tip graph route candidates=%1 selected=%2 headNode=(%3,%4) tailNode=(%5,%6)")
                                .arg(static_cast<int>(candidates.size()))
                                .arg(bestIdx)
                                .arg(graph.points[headNode].x)
                                .arg(graph.points[headNode].y)
                                .arg(graph.points[tailNode].x)
                                .arg(graph.points[tailNode].y));
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
            const Candidate& c = candidates[i];
            diagnostics->append(
                QStringLiteral("0-tip graph candidate %1: len=%2 endpointDist=%3 centerDist=%4 crossSum=%5 crossPenalty=%6 score=%7%8")
                    .arg(i)
                    .arg(c.len, 0, 'f', 2)
                    .arg(c.endpointDist, 0, 'f', 2)
                    .arg(c.centerDist, 0, 'f', 2)
                    .arg(c.crossSum, 0, 'f', 4)
                    .arg(c.crossPenalty, 0, 'f', 1)
                    .arg(c.score, 0, 'f', 2)
                    .arg(i == bestIdx ? QStringLiteral(" SELECTED") : QString()));
        }
    }

    outCenterline = candidates[bestIdx].points;
    return outCenterline.size() >= 2;
}

// Select the junction that best represents the loop-aware self-crossing route.
static JunctionSelection selectLoopAwareJunction(const Centerline::SkeletonGraph& graph,
                                                 int srcIdx,
                                                 const cv::Point2f& predictedHidden,
                                                 const cv::Point2f& originOffset,
                                                 float refLength)
{
    JunctionSelection selection;
    const std::vector<JunctionCluster> clusters = findJunctionClusters(graph);
    selection.clusterCount = static_cast<int>(clusters.size());

    const float minTrunkLen = refLength > 0.f ? std::max(3.f, 0.10f * refLength) : 3.f;
    float bestScore = std::numeric_limits<float>::max();
    int bestLoopCount = 0;
    float bestLoopLength = 0.f;

    for (const JunctionCluster& cluster : clusters) {
        std::vector<int> trunk;
        if (!shortestPathToCluster(graph, srcIdx, cluster, trunk) || trunk.size() < 2) {
            selection.diagnostics << QStringLiteral("cluster %1 unreachable")
                                         .arg(cluster.id);
            continue;
        }
        const int incomingIdx = trunk[trunk.size() - 2];
        const std::vector<JunctionPort> ports = clusterPorts(graph, cluster);

        const float trunkLen = nodePathLength(graph, trunk);
        if (trunkLen < minTrunkLen) {
            selection.diagnostics << QStringLiteral("cluster %1 rejected short trunk=%2 min=%3 rep=(%4,%5)")
                                         .arg(cluster.id)
                                         .arg(trunkLen, 0, 'f', 2)
                                         .arg(minTrunkLen, 0, 'f', 2)
                                         .arg(graph.points[trunk.back()].x)
                                         .arg(graph.points[trunk.back()].y);
            continue;
        }

        int loopCount = 0;
        float clusterBestLoopLength = std::numeric_limits<float>::max();
        float clusterBestHiddenDist = std::numeric_limits<float>::max();
        const float minLoopLength = refLength > 0.f ? std::max(6.f, 0.20f * refLength) : 6.f;
        for (const JunctionPort& port : ports) {
            if (port.outsideNode == incomingIdx) {
                continue;
            }
            LoopReturnPath loop;
            if (!findLoopReturnPath(graph, cluster, port, loop)) {
                continue;
            }
            if (loop.length < minLoopLength) {
                continue;
            }
            ++loopCount;
            clusterBestLoopLength = std::min(clusterBestLoopLength, loop.length);
            clusterBestHiddenDist = std::min(
                clusterBestHiddenDist,
                minPathDistanceToPoint(graph, loop.nodes, predictedHidden, originOffset));
        }

        const bool hasLoop = loopCount > 0;
        const float loopLenForLog = hasLoop ? clusterBestLoopLength : 0.f;
        selection.diagnostics << QStringLiteral("cluster %1 rep=(%2,%3) trunk=%4 ports=%5 returningLoops=%6 bestLoop=%7")
                                     .arg(cluster.id)
                                     .arg(graph.points[trunk.back()].x)
                                     .arg(graph.points[trunk.back()].y)
                                     .arg(trunkLen, 0, 'f', 2)
                                     .arg(static_cast<int>(ports.size()))
                                     .arg(loopCount)
                                     .arg(loopLenForLog, 0, 'f', 2);
        if (!hasLoop) {
            continue;
        }

        float score = trunkLen;
        if (refLength > 0.f) {
            score += 0.25f * std::abs(clusterBestLoopLength - refLength);
        }
        if (std::isfinite(clusterBestHiddenDist)) {
            score += 0.5f * clusterBestHiddenDist;
        }
        if (score < bestScore) {
            bestScore = score;
            selection.valid = true;
            selection.fallbackUsed = false;
            selection.selectedCluster = cluster.id;
            selection.junctionIdx = trunk.back();
            selection.incomingIdx = incomingIdx;
            selection.trunk = std::move(trunk);
            selection.cluster = cluster;
            bestLoopCount = loopCount;
            bestLoopLength = clusterBestLoopLength;
        }
    }

    if (selection.valid) {
        selection.diagnostics << QStringLiteral("selected loop-valid cluster %1 returningLoops=%2 bestLoop=%3")
                                     .arg(selection.selectedCluster)
                                     .arg(bestLoopCount)
                                     .arg(bestLoopLength, 0, 'f', 2);
        return selection;
    }

    const int fallback = nearestReachableJunction(graph, srcIdx);
    if (fallback >= 0) {
        std::vector<int> trunk;
        if (shortestNodePath(graph, srcIdx, fallback, trunk) && trunk.size() >= 2) {
            const float trunkLen = nodePathLength(graph, trunk);
            if (trunkLen < minTrunkLen) {
                selection.diagnostics << QStringLiteral("fallback nearest degree-3 node rejected short trunk=%1 min=%2 node=(%3,%4)")
                                             .arg(trunkLen, 0, 'f', 2)
                                             .arg(minTrunkLen, 0, 'f', 2)
                                             .arg(graph.points[fallback].x)
                                             .arg(graph.points[fallback].y);
                return selection;
            }

            JunctionCluster cluster;
            const int n = static_cast<int>(graph.points.size());
            cluster.id = -1;
            cluster.nodes = {fallback};
            cluster.contains.assign(static_cast<size_t>(n), 0);
            cluster.contains[static_cast<size_t>(fallback)] = 1;

            selection.valid = true;
            selection.fallbackUsed = true;
            selection.selectedCluster = -1;
            selection.junctionIdx = fallback;
            selection.incomingIdx = trunk[trunk.size() - 2];
            selection.trunk = std::move(trunk);
            selection.cluster = std::move(cluster);
            selection.diagnostics << QStringLiteral("no loop-valid junction found; fallback nearest degree-3 node=(%1,%2)")
                                         .arg(graph.points[fallback].x)
                                         .arg(graph.points[fallback].y);
        }
    }
    return selection;
}

// Trace a simple skeleton branch from a starting node until a stop condition.
static std::vector<int> traceSkeletonBranch(const Centerline::SkeletonGraph& graph,
                                            int junctionIdx,
                                            int incomingIdx,
                                            int firstBranchIdx)
{
    std::vector<int> path;
    const int n = static_cast<int>(graph.points.size());
    if (junctionIdx < 0 || incomingIdx < 0 || firstBranchIdx < 0 ||
        junctionIdx >= n || incomingIdx >= n || firstBranchIdx >= n) {
        return path;
    }

    std::vector<char> visited(static_cast<size_t>(n), 0);
    path.push_back(junctionIdx);
    path.push_back(firstBranchIdx);
    visited[static_cast<size_t>(junctionIdx)] = 1;
    visited[static_cast<size_t>(firstBranchIdx)] = 1;

    int prev = junctionIdx;
    int cur = firstBranchIdx;
    while (true) {
        std::vector<int> candidates;
        for (int nb : graph.adjacency[cur]) {
            if (nb == prev) continue;
            if (nb == junctionIdx) {
                path.push_back(nb);
                return path;
            }
            if (!visited[static_cast<size_t>(nb)]) candidates.push_back(nb);
        }
        if (candidates.empty()) break;

        const cv::Point& pp = graph.points[prev];
        const cv::Point& cp = graph.points[cur];
        const cv::Point2f inVec(static_cast<float>(cp.x - pp.x),
                                static_cast<float>(cp.y - pp.y));
        int best = candidates.front();
        float bestDot = -std::numeric_limits<float>::max();
        const float inNorm = std::max(1e-6f, std::hypot(inVec.x, inVec.y));
        for (int nb : candidates) {
            const cv::Point& np = graph.points[nb];
            const cv::Point2f outVec(static_cast<float>(np.x - cp.x),
                                     static_cast<float>(np.y - cp.y));
            const float outNorm = std::max(1e-6f, std::hypot(outVec.x, outVec.y));
            const float dot = (inVec.x * outVec.x + inVec.y * outVec.y) / (inNorm * outNorm);
            if (dot > bestDot) {
                bestDot = dot;
                best = nb;
            }
        }

        prev = cur;
        cur = best;
        path.push_back(cur);
        visited[static_cast<size_t>(cur)] = 1;
    }

    return path;
}

// Trace a branch leaving a junction cluster through one of its ports.
static std::vector<int> traceSkeletonBranchFromCluster(const Centerline::SkeletonGraph& graph,
                                                       const JunctionCluster& cluster,
                                                       int clusterNode,
                                                       int firstBranchIdx)
{
    std::vector<int> path;
    const int n = static_cast<int>(graph.points.size());
    if (clusterNode < 0 || firstBranchIdx < 0 ||
        clusterNode >= n || firstBranchIdx >= n ||
        !cluster.contains[static_cast<size_t>(clusterNode)] ||
        cluster.contains[static_cast<size_t>(firstBranchIdx)]) {
        return path;
    }

    std::vector<char> visited(static_cast<size_t>(n), 0);
    path.push_back(clusterNode);
    path.push_back(firstBranchIdx);
    for (int node : cluster.nodes) {
        visited[static_cast<size_t>(node)] = 1;
    }
    visited[static_cast<size_t>(firstBranchIdx)] = 1;

    int prev = clusterNode;
    int cur = firstBranchIdx;
    while (true) {
        std::vector<int> candidates;
        for (int nb : graph.adjacency[cur]) {
            if (nb == prev) continue;
            if (cluster.contains[static_cast<size_t>(nb)]) {
                path.push_back(nb);
                return path;
            }
            if (!visited[static_cast<size_t>(nb)]) candidates.push_back(nb);
        }
        if (candidates.empty()) break;

        const cv::Point& pp = graph.points[prev];
        const cv::Point& cp = graph.points[cur];
        const cv::Point2f inVec(static_cast<float>(cp.x - pp.x),
                                static_cast<float>(cp.y - pp.y));
        int best = candidates.front();
        float bestDot = -std::numeric_limits<float>::max();
        const float inNorm = std::max(1e-6f, std::hypot(inVec.x, inVec.y));
        for (int nb : candidates) {
            const cv::Point& np = graph.points[nb];
            const cv::Point2f outVec(static_cast<float>(np.x - cp.x),
                                     static_cast<float>(np.y - cp.y));
            const float outNorm = std::max(1e-6f, std::hypot(outVec.x, outVec.y));
            const float dot = (inVec.x * outVec.x + inVec.y * outVec.y) / (inNorm * outNorm);
            if (dot > bestDot) {
                bestDot = dot;
                best = nb;
            }
        }

        prev = cur;
        cur = best;
        path.push_back(cur);
        visited[static_cast<size_t>(cur)] = 1;
    }

    return path;
}

struct D3RouteDebug {
    bool available = false;
    bool startIsHead = false;
    int selectedCandidate = -1;
    int junctionClusterCount = 0;
    int selectedJunctionCluster = -1;
    bool junctionFallbackUsed = false;
    cv::Point2f start = {-1.f, -1.f};
    cv::Point2f junction = {-1.f, -1.f};
    cv::Point2f center = {-1.f, -1.f};
    cv::Point2f end = {-1.f, -1.f};
    std::vector<std::vector<cv::Point2f>> candidatePaths;
    QStringList junctionDiagnostics;
};

// D-3 hidden-tip routing: walk from the known visible tip to the first
// skeleton junction, score every outgoing branch as a whole candidate path,
// then continue along the best branch until its distance to the predicted
// hidden position is minimized. This avoids the failure mode where Dijkstra
// terminates by the shortest route to the node nearest Tpred and the failure
// mode where one noisy junction pixel decides the branch orientation.
// Route from a visible tip toward an actual or predicted hidden target.
static bool skeletonPathTowardPredictedHidden(const Centerline::SkeletonGraph& graph,
                                              int srcIdx,
                                              const cv::Point2f& predictedHidden,
                                              bool targetIsActualTip,
                                              bool pathStartsAtHead,
                                              const cv::Point2f& predictedCenter,
                                              bool hasPredictedCenter,
                                              const cv::Point2f& originOffset,
                                              const std::vector<cv::Point2f>& previousCenterline,
                                              float prevTurningAngle,
                                              float angleThreshold,
                                              int nPoints,
                                              float refLength,
                                              QStringList* diagnostics,
                                              D3RouteDebug* routeDebug,
                                              std::vector<cv::Point2f>& outPath)
{
    outPath.clear();
    (void)previousCenterline;
    (void)angleThreshold;
    (void)nPoints;
    JunctionSelection junction = selectLoopAwareJunction(graph, srcIdx, predictedHidden,
                                                         originOffset, refLength);
    if (!junction.valid || junction.trunk.size() < 2) {
        return false;
    }
    const int junctionIdx = junction.junctionIdx;
    const int incomingIdx = junction.incomingIdx;
    const std::vector<int>& trunk = junction.trunk;
    const std::vector<JunctionPort> ports = clusterPorts(graph, junction.cluster);

    if (diagnostics) {
        diagnostics->append(QStringLiteral("D-3 junction clusters=%1 selected=%2 fallback=%3")
                                .arg(junction.clusterCount)
                                .arg(junction.selectedCluster)
                                .arg(junction.fallbackUsed ? "Y" : "N"));
        for (const QString& detail : junction.diagnostics) {
            diagnostics->append(QStringLiteral("D-3 junction %1").arg(detail));
        }
    }
    if (routeDebug) {
        routeDebug->available = true;
        routeDebug->selectedCandidate = -1;
        routeDebug->junctionClusterCount = junction.clusterCount;
        routeDebug->selectedJunctionCluster = junction.selectedCluster;
        routeDebug->junctionFallbackUsed = junction.fallbackUsed;
        routeDebug->start = cv::Point2f(
            static_cast<float>(graph.points[srcIdx].x) + originOffset.x,
            static_cast<float>(graph.points[srcIdx].y) + originOffset.y);
        routeDebug->junction = cv::Point2f(
            static_cast<float>(graph.points[junctionIdx].x) + originOffset.x,
            static_cast<float>(graph.points[junctionIdx].y) + originOffset.y);
        routeDebug->center = predictedCenter;
        routeDebug->end = predictedHidden;
        routeDebug->candidatePaths.clear();
        routeDebug->junctionDiagnostics = junction.diagnostics;
    }

    struct BranchCandidate {
        std::vector<int> nodes; // junction -> ...
        std::vector<int> fullNodes;
        int bestIndex = -1;
        float hiddenDist = std::numeric_limits<float>::max();
        float centerDist = 0.f;
        float crossSum = 0.f;
        float crossPenalty = 0.f;
        float pathLen = 0.f;
        float totalScore = std::numeric_limits<float>::max();
    };

    std::vector<BranchCandidate> candidates;
    for (const JunctionPort& port : ports) {
        if (port.outsideNode == incomingIdx) continue;
        BranchCandidate c;
        std::vector<int> internalPath =
            clusterInternalPath(graph, junction.cluster, junctionIdx, port.clusterNode);
        std::vector<int> branchPath =
            traceSkeletonBranchFromCluster(graph, junction.cluster,
                                           port.clusterNode, port.outsideNode);
        if (internalPath.empty() || branchPath.size() < 2) continue;
        c.nodes = internalPath;
        c.nodes.insert(c.nodes.end(), branchPath.begin() + 1, branchPath.end());
        if (c.nodes.size() < 2) continue;

        std::vector<float> cumulative(c.nodes.size(), 0.f);
        for (int i = 1; i < static_cast<int>(c.nodes.size()); ++i) {
            const cv::Point& a = graph.points[c.nodes[i - 1]];
            const cv::Point& b = graph.points[c.nodes[i]];
            const int sx = b.x - a.x;
            const int sy = b.y - a.y;
            cumulative[i] = cumulative[i - 1] +
                ((sx != 0 && sy != 0) ? std::sqrt(2.f) : 1.f);
        }

        const float trunkLen = nodePathLength(graph, trunk);
        const float minUsableLen = refLength > 0.f ? 0.75f * refLength : 0.f;
        int bestLongEnoughIndex = -1;
        float bestLongEnoughDist = std::numeric_limits<float>::max();
        float bestDist = std::numeric_limits<float>::max();
        for (int i = 1; i < static_cast<int>(c.nodes.size()); ++i) {
            const cv::Point& p = graph.points[c.nodes[i]];
            const float wx = static_cast<float>(p.x) + originOffset.x;
            const float wy = static_cast<float>(p.y) + originOffset.y;
            const float dx = wx - predictedHidden.x;
            const float dy = wy - predictedHidden.y;
            const float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < bestDist) {
                bestDist = dist;
                c.bestIndex = i;
            }
            if (trunkLen + cumulative[i] >= minUsableLen &&
                dist < bestLongEnoughDist) {
                bestLongEnoughDist = dist;
                bestLongEnoughIndex = i;
            }
        }
        if (targetIsActualTip) {
            c.bestIndex = static_cast<int>(c.nodes.size()) - 1;
            const cv::Point& endPoint = graph.points[c.nodes[c.bestIndex]];
            c.hiddenDist = ptDist(cv::Point2f(static_cast<float>(endPoint.x) + originOffset.x,
                                              static_cast<float>(endPoint.y) + originOffset.y),
                                  predictedHidden);
        } else if (bestLongEnoughIndex >= 1) {
            c.bestIndex = bestLongEnoughIndex;
            c.hiddenDist = bestLongEnoughDist;
        } else {
            c.hiddenDist = bestDist;
        }
        if (c.bestIndex < 1) continue;

        c.fullNodes = trunk;
        c.fullNodes.insert(c.fullNodes.end(),
                           c.nodes.begin() + 1,
                           c.nodes.begin() + c.bestIndex + 1);
        if (targetIsActualTip) {
            const int targetNode = nearestSkeletonNode(graph, predictedHidden, originOffset);
            if (targetNode >= 0 && !c.fullNodes.empty() && c.fullNodes.back() != targetNode) {
                std::vector<int> extension;
                if (shortestNodePath(graph, c.fullNodes.back(), targetNode, extension) &&
                    extension.size() >= 2) {
                    c.fullNodes.insert(c.fullNodes.end(), extension.begin() + 1, extension.end());
                    c.hiddenDist = 0.f;
                }
            }
        }
        c.pathLen = nodePathLength(graph, c.fullNodes);
        if (hasPredictedCenter && c.fullNodes.size() >= 2) {
            const float halfLen = 0.5f * c.pathLen;
            float walked = 0.f;
            cv::Point2f midpoint(
                static_cast<float>(graph.points[c.fullNodes.back()].x) + originOffset.x,
                static_cast<float>(graph.points[c.fullNodes.back()].y) + originOffset.y);
            for (int i = 1; i < static_cast<int>(c.fullNodes.size()); ++i) {
                const cv::Point& a = graph.points[c.fullNodes[i - 1]];
                const cv::Point& b = graph.points[c.fullNodes[i]];
                const int sx = b.x - a.x;
                const int sy = b.y - a.y;
                const float segLen = (sx != 0 && sy != 0) ? std::sqrt(2.f) : 1.f;
                if (walked + segLen >= halfLen) {
                    const float t = (halfLen - walked) / std::max(segLen, 1e-6f);
                    midpoint = cv::Point2f(
                        (static_cast<float>(a.x) + t * static_cast<float>(sx)) + originOffset.x,
                        (static_cast<float>(a.y) + t * static_cast<float>(sy)) + originOffset.y);
                    break;
                }
                walked += segLen;
            }
            c.centerDist = ptDist(midpoint, predictedCenter);
        }
        const std::vector<cv::Point2f> candidatePath =
            nodesToWorldPath(graph, c.fullNodes, originOffset);
        if (routeDebug) {
            routeDebug->candidatePaths.push_back(candidatePath);
        }
        if (pathStartsAtHead) {
            c.crossSum = centerlineCrossSum(candidatePath);
        } else {
            std::vector<cv::Point2f> headToTailPath = candidatePath;
            std::reverse(headToTailPath.begin(), headToTailPath.end());
            c.crossSum = centerlineCrossSum(headToTailPath);
        }
        constexpr float kCrossSumEpsilon = 1e-4f;
        if (std::abs(prevTurningAngle) > kCrossSumEpsilon &&
            std::abs(c.crossSum) > kCrossSumEpsilon) {
            c.crossPenalty = (c.crossSum * prevTurningAngle > 0.f) ? 0.f : 1.f;
        }

        constexpr float kHiddenWeight = 3.0f;
        constexpr float kCenterWeight = 2.0f;
        constexpr float kCrossMismatchWeight = 100.0f;
        constexpr float kLengthWeight = 1.0f;
        c.totalScore =
            kHiddenWeight * c.hiddenDist +
            kCenterWeight * c.centerDist +
            kCrossMismatchWeight * c.crossPenalty;
        if (refLength > 0.f) {
            c.totalScore += kLengthWeight * std::abs(c.pathLen - refLength);
        }
        candidates.push_back(std::move(c));
    }
    if (candidates.empty()) return false;

    int bestCandidate = -1;
    float bestScore = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        const BranchCandidate& c = candidates[i];
        if (c.totalScore < bestScore) {
            bestScore = c.totalScore;
            bestCandidate = i;
        }
    }
    if (bestCandidate < 0) return false;

    if (diagnostics) {
        diagnostics->append(QStringLiteral("%1 whole-path candidates=%2 selected=%3")
                                .arg(targetIsActualTip ? "D-2" : "D-3")
                                .arg(static_cast<int>(candidates.size()))
                                .arg(bestCandidate));
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
            const BranchCandidate& c = candidates[i];
            diagnostics->append(
                QStringLiteral("%1 candidate %2: len=%3 targetDist=%4 centerDist=%5 crossSum=%6 crossPenalty=%7 score=%8%9")
                    .arg(targetIsActualTip ? "D-2" : "D-3")
                    .arg(i)
                    .arg(c.pathLen, 0, 'f', 2)
                    .arg(c.hiddenDist, 0, 'f', 2)
                    .arg(c.centerDist, 0, 'f', 2)
                    .arg(c.crossSum, 0, 'f', 4)
                    .arg(c.crossPenalty, 0, 'f', 1)
                    .arg(c.totalScore, 0, 'f', 2)
                    .arg(i == bestCandidate ? QStringLiteral(" SELECTED") : QString()));
        }
    }

    const BranchCandidate& picked = candidates[bestCandidate];
    if (routeDebug) {
        routeDebug->selectedCandidate = bestCandidate;
    }
    outPath = nodesToWorldPath(graph, picked.fullNodes, originOffset);
    return outPath.size() >= 2;
}

// Build a copy of a blob with a synthetic circular hole punched at the
// distance-transform maximum.  Used as a fallback when a Clean-topology
// D-1 path is suspiciously short (worm tightly self-coiled but no hole
// visible in the mask yet).  The hole radius is 80% of the DT value at
// the peak (≈ the local body half-width).
static bool addSyntheticHoleAtDTMax(const Tracking::DetectedBlob& src,
                                     const cv::Mat& distTransform,
                                     const cv::Rect& localBounds,
                                     Tracking::DetectedBlob& out)
{
    if (distTransform.empty()) return false;
    double maxVal = 0.0;
    cv::Point maxLoc;
    cv::minMaxLoc(distTransform, nullptr, &maxVal, nullptr, &maxLoc);
    const float radius = static_cast<float>(maxVal * 0.8);
    if (radius < 3.f) return false;

    const float cx = static_cast<float>(maxLoc.x + localBounds.x);
    const float cy = static_cast<float>(maxLoc.y + localBounds.y);
    const int nPts = std::max(8, static_cast<int>(2.f * float(CV_PI) * radius));

    out = src;
    std::vector<cv::Point> hole;
    hole.reserve(nPts);
    for (int i = 0; i < nPts; ++i) {
        const float ang = 2.f * float(CV_PI) * i / nPts;
        hole.push_back({static_cast<int>(std::lround(cx + radius * std::cos(ang))),
                        static_cast<int>(std::lround(cy + radius * std::sin(ang)))});
    }
    out.holeContourPoints.push_back(std::move(hole));
    return true;
}

// ── Active-contour ("snake") refinement ─────────────────────────────────────
//
// For ring/coiled frames, skeletonization can never produce a self-intersecting
// centerline because the skeleton of a planar mask is a planar tree. When a
// worm physically crosses over itself, the true 2D centerline IS self-intersecting.
//
// This helper evolves a parametric polyline (which has no topological constraint
// against self-intersection) under three forces:
//   - tension  (alpha) → discrete Laplacian: V[i-1] - 2V[i] + V[i+1]
//   - rigidity (beta)  → discrete biharmonic: -(V[i-2] - 4V[i-1] + 6V[i] - 4V[i+1] + V[i+2])
//   - image    (lambda)→ ∇D where D is the distance transform of the blob mask.
//                        D's ridge IS the medial axis, so following ∇D pulls the
//                        snake onto the body axis.
//
// Snake core takes a pre-built mask, an explicit init polyline, and explicit
// pinned head/tail positions. The active pipeline uses it to lightly refine
// Clean-frame skeleton paths while keeping the DT, gradient, Euler loop, and
// overlap detection in one place.
//
// `v` is mutated in place: caller passes the init polyline; on success it
// contains the refined, nPoint-resampled centerline.
static bool refineSnakeCore(const Tracking::DetectedBlob& blob,
                            const cv::Mat& mask,
                            const cv::Rect& bounds,
                            std::vector<cv::Point2f>& v,
                            const cv::Point2f& pinHead,
                            const cv::Point2f& pinTail,
                            int nPoints,
                            const Centerline::CenterlineSnakeParams& params,
                            cv::Point2f& outOverlapCenter,
                            bool& outHasOverlap)
{
    outHasOverlap = false;
    if (mask.empty() || v.empty() || blob.contourPoints.empty()) return false;

    // Resample to canonical nPoint count.
    if (static_cast<int>(v.size()) != nPoints) v = resample(v, nPoints);
    if (static_cast<int>(v.size()) < 4) return false;

    // Distance transform → its ridges ARE the medial axis. Smooth a little so
    // ∇D is well-defined off-ridge. Sobel gives the gradient field that
    // attracts the snake toward the ridge (highest-D pixels).
    cv::Mat dt;
    cv::distanceTransform(mask, dt, cv::DIST_L2, 3);
    cv::GaussianBlur(dt, dt, cv::Size(0, 0), 1.0);
    cv::Mat gx, gy;
    cv::Sobel(dt, gx, CV_32F, 1, 0, 3);
    cv::Sobel(dt, gy, CV_32F, 0, 1, 3);

    // Normalize gradient magnitude scale so lambda has roughly mask-size-
    // independent meaning.
    double gxMin = 0.0, gxMax = 0.0, gyMin = 0.0, gyMax = 0.0;
    cv::minMaxLoc(gx, &gxMin, &gxMax);
    cv::minMaxLoc(gy, &gyMin, &gyMax);
    const double gradScale = std::max({std::abs(gxMin), std::abs(gxMax),
                                       std::abs(gyMin), std::abs(gyMax), 1.0});
    gx /= static_cast<float>(gradScale);
    gy /= static_cast<float>(gradScale);

    auto sampleGradient = [&](const cv::Point2f& world) -> cv::Point2f {
        const int lx = std::clamp(static_cast<int>(std::lround(world.x - bounds.x)),
                                  0, gx.cols - 1);
        const int ly = std::clamp(static_cast<int>(std::lround(world.y - bounds.y)),
                                  0, gx.rows - 1);
        return cv::Point2f(gx.at<float>(ly, lx), gy.at<float>(ly, lx));
    };
    auto isInsideMask = [&](const cv::Point2f& world) -> bool {
        const int lx = static_cast<int>(std::lround(world.x - bounds.x));
        const int ly = static_cast<int>(std::lround(world.y - bounds.y));
        if (lx < 0 || ly < 0 || lx >= mask.cols || ly >= mask.rows) return false;
        return mask.at<uchar>(ly, lx) != 0;
    };

    // Pin endpoints. Caller supplies the positions (e.g. assigned head/tail
    // tip points, or prev-frame endpoints snapped to current contour).
    v.front() = pinHead;
    v.back()  = pinTail;

    // Explicit-Euler gradient descent on the discretized energy.
    const float alpha  = static_cast<float>(params.alpha);
    const float beta   = static_cast<float>(params.beta);
    const float lambda = static_cast<float>(params.lambda);
    const float tau    = static_cast<float>(std::max(1e-3, params.stepSize));
    const int n = static_cast<int>(v.size());

    std::vector<cv::Point2f> next(v.size());
    for (int iter = 0; iter < std::max(1, params.iterations); ++iter) {
        next.front() = v.front();
        next.back()  = v.back();

        for (int i = 1; i < n - 1; ++i) {
            const cv::Point2f tens = v[i - 1] - 2.f * v[i] + v[i + 1];
            cv::Point2f rig(0.f, 0.f);
            if (i >= 2 && i <= n - 3) {
                rig = v[i - 2] - 4.f * v[i - 1] + 6.f * v[i] - 4.f * v[i + 1] + v[i + 2];
            }
            const cv::Point2f img = sampleGradient(v[i]);
            const cv::Point2f force = alpha * tens - beta * rig + lambda * img;
            cv::Point2f candidate = v[i] + tau * force;
            if (!isInsideMask(candidate)) {
                candidate = nearestContourPoint(blob.contourPoints, candidate);
            }
            next[i] = candidate;
        }
        v.swap(next);
    }

    if (static_cast<int>(v.size()) != nPoints) v = resample(v, nPoints);
    if (v.size() < 2) return false;

    // Self-intersection detection for debug overlay.
    auto segmentsIntersect = [](const cv::Point2f& a, const cv::Point2f& b,
                                const cv::Point2f& c, const cv::Point2f& d,
                                cv::Point2f& crossing) -> bool {
        const cv::Point2f r = b - a;
        const cv::Point2f s = d - c;
        const float denom = r.x * s.y - r.y * s.x;
        if (std::abs(denom) < 1e-6f) return false;
        const float t = ((c.x - a.x) * s.y - (c.y - a.y) * s.x) / denom;
        const float u = ((c.x - a.x) * r.y - (c.y - a.y) * r.x) / denom;
        if (t > 0.f && t < 1.f && u > 0.f && u < 1.f) {
            crossing = a + t * r;
            return true;
        }
        return false;
    };
    for (int i = 0; i < static_cast<int>(v.size()) - 1 && !outHasOverlap; ++i) {
        for (int j = i + 2; j < static_cast<int>(v.size()) - 1 && !outHasOverlap; ++j) {
            cv::Point2f x;
            if (segmentsIntersect(v[i], v[i + 1], v[j], v[j + 1], x)) {
                outOverlapCenter = x;
                outHasOverlap = true;
            }
        }
    }
    return true;
}

// Helper: build the local mask + padded bounds for a blob.
static bool buildSnakeMask(const Tracking::DetectedBlob& blob,
                           cv::Mat& outMask, cv::Rect& outBounds)
{
    if (blob.contourPoints.empty()) return false;
    cv::Rect bounds = cv::boundingRect(blob.contourPoints);
    constexpr int kPad = 8;
    bounds.x -= kPad;
    bounds.y -= kPad;
    bounds.width += 2 * kPad;
    bounds.height += 2 * kPad;
    if (bounds.width <= 1 || bounds.height <= 1) return false;
    cv::Mat mask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    fillBlobMask(mask, blob, bounds, cv::Point2f(0.f, 0.f));
    if (cv::countNonZero(mask) < 4) return false;
    outMask = mask;
    outBounds = bounds;
    return true;
}

// Copy endpoint-detection internals into a frame debug record for export.
static void captureEndpointDebug(const Centerline::EndpointResult& er,
                                 Debug::CenterlineFrameDebug& record)
{
    record.endpointLocalBounds = er.localBounds;
    record.skeletonPixels.clear();
    record.skeletonEndpointPoints.clear();
    record.contourCurvaturePoints = er.contourPoints;
    record.contourCurvatures = er.contourCurvatures;
    record.contourCurvaturePeaks = er.contourCurvaturePeaks;

    const cv::Point2f origin(static_cast<float>(er.localBounds.x),
                             static_cast<float>(er.localBounds.y));
    if (!er.skeleton.skeleton.empty()) {
        for (int y = 0; y < er.skeleton.skeleton.rows; ++y) {
            const uchar* row = er.skeleton.skeleton.ptr<uchar>(y);
            for (int x = 0; x < er.skeleton.skeleton.cols; ++x) {
                if (!row[x]) {
                    continue;
                }
                record.skeletonPixels.push_back(
                    cv::Point2f(static_cast<float>(x) + origin.x,
                                static_cast<float>(y) + origin.y));
            }
        }
    }

    for (int idx : er.skeleton.endpointIndices) {
        if (idx < 0 || idx >= static_cast<int>(er.skeleton.points.size())) {
            continue;
        }
        const cv::Point& point = er.skeleton.points[idx];
        record.skeletonEndpointPoints.push_back(
            cv::Point2f(static_cast<float>(point.x) + origin.x,
                        static_cast<float>(point.y) + origin.y));
    }

    record.distanceTransform = Debug::DistanceTransformDebug{};
    record.distanceTransform.localBounds = er.localBounds;
    if (!er.distTransform.empty() && er.distTransform.type() == CV_32F) {
        record.distanceTransform.rows = er.distTransform.rows;
        record.distanceTransform.cols = er.distTransform.cols;
        record.distanceTransform.values.reserve(
            static_cast<size_t>(er.distTransform.rows * er.distTransform.cols));
        for (int y = 0; y < er.distTransform.rows; ++y) {
            const float* row = er.distTransform.ptr<float>(y);
            for (int x = 0; x < er.distTransform.cols; ++x) {
                record.distanceTransform.values.push_back(row[x]);
            }
        }
    }
}


namespace Centerline {

// Public wrapper for the processor's internal polyline length helper.
float arcLength(const std::vector<cv::Point2f>& points)
{
    return arcLen(points);
}

// Run one frame of the centerline pipeline and update predictor/previous-frame state.
CenterlineFrameResult processFrame(const CenterlineFrameContext& ctx,
                                   const CenterlineFrameRequest& req,
                                   CenterlineSweepState& state,
                                   CenterlineFrameIo& io)
{
    CenterlineFrameResult result;
    if (!ctx.sortedPoints) {
        return result;
    }

    auto& predictor = state.predictor;
    auto& prevState = state.prevState;
    const std::vector<Tracking::WormTrackPoint>& points = *ctx.sortedPoints;
    const int i = req.pointIndex;

    auto inMergeGroupAtFrame = [&](int frameNumber) -> bool {
        const QList<QList<int>> groups = io.getMergeGroupsForFrame(frameNumber);
        for (const QList<int>& g : groups) {
            if (g.size() > 1 && g.contains(ctx.wormId)) return true;
        }
        return false;
    };

    auto nearestCandidateIdx = [](const Tracking::DetectedBlob& b,
                                  const cv::Point2f& target) -> int {
        int bestIdx = -1;
        float bestDistSq = std::numeric_limits<float>::max();
        for (size_t idx = 0; idx < b.tipCandidates.size(); ++idx) {
            const cv::Point2f d = b.tipCandidates[idx].point - target;
            const float dsq = d.x * d.x + d.y * d.y;
            if (dsq < bestDistSq) { bestDistSq = dsq; bestIdx = static_cast<int>(idx); }
        }
        return bestIdx;
    };

    auto loadPreviousFrameContext =
        [&](int frameNumber, int frameStep,
            Centerline::HeadTailPredictor& outPredictor,
            CenterlineState& outPrevState,
            float& outLocalRefLength) -> bool {
        auto blobAt = [&](int f, Tracking::DetectedBlob& out) -> bool {
            const QMap<int, Tracking::DetectedBlob> blobs = io.getDetectedBlobsForFrame(f);
            if (!blobs.contains(ctx.wormId)) return false;
            out = blobs[ctx.wormId];
            return out.isValid && !out.contourPoints.empty();
        };

        Tracking::DetectedBlob prevBlob;
        if (!blobAt(frameNumber - frameStep, prevBlob)) {
            return false;
        }

        outPredictor = Centerline::HeadTailPredictor{};
        const auto prevHead =
            (prevBlob.assignedHeadTipIdx >= 0 &&
             prevBlob.assignedHeadTipIdx < static_cast<int>(prevBlob.tipCandidates.size()))
                ? prevBlob.tipCandidates[prevBlob.assignedHeadTipIdx].point
                : cv::Point2f(-1.f, -1.f);
        const auto prevTail =
            (prevBlob.assignedTailTipIdx >= 0 &&
             prevBlob.assignedTailTipIdx < static_cast<int>(prevBlob.tipCandidates.size()))
                ? prevBlob.tipCandidates[prevBlob.assignedTailTipIdx].point
                : cv::Point2f(-1.f, -1.f);
        outPredictor.lastHeadPos = prevHead;
        outPredictor.lastTailPos = prevTail;
        outPredictor.hasPrev = (prevHead.x >= 0.f || prevTail.x >= 0.f);

        Tracking::DetectedBlob prevPrevBlob;
        if (blobAt(frameNumber - (2 * frameStep), prevPrevBlob)) {
            const auto prevPrevHead =
                (prevPrevBlob.assignedHeadTipIdx >= 0 &&
                 prevPrevBlob.assignedHeadTipIdx < static_cast<int>(prevPrevBlob.tipCandidates.size()))
                    ? prevPrevBlob.tipCandidates[prevPrevBlob.assignedHeadTipIdx].point
                    : cv::Point2f(-1.f, -1.f);
            const auto prevPrevTail =
                (prevPrevBlob.assignedTailTipIdx >= 0 &&
                 prevPrevBlob.assignedTailTipIdx < static_cast<int>(prevPrevBlob.tipCandidates.size()))
                    ? prevPrevBlob.tipCandidates[prevPrevBlob.assignedTailTipIdx].point
                    : cv::Point2f(-1.f, -1.f);
            if (prevPrevHead.x >= 0.f && prevHead.x >= 0.f)
                outPredictor.velHead = prevHead - prevPrevHead;
            if (prevPrevTail.x >= 0.f && prevTail.x >= 0.f)
                outPredictor.velTail = prevTail - prevPrevTail;
            const bool havePrevPrevCenter =
                prevPrevBlob.centerlinePoints.size() >= 2 &&
                prevBlob.centerlinePoints.size() >= 2;
            if (havePrevPrevCenter) {
                const cv::Point2f prevPrevCenter =
                    prevPrevBlob.centerlinePoints[prevPrevBlob.centerlinePoints.size() / 2];
                const cv::Point2f prevCenter =
                    prevBlob.centerlinePoints[prevBlob.centerlinePoints.size() / 2];
                outPredictor.velCenter = prevCenter - prevPrevCenter;
            }
            outPredictor.hasVelocity =
                (prevPrevHead.x >= 0.f && prevHead.x >= 0.f) ||
                (prevPrevTail.x >= 0.f && prevTail.x >= 0.f) ||
                havePrevPrevCenter;
        }

        outPrevState = CenterlineState{};
        if (prevBlob.centerlinePoints.size() >= 2) {
            outPrevState.points.assign(prevBlob.centerlinePoints.begin(),
                                       prevBlob.centerlinePoints.end());
            outPrevState.blobCentroid = blobCentroid(prevBlob);
            outPrevState.blob = prevBlob;
            outPrevState.valid = true;
            outPrevState.turningAngle = centerlineCrossSum(outPrevState.points);
            outLocalRefLength = arcLen(outPrevState.points);
            outPredictor.lastCenterPos =
                outPrevState.points[outPrevState.points.size() / 2];
        }

        if (outLocalRefLength > 0.f) {
            outPredictor.refDistance = std::max(8.f, 0.5f * outLocalRefLength);
        }
        return outPredictor.hasPrev || outPrevState.valid;
    };

if (i < 0 || i >= static_cast<int>(points.size())) return result;
const Tracking::WormTrackPoint& tp = points[i];

if (tp.quality == Tracking::TrackPointQuality::Lost) {
    predictor = Centerline::HeadTailPredictor{};
    prevState.valid = false;
    return result;
}

QMap<int, Tracking::DetectedBlob> frameBlobs =
    io.getDetectedBlobsForFrame(tp.frameNumberOriginal);
if (!frameBlobs.contains(ctx.wormId)) return result;
Tracking::DetectedBlob blob = frameBlobs[ctx.wormId];
if (!blob.isValid || blob.contourPoints.empty()) return result;
blob.hasCenterlineCutPoint = false;

const bool inMerge = inMergeGroupAtFrame(tp.frameNumberOriginal);
Centerline::HeadTailPredictor framePredictor = predictor;
CenterlineState framePrevState = prevState;
float frameRefLength = ctx.refLength;
if (!req.isKeyframeBootstrap) {
    float localRefLength = frameRefLength;
    if (loadPreviousFrameContext(tp.frameNumberOriginal, req.step,
                                 framePredictor,
                                 framePrevState,
                                 localRefLength)) {
        if (localRefLength > 0.f) {
            frameRefLength = localRefLength;
        }
    }
}

// ── STEP 1: detect endpoints, write back tip data ───────────
const Centerline::TipFeatureBaseline baseline =
    io.getTipBaseline(ctx.wormId);
const bool captureDebug =
    ctx.captureDebug;
Debug::CenterlineFrameDebug debugRecord;
debugRecord.wormId = ctx.wormId;
debugRecord.frameNumber = tp.frameNumberOriginal;
debugRecord.directionStep = req.step;
debugRecord.keyframeBootstrap = req.isKeyframeBootstrap;
debugRecord.inMergeGroup = inMerge;
debugRecord.predictorBefore = framePredictor;
debugRecord.baselineBefore = baseline;
debugRecord.refLength = frameRefLength;
debugRecord.previousTurningAngle = framePrevState.turningAngle;
debugRecord.predictedHead = framePredictor.hasVelocity
    ? framePredictor.lastHeadPos + framePredictor.velHead
    : framePredictor.lastHeadPos;
debugRecord.predictedTail = framePredictor.hasVelocity
    ? framePredictor.lastTailPos + framePredictor.velTail
    : framePredictor.lastTailPos;
debugRecord.predictedCenter = framePredictor.hasVelocity
    ? framePredictor.lastCenterPos + framePredictor.velCenter
    : framePredictor.lastCenterPos;
debugRecord.decisions << QStringLiteral("loaded live predictor and previous-frame state");

Centerline::EndpointResult er = Centerline::detectEndpoints(
        blob, framePredictor, baseline, inMerge);

// ── OMEGA UNZIPPER START ──────────────────────────────────────────
if (er.topology == Tracking::TopologyState::SelfCrossed && framePredictor.hasVelocity) {
    const cv::Point2f predictedHead = framePredictor.lastHeadPos + framePredictor.velHead;
    const cv::Point2f predictedTail = framePredictor.lastTailPos + framePredictor.velTail;
    float tipDist = ptDist(predictedHead, predictedTail);

    // 1. Proximity Check
    if (baseline.isReliable() && tipDist < baseline.meanWidth * 2.5f) {
        cv::Point2f midPoint = (predictedHead + predictedTail) * 0.5f;
        int localX = static_cast<int>(std::lround(midPoint.x - er.localBounds.x));
        int localY = static_cast<int>(std::lround(midPoint.y - er.localBounds.y));

        if (localX >= 0 && localY >= 0 &&
            localX < er.distTransform.cols && localY < er.distTransform.rows) {

            // 2. Width Check
            float localRadius = er.distTransform.at<float>(localY, localX);
            float normalRadius = baseline.meanWidth / 2.0f;

            if (localRadius > normalRadius * 1.4f) {
                Tracking::DetectedBlob splitBlob = blob;

                // 3. The Cut (Draw black line between predicted tips)
                cv::Point2f cutA = predictedHead - cv::Point2f(static_cast<float>(er.localBounds.x), static_cast<float>(er.localBounds.y));
                cv::Point2f cutB = predictedTail - cv::Point2f(static_cast<float>(er.localBounds.x), static_cast<float>(er.localBounds.y));

                if (Tracking::populateCenterlineFromContourWithCut(splitBlob, cutA, cutB, 2)) {

                    // 4. Re-evaluate topology
                    Centerline::EndpointResult splitEr =
                        Centerline::detectEndpoints(splitBlob, framePredictor, baseline, inMerge);

                    if (splitEr.topology == Tracking::TopologyState::Clean) {
                        blob = splitBlob;
                        er = splitEr;
                        debugRecord.decisions << QStringLiteral("Omega handle detected and unzipped via predictive DT");
                    }
                }
            }
        }
    }
}
// ── OMEGA UNZIPPER END ────────────────────────────────────────────

if (captureDebug) {
    captureEndpointDebug(er, debugRecord);
}

// Convert TrueTips → TipCandidates. Source = SkeletonEndpoint
// for ALL tips so the renderer's filled-green-dot styling
// applies regardless of whether the position came from the
// skeleton-snap or the curvature-peak extension.
blob.tipCandidates.clear();
for (const Centerline::TrueTip& t : er.tips) {
    Tracking::TipCandidate tc;
    tc.point     = t.point;
    tc.curvature = t.curvature;
    tc.width     = t.width;
    tc.source    = t.extended ? Tracking::TipCandidate::Source::CurvaturePeak
                              : Tracking::TipCandidate::Source::SkeletonEndpoint;
    blob.tipCandidates.push_back(tc);
}
blob.assignedHeadTipIdx = er.headIdx;
blob.assignedTailTipIdx = er.tailIdx;
blob.topologyState      = er.topology;
debugRecord.decisions << QStringLiteral("detectEndpoints topology=%1 tips=%2 headIdx=%3 tailIdx=%4")
                             .arg(Tracking::topologyStateToString(er.topology))
                             .arg(static_cast<int>(blob.tipCandidates.size()))
                             .arg(blob.assignedHeadTipIdx)
                             .arg(blob.assignedTailTipIdx);
enforceSelfCrossedTwoTipPredictorRoles(blob,
                                       framePredictor,
                                       &debugRecord.decisions);
if (framePrevState.valid) {
    enforceTwoTipCenterlineOrderRoles(blob,
                                      framePrevState.points,
                                      framePrevState.blobCentroid,
                                      &debugRecord.decisions);
}
debugRecord.topology = er.topology;
debugRecord.assignedHeadTipIdx = blob.assignedHeadTipIdx;
debugRecord.assignedTailTipIdx = blob.assignedTailTipIdx;
debugRecord.tipCandidates = blob.tipCandidates;

// Copy bilateral cap debug — parallel to tipCandidates, with role labels.
debugRecord.tipCapDebug = er.tipCapDebug;
debugRecord.tipCapRoles.resize(er.tipCapDebug.size());
for (int ci = 0; ci < static_cast<int>(er.tipCapDebug.size()); ++ci) {
    if (ci == blob.assignedHeadTipIdx)      debugRecord.tipCapRoles[ci] = QStringLiteral("head");
    else if (ci == blob.assignedTailTipIdx) debugRecord.tipCapRoles[ci] = QStringLiteral("tail");
    else                                    debugRecord.tipCapRoles[ci] = QString();
}

// Sample baseline (curvature + width) on Clean frames with two
// tips. Body-length sample is taken AFTER req.step 3 below using
// the resampled centerline.
const bool cleanWithTwoTips =
    er.topology == Tracking::TopologyState::Clean &&
    er.tips.size() == 2;
if (cleanWithTwoTips) {
    for (const Centerline::TrueTip& t : er.tips) {
        io.recordTipFeatureSample(
            ctx.wormId, std::abs(t.curvature), t.width);
    }
}

// Keyframe bootstrap: detectEndpoints leaves head/tail at -1
// because predictor.hasPrev is false on the first call. With
// ≥2 tips we arbitrarily pick (0, 1) so Step 2's Clean branch
// can build a centerline; head/tail will be re-derived from
// the centerline orientation after Step 3.
if (req.isKeyframeBootstrap && !er.tips.empty()) {
    if (er.tips.size() >= 2 && blob.assignedHeadTipIdx < 0) {
        blob.assignedHeadTipIdx = 0;
        blob.assignedTailTipIdx = 1;
        debugRecord.decisions << QStringLiteral("keyframe bootstrap assigned head/tail candidate indices 0/1");
    } else if (blob.assignedHeadTipIdx < 0 &&
               blob.assignedTailTipIdx < 0) {
        blob.assignedHeadTipIdx = 0;
        debugRecord.decisions << QStringLiteral("keyframe bootstrap assigned single head candidate index 0");
    }
    debugRecord.assignedHeadTipIdx = blob.assignedHeadTipIdx;
    debugRecord.assignedTailTipIdx = blob.assignedTailTipIdx;
}

// ── STEP 2: build centerline based on topology ──────────────
std::vector<cv::Point2f> centerline;
cv::Point2f overlapCenter(0.f, 0.f);
bool hasOverlap = false;
bool snakeRan = false;
const cv::Point2f originOffset(static_cast<float>(er.localBounds.x),
                               static_cast<float>(er.localBounds.y));
// angleThreshold used by both RHR arc-pick (Step 2) and RHR veto (Step 4).
const float angleThreshold = static_cast<float>(
    std::max(0.0, ctx.snakeParams.orientationAngleThreshold));

// Shared lambda: skeleton-arc dispatch for SelfCrossed frames.
// Finds both arcs of the skeleton from the known tip to the
// target (actual or predicted), picks by RHR then length, and
// registers the arc's far terminus as the hypothesised tip when
// there is no actual target.
auto runSkeletonArcDispatch =
    [&](const Tracking::DetectedBlob& dispBlob,
        const Centerline::EndpointResult& dispEr,
        const cv::Point2f& dispOrigin) -> bool {
    const int hIdx = dispBlob.assignedHeadTipIdx;
    const int tIdx = dispBlob.assignedTailTipIdx;
    const bool hasHead = (hIdx >= 0 &&
        hIdx < static_cast<int>(dispBlob.tipCandidates.size()));
    const bool hasTail = (tIdx >= 0 &&
        tIdx < static_cast<int>(dispBlob.tipCandidates.size()));
    if (!hasHead && !hasTail) return false;

    const int knownIdx   = hasHead ? hIdx : tIdx;
    const cv::Point2f knownPos = dispBlob.tipCandidates[knownIdx].point;
    const int srcNode = nearestSkeletonNode(dispEr.skeleton, knownPos, dispOrigin);
    if (srcNode < 0) return false;

    // Target: actual other tip (D-2) or predicted position (D-3).
    const bool hasActualTarget = hasHead && hasTail;
    cv::Point2f targetPos(-1.f, -1.f);
    const bool hasPredictedCenter =
        framePredictor.lastCenterPos.x != 0.f ||
        framePredictor.lastCenterPos.y != 0.f;
    cv::Point2f predictedCenter = framePredictor.hasVelocity
        ? framePredictor.lastCenterPos + framePredictor.velCenter
        : framePredictor.lastCenterPos;
    if (hasPredictedCenter) {
        cv::Point2f snappedCenter;
        if (nearestMaskPoint(dispBlob, predictedCenter, snappedCenter)) {
            predictedCenter = snappedCenter;
        }
    }
    if (hasActualTarget) {
        targetPos = dispBlob.tipCandidates[hasHead ? tIdx : hIdx].point;
    } else if (framePredictor.hasPrev) {
        const bool hiddenIsHead = !hasHead;
        const cv::Point2f& last = hiddenIsHead
            ? framePredictor.lastHeadPos : framePredictor.lastTailPos;
        const cv::Point2f& vel  = hiddenIsHead
            ? framePredictor.velHead    : framePredictor.velTail;
        if (last.x != 0.f || last.y != 0.f) {
            const Tracking::DetectedBlob* previousBlob =
                framePrevState.valid ? &framePrevState.blob : nullptr;
            const HiddenTipTarget predicted = predictHiddenTipTarget(
                dispBlob, previousBlob, last, vel, framePredictor.hasVelocity);
            debugRecord.hiddenTipMaskDiffArea = predicted.maskDiffArea;
            debugRecord.hiddenTipMaskDiffSelectedArea = predicted.selectedMaskDiffArea;
            if (predicted.hasTarget) {
                targetPos = predicted.target;
                debugRecord.hiddenTipTarget = targetPos;
                debugRecord.decisions << QStringLiteral("D-3 predicted hidden target at (%1,%2)")
                                             .arg(targetPos.x, 0, 'f', 2)
                                             .arg(targetPos.y, 0, 'f', 2);
                debugRecord.decisions << QStringLiteral("D-3 mask-diff area total=%1 selected=%2")
                                             .arg(predicted.maskDiffArea)
                                             .arg(predicted.selectedMaskDiffArea);
            }
        }
    }

    if (!hasActualTarget && (targetPos.x != -1.f || targetPos.y != -1.f) &&
        framePrevState.valid &&
        framePrevState.blob.topologyState == Tracking::TopologyState::SelfCrossed &&
        dispEr.topology == Tracking::TopologyState::SelfCrossed) {
        const QMap<int, Tracking::DetectedBlob> prevPrevBlobs =
            io.getDetectedBlobsForFrame(tp.frameNumberOriginal - (2 * req.step));
        const bool prevPrevSelfCrossed =
            prevPrevBlobs.contains(ctx.wormId) &&
            prevPrevBlobs[ctx.wormId].topologyState == Tracking::TopologyState::SelfCrossed;
        const bool secondSelfCrossedFrame = !prevPrevSelfCrossed;
        if (secondSelfCrossedFrame) {
            const JunctionSelection junction = selectLoopAwareJunction(
                dispEr.skeleton, srcNode, targetPos, dispOrigin, frameRefLength);
            if (junction.valid && !junction.fallbackUsed) {
                const cv::Point& jp = dispEr.skeleton.points[junction.junctionIdx];
                const cv::Point2f junctionPoint(
                    static_cast<float>(jp.x) + dispOrigin.x,
                    static_cast<float>(jp.y) + dispOrigin.y);
                const float bumpRadius = frameRefLength > 0.f
                    ? std::max(8.f, 0.35f * frameRefLength)
                    : 12.f;
                const float curvatureThreshold =
                    baseline.curvatureSamples >= 4
                        ? std::max(0.12f, 0.75f * baseline.meanAbsCurvature)
                        : 0.12f;
                bool hasSecondBumpNearJunction = false;
                for (int tipIdx = 0; tipIdx < static_cast<int>(dispEr.tips.size()); ++tipIdx) {
                    if (tipIdx == knownIdx) {
                        continue;
                    }
                    const Centerline::TrueTip& tip = dispEr.tips[tipIdx];
                    if (ptDist(tip.point, junctionPoint) <= bumpRadius &&
                        std::abs(tip.curvature) >= curvatureThreshold) {
                        hasSecondBumpNearJunction = true;
                        break;
                    }
                }
                debugRecord.decisions << QStringLiteral("D-3 second SelfCrossed frame: junction target candidate=(%1,%2) secondBump=%3")
                                             .arg(junctionPoint.x, 0, 'f', 2)
                                             .arg(junctionPoint.y, 0, 'f', 2)
                                             .arg(hasSecondBumpNearJunction ? "Y" : "N");
                if (!hasSecondBumpNearJunction) {
                    targetPos = junctionPoint;
                    debugRecord.hiddenTipTarget = targetPos;
                    debugRecord.decisions << QStringLiteral("D-3 second SelfCrossed frame: hidden target overridden to loop junction");
                }
            }
        }
    }

    int goalNode = -1;
    if (targetPos.x != -1.f || targetPos.y != -1.f) {
        goalNode = nearestSkeletonNode(dispEr.skeleton, targetPos, dispOrigin);
    } else {
        goalNode = farthestSkeletonNode(dispEr.skeleton, srcNode);
    }
    if (!hasActualTarget && goalNode == srcNode &&
        dispEr.topology == Tracking::TopologyState::SelfCrossed) {
        const float targetToKnown = ptDist(targetPos, knownPos);
        const bool weakMaskDiff =
            debugRecord.hiddenTipMaskDiffArea > 0 &&
            debugRecord.hiddenTipMaskDiffArea < 0.35f * frameRefLength;
        if (targetToKnown < 0.25f * frameRefLength || weakMaskDiff) {
            const JunctionSelection junction = selectLoopAwareJunction(
                dispEr.skeleton, srcNode, targetPos, dispOrigin, frameRefLength);
            if (junction.valid && !junction.fallbackUsed) {
                const cv::Point& jp = dispEr.skeleton.points[junction.junctionIdx];
                targetPos = cv::Point2f(static_cast<float>(jp.x) + dispOrigin.x,
                                        static_cast<float>(jp.y) + dispOrigin.y);
                goalNode = nearestSkeletonNode(dispEr.skeleton, targetPos, dispOrigin);
                debugRecord.hiddenTipTarget = targetPos;
                debugRecord.decisions << QStringLiteral("D-3 degenerate hidden target near known tip; overridden to loop junction (%1,%2)")
                                             .arg(targetPos.x, 0, 'f', 2)
                                             .arg(targetPos.y, 0, 'f', 2);
            }
        }
    }
    if (goalNode < 0 || goalNode == srcNode) return false;

    // SelfCrossed routing is loop-aware for both D-2 and D-3.
    // D-2 has an actual second tip target; D-3 has a predicted
    // hidden target. In both cases, avoid the crossing shortcut by
    // routing through the loop-valid junction candidates first.
    std::vector<cv::Point2f> arcA, arcB;
    std::vector<cv::Point2f> chosen;
    if ((targetPos.x != -1.f || targetPos.y != -1.f) &&
        [&]() {
            std::vector<cv::Point2f> previousForRoute;
            if (framePrevState.valid) {
                previousForRoute = framePrevState.points;
                if (!hasHead) {
                    std::reverse(previousForRoute.begin(), previousForRoute.end());
                }
            }
            D3RouteDebug routeDebug;
            routeDebug.startIsHead = hasHead;
            const bool ok = skeletonPathTowardPredictedHidden(dispEr.skeleton,
                                                               srcNode,
                                                               targetPos,
                                                               hasActualTarget,
                                                               hasHead,
                                                               predictedCenter,
                                                               hasPredictedCenter,
                                                               dispOrigin,
                                                               previousForRoute,
                                                               framePrevState.turningAngle,
                                                               angleThreshold,
                                                               ctx.nPts,
                                                               frameRefLength,
                                                               &debugRecord.decisions,
                                                               &routeDebug,
                                                               chosen);
            if (routeDebug.available) {
                debugRecord.d3RouteDebugAvailable = true;
                debugRecord.d3RouteStartIsHead = routeDebug.startIsHead;
                debugRecord.d3SelectedCandidate = routeDebug.selectedCandidate;
                debugRecord.d3JunctionClusterCount = routeDebug.junctionClusterCount;
                debugRecord.d3SelectedJunctionCluster = routeDebug.selectedJunctionCluster;
                debugRecord.d3JunctionFallbackUsed = routeDebug.junctionFallbackUsed;
                debugRecord.d3RouteStart = routeDebug.start;
                debugRecord.d3RouteJunction = routeDebug.junction;
                debugRecord.d3RouteCenter = routeDebug.center;
                debugRecord.d3RouteEnd = routeDebug.end;
                debugRecord.d3CandidatePaths = routeDebug.candidatePaths;
                debugRecord.d3JunctionDiagnostics = routeDebug.junctionDiagnostics;
            }
            return ok;
        }()) {
        debugRecord.decisions << QStringLiteral("%1 loop-aware whole-path dispatch selected centerline")
                                     .arg(hasActualTarget ? "D-2" : "D-3");
    } else if (skeletonBothArcs(dispEr.skeleton, srcNode, goalNode,
                                dispOrigin, arcA, arcB)) {
        chosen = pickArcByRHR(arcA, arcB,
                              framePrevState.turningAngle,
                              angleThreshold, frameRefLength);
    } else if (!arcA.empty()) {
        chosen = arcA; // open-curve skeleton: only one arc
    } else {
        return false;
    }
    if (chosen.size() < 2) return false;

    // Snap endpoints to the actual tip positions if curvature-extended.
    // Takes into account known may be head OR tail
    if (hasActualTarget) {
        // D-2: Both are known. Ensure orientation matches Head->Tail assumption.
        if (hasHead && dispEr.tips[hIdx].extended) chosen.front() = dispEr.tips[hIdx].point;
        if (hasTail && dispEr.tips[tIdx].extended) chosen.back()  = dispEr.tips[tIdx].point;
    } else {
        // D-3: Only one is known. 'chosen.front()' is ALWAYS the known tip.
        const int knownIdx = hasHead ? hIdx : tIdx;
        if (dispEr.tips[knownIdx].extended) {
            chosen.front() = dispEr.tips[knownIdx].point;
        }
    }
    if (!hasActualTarget && framePredictor.hasPrev) {
        const cv::Point2f& knownLast = hasHead
            ? framePredictor.lastHeadPos : framePredictor.lastTailPos;
        const cv::Point2f& knownVel = hasHead
            ? framePredictor.velHead : framePredictor.velTail;
        if (knownLast.x != 0.f || knownLast.y != 0.f) {
            cv::Point2f knownPred = framePredictor.hasVelocity
                ? (knownLast + knownVel)
                : knownLast;
            cv::Point2f snappedKnown;
            if (nearestMaskPoint(dispBlob, knownPred, snappedKnown)) {
                knownPred = snappedKnown;
            }
            chosen.front() = knownPred;
        }
    }

    centerline = std::move(chosen);

    // D-3: register the arc terminus as the hypothesised hidden tip.
    if (!hasActualTarget) {
        const bool havePredictedHidden =
            targetPos.x != -1.f || targetPos.y != -1.f;
        const cv::Point2f hiddenPoint =
            havePredictedHidden ? targetPos : centerline.back();
        centerline.back() = hiddenPoint;
        debugRecord.hiddenTipHypothesized = true;
        debugRecord.hiddenTipTarget = targetPos;
        debugRecord.hiddenTipFinal = hiddenPoint;
        Tracking::TipCandidate hyp;
        hyp.point  = hiddenPoint;
        hyp.source = Tracking::TipCandidate::Source::HypothesizedHidden;
        blob.tipCandidates.push_back(hyp);
        const int newIdx = static_cast<int>(blob.tipCandidates.size()) - 1;
        if (hasHead) blob.assignedTailTipIdx = newIdx;
        else {
            blob.assignedHeadTipIdx = newIdx;
            std::reverse(centerline.begin(), centerline.end());
        }
        debugRecord.decisions << QStringLiteral("D-3 registered hypothesized hidden tip at (%1,%2)")
                                     .arg(hiddenPoint.x, 0, 'f', 2)
                                     .arg(hiddenPoint.y, 0, 'f', 2);
    }
    return true;
};

if (er.topology == Tracking::TopologyState::Clean &&
    blob.assignedHeadTipIdx >= 0 &&
    blob.assignedTailTipIdx >= 0 &&
    static_cast<int>(er.skeleton.endpointIndices.size()) >
        std::max(blob.assignedHeadTipIdx, blob.assignedTailTipIdx)) {

    const int hGraphIdx =
        er.skeleton.endpointIndices[blob.assignedHeadTipIdx];
    const int tGraphIdx =
        er.skeleton.endpointIndices[blob.assignedTailTipIdx];

    std::vector<cv::Point2f> graphPath;
    if (skeletonGraphPath(er.skeleton, hGraphIdx, tGraphIdx,
                          originOffset, graphPath)) {
        centerline = std::move(graphPath);
        debugRecord.branch = Debug::CenterlineBranch::D1CleanGraphPath;
        debugRecord.decisions << QStringLiteral("D-1 clean skeleton graph path selected");
        if (!centerline.empty())
            centerline.front() = trustedTipPointForCleanD1(er.tips[blob.assignedHeadTipIdx]);
        if (!centerline.empty())
            centerline.back() = trustedTipPointForCleanD1(er.tips[blob.assignedTailTipIdx]);

        // If D-1 result is suspiciously short the worm is
        // tightly self-coiled but had no visible hole in the
        // mask. Punch a synthetic hole at the DT maximum and
        // re-run the SelfCrossed skeleton-arc dispatch.
        if (frameRefLength > 0.f &&
            arcLen(centerline) < 0.5f * frameRefLength) {
            Tracking::DetectedBlob holeBlob;
            if (addSyntheticHoleAtDTMax(blob, er.distTransform,
                                         er.localBounds, holeBlob)) {
                debugRecord.syntheticHoleUsed = true;
                debugRecord.decisions << QStringLiteral("D-1 path was short; synthetic hole retry attempted");
                const Centerline::EndpointResult er2 =
                    Centerline::detectEndpoints(
                        holeBlob, framePredictor, baseline, inMerge);
                const cv::Point2f origin2(
                    static_cast<float>(er2.localBounds.x),
                    static_cast<float>(er2.localBounds.y));
                // Temporarily swap blob state so the lambda
                // writes into the right place.
                std::vector<cv::Point2f> prevCl = centerline;
                Tracking::DetectedBlob savedBlob = blob;
                blob = holeBlob;
                blob.tipCandidates.clear();
                for (const Centerline::TrueTip& t : er2.tips) {
                    Tracking::TipCandidate tc;
                    tc.point     = t.point;
                    tc.curvature = t.curvature;
                    tc.width     = t.width;
                    tc.source    = t.extended ? Tracking::TipCandidate::Source::CurvaturePeak
                                              : Tracking::TipCandidate::Source::SkeletonEndpoint;
                    blob.tipCandidates.push_back(tc);
                }
                blob.assignedHeadTipIdx = er2.headIdx;
                blob.assignedTailTipIdx = er2.tailIdx;
                blob.topologyState      = er2.topology;
                centerline.clear();
                if (!runSkeletonArcDispatch(blob, er2, origin2)) {
                    // Restore if synthetic-hole dispatch also failed.
                    blob = savedBlob;
                    centerline = prevCl;
                    debugRecord.decisions << QStringLiteral("synthetic hole retry failed; restored D-1 path");
                }
                else {
                    debugRecord.branch = Debug::CenterlineBranch::D1SyntheticHoleRetry;
                    debugRecord.decisions << QStringLiteral("synthetic hole retry supplied centerline");
                }
            }
        }
    }
}
else if (er.topology == Tracking::TopologyState::SelfCrossed) {
    // ── Hole-first: ensure the blob has a ring hole ──────────────
    // If no contour hole exists, punch a synthetic one at the
    // distance-transform maximum (the widest body cross-section).
    // This ensures the 0-tip ring-cut routing always has a hole
    // to work with, and unifies the no-hole SelfCrossed case
    // rather than silently falling through to D-4.
    if (blob.holeContourPoints.empty()) {
        Tracking::DetectedBlob holeBlob;
        if (addSyntheticHoleAtDTMax(blob, er.distTransform,
                                    er.localBounds, holeBlob)) {
            const Centerline::EndpointResult er2 =
                Centerline::detectEndpoints(
                    holeBlob, framePredictor, baseline, inMerge);
            holeBlob.tipCandidates.clear();
            for (const Centerline::TrueTip& t : er2.tips) {
                Tracking::TipCandidate tc;
                tc.point     = t.point;
                tc.curvature = t.curvature;
                tc.width     = t.width;
                tc.source    = t.extended
                    ? Tracking::TipCandidate::Source::CurvaturePeak
                    : Tracking::TipCandidate::Source::SkeletonEndpoint;
                holeBlob.tipCandidates.push_back(tc);
            }
            holeBlob.assignedHeadTipIdx = er2.headIdx;
            holeBlob.assignedTailTipIdx = er2.tailIdx;
            holeBlob.topologyState      = er2.topology;
            blob = holeBlob;
            er   = er2;
            debugRecord.syntheticHoleUsed = true;
            debugRecord.decisions << QStringLiteral(
                "SelfCrossed no-hole: punched synthetic hole at DT max, re-detected endpoints");
        }
    }

    // Derive local origin from the (potentially updated) er so
    // skeleton node lookups are in the right coordinate frame.
    const cv::Point2f scOrigin(
        static_cast<float>(er.localBounds.x),
        static_cast<float>(er.localBounds.y));

    // ── Tip-count dispatch ────────────────────────────────────────
    const int hIdx = blob.assignedHeadTipIdx;
    const int tIdx = blob.assignedTailTipIdx;
    const bool hasHead = (hIdx >= 0 && hIdx < static_cast<int>(blob.tipCandidates.size()));
    const bool hasTail = (tIdx >= 0 && tIdx < static_cast<int>(blob.tipCandidates.size()));

    if (hasHead || hasTail) {
        // D-2 (two tips) / D-3 (one tip): trace the skeleton
        // arc from the known tip to the target, picking the arc
        // whose cross-sum sign matches the previous frame (RHR).
        debugRecord.branch = (hasHead && hasTail)
            ? Debug::CenterlineBranch::D2TwoKnownTips
            : Debug::CenterlineBranch::D3OneKnownTipHiddenPrediction;
        debugRecord.decisions << QStringLiteral("%1 skeleton-arc dispatch attempted")
                                     .arg(Debug::centerlineBranchToString(debugRecord.branch));
        runSkeletonArcDispatch(blob, er, scOrigin);
    }
    else if (!blob.holeContourPoints.empty()) {
        // 0 tips, ring (real or synthetic): route between predicted
        // endpoint positions using the ring skeleton or a cut.
        debugRecord.branch = Debug::CenterlineBranch::ZeroTipRingCut;
        const bool hasPredictedTips =
            framePredictor.hasPrev &&
            (debugRecord.predictedHead.x != 0.f || debugRecord.predictedHead.y != 0.f) &&
            (debugRecord.predictedTail.x != 0.f || debugRecord.predictedTail.y != 0.f);
        const bool hasCenterPrediction =
            framePredictor.lastCenterPos.x != 0.f ||
            framePredictor.lastCenterPos.y != 0.f;
        cv::Point2f cutPoint(-1.f, -1.f);
        bool zeroTipOk = false;
        if (hasPredictedTips) {
            zeroTipOk = selectZeroTipRingGraphCenterline(er.skeleton,
                                                          scOrigin,
                                                          debugRecord.predictedHead,
                                                          debugRecord.predictedTail,
                                                          debugRecord.predictedCenter,
                                                          hasCenterPrediction,
                                                          framePrevState.turningAngle,
                                                          frameRefLength,
                                                          &debugRecord.decisions,
                                                          centerline);
        }
        if (!zeroTipOk) {
            zeroTipOk = selectZeroTipRingCutCenterline(blob,
                                                       debugRecord.predictedHead,
                                                       debugRecord.predictedTail,
                                                       hasPredictedTips,
                                                       debugRecord.predictedCenter,
                                                       hasCenterPrediction,
                                                       framePrevState.turningAngle,
                                                       frameRefLength,
                                                       &debugRecord.decisions,
                                                       centerline,
                                                       cutPoint);
            if (zeroTipOk) {
                blob.centerlineCutPoint = cutPoint;
                blob.hasCenterlineCutPoint = true;
            }
        }

        if (zeroTipOk) {
            Tracking::TipCandidate hypHead;
            hypHead.point = centerline.front();
            hypHead.source = Tracking::TipCandidate::Source::HypothesizedHidden;
            Tracking::TipCandidate hypTail;
            hypTail.point = centerline.back();
            hypTail.source = Tracking::TipCandidate::Source::HypothesizedHidden;
            blob.tipCandidates.push_back(hypHead);
            blob.tipCandidates.push_back(hypTail);
            blob.assignedHeadTipIdx = static_cast<int>(blob.tipCandidates.size()) - 2;
            blob.assignedTailTipIdx = static_cast<int>(blob.tipCandidates.size()) - 1;
            debugRecord.decisions << QStringLiteral("0-tip ring supplied predictor-scored centerline");
        }
    }
    // Implicit: if 0 tips and synthetic hole punch also failed,
    // blob.holeContourPoints remains empty and centerline stays
    // empty → falls through to the D-4 fallback below.
}

if (centerline.empty()) {
    // D-4 fallback: legacy contour-skeleton path.
    debugRecord.fallbackUsed = true;
    debugRecord.branch = Debug::CenterlineBranch::D4FallbackContourSkeleton;
    Tracking::DetectedBlob fallback = blob;
    if (Tracking::populateCenterlineFromContour(fallback) &&
        fallback.centerlinePoints.size() >= 2) {
        centerline.assign(fallback.centerlinePoints.begin(),
                          fallback.centerlinePoints.end());
        debugRecord.decisions << QStringLiteral("D-4 fallback contour skeleton supplied centerline");
    }
}

debugRecord.initialCenterline = centerline;
debugRecord.initialArcLength = centerline.size() >= 2 ? arcLen(centerline) : 0.f;
debugRecord.tipCandidates = blob.tipCandidates;
debugRecord.assignedHeadTipIdx = blob.assignedHeadTipIdx;
debugRecord.assignedTailTipIdx = blob.assignedTailTipIdx;

if (centerline.size() < 2) {
    // No centerline producible. Persist what we DID compute
    // (tip data) so the renderer still shows green dots.
    io.setDetectedBlobForFrame(tp.frameNumberOriginal, ctx.wormId, blob);
    result.wroteBlob = true;
    result.blob = blob;
    debugRecord.decisions << QStringLiteral("no centerline producible; persisted endpoint/debug blob state only");
    if (captureDebug) {
        io.setCenterlineDebugFrame(debugRecord);
    }
    result.processed = true;
    result.debugRecord = debugRecord;
    prevState.valid = false;
    return result;
}

// ── STEP 3: resample to nPoints ─────────────────────────────
if (static_cast<int>(centerline.size()) != ctx.nPts)
    centerline = resample(centerline, ctx.nPts);
debugRecord.resampledCenterline = centerline;

if (cleanWithTwoTips) {
    io.recordBodyLengthSample(ctx.wormId, arcLen(centerline));
}

// ── STEP 4: snake refinement (Clean only) + RHR veto ────────
// SelfCrossed centerlines are produced by skeleton-arc dispatch
// (D-2/D-3) which traces the skeleton directly; no snake pass.
if (er.topology == Tracking::TopologyState::Clean) {
    cv::Mat mask;
    cv::Rect bounds;
    if (buildSnakeMask(blob, mask, bounds)) {
        const cv::Point2f pinH = (blob.assignedHeadTipIdx >= 0 &&
                                  blob.assignedHeadTipIdx <
                                    static_cast<int>(blob.tipCandidates.size()))
            ? blob.tipCandidates[blob.assignedHeadTipIdx].point
            : centerline.front();
        const cv::Point2f pinT = (blob.assignedTailTipIdx >= 0 &&
                                  blob.assignedTailTipIdx <
                                    static_cast<int>(blob.tipCandidates.size()))
            ? blob.tipCandidates[blob.assignedTailTipIdx].point
            : centerline.back();
        cv::Point2f tmpOverlap(0.f, 0.f);
        bool tmpHasOverlap = false;
        refineSnakeCore(blob, mask, bounds, centerline,
                        pinH, pinT, ctx.nPts, ctx.snakeParams,
                        tmpOverlap, tmpHasOverlap);
        snakeRan = true;
        debugRecord.decisions << QStringLiteral("snake refinement ran on clean topology frame");
    }
}

// Right-hand-rule orientation veto. The consecutive-segment
// cross-sum negates when traversal direction is reversed; worms
// don't instantaneously flip their coiling sense, so a sign flip
// across frames is strong evidence of a wrong orientation pick.
const float curTurning = centerlineCrossSum(centerline);
bool flipped = false;
constexpr float kCrossSumEpsilon = 1e-4f;
const bool allowRhrFlip =
    debugRecord.branch != Debug::CenterlineBranch::D1CleanGraphPath;
if (allowRhrFlip &&
    framePrevState.valid &&
    std::abs(framePrevState.turningAngle) > kCrossSumEpsilon &&
    std::abs(curTurning) > kCrossSumEpsilon &&
    curTurning * framePrevState.turningAngle < 0.f) {
    std::reverse(centerline.begin(), centerline.end());
    flipped = true;
    std::swap(blob.assignedHeadTipIdx, blob.assignedTailTipIdx);
    debugRecord.decisions << QStringLiteral("RHR orientation veto flipped centerline");
}

// Keyframe bootstrap re-derivation: now that we have a
// centerline, set head/tail from its natural orientation
// (front = head). This stabilises the convention regardless
// of which tip arbitrarily got tips[0] in detectEndpoints.
if (req.isKeyframeBootstrap && centerline.size() >= 2 &&
    !blob.tipCandidates.empty()) {
    const int headIdx = nearestCandidateIdx(blob, centerline.front());
    int       tailIdx = nearestCandidateIdx(blob, centerline.back());
    if (tailIdx == headIdx) tailIdx = -1;
    blob.assignedHeadTipIdx = headIdx;
    blob.assignedTailTipIdx = tailIdx;
    debugRecord.decisions << QStringLiteral("keyframe bootstrap re-derived head/tail from centerline orientation");
}

// Persist centerline + cut/overlap marker on the blob.
blob.centerlinePoints.assign(centerline.begin(), centerline.end());
if (hasOverlap) {
    blob.centerlineCutPoint = overlapCenter;
    blob.hasCenterlineCutPoint = true;
}

io.setDetectedBlobForFrame(tp.frameNumberOriginal, ctx.wormId, blob);
result.wroteBlob = true;
result.blob = blob;

debugRecord.snakeRan = snakeRan;
debugRecord.rhrFlipped = flipped;
debugRecord.finalCenterline = centerline;
debugRecord.finalArcLength = arcLen(centerline);
debugRecord.finalTurningAngle = flipped ? -curTurning : curTurning;
debugRecord.tipCandidates = blob.tipCandidates;
debugRecord.assignedHeadTipIdx = blob.assignedHeadTipIdx;
debugRecord.assignedTailTipIdx = blob.assignedTailTipIdx;
debugRecord.topology = blob.topologyState;
if (captureDebug) {
    io.setCenterlineDebugFrame(debugRecord);
}

// ── STEP 5: predictor update ────────────────────────────────
const int hIdx = blob.assignedHeadTipIdx;
const int tIdx = blob.assignedTailTipIdx;
if (hIdx >= 0 && hIdx < static_cast<int>(blob.tipCandidates.size())) {
    const cv::Point2f newH = blob.tipCandidates[hIdx].point;
    if (predictor.hasPrev)
        predictor.velHead = newH - predictor.lastHeadPos;
    predictor.lastHeadPos = newH;
}
if (tIdx >= 0 && tIdx < static_cast<int>(blob.tipCandidates.size())) {
    const cv::Point2f newT = blob.tipCandidates[tIdx].point;
    if (predictor.hasPrev)
        predictor.velTail = newT - predictor.lastTailPos;
    predictor.lastTailPos = newT;
}
if (centerline.size() >= 2) {
    const cv::Point2f newC = centerline[centerline.size() / 2];
    if (predictor.hasPrev)
        predictor.velCenter = newC - predictor.lastCenterPos;
    predictor.lastCenterPos = newC;
}
predictor.hasVelocity = predictor.hasPrev &&
                        (hIdx >= 0 || tIdx >= 0 || centerline.size() >= 2);
predictor.hasPrev = true;
if (frameRefLength > 0.f)
    predictor.refDistance = std::max(8.f, 0.5f * frameRefLength);

// CenterlineState carry for next frame's RHR check.
prevState.points       = centerline;
prevState.blobCentroid = blobCentroid(blob);
prevState.blob         = blob;
prevState.valid        = true;
prevState.turningAngle = flipped ? -curTurning : curTurning;

    result.processed = true;
    result.debugRecord = debugRecord;
    return result;

}

} // namespace Centerline
