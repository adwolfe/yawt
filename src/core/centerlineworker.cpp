#include "centerlineworker.h"
#include "../data/trackingcommon.h"
#include "../utils/loggingcategories.h"
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

// ── geometry helpers ────────────────────────────────────────────────────────

static float ptDist(const cv::Point2f& a, const cv::Point2f& b)
{
    float dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

static float arcLen(const std::vector<cv::Point2f>& pts)
{
    float len = 0.f;
    for (size_t i = 1; i < pts.size(); ++i)
        len += ptDist(pts[i - 1], pts[i]);
    return len;
}

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

static cv::Point2f nearestContourPoint(const std::vector<cv::Point>& contour,
                                       const cv::Point2f& target)
{
    const int idx = nearestContourIdx(contour, target);
    return cv::Point2f(static_cast<float>(contour[idx].x),
                       static_cast<float>(contour[idx].y));
}

static bool nearestHolePoint(const Tracking::DetectedBlob& blob,
                             const cv::Point2f& target,
                             cv::Point2f& outPoint)
{
    bool found = false;
    float bestD = std::numeric_limits<float>::max();
    for (const std::vector<cv::Point>& hole : blob.holeContourPoints) {
        for (const cv::Point& p : hole) {
            const float dx = static_cast<float>(p.x) - target.x;
            const float dy = static_cast<float>(p.y) - target.y;
            const float d = dx * dx + dy * dy;
            if (d < bestD) {
                bestD = d;
                outPoint = cv::Point2f(static_cast<float>(p.x), static_cast<float>(p.y));
                found = true;
            }
        }
    }
    return found;
}

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

// Centerline state we carry from one frame to the next.  Storing the entire
// resampled point sequence (not just nose/tail/midpoint) lets us orient by
// shape correspondence rather than endpoint distance — which is far more
// robust when the worm coils, because nose and tail can become spatially
// close yet the *order* along the line still distinguishes them.
struct CenterlineState {
    std::vector<cv::Point2f> points;
    cv::Point2f blobCentroid;
    Tracking::DetectedBlob blob;
    bool valid = false;
    // Total signed turning angle of the centerline (∑ signed inter-segment
    // angles, nose→tail). Negated when traversal direction is reversed.
    // Used as a right-hand-rule orientation consistency check between frames.
    float turningAngle = 0.f;

    cv::Point2f nose()     const { return points.front(); }
    cv::Point2f tail()     const { return points.back();  }
    cv::Point2f midpoint() const { return points[points.size() / 2]; }
};

// Total signed turning angle of a discrete polyline: the sum of signed
// angles between consecutive segment pairs (atan2 of cross/dot products).
// For a nose→tail traversal this equals the net tangent rotation from the
// first segment to the last. It is negated when the traversal direction is
// reversed — making it a reliable orientation invariant when the worm is
// non-trivially bent.  Returns 0 for point sets with fewer than 3 points.
static float totalSignedTurningAngle(const std::vector<cv::Point2f>& pts)
{
    float total = 0.f;
    const int n = static_cast<int>(pts.size());
    for (int i = 1; i < n - 1; ++i) {
        const cv::Point2f v1 = pts[i]     - pts[i - 1];
        const cv::Point2f v2 = pts[i + 1] - pts[i];
        // signed angle: positive = CCW turn (right-hand rule: +z out of screen)
        total += std::atan2(v1.x * v2.y - v1.y * v2.x,
                            v1.x * v2.x + v1.y * v2.y);
    }
    return total;
}

// Orient pts by shape correspondence with prev: pick whichever orientation
// (forward or reversed) minimises the sum of pointwise distances to prev.
// Both vectors must have the same length.
static void orientByShape(std::vector<cv::Point2f>& pts,
                          const std::vector<cv::Point2f>& prev)
{
    if (pts.size() < 2 || pts.size() != prev.size()) return;
    float fwdSum = 0.f, revSum = 0.f;
    int n = static_cast<int>(pts.size());
    for (int i = 0; i < n; ++i) {
        fwdSum += ptDist(pts[i], prev[i]);
        revSum += ptDist(pts[i], prev[n - 1 - i]);
    }
    if (revSum < fwdSum)
        std::reverse(pts.begin(), pts.end());
}

static cv::Point2f blobCentroid(const Tracking::DetectedBlob& blob)
{
    return cv::Point2f(static_cast<float>(blob.centroid.x()),
                       static_cast<float>(blob.centroid.y()));
}

static bool inconsistentWithPreviousFrame(const std::vector<cv::Point2f>& pts,
                                          float curLen,
                                          const CenterlineState& prev,
                                          const cv::Point2f& curBlobCentroid,
                                          float refLength)
{
    if (!prev.valid || pts.size() != prev.points.size() || pts.empty() ||
        refLength <= 0.f || curLen <= 0.f) {
        return false;
    }

    const cv::Point2f expectedOffset = curBlobCentroid - prev.blobCentroid;
    const float midpointShift =
        ptDist(pts[pts.size() / 2], prev.midpoint() + expectedOffset);

    float pointwiseShift = 0.f;
    for (size_t i = 0; i < pts.size(); ++i) {
        pointwiseShift += ptDist(pts[i], prev.points[i] + expectedOffset);
    }
    pointwiseShift /= static_cast<float>(pts.size());

    const float prevLen = arcLen(prev.points);
    const bool lengthShrank = prevLen > 0.f && curLen < 0.85f * prevLen;
    const float allowedShift = std::max(8.f, 0.25f * refLength);

    return lengthShrank &&
           (midpointShift > allowedShift || pointwiseShift > allowedShift);
}

static float continuityScore(std::vector<cv::Point2f>& pts,
                             const CenterlineState& prev,
                             const cv::Point2f& expectedOffset,
                             float refLength)
{
    if (!prev.valid || pts.size() != prev.points.size() || pts.empty()) {
        return std::numeric_limits<float>::max();
    }

    auto scoreFor = [&](const std::vector<cv::Point2f>& candidate) {
        float pointwise = 0.f;
        for (size_t i = 0; i < candidate.size(); ++i) {
            pointwise += ptDist(candidate[i], prev.points[i] + expectedOffset);
        }
        pointwise /= static_cast<float>(candidate.size());

        const float midpoint =
            ptDist(candidate[candidate.size() / 2], prev.midpoint() + expectedOffset);
        const float endpoints =
            ptDist(candidate.front(), prev.nose() + expectedOffset) +
            ptDist(candidate.back(), prev.tail() + expectedOffset);
        const float lengthPenalty = refLength > 0.f
            ? std::abs(arcLen(candidate) - refLength) * 0.25f
            : 0.f;

        return pointwise + 0.75f * midpoint + 0.35f * endpoints + lengthPenalty;
    };

    const float forwardScore = scoreFor(pts);
    std::vector<cv::Point2f> reversed = pts;
    std::reverse(reversed.begin(), reversed.end());
    const float reverseScore = scoreFor(reversed);
    if (reverseScore < forwardScore) {
        pts = std::move(reversed);
        return reverseScore;
    }
    return forwardScore;
}

static bool buildSplitRingCandidate(const Tracking::DetectedBlob& blob,
                                    const cv::Point2f& cutHint,
                                    int cutThickness,
                                    int nPoints,
                                    std::vector<cv::Point2f>& outPts,
                                    cv::Point2f& outCutPoint)
{
    if (blob.contourPoints.empty() || blob.holeContourPoints.empty()) {
        return false;
    }

    cv::Point2f holePoint;
    if (!nearestHolePoint(blob, cutHint, holePoint)) {
        return false;
    }

    const cv::Point2f outerPoint = nearestContourPoint(blob.contourPoints, cutHint);
    outCutPoint = outerPoint;
    Tracking::DetectedBlob splitBlob = blob;
    if (!Tracking::populateCenterlineFromContourWithCut(splitBlob, outerPoint, holePoint, cutThickness)) {
        return false;
    }

    outPts.assign(splitBlob.centerlinePoints.begin(), splitBlob.centerlinePoints.end());
    if (static_cast<int>(outPts.size()) != nPoints) {
        outPts = resample(outPts, nPoints);
    }
    return outPts.size() >= 2;
}

static bool buildSplitRingCandidateWithCut(const Tracking::DetectedBlob& blob,
                                           const cv::Point2f& cutStart,
                                           const cv::Point2f& cutEnd,
                                           int cutThickness,
                                           int nPoints,
                                           std::vector<cv::Point2f>& outPts,
                                           cv::Point2f& outCutPoint)
{
    Tracking::DetectedBlob splitBlob = blob;
    if (!Tracking::populateCenterlineFromContourWithCut(splitBlob, cutStart, cutEnd, cutThickness)) {
        return false;
    }

    outPts.assign(splitBlob.centerlinePoints.begin(), splitBlob.centerlinePoints.end());
    if (static_cast<int>(outPts.size()) != nPoints) {
        outPts = resample(outPts, nPoints);
    }
    outCutPoint = (cutStart + cutEnd) * 0.5f;
    return outPts.size() >= 2;
}

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

static bool diffBasedCutHint(const Tracking::DetectedBlob& current,
                             const CenterlineState& prev,
                             const cv::Point2f& expectedOffset,
                             const cv::Point2f& expectedNose,
                             const cv::Point2f& expectedTail,
                             cv::Point2f& outCutStart,
                             cv::Point2f& outCutEnd,
                             cv::Point2f& outCutCenter)
{
    if (!prev.valid || prev.blob.contourPoints.empty() || current.contourPoints.empty()) {
        return false;
    }

    cv::Rect currentBounds = cv::boundingRect(current.contourPoints);
    std::vector<cv::Point> shiftedPrevContour;
    shiftedPrevContour.reserve(prev.blob.contourPoints.size());
    for (const cv::Point& pt : prev.blob.contourPoints) {
        shiftedPrevContour.push_back(cv::Point(
            static_cast<int>(std::lround(static_cast<float>(pt.x) + expectedOffset.x)),
            static_cast<int>(std::lround(static_cast<float>(pt.y) + expectedOffset.y))));
    }
    cv::Rect prevBounds = cv::boundingRect(shiftedPrevContour);
    cv::Rect bounds = currentBounds | prevBounds;
    bounds.x -= 2;
    bounds.y -= 2;
    bounds.width += 4;
    bounds.height += 4;
    if (bounds.width <= 1 || bounds.height <= 1) {
        return false;
    }

    cv::Mat currentMask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    cv::Mat previousMask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    fillBlobMask(currentMask, current, bounds, cv::Point2f(0.f, 0.f));
    fillBlobMask(previousMask, prev.blob, bounds, expectedOffset);

    cv::Mat inversePrevious;
    cv::bitwise_not(previousMask, inversePrevious);
    cv::Mat delta;
    cv::bitwise_and(currentMask, inversePrevious, delta);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int componentCount =
        cv::connectedComponentsWithStats(delta, labels, stats, centroids, 8, CV_32S);
    if (componentCount <= 1) {
        return false;
    }

    const cv::Point2f expectedGapMid = (expectedNose + expectedTail) * 0.5f;
    int bestLabel = -1;
    float bestScore = std::numeric_limits<float>::max();
    for (int label = 1; label < componentCount; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < 2) {
            continue;
        }
        const cv::Point2f centroid(
            static_cast<float>(centroids.at<double>(label, 0) + bounds.x),
            static_cast<float>(centroids.at<double>(label, 1) + bounds.y));
        const float score = ptDist(centroid, expectedGapMid) - 0.15f * static_cast<float>(area);
        if (score < bestScore) {
            bestScore = score;
            bestLabel = label;
        }
    }
    if (bestLabel < 0) {
        return false;
    }

    cv::Point2f componentCentroid(
        static_cast<float>(centroids.at<double>(bestLabel, 0) + bounds.x),
        static_cast<float>(centroids.at<double>(bestLabel, 1) + bounds.y));
    const cv::Point2f nearestFeature =
        ptDist(componentCentroid, expectedNose) <= ptDist(componentCentroid, expectedTail)
            ? expectedNose
            : expectedTail;

    std::vector<cv::Point2f> componentPoints;
    componentPoints.reserve(static_cast<size_t>(stats.at<int>(bestLabel, cv::CC_STAT_AREA)));
    float farthestDistance = -1.f;
    for (int y = 0; y < labels.rows; ++y) {
        const int* labelRow = labels.ptr<int>(y);
        for (int x = 0; x < labels.cols; ++x) {
            if (labelRow[x] != bestLabel) {
                continue;
            }
            const cv::Point2f point(static_cast<float>(bounds.x + x),
                                    static_cast<float>(bounds.y + y));
            componentPoints.push_back(point);
            const float distance = ptDist(point, nearestFeature);
            if (distance > farthestDistance) {
                farthestDistance = distance;
            }
        }
    }

    if (componentPoints.empty() || farthestDistance <= 0.f) {
        return false;
    }

    std::vector<cv::Point2f> farEdgePoints;
    farEdgePoints.reserve(componentPoints.size());
    const float bandWidth = 2.5f;
    for (const cv::Point2f& point : componentPoints) {
        if (ptDist(point, nearestFeature) >= farthestDistance - bandWidth) {
            farEdgePoints.push_back(point);
        }
    }
    if (farEdgePoints.empty()) {
        return false;
    }

    cv::Point2f center(0.f, 0.f);
    for (const cv::Point2f& point : farEdgePoints) {
        center += point;
    }
    center *= 1.f / static_cast<float>(farEdgePoints.size());

    cv::Point2f direction(0.f, 0.f);
    if (farEdgePoints.size() >= 2) {
        double xx = 0.0;
        double xy = 0.0;
        double yy = 0.0;
        for (const cv::Point2f& point : farEdgePoints) {
            const double dx = static_cast<double>(point.x - center.x);
            const double dy = static_cast<double>(point.y - center.y);
            xx += dx * dx;
            xy += dx * dy;
            yy += dy * dy;
        }
        const double theta = 0.5 * std::atan2(2.0 * xy, xx - yy);
        direction = cv::Point2f(static_cast<float>(std::cos(theta)),
                                static_cast<float>(std::sin(theta)));
    }
    if (ptDist(direction, cv::Point2f(0.f, 0.f)) < 1e-3f) {
        const cv::Point2f away = center - nearestFeature;
        direction = cv::Point2f(-away.y, away.x);
    }
    const float directionLen = ptDist(direction, cv::Point2f(0.f, 0.f));
    if (directionLen < 1e-3f) {
        return false;
    }
    direction *= 1.f / directionLen;

    float minProjection = std::numeric_limits<float>::max();
    float maxProjection = -std::numeric_limits<float>::max();
    for (const cv::Point2f& point : farEdgePoints) {
        const cv::Point2f delta = point - center;
        const float projection = delta.x * direction.x + delta.y * direction.y;
        minProjection = std::min(minProjection, projection);
        maxProjection = std::max(maxProjection, projection);
    }

    const float halfLength = std::max(4.f, 0.5f * (maxProjection - minProjection) + 2.f);
    outCutStart = center - direction * halfLength;
    outCutEnd = center + direction * halfLength;
    outCutCenter = center;
    return true;
}

// ── Geodesic shortest path through a mask (Phase C.3) ──────────────────────
//
// Dijkstra on the 8-connected pixel graph of a CV_8UC1 mask: orthogonal edges
// cost 1, diagonal edges cost √2, edges leaving the mask are forbidden. Used
// to thread a polyline through a curved/coiled body shape from one tip to
// another, replacing the prev-centerline-as-snake-init that was poisoned by
// stale topology in self-crossed frames.

struct GeodesicResult {
    cv::Mat distMap;      // CV_32F, per-pixel geodesic distance from start; ∞ outside mask
    cv::Mat parentMap;    // CV_32SC1, parent flat index for path back-trace (-1 if unreached)
};

static bool runDijkstraInMask(const cv::Mat& mask,
                              const cv::Point& start,
                              GeodesicResult& result)
{
    if (mask.empty() || mask.type() != CV_8UC1) return false;
    const int rows = mask.rows, cols = mask.cols;
    if (start.x < 0 || start.y < 0 || start.x >= cols || start.y >= rows) return false;
    if (mask.at<uchar>(start) == 0) return false;

    result.distMap = cv::Mat(rows, cols, CV_32F,
                             cv::Scalar(std::numeric_limits<float>::infinity()));
    result.parentMap = cv::Mat(rows, cols, CV_32SC1, cv::Scalar(-1));
    result.distMap.at<float>(start) = 0.f;

    using Entry = std::pair<float, int>;  // (dist, flat idx)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
    pq.push({0.f, start.y * cols + start.x});

    static const float sqrt2 = std::sqrt(2.f);
    static const std::array<int, 8> dx = {-1,  0,  1, -1,  1, -1,  0,  1};
    static const std::array<int, 8> dy = {-1, -1, -1,  0,  0,  1,  1,  1};
    static const std::array<float, 8> ew = {sqrt2, 1.f, sqrt2, 1.f, 1.f, sqrt2, 1.f, sqrt2};

    while (!pq.empty()) {
        const auto [d, idx] = pq.top();
        pq.pop();
        const int cy = idx / cols, cx = idx % cols;
        if (d > result.distMap.at<float>(cy, cx)) continue;
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + dx[k], ny = cy + dy[k];
            if (nx < 0 || ny < 0 || nx >= cols || ny >= rows) continue;
            if (mask.at<uchar>(ny, nx) == 0) continue;
            const float nd = d + ew[k];
            if (nd < result.distMap.at<float>(ny, nx)) {
                result.distMap.at<float>(ny, nx) = nd;
                result.parentMap.at<int>(ny, nx) = idx;
                pq.push({nd, ny * cols + nx});
            }
        }
    }
    return true;
}

// Reconstruct an ordered start→goal path from a populated parent map.
// Returns false if goal is unreachable from start.
static bool reconstructPath(const GeodesicResult& g,
                            const cv::Point& start,
                            const cv::Point& goal,
                            std::vector<cv::Point>& outPath)
{
    outPath.clear();
    if (g.parentMap.empty()) return false;
    const int cols = g.parentMap.cols;
    if (!std::isfinite(g.distMap.at<float>(goal))) return false;

    std::vector<cv::Point> reverse;
    reverse.push_back(goal);
    int curIdx = goal.y * cols + goal.x;
    const int startIdx = start.y * cols + start.x;
    while (curIdx != startIdx) {
        const int parent = g.parentMap.at<int>(curIdx / cols, curIdx % cols);
        if (parent < 0) return false;  // disconnected
        curIdx = parent;
        reverse.emplace_back(curIdx % cols, curIdx / cols);
    }
    outPath.assign(reverse.rbegin(), reverse.rend());
    return outPath.size() >= 2;
}

// Convenience: one-shot start→goal geodesic.
static bool geodesicPathInMask(const cv::Mat& mask,
                               const cv::Point& start,
                               const cv::Point& goal,
                               std::vector<cv::Point>& outPath)
{
    GeodesicResult g;
    if (!runDijkstraInMask(mask, start, g)) return false;
    return reconstructPath(g, start, goal, outPath);
}

// ── Skeleton-graph shortest path (used by the new Clean centerline branch) ─
//
// Dijkstra on the pixel-graph of the (already-thinned) skeleton, returning
// the start→goal path as world-coords cv::Point2f. Edge weights are 1 for
// orthogonal, sqrt(2) for diagonal moves between skeleton pixels — same
// convention as Tracking::dijkstraSkeleton (kept in trackingcommon.cpp's
// anonymous namespace, so we re-implement here rather than expose it).
//
// `originOffset` is added to each result point so the caller doesn't have to
// translate from local mask coords to video coords separately.
static bool skeletonGraphPath(const Tracking::SkeletonGraph& graph,
                              int startIdx, int goalIdx,
                              const cv::Point2f& originOffset,
                              std::vector<cv::Point2f>& outPath)
{
    outPath.clear();
    if (startIdx < 0 || goalIdx < 0 ||
        startIdx >= static_cast<int>(graph.points.size()) ||
        goalIdx  >= static_cast<int>(graph.points.size())) return false;
    if (startIdx == goalIdx) return false;

    const int n = static_cast<int>(graph.points.size());
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int>    parent(n, -1);
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
        for (int neigh : graph.adjacency[cur]) {
            const cv::Point& np = graph.points[neigh];
            const int dx = np.x - cp.x, dy = np.y - cp.y;
            const double w = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            const double nd = d + w;
            if (nd < dist[neigh]) {
                dist[neigh] = nd;
                parent[neigh] = cur;
                pq.push({nd, neigh});
            }
        }
    }

    if (!std::isfinite(dist[goalIdx])) return false;

    std::vector<cv::Point2f> reverse;
    int cur = goalIdx;
    while (cur != -1) {
        const cv::Point& p = graph.points[cur];
        reverse.emplace_back(static_cast<float>(p.x) + originOffset.x,
                             static_cast<float>(p.y) + originOffset.y);
        if (cur == startIdx) break;
        cur = parent[cur];
    }
    if (cur == -1) return false;

    outPath.assign(reverse.rbegin(), reverse.rend());
    return outPath.size() >= 2;
}

// ── Skeleton-arc helpers for SelfCrossed dispatch (D-2 / D-3) ──────────────
//
// For a ring or coiled worm the shortest skeleton path goes the wrong way —
// it takes the short arc across the ring rather than tracing the body axis
// all the way around. The three helpers below let the caller enumerate BOTH
// arcs of the skeleton between a source and a goal node, then pick the arc
// whose total signed turning angle matches the previous frame's (right-hand
// rule), falling back to arc-length proximity to refLength when no previous
// turning state is available.

// Nearest skeleton graph node (by squared Euclidean distance) to a world point.
static int nearestSkeletonNode(const Tracking::SkeletonGraph& graph,
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
static int farthestSkeletonNode(const Tracking::SkeletonGraph& graph, int srcIdx)
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
static bool skeletonBothArcs(const Tracking::SkeletonGraph& graph,
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
// Primary selector: the arc whose totalSignedTurningAngle matches the sign
//   of prevTurningAngle (right-hand rule from the previous frame).
// Fallback when prev angle is below the threshold or both/neither match:
//   pick the arc whose arcLen is closer to refLength.
// Last resort: return arcA.
static std::vector<cv::Point2f> pickArcByRHR(
    const std::vector<cv::Point2f>& arcA,
    const std::vector<cv::Point2f>& arcB,
    float prevTurningAngle,
    float angleThreshold,
    float refLength)
{
    const float turA = totalSignedTurningAngle(arcA);
    const float turB = totalSignedTurningAngle(arcB);

    if (std::abs(prevTurningAngle) > angleThreshold) {
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
// Initialization is the previous-frame centerline translated by centroid motion;
// endpoints are pinned to the nearest outer-contour point of the predicted nose/tail
// (worm endpoints must lie on the boundary). When the U-shaped prior should curl
// into a self-crossing curve, gradient descent finds that minimum naturally.
//
// Returns false on any structural failure; outPts is populated with nPoints
// resampled points on success, and outOverlapCenter / outHasOverlap describe
// any self-intersection detected in the result (used for debug overlay).
// Snake core: takes a pre-built mask + an explicit init polyline + explicit
// pinned head/tail positions. Both the legacy prev-centerline path and the
// new geodesic-from-tips path (Phase C.3 / D-2, D-3) wrap this function so
// the snake math (DT, gradient, Euler loop, overlap detection) is single-
// sourced.
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
                            const Tracking::CenterlineSnakeParams& params,
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

// Legacy wrapper: prev-centerline-based init, used by Pass 2's ring branch.
static bool refineCenterlineSnake(const Tracking::DetectedBlob& blob,
                                  const CenterlineState& prev,
                                  float /*refLength*/,
                                  int nPoints,
                                  const Tracking::CenterlineSnakeParams& params,
                                  std::vector<cv::Point2f>& outPts,
                                  cv::Point2f& outOverlapCenter,
                                  bool& outHasOverlap)
{
    outHasOverlap = false;
    if (!params.enabled || !prev.valid || prev.points.size() < 4 ||
        blob.contourPoints.empty()) return false;

    // Init: prev centerline translated by centroid motion.
    const cv::Point2f curCentroid = blobCentroid(blob);
    const cv::Point2f offset = curCentroid - prev.blobCentroid;
    std::vector<cv::Point2f> v;
    v.reserve(prev.points.size());
    for (const cv::Point2f& p : prev.points) v.push_back(p + offset);

    cv::Mat mask;
    cv::Rect bounds;
    if (!buildSnakeMask(blob, mask, bounds)) return false;

    // Pin endpoints to nearest contour of inherited endpoints.
    const cv::Point2f pinHead = nearestContourPoint(blob.contourPoints, v.front());
    const cv::Point2f pinTail = nearestContourPoint(blob.contourPoints, v.back());

    if (!refineSnakeCore(blob, mask, bounds, v, pinHead, pinTail,
                         nPoints, params, outOverlapCenter, outHasOverlap)) {
        return false;
    }
    outPts = std::move(v);
    return true;
}

// New (Phase C.3 / D-2): snake with init derived from a mask-constrained
// geodesic between the assigned head and tail. Endpoints are pinned exactly
// to the supplied head/tail positions (no nearest-contour snap — they're
// already on the outer contour by definition of how tipCandidates were
// produced). Used when the worm is in SelfCrossed topology and both tips
// have been successfully assigned in Pass 4.
static bool refineCenterlineSnakeFromTips(const Tracking::DetectedBlob& blob,
                                          const cv::Point2f& head,
                                          const cv::Point2f& tail,
                                          int nPoints,
                                          const Tracking::CenterlineSnakeParams& params,
                                          std::vector<cv::Point2f>& outPts,
                                          cv::Point2f& outOverlapCenter,
                                          bool& outHasOverlap)
{
    outHasOverlap = false;
    if (!params.enabled || blob.contourPoints.empty()) return false;

    cv::Mat mask;
    cv::Rect bounds;
    if (!buildSnakeMask(blob, mask, bounds)) return false;

    // Convert head/tail to mask coords.
    const cv::Point headLocal(std::clamp(static_cast<int>(std::lround(head.x - bounds.x)), 0, mask.cols - 1),
                              std::clamp(static_cast<int>(std::lround(head.y - bounds.y)), 0, mask.rows - 1));
    const cv::Point tailLocal(std::clamp(static_cast<int>(std::lround(tail.x - bounds.x)), 0, mask.cols - 1),
                              std::clamp(static_cast<int>(std::lround(tail.y - bounds.y)), 0, mask.rows - 1));

    // Geodesic from head to tail through the mask.
    std::vector<cv::Point> pathLocal;
    if (!geodesicPathInMask(mask, headLocal, tailLocal, pathLocal)) return false;
    if (pathLocal.size() < 2) return false;

    // Convert to video coords for the snake.
    std::vector<cv::Point2f> v;
    v.reserve(pathLocal.size());
    for (const cv::Point& p : pathLocal) {
        v.emplace_back(static_cast<float>(p.x + bounds.x),
                       static_cast<float>(p.y + bounds.y));
    }

    if (!refineSnakeCore(blob, mask, bounds, v, head, tail,
                         nPoints, params, outOverlapCenter, outHasOverlap)) {
        return false;
    }
    outPts = std::move(v);
    return true;
}

// New (Phase C.3 / D-3): only one tip is assigned. Hypothesize where the
// hidden tip lies by finding the mask pixel that is geodesically farthest
// from the known tip (double-Dijkstra, same principle as extractCenterline-
// FromMask's fallback). This is the natural terminus of the body axis through
// the mask and is far more reliable than projecting to the outer contour at
// a body-length target distance — the outer contour of a coiled/self-crossing
// worm traces the outer boundary of the whole mass, not the hidden tip.
//
// Returns the hypothesized other tip via outHiddenTip on success.
static bool refineCenterlineSnakeFromOneTip(const Tracking::DetectedBlob& blob,
                                            const cv::Point2f& knownTip,
                                            int nPoints,
                                            const Tracking::CenterlineSnakeParams& params,
                                            std::vector<cv::Point2f>& outPts,
                                            cv::Point2f& outHiddenTip,
                                            cv::Point2f& outOverlapCenter,
                                            bool& outHasOverlap)
{
    outHasOverlap = false;
    if (!params.enabled || blob.contourPoints.empty()) return false;

    cv::Mat mask;
    cv::Rect bounds;
    if (!buildSnakeMask(blob, mask, bounds)) return false;

    const cv::Point knownLocal(std::clamp(static_cast<int>(std::lround(knownTip.x - bounds.x)), 0, mask.cols - 1),
                               std::clamp(static_cast<int>(std::lround(knownTip.y - bounds.y)), 0, mask.rows - 1));

    GeodesicResult g;
    if (!runDijkstraInMask(mask, knownLocal, g)) return false;

    // Find the mask pixel that is geodesically farthest from the known tip.
    // This is the natural far end of the body axis — the same double-Dijkstra
    // principle used in extractCenterlineFromMask when skeleton endpoints are
    // absent. Scanning all mask pixels (not just the outer contour) means we
    // correctly reach interior points on a self-crossing body.
    cv::Point bestLocal = knownLocal;
    float bestDist = 0.f;
    for (int y = 0; y < mask.rows; ++y) {
        const float* distRow = g.distMap.ptr<float>(y);
        const uchar* maskRow = mask.ptr<uchar>(y);
        for (int x = 0; x < mask.cols; ++x) {
            if (maskRow[x] == 0) continue;
            if (std::isfinite(distRow[x]) && distRow[x] > bestDist) {
                bestDist = distRow[x];
                bestLocal = cv::Point(x, y);
            }
        }
    }
    if (bestDist < 1.f) return false;  // degenerate: all points at distance 0

    std::vector<cv::Point> pathLocal;
    if (!reconstructPath(g, knownLocal, bestLocal, pathLocal)) return false;
    if (pathLocal.size() < 2) return false;

    const cv::Point2f hiddenTip(static_cast<float>(bestLocal.x + bounds.x),
                                static_cast<float>(bestLocal.y + bounds.y));

    std::vector<cv::Point2f> v;
    v.reserve(pathLocal.size());
    for (const cv::Point& p : pathLocal) {
        v.emplace_back(static_cast<float>(p.x + bounds.x),
                       static_cast<float>(p.y + bounds.y));
    }

    if (!refineSnakeCore(blob, mask, bounds, v, knownTip, hiddenTip,
                         nPoints, params, outOverlapCenter, outHasOverlap)) {
        return false;
    }
    outPts = std::move(v);
    outHiddenTip = hiddenTip;
    return true;
}

// Compute, orient, validate and (if needed) repair the centerline for one frame.
// Returns the resulting CenterlineState (always valid if the function returns
// true; invalid if no usable centerline could be produced).
static bool processOneFrame(Tracking::DetectedBlob& blob,
                            const CenterlineState& prev,
                            float refLength,
                            int nPoints,
                            float minArcFraction,
                            const Tracking::CenterlineSnakeParams& snakeParams,
                            CenterlineState& out)
{
    if (!blob.isValid || blob.contourPoints.empty()) return false;
    blob.hasCenterlineCutPoint = false;

    // Phase-B preprocessing: surface candidate nose/tail points on this blob
    // independently of any temporal state. Cheap (skeleton + curvature scan +
    // distance transform), and the results travel with the blob into storage
    // for the debug overlay to consume. Downstream head/tail assignment will
    // read these candidates instead of inheriting endpoints from the previous
    // frame's centerline.
    Tracking::findTipCandidates(blob);

    if (blob.centerlinePoints.empty())
        Tracking::populateCenterlineFromContour(blob);
    if (blob.centerlinePoints.empty()) return false;

    std::vector<cv::Point2f> pts(blob.centerlinePoints.begin(),
                                 blob.centerlinePoints.end());

    // Always resample to a consistent point count so shape-correspondence
    // comparisons are meaningful.
    if (static_cast<int>(pts.size()) != nPoints)
        pts = resample(pts, nPoints);

    // Orient by shape correspondence with the previous frame.  This is more
    // robust than nose-distance alone because nose and tail can be spatially
    // close on a coiled worm; the *order* along the polyline still
    // distinguishes them when matched against the previous frame's ordering.
    if (prev.valid && prev.points.size() == pts.size())
        orientByShape(pts, prev.points);

    bool isRing  = !blob.holeContourPoints.empty();
    float curLen = arcLen(pts);
    bool tooShort = (refLength > 0.f && curLen < minArcFraction * refLength);
    const cv::Point2f curBlobCentroid = blobCentroid(blob);
    const bool inconsistent =
        inconsistentWithPreviousFrame(pts, curLen, prev, curBlobCentroid, refLength);
    const bool needsRepair = tooShort || inconsistent;

    // ─── Fallback A: ring blobs ──────────────────────────────────────────
    // A ring is a closed worm mask, not a fundamentally different body shape.
    // Cut the ring open near the predicted closure seam, skeletonize the split
    // mask, and choose the candidate that best preserves frame-to-frame shape.
    if (isRing && prev.valid) {
        const cv::Point2f expectedOffset = prev.valid
            ? (curBlobCentroid - prev.blobCentroid)
            : cv::Point2f(0.f, 0.f);
        const cv::Point2f expectedNose = prev.nose() + expectedOffset;
        const cv::Point2f expectedTail = prev.tail() + expectedOffset;
        const cv::Point2f expectedMid = prev.midpoint() + expectedOffset;
        const cv::Point2f expectedEndGapMid = (expectedNose + expectedTail) * 0.5f;

        struct CutCandidateSeed {
            cv::Point2f start;
            cv::Point2f end;
            cv::Point2f hint;
            bool hasSegment = false;
        };

        std::vector<CutCandidateSeed> cutSeeds;
        cv::Point2f diffCutStart;
        cv::Point2f diffCutEnd;
        cv::Point2f diffCutCenter;
        if (diffBasedCutHint(blob, prev, expectedOffset, expectedNose, expectedTail,
                             diffCutStart, diffCutEnd, diffCutCenter)) {
            cutSeeds.push_back(CutCandidateSeed{diffCutStart, diffCutEnd, diffCutCenter, true});
        }
        std::vector<cv::Point2f> cutHints;
        cutHints.push_back(expectedEndGapMid);
        cutHints.push_back(expectedNose);
        cutHints.push_back(expectedTail);
        cutHints.push_back(expectedMid);
        cutHints.push_back(expectedNose * 0.75f + expectedTail * 0.25f);
        cutHints.push_back(expectedNose * 0.25f + expectedTail * 0.75f);
        for (const cv::Point2f& hint : cutHints) {
            cutSeeds.push_back(CutCandidateSeed{cv::Point2f(), cv::Point2f(), hint, false});
        }

        std::vector<cv::Point2f> currentForScore = pts;
        const float currentScore =
            continuityScore(currentForScore, prev, expectedOffset, refLength);
        float bestScore = std::numeric_limits<float>::max();
        std::vector<cv::Point2f> bestPts;
        cv::Point2f bestCutPoint;
        bool bestIsSnake = false;
        const int cutThickness = 3;

        // Snake candidate first: parametric polyline refined toward the medial axis,
        // initialized from the prev centerline. Unlike skeleton-with-cut, the snake
        // is allowed to self-intersect — required when the body crosses over itself.
        if (snakeParams.enabled) {
            std::vector<cv::Point2f> snakePts;
            cv::Point2f snakeOverlapCenter;
            bool snakeHasOverlap = false;
            if (refineCenterlineSnake(blob, prev, refLength, nPoints,
                                      snakeParams, snakePts,
                                      snakeOverlapCenter, snakeHasOverlap)) {
                const float snakeScore =
                    continuityScore(snakePts, prev, expectedOffset, refLength);
                if (snakeScore < bestScore) {
                    bestScore = snakeScore;
                    bestPts = std::move(snakePts);
                    // For the snake, the "cut point" is repurposed as the
                    // self-crossing location for the debug overlay (or the
                    // closure-region centroid as a fallback when no crossing
                    // was detected this frame).
                    if (snakeHasOverlap) {
                        bestCutPoint = snakeOverlapCenter;
                    } else {
                        bestCutPoint = (expectedNose + expectedTail) * 0.5f;
                    }
                    bestIsSnake = true;
                }
            }
        }

        // Legacy cut-skeleton candidates kept as a fallback / A-B comparison
        // partner. Use whichever continuity score is lower.
        for (const CutCandidateSeed& seed : cutSeeds) {
            std::vector<cv::Point2f> candidate;
            cv::Point2f cutPoint;
            const bool built = seed.hasSegment
                ? buildSplitRingCandidateWithCut(blob, seed.start, seed.end, cutThickness,
                                                 nPoints, candidate, cutPoint)
                : buildSplitRingCandidate(blob, seed.hint, cutThickness,
                                          nPoints, candidate, cutPoint);
            if (!built) {
                continue;
            }
            const float score =
                continuityScore(candidate, prev, expectedOffset, refLength);
            if (score < bestScore) {
                bestScore = score;
                bestPts = std::move(candidate);
                bestCutPoint = cutPoint;
                bestIsSnake = false;
            }
        }

        // The snake doesn't need the "needsRepair" gate the cut-skeleton path
        // uses, because its initialization IS the prev centerline — it can
        // only refine, not regress. Always accept a snake winner; for cut
        // candidates fall back to the original conservative gate.
        const bool acceptBest = !bestPts.empty() &&
            (bestIsSnake || needsRepair || bestScore + 2.f < currentScore);
        if (acceptBest) {
            pts = std::move(bestPts);
            blob.centerlineCutPoint = bestCutPoint;
            blob.hasCenterlineCutPoint = true;
        }
    }
    // ─── Fallback B: non-ring blobs ──────────────────────────────────────
    // Walking the outer contour wraps the full perimeter (~2× body length),
    // so it would follow one body *edge* rather than the centerline — worse
    // than the original skeleton.  Instead, when the skeleton is short and
    // a previous frame is available, treat the previous centerline as a
    // shape template: translate it so its centroid aligns with the current
    // blob's centroid, then snap any point that lands outside the blob to
    // the nearest outer contour point.  Worms barely move between frames,
    // so the previous shape is a good estimate of the missing portion.
    else if (!isRing && needsRepair && prev.valid &&
             prev.points.size() == pts.size() &&
             !blob.contourPoints.empty()) {

        // Centroid of the previous (resampled, ordered) centerline points.
        cv::Point2f prevCentroid(0.f, 0.f);
        for (const cv::Point2f& p : prev.points) prevCentroid += p;
        prevCentroid *= 1.f / static_cast<float>(prev.points.size());

        // Centroid of the current blob (mean of contour pixels — robust to
        // skeleton failure modes since it depends only on the boundary).
        cv::Point2f curCentroid(0.f, 0.f);
        for (const cv::Point& p : blob.contourPoints)
            curCentroid += cv::Point2f(p.x, p.y);
        curCentroid *= 1.f / static_cast<float>(blob.contourPoints.size());

        cv::Point2f offset = curCentroid - prevCentroid;

        std::vector<cv::Point2f> templatePts(prev.points.size());
        for (size_t i = 0; i < prev.points.size(); ++i)
            templatePts[i] = prev.points[i] + offset;

        // Snap out-of-blob points back onto the contour.  pointPolygonTest
        // returns >0 inside, ==0 on edge, <0 outside.
        for (cv::Point2f& p : templatePts) {
            if (cv::pointPolygonTest(blob.contourPoints, p, false) < 0) {
                int idx = nearestContourIdx(blob.contourPoints, p);
                p = cv::Point2f(blob.contourPoints[idx].x,
                                blob.contourPoints[idx].y);
            }
        }

        if (inconsistent || arcLen(templatePts) > curLen) {
            pts = templatePts;  // already ordered, already nPoints long
        }
    }

    if (pts.size() < 2) return false;

    // ─── Right-hand-rule orientation veto ───────────────────────────────
    // orientByShape and continuityScore both choose the orientation that
    // minimises pointwise distance to prev.  That works well for small
    // inter-frame motion but can be fooled when the worm is coiled and the
    // score difference between forward and reversed is small.
    //
    // The total signed turning angle (∫κ ds) of the polyline is a stronger
    // global signal: it equals the net tangent rotation nose→tail, and it
    // changes sign when the traversal direction is reversed.  Worms don't
    // instantaneously flip their coiling direction, so the sign of this
    // angle should be consistent between adjacent frames once it becomes
    // large enough to be reliable (> threshold).
    //
    // Near-straight frames have |angle| ≈ 0 and are genuinely ambiguous in
    // orientation; the check is skipped there and orientByShape's decision
    // stands.
    const float curTurningAngle = totalSignedTurningAngle(pts);
    const float angleThreshold  = static_cast<float>(
        std::max(0.0, snakeParams.orientationAngleThreshold));
    bool vetoFlipped = false;
    if (prev.valid
        && std::abs(prev.turningAngle) > angleThreshold
        && std::abs(curTurningAngle)   > angleThreshold
        && curTurningAngle * prev.turningAngle < 0.f) {
        std::reverse(pts.begin(), pts.end());
        vetoFlipped = true;
    }

    blob.centerlinePoints.assign(pts.begin(), pts.end());
    out.points       = pts;
    out.blobCentroid = curBlobCentroid;
    out.blob         = blob;
    out.valid        = true;
    // Store the angle after any veto flip (flip negates it).
    out.turningAngle = vetoFlipped ? -curTurningAngle : curTurningAngle;
    return true;
}

// ── CenterlineWorker ────────────────────────────────────────────────────────

// Default point count — overridden at runtime by CenterlineSnakeParams::nPoints.
// More points reduce kinking artefacts on highly curved bodies; fewer are faster.
static constexpr int   kCenterlinePointsDefault = 20;
static constexpr float kMinArcLengthFraction    = 0.5f;

CenterlineWorker::CenterlineWorker(TrackingDataStorage* storage, QObject* parent)
    : QObject(parent), m_storage(storage) {}

void CenterlineWorker::setSnakeParams(const Tracking::CenterlineSnakeParams& params)
{
    m_snakeParams = params;
}

// Compile-time switch between the legacy 5-pass pipeline and the new
// 2-sweep / 5-step pipeline. Flip to `true` once the new pipeline has been
// visually validated (green dots present at expected tip locations on a
// known clip; centerlines tracking the body axis).
static constexpr bool kUseNewPipeline = true;

void CenterlineWorker::doWork()
{
    if (kUseNewPipeline) {
        doWorkNew();
    } else {
        doWorkLegacy();
    }
}

// New 2-sweep / 5-step pipeline. See CENTERLINE_REWRITE_PLAN.md for the
// full design. Structure:
//
//   Sweep 0 — read-only walk over all non-merged, non-lost frames; build a
//             throwaway skeleton centerline per frame; collect arc lengths;
//             refLength = median.
//
//   Sweep 1 — keyframe-outward bidirectional per-frame loop. Each frame:
//             Step 1: detectEndpoints() → tip data + topology + assignment.
//                     Write back to blob.tipCandidates (source = SkeletonEndpoint),
//                     blob.assignedHeadTipIdx, blob.assignedTailTipIdx,
//                     blob.topologyState. On Clean frames, sample baseline.
//             Step 2: build centerline.
//                       Clean    → skeleton-graph Dijkstra head→tail.
//                       Ring     → synthetic-hole punch + re-skeletonize.
//                       SC, 2tp  → D-2 (refineCenterlineSnakeFromTips).
//                       SC, 1tp  → D-3 (refineCenterlineSnakeFromOneTip)
//                                  + append HypothesizedHidden tip candidate.
//                       fallback → populateCenterlineFromContour (D-4).
//             Step 3: resample to nPoints.
//             Step 4: snake refinement (Clean only); right-hand-rule veto.
//             Step 5: predictor update for next frame.
void CenterlineWorker::doWorkNew()
{
    if (!m_storage) {
        emit failed("No storage provided to CenterlineWorker");
        return;
    }

    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    const int totalWorms = static_cast<int>(tracks.size());
    if (totalWorms == 0) {
        emit progress(100);
        emit finished();
        return;
    }

    // A rerun reads the same clean frames, so re-accumulating into existing
    // baseline counts would inflate the sample count without new info.
    m_storage->clearAllTipBaselines();

    const int nPts = std::max(4, m_snakeParams.nPoints);
    int processedWorms = 0;

    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        const int wormId = it->first;

        std::vector<Tracking::WormTrackPoint> sortedPoints = it->second;
        std::sort(sortedPoints.begin(), sortedPoints.end(),
                  [](const Tracking::WormTrackPoint& a,
                     const Tracking::WormTrackPoint& b) {
                      return a.frameNumberOriginal < b.frameNumberOriginal;
                  });

        // Per-worm keyframe (the user-clicked frame).
        int keyframe = -1;
        if (const TableItems::ClickedItem* item = m_storage->getItem(wormId))
            keyframe = item->frameOfSelection;

        // ── Sweep 0 — body length learning ──────────────────────────────
        // Read-only: skeleton on a TEMPORARY blob copy so storage stays
        // untouched. Only non-ring, non-merged, non-lost frames contribute.
        std::vector<float> validLengths;
        validLengths.reserve(sortedPoints.size());
        for (const Tracking::WormTrackPoint& tp : sortedPoints) {
            if (tp.quality == Tracking::TrackPointQuality::Merged ||
                tp.quality == Tracking::TrackPointQuality::Lost) continue;
            QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (!frameBlobs.contains(wormId)) continue;
            Tracking::DetectedBlob temp = frameBlobs[wormId];
            if (!temp.isValid || temp.contourPoints.empty()) continue;
            if (!temp.holeContourPoints.empty()) continue;
            if (Tracking::populateCenterlineFromContour(temp) &&
                temp.centerlinePoints.size() >= 2) {
                std::vector<cv::Point2f> p(temp.centerlinePoints.begin(),
                                           temp.centerlinePoints.end());
                validLengths.push_back(arcLen(p));
            }
        }

        float refLength = 0.f;
        if (!validLengths.empty()) {
            std::nth_element(validLengths.begin(),
                             validLengths.begin() + validLengths.size() / 2,
                             validLengths.end());
            refLength = validLengths[validLengths.size() / 2];
        }

        // Find keyframe index. Fall back to frame 0 if the click frame is
        // missing from the track (rare; happens on retracking).
        int keyframeIdx = -1;
        if (keyframe >= 0) {
            for (size_t i = 0; i < sortedPoints.size(); ++i) {
                if (sortedPoints[i].frameNumberOriginal == keyframe) {
                    keyframeIdx = static_cast<int>(i);
                    break;
                }
            }
        }
        if (keyframeIdx < 0) keyframeIdx = 0;

        // ── helpers used inside the per-frame lambda ────────────────────
        auto inMergeGroupAtFrame = [&](int frameNumber) -> bool {
            const QList<QList<int>> groups =
                m_storage->getMergeGroupsForFrame(frameNumber);
            for (const QList<int>& g : groups) {
                if (g.size() > 1 && g.contains(wormId)) return true;
            }
            return false;
        };

        // Map a world point to the index of the closest tip candidate on a
        // blob — used by the keyframe seed to bootstrap head/tail from the
        // centerline's natural orientation.
        auto nearestCandidateIdx = [](const Tracking::DetectedBlob& b,
                                      const cv::Point2f& target) -> int {
            int bestIdx = -1;
            float bestDistSq = std::numeric_limits<float>::max();
            for (size_t i = 0; i < b.tipCandidates.size(); ++i) {
                const cv::Point2f d = b.tipCandidates[i].point - target;
                const float dsq = d.x * d.x + d.y * d.y;
                if (dsq < bestDistSq) { bestDistSq = dsq; bestIdx = static_cast<int>(i); }
            }
            return bestIdx;
        };

        // ── per-frame body of Sweep 1 ────────────────────────────────────
        // `predictor` and `prevState` are CARRIED across frames within one
        // direction; they reset on Lost frames and at the keyframe boundary
        // between forward and backward passes.
        auto processFrame = [&](int i,
                                Tracking::HeadTailPredictor& predictor,
                                CenterlineState& prevState,
                                bool isKeyframeBootstrap) -> void {
            if (i < 0 || i >= static_cast<int>(sortedPoints.size())) return;
            const Tracking::WormTrackPoint& tp = sortedPoints[i];

            if (tp.quality == Tracking::TrackPointQuality::Lost) {
                predictor = Tracking::HeadTailPredictor{};
                prevState.valid = false;
                return;
            }

            QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (!frameBlobs.contains(wormId)) return;
            Tracking::DetectedBlob blob = frameBlobs[wormId];
            if (!blob.isValid || blob.contourPoints.empty()) return;
            blob.hasCenterlineCutPoint = false;

            const bool inMerge = inMergeGroupAtFrame(tp.frameNumberOriginal);

            // ── STEP 1: detect endpoints, write back tip data ───────────
            const Tracking::TipFeatureBaseline baseline =
                m_storage->getTipBaseline(wormId);
            Tracking::EndpointResult er = Tracking::detectEndpoints(
                blob, predictor, baseline, inMerge);

            // Convert TrueTips → TipCandidates. Source = SkeletonEndpoint
            // for ALL tips so the renderer's filled-green-dot styling
            // applies regardless of whether the position came from the
            // skeleton-snap or the curvature-peak extension.
            blob.tipCandidates.clear();
            for (const Tracking::TrueTip& t : er.tips) {
                Tracking::TipCandidate tc;
                tc.point     = t.point;
                tc.curvature = t.curvature;
                tc.width     = t.width;
                tc.source    = Tracking::TipCandidate::Source::SkeletonEndpoint;
                blob.tipCandidates.push_back(tc);
            }
            blob.assignedHeadTipIdx = er.headIdx;
            blob.assignedTailTipIdx = er.tailIdx;
            blob.topologyState      = er.topology;

            // Sample baseline (curvature + width) on Clean frames with two
            // tips. Body-length sample is taken AFTER step 3 below using
            // the resampled centerline.
            const bool cleanWithTwoTips =
                er.topology == Tracking::TopologyState::Clean &&
                er.tips.size() == 2;
            if (cleanWithTwoTips) {
                for (const Tracking::TrueTip& t : er.tips) {
                    m_storage->recordTipFeatureSample(
                        wormId, std::abs(t.curvature), t.width);
                }
            }

            // Keyframe bootstrap: detectEndpoints leaves head/tail at -1
            // because predictor.hasPrev is false on the first call. With
            // ≥2 tips we arbitrarily pick (0, 1) so Step 2's Clean branch
            // can build a centerline; head/tail will be re-derived from
            // the centerline orientation after Step 3.
            if (isKeyframeBootstrap && !er.tips.empty()) {
                if (er.tips.size() >= 2 && blob.assignedHeadTipIdx < 0) {
                    blob.assignedHeadTipIdx = 0;
                    blob.assignedTailTipIdx = 1;
                } else if (blob.assignedHeadTipIdx < 0 &&
                           blob.assignedTailTipIdx < 0) {
                    blob.assignedHeadTipIdx = 0;
                }
            }

            // ── STEP 2: build centerline based on topology ──────────────
            std::vector<cv::Point2f> centerline;
            cv::Point2f overlapCenter(0.f, 0.f);
            bool hasOverlap = false;
            const cv::Point2f originOffset(static_cast<float>(er.localBounds.x),
                                           static_cast<float>(er.localBounds.y));
            // angleThreshold used by both RHR arc-pick (Step 2) and RHR veto (Step 4).
            const float angleThreshold = static_cast<float>(
                std::max(0.0, m_snakeParams.orientationAngleThreshold));

            // Shared lambda: skeleton-arc dispatch for SelfCrossed frames.
            // Finds both arcs of the skeleton from the known tip to the
            // target (actual or predicted), picks by RHR then length, and
            // registers the arc's far terminus as the hypothesised tip when
            // there is no actual target.
            auto runSkeletonArcDispatch =
                [&](const Tracking::DetectedBlob& dispBlob,
                    const Tracking::EndpointResult& dispEr,
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
                if (hasActualTarget) {
                    targetPos = dispBlob.tipCandidates[hasHead ? tIdx : hIdx].point;
                } else if (predictor.hasPrev) {
                    const bool hiddenIsHead = !hasHead;
                    const cv::Point2f& last = hiddenIsHead
                        ? predictor.lastHeadPos : predictor.lastTailPos;
                    const cv::Point2f& vel  = hiddenIsHead
                        ? predictor.velHead    : predictor.velTail;
                    if (last.x != 0.f || last.y != 0.f)
                        targetPos = predictor.hasVelocity ? (last + vel) : last;
                }

                int goalNode = -1;
                if (targetPos.x != -1.f || targetPos.y != -1.f) {
                    goalNode = nearestSkeletonNode(dispEr.skeleton, targetPos, dispOrigin);
                } else {
                    goalNode = farthestSkeletonNode(dispEr.skeleton, srcNode);
                }
                if (goalNode < 0 || goalNode == srcNode) return false;

                // Enumerate both arcs; pick via RHR then length.
                std::vector<cv::Point2f> arcA, arcB;
                std::vector<cv::Point2f> chosen;
                if (skeletonBothArcs(dispEr.skeleton, srcNode, goalNode,
                                     dispOrigin, arcA, arcB)) {
                    chosen = pickArcByRHR(arcA, arcB,
                                          prevState.turningAngle,
                                          angleThreshold, refLength);
                } else if (!arcA.empty()) {
                    chosen = arcA; // open-curve skeleton: only one arc
                } else {
                    return false;
                }
                if (chosen.size() < 2) return false;

                // Snap endpoints to the actual tip positions if curvature-extended.
                if (hasHead && dispEr.tips[hIdx].extended)
                    chosen.front() = dispEr.tips[hIdx].point;
                if (hasTail && dispEr.tips[tIdx].extended)
                    chosen.back()  = dispEr.tips[tIdx].point;

                centerline = std::move(chosen);

                // D-3: register the arc terminus as the hypothesised hidden tip.
                if (!hasActualTarget) {
                    Tracking::TipCandidate hyp;
                    hyp.point  = centerline.back();
                    hyp.source = Tracking::TipCandidate::Source::HypothesizedHidden;
                    blob.tipCandidates.push_back(hyp);
                    const int newIdx = static_cast<int>(blob.tipCandidates.size()) - 1;
                    if (hasHead) blob.assignedTailTipIdx = newIdx;
                    else         blob.assignedHeadTipIdx = newIdx;
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
                    if (er.tips[blob.assignedHeadTipIdx].extended &&
                        !centerline.empty())
                        centerline.front() = er.tips[blob.assignedHeadTipIdx].point;
                    if (er.tips[blob.assignedTailTipIdx].extended &&
                        !centerline.empty())
                        centerline.back() = er.tips[blob.assignedTailTipIdx].point;

                    // If D-1 result is suspiciously short the worm is
                    // tightly self-coiled but had no visible hole in the
                    // mask. Punch a synthetic hole at the DT maximum and
                    // re-run the SelfCrossed skeleton-arc dispatch.
                    if (refLength > 0.f &&
                        arcLen(centerline) < 0.5f * refLength) {
                        Tracking::DetectedBlob holeBlob;
                        if (addSyntheticHoleAtDTMax(blob, er.distTransform,
                                                     er.localBounds, holeBlob)) {
                            const Tracking::EndpointResult er2 =
                                Tracking::detectEndpoints(
                                    holeBlob, predictor, baseline, inMerge);
                            const cv::Point2f origin2(
                                static_cast<float>(er2.localBounds.x),
                                static_cast<float>(er2.localBounds.y));
                            // Temporarily swap blob state so the lambda
                            // writes into the right place.
                            std::vector<cv::Point2f> prevCl = centerline;
                            Tracking::DetectedBlob savedBlob = blob;
                            blob = holeBlob;
                            blob.tipCandidates.clear();
                            for (const Tracking::TrueTip& t : er2.tips) {
                                Tracking::TipCandidate tc;
                                tc.point     = t.point;
                                tc.curvature = t.curvature;
                                tc.width     = t.width;
                                tc.source    = Tracking::TipCandidate::Source::SkeletonEndpoint;
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
                            }
                        }
                    }
                }
            }
            else if (er.topology == Tracking::TopologyState::SelfCrossed) {
                const int hIdx = blob.assignedHeadTipIdx;
                const int tIdx = blob.assignedTailTipIdx;
                const bool hasHead = (hIdx >= 0 && hIdx < static_cast<int>(blob.tipCandidates.size()));
                const bool hasTail = (tIdx >= 0 && tIdx < static_cast<int>(blob.tipCandidates.size()));

                if (hasHead || hasTail) {
                    // D-2 (two tips) / D-3 (one tip): trace the skeleton
                    // arc from the known tip to the target, picking the arc
                    // whose turning angle matches the previous frame (RHR).
                    runSkeletonArcDispatch(blob, er, originOffset);
                }
                else if (!blob.holeContourPoints.empty()) {
                    // 0 tips, closed ring: synthetic-hole-axis cut.
                    const std::vector<cv::Point>& hole = blob.holeContourPoints.front();
                    if (hole.size() >= 5) {
                        cv::RotatedRect rr = cv::fitEllipse(hole);
                        const float ang = static_cast<float>(rr.angle * CV_PI / 180.0);
                        const float halfMajor = std::max(rr.size.width,
                                                         rr.size.height) * 0.5f + 6.f;
                        const cv::Point2f dir(std::cos(ang), std::sin(ang));
                        const cv::Point2f cutA(rr.center.x - dir.x * halfMajor,
                                               rr.center.y - dir.y * halfMajor);
                        const cv::Point2f cutB(rr.center.x + dir.x * halfMajor,
                                               rr.center.y + dir.y * halfMajor);
                        Tracking::DetectedBlob cutBlob = blob;
                        if (Tracking::populateCenterlineFromContourWithCut(
                                cutBlob, cutA, cutB, /*thickness*/ 3) &&
                            cutBlob.centerlinePoints.size() >= 2) {
                            centerline.assign(cutBlob.centerlinePoints.begin(),
                                              cutBlob.centerlinePoints.end());
                            blob.centerlineCutPoint = (cutA + cutB) * 0.5f;
                            blob.hasCenterlineCutPoint = true;
                        }
                    }
                }
            }

            if (centerline.empty()) {
                // D-4 fallback: legacy contour-skeleton path.
                Tracking::DetectedBlob fallback = blob;
                if (Tracking::populateCenterlineFromContour(fallback) &&
                    fallback.centerlinePoints.size() >= 2) {
                    centerline.assign(fallback.centerlinePoints.begin(),
                                      fallback.centerlinePoints.end());
                }
            }

            if (centerline.size() < 2) {
                // No centerline producible. Persist what we DID compute
                // (tip data) so the renderer still shows green dots.
                m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal,
                                                   wormId, blob);
                prevState.valid = false;
                return;
            }

            // ── STEP 3: resample to nPoints ─────────────────────────────
            if (static_cast<int>(centerline.size()) != nPts)
                centerline = resample(centerline, nPts);

            if (cleanWithTwoTips) {
                m_storage->recordBodyLengthSample(wormId, arcLen(centerline));
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
                                    pinH, pinT, nPts, m_snakeParams,
                                    tmpOverlap, tmpHasOverlap);
                }
            }

            // Right-hand-rule orientation veto. The total signed turning
            // angle of the polyline negates when the traversal direction
            // is reversed; worms don't instantaneously flip their coiling
            // sense, so a sign flip across frames is a strong evidence of
            // a wrong orientation pick.
            const float curTurning = totalSignedTurningAngle(centerline);
            bool flipped = false;
            if (prevState.valid &&
                std::abs(prevState.turningAngle) > angleThreshold &&
                std::abs(curTurning) > angleThreshold &&
                curTurning * prevState.turningAngle < 0.f) {
                std::reverse(centerline.begin(), centerline.end());
                flipped = true;
                std::swap(blob.assignedHeadTipIdx, blob.assignedTailTipIdx);
            }

            // Keyframe bootstrap re-derivation: now that we have a
            // centerline, set head/tail from its natural orientation
            // (front = head). This stabilises the convention regardless
            // of which tip arbitrarily got tips[0] in detectEndpoints.
            if (isKeyframeBootstrap && centerline.size() >= 2 &&
                !blob.tipCandidates.empty()) {
                const int headIdx = nearestCandidateIdx(blob, centerline.front());
                int       tailIdx = nearestCandidateIdx(blob, centerline.back());
                if (tailIdx == headIdx) tailIdx = -1;
                blob.assignedHeadTipIdx = headIdx;
                blob.assignedTailTipIdx = tailIdx;
            }

            // Persist centerline + cut/overlap marker on the blob.
            blob.centerlinePoints.assign(centerline.begin(), centerline.end());
            if (hasOverlap) {
                blob.centerlineCutPoint = overlapCenter;
                blob.hasCenterlineCutPoint = true;
            }

            m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal,
                                               wormId, blob);

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
            predictor.hasVelocity = predictor.hasPrev &&
                                    (hIdx >= 0 || tIdx >= 0);
            predictor.hasPrev = true;
            if (refLength > 0.f)
                predictor.refDistance = std::max(8.f, 0.5f * refLength);

            // CenterlineState carry for next frame's RHR check.
            prevState.points       = centerline;
            prevState.blobCentroid = blobCentroid(blob);
            prevState.blob         = blob;
            prevState.valid        = true;
            prevState.turningAngle = flipped ? -curTurning : curTurning;
        };

        // ── Sweep 1 entry: process keyframe, then propagate outward ────
        Tracking::HeadTailPredictor seedPredictor;
        CenterlineState seedState;
        processFrame(keyframeIdx, seedPredictor, seedState,
                     /*isKeyframeBootstrap=*/true);

        // Forward pass.
        {
            Tracking::HeadTailPredictor predictor = seedPredictor;
            CenterlineState prevState = seedState;
            for (int i = keyframeIdx + 1;
                 i < static_cast<int>(sortedPoints.size()); ++i) {
                processFrame(i, predictor, prevState,
                             /*isKeyframeBootstrap=*/false);
            }
        }

        // Backward pass.
        {
            Tracking::HeadTailPredictor predictor = seedPredictor;
            CenterlineState prevState = seedState;
            for (int i = keyframeIdx - 1; i >= 0; --i) {
                processFrame(i, predictor, prevState,
                             /*isKeyframeBootstrap=*/false);
            }
        }

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}

void CenterlineWorker::doWorkLegacy()
{
    if (!m_storage) {
        emit failed("No storage provided to CenterlineWorker");
        return;
    }

    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    int totalWorms = static_cast<int>(tracks.size());
    if (totalWorms == 0) {
        emit progress(100);
        emit finished();
        return;
    }

    // Discard any prior baselines: a rerun reads from the same clean frames
    // and produces the same distributions, so re-accumulating into existing
    // counts would double the sample count without adding information.
    m_storage->clearAllTipBaselines();

    int processedWorms = 0;

    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        int wormId = it->first;

        // Track points are processed in temporal order.  The blob centerlines
        // are written back to storage but the WormTrackPoint.position values
        // are *not* modified — the original blob centroid is preserved.
        std::vector<Tracking::WormTrackPoint> sortedPoints = it->second;
        std::sort(sortedPoints.begin(), sortedPoints.end(),
                  [](const Tracking::WormTrackPoint& a,
                     const Tracking::WormTrackPoint& b) {
                      return a.frameNumberOriginal < b.frameNumberOriginal;
                  });

        // Per-worm keyframe: the frame on which the user originally clicked
        // this worm.  Centerline correction propagates outward from there in
        // both directions because the keyframe is guaranteed to have a clean,
        // separated blob (worms are picked when they're individually visible).
        int keyframe = -1;
        if (const TableItems::ClickedItem* item = m_storage->getItem(wormId))
            keyframe = item->frameOfSelection;

        // ── Pass 1: compute first-pass centerlines, learn body length ──
        std::vector<float> validLengths;
        validLengths.reserve(sortedPoints.size());

        for (const Tracking::WormTrackPoint& tp : sortedPoints) {
            if (tp.quality == Tracking::TrackPointQuality::Merged ||
                tp.quality == Tracking::TrackPointQuality::Lost)
                continue;

            QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (!frameBlobs.contains(wormId)) continue;

            Tracking::DetectedBlob blob = frameBlobs[wormId];
            if (!blob.isValid || blob.contourPoints.empty()) continue;

            if (blob.centerlinePoints.empty())
                Tracking::populateCenterlineFromContour(blob);

            if (!blob.centerlinePoints.empty()) {
                // Reference length is taken only from non-ring frames where
                // the skeleton is reliable.
                if (blob.holeContourPoints.empty()) {
                    std::vector<cv::Point2f> p(blob.centerlinePoints.begin(),
                                               blob.centerlinePoints.end());
                    validLengths.push_back(arcLen(p));
                }
                m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);
            }
        }

        // Use the median length as a robust reference (resistant to outliers).
        float refLength = 0.f;
        if (!validLengths.empty()) {
            std::nth_element(validLengths.begin(),
                             validLengths.begin() + validLengths.size() / 2,
                             validLengths.end());
            refLength = validLengths[validLengths.size() / 2];
        }

        // ── Pass 2: keyframe-outward orientation and repair ──
        // Find the index in sortedPoints whose frame == keyframe (or fall back
        // to the first frame if the keyframe didn't yield a valid track point).
        int keyframeIdx = -1;
        if (keyframe >= 0) {
            for (size_t i = 0; i < sortedPoints.size(); ++i) {
                if (sortedPoints[i].frameNumberOriginal == keyframe) {
                    keyframeIdx = static_cast<int>(i);
                    break;
                }
            }
        }
        if (keyframeIdx < 0) keyframeIdx = 0;

        auto runDirection = [&](int startIdx, int step,
                                CenterlineState seedState) {
            CenterlineState prev = seedState;
            int n = static_cast<int>(sortedPoints.size());
            for (int i = startIdx; i >= 0 && i < n; i += step) {
                Tracking::WormTrackPoint& tp = sortedPoints[i];
                if (tp.quality == Tracking::TrackPointQuality::Merged ||
                    tp.quality == Tracking::TrackPointQuality::Lost) {
                    // No prediction available across merged/lost gaps; keep
                    // the first-pass guess for those frames untouched and
                    // resume orientation when we see a clean frame again.
                    prev.valid = false;
                    continue;
                }

                QMap<int, Tracking::DetectedBlob> frameBlobs =
                    m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
                if (!frameBlobs.contains(wormId)) continue;

                Tracking::DetectedBlob blob = frameBlobs[wormId];

                CenterlineState out;
                if (processOneFrame(blob, prev, refLength,
                                    std::max(4, m_snakeParams.nPoints),
                                    kMinArcLengthFraction,
                                    m_snakeParams, out)) {
                    m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);
                    prev = out;
                } else {
                    prev.valid = false;
                }
            }
        };

        // Seed from the keyframe itself; we orient it without history (its
        // first-pass centerline simply defines our convention for this worm).
        CenterlineState seed;
        {
            const Tracking::WormTrackPoint& tp = sortedPoints[keyframeIdx];
            QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (frameBlobs.contains(wormId)) {
                Tracking::DetectedBlob blob = frameBlobs[wormId];
                CenterlineState out;
                CenterlineState empty;
                if (processOneFrame(blob, empty, refLength,
                                    std::max(4, m_snakeParams.nPoints),
                                    kMinArcLengthFraction,
                                    m_snakeParams, out)) {
                    m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);
                    seed = out;
                }
            }
        }

        // Propagate forward and backward from the keyframe.
        runDirection(keyframeIdx + 1, +1, seed);
        runDirection(keyframeIdx - 1, -1, seed);

        // ── Pass 3: per-worm tip-feature baseline (Phase A) ─────────────────
        // For every clean-topology frame this worm has, sample both tips'
        // (|curvature|, width) features and the refined centerline arc-length
        // into the worm's running baseline. Downstream head/tail assignment
        // scores ambiguous tip candidates against this distribution.
        //
        // "Clean" here means: blob is valid, has no holes (no ring topology),
        // the worm is not part of a merge group at this frame, and the
        // tip-candidate detector surfaced exactly two SkeletonEndpoint tips
        // (the strongest available operational signal that the topology is
        // unambiguous and head/tail are both visible).
        for (const Tracking::WormTrackPoint& tp : sortedPoints) {
            if (tp.quality != Tracking::TrackPointQuality::Single &&
                tp.quality != Tracking::TrackPointQuality::Split)
                continue;

            // Exclude frames where this worm shares its blob with another.
            const QList<QList<int>> mergeGroups =
                m_storage->getMergeGroupsForFrame(tp.frameNumberOriginal);
            bool inMerge = false;
            for (const QList<int>& group : mergeGroups) {
                if (group.size() > 1 && group.contains(wormId)) { inMerge = true; break; }
            }
            if (inMerge) continue;

            const QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (!frameBlobs.contains(wormId)) continue;
            const Tracking::DetectedBlob& blob = frameBlobs[wormId];
            if (!blob.isValid || !blob.holeContourPoints.empty()) continue;

            // Require exactly two skeleton-derived candidates — our operational
            // definition of "clean topology with both tips visible."
            int firstSkel = -1, secondSkel = -1;
            int extraSkel = 0;
            for (size_t i = 0; i < blob.tipCandidates.size(); ++i) {
                if (blob.tipCandidates[i].source !=
                    Tracking::TipCandidate::Source::SkeletonEndpoint) continue;
                if (firstSkel < 0) firstSkel = static_cast<int>(i);
                else if (secondSkel < 0) secondSkel = static_cast<int>(i);
                else ++extraSkel;
            }
            if (firstSkel < 0 || secondSkel < 0 || extraSkel > 0) continue;

            const Tracking::TipCandidate& t1 = blob.tipCandidates[firstSkel];
            const Tracking::TipCandidate& t2 = blob.tipCandidates[secondSkel];
            m_storage->recordTipFeatureSample(wormId, std::abs(t1.curvature), t1.width);
            m_storage->recordTipFeatureSample(wormId, std::abs(t2.curvature), t2.width);

            if (blob.centerlinePoints.size() >= 2) {
                std::vector<cv::Point2f> p(blob.centerlinePoints.begin(),
                                           blob.centerlinePoints.end());
                m_storage->recordBodyLengthSample(wormId, arcLen(p));
            }
        }

        // ── Pass 4a: topology classification (Phase C.2) ────────────────────
        // Classify every blob this worm has into Clean / SelfCrossed / Merged
        // / Lost before head/tail assignment runs. The classification feeds
        // both the per-frame log line and the downstream Phase D dispatch
        // (Pass 5: geodesic-from-tips centerline init on SelfCrossed frames).
        for (const Tracking::WormTrackPoint& tp : sortedPoints) {
            const QList<QList<int>> mergeGroups =
                m_storage->getMergeGroupsForFrame(tp.frameNumberOriginal);
            bool inMerge = false;
            for (const QList<int>& group : mergeGroups) {
                if (group.size() > 1 && group.contains(wormId)) { inMerge = true; break; }
            }

            QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (!frameBlobs.contains(wormId)) continue;
            Tracking::DetectedBlob blob = frameBlobs[wormId];
            Tracking::classifyTopology(blob, inMerge);
            m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);
        }

        // ── Pass 4b: head/tail assignment (Phase C.1) ───────────────────────
        // Walks frames temporally from keyframe outward, mirroring Pass 2's
        // direction structure. For each frame:
        //   • At the keyframe, bootstrap the assignment from the centerline's
        //     own endpoint convention (front-of-polyline → head, back → tail,
        //     mapped to the nearest tip candidates). The centerline was
        //     finalised in Pass 2, so its orientation is already consistent
        //     with the worm's per-individual convention.
        //   • At every other frame, call assignHeadTail() with a predictor
        //     state seeded from the previous frame. Predictor velocity engages
        //     after two successful assignments.
        // Each frame's assignment indices are written back to storage.
        const Tracking::TipFeatureBaseline baseline = m_storage->getTipBaseline(wormId);

        // Map a target point to the nearest tip-candidate index. Used by both
        // the keyframe seed and any sanity-check overrides on disrupted frames.
        auto nearestCandidateIdx = [](const Tracking::DetectedBlob& b,
                                      const cv::Point2f& target) -> int {
            int bestIdx = -1;
            float bestDistSq = std::numeric_limits<float>::max();
            for (size_t i = 0; i < b.tipCandidates.size(); ++i) {
                const cv::Point2f d = b.tipCandidates[i].point - target;
                const float dsq = d.x * d.x + d.y * d.y;
                if (dsq < bestDistSq) { bestDistSq = dsq; bestIdx = static_cast<int>(i); }
            }
            return bestIdx;
        };

        // Seed predictor state from the keyframe's centerline.
        Tracking::HeadTailPredictor seedPredictor;
        {
            const Tracking::WormTrackPoint& tp = sortedPoints[keyframeIdx];
            QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (frameBlobs.contains(wormId)) {
                Tracking::DetectedBlob blob = frameBlobs[wormId];
                if (blob.isValid && !blob.tipCandidates.empty()
                    && blob.centerlinePoints.size() >= 2) {
                    const int headIdx = nearestCandidateIdx(blob, blob.centerlinePoints.front());
                    int       tailIdx = nearestCandidateIdx(blob, blob.centerlinePoints.back());
                    if (tailIdx == headIdx) tailIdx = -1;  // degenerate: single candidate
                    blob.assignedHeadTipIdx = headIdx;
                    blob.assignedTailTipIdx = tailIdx;
                    m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);

                    if (headIdx >= 0) {
                        seedPredictor.hasPrev = true;
                        seedPredictor.lastHeadPos = blob.tipCandidates[headIdx].point;
                        if (tailIdx >= 0)
                            seedPredictor.lastTailPos = blob.tipCandidates[tailIdx].point;
                        else
                            seedPredictor.lastTailPos = blob.tipCandidates[headIdx].point;
                        if (baseline.lengthSamples > 0)
                            seedPredictor.refDistance = std::max(8.f, 0.5f * baseline.meanBodyLength);
                        else if (refLength > 0.f)
                            seedPredictor.refDistance = std::max(8.f, 0.5f * refLength);

                        YAWT_DEBUG(lcCoreCenterlineWorker) << QString::asprintf(
                            "Pass4 seed   worm=%d frame=%d  head=cand[%d] tail=cand[%d]  refDist=%.1f",
                            wormId, tp.frameNumberOriginal, headIdx, tailIdx,
                            static_cast<double>(seedPredictor.refDistance));
                    }
                }
            }
        }

        auto runAssignmentDirection = [&](int startIdx, int step,
                                          Tracking::HeadTailPredictor predictor) {
            const int n = static_cast<int>(sortedPoints.size());
            for (int i = startIdx; i >= 0 && i < n; i += step) {
                const Tracking::WormTrackPoint& tp = sortedPoints[i];
                if (tp.quality == Tracking::TrackPointQuality::Lost) {
                    // Don't propagate priors across a lost segment — when we
                    // resume tracking the worm is in an unknown position.
                    predictor = Tracking::HeadTailPredictor{};
                    YAWT_DEBUG(lcCoreCenterlineWorker) << QString::asprintf(
                        "Pass4 lost   worm=%d frame=%d  -> predictor reset",
                        wormId, tp.frameNumberOriginal);
                    continue;
                }

                QMap<int, Tracking::DetectedBlob> frameBlobs =
                    m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
                if (!frameBlobs.contains(wormId)) continue;
                Tracking::DetectedBlob blob = frameBlobs[wormId];

                // Context header — the assignHeadTail debug lines that follow
                // will be attributable to this (worm, frame) pair.
                YAWT_DEBUG(lcCoreCenterlineWorker) << QString::asprintf(
                    "Pass4        worm=%d frame=%d  step=%+d  topo=%s",
                    wormId, tp.frameNumberOriginal, step,
                    qUtf8Printable(Tracking::topologyStateToString(blob.topologyState)));

                if (!Tracking::assignHeadTail(blob, predictor, baseline)) continue;

                // ── Inline D-3: one assigned tip on a SelfCrossed frame ────────
                // assignHeadTail may leave one role unset (-1) when only a single
                // skeleton endpoint was visible. Run the body-length geodesic HERE
                // (not in a later pass) so the predictor's lastHeadPos/lastTailPos
                // is updated before the NEXT frame's assignment runs. Without this
                // the missing role's predicted position goes stale across runs of
                // consecutive self-crossed frames.
                {
                    const int hIdx = blob.assignedHeadTipIdx;
                    const int tIdx = blob.assignedTailTipIdx;
                    const bool oneHead = (hIdx >= 0 && tIdx < 0);
                    const bool oneTail = (tIdx >= 0 && hIdx < 0);
                    if ((oneHead || oneTail)
                        && blob.topologyState == Tracking::TopologyState::SelfCrossed) {
                        const int knownIdx = oneHead ? hIdx : tIdx;
                        const cv::Point2f knownTip = blob.tipCandidates[knownIdx].point;
                        const int nPts = std::max(4, m_snakeParams.nPoints);
                        std::vector<cv::Point2f> refined;
                        cv::Point2f hiddenTip, overlapCenter;
                        bool hasOverlap = false;
                        if (refineCenterlineSnakeFromOneTip(blob, knownTip,
                                                            nPts, m_snakeParams,
                                                            refined, hiddenTip,
                                                            overlapCenter, hasOverlap)) {
                            Tracking::TipCandidate hypTip;
                            hypTip.point  = hiddenTip;
                            hypTip.source = Tracking::TipCandidate::Source::HypothesizedHidden;
                            blob.tipCandidates.push_back(hypTip);
                            const int newIdx = static_cast<int>(blob.tipCandidates.size()) - 1;
                            if (oneHead) blob.assignedTailTipIdx = newIdx;
                            else         blob.assignedHeadTipIdx = newIdx;
                            blob.centerlinePoints.assign(refined.begin(), refined.end());
                            if (hasOverlap) {
                                blob.centerlineCutPoint = overlapCenter;
                                blob.hasCenterlineCutPoint = true;
                            }
                            YAWT_DEBUG(lcCoreCenterlineWorker) << QString::asprintf(
                                "Pass4 D-3    worm=%d frame=%d  known=%s @ (%.1f,%.1f) "
                                "-> hidden=(%.1f,%.1f)  geodDist=%.1f",
                                wormId, tp.frameNumberOriginal,
                                oneHead ? "head" : "tail",
                                static_cast<double>(knownTip.x),
                                static_cast<double>(knownTip.y),
                                static_cast<double>(hiddenTip.x),
                                static_cast<double>(hiddenTip.y),
                                static_cast<double>(ptDist(knownTip, hiddenTip)));
                        }
                    }
                }

                m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);

                // Update predictor for the next frame. Velocity engages after
                // we have two successful assignments in a row for that role.
                // The indices here reflect both Pass-4b skeleton assignments and
                // any D-3 hidden-tip that was just inferred above.
                const int headIdx = blob.assignedHeadTipIdx;
                const int tailIdx = blob.assignedTailTipIdx;
                if (headIdx >= 0) {
                    const cv::Point2f newHead = blob.tipCandidates[headIdx].point;
                    predictor.velHead = newHead - predictor.lastHeadPos;
                    predictor.lastHeadPos = newHead;
                }
                if (tailIdx >= 0) {
                    const cv::Point2f newTail = blob.tipCandidates[tailIdx].point;
                    predictor.velTail = newTail - predictor.lastTailPos;
                    predictor.lastTailPos = newTail;
                }
                // hasVelocity becomes true after the first per-direction step
                // that found at least one assignment (we just updated velHead
                // and/or velTail with a meaningful delta).
                predictor.hasVelocity = predictor.hasPrev &&
                                        (headIdx >= 0 || tailIdx >= 0);
                predictor.hasPrev = true;
            }
        };

        runAssignmentDirection(keyframeIdx + 1, +1, seedPredictor);
        runAssignmentDirection(keyframeIdx - 1, -1, seedPredictor);

        // ── Pass 5: D-2 geodesic centerline refinement (Phase C.3) ─────────
        // For SelfCrossed frames where BOTH head and tail are now assigned
        // (either both from skeleton endpoints in Pass 4b, or head from skel +
        // tail from the inline D-3 above), replace the Pass 2 centerline with
        // a geodesic shortest path through the current blob mask from the
        // assigned head to the assigned tail. This removes the stale-topology
        // bias of the Pass 2 snake init.
        //
        // D-3 (single visible tip → infer hidden tip by body-length geodesic)
        // is now handled inline in runAssignmentDirection so that the predictor
        // is updated immediately. Pass 5 only needs to re-run the snake for
        // frames where both tips were already assigned by Pass 4b (D-2 case).
        //
        // Clean frames are skipped — skeleton centerline is correct.
        // Merged / Lost frames are skipped — geodesic not meaningful.
        // SelfCrossed with no assigned tips → D-4 fallback (Pass 2 result kept).
        const int nPointsSnake = std::max(4, m_snakeParams.nPoints);
        int d2Count = 0;
        for (const Tracking::WormTrackPoint& tp : sortedPoints) {
            QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (!frameBlobs.contains(wormId)) continue;
            Tracking::DetectedBlob blob = frameBlobs[wormId];
            if (blob.topologyState != Tracking::TopologyState::SelfCrossed) continue;

            const int hIdx = blob.assignedHeadTipIdx;
            const int tIdx = blob.assignedTailTipIdx;
            const bool hasHead = (hIdx >= 0 && hIdx < static_cast<int>(blob.tipCandidates.size()));
            const bool hasTail = (tIdx >= 0 && tIdx < static_cast<int>(blob.tipCandidates.size()));

            // D-3 frames (HypothesizedHidden tail) were already refined inline;
            // skip them here to avoid re-running the snake redundantly.
            if (hasTail && blob.tipCandidates[tIdx].source ==
                               Tracking::TipCandidate::Source::HypothesizedHidden) continue;
            if (hasHead && blob.tipCandidates[hIdx].source ==
                               Tracking::TipCandidate::Source::HypothesizedHidden) continue;

            if (!hasHead || !hasTail) continue;

            // D-2: geodesic head → tail, both tips are skeleton-confirmed.
            const cv::Point2f head = blob.tipCandidates[hIdx].point;
            const cv::Point2f tail = blob.tipCandidates[tIdx].point;
            std::vector<cv::Point2f> refined;
            cv::Point2f overlapCenter;
            bool hasOverlap = false;
            if (refineCenterlineSnakeFromTips(blob, head, tail, nPointsSnake,
                                              m_snakeParams,
                                              refined, overlapCenter, hasOverlap)) {
                blob.centerlinePoints.assign(refined.begin(), refined.end());
                if (hasOverlap) {
                    blob.centerlineCutPoint = overlapCenter;
                    blob.hasCenterlineCutPoint = true;
                }
                m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);
                ++d2Count;
                YAWT_DEBUG(lcCoreCenterlineWorker) << QString::asprintf(
                    "Pass5 D-2    worm=%d frame=%d  head=(%.1f,%.1f) tail=(%.1f,%.1f)  overlap=%d",
                    wormId, tp.frameNumberOriginal,
                    static_cast<double>(head.x), static_cast<double>(head.y),
                    static_cast<double>(tail.x), static_cast<double>(tail.y),
                    hasOverlap ? 1 : 0);
            }
        }
        YAWT_DEBUG(lcCoreCenterlineWorker) << QString::asprintf(
            "Pass5 summary worm=%d  D-2=%d",
            wormId, d2Count);

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}

// ── Diagnostic export: per-stage images + decision log ─────────────────────
//
// Re-runs the per-frame centerline pipeline (same dispatch as Sweep 1's
// processFrame in doWorkNew) for one (wormId, frameNumber), writing an image
// after each stage that contributed a decision plus a text log explaining
// what was decided and why. Reads storage only; never writes back.
//
// Predictor state is reconstructed from the immediately adjacent frame's
// stored blob (frameNumber-1, falling back to frameNumber+1) so head/tail
// assignment behaves like the live sweep on this frame in isolation.

namespace {

constexpr int kExportScale = 4;
constexpr int kExportPad   = 12;

static cv::Rect computeExportBounds(const Tracking::DetectedBlob& blob)
{
    cv::Rect b = cv::boundingRect(blob.contourPoints);
    b.x -= kExportPad;
    b.y -= kExportPad;
    b.width  += 2 * kExportPad;
    b.height += 2 * kExportPad;
    return b;
}

static cv::Point worldToCanvas(const cv::Point2f& w, const cv::Rect& bounds)
{
    return cv::Point(static_cast<int>(std::lround((w.x - bounds.x) * kExportScale)),
                     static_cast<int>(std::lround((w.y - bounds.y) * kExportScale)));
}

static cv::Mat makeBaseCanvas(const Tracking::DetectedBlob& blob,
                              const cv::Rect& bounds)
{
    cv::Mat mask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    fillBlobMask(mask, blob, bounds, cv::Point2f(0.f, 0.f));

    cv::Mat upMask;
    cv::resize(mask, upMask, cv::Size(), kExportScale, kExportScale, cv::INTER_NEAREST);

    cv::Mat canvas = cv::Mat::zeros(upMask.size(), CV_8UC3);
    canvas.setTo(cv::Scalar(50, 50, 50), upMask == 255);
    return canvas;
}

static void drawContoursOverlay(cv::Mat& canvas,
                                const Tracking::DetectedBlob& blob,
                                const cv::Rect& bounds)
{
    std::vector<cv::Point> outer;
    outer.reserve(blob.contourPoints.size());
    for (const cv::Point& p : blob.contourPoints) {
        outer.push_back(cv::Point((p.x - bounds.x) * kExportScale,
                                  (p.y - bounds.y) * kExportScale));
    }
    if (!outer.empty()) {
        std::vector<std::vector<cv::Point>> cs{outer};
        cv::polylines(canvas, cs, /*closed=*/true, cv::Scalar(0, 220, 0),
                      1, cv::LINE_AA);
    }
    for (const auto& hole : blob.holeContourPoints) {
        std::vector<cv::Point> h;
        h.reserve(hole.size());
        for (const cv::Point& p : hole) {
            h.push_back(cv::Point((p.x - bounds.x) * kExportScale,
                                  (p.y - bounds.y) * kExportScale));
        }
        std::vector<std::vector<cv::Point>> cs{h};
        cv::polylines(canvas, cs, true, cv::Scalar(0, 0, 220), 1, cv::LINE_AA);
    }
}

static void drawTitle(cv::Mat& canvas, const QString& title)
{
    cv::putText(canvas, title.toStdString(), cv::Point(8, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

static void drawText(cv::Mat& canvas, const QString& s, cv::Point at,
                     cv::Scalar color = cv::Scalar(255, 255, 255))
{
    cv::putText(canvas, s.toStdString(), at, cv::FONT_HERSHEY_SIMPLEX,
                0.45, color, 1, cv::LINE_AA);
}

static void writeStageImage(const cv::Mat& img, const QString& dir,
                            const QString& name)
{
    const QString path = QDir(dir).absoluteFilePath(name);
    cv::imwrite(path.toStdString(), img);
}

static Tracking::HeadTailPredictor reconstructPredictor(
    TrackingDataStorage* storage, int wormId, int frameNumber,
    QString& outNote)
{
    Tracking::HeadTailPredictor predictor;
    auto blobAt = [&](int f, Tracking::DetectedBlob& out) -> bool {
        const auto map = storage->getDetectedBlobsForFrame(f);
        if (!map.contains(wormId)) return false;
        out = map[wormId];
        return out.isValid && !out.contourPoints.empty();
    };

    Tracking::DetectedBlob adj;
    int adjFrame = -1;
    if (blobAt(frameNumber - 1, adj))      adjFrame = frameNumber - 1;
    else if (blobAt(frameNumber + 1, adj)) adjFrame = frameNumber + 1;
    if (adjFrame < 0) {
        outNote = "no adjacent frame with a stored blob — predictor is empty";
        return predictor;
    }

    auto adjHead = (adj.assignedHeadTipIdx >= 0 &&
                    adj.assignedHeadTipIdx < (int)adj.tipCandidates.size())
                   ? adj.tipCandidates[adj.assignedHeadTipIdx].point
                   : cv::Point2f(-1.f, -1.f);
    auto adjTail = (adj.assignedTailTipIdx >= 0 &&
                    adj.assignedTailTipIdx < (int)adj.tipCandidates.size())
                   ? adj.tipCandidates[adj.assignedTailTipIdx].point
                   : cv::Point2f(-1.f, -1.f);

    predictor.lastHeadPos = adjHead;
    predictor.lastTailPos = adjTail;
    predictor.hasPrev = (adjHead.x >= 0 || adjTail.x >= 0);

    Tracking::DetectedBlob adj2;
    const int adj2Frame = adjFrame + (adjFrame < frameNumber ? -1 : 1);
    if (blobAt(adj2Frame, adj2)) {
        const auto h2 = (adj2.assignedHeadTipIdx >= 0 &&
                         adj2.assignedHeadTipIdx < (int)adj2.tipCandidates.size())
                        ? adj2.tipCandidates[adj2.assignedHeadTipIdx].point
                        : cv::Point2f(-1.f, -1.f);
        const auto t2 = (adj2.assignedTailTipIdx >= 0 &&
                         adj2.assignedTailTipIdx < (int)adj2.tipCandidates.size())
                        ? adj2.tipCandidates[adj2.assignedTailTipIdx].point
                        : cv::Point2f(-1.f, -1.f);
        if (h2.x >= 0 && adjHead.x >= 0) predictor.velHead = adjHead - h2;
        if (t2.x >= 0 && adjTail.x >= 0) predictor.velTail = adjTail - t2;
        predictor.hasVelocity = (h2.x >= 0 && adjHead.x >= 0) ||
                                (t2.x >= 0 && adjTail.x >= 0);
    }

    outNote = QString("predictor reconstructed from frame %1%2")
        .arg(adjFrame).arg(predictor.hasVelocity ? " (with velocity)" : "");
    return predictor;
}

} // anonymous namespace

bool CenterlineWorker::exportProcessForFrame(
    TrackingDataStorage* storage,
    int wormId,
    int frameNumber,
    const Tracking::CenterlineSnakeParams& snakeParams,
    const QString& outputDir,
    QString* outErrorMsg)
{
    auto fail = [&](const QString& msg) -> bool {
        if (outErrorMsg) *outErrorMsg = msg;
        return false;
    };
    if (!storage)            return fail("no storage");
    if (outputDir.isEmpty()) return fail("empty output dir");
    if (!QDir().mkpath(outputDir)) return fail("could not create output dir");

    const QMap<int, Tracking::DetectedBlob> frameBlobs =
        storage->getDetectedBlobsForFrame(frameNumber);
    if (!frameBlobs.contains(wormId))
        return fail(QString("no stored blob for worm %1 on frame %2")
                    .arg(wormId).arg(frameNumber));
    Tracking::DetectedBlob blob = frameBlobs[wormId];
    if (!blob.isValid || blob.contourPoints.empty())
        return fail("blob is invalid or has no contour");

    bool inMerge = false;
    {
        const QList<QList<int>> groups =
            storage->getMergeGroupsForFrame(frameNumber);
        for (const auto& g : groups)
            if (g.size() > 1 && g.contains(wormId)) { inMerge = true; break; }
    }

    QString predictorNote;
    Tracking::HeadTailPredictor predictor =
        reconstructPredictor(storage, wormId, frameNumber, predictorNote);
    const Tracking::TipFeatureBaseline baseline = storage->getTipBaseline(wormId);

    const int nPts = std::max(4, snakeParams.nPoints);
    const float angleThreshold = static_cast<float>(
        std::max(0.0, snakeParams.orientationAngleThreshold));

    // Recover the previous frame's turning angle from its stored centerline
    // so the RHR arc-picker in Stage 05 behaves like the live sweep.
    float prevTurningAngle = 0.f;
    {
        const int adjFrame = frameNumber - 1;
        const auto adjBlobs = storage->getDetectedBlobsForFrame(adjFrame);
        if (adjBlobs.contains(wormId)) {
            const auto& pts = adjBlobs[wormId].centerlinePoints;
            if (pts.size() >= 3) {
                std::vector<cv::Point2f> v(pts.begin(), pts.end());
                prevTurningAngle = totalSignedTurningAngle(v);
            }
        }
    }

    QString logBuf;
    QTextStream log(&logBuf);
    log << "=== Centerline process export ===\n";
    log << "worm: "  << wormId << "  frame: " << frameNumber << "\n";
    log << "outputDir: " << outputDir << "\n";
    log << "snake.enabled=" << (snakeParams.enabled ? "true" : "false")
        << " alpha=" << snakeParams.alpha
        << " beta=" << snakeParams.beta
        << " lambda=" << snakeParams.lambda
        << " iters=" << snakeParams.iterations
        << " step=" << snakeParams.stepSize
        << " orientThr=" << snakeParams.orientationAngleThreshold
        << " nPoints=" << snakeParams.nPoints << "\n";
    log << "inMergeGroup=" << (inMerge ? "true" : "false") << "\n";
    log << "predictor: hasPrev=" << (predictor.hasPrev ? "Y" : "N")
        << " hasVel=" << (predictor.hasVelocity ? "Y" : "N")
        << " lastHead=(" << predictor.lastHeadPos.x << "," << predictor.lastHeadPos.y << ")"
        << " lastTail=(" << predictor.lastTailPos.x << "," << predictor.lastTailPos.y << ")"
        << " velHead=(" << predictor.velHead.x << "," << predictor.velHead.y << ")"
        << " velTail=(" << predictor.velTail.x << "," << predictor.velTail.y << ")\n";
    log << "predictorSource: " << predictorNote << "\n";
    log << "baseline: kappaSamples=" << baseline.curvatureSamples
        << " (|kappa|=" << baseline.meanAbsCurvature << "+/-" << baseline.curvatureStdDev() << ")"
        << " widthSamples=" << baseline.widthSamples
        << " (w=" << baseline.meanWidth << "+/-" << baseline.widthStdDev() << ")"
        << " lengthSamples=" << baseline.lengthSamples
        << " (L=" << baseline.meanBodyLength << "+/-" << baseline.bodyLengthStdDev() << ")"
        << " reliable=" << (baseline.isReliable() ? "Y" : "N") << "\n";
    log << "blob: area=" << blob.area
        << " hullArea=" << blob.convexHullArea
        << " holes=" << blob.holeContourPoints.size()
        << " contourPts=" << blob.contourPoints.size()
        << " touchesROIbound=" << (blob.touchesROIboundary ? "Y" : "N") << "\n";
    log << "\n";

    // Stage 00 — raw mask + outer/hole contours.
    const cv::Rect bounds = computeExportBounds(blob);
    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        drawTitle(canvas, QString("00 mask  worm %1  frame %2")
                  .arg(wormId).arg(frameNumber));
        drawText(canvas,
            QString("outer=green holes=red  area=%1 hullArea=%2 holes=%3")
                .arg(blob.area, 0, 'f', 0)
                .arg(blob.convexHullArea, 0, 'f', 0)
                .arg(blob.holeContourPoints.size()),
            cv::Point(8, canvas.rows - 10));
        writeStageImage(canvas, outputDir, "00_mask.png");
    }
    log << "Stage 00 (mask): bounds=" << bounds.x << "," << bounds.y
        << " " << bounds.width << "x" << bounds.height << "\n\n";

    // Stage 01/02/03/04 — detectEndpoints output.
    Tracking::EndpointResult er = Tracking::detectEndpoints(
        blob, predictor, baseline, inMerge);
    const cv::Point2f originOffset(static_cast<float>(er.localBounds.x),
                                   static_cast<float>(er.localBounds.y));

    // 01 — skeleton + degree-1 endpoints.
    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        if (!er.skeleton.skeleton.empty()) {
            for (int y = 0; y < er.skeleton.skeleton.rows; ++y) {
                const uchar* row = er.skeleton.skeleton.ptr<uchar>(y);
                for (int x = 0; x < er.skeleton.skeleton.cols; ++x) {
                    if (!row[x]) continue;
                    const cv::Point2f wp(static_cast<float>(x) + originOffset.x,
                                         static_cast<float>(y) + originOffset.y);
                    cv::circle(canvas, worldToCanvas(wp, bounds), 1,
                               cv::Scalar(255, 255, 0), cv::FILLED);
                }
            }
        }
        for (int idx : er.skeleton.endpointIndices) {
            if (idx < 0 || idx >= (int)er.skeleton.points.size()) continue;
            const cv::Point& lp = er.skeleton.points[idx];
            const cv::Point2f wp(static_cast<float>(lp.x) + originOffset.x,
                                 static_cast<float>(lp.y) + originOffset.y);
            cv::circle(canvas, worldToCanvas(wp, bounds), 6,
                       cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        }
        drawTitle(canvas, QString("01 skeleton  degree-1 endpoints=%1")
                  .arg(er.skeleton.endpointIndices.size()));
        writeStageImage(canvas, outputDir, "01_skeleton.png");
    }
    log << "Stage 01 (skeleton): points=" << er.skeleton.points.size()
        << "  degree-1 endpoints=" << er.skeleton.endpointIndices.size() << "\n\n";

    // 02 — distance transform (heatmap).
    if (!er.distTransform.empty()) {
        cv::Mat dt8;
        double mn = 0, mx = 0;
        cv::minMaxLoc(er.distTransform, &mn, &mx);
        er.distTransform.convertTo(dt8, CV_8U, (mx > 0 ? 255.0 / mx : 0.0));
        cv::Mat dtFull = cv::Mat::zeros(bounds.height, bounds.width, CV_8U);
        const int dx = er.localBounds.x - bounds.x;
        const int dy = er.localBounds.y - bounds.y;
        for (int y = 0; y < dt8.rows; ++y) {
            const int dest = y + dy;
            if (dest < 0 || dest >= dtFull.rows) continue;
            for (int x = 0; x < dt8.cols; ++x) {
                const int destX = x + dx;
                if (destX < 0 || destX >= dtFull.cols) continue;
                dtFull.at<uchar>(dest, destX) = dt8.at<uchar>(y, x);
            }
        }
        cv::Mat dtUp;
        cv::resize(dtFull, dtUp, cv::Size(),
                   kExportScale, kExportScale, cv::INTER_NEAREST);
        cv::Mat heat;
        cv::applyColorMap(dtUp, heat, cv::COLORMAP_INFERNO);
        drawTitle(heat, QString("02 distance transform  max=%1px")
                  .arg(mx, 0, 'f', 1));
        writeStageImage(heat, outputDir, "02_distance_transform.png");
        log << "Stage 02 (distance transform): max=" << mx << " px (medial-axis ridge)\n\n";
    }

    // 03 — TrueTips with curvature/width labels.
    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        for (size_t i = 0; i < er.tips.size(); ++i) {
            const auto& t = er.tips[i];
            const cv::Point cp = worldToCanvas(t.point, bounds);
            const cv::Scalar col = t.extended
                ? cv::Scalar(255, 80, 255)
                : cv::Scalar(80, 255, 80);
            cv::circle(canvas, cp, 7, col, cv::FILLED);
            cv::circle(canvas, cp, 7, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
            if (t.extended) {
                cv::circle(canvas, worldToCanvas(t.skelPoint, bounds), 3,
                           cv::Scalar(80, 255, 80), 1, cv::LINE_AA);
            }
            drawText(canvas,
                QString("tip%1 |k|=%2 w=%3%4")
                    .arg(i)
                    .arg(std::abs(t.curvature), 0, 'f', 3)
                    .arg(t.width, 0, 'f', 1)
                    .arg(t.extended ? " ext" : ""),
                cv::Point(cp.x + 9, cp.y - 4));
        }
        drawTitle(canvas, QString("03 true tips  n=%1").arg(er.tips.size()));
        writeStageImage(canvas, outputDir, "03_true_tips.png");
    }
    log << "Stage 03 (true tips):\n";
    for (size_t i = 0; i < er.tips.size(); ++i) {
        const auto& t = er.tips[i];
        log << "  tip[" << i << "] point=(" << t.point.x << "," << t.point.y << ")"
            << " skel=(" << t.skelPoint.x << "," << t.skelPoint.y << ")"
            << " |kappa|=" << std::abs(t.curvature)
            << " width=" << t.width
            << " extended=" << (t.extended ? "Y" : "N") << "\n";
    }
    log << "\n";

    // 04 — head/tail assignment + topology.
    log << "Stage 04 (head/tail + topology):\n";
    log << "  topology    = " << Tracking::topologyStateToString(er.topology) << "\n";
    log << "  inMergeGroup= " << (inMerge ? "true" : "false") << "\n";
    log << "  headIdx     = " << er.headIdx << "   tailIdx = " << er.tailIdx << "\n";
    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        for (size_t i = 0; i < er.tips.size(); ++i) {
            const cv::Point cp = worldToCanvas(er.tips[i].point, bounds);
            cv::Scalar col(180, 180, 180);
            if ((int)i == er.headIdx)      col = cv::Scalar(0, 0, 255);
            else if ((int)i == er.tailIdx) col = cv::Scalar(255, 80, 0);
            cv::circle(canvas, cp, 8, col, cv::FILLED);
            cv::circle(canvas, cp, 8, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
            const char* label = ((int)i == er.headIdx) ? "H"
                              : ((int)i == er.tailIdx) ? "T" : "?";
            drawText(canvas, label, cv::Point(cp.x - 4, cp.y + 5));
        }
        if (predictor.hasPrev) {
            auto drawPred = [&](const cv::Point2f& last, const cv::Point2f& vel,
                                const cv::Scalar& col, const char* label) {
                if (last.x < 0) return;
                cv::Point2f pred = predictor.hasVelocity ? (last + vel) : last;
                cv::circle(canvas, worldToCanvas(pred, bounds), 5, col, 1, cv::LINE_AA);
                drawText(canvas, label,
                         worldToCanvas(pred, bounds) + cv::Point(7, -4), col);
            };
            drawPred(predictor.lastHeadPos, predictor.velHead,
                     cv::Scalar(0, 0, 200), "Hpred");
            drawPred(predictor.lastTailPos, predictor.velTail,
                     cv::Scalar(200, 80, 0), "Tpred");
        }
        drawTitle(canvas, QString("04 head/tail  topo=%1")
                  .arg(Tracking::topologyStateToString(er.topology)));
        drawText(canvas,
            QString("predictor: %1").arg(predictor.hasPrev
                ? (predictor.hasVelocity ? "prev+vel" : "prev only")
                : "none"),
            cv::Point(8, canvas.rows - 10));
        writeStageImage(canvas, outputDir, "04_head_tail.png");
    }

    // Mirror onto blob what detectEndpoints would have written so subsequent
    // stages match Sweep 1 exactly.
    blob.hasCenterlineCutPoint = false;
    blob.tipCandidates.clear();
    for (const Tracking::TrueTip& t : er.tips) {
        Tracking::TipCandidate tc;
        tc.point     = t.point;
        tc.curvature = t.curvature;
        tc.width     = t.width;
        tc.source    = Tracking::TipCandidate::Source::SkeletonEndpoint;
        blob.tipCandidates.push_back(tc);
    }
    blob.assignedHeadTipIdx = er.headIdx;
    blob.assignedTailTipIdx = er.tailIdx;
    blob.topologyState      = er.topology;

    // Stage 05 — Step 2 dispatch (mirrors doWorkNew, same logic).
    std::vector<cv::Point2f> centerline;
    bool hasOverlap = false;
    cv::Point2f overlapCenter(0.f, 0.f);
    QString step2Branch = "none";
    QString step2Detail;

    // Compute refLength here so the dispatch lambda can capture it.
    float refLength = 0.f;
    {
        const Tracking::TipFeatureBaseline bl = storage->getTipBaseline(wormId);
        refLength = bl.meanBodyLength;
    }

    // Skeleton-arc helper: finds both arcs from known tip to target, picks
    // by RHR then length. Mutates blob (adds hypothesised tip for D-3).
    auto exportSkeletonArcDispatch =
        [&](const Tracking::DetectedBlob& dispBlob,
            const Tracking::EndpointResult& dispEr,
            const cv::Point2f& dispOrigin,
            QString& branchOut, QString& detailOut) -> bool {
        const int hIdx = dispBlob.assignedHeadTipIdx;
        const int tIdx = dispBlob.assignedTailTipIdx;
        const bool hasHead = (hIdx >= 0 && hIdx < (int)dispBlob.tipCandidates.size());
        const bool hasTail = (tIdx >= 0 && tIdx < (int)dispBlob.tipCandidates.size());
        if (!hasHead && !hasTail) return false;

        const int knownIdx = hasHead ? hIdx : tIdx;
        const cv::Point2f knownPos = dispBlob.tipCandidates[knownIdx].point;
        const int srcNode = nearestSkeletonNode(dispEr.skeleton, knownPos, dispOrigin);
        if (srcNode < 0) return false;

        const bool hasActualTarget = hasHead && hasTail;
        cv::Point2f targetPos(-1.f, -1.f);
        if (hasActualTarget) {
            targetPos = dispBlob.tipCandidates[hasHead ? tIdx : hIdx].point;
            branchOut = "SelfCrossed / skeleton-arc RHR (D-2 two tips)";
        } else {
            branchOut = "SelfCrossed / skeleton-arc RHR (D-3 one tip)";
            const bool hiddenIsHead = !hasHead;
            if (predictor.hasPrev) {
                const cv::Point2f& last = hiddenIsHead
                    ? predictor.lastHeadPos : predictor.lastTailPos;
                const cv::Point2f& vel  = hiddenIsHead
                    ? predictor.velHead    : predictor.velTail;
                if (last.x != 0.f || last.y != 0.f)
                    targetPos = predictor.hasVelocity ? (last + vel) : last;
            }
        }

        int goalNode = -1;
        if (targetPos.x != -1.f || targetPos.y != -1.f) {
            goalNode = nearestSkeletonNode(dispEr.skeleton, targetPos, dispOrigin);
            detailOut = QString("target=(%1,%2)").arg(targetPos.x,'f',1).arg(targetPos.y,'f',1);
        } else {
            goalNode = farthestSkeletonNode(dispEr.skeleton, srcNode);
            detailOut = "target=farthest skeleton node (no predictor)";
        }
        if (goalNode < 0 || goalNode == srcNode) return false;

        std::vector<cv::Point2f> arcA, arcB, chosen;
        if (skeletonBothArcs(dispEr.skeleton, srcNode, goalNode,
                             dispOrigin, arcA, arcB)) {
            chosen = pickArcByRHR(arcA, arcB, prevTurningAngle,
                                   angleThreshold, refLength);
            detailOut += QString("  arcA_len=%1  arcB_len=%2  prevTurning=%3")
                         .arg(arcLen(arcA),'f',1)
                         .arg(arcLen(arcB),'f',1)
                         .arg(prevTurningAngle,'f',3);
        } else if (!arcA.empty()) {
            chosen = arcA;
            detailOut += "  single arc (open curve)";
        } else {
            return false;
        }
        if (chosen.size() < 2) return false;

        if (hasHead && dispEr.tips[hIdx].extended) chosen.front() = dispEr.tips[hIdx].point;
        if (hasTail && dispEr.tips[tIdx].extended) chosen.back()  = dispEr.tips[tIdx].point;

        centerline = std::move(chosen);

        if (!hasActualTarget) {
            Tracking::TipCandidate hyp;
            hyp.point  = centerline.back();
            hyp.source = Tracking::TipCandidate::Source::HypothesizedHidden;
            blob.tipCandidates.push_back(hyp);
            const int newIdx = (int)blob.tipCandidates.size() - 1;
            if (hasHead) blob.assignedTailTipIdx = newIdx;
            else         blob.assignedHeadTipIdx = newIdx;
            detailOut += QString("  hiddenTip=(%1,%2)")
                         .arg(centerline.back().x,'f',1)
                         .arg(centerline.back().y,'f',1);
        }
        return true;
    };

    if (er.topology == Tracking::TopologyState::Clean &&
        blob.assignedHeadTipIdx >= 0 && blob.assignedTailTipIdx >= 0 &&
        (int)er.skeleton.endpointIndices.size() >
            std::max(blob.assignedHeadTipIdx, blob.assignedTailTipIdx)) {
        const int hGraphIdx = er.skeleton.endpointIndices[blob.assignedHeadTipIdx];
        const int tGraphIdx = er.skeleton.endpointIndices[blob.assignedTailTipIdx];
        std::vector<cv::Point2f> graphPath;
        if (skeletonGraphPath(er.skeleton, hGraphIdx, tGraphIdx,
                              originOffset, graphPath)) {
            centerline = std::move(graphPath);
            if (er.tips[blob.assignedHeadTipIdx].extended && !centerline.empty())
                centerline.front() = er.tips[blob.assignedHeadTipIdx].point;
            if (er.tips[blob.assignedTailTipIdx].extended && !centerline.empty())
                centerline.back()  = er.tips[blob.assignedTailTipIdx].point;
            const float d1Len = arcLen(centerline);
            step2Branch = "Clean / skeleton-graph path (D-1)";
            step2Detail = QString("path=%1pts arcLen=%2")
                          .arg(centerline.size()).arg(d1Len,'f',1);

            if (refLength > 0.f && d1Len < 0.5f * refLength) {
                step2Detail += QString("  [SHORT refLen=%1 ratio=%2]")
                               .arg(refLength,'f',1).arg(d1Len/refLength,'f',2);
                // Punch synthetic hole, re-detect, retry as SelfCrossed.
                Tracking::DetectedBlob holeBlob;
                if (addSyntheticHoleAtDTMax(blob, er.distTransform,
                                             er.localBounds, holeBlob)) {
                    const Tracking::EndpointResult er2 =
                        Tracking::detectEndpoints(holeBlob, predictor, baseline, inMerge);
                    const cv::Point2f origin2(static_cast<float>(er2.localBounds.x),
                                              static_cast<float>(er2.localBounds.y));
                    std::vector<cv::Point2f> prevCl = centerline;
                    Tracking::DetectedBlob savedBlob = blob;
                    blob = holeBlob;
                    blob.tipCandidates.clear();
                    for (const Tracking::TrueTip& t : er2.tips) {
                        Tracking::TipCandidate tc;
                        tc.point = t.point; tc.curvature = t.curvature;
                        tc.width = t.width;
                        tc.source = Tracking::TipCandidate::Source::SkeletonEndpoint;
                        blob.tipCandidates.push_back(tc);
                    }
                    blob.assignedHeadTipIdx = er2.headIdx;
                    blob.assignedTailTipIdx = er2.tailIdx;
                    blob.topologyState      = er2.topology;
                    centerline.clear();
                    QString hb, hd;
                    if (!exportSkeletonArcDispatch(blob, er2, origin2, hb, hd)) {
                        blob = savedBlob;
                        centerline = prevCl;
                        step2Detail += "  synth-hole dispatch FAILED";
                    } else {
                        step2Branch = "D-1 too-short → synth-hole + " + hb;
                        step2Detail += "  hole:" + hd;
                    }
                }
            }
        } else {
            step2Branch = "Clean / skeleton-graph FAILED";
        }
    }
    else if (er.topology == Tracking::TopologyState::SelfCrossed) {
        const int hIdx = blob.assignedHeadTipIdx;
        const int tIdx = blob.assignedTailTipIdx;
        const bool hasHead = (hIdx >= 0 && hIdx < (int)blob.tipCandidates.size());
        const bool hasTail = (tIdx >= 0 && tIdx < (int)blob.tipCandidates.size());

        if (hasHead || hasTail) {
            exportSkeletonArcDispatch(blob, er, originOffset, step2Branch, step2Detail);
        }
        else if (!blob.holeContourPoints.empty()) {
            step2Branch = "SelfCrossed / synthetic hole-axis cut (0-tip ring)";
            const std::vector<cv::Point>& hole = blob.holeContourPoints.front();
            if (hole.size() >= 5) {
                cv::RotatedRect rr = cv::fitEllipse(hole);
                const float ang = static_cast<float>(rr.angle * CV_PI / 180.0);
                const float halfMajor = std::max(rr.size.width, rr.size.height) * 0.5f + 6.f;
                const cv::Point2f dir(std::cos(ang), std::sin(ang));
                const cv::Point2f cutA(rr.center.x - dir.x * halfMajor,
                                       rr.center.y - dir.y * halfMajor);
                const cv::Point2f cutB(rr.center.x + dir.x * halfMajor,
                                       rr.center.y + dir.y * halfMajor);
                Tracking::DetectedBlob cutBlob = blob;
                if (Tracking::populateCenterlineFromContourWithCut(cutBlob, cutA, cutB, 3) &&
                    cutBlob.centerlinePoints.size() >= 2) {
                    centerline.assign(cutBlob.centerlinePoints.begin(),
                                      cutBlob.centerlinePoints.end());
                    blob.centerlineCutPoint = (cutA + cutB) * 0.5f;
                    blob.hasCenterlineCutPoint = true;
                    step2Detail = QString("cut center=(%1,%2)")
                        .arg(blob.centerlineCutPoint.x,'f',1)
                        .arg(blob.centerlineCutPoint.y,'f',1);
                } else {
                    step2Detail = "populateCenterlineFromContourWithCut FAILED";
                }
            }
        }
    }

    if (centerline.empty()) {
        step2Branch += " · fallback D-4 (legacy contour-skeleton)";
        Tracking::DetectedBlob fallback = blob;
        if (Tracking::populateCenterlineFromContour(fallback) &&
            fallback.centerlinePoints.size() >= 2) {
            centerline.assign(fallback.centerlinePoints.begin(),
                              fallback.centerlinePoints.end());
        }
    }

    log << "Stage 05 (Step 2 dispatch):\n";
    log << "  branch = " << step2Branch << "\n";
    log << "  detail = " << step2Detail << "\n";
    log << "  prevTurningAngle = " << prevTurningAngle
        << " rad (thresh=" << angleThreshold << ")\n";
    log << "  refLength (baseline mean) = " << refLength << " px\n";
    log << "  initial centerline points = " << centerline.size() << "\n\n";

    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        if (centerline.size() >= 2) {
            std::vector<cv::Point> pts;
            pts.reserve(centerline.size());
            for (const auto& p : centerline) pts.push_back(worldToCanvas(p, bounds));
            cv::polylines(canvas, std::vector<std::vector<cv::Point>>{pts},
                          false, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
            for (size_t i = 0; i < pts.size(); ++i)
                cv::circle(canvas, pts[i], 2, cv::Scalar(0, 200, 255), cv::FILLED);
            cv::circle(canvas, pts.front(), 6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            cv::circle(canvas, pts.back(),  6, cv::Scalar(255, 80, 0), 2, cv::LINE_AA);
        }
        if (blob.hasCenterlineCutPoint) {
            cv::drawMarker(canvas, worldToCanvas(blob.centerlineCutPoint, bounds),
                           cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS,
                           14, 2, cv::LINE_AA);
        }
        drawTitle(canvas, QString("05 initial centerline  %1").arg(step2Branch));
        writeStageImage(canvas, outputDir, "05_initial_centerline.png");
    }

    auto flushLog = [&]() {
        QFile f(QDir(outputDir).absoluteFilePath("log.txt"));
        if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
            f.write(logBuf.toUtf8());
    };

    if (centerline.size() < 2) {
        log << "ABORT: no centerline producible.\n";
        flushLog();
        if (outErrorMsg) *outErrorMsg = "no centerline producible (see log.txt)";
        return true;
    }

    // Stage 06 — resample.
    if ((int)centerline.size() != nPts)
        centerline = resample(centerline, nPts);
    log << "Stage 06 (resample): n=" << centerline.size()
        << "  arcLen=" << arcLen(centerline) << " px\n\n";
    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        std::vector<cv::Point> pts;
        for (const auto& p : centerline) pts.push_back(worldToCanvas(p, bounds));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{pts},
                      false, cv::Scalar(200, 200, 255), 2, cv::LINE_AA);
        for (const auto& p : pts) cv::circle(canvas, p, 3, cv::Scalar(255, 255, 0), cv::FILLED);
        cv::circle(canvas, pts.front(), 7, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::circle(canvas, pts.back(),  7, cv::Scalar(255, 80, 0), 2, cv::LINE_AA);
        drawTitle(canvas, QString("06 resampled to %1 points").arg(nPts));
        writeStageImage(canvas, outputDir, "06_resampled.png");
    }

    // Stage 07 — snake refinement (Clean only).
    bool snakeRan = false;
    if (er.topology == Tracking::TopologyState::Clean) {
        cv::Mat mask;
        cv::Rect sbBounds;
        if (buildSnakeMask(blob, mask, sbBounds)) {
            const cv::Point2f pinH =
                (blob.assignedHeadTipIdx >= 0 &&
                 blob.assignedHeadTipIdx < (int)blob.tipCandidates.size())
                ? blob.tipCandidates[blob.assignedHeadTipIdx].point
                : centerline.front();
            const cv::Point2f pinT =
                (blob.assignedTailTipIdx >= 0 &&
                 blob.assignedTailTipIdx < (int)blob.tipCandidates.size())
                ? blob.tipCandidates[blob.assignedTailTipIdx].point
                : centerline.back();
            cv::Point2f tmpOverlap(0.f, 0.f);
            bool tmpHasOverlap = false;
            refineSnakeCore(blob, mask, sbBounds, centerline, pinH, pinT,
                            nPts, snakeParams, tmpOverlap, tmpHasOverlap);
            snakeRan = true;
            if (tmpHasOverlap) { overlapCenter = tmpOverlap; hasOverlap = true; }
        }
    }
    log << "Stage 07 (snake refinement): ran="
        << (snakeRan ? "Y" : "N (skipped — SelfCrossed already refined or snake disabled)")
        << "  hasOverlap=" << (hasOverlap ? "Y" : "N") << "\n\n";
    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        std::vector<cv::Point> pts;
        for (const auto& p : centerline) pts.push_back(worldToCanvas(p, bounds));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{pts},
                      false, cv::Scalar(80, 255, 255), 2, cv::LINE_AA);
        for (const auto& p : pts) cv::circle(canvas, p, 2, cv::Scalar(80, 255, 255), cv::FILLED);
        cv::circle(canvas, pts.front(), 7, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::circle(canvas, pts.back(),  7, cv::Scalar(255, 80, 0), 2, cv::LINE_AA);
        if (hasOverlap) {
            cv::drawMarker(canvas, worldToCanvas(overlapCenter, bounds),
                           cv::Scalar(0, 0, 255), cv::MARKER_STAR, 14, 2, cv::LINE_AA);
            drawText(canvas, "overlap detected",
                     worldToCanvas(overlapCenter, bounds) + cv::Point(10, 0),
                     cv::Scalar(0, 0, 255));
        }
        drawTitle(canvas, QString("07 snake refined  %1")
                  .arg(snakeRan ? "Clean -> refineSnakeCore"
                                : "skipped (SelfCrossed uses skeleton arc, not snake)"));
        writeStageImage(canvas, outputDir, "07_snake_refined.png");
    }

    // Stage 08 — RHR turning-angle check (informational only in export mode).
    const float curTurning = totalSignedTurningAngle(centerline);
    log << "Stage 08 (RHR turning-angle check):\n"
        << "  turning angle = " << curTurning << " rad ("
        << (curTurning * 180.0f / float(CV_PI)) << " deg)\n"
        << "  threshold = " << snakeParams.orientationAngleThreshold << " rad\n"
        << "  no previous-frame sweep state in export mode -> no flip applied.\n\n";

    // Stage 09 — final centerline + head/tail markers.
    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        std::vector<cv::Point> pts;
        for (const auto& p : centerline) pts.push_back(worldToCanvas(p, bounds));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{pts},
                      false, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::circle(canvas, pts.front(), 9, cv::Scalar(0, 0, 255), cv::FILLED);
        cv::circle(canvas, pts.front(), 9, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        cv::circle(canvas, pts.back(),  9, cv::Scalar(255, 80, 0), cv::FILLED);
        cv::circle(canvas, pts.back(),  9, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        drawText(canvas, "H", pts.front() + cv::Point(-4, 5));
        drawText(canvas, "T", pts.back()  + cv::Point(-4, 5));
        drawTitle(canvas, QString("09 final  arcLen=%1 px  turning=%2 rad")
                  .arg(arcLen(centerline), 0, 'f', 1)
                  .arg(curTurning, 0, 'f', 3));
        writeStageImage(canvas, outputDir, "09_final.png");
    }

    log << "Stage 09 (final centerline):\n"
        << "  arcLength = " << arcLen(centerline) << " px\n"
        << "  n points  = " << centerline.size() << "\n"
        << "  head world = (" << centerline.front().x << "," << centerline.front().y << ")\n"
        << "  tail world = (" << centerline.back().x  << "," << centerline.back().y << ")\n";

    // Stored result comparison.
    log << "\n--- Stored result comparison ---\n";
    const Tracking::DetectedBlob& stored = frameBlobs[wormId];
    log << "stored centerline points = " << stored.centerlinePoints.size() << "\n";
    log << "stored topologyState     = "
        << Tracking::topologyStateToString(stored.topologyState) << "\n";
    log << "stored head/tail tipIdx  = " << stored.assignedHeadTipIdx
        << " / " << stored.assignedTailTipIdx << "\n";
    if (!stored.centerlinePoints.empty()) {
        const auto sh = stored.centerlinePoints.front();
        const auto st = stored.centerlinePoints.back();
        log << "stored head world = (" << sh.x << "," << sh.y << ")\n";
        log << "stored tail world = (" << st.x << "," << st.y << ")\n";
    }

    flushLog();

    YAWT_INFO(lcCoreCenterlineWorker)
        << QString("exportProcessForFrame: worm %1 frame %2 -> %3")
            .arg(wormId).arg(frameNumber).arg(outputDir);
    return true;
}
