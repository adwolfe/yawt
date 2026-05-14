#include "centerlineworker.h"
#include "../debug/debugdatastore.h"
#include "../debug/debugrecords.h"
#include "../data/trackingcommon.h"
#include "../utils/debugutils.h"
#include "../utils/loggingcategories.h"
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
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
    // Sum of consecutive 2D segment cross products along the centerline.
    // Positive means counterclockwise, negative means clockwise, and the sign
    // flips when traversal direction is reversed.
    float turningAngle = 0.f;

    cv::Point2f nose()     const { return points.front(); }
    cv::Point2f tail()     const { return points.back();  }
    cv::Point2f midpoint() const { return points[points.size() / 2]; }
};

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

static cv::Point2f blobCentroid(const Tracking::DetectedBlob& blob)
{
    return cv::Point2f(static_cast<float>(blob.centroid.x()),
                       static_cast<float>(blob.centroid.y()));
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

static bool enforceSelfCrossedTwoTipPredictorRoles(
    Tracking::DetectedBlob& blob,
    const Tracking::HeadTailPredictor& predictor,
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
// whose consecutive-segment cross-sum sign matches the previous frame's
// right-hand-rule state, falling back to arc-length proximity to refLength
// when no previous state is available.

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
// Primary selector: the arc whose consecutive-segment cross-sum matches the
// sign of the previous frame's cross-sum. Fallback when the previous cross-sum
// is too close to zero or both/neither match: pick the arc whose arcLen is
// closer to refLength.
// Last resort: return arcA.
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

static std::vector<cv::Point2f> nodesToWorldPath(const Tracking::SkeletonGraph& graph,
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

static float nodePathLength(const Tracking::SkeletonGraph& graph,
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

static bool shortestNodePath(const Tracking::SkeletonGraph& graph,
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

static int nearestReachableJunction(const Tracking::SkeletonGraph& graph,
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

static bool isJunctionNode(const Tracking::SkeletonGraph& graph, int idx)
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

static std::vector<JunctionCluster> findJunctionClusters(const Tracking::SkeletonGraph& graph)
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

static std::vector<JunctionPort> clusterPorts(const Tracking::SkeletonGraph& graph,
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

static bool shortestPathToCluster(const Tracking::SkeletonGraph& graph,
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

static std::vector<int> clusterInternalPath(const Tracking::SkeletonGraph& graph,
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

static bool findLoopReturnPath(const Tracking::SkeletonGraph& graph,
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

static float minPathDistanceToPoint(const Tracking::SkeletonGraph& graph,
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

static JunctionSelection selectLoopAwareJunction(const Tracking::SkeletonGraph& graph,
                                                 int srcIdx,
                                                 const cv::Point2f& predictedHidden,
                                                 const cv::Point2f& originOffset,
                                                 float refLength)
{
    JunctionSelection selection;
    const std::vector<JunctionCluster> clusters = findJunctionClusters(graph);
    selection.clusterCount = static_cast<int>(clusters.size());

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

        const float trunkLen = nodePathLength(graph, trunk);
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

static std::vector<int> traceSkeletonBranch(const Tracking::SkeletonGraph& graph,
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

static std::vector<int> traceSkeletonBranchFromCluster(const Tracking::SkeletonGraph& graph,
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
static bool skeletonPathTowardPredictedHidden(const Tracking::SkeletonGraph& graph,
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

static void captureEndpointDebug(const Tracking::EndpointResult& er,
                                 Debug::CenterlineFrameDebug& record)
{
    record.endpointLocalBounds = er.localBounds;
    record.skeletonPixels.clear();
    record.skeletonEndpointPoints.clear();

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

// ── CenterlineWorker ────────────────────────────────────────────────────────

CenterlineWorker::CenterlineWorker(TrackingDataStorage* storage,
                                   Debug::DebugDataStore* debugStore,
                                   QObject* parent)
    : QObject(parent), m_storage(storage), m_debugStore(debugStore) {}

void CenterlineWorker::setSnakeParams(const Tracking::CenterlineSnakeParams& params)
{
    m_snakeParams = params;
}

// 2-sweep / 5-step pipeline. See CENTERLINE_REWRITE_PLAN.md for the
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
//                       SC       → skeleton-arc dispatch with optional
//                                  HypothesizedHidden tip candidate.
//                       fallback → populateCenterlineFromContour (D-4).
//             Step 3: resample to nPoints.
//             Step 4: snake refinement (Clean only); right-hand-rule veto.
//             Step 5: predictor update for next frame.
void CenterlineWorker::doWork()
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

        auto loadPreviousFrameContext =
            [&](int frameNumber, int step,
                Tracking::HeadTailPredictor& outPredictor,
                CenterlineState& outPrevState,
                float& outLocalRefLength) -> bool {
            auto blobAt = [&](int f, Tracking::DetectedBlob& out) -> bool {
                const QMap<int, Tracking::DetectedBlob> blobs =
                    m_storage->getDetectedBlobsForFrame(f);
                if (!blobs.contains(wormId)) return false;
                out = blobs[wormId];
                return out.isValid && !out.contourPoints.empty();
            };

            Tracking::DetectedBlob prevBlob;
            if (!blobAt(frameNumber - step, prevBlob)) {
                return false;
            }

            outPredictor = Tracking::HeadTailPredictor{};
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
            if (blobAt(frameNumber - (2 * step), prevPrevBlob)) {
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

        // ── per-frame body of Sweep 1 ────────────────────────────────────
        // `predictor` and `prevState` are CARRIED across frames within one
        // direction; they reset on Lost frames and at the keyframe boundary
        // between forward and backward passes.
        auto processFrame = [&](int i, int step,
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
            Tracking::HeadTailPredictor framePredictor = predictor;
            CenterlineState framePrevState = prevState;
            float frameRefLength = refLength;
            if (!isKeyframeBootstrap) {
                float localRefLength = frameRefLength;
                if (loadPreviousFrameContext(tp.frameNumberOriginal, step,
                                             framePredictor,
                                             framePrevState,
                                             localRefLength)) {
                    if (localRefLength > 0.f) {
                        frameRefLength = localRefLength;
                    }
                }
            }

            // ── STEP 1: detect endpoints, write back tip data ───────────
            const Tracking::TipFeatureBaseline baseline =
                m_storage->getTipBaseline(wormId);
            const bool captureDebug =
                m_debugStore && DebugUtils::isDebugCaptureEnabled();
            Debug::CenterlineFrameDebug debugRecord;
            debugRecord.wormId = wormId;
            debugRecord.frameNumber = tp.frameNumberOriginal;
            debugRecord.directionStep = step;
            debugRecord.keyframeBootstrap = isKeyframeBootstrap;
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

            Tracking::EndpointResult er = Tracking::detectEndpoints(
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
                                Tracking::EndpointResult splitEr =
                                    Tracking::detectEndpoints(splitBlob, framePredictor, baseline, inMerge);

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
                        m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal - (2 * step));
                    const bool prevPrevSelfCrossed =
                        prevPrevBlobs.contains(wormId) &&
                        prevPrevBlobs[wormId].topologyState == Tracking::TopologyState::SelfCrossed;
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
                                const Tracking::TrueTip& tip = dispEr.tips[tipIdx];
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
                                                                           nPts,
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
                    if (frameRefLength > 0.f &&
                        arcLen(centerline) < 0.5f * frameRefLength) {
                        Tracking::DetectedBlob holeBlob;
                        if (addSyntheticHoleAtDTMax(blob, er.distTransform,
                                                     er.localBounds, holeBlob)) {
                            debugRecord.syntheticHoleUsed = true;
                            debugRecord.decisions << QStringLiteral("D-1 path was short; synthetic hole retry attempted");
                            const Tracking::EndpointResult er2 =
                                Tracking::detectEndpoints(
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
                    runSkeletonArcDispatch(blob, er, originOffset);
                }
                else if (!blob.holeContourPoints.empty()) {
                    // 0 tips, closed ring: synthetic-hole-axis cut.
                    debugRecord.branch = Debug::CenterlineBranch::ZeroTipRingCut;
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
                            debugRecord.decisions << QStringLiteral("0-tip ring cut supplied centerline");
                        }
                    }
                }
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
                m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal,
                                                   wormId, blob);
                debugRecord.decisions << QStringLiteral("no centerline producible; persisted endpoint/debug blob state only");
                if (captureDebug) {
                    m_debugStore->setCenterlineFrame(debugRecord);
                }
                prevState.valid = false;
                return;
            }

            // ── STEP 3: resample to nPoints ─────────────────────────────
            if (static_cast<int>(centerline.size()) != nPts)
                centerline = resample(centerline, nPts);
            debugRecord.resampledCenterline = centerline;

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
            if (isKeyframeBootstrap && centerline.size() >= 2 &&
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

            m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal,
                                               wormId, blob);

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
                m_debugStore->setCenterlineFrame(debugRecord);
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
        };

        // ── Sweep 1 entry: process keyframe, then propagate outward ────
                Tracking::HeadTailPredictor seedPredictor;
                CenterlineState seedState;
                processFrame(keyframeIdx, 1, seedPredictor, seedState,
                             /*isKeyframeBootstrap=*/true);

                // Forward pass.
                {
                    Tracking::HeadTailPredictor predictor = seedPredictor;
                    CenterlineState prevState = seedState;
                    for (int i = keyframeIdx + 1;
                         i < static_cast<int>(sortedPoints.size()); ++i) {
                        processFrame(i, 1, predictor, prevState,
                                     /*isKeyframeBootstrap=*/false);
                    }
                }

                // Backward pass.
                {
                    Tracking::HeadTailPredictor predictor = seedPredictor;
                    CenterlineState prevState = seedState;
                    for (int i = keyframeIdx - 1; i >= 0; --i) {
                        processFrame(i, -1, predictor, prevState,
                                     /*isKeyframeBootstrap=*/false);
                    }
                }

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}

// ── Diagnostic export: per-stage images + decision log ─────────────────────
//
// Re-runs the per-frame centerline pipeline (same dispatch as Sweep 1's
// processFrame in doWork) for one (wormId, frameNumber), writing an image
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
	    if (adj.centerlinePoints.size() >= 2) {
	        predictor.lastCenterPos = adj.centerlinePoints[adj.centerlinePoints.size() / 2];
	    }
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
	        const bool haveCenterVelocity =
	            adj2.centerlinePoints.size() >= 2 && adj.centerlinePoints.size() >= 2;
	        if (haveCenterVelocity) {
	            const cv::Point2f c2 = adj2.centerlinePoints[adj2.centerlinePoints.size() / 2];
	            const cv::Point2f c1 = adj.centerlinePoints[adj.centerlinePoints.size() / 2];
	            predictor.velCenter = c1 - c2;
	        }
	        predictor.hasVelocity = (h2.x >= 0 && adjHead.x >= 0) ||
	                                (t2.x >= 0 && adjTail.x >= 0) ||
	                                haveCenterVelocity;
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
    Tracking::DetectedBlob previousBlobForExport;
    Tracking::DetectedBlob previousPreviousBlobForExport;
    const auto previousBlobsForExport =
        storage->getDetectedBlobsForFrame(frameNumber - 1);
    const bool hasPreviousBlobForExport =
        previousBlobsForExport.contains(wormId) &&
        previousBlobsForExport[wormId].isValid &&
        !previousBlobsForExport[wormId].contourPoints.empty();
    if (hasPreviousBlobForExport) {
        previousBlobForExport = previousBlobsForExport[wormId];
    }
    const auto previousPreviousBlobsForExport =
        storage->getDetectedBlobsForFrame(frameNumber - 2);
    const bool hasPreviousPreviousBlobForExport =
        previousPreviousBlobsForExport.contains(wormId) &&
        previousPreviousBlobsForExport[wormId].isValid &&
        !previousPreviousBlobsForExport[wormId].contourPoints.empty();
    if (hasPreviousPreviousBlobForExport) {
        previousPreviousBlobForExport = previousPreviousBlobsForExport[wormId];
    }
    const cv::Point2f predictedHead = predictor.hasVelocity
        ? predictor.lastHeadPos + predictor.velHead
        : predictor.lastHeadPos;
    const cv::Point2f predictedTail = predictor.hasVelocity
        ? predictor.lastTailPos + predictor.velTail
        : predictor.lastTailPos;
    const bool hasPredictedCenter =
        predictor.lastCenterPos.x != 0.f || predictor.lastCenterPos.y != 0.f;
    cv::Point2f predictedCenter = predictor.hasVelocity
        ? predictor.lastCenterPos + predictor.velCenter
        : predictor.lastCenterPos;
    cv::Point2f snappedHead = predictedHead;
    cv::Point2f snappedTail = predictedTail;
    cv::Point2f snappedCenter = predictedCenter;
    nearestMaskPoint(blob, predictedHead, snappedHead);
    nearestMaskPoint(blob, predictedTail, snappedTail);
    if (hasPredictedCenter) {
        nearestMaskPoint(blob, predictedCenter, snappedCenter);
        predictedCenter = snappedCenter;
    }

    const HiddenTipTarget hiddenHeadTarget = predictHiddenTipTarget(
        blob,
        hasPreviousBlobForExport ? &previousBlobForExport : nullptr,
        predictor.lastHeadPos,
        predictor.velHead,
        predictor.hasVelocity);
    const HiddenTipTarget hiddenTailTarget = predictHiddenTipTarget(
        blob,
        hasPreviousBlobForExport ? &previousBlobForExport : nullptr,
        predictor.lastTailPos,
        predictor.velTail,
        predictor.hasVelocity);

    const int nPts = std::max(4, snakeParams.nPoints);
    const float angleThreshold = static_cast<float>(
        std::max(0.0, snakeParams.orientationAngleThreshold));

    // Recover the previous frame's cross-sum from its stored centerline
    // so the RHR arc-picker in Stage 05 behaves like the live sweep.
    float prevTurningAngle = 0.f;
    float prevFrameArcLength = 0.f;
    std::vector<cv::Point2f> previousCenterlineForExport;
    {
        const int adjFrame = frameNumber - 1;
        const auto adjBlobs = storage->getDetectedBlobsForFrame(adjFrame);
        if (adjBlobs.contains(wormId)) {
            const auto& pts = adjBlobs[wormId].centerlinePoints;
            if (pts.size() >= 2) {
                std::vector<cv::Point2f> v(pts.begin(), pts.end());
                prevTurningAngle = centerlineCrossSum(v);
                prevFrameArcLength = arcLen(v);
                previousCenterlineForExport = v;
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
        << " lastCenter=(" << predictor.lastCenterPos.x << "," << predictor.lastCenterPos.y << ")"
        << " velHead=(" << predictor.velHead.x << "," << predictor.velHead.y << ")"
        << " velTail=(" << predictor.velTail.x << "," << predictor.velTail.y << ")"
        << " velCenter=(" << predictor.velCenter.x << "," << predictor.velCenter.y << ")\n";
    log << "predicted: Hpred=(" << predictedHead.x << "," << predictedHead.y << ")"
        << " Tpred=(" << predictedTail.x << "," << predictedTail.y << ")"
        << " Cpred=(" << predictedCenter.x << "," << predictedCenter.y << ")\n";
    log << "snapped: Hmask=(" << snappedHead.x << "," << snappedHead.y << ")"
        << " Tmask=(" << snappedTail.x << "," << snappedTail.y << ")"
        << " Cmask=(" << snappedCenter.x << "," << snappedCenter.y << ")\n";
    if (hiddenHeadTarget.hasTarget) {
        log << "hiddenHeadTarget: blended=(" << hiddenHeadTarget.target.x << ","
            << hiddenHeadTarget.target.y << ") vel=(" << hiddenHeadTarget.velocityTarget.x
            << "," << hiddenHeadTarget.velocityTarget.y << ") maskCue="
            << (hiddenHeadTarget.hasMaskCue ? "Y" : "N");
        if (hiddenHeadTarget.hasMaskCue) {
            log << " (" << hiddenHeadTarget.maskCue.x << "," << hiddenHeadTarget.maskCue.y << ")";
        }
        log << "\n";
    }
    if (hiddenTailTarget.hasTarget) {
        log << "hiddenTailTarget: blended=(" << hiddenTailTarget.target.x << ","
            << hiddenTailTarget.target.y << ") vel=(" << hiddenTailTarget.velocityTarget.x
            << "," << hiddenTailTarget.velocityTarget.y << ") maskCue="
            << (hiddenTailTarget.hasMaskCue ? "Y" : "N");
        if (hiddenTailTarget.hasMaskCue) {
            log << " (" << hiddenTailTarget.maskCue.x << "," << hiddenTailTarget.maskCue.y << ")";
        }
        log << "\n";
    }
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

    // ── OMEGA UNZIPPER START ──────────────────────────────────────────
    if (er.topology == Tracking::TopologyState::SelfCrossed && predictor.hasVelocity) {
        const cv::Point2f predictedHead = predictor.lastHeadPos + predictor.velHead;
        const cv::Point2f predictedTail = predictor.lastTailPos + predictor.velTail;
        float tipDist = ptDist(predictedHead, predictedTail);

        if (baseline.isReliable() && tipDist < baseline.meanWidth * 2.5f) {
            cv::Point2f midPoint = (predictedHead + predictedTail) * 0.5f;
            int localX = static_cast<int>(std::lround(midPoint.x - er.localBounds.x));
            int localY = static_cast<int>(std::lround(midPoint.y - er.localBounds.y));

            if (localX >= 0 && localY >= 0 &&
                localX < er.distTransform.cols && localY < er.distTransform.rows) {

                float localRadius = er.distTransform.at<float>(localY, localX);
                float normalRadius = baseline.meanWidth / 2.0f;

                if (localRadius > normalRadius * 1.4f) {
                    Tracking::DetectedBlob splitBlob = blob;
                    cv::Point2f cutA = predictedHead - cv::Point2f(static_cast<float>(er.localBounds.x), static_cast<float>(er.localBounds.y));
                    cv::Point2f cutB = predictedTail - cv::Point2f(static_cast<float>(er.localBounds.x), static_cast<float>(er.localBounds.y));

                    if (Tracking::populateCenterlineFromContourWithCut(splitBlob, cutA, cutB, 2)) {
                        Tracking::EndpointResult splitEr =
                            Tracking::detectEndpoints(splitBlob, predictor, baseline, inMerge);

                        if (splitEr.topology == Tracking::TopologyState::Clean) {
                            blob = splitBlob;
                            er = splitEr;
                            log << "Omega handle detected and unzipped via predictive DT.\n";
                        }
                    }
                }
            }
        }
    }
    // ── OMEGA UNZIPPER END ────────────────────────────────────────────

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
                                    const cv::Scalar& col, const char* label,
                                    const HiddenTipTarget* hiddenTarget) {
                    if (last.x < 0) return;
                    cv::Point2f pred = predictor.hasVelocity ? (last + vel) : last;
                    cv::Point2f snapped = pred;
                    nearestMaskPoint(blob, pred, snapped);
                    cv::circle(canvas, worldToCanvas(pred, bounds), 5, col, 1, cv::LINE_AA);
                    cv::circle(canvas, worldToCanvas(snapped, bounds), 3, col, cv::FILLED);
                    cv::line(canvas, worldToCanvas(pred, bounds),
                             worldToCanvas(snapped, bounds), col, 1, cv::LINE_AA);
                    drawText(canvas, label,
                             worldToCanvas(snapped, bounds) + cv::Point(7, -4), col);
                    if (hiddenTarget && hiddenTarget->hasMaskCue) {
                        cv::drawMarker(canvas, worldToCanvas(hiddenTarget->maskCue, bounds),
                                       col, cv::MARKER_DIAMOND, 11, 1, cv::LINE_AA);
                        cv::circle(canvas, worldToCanvas(hiddenTarget->target, bounds),
                                   5, col, cv::FILLED, cv::LINE_AA);
                        cv::line(canvas, worldToCanvas(hiddenTarget->maskCue, bounds),
                                 worldToCanvas(hiddenTarget->target, bounds),
                                 col, 1, cv::LINE_AA);
                        drawText(canvas, QString("%1 target").arg(label),
                                 worldToCanvas(hiddenTarget->target, bounds) + cv::Point(7, 10), col);
                    }
                };
            drawPred(predictor.lastHeadPos, predictor.velHead,
                     cv::Scalar(0, 0, 200), "Hpred", &hiddenHeadTarget);
            drawPred(predictor.lastTailPos, predictor.velTail,
                     cv::Scalar(200, 80, 0), "Tpred", &hiddenTailTarget);
            if (hasPredictedCenter) {
                drawPred(predictor.lastCenterPos, predictor.velCenter,
                         cv::Scalar(0, 220, 220), "Cpred", nullptr);
            }
        }
        drawTitle(canvas, QString("04 head/tail  topo=%1")
                  .arg(Tracking::topologyStateToString(er.topology)));
        drawText(canvas,
            QString("predictor: %1").arg(predictor.hasPrev
                ? (predictor.hasVelocity ? "prev+vel+mask" : "prev+mask")
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
    QStringList roleDiagnostics;
    enforceSelfCrossedTwoTipPredictorRoles(blob, predictor, &roleDiagnostics);
    if (!previousCenterlineForExport.empty() && hasPreviousBlobForExport) {
        enforceTwoTipCenterlineOrderRoles(blob,
                                          previousCenterlineForExport,
                                          blobCentroid(previousBlobForExport),
                                          &roleDiagnostics);
    }

    // Stage 05 — Step 2 dispatch (mirrors doWork, same logic).
    std::vector<cv::Point2f> centerline;
    bool hasOverlap = false;
    cv::Point2f overlapCenter(0.f, 0.f);
    QString step2Branch = "none";
    QString step2Detail;

    // Compute refLength here so the dispatch lambda can capture it.
    float refLength = 0.f;
    {
        const Tracking::TipFeatureBaseline bl = storage->getTipBaseline(wormId);
        refLength = prevFrameArcLength > 0.f ? prevFrameArcLength : bl.meanBodyLength;
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
        cv::Point2f centerForRoute = predictedCenter;
        if (hasPredictedCenter) {
            nearestMaskPoint(dispBlob, centerForRoute, centerForRoute);
        }
        if (hasActualTarget) {
            targetPos = dispBlob.tipCandidates[hasHead ? tIdx : hIdx].point;
            branchOut = "SelfCrossed / skeleton-arc RHR (D-2 two tips)";
        } else {
            branchOut = "SelfCrossed / skeleton-arc RHR (D-3 one tip)";
            const bool hiddenIsHead = !hasHead;
            detailOut = hiddenIsHead ? "hiddenRole=head" : "hiddenRole=tail";
            if (predictor.hasPrev) {
                const cv::Point2f& last = hiddenIsHead
                    ? predictor.lastHeadPos : predictor.lastTailPos;
                const cv::Point2f& vel  = hiddenIsHead
                    ? predictor.velHead    : predictor.velTail;
                if (last.x != 0.f || last.y != 0.f) {
                    const HiddenTipTarget predicted = predictHiddenTipTarget(
                        dispBlob,
                        hasPreviousBlobForExport ? &previousBlobForExport : nullptr,
                        last,
                        vel,
                        predictor.hasVelocity);
                    if (predicted.hasTarget) {
                        targetPos = predicted.target;
                    }
                    detailOut += QString("  maskDiffArea=%1 selected=%2")
                                 .arg(predicted.maskDiffArea)
                                 .arg(predicted.selectedMaskDiffArea);
                    if (predicted.hasMaskCue) {
                        detailOut += QString("  maskCue=(%1,%2) velTarget=(%3,%4)")
                                     .arg(predicted.maskCue.x,'f',1)
                                     .arg(predicted.maskCue.y,'f',1)
                                     .arg(predicted.velocityTarget.x,'f',1)
                                     .arg(predicted.velocityTarget.y,'f',1);
                    }
                }
            }
        }

        if (!hasActualTarget && (targetPos.x != -1.f || targetPos.y != -1.f) &&
            hasPreviousBlobForExport &&
            previousBlobForExport.topologyState == Tracking::TopologyState::SelfCrossed &&
            dispEr.topology == Tracking::TopologyState::SelfCrossed) {
            const bool prevPrevSelfCrossed =
                hasPreviousPreviousBlobForExport &&
                previousPreviousBlobForExport.topologyState == Tracking::TopologyState::SelfCrossed;
            if (!prevPrevSelfCrossed) {
                const JunctionSelection junction = selectLoopAwareJunction(
                    dispEr.skeleton, srcNode, targetPos, dispOrigin, refLength);
                if (junction.valid && !junction.fallbackUsed) {
                    const cv::Point& jp = dispEr.skeleton.points[junction.junctionIdx];
                    const cv::Point2f junctionPoint(
                        static_cast<float>(jp.x) + dispOrigin.x,
                        static_cast<float>(jp.y) + dispOrigin.y);
                    const float bumpRadius = refLength > 0.f
                        ? std::max(8.f, 0.35f * refLength)
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
                        const Tracking::TrueTip& tip = dispEr.tips[tipIdx];
                        if (ptDist(tip.point, junctionPoint) <= bumpRadius &&
                            std::abs(tip.curvature) >= curvatureThreshold) {
                            hasSecondBumpNearJunction = true;
                            break;
                        }
                    }
                    detailOut += QString("  secondSelfCrossedJunction=(%1,%2) secondBump=%3")
                                 .arg(junctionPoint.x, 0, 'f', 1)
                                 .arg(junctionPoint.y, 0, 'f', 1)
                                 .arg(hasSecondBumpNearJunction ? "Y" : "N");
                    if (!hasSecondBumpNearJunction) {
                        targetPos = junctionPoint;
                        detailOut += "  hidden target overridden to junction";
                    }
                }
            }
        }

        int goalNode = -1;
        if (targetPos.x != -1.f || targetPos.y != -1.f) {
            goalNode = nearestSkeletonNode(dispEr.skeleton, targetPos, dispOrigin);
            if (!detailOut.isEmpty()) detailOut += "  ";
            detailOut += QString("target=(%1,%2)").arg(targetPos.x,'f',1).arg(targetPos.y,'f',1);
            if (hasPredictedCenter) {
                detailOut += QString("  center=(%1,%2)")
                             .arg(centerForRoute.x,'f',1)
                             .arg(centerForRoute.y,'f',1);
            }
        } else {
            goalNode = farthestSkeletonNode(dispEr.skeleton, srcNode);
            if (!detailOut.isEmpty()) detailOut += "  ";
            detailOut += "target=farthest skeleton node (no predictor)";
        }
        if (!hasActualTarget && goalNode == srcNode &&
            dispEr.topology == Tracking::TopologyState::SelfCrossed) {
            const float targetToKnown = ptDist(targetPos, knownPos);
            const bool weakTarget =
                targetToKnown < 0.25f * refLength ||
                targetToKnown < 6.f;
            if (weakTarget) {
                const JunctionSelection junction = selectLoopAwareJunction(
                    dispEr.skeleton, srcNode, targetPos, dispOrigin, refLength);
                if (junction.valid && !junction.fallbackUsed) {
                    const cv::Point& jp = dispEr.skeleton.points[junction.junctionIdx];
                    targetPos = cv::Point2f(static_cast<float>(jp.x) + dispOrigin.x,
                                            static_cast<float>(jp.y) + dispOrigin.y);
                    goalNode = nearestSkeletonNode(dispEr.skeleton, targetPos, dispOrigin);
                    detailOut += QString("  degenerate target near known tip; target overridden to junction=(%1,%2)")
                                 .arg(targetPos.x, 0, 'f', 1)
                                 .arg(targetPos.y, 0, 'f', 1);
                }
            }
        }
        if (goalNode < 0 || goalNode == srcNode) return false;

        std::vector<cv::Point2f> arcA, arcB, chosen;
        if ((targetPos.x != -1.f || targetPos.y != -1.f) &&
            [&]() {
                std::vector<cv::Point2f> previousForRoute = previousCenterlineForExport;
                if (!hasHead) {
                    std::reverse(previousForRoute.begin(), previousForRoute.end());
                }
                QStringList routeDiagnostics;
                const bool ok = skeletonPathTowardPredictedHidden(dispEr.skeleton,
                                                                   srcNode,
                                                                   targetPos,
                                                                   hasActualTarget,
                                                                   hasHead,
                                                                   centerForRoute,
                                                                   hasPredictedCenter,
                                                                   dispOrigin,
                                                                   previousForRoute,
                                                                   prevTurningAngle,
                                                                   angleThreshold,
                                                                   nPts,
                                                                   refLength,
                                                                   &routeDiagnostics,
                                                                   nullptr,
                                                                   chosen);
                if (!routeDiagnostics.isEmpty()) {
                    if (!detailOut.isEmpty()) detailOut += "  ";
                    detailOut += routeDiagnostics.join(QStringLiteral(" | "));
                }
                return ok;
            }()) {
            detailOut += hasActualTarget ? "  whole-path D-2" : "  whole-path D-3";
        } else if (skeletonBothArcs(dispEr.skeleton, srcNode, goalNode,
                                    dispOrigin, arcA, arcB)) {
            chosen = pickArcByRHR(arcA, arcB, prevTurningAngle,
                                   angleThreshold, refLength);
            detailOut += QString("  arcA_len=%1  arcB_len=%2  prevCrossSum=%3")
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
        if (!hasActualTarget && predictor.hasPrev) {
            const cv::Point2f& knownLast = hasHead
                ? predictor.lastHeadPos : predictor.lastTailPos;
            const cv::Point2f& knownVel = hasHead
                ? predictor.velHead : predictor.velTail;
            if (knownLast.x != 0.f || knownLast.y != 0.f) {
                cv::Point2f knownPred = predictor.hasVelocity
                    ? (knownLast + knownVel)
                    : knownLast;
                nearestMaskPoint(dispBlob, knownPred, knownPred);
                chosen.front() = knownPred;
            }
        }

        centerline = std::move(chosen);

        if (!hasActualTarget) {
            const bool havePredictedHidden =
                targetPos.x != -1.f || targetPos.y != -1.f;
            const cv::Point2f hiddenPoint =
                havePredictedHidden ? targetPos : centerline.back();
            centerline.back() = hiddenPoint;
            Tracking::TipCandidate hyp;
            hyp.point  = hiddenPoint;
            hyp.source = Tracking::TipCandidate::Source::HypothesizedHidden;
            blob.tipCandidates.push_back(hyp);
            const int newIdx = (int)blob.tipCandidates.size() - 1;
            if (hasHead) blob.assignedTailTipIdx = newIdx;
            else {
                blob.assignedHeadTipIdx = newIdx;
                std::reverse(centerline.begin(), centerline.end());
            }
            detailOut += QString("  hiddenTip=(%1,%2)")
                         .arg(hiddenPoint.x,'f',1)
                         .arg(hiddenPoint.y,'f',1);
            if (targetPos.x != -1.f || targetPos.y != -1.f) {
                detailOut += QString("  hiddenToPred=%1  pathLen=%2")
                             .arg(ptDist(hiddenPoint, targetPos),'f',1)
                             .arg(arcLen(centerline),'f',1);
            }
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
    for (const QString& line : roleDiagnostics) {
        log << "  role = " << line << "\n";
    }
    log << "  detail = " << step2Detail << "\n";
    log << "  previousCrossSum = " << prevTurningAngle << "\n";
    log << "  refLength = " << refLength << " px ("
        << (prevFrameArcLength > 0.f ? "previous frame arc length" : "baseline mean")
        << ")\n";
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

    // Stage 08 — RHR cross-sum check.
    const float curTurning = centerlineCrossSum(centerline);
    bool exportRhrFlipped = false;
    constexpr float kCrossSumEpsilon = 1e-4f;
    const bool exportAllowRhrFlip =
        step2Branch != QStringLiteral("Clean / skeleton-graph path (D-1)");
    if (exportAllowRhrFlip &&
        std::abs(prevTurningAngle) > kCrossSumEpsilon &&
        std::abs(curTurning) > kCrossSumEpsilon &&
        curTurning * prevTurningAngle < 0.f) {
        std::reverse(centerline.begin(), centerline.end());
        std::swap(blob.assignedHeadTipIdx, blob.assignedTailTipIdx);
        exportRhrFlipped = true;
    }
    const float finalTurning = exportRhrFlipped ? -curTurning : curTurning;
    log << "Stage 08 (RHR cross-sum check):\n"
        << "  crossSum = " << curTurning << "\n"
        << "  threshold = " << kCrossSumEpsilon << "\n"
        << "  previous crossSum = " << prevTurningAngle << "\n"
        << "  allowed = " << (exportAllowRhrFlip ? "Y" : "N (D-1 clean)") << "\n"
        << "  flipped = " << (exportRhrFlipped ? "Y" : "N") << "\n\n";

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
        drawTitle(canvas, QString("09 final  arcLen=%1 px  crossSum=%2")
                  .arg(arcLen(centerline), 0, 'f', 1)
                  .arg(finalTurning, 0, 'f', 3));
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
