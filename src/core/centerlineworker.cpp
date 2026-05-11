#include "centerlineworker.h"
#include "../data/trackingcommon.h"
#include <QDebug>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

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
static bool refineCenterlineSnake(const Tracking::DetectedBlob& blob,
                                  const CenterlineState& prev,
                                  float refLength,
                                  int nPoints,
                                  const Tracking::CenterlineSnakeParams& params,
                                  std::vector<cv::Point2f>& outPts,
                                  cv::Point2f& outOverlapCenter,
                                  bool& outHasOverlap)
{
    outHasOverlap = false;
    if (!params.enabled || !prev.valid || prev.points.size() < 4 ||
        blob.contourPoints.empty()) {
        return false;
    }

    // 1. Initial guess: prev centerline translated by centroid motion. This is
    //    the same prediction the existing fallback uses; the snake just refines it.
    const cv::Point2f curCentroid = blobCentroid(blob);
    const cv::Point2f offset = curCentroid - prev.blobCentroid;

    std::vector<cv::Point2f> v;
    v.reserve(prev.points.size());
    for (const cv::Point2f& p : prev.points) {
        v.push_back(p + offset);
    }
    if (static_cast<int>(v.size()) != nPoints) {
        v = resample(v, nPoints);
    }
    if (static_cast<int>(v.size()) < 4) {
        return false;
    }

    // 2. Build a local mask of the current blob (outer contour with holes carved out).
    cv::Rect bounds = cv::boundingRect(blob.contourPoints);
    constexpr int kPad = 8;
    bounds.x -= kPad;
    bounds.y -= kPad;
    bounds.width += 2 * kPad;
    bounds.height += 2 * kPad;
    if (bounds.width <= 1 || bounds.height <= 1) {
        return false;
    }

    cv::Mat mask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    fillBlobMask(mask, blob, bounds, cv::Point2f(0.f, 0.f));
    if (cv::countNonZero(mask) < 4) {
        return false;
    }

    // 3. Distance transform → its ridges ARE the medial axis. Smooth a little so
    //    ∇D is well-defined off-ridge. Sobel gives the gradient field that
    //    attracts the snake toward the ridge (highest-D pixels).
    cv::Mat dt;
    cv::distanceTransform(mask, dt, cv::DIST_L2, 3);
    cv::GaussianBlur(dt, dt, cv::Size(0, 0), 1.0);
    cv::Mat gx;
    cv::Mat gy;
    cv::Sobel(dt, gx, CV_32F, 1, 0, 3);
    cv::Sobel(dt, gy, CV_32F, 0, 1, 3);

    // Normalize gradient magnitude scale so lambda has roughly mask-size-
    // independent meaning. Without this, large blobs have larger ∇D and the
    // user-tuned lambda would behave very differently across frames.
    double gxMin = 0.0;
    double gxMax = 0.0;
    double gyMin = 0.0;
    double gyMax = 0.0;
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
        if (lx < 0 || ly < 0 || lx >= mask.cols || ly >= mask.rows) {
            return false;
        }
        return mask.at<uchar>(ly, lx) != 0;
    };

    // 4. Pin endpoints to the nearest outer-contour point of the predicted
    //    nose/tail. Worm endpoints must lie on the body boundary, not interior.
    v.front() = nearestContourPoint(blob.contourPoints, v.front());
    v.back()  = nearestContourPoint(blob.contourPoints, v.back());

    // 5. Explicit-Euler gradient descent on the discretized energy. With only
    //    nPoints=20 control points this is trivially fast even for many iterations.
    const float alpha = static_cast<float>(params.alpha);
    const float beta  = static_cast<float>(params.beta);
    const float lambda = static_cast<float>(params.lambda);
    const float tau   = static_cast<float>(std::max(1e-3, params.stepSize));
    const int n = static_cast<int>(v.size());

    std::vector<cv::Point2f> next(v.size());
    for (int iter = 0; iter < std::max(1, params.iterations); ++iter) {
        next.front() = v.front();   // pinned
        next.back()  = v.back();    // pinned

        for (int i = 1; i < n - 1; ++i) {
            // Tension: discrete Laplacian (smooths spacing, resists stretching).
            const cv::Point2f tens = v[i - 1] - 2.f * v[i] + v[i + 1];

            // Rigidity: discrete biharmonic (resists bending). Only well-defined
            // two points away from each endpoint; degenerates to zero near pins.
            cv::Point2f rig(0.f, 0.f);
            if (i >= 2 && i <= n - 3) {
                rig = v[i - 2] - 4.f * v[i - 1] + 6.f * v[i] - 4.f * v[i + 1] + v[i + 2];
            }

            // Image: gradient of the (smoothed) distance transform. Pulls the
            // point toward the closest medial-axis ridge.
            const cv::Point2f img = sampleGradient(v[i]);

            // Sign convention for biharmonic is negative — the rigidity force
            // opposes the biharmonic, like the Laplacian opposes itself for tension.
            const cv::Point2f force = alpha * tens - beta * rig + lambda * img;
            cv::Point2f candidate = v[i] + tau * force;

            // Hard constraint: stay inside the blob mask. If the step lands
            // outside, snap back to the closest contour point — better than
            // letting the snake escape into background where ∇D is degenerate.
            if (!isInsideMask(candidate)) {
                candidate = nearestContourPoint(blob.contourPoints, candidate);
            }
            next[i] = candidate;
        }

        v.swap(next);
    }

    // 6. Final resample to the canonical point count (preserves arc-length spacing).
    if (static_cast<int>(v.size()) != nPoints) {
        v = resample(v, nPoints);
    }
    if (v.size() < 2) {
        return false;
    }

    outPts = std::move(v);

    // 7. Detect any self-intersection (debug overlay aid). Reuses the standard
    //    parametric segment-intersection test on non-adjacent segment pairs.
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
    for (int i = 0; i < static_cast<int>(outPts.size()) - 1 && !outHasOverlap; ++i) {
        for (int j = i + 2; j < static_cast<int>(outPts.size()) - 1 && !outHasOverlap; ++j) {
            cv::Point2f x;
            if (segmentsIntersect(outPts[i], outPts[i + 1], outPts[j], outPts[j + 1], x)) {
                outOverlapCenter = x;
                outHasOverlap = true;
            }
        }
    }

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

void CenterlineWorker::doWork()
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

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}
