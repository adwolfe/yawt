#include "centerlineworker.h"
#include "../data/trackingcommon.h"
#include <QDebug>
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

// Walk the contour from startIdx in direction dir (+1 or -1) until we have
// accumulated targetLength pixels of arc, then return the raw point list.
static std::vector<cv::Point2f> walkContour(const std::vector<cv::Point>& contour,
                                            int startIdx, int dir,
                                            float targetLength)
{
    int n = static_cast<int>(contour.size());
    std::vector<cv::Point2f> pts;
    pts.push_back(cv::Point2f(contour[startIdx].x, contour[startIdx].y));
    float acc = 0.f;
    for (int step = 1; step < n; ++step) {
        int cur  = ((startIdx + dir * step      ) % n + n) % n;
        int prev = ((startIdx + dir * (step - 1)) % n + n) % n;
        float dx = contour[cur].x - contour[prev].x;
        float dy = contour[cur].y - contour[prev].y;
        acc += std::sqrt(dx * dx + dy * dy);
        pts.push_back(cv::Point2f(contour[cur].x, contour[cur].y));
        if (acc >= targetLength) break;
    }
    return pts;
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

// Endpoints/midpoint we carry from one frame to the next.
struct CenterlineState {
    cv::Point2f nose;      // centerlinePoints.front()
    cv::Point2f tail;      // centerlinePoints.back()
    cv::Point2f midpoint;  // centerlinePoints[N/2]
    bool valid = false;
};

// Compute, orient, validate and (if needed) repair the centerline for one frame.
// Returns the resulting CenterlineState (always valid if the function returns
// true; invalid if no usable centerline could be produced).
static bool processOneFrame(Tracking::DetectedBlob& blob,
                            const CenterlineState& prev,
                            float refLength,
                            int nPoints,
                            float minArcFraction,
                            CenterlineState& out)
{
    if (!blob.isValid || blob.contourPoints.empty()) return false;

    if (blob.centerlinePoints.empty())
        Tracking::populateCenterlineFromContour(blob);
    if (blob.centerlinePoints.empty()) return false;

    std::vector<cv::Point2f> pts(blob.centerlinePoints.begin(),
                                 blob.centerlinePoints.end());

    // Orient using prev.nose so that pts.front() is the head end.
    if (prev.valid) {
        if (ptDist(pts.back(), prev.nose) < ptDist(pts.front(), prev.nose))
            std::reverse(pts.begin(), pts.end());
    }

    bool isRing = !blob.holeContourPoints.empty();
    float curLen = arcLen(pts);

    // Fallback: if the skeleton path is too short relative to the worm's known
    // body length, walk the outer contour from the nearest point to the
    // last-known nose.  Try both directions; if we have a previous tail we
    // pick whichever direction's far end is closer to it (preserves head/tail
    // orientation as well as length).
    bool used_fallback = false;
    if (refLength > 0.f && curLen < minArcFraction * refLength) {
        cv::Point2f startHint = prev.valid ? prev.nose : pts.front();
        int startIdx = nearestContourIdx(blob.contourPoints, startHint);
        auto fwd = walkContour(blob.contourPoints, startIdx, +1, refLength);
        auto bwd = walkContour(blob.contourPoints, startIdx, -1, refLength);

        std::vector<cv::Point2f>* best = nullptr;
        if (prev.valid && !fwd.empty() && !bwd.empty()) {
            float dFwd = ptDist(fwd.back(), prev.tail);
            float dBwd = ptDist(bwd.back(), prev.tail);
            best = (dFwd <= dBwd) ? &fwd : &bwd;
        } else {
            best = (arcLen(fwd) >= arcLen(bwd)) ? &fwd : &bwd;
        }
        if (best && arcLen(*best) > curLen) {
            pts = resample(*best, nPoints);
            used_fallback = true;
        }
    }

    if (!used_fallback && static_cast<int>(pts.size()) != nPoints)
        pts = resample(pts, nPoints);

    if (pts.size() < 2) return false;

    blob.centerlinePoints.assign(pts.begin(), pts.end());
    out.nose     = pts.front();
    out.tail     = pts.back();
    out.midpoint = pts[pts.size() / 2];
    out.valid    = true;
    return true;
}

// ── CenterlineWorker ────────────────────────────────────────────────────────

static constexpr int   kCenterlinePoints     = 10;
static constexpr float kMinArcLengthFraction = 0.5f;

CenterlineWorker::CenterlineWorker(TrackingDataStorage* storage, QObject* parent)
    : QObject(parent), m_storage(storage) {}

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
                                    kCenterlinePoints,
                                    kMinArcLengthFraction, out)) {
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
                                    kCenterlinePoints,
                                    kMinArcLengthFraction, out)) {
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
