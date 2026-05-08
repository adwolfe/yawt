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

// Orient pts so front is closer to headHint than back is.
static void orientToHead(std::vector<cv::Point2f>& pts, const cv::Point2f& headHint)
{
    if (pts.size() < 2) return;
    if (ptDist(pts.back(), headHint) < ptDist(pts.front(), headHint))
        std::reverse(pts.begin(), pts.end());
}

// Index of the contour point nearest to target (video coordinates).
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

// Walk the contour (in direction dir = +1 or -1) from startIdx until we have
// accumulated targetLength pixels of arc, then return the raw point list.
static std::vector<cv::Point2f> walkContour(const std::vector<cv::Point>& contour,
                                            int startIdx, int dir,
                                            float targetLength)
{
    int n = static_cast<int>(contour.size());
    std::vector<cv::Point2f> pts;
    pts.push_back(cv::Point2f(contour[startIdx].x,
                              contour[startIdx].y));
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

// Resample a polyline to exactly nPoints evenly spaced points.
static std::vector<cv::Point2f> resample(const std::vector<cv::Point2f>& pts, int nPoints)
{
    if (static_cast<int>(pts.size()) <= nPoints) return pts;
    // Build cumulative arc-length table
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

// ── CenterlineWorker ────────────────────────────────────────────────────────

static constexpr int   kCenterlinePoints    = 10;
// If the skeleton path length is below this fraction of the reference body
// length, the skeletonisation failed to trace the full worm and we fall back
// to contour traversal.
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

        // Sort a local copy of track points by frame so we can process them
        // in temporal order (essential for head/tail continuity).
        std::vector<Tracking::WormTrackPoint> trackPoints = it->second;
        std::sort(trackPoints.begin(), trackPoints.end(),
                  [](const Tracking::WormTrackPoint& a,
                     const Tracking::WormTrackPoint& b) {
                      return a.frameNumberOriginal < b.frameNumberOriginal;
                  });

        // ── Pass 1: compute centerlines and learn reference body length ───
        // Reference length is taken only from single (non-ring) frames because
        // the skeleton is most reliable there.  Ring frames are deliberately
        // excluded: their skeleton path may be shorter than the full body.
        float totalRefLen = 0.f;
        int   refCount    = 0;

        for (Tracking::WormTrackPoint& tp : trackPoints) {
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
                // Accumulate reference length only from uncoiled (non-ring) frames.
                if (blob.holeContourPoints.empty()) {
                    std::vector<cv::Point2f> clPts(blob.centerlinePoints.begin(),
                                                   blob.centerlinePoints.end());
                    totalRefLen += arcLen(clPts);
                    ++refCount;
                }
                m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);
            }
        }

        float referenceLength = (refCount > 0) ? totalRefLen / refCount : 0.f;

        // ── Pass 2: orient, validate, correct ───────────────────────────
        // lastHead tracks the front endpoint of the previous valid centerline.
        // It starts invalid (-1, -1) so the very first frame is used as-is.
        cv::Point2f lastHead(-1.f, -1.f);
        bool trackModified = false;

        for (Tracking::WormTrackPoint& tp : trackPoints) {
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
            if (blob.centerlinePoints.empty()) continue;

            std::vector<cv::Point2f> clPts(blob.centerlinePoints.begin(),
                                           blob.centerlinePoints.end());
            bool isRing     = !blob.holeContourPoints.empty();
            bool headKnown  = (lastHead.x >= 0.f);

            // Orient so that clPts.front() is the head end.
            if (headKnown)
                orientToHead(clPts, lastHead);

            // For ring blobs, validate the skeleton path against the reference
            // body length.  When the path is too short it means skeletonisation
            // lost part of the body (e.g. where the self-touch gap fluctuated).
            // Fall back to contour traversal: the outer contour of a ring blob
            // IS the worm body, so walking it for referenceLength pixels gives
            // a reliable path regardless of skeleton topology.
            if (isRing && referenceLength > 0.f &&
                arcLen(clPts) < kMinArcLengthFraction * referenceLength) {

                cv::Point2f startHint = headKnown ? lastHead : clPts.front();
                int startIdx = nearestContourIdx(blob.contourPoints, startHint);

                // Try both contour directions; keep the longer walk.
                auto fwd = walkContour(blob.contourPoints, startIdx, +1, referenceLength);
                auto bwd = walkContour(blob.contourPoints, startIdx, -1, referenceLength);
                auto& best = (arcLen(fwd) >= arcLen(bwd)) ? fwd : bwd;

                if (arcLen(best) > arcLen(clPts)) {
                    clPts = resample(best, kCenterlinePoints);
                    // Re-orient after fallback so head is still at front.
                    if (headKnown)
                        orientToHead(clPts, lastHead);
                }
            }

            // Resample to a consistent point count for export / downstream use.
            if (static_cast<int>(clPts.size()) != kCenterlinePoints)
                clPts = resample(clPts, kCenterlinePoints);

            // Update head for the next frame.
            lastHead = clPts.front();

            // Write corrected centerline back to blob in storage.
            blob.centerlinePoints.assign(clPts.begin(), clPts.end());
            m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);

            // For ring blobs the geometric centroid is inside the hole.
            // Use the centerline midpoint as the authoritative body position.
            if (isRing) {
                tp.position = clPts[clPts.size() / 2];
                trackModified = true;
            }
        }

        if (trackModified)
            m_storage->setTrackForItem(wormId, trackPoints);

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}
