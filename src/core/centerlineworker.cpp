#include "centerlineworker.h"
#include "centerlineprocessor.h"
#include "../debug/debugdatastore.h"
#include "../debug/debugrecords.h"
#include "../data/trackingcommon.h"
#include "../utils/debugutils.h"
#include "../utils/loggingcategories.h"
#include <QDebug>
#include <QMutexLocker>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <functional>

// Refine head/tail assignment by examining clean segments of the track.
//
// For each continuous run of Clean-topology frames long enough to establish a
// reliable direction (>= fps * kWindowSeconds frames), the motion of the worm's
// centroid is compared against the front-to-back axis of the centerline.  If
// the back end is leading (score < 0), the centerline and head/tail indices are
// reversed for every frame in that segment.
//
// maxReversalFraction: if more than this fraction of usable direction steps
// within a segment point against the majority direction, the segment is treated
// as ambiguous (direction reversal / turning event) and skipped.
//
// Returns the frame numbers whose centerlines were reversed.
static QList<int> refineHeadTailByDirection(
    TrackingDataStorage* storage,
    QMutex* storageMutex,
    int wormId,
    const std::vector<Tracking::WormTrackPoint>& sortedPoints,
    double fps,
    float maxReversalFraction)
{
    QList<int> flippedFrames;
    if (!storage) return flippedFrames;

    static constexpr double kWindowSeconds = 5.0;
    const int minFrames = std::max(3, static_cast<int>(fps * kWindowSeconds));

    struct SegFrame {
        int frameNumber;
        cv::Point2f centroid;
        cv::Point2f front;
        cv::Point2f back;
    };

    auto tryLoadClean = [&](const Tracking::WormTrackPoint& tp) -> std::optional<SegFrame> {
        QMap<int, Tracking::DetectedBlob> blobs;
        {
            QMutexLocker locker(storageMutex);
            blobs = storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
        }
        if (!blobs.contains(wormId)) return std::nullopt;
        const Tracking::DetectedBlob& blob = blobs[wormId];
        if (!blob.isValid || blob.centerlinePoints.size() < 2) return std::nullopt;
        if (blob.topologyState != Tracking::TopologyState::Clean) return std::nullopt;
        return SegFrame{tp.frameNumberOriginal, tp.position,
                        blob.centerlinePoints.front(), blob.centerlinePoints.back()};
    };

    // Returns {netScore, minorityFraction} for a segment.
    // netScore > 0 → front is head. minorityFraction is the fraction of usable
    // steps that go against the majority — high values indicate a reversal.
    struct SegmentStats { float score; float minorityFraction; };
    auto analyzeSegment = [](const std::vector<SegFrame>& seg) -> SegmentStats {
        float fwdSteps = 0.f, revSteps = 0.f;
        for (size_t i = 1; i < seg.size(); ++i) {
            const cv::Point2f motion = seg[i].centroid - seg[i - 1].centroid;
            const float mLen = std::hypot(motion.x, motion.y);
            if (mLen < 0.5f) continue;
            const cv::Point2f axis = seg[i - 1].front - seg[i - 1].back;
            const float aLen = std::hypot(axis.x, axis.y);
            if (aLen < 2.f) continue;
            const float align = (motion.x * axis.x + motion.y * axis.y) / (mLen * aLen);
            if (align >= 0.f) fwdSteps += align;
            else              revSteps += -align;
        }
        const float total = fwdSteps + revSteps;
        const float minFrac = (total > 1e-6f)
            ? std::min(fwdSteps, revSteps) / total
            : 0.f;
        return {fwdSteps - revSteps, minFrac};
    };

    auto flipSegment = [&](const std::vector<SegFrame>& seg) {
        for (const SegFrame& sf : seg) {
            QMap<int, Tracking::DetectedBlob> blobs;
            {
                QMutexLocker locker(storageMutex);
                blobs = storage->getDetectedBlobsForFrame(sf.frameNumber);
            }
            if (!blobs.contains(wormId)) continue;
            Tracking::DetectedBlob blob = blobs[wormId];
            if (blob.centerlinePoints.size() >= 2)
                std::reverse(blob.centerlinePoints.begin(), blob.centerlinePoints.end());
            std::swap(blob.assignedHeadTipIdx, blob.assignedTailTipIdx);
            {
                QMutexLocker locker(storageMutex);
                storage->setDetectedBlobForFrame(sf.frameNumber, wormId, blob);
            }
            flippedFrames.append(sf.frameNumber);
        }
    };

    auto finalizeSegment = [&](const std::vector<SegFrame>& seg) {
        if (static_cast<int>(seg.size()) < minFrames) return;

        auto [score, minFrac] = analyzeSegment(seg);

        // Skip this segment if there is a significant direction reversal within it.
        if (minFrac > maxReversalFraction) {
            YAWT_INFO(lcCoreCenterlineWorker)
                << QStringLiteral("Worm %1 segment [%2–%3] (%4 frames): "
                                  "skipped (reversal fraction=%5 > threshold=%6)")
                       .arg(wormId)
                       .arg(seg.front().frameNumber)
                       .arg(seg.back().frameNumber)
                       .arg(static_cast<int>(seg.size()))
                       .arg(minFrac, 0, 'f', 3)
                       .arg(maxReversalFraction, 0, 'f', 3);
            return;
        }

        const bool needsFlip = score < 0.f;
        if (needsFlip) flipSegment(seg);

        YAWT_INFO(lcCoreCenterlineWorker)
            << QStringLiteral("Worm %1 segment [%2–%3] (%4 frames): "
                              "score=%5 reversalFrac=%6 flip=%7")
                   .arg(wormId)
                   .arg(seg.front().frameNumber)
                   .arg(seg.back().frameNumber)
                   .arg(static_cast<int>(seg.size()))
                   .arg(score, 0, 'f', 3)
                   .arg(minFrac, 0, 'f', 3)
                   .arg(needsFlip ? "yes" : "no");
    };

    std::vector<SegFrame> currentSegment;

    for (const Tracking::WormTrackPoint& tp : sortedPoints) {
        if (tp.quality == Tracking::TrackPointQuality::Lost) {
            finalizeSegment(currentSegment);
            currentSegment.clear();
            continue;
        }
        auto frame = tryLoadClean(tp);
        if (!frame) {
            finalizeSegment(currentSegment);
            currentSegment.clear();
            continue;
        }
        currentSegment.push_back(*frame);
    }
    finalizeSegment(currentSegment);

    return flippedFrames;
}

// ── Geometry-based head/tail refinement ─────────────────────────────────────
//
// Three tip statistics per frame:
//   A – contour asymmetry at the tip apex (head tends to be more symmetric)
//   B – local centerline curvature variance near the tip (head is more flexible)
//   C – |signed tip curvature| from TipCandidate
//
// Global Cohen's d (threshold 0.4, min 15 frames) selects which statistics are
// informative for this worm.  Then each clean segment >= fps*2 s is given a
// majority vote across the significant statistics; if the observed head/tail
// difference contradicts the global signal, the segment is flipped.

struct TipGeomFeatures {
    float asymmetry = 0.f;
    float curvVar   = 0.f;
    float curvature = 0.f;
    bool  valid     = false;
};

static TipGeomFeatures computeTipGeomFeatures(
    const Tracking::DetectedBlob& blob,
    int tipIdx,
    bool isFront)
{
    TipGeomFeatures result;
    if (tipIdx < 0 || tipIdx >= static_cast<int>(blob.tipCandidates.size())) return result;
    if (blob.centerlinePoints.size() < 4) return result;
    if (blob.contourPoints.size() < 6)    return result;

    const cv::Point2f tipPoint = blob.tipCandidates[tipIdx].point;
    const auto& cl     = blob.centerlinePoints;
    const auto& contour = blob.contourPoints;
    const int   N      = static_cast<int>(contour.size());

    // Feature C
    result.curvature = std::abs(blob.tipCandidates[tipIdx].curvature);

    // Inward body axis from tip (averaged over a few points to smooth noise)
    cv::Point2f bodyAxis;
    if (isFront) {
        bodyAxis = cl[std::min(3, static_cast<int>(cl.size()) - 1)] - cl[0];
    } else {
        int last = static_cast<int>(cl.size()) - 1;
        bodyAxis = cl[std::max(0, last - 3)] - cl[last];
    }
    const float axisLen = std::hypot(bodyAxis.x, bodyAxis.y);
    if (axisLen < 1.f) return result;
    bodyAxis.x /= axisLen;
    bodyAxis.y /= axisLen;

    // Feature A: contour asymmetry – nearest contour point to tip, then K
    // points on each side.  Mean angle each side makes with body axis.
    int tipContourIdx = 0;
    {
        float minDist2 = 1e9f;
        for (int i = 0; i < N; ++i) {
            const float dx = contour[i].x - tipPoint.x;
            const float dy = contour[i].y - tipPoint.y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < minDist2) { minDist2 = d2; tipContourIdx = i; }
        }
    }
    const int K = std::min(10, N / 4);
    if (K >= 2) {
        auto meanAngleFromAxis = [&](bool leftSide) -> float {
            float sumAngle = 0.f;
            int   count   = 0;
            for (int k = 1; k <= K; ++k) {
                const int idx = leftSide
                    ? ((tipContourIdx - k) % N + N) % N
                    :  (tipContourIdx + k) % N;
                cv::Point2f v(contour[idx].x - tipPoint.x,
                              contour[idx].y - tipPoint.y);
                const float len = std::hypot(v.x, v.y);
                if (len < 0.5f) continue;
                v.x /= len; v.y /= len;
                const float dot = std::clamp(v.x * bodyAxis.x + v.y * bodyAxis.y,
                                             -1.f, 1.f);
                sumAngle += std::acos(dot);
                ++count;
            }
            return count > 0 ? sumAngle / count : 0.f;
        };
        result.asymmetry = std::abs(meanAngleFromAxis(true) - meanAngleFromAxis(false));
    }

    // Feature B: centerline curvature variance in the first/last 20% of points
    {
        const int Kcl     = std::max(2, static_cast<int>(cl.size()) / 5);
        const int startCl = isFront ? 0 : static_cast<int>(cl.size()) - Kcl;
        const int endCl   = isFront ? Kcl : static_cast<int>(cl.size());
        std::vector<float> angles;
        for (int i = startCl + 1; i < endCl - 1; ++i) {
            if (i < 1 || i >= static_cast<int>(cl.size()) - 1) continue;
            cv::Point2f v1 = cl[i] - cl[i - 1];
            cv::Point2f v2 = cl[i + 1] - cl[i];
            const float l1 = std::hypot(v1.x, v1.y);
            const float l2 = std::hypot(v2.x, v2.y);
            if (l1 < 0.5f || l2 < 0.5f) continue;
            v1.x /= l1; v1.y /= l1;
            v2.x /= l2; v2.y /= l2;
            angles.push_back(std::acos(std::clamp(v1.x*v2.x + v1.y*v2.y, -1.f, 1.f)));
        }
        if (angles.size() >= 2) {
            float mean = 0.f;
            for (float a : angles) mean += a;
            mean /= angles.size();
            float var = 0.f;
            for (float a : angles) var += (a - mean) * (a - mean);
            result.curvVar = var / angles.size();
        }
    }

    result.valid = true;
    return result;
}

static QList<int> refineHeadTailByGeometry(
    TrackingDataStorage* storage,
    QMutex* storageMutex,
    int wormId,
    const std::vector<Tracking::WormTrackPoint>& sortedPoints,
    double fps)
{
    QList<int> flippedFrames;
    if (!storage) return flippedFrames;

    static constexpr double kMinSegSeconds  = 2.0;
    const int minSegFrames = std::max(3, static_cast<int>(fps * kMinSegSeconds));
    static constexpr int   kMinSamples      = 15;
    static constexpr float kCohensThreshold = 0.4f;

    struct FrameGeom {
        int frameNumber = -1;
        TipGeomFeatures head;
        TipGeomFeatures tail;
    };

    // Load features for one track point; returns nullopt for non-Clean frames.
    auto tryLoadGeom = [&](const Tracking::WormTrackPoint& tp)
        -> std::optional<FrameGeom>
    {
        QMap<int, Tracking::DetectedBlob> blobs;
        {
            QMutexLocker locker(storageMutex);
            blobs = storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
        }
        if (!blobs.contains(wormId)) return std::nullopt;
        const Tracking::DetectedBlob& blob = blobs[wormId];
        if (!blob.isValid) return std::nullopt;
        if (blob.topologyState != Tracking::TopologyState::Clean) return std::nullopt;
        if (blob.assignedHeadTipIdx < 0 || blob.assignedTailTipIdx < 0) return std::nullopt;
        if (blob.centerlinePoints.size() < 4) return std::nullopt;

        FrameGeom fg;
        fg.frameNumber = tp.frameNumberOriginal;
        fg.head = computeTipGeomFeatures(blob, blob.assignedHeadTipIdx, true);
        fg.tail = computeTipGeomFeatures(blob, blob.assignedTailTipIdx, false);
        if (!fg.head.valid || !fg.tail.valid) return std::nullopt;
        return fg;
    };

    // Pass 1: collect all clean-frame geometries, parallel to sortedPoints.
    std::vector<std::optional<FrameGeom>> perPoint;
    perPoint.reserve(sortedPoints.size());
    std::vector<FrameGeom> allFrameGeoms;
    for (const Tracking::WormTrackPoint& tp : sortedPoints) {
        auto fg = tryLoadGeom(tp);
        if (fg) allFrameGeoms.push_back(*fg);
        perPoint.push_back(std::move(fg));
    }

    if (static_cast<int>(allFrameGeoms.size()) < kMinSamples) return flippedFrames;

    // Build per-statistic head/tail vectors from all clean frames.
    std::vector<float> headA, tailA, headB, tailB, headC, tailC;
    headA.reserve(allFrameGeoms.size()); tailA.reserve(allFrameGeoms.size());
    headB.reserve(allFrameGeoms.size()); tailB.reserve(allFrameGeoms.size());
    headC.reserve(allFrameGeoms.size()); tailC.reserve(allFrameGeoms.size());
    for (const FrameGeom& fg : allFrameGeoms) {
        headA.push_back(fg.head.asymmetry);  tailA.push_back(fg.tail.asymmetry);
        headB.push_back(fg.head.curvVar);    tailB.push_back(fg.tail.curvVar);
        headC.push_back(fg.head.curvature);  tailC.push_back(fg.tail.curvature);
    }

    // Cohen's d = (mean_head − mean_tail) / pooled_std.
    // Positive d → head typically larger; negative → tail typically larger.
    auto cohensD = [](const std::vector<float>& h,
                      const std::vector<float>& t) -> float {
        if (h.size() < 2 || t.size() < 2) return 0.f;
        float mH = 0.f, mT = 0.f;
        for (float v : h) mH += v;
        for (float v : t) mT += v;
        mH /= h.size(); mT /= t.size();
        float vH = 0.f, vT = 0.f;
        for (float v : h) vH += (v - mH) * (v - mH);
        for (float v : t) vT += (v - mT) * (v - mT);
        vH /= (h.size() - 1); vT /= (t.size() - 1);
        const float pooled = std::sqrt((vH + vT) / 2.f);
        return pooled < 1e-9f ? 0.f : (mH - mT) / pooled;
    };

    const float dA = cohensD(headA, tailA);
    const float dB = cohensD(headB, tailB);
    const float dC = cohensD(headC, tailC);

    YAWT_INFO(lcCoreCenterlineWorker)
        << QStringLiteral("Worm %1 geometry: Cohen's d  A=%2  B=%3  C=%4  "
                          "(threshold ±%5, n=%6)")
               .arg(wormId)
               .arg(dA, 0, 'f', 3).arg(dB, 0, 'f', 3).arg(dC, 0, 'f', 3)
               .arg(kCohensThreshold, 0, 'f', 2)
               .arg(allFrameGeoms.size());

    const bool sigA = std::abs(dA) >= kCohensThreshold;
    const bool sigB = std::abs(dB) >= kCohensThreshold;
    const bool sigC = std::abs(dC) >= kCohensThreshold;

    if (!sigA && !sigB && !sigC) {
        YAWT_INFO(lcCoreCenterlineWorker)
            << QStringLiteral("Worm %1 geometry: no significant statistics, "
                              "skipping geometry refinement").arg(wormId);
        return flippedFrames;
    }

    // Flip all frames in a segment.
    auto flipSegGeom = [&](const std::vector<FrameGeom>& seg) {
        for (const FrameGeom& fg : seg) {
            QMap<int, Tracking::DetectedBlob> blobs;
            {
                QMutexLocker locker(storageMutex);
                blobs = storage->getDetectedBlobsForFrame(fg.frameNumber);
            }
            if (!blobs.contains(wormId)) continue;
            Tracking::DetectedBlob blob = blobs[wormId];
            if (blob.centerlinePoints.size() >= 2)
                std::reverse(blob.centerlinePoints.begin(), blob.centerlinePoints.end());
            std::swap(blob.assignedHeadTipIdx, blob.assignedTailTipIdx);
            {
                QMutexLocker locker(storageMutex);
                storage->setDetectedBlobForFrame(fg.frameNumber, wormId, blob);
            }
            flippedFrames.append(fg.frameNumber);
        }
    };

    // Vote across significant statistics for one segment.
    auto finalizeSegGeom = [&](const std::vector<FrameGeom>& seg) {
        if (static_cast<int>(seg.size()) < minSegFrames) return;

        // Segment median head – median tail for a statistic.
        auto segMedianDiff = [&](
            std::function<float(const FrameGeom&)> headGetter,
            std::function<float(const FrameGeom&)> tailGetter) -> float
        {
            std::vector<float> hv, tv;
            hv.reserve(seg.size()); tv.reserve(seg.size());
            for (const auto& fg : seg) {
                hv.push_back(headGetter(fg));
                tv.push_back(tailGetter(fg));
            }
            std::sort(hv.begin(), hv.end());
            std::sort(tv.begin(), tv.end());
            return hv[hv.size() / 2] - tv[tv.size() / 2];
        };

        int voteFlip = 0, voteKeep = 0;
        if (sigA) {
            const float diff = segMedianDiff(
                [](const FrameGeom& f){ return f.head.asymmetry; },
                [](const FrameGeom& f){ return f.tail.asymmetry; });
            if (diff * dA < 0.f) ++voteFlip; else ++voteKeep;
        }
        if (sigB) {
            const float diff = segMedianDiff(
                [](const FrameGeom& f){ return f.head.curvVar; },
                [](const FrameGeom& f){ return f.tail.curvVar; });
            if (diff * dB < 0.f) ++voteFlip; else ++voteKeep;
        }
        if (sigC) {
            const float diff = segMedianDiff(
                [](const FrameGeom& f){ return f.head.curvature; },
                [](const FrameGeom& f){ return f.tail.curvature; });
            if (diff * dC < 0.f) ++voteFlip; else ++voteKeep;
        }

        const bool needsFlip = voteFlip > voteKeep;
        if (needsFlip) flipSegGeom(seg);

        YAWT_INFO(lcCoreCenterlineWorker)
            << QStringLiteral("Worm %1 geo-seg [%2–%3] (%4 frames): "
                              "voteFlip=%5 voteKeep=%6 flip=%7")
                   .arg(wormId)
                   .arg(seg.front().frameNumber)
                   .arg(seg.back().frameNumber)
                   .arg(static_cast<int>(seg.size()))
                   .arg(voteFlip).arg(voteKeep)
                   .arg(needsFlip ? "yes" : "no");
    };

    // Pass 2: walk perPoint, finalize segments on breaks.
    std::vector<FrameGeom> currentSeg;
    for (size_t i = 0; i < sortedPoints.size(); ++i) {
        if (!perPoint[i]) {
            finalizeSegGeom(currentSeg);
            currentSeg.clear();
        } else {
            currentSeg.push_back(*perPoint[i]);
        }
    }
    finalizeSegGeom(currentSeg);

    return flippedFrames;
}

// ── CenterlineWorker ────────────────────────────────────────────────────────

CenterlineWorker::CenterlineWorker(TrackingDataStorage* storage,
                                   Debug::DebugDataStore* debugStore,
                                   QObject* parent)
    : QObject(parent), m_storage(storage), m_debugStore(debugStore) {}

void CenterlineWorker::setSnakeParams(const Centerline::CenterlineSnakeParams& params)
{
    m_snakeParams = params;
}

void CenterlineWorker::setWormIds(const QList<int>& wormIds)
{
    m_wormIds = wormIds;
}

void CenterlineWorker::setClearBaselinesAtStart(bool clearAtStart)
{
    m_clearBaselinesAtStart = clearAtStart;
}

void CenterlineWorker::setSharedStorageMutex(const QSharedPointer<QMutex>& mutex)
{
    m_sharedStorageMutex = mutex;
}

void CenterlineWorker::setSkipMergedFrames(bool skip)
{
    m_skipMergedFrames = skip;
}

void CenterlineWorker::setFps(double fps)
{
    m_fps = fps > 0.0 ? fps : 25.0;
}

void CenterlineWorker::setMaxReversalFraction(float fraction)
{
    m_maxReversalFraction = qBound(0.f, fraction, 1.f);
}

void CenterlineWorker::setSmoothCenterline(bool smooth)
{
    m_smoothCenterline = smooth;
}

void CenterlineWorker::setSgHalfWindow(int halfWindow)
{
    m_sgHalfWindow = std::max(1, halfWindow);
}

QMap<int, Tracking::DetectedBlob> CenterlineWorker::getDetectedBlobsForFrame(int frameNumber) const
{
    QMutexLocker locker(m_sharedStorageMutex.data());
    return m_storage->getDetectedBlobsForFrame(frameNumber);
}

QList<QList<int>> CenterlineWorker::getMergeGroupsForFrame(int frameNumber) const
{
    QMutexLocker locker(m_sharedStorageMutex.data());
    return m_storage->getMergeGroupsForFrame(frameNumber);
}

Centerline::TipFeatureBaseline CenterlineWorker::getTipBaseline(int wormId) const
{
    QMutexLocker locker(m_sharedStorageMutex.data());
    return m_storage->getTipBaseline(wormId);
}

void CenterlineWorker::setDetectedBlobForFrame(int frameNumber, int wormId,
                                               const Tracking::DetectedBlob& blob)
{
    QMutexLocker locker(m_sharedStorageMutex.data());
    m_storage->setDetectedBlobForFrame(frameNumber, wormId, blob);
}

void CenterlineWorker::recordTipFeatureSample(int wormId, float curvatureMagnitude, float width)
{
    QMutexLocker locker(m_sharedStorageMutex.data());
    m_storage->recordTipFeatureSample(wormId, curvatureMagnitude, width);
}

void CenterlineWorker::recordBodyLengthSample(int wormId, float length)
{
    QMutexLocker locker(m_sharedStorageMutex.data());
    m_storage->recordBodyLengthSample(wormId, length);
}

void CenterlineWorker::setCenterlineDebugFrame(const Debug::CenterlineFrameDebug& record)
{
    if (!m_debugStore) {
        return;
    }
    QMutexLocker locker(m_sharedStorageMutex.data());
    m_debugStore->setCenterlineFrame(record);
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
// Degree-2 Savitzky-Golay smoothing over a 1-D float sequence.
// Half-window h means we look h frames on each side; boundary frames are left unchanged.
// Formula: c[k] = 3h(h+1) - 1 - 5k^2,  norm = (2h-1)(2h+1)(2h+3)/3
static std::vector<float> savitzkyGolay(const std::vector<float>& y, int h)
{
    const int n = static_cast<int>(y.size());
    if (n < 2 * h + 1 || h < 1) return y;

    const double norm = (2.0 * h - 1) * (2.0 * h + 1) * (2.0 * h + 3) / 3.0;
    std::vector<float> out(y);
    for (int i = h; i < n - h; ++i) {
        double sum = 0.0;
        for (int k = -h; k <= h; ++k)
            sum += (3.0 * h * (h + 1) - 1.0 - 5.0 * k * k) * y[i + k];
        out[i] = static_cast<float>(sum / norm);
    }
    return out;
}

// Apply S-G smoothing to tip positions and re-relax the centerline for one worm.
// sortedPoints must be in frame order and already written to storage.
static void smoothTipsAndRelaxCenterlines(
    TrackingDataStorage* storage,
    QMutex* storageMutex,
    int wormId,
    const std::vector<Tracking::WormTrackPoint>& sortedPoints,
    int sgHalfWindow,
    int nPts,
    const Centerline::CenterlineSnakeParams& snakeParams)
{
    // Gather per-frame tip info for frames with valid, clean-topology blobs.
    struct FrameEntry {
        int frame;
        cv::Point2f head;
        cv::Point2f tail;
    };
    // Split into consecutive runs of valid frames; merged/lost breaks a run.
    // We process each run independently so S-G never bridges a gap.
    std::vector<std::vector<FrameEntry>> runs;
    std::vector<FrameEntry> current;

    for (const auto& tp : sortedPoints) {
        if (tp.quality == Tracking::TrackPointQuality::Merged ||
            tp.quality == Tracking::TrackPointQuality::Lost) {
            if (!current.empty()) { runs.push_back(std::move(current)); current.clear(); }
            continue;
        }
        QMap<int, Tracking::DetectedBlob> frameBlobs;
        {
            QMutexLocker lk(storageMutex);
            frameBlobs = storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
        }
        if (!frameBlobs.contains(wormId)) {
            if (!current.empty()) { runs.push_back(std::move(current)); current.clear(); }
            continue;
        }
        const Tracking::DetectedBlob& blob = frameBlobs[wormId];
        if (blob.topologyState != Tracking::TopologyState::Clean) {
            if (!current.empty()) { runs.push_back(std::move(current)); current.clear(); }
            continue;
        }
        const int hIdx = blob.assignedHeadTipIdx;
        const int tIdx = blob.assignedTailTipIdx;
        if (hIdx < 0 || tIdx < 0 ||
            hIdx >= static_cast<int>(blob.tipCandidates.size()) ||
            tIdx >= static_cast<int>(blob.tipCandidates.size())) {
            if (!current.empty()) { runs.push_back(std::move(current)); current.clear(); }
            continue;
        }
        current.push_back({tp.frameNumberOriginal,
                           blob.tipCandidates[hIdx].point,
                           blob.tipCandidates[tIdx].point});
    }
    if (!current.empty()) runs.push_back(std::move(current));

    for (auto& run : runs) {
        const int sz = static_cast<int>(run.size());
        if (sz < 2 * sgHalfWindow + 1) continue;

        // Build per-coordinate time series.
        std::vector<float> hx(sz), hy(sz), tx(sz), ty(sz);
        for (int i = 0; i < sz; ++i) {
            hx[i] = run[i].head.x;  hy[i] = run[i].head.y;
            tx[i] = run[i].tail.x;  ty[i] = run[i].tail.y;
        }
        const auto shx = savitzkyGolay(hx, sgHalfWindow);
        const auto shy = savitzkyGolay(hy, sgHalfWindow);
        const auto stx = savitzkyGolay(tx, sgHalfWindow);
        const auto sty = savitzkyGolay(ty, sgHalfWindow);

        for (int i = 0; i < sz; ++i) {
            const cv::Point2f newHead(shx[i], shy[i]);
            const cv::Point2f newTail(stx[i], sty[i]);
            if (newHead == run[i].head && newTail == run[i].tail) continue;

            QMap<int, Tracking::DetectedBlob> frameBlobs;
            {
                QMutexLocker lk(storageMutex);
                frameBlobs = storage->getDetectedBlobsForFrame(run[i].frame);
            }
            if (!frameBlobs.contains(wormId)) continue;
            Tracking::DetectedBlob blob = frameBlobs[wormId];

            blob.tipCandidates[blob.assignedHeadTipIdx].point = newHead;
            blob.tipCandidates[blob.assignedTailTipIdx].point = newTail;
            Centerline::relaxCenterlineToSmoothedTips(blob, nPts, snakeParams);

            QMutexLocker lk(storageMutex);
            storage->setDetectedBlobForFrame(run[i].frame, wormId, blob);
        }
    }
}

//             Step 5: predictor update for next frame.
void CenterlineWorker::doWork()
{
    if (!m_storage) {
        emit failed("No storage provided to CenterlineWorker");
        return;
    }

    if (m_sharedStorageMutex.isNull()) {
        m_sharedStorageMutex = QSharedPointer<QMutex>::create();
    }

    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    std::vector<std::pair<int, std::vector<Tracking::WormTrackPoint>>> assignedTracks;
    assignedTracks.reserve(m_wormIds.isEmpty()
                               ? tracks.size()
                               : static_cast<size_t>(m_wormIds.size()));
    QSet<int> assignedIds;
    for (int wormId : m_wormIds) {
        assignedIds.insert(wormId);
    }
    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        if (!assignedIds.isEmpty() && !assignedIds.contains(it->first)) {
            continue;
        }
        assignedTracks.push_back(*it);
    }

    const int totalWorms = static_cast<int>(assignedTracks.size());
    if (totalWorms == 0) {
        emit progress(100);
        emit finished();
        return;
    }

    // A rerun reads the same clean frames, so re-accumulating into existing
    // baseline counts would inflate the sample count without new info.
    if (m_clearBaselinesAtStart) {
        QMutexLocker locker(m_sharedStorageMutex.data());
        m_storage->clearAllTipBaselines();
    }

    const int nPts = std::max(4, m_snakeParams.nPoints);
    int processedWorms = 0;

    for (const auto& trackEntry : assignedTracks) {
        const int wormId = trackEntry.first;

        std::vector<Tracking::WormTrackPoint> sortedPoints = trackEntry.second;
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
                getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (!frameBlobs.contains(wormId)) continue;
            Tracking::DetectedBlob temp = frameBlobs[wormId];
            if (!temp.isValid || temp.contourPoints.empty()) continue;
            if (!temp.holeContourPoints.empty()) continue;
            if (Tracking::populateCenterlineFromContour(temp) &&
                temp.centerlinePoints.size() >= 2) {
                std::vector<cv::Point2f> p(temp.centerlinePoints.begin(),
                                           temp.centerlinePoints.end());
                validLengths.push_back(Centerline::arcLength(p));
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

        Centerline::CenterlineFrameContext context;
        context.wormId = wormId;
        context.sortedPoints = &sortedPoints;
        context.nPts = nPts;
        context.refLength = refLength;
        context.snakeParams = m_snakeParams;
        context.captureDebug = m_debugStore && DebugUtils::isDebugCaptureEnabled();

        Centerline::CenterlineFrameIo io;
        io.getDetectedBlobsForFrame = [this](int frameNumber) {
            return getDetectedBlobsForFrame(frameNumber);
        };
        io.getMergeGroupsForFrame = [this](int frameNumber) {
            return getMergeGroupsForFrame(frameNumber);
        };
        io.getTipBaseline = [this](int id) {
            return getTipBaseline(id);
        };
        io.setDetectedBlobForFrame = [this](int frameNumber, int id,
                                            const Tracking::DetectedBlob& blob) {
            setDetectedBlobForFrame(frameNumber, id, blob);
        };
        io.recordTipFeatureSample = [this](int id, float curvatureMagnitude, float width) {
            recordTipFeatureSample(id, curvatureMagnitude, width);
        };
        io.recordBodyLengthSample = [this](int id, float length) {
            recordBodyLengthSample(id, length);
        };
        io.setCenterlineDebugFrame = [this](const Debug::CenterlineFrameDebug& record) {
            setCenterlineDebugFrame(record);
        };

        // ── Sweep 1 entry: process keyframe, then propagate outward ────
        // skipIfMerged is passed on every request; processFrame uses the
        // per-worm TrackPointQuality to decide whether a frame is merged.
        // result.processed == false means the frame was skipped (merged or lost);
        // the next frame is then treated as a fresh keyframe bootstrap.
        Centerline::CenterlineSweepState seedState;
        bool seedWasSkipped = false;
        {
            Centerline::CenterlineFrameRequest req{keyframeIdx, 1, true};
            req.skipIfMerged = m_skipMergedFrames;
            auto r = Centerline::processFrame(context, req, seedState, io);
            seedWasSkipped = !r.processed;
            if (seedWasSkipped) seedState = Centerline::CenterlineSweepState{};
        }

        // Forward pass.
        {
            Centerline::CenterlineSweepState sweepState = seedState;
            bool prevWasSkipped = seedWasSkipped;
            for (int i = keyframeIdx + 1;
                 i < static_cast<int>(sortedPoints.size()); ++i) {
                const bool bootstrap = prevWasSkipped;
                if (bootstrap) sweepState = Centerline::CenterlineSweepState{};
                Centerline::CenterlineFrameRequest req{i, 1, bootstrap};
                req.skipIfMerged = m_skipMergedFrames;
                auto r = Centerline::processFrame(context, req, sweepState, io);
                prevWasSkipped = !r.processed;
            }
        }

        // Backward pass.
        {
            Centerline::CenterlineSweepState sweepState = seedState;
            bool prevWasSkipped = seedWasSkipped;
            for (int i = keyframeIdx - 1; i >= 0; --i) {
                const bool bootstrap = prevWasSkipped;
                if (bootstrap) sweepState = Centerline::CenterlineSweepState{};
                Centerline::CenterlineFrameRequest req{i, -1, bootstrap};
                req.skipIfMerged = m_skipMergedFrames;
                auto r = Centerline::processFrame(context, req, sweepState, io);
                prevWasSkipped = !r.processed;
            }
        }

        const QList<int> dirSwapped = refineHeadTailByDirection(
            m_storage, m_sharedStorageMutex.data(),
            wormId, sortedPoints, m_fps, m_maxReversalFraction);
        emit headTailDirectionSwapEvent(wormId, dirSwapped);

        const QList<int> geoSwapped = refineHeadTailByGeometry(
            m_storage, m_sharedStorageMutex.data(),
            wormId, sortedPoints, m_fps);
        emit headTailGeometrySwapEvent(wormId, geoSwapped);

        // XOR: a frame flipped by both passes cancels out (net no change).
        QSet<int> netSet;
        for (int f : dirSwapped) netSet.insert(f);
        for (int f : geoSwapped) {
            if (netSet.contains(f)) netSet.remove(f);
            else netSet.insert(f);
        }
        emit headTailSwapEvent(wormId, QList<int>(netSet.begin(), netSet.end()));

        if (m_smoothCenterline) {
            smoothTipsAndRelaxCenterlines(
                m_storage, m_sharedStorageMutex.data(),
                wormId, sortedPoints,
                m_sgHalfWindow, nPts, m_snakeParams);
        }

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}
