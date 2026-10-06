#include "centerlineworker.h"
#include "centerlineprocessor.h"
#include "centerlinetrack.h"
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
#include <map>

// Clean frames an island needs, in seconds, before it anchors a contact bridge.
static constexpr double kMinIslandSeconds = 1.0;

// ── Pass 3: name head and tail once per continuity chain ────────────────────
//
// processTrackContinuity() leaves each chain internally consistent: its clean
// frames, bridged contacts and short clean runs all agree on which physical end
// is which. These passes only decide which end of a chain is the head, using
// the chain's clean frames as evidence, and flip the whole chain as a unit, so
// a contact frame can never disagree with its neighbours afterwards.

namespace {

struct ChainDecision {
    bool decided = false;      // head/tail naming settled
    bool flipped = false;      // reversed since processTrackContinuity
    QStringList notes;
};

struct CleanFrame {
    int frame;
    cv::Point2f centroid;
    const Tracking::DetectedBlob* blob;   // valid while `store` below lives
};

// Clean frames of one chain with a usable centerline and both roles assigned.
std::vector<CleanFrame> cleanFramesOf(const std::vector<int>& chain,
                                      const std::map<int, Tracking::DetectedBlob>& store,
                                      const std::map<int, cv::Point2f>& centroids)
{
    std::vector<CleanFrame> out;
    for (int f : chain) {
        auto it = store.find(f);
        if (it == store.end()) continue;
        const auto& cl = it->second.centerline;
        if (cl.topology != Tracking::TopologyState::Clean || cl.points.size() < 4) continue;
        if (cl.headTipIdx < 0 || cl.tailTipIdx < 0) continue;
        auto c = centroids.find(f);
        if (c == centroids.end()) continue;
        out.push_back({f, c->second, &it->second});
    }
    return out;
}

void flipChain(Centerline::CenterlineFrameIo& io, int wormId, const std::vector<int>& chain,
               const QString& reason, QList<int>& flipped)
{
    for (int f : chain) {
        Centerline::reverseStoredFrame(io, wormId, f, reason);
        flipped.append(f);
    }
}

std::map<int, Tracking::DetectedBlob> loadChainBlobs(Centerline::CenterlineFrameIo& io, int wormId,
                                                    const std::vector<std::vector<int>>& chains)
{
    std::map<int, Tracking::DetectedBlob> store;
    for (const auto& chain : chains)
        for (int f : chain) {
            const QMap<int, Tracking::DetectedBlob> blobs = io.getDetectedBlobsForFrame(f);
            if (blobs.contains(wormId)) store[f] = blobs[wormId];
        }
    return store;
}

} // namespace

// Motion: over consecutive clean frames, the centroid should move toward the
// head more often than toward the tail. A chain needs kWindowSeconds of clean
// frames; one whose steps are split between directions by more than
// maxReversalFraction (reversal bouts) is left undecided.
static QList<int> refineChainsByMotion(Centerline::CenterlineFrameIo& io,
                                       int wormId,
                                       const std::map<int, cv::Point2f>& centroids,
                                       const std::vector<std::vector<int>>& chains,
                                       double fps,
                                       float maxReversalFraction,
                                       std::vector<ChainDecision>& decisions)
{
    QList<int> flipped;
    static constexpr double kWindowSeconds = 5.0;
    const int minFrames = std::max(3, static_cast<int>(fps * kWindowSeconds));
    const std::map<int, Tracking::DetectedBlob> store = loadChainBlobs(io, wormId, chains);

    for (size_t c = 0; c < chains.size(); ++c) {
        const std::vector<CleanFrame> clean = cleanFramesOf(chains[c], store, centroids);
        if (static_cast<int>(clean.size()) < minFrames) {
            decisions[c].notes << QStringLiteral("motion: %1 clean frames, need %2")
                                      .arg(clean.size()).arg(minFrames);
            continue;
        }
        float fwd = 0.f, rev = 0.f;
        for (size_t i = 1; i < clean.size(); ++i) {
            if (clean[i].frame != clean[i - 1].frame + 1) continue;
            const cv::Point2f motion = clean[i].centroid - clean[i - 1].centroid;
            const float mLen = std::hypot(motion.x, motion.y);
            if (mLen < 0.5f) continue;
            const auto& pts = clean[i - 1].blob->centerline.points;
            const cv::Point2f axis = pts.front() - pts.back();
            const float aLen = std::hypot(axis.x, axis.y);
            if (aLen < 2.f) continue;
            const float align = (motion.x * axis.x + motion.y * axis.y) / (mLen * aLen);
            if (align >= 0.f) fwd += align; else rev -= align;
        }
        const float total = fwd + rev;
        const float minority = total > 1e-6f ? std::min(fwd, rev) / total : 0.f;
        if (total <= 1e-6f) {
            decisions[c].notes << QStringLiteral("motion: no usable steps");
            continue;
        }
        if (minority > maxReversalFraction) {
            decisions[c].notes << QStringLiteral("motion: reversal fraction %1 > %2")
                                      .arg(minority, 0, 'f', 2).arg(maxReversalFraction, 0, 'f', 2);
            continue;
        }
        decisions[c].decided = true;
        const bool flip = fwd < rev;
        decisions[c].flipped = flip;
        if (flip) flipChain(io, wormId, chains[c], QStringLiteral("motion: chain's back end leads"), flipped);
        YAWT_INFO(lcCoreCenterlineWorker)
            << QStringLiteral("Worm %1 chain [%2–%3] (%4 clean frames): motion score=%5 reversalFrac=%6 flip=%7")
                   .arg(wormId).arg(chains[c].front()).arg(chains[c].back()).arg(clean.size())
                   .arg(fwd - rev, 0, 'f', 3).arg(minority, 0, 'f', 3).arg(flip ? "yes" : "no");
    }
    return flipped;
}

// ── Geometry-based head/tail naming ─────────────────────────────────────────
//
// Three tip statistics per frame:
//   A – contour asymmetry at the tip apex (head tends to be more symmetric)
//   B – local centerline curvature variance near the tip (head is more flexible)
//   C – |signed tip curvature| from TipCandidate
//
// Cohen's d (threshold 0.4, min 15 frames) over the chains motion already named
// selects which statistics distinguish this worm's head from its tail; each
// chain motion left undecided is then named by a majority vote of those
// statistics over its clean frames.

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
    if (tipIdx < 0 || tipIdx >= static_cast<int>(blob.centerline.tipCandidates.size())) return result;
    if (blob.centerline.points.size() < 4) return result;
    if (blob.contourPoints.size() < 6)    return result;

    const cv::Point2f tipPoint = blob.centerline.tipCandidates[tipIdx].point;
    const auto& cl     = blob.centerline.points;
    const auto& contour = blob.contourPoints;
    const int   N      = static_cast<int>(contour.size());

    // Feature C
    result.curvature = std::abs(blob.centerline.tipCandidates[tipIdx].curvature);

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

static QList<int> refineChainsByGeometry(Centerline::CenterlineFrameIo& io,
                                         int wormId,
                                         const std::map<int, cv::Point2f>& centroids,
                                         const std::vector<std::vector<int>>& chains,
                                         double fps,
                                         std::vector<ChainDecision>& decisions)
{
    QList<int> flipped;
    static constexpr double kMinSegSeconds  = 2.0;
    const int minSegFrames = std::max(3, static_cast<int>(fps * kMinSegSeconds));
    static constexpr int   kMinSamples      = 15;
    static constexpr float kCohensThreshold = 0.4f;
    const std::map<int, Tracking::DetectedBlob> store = loadChainBlobs(io, wormId, chains);

    struct FrameGeom { TipGeomFeatures head, tail; };
    std::vector<std::vector<FrameGeom>> perChain(chains.size());
    for (size_t c = 0; c < chains.size(); ++c) {
        for (const CleanFrame& cf : cleanFramesOf(chains[c], store, centroids)) {
            FrameGeom fg{computeTipGeomFeatures(*cf.blob, cf.blob->centerline.headTipIdx, true),
                         computeTipGeomFeatures(*cf.blob, cf.blob->centerline.tailTipIdx, false)};
            if (fg.head.valid && fg.tail.valid) perChain[c].push_back(fg);
        }
    }

    // Learn which statistics separate head from tail on chains motion named;
    // fall back to every chain when motion named none.
    bool anyDecided = false;
    for (const auto& d : decisions) anyDecided |= d.decided;
    std::vector<float> headA, tailA, headB, tailB, headC, tailC;
    for (size_t c = 0; c < chains.size(); ++c) {
        if (anyDecided && !decisions[c].decided) continue;
        for (const FrameGeom& fg : perChain[c]) {
            headA.push_back(fg.head.asymmetry);  tailA.push_back(fg.tail.asymmetry);
            headB.push_back(fg.head.curvVar);    tailB.push_back(fg.tail.curvVar);
            headC.push_back(fg.head.curvature);  tailC.push_back(fg.tail.curvature);
        }
    }
    if (static_cast<int>(headA.size()) < kMinSamples) {
        for (auto& d : decisions)
            if (!d.decided) d.notes << QStringLiteral("geometry: %1 reference frames, need %2")
                                           .arg(headA.size()).arg(kMinSamples);
        return flipped;
    }

    auto cohensD = [](const std::vector<float>& h, const std::vector<float>& t) -> float {
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
    const bool sigA = std::abs(dA) >= kCohensThreshold;
    const bool sigB = std::abs(dB) >= kCohensThreshold;
    const bool sigC = std::abs(dC) >= kCohensThreshold;
    YAWT_INFO(lcCoreCenterlineWorker)
        << QStringLiteral("Worm %1 geometry: Cohen's d  A=%2  B=%3  C=%4  (threshold ±%5, n=%6)")
               .arg(wormId).arg(dA, 0, 'f', 3).arg(dB, 0, 'f', 3).arg(dC, 0, 'f', 3)
               .arg(kCohensThreshold, 0, 'f', 2).arg(headA.size());
    if (!sigA && !sigB && !sigC) {
        for (auto& d : decisions)
            if (!d.decided) d.notes << QStringLiteral("geometry: no statistic separates head from tail");
        return flipped;
    }

    auto median = [](std::vector<float> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    for (size_t c = 0; c < chains.size(); ++c) {
        if (decisions[c].decided) continue;
        const auto& seg = perChain[c];
        if (static_cast<int>(seg.size()) < minSegFrames) {
            decisions[c].notes << QStringLiteral("geometry: %1 clean frames, need %2")
                                      .arg(seg.size()).arg(minSegFrames);
            continue;
        }
        int voteFlip = 0, voteKeep = 0;
        auto vote = [&](bool sig, float d, auto headOf, auto tailOf) {
            if (!sig) return;
            std::vector<float> hv, tv;
            for (const FrameGeom& fg : seg) { hv.push_back(headOf(fg)); tv.push_back(tailOf(fg)); }
            if ((median(hv) - median(tv)) * d < 0.f) ++voteFlip; else ++voteKeep;
        };
        vote(sigA, dA, [](const FrameGeom& f) { return f.head.asymmetry; }, [](const FrameGeom& f) { return f.tail.asymmetry; });
        vote(sigB, dB, [](const FrameGeom& f) { return f.head.curvVar; }, [](const FrameGeom& f) { return f.tail.curvVar; });
        vote(sigC, dC, [](const FrameGeom& f) { return f.head.curvature; }, [](const FrameGeom& f) { return f.tail.curvature; });
        if (voteFlip == voteKeep) {
            decisions[c].notes << QStringLiteral("geometry: tied vote %1-%2").arg(voteFlip).arg(voteKeep);
            continue;
        }
        decisions[c].decided = true;
        decisions[c].flipped = voteFlip > voteKeep;
        if (voteFlip > voteKeep)
            flipChain(io, wormId, chains[c], QStringLiteral("geometry: tip statistics favour the other end"), flipped);
        YAWT_INFO(lcCoreCenterlineWorker)
            << QStringLiteral("Worm %1 chain [%2–%3]: geometry voteFlip=%4 voteKeep=%5")
                   .arg(wormId).arg(chains[c].front()).arg(chains[c].back()).arg(voteFlip).arg(voteKeep);
    }
    return flipped;
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

QList<Tracking::MergeGroup> CenterlineWorker::getMergeGroupsForFrame(int frameNumber) const
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

// Degree-2 Savitzky-Golay smoothing over a 1-D float sequence.
// Half-window h means we look h samples on each side; boundary samples are unchanged.
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

// Apply S-G smoothing to the trace midpoint and re-relax the centerline for one worm.
// sortedPoints must be in frame order and already written to storage.
static void smoothMidpointsAndRelaxCenterlines(
    TrackingDataStorage* storage,
    QMutex* storageMutex,
    int wormId,
    const Tracking::Track& sortedPoints,
    int sgHalfWindow,
    int nPts,
    const Centerline::CenterlineSnakeParams& snakeParams)
{
    // Gather centerline midpoints for frames with valid, clean-topology blobs.
    struct FrameEntry {
        int frame;
        cv::Point2f midpoint;
    };
    // Split into consecutive runs of valid frames; merged/lost breaks a run.
    // We process each run independently so S-G never bridges a gap.
    std::vector<std::vector<FrameEntry>> runs;
    std::vector<FrameEntry> current;

    for (const auto& tp : sortedPoints) {
        if (!current.empty() && tp.frameNumber != current.back().frame + 1) {
            runs.push_back(std::move(current));
            current.clear();
        }
        if (tp.quality == Tracking::TrackPointQuality::Merged ||
            tp.quality == Tracking::TrackPointQuality::Lost) {
            if (!current.empty()) { runs.push_back(std::move(current)); current.clear(); }
            continue;
        }
        QMap<int, Tracking::DetectedBlob> frameBlobs;
        {
            QMutexLocker lk(storageMutex);
            frameBlobs = storage->getDetectedBlobsForFrame(tp.frameNumber);
        }
        if (!frameBlobs.contains(wormId)) {
            if (!current.empty()) { runs.push_back(std::move(current)); current.clear(); }
            continue;
        }
        const Tracking::DetectedBlob& blob = frameBlobs[wormId];
        if (blob.centerline.topology != Tracking::TopologyState::Clean) {
            if (!current.empty()) { runs.push_back(std::move(current)); current.clear(); }
            continue;
        }
        const int hIdx = blob.centerline.headTipIdx;
        const int tIdx = blob.centerline.tailTipIdx;
        if (hIdx < 0 || tIdx < 0 ||
            hIdx >= static_cast<int>(blob.centerline.tipCandidates.size()) ||
            tIdx >= static_cast<int>(blob.centerline.tipCandidates.size()) ||
            blob.centerline.points.empty()) {
            if (!current.empty()) { runs.push_back(std::move(current)); current.clear(); }
            continue;
        }
        current.push_back({tp.frameNumber,
                           blob.centerline.points[blob.centerline.points.size() / 2]});
    }
    if (!current.empty()) runs.push_back(std::move(current));

    for (auto& run : runs) {
        const int sz = static_cast<int>(run.size());
        if (sz < 2 * sgHalfWindow + 1) continue;

        // Build the two midpoint-coordinate time series.
        std::vector<float> mx(sz), my(sz);
        for (int i = 0; i < sz; ++i) {
            mx[i] = run[i].midpoint.x;
            my[i] = run[i].midpoint.y;
        }
        const auto smx = savitzkyGolay(mx, sgHalfWindow);
        const auto smy = savitzkyGolay(my, sgHalfWindow);

        for (int i = 0; i < sz; ++i) {
            const cv::Point2f target(smx[i], smy[i]);
            if (target == run[i].midpoint) continue;

            QMap<int, Tracking::DetectedBlob> frameBlobs;
            {
                QMutexLocker lk(storageMutex);
                frameBlobs = storage->getDetectedBlobsForFrame(run[i].frame);
            }
            if (!frameBlobs.contains(wormId)) continue;
            Tracking::DetectedBlob blob = frameBlobs[wormId];

            if (!Centerline::relaxCenterlineToSmoothedMidpoint(
                    blob, target, nPts, snakeParams)) continue;

            QMutexLocker lk(storageMutex);
            storage->setDetectedBlobForFrame(run[i].frame, wormId, blob);
        }
    }
}

// Per worm (docs/centerline_pipeline.md defines the vocabulary):
//
//   Sweep 0 — read-only walk over all non-merged, non-lost frames; build a
//             throwaway skeleton centerline per frame; refLength = median of
//             resampled lengths (the baseline measure).
//   Pass 1  — every clean frame on its own (processFrame); consecutive clean
//             frames are linked into islands by matching whole centerlines.
//   Pass 2  — each gap between anchoring islands is bridged from both sides
//             (processTrackContinuity), forming continuity chains.
//   Pass 3  — motion, then geometry, names head and tail once per chain.
//   Pass 4  — frames that could not be settled are flagged for review.
//   Then optional midpoint smoothing of clean frames.
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
    std::vector<std::pair<int, Tracking::Track>> assignedTracks;
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

        Tracking::Track sortedPoints = trackEntry.second;
        std::sort(sortedPoints.begin(), sortedPoints.end(),
                  [](const Tracking::TrackPoint& a,
                     const Tracking::TrackPoint& b) {
                      return a.frameNumber < b.frameNumber;
                  });

        // ── Sweep 0 — body length learning ──────────────────────────────
        // Read-only: skeleton on a TEMPORARY blob copy so storage stays
        // untouched. Only non-ring, non-merged, non-lost frames contribute.
        std::vector<float> validLengths;
        validLengths.reserve(sortedPoints.size());
        for (const Tracking::TrackPoint& tp : sortedPoints) {
            if (tp.quality == Tracking::TrackPointQuality::Merged ||
                tp.quality == Tracking::TrackPointQuality::Lost) continue;
            QMap<int, Tracking::DetectedBlob> frameBlobs =
                getDetectedBlobsForFrame(tp.frameNumber);
            if (!frameBlobs.contains(wormId)) continue;
            Tracking::DetectedBlob temp = frameBlobs[wormId];
            if (!temp.isValid || temp.contourPoints.empty()) continue;
            if (!temp.holeContourPoints.empty()) continue;
            if (Centerline::populateCenterlineFromContour(temp) &&
                temp.centerline.points.size() >= 2) {
                std::vector<cv::Point2f> p(temp.centerline.points.begin(),
                                           temp.centerline.points.end());
                // Same measure as the clean-frame baseline, so the two agree.
                validLengths.push_back(Centerline::resampledArcLength(p, nPts));
            }
        }

        float refLength = 0.f;
        if (!validLengths.empty()) {
            std::nth_element(validLengths.begin(),
                             validLengths.begin() + validLengths.size() / 2,
                             validLengths.end());
            refLength = validLengths[validLengths.size() / 2];
        }

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

        io.getCenterlineDebugFrame = [this](int id, int frameNumber, Debug::CenterlineFrameDebug& out) {
            if (!m_debugStore) return false;
            QMutexLocker locker(m_sharedStorageMutex.data());
            return m_debugStore->getCenterlineFrame(id, frameNumber, out);
        };

        // ── Passes 1-2: clean islands and two-sided contact bridges ────────
        Centerline::TrackPassConfig passConfig;
        passConfig.skipMergedFrames = m_skipMergedFrames;
        passConfig.minIslandFrames = std::max(3, static_cast<int>(std::lround(m_fps * kMinIslandSeconds)));
        const Centerline::TrackPassResult continuity =
            Centerline::processTrackContinuity(context, io, passConfig);
        for (const QString& line : continuity.log)
            YAWT_INFO(lcCoreCenterlineWorker) << line;

        // ── Pass 3: name head and tail once per chain ────────────────────
        std::map<int, cv::Point2f> centroids;
        for (const Tracking::TrackPoint& tp : sortedPoints)
            centroids[tp.frameNumber] = tp.position;
        // Direct evidence first; then continuity across weak bridges carries a
        // decided chain's naming to its undecided neighbours before geometry
        // is consulted, and again afterwards.
        std::vector<ChainDecision> decisions(continuity.chains.size());
        auto propagate = [&]() {
            std::vector<bool> decided(decisions.size()), flipped(decisions.size());
            for (size_t c = 0; c < decisions.size(); ++c) {
                decided[c] = decisions[c].decided;
                flipped[c] = decisions[c].flipped;
            }
            const QList<int> frames = Centerline::propagateAcrossWeakLinks(io, wormId, continuity, decided, flipped);
            for (size_t c = 0; c < decisions.size(); ++c) {
                if (decided[c] && !decisions[c].decided)
                    decisions[c].notes << QStringLiteral("named by continuity across a weak bridge");
                decisions[c].decided = decided[c];
                decisions[c].flipped = flipped[c];
            }
            return frames;
        };
        const QList<int> motionSwapped = refineChainsByMotion(
            io, wormId, centroids, continuity.chains, m_fps, m_maxReversalFraction, decisions);
        emit headTailMotionSwapEvent(wormId, motionSwapped);
        QList<int> linkSwapped = propagate();
        const QList<int> geoSwapped = refineChainsByGeometry(
            io, wormId, centroids, continuity.chains, m_fps, decisions);
        emit headTailGeometrySwapEvent(wormId, geoSwapped);
        linkSwapped += propagate();

        // XOR: a frame flipped an even number of times is unchanged.
        QSet<int> netSet;
        for (const QList<int>* list : std::initializer_list<const QList<int>*>{&motionSwapped, &linkSwapped, &geoSwapped})
            for (int f : *list) {
                if (netSet.contains(f)) netSet.remove(f);
                else netSet.insert(f);
            }
        emit headTailSwapEvent(wormId, QList<int>(netSet.begin(), netSet.end()));

        // ── Pass 4: flag what could not be settled ───────────────────────
        QSet<int> reviewFrames;
        for (const auto& [frame, reason] : continuity.review) reviewFrames.insert(frame);
        for (size_t c = 0; c < continuity.chains.size(); ++c) {
            if (decisions[c].decided) continue;
            const QString reason = QStringLiteral("head/tail naming undetermined for chain %1-%2 (%3)")
                .arg(continuity.chains[c].front()).arg(continuity.chains[c].back())
                .arg(decisions[c].notes.join(QStringLiteral("; ")));
            for (int f : continuity.chains[c]) {
                Centerline::flagStoredFrame(io, wormId, f, reason);
                reviewFrames.insert(f);
            }
        }
        emit centerlineReviewEvent(wormId, QList<int>(reviewFrames.begin(), reviewFrames.end()));

        if (m_smoothCenterline) {
            smoothMidpointsAndRelaxCenterlines(
                m_storage, m_sharedStorageMutex.data(),
                wormId, sortedPoints,
                m_sgHalfWindow, nPts, m_snakeParams);
        }

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}
