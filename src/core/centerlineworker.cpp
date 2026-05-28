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

        const QList<int> swapped = refineHeadTailByDirection(
            m_storage, m_sharedStorageMutex.data(),
            wormId, sortedPoints, m_fps, m_maxReversalFraction);
        emit headTailSwapEvent(wormId, swapped);

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}
