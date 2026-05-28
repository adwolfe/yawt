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

static bool relabelTrackHeadTailByMotion(
    TrackingDataStorage* storage,
    QMutex* storageMutex,
    int wormId,
    const std::vector<Tracking::WormTrackPoint>& sortedPoints)
{
    if (!storage) {
        return false;
    }

    struct MotionSample {
        int frameNumber = -1;
        cv::Point2f centroid;
        cv::Point2f front;
        cv::Point2f back;
    };

    std::vector<MotionSample> samples;
    samples.reserve(sortedPoints.size());
    for (const Tracking::WormTrackPoint& tp : sortedPoints) {
        if (tp.quality == Tracking::TrackPointQuality::Lost) {
            continue;
        }

        QMap<int, Tracking::DetectedBlob> blobs;
        {
            QMutexLocker locker(storageMutex);
            blobs = storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
        }
        if (!blobs.contains(wormId)) {
            continue;
        }

        const Tracking::DetectedBlob& blob = blobs[wormId];
        if (!blob.isValid || blob.centerlinePoints.size() < 2) {
            continue;
        }

        MotionSample sample;
        sample.frameNumber = tp.frameNumberOriginal;
        sample.centroid = tp.position;
        sample.front = blob.centerlinePoints.front();
        sample.back = blob.centerlinePoints.back();
        samples.push_back(sample);
    }

    float currentForwardEvidence = 0.f;
    float reverseEvidence = 0.f;
    int usableSteps = 0;
    for (size_t i = 1; i < samples.size(); ++i) {
        const cv::Point2f movement = samples[i].centroid - samples[i - 1].centroid;
        const float movementLen = std::hypot(movement.x, movement.y);
        if (movementLen < 0.5f) {
            continue;
        }

        const cv::Point2f tailToHead = samples[i - 1].front - samples[i - 1].back;
        const float axisLen = std::hypot(tailToHead.x, tailToHead.y);
        if (axisLen < 2.f) {
            continue;
        }

        const float alignment =
            (movement.x * tailToHead.x + movement.y * tailToHead.y) /
            (movementLen * axisLen);
        if (alignment >= 0.f) {
            currentForwardEvidence += alignment;
        } else {
            reverseEvidence += -alignment;
        }
        ++usableSteps;
    }

    const float totalEvidence = currentForwardEvidence + reverseEvidence;
    if (usableSteps < 3 || totalEvidence <= 1e-3f ||
        reverseEvidence <= currentForwardEvidence ||
        (reverseEvidence - currentForwardEvidence) < 0.05f * totalEvidence) {
        return false;
    }

    for (const MotionSample& sample : samples) {
        QMap<int, Tracking::DetectedBlob> blobs;
        {
            QMutexLocker locker(storageMutex);
            blobs = storage->getDetectedBlobsForFrame(sample.frameNumber);
        }
        if (!blobs.contains(wormId)) {
            continue;
        }

        Tracking::DetectedBlob blob = blobs[wormId];
        if (blob.centerlinePoints.size() >= 2) {
            std::reverse(blob.centerlinePoints.begin(), blob.centerlinePoints.end());
        }
        std::swap(blob.assignedHeadTipIdx, blob.assignedTailTipIdx);
        {
            QMutexLocker locker(storageMutex);
            storage->setDetectedBlobForFrame(sample.frameNumber, wormId, blob);
        }
    }

    YAWT_INFO(lcCoreCenterlineWorker)
        << QStringLiteral("Worm %1 final head/tail relabel: reverse=%2 forward=%3 steps=%4")
               .arg(wormId)
               .arg(reverseEvidence, 0, 'f', 2)
               .arg(currentForwardEvidence, 0, 'f', 2)
               .arg(usableSteps);
    return true;
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

        relabelTrackHeadTailByMotion(m_storage, m_sharedStorageMutex.data(),
                                     wormId, sortedPoints);

        ++processedWorms;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}
