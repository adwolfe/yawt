#ifndef CENTERLINEWORKER_H
#define CENTERLINEWORKER_H

#include <QObject>
#include <QList>
#include <QMutex>
#include <QSharedPointer>
#include <QString>
#include "centerlinetypes.h"
#include "../data/trackingdatastorage.h"
#include "../data/trackingcommon.h"

namespace Debug {
class DebugDataStore;
struct CenterlineFrameDebug;
}

/**
 * @class CenterlineWorker
 * @brief Background worker that populates centerline data after tracking.
 *
 * Runs on a QThread after tracking completes. Iterates over all stored blobs, calls
 * populateCenterlineFromContour() for blobs that have contour data but no centerline,
 * and writes the result back to storage via setDetectedBlobForFrame().
 *
 * Always skips Lost frames. Merged frames are skipped when setSkipMergedFrames(true)
 * is enabled; otherwise they are processed with ambiguous-blob topology.
 *
 * For ring/coiled frames where a previous-frame centerline is available, runs the
 * active-contour ("snake") refinement using parameters set via setSnakeParams(). The
 * snake is allowed to self-intersect; legacy skeleton-with-cut path is kept as a
 * fallback when the snake is disabled or fails.
 */
class CenterlineWorker : public QObject {
    Q_OBJECT

public:
    explicit CenterlineWorker(TrackingDataStorage* storage,
                              Debug::DebugDataStore* debugStore = nullptr,
                              QObject* parent = nullptr);

    /**
     * @brief Configure the active-contour refinement used on ring/coiled frames.
     *        Must be called before doWork() to take effect.
     */
    void setSnakeParams(const Centerline::CenterlineSnakeParams& params);
    void setWormIds(const QList<int>& wormIds);
    void setClearBaselinesAtStart(bool clearAtStart);
    void setSharedStorageMutex(const QSharedPointer<QMutex>& mutex);
    void setSkipMergedFrames(bool skip);
    void setFps(double fps);
    // Fraction of within-window steps allowed to point against the majority direction
    // before the segment is considered ambiguous and skipped (0 = any reversal skips,
    // 1 = never skip). Default 0.25.
    void setMaxReversalFraction(float fraction);

public slots:
    void doWork();

signals:
    void progress(int percentage);
    void finished();
    void failed(const QString& reason);
    // Emitted once per worm after head/tail refinement; lists the frame numbers
    // whose centerline was reversed relative to the processFrame assignment.
    void headTailSwapEvent(int wormId, QList<int> swappedFrames);

private:
    QMap<int, Tracking::DetectedBlob> getDetectedBlobsForFrame(int frameNumber) const;
    QList<QList<int>> getMergeGroupsForFrame(int frameNumber) const;
    Centerline::TipFeatureBaseline getTipBaseline(int wormId) const;
    void setDetectedBlobForFrame(int frameNumber, int wormId, const Tracking::DetectedBlob& blob);
    void recordTipFeatureSample(int wormId, float curvatureMagnitude, float width);
    void recordBodyLengthSample(int wormId, float length);
    void setCenterlineDebugFrame(const Debug::CenterlineFrameDebug& record);

    TrackingDataStorage* m_storage;
    Debug::DebugDataStore* m_debugStore;
    Centerline::CenterlineSnakeParams m_snakeParams;
    QList<int> m_wormIds;
    bool m_clearBaselinesAtStart = true;
    bool m_skipMergedFrames = false;
    double m_fps = 25.0;
    float m_maxReversalFraction = 0.25f;
    QSharedPointer<QMutex> m_sharedStorageMutex;
};

#endif // CENTERLINEWORKER_H
