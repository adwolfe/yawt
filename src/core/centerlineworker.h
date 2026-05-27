#ifndef CENTERLINEWORKER_H
#define CENTERLINEWORKER_H

#include <QObject>
#include <QList>
#include <QMutex>
#include <QSharedPointer>
#include <QString>
#include "../data/trackingdatastorage.h"
#include "../data/trackingcommon.h"

namespace Debug {
class DebugDataStore;
struct CenterlineFrameDebug;
}

/**
 * @class CenterlineWorker
 * @brief Background worker that populates centerline data for all non-merged track points.
 *
 * Runs on a QThread after tracking completes. Iterates over all stored blobs, calls
 * populateCenterlineFromContour() for blobs that have contour data but no centerline,
 * and writes the result back to storage via setDetectedBlobForFrame().
 *
 * Skips frames where TrackPointQuality is Merged or Lost, as those have ambiguous blobs.
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
    void setSnakeParams(const Tracking::CenterlineSnakeParams& params);
    void setWormIds(const QList<int>& wormIds);
    void setClearBaselinesAtStart(bool clearAtStart);
    void setSharedStorageMutex(const QSharedPointer<QMutex>& mutex);

public slots:
    void doWork();

signals:
    void progress(int percentage);
    void finished();
    void failed(const QString& reason);

private:
    QMap<int, Tracking::DetectedBlob> getDetectedBlobsForFrame(int frameNumber) const;
    QList<QList<int>> getMergeGroupsForFrame(int frameNumber) const;
    Tracking::TipFeatureBaseline getTipBaseline(int wormId) const;
    void setDetectedBlobForFrame(int frameNumber, int wormId, const Tracking::DetectedBlob& blob);
    void recordTipFeatureSample(int wormId, float curvatureMagnitude, float width);
    void recordBodyLengthSample(int wormId, float length);
    void setCenterlineDebugFrame(const Debug::CenterlineFrameDebug& record);

    TrackingDataStorage* m_storage;
    Debug::DebugDataStore* m_debugStore;
    Tracking::CenterlineSnakeParams m_snakeParams;
    QList<int> m_wormIds;
    bool m_clearBaselinesAtStart = true;
    QSharedPointer<QMutex> m_sharedStorageMutex;
};

#endif // CENTERLINEWORKER_H
