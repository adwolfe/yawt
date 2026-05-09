#ifndef CENTERLINEWORKER_H
#define CENTERLINEWORKER_H

#include <QObject>
#include "../data/trackingdatastorage.h"
#include "../data/trackingcommon.h"

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
    explicit CenterlineWorker(TrackingDataStorage* storage, QObject* parent = nullptr);

    /**
     * @brief Configure the active-contour refinement used on ring/coiled frames.
     *        Must be called before doWork() to take effect.
     */
    void setSnakeParams(const Tracking::CenterlineSnakeParams& params);

public slots:
    void doWork();

signals:
    void progress(int percentage);
    void finished();
    void failed(const QString& reason);

private:
    TrackingDataStorage* m_storage;
    Tracking::CenterlineSnakeParams m_snakeParams;
};

#endif // CENTERLINEWORKER_H
