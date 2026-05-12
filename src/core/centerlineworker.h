#ifndef CENTERLINEWORKER_H
#define CENTERLINEWORKER_H

#include <QObject>
#include <QString>
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

    /**
     * @brief Diagnostic: re-runs the per-frame centerline pipeline for one
     *        (wormId, frameNumber) using the supplied snake params, and writes
     *        per-stage visualisation images plus a decision log to outputDir.
     *
     * Reads from `storage` only; never writes back. Predictor state is
     *  reconstructed from the worm's blob in the immediately adjacent frame
     * (frameNumber-1 if available, otherwise frameNumber+1) so head/tail
     * assignment behaves like the live sweep on that frame.
     *
     * @return true on success. On false, *outErrorMsg (if non-null) is filled.
     */
    static bool exportProcessForFrame(TrackingDataStorage* storage,
                                      int wormId,
                                      int frameNumber,
                                      const Tracking::CenterlineSnakeParams& snakeParams,
                                      const QString& outputDir,
                                      QString* outErrorMsg = nullptr);

public slots:
    void doWork();

signals:
    void progress(int percentage);
    void finished();
    void failed(const QString& reason);

private:
    // The legacy 5-pass pipeline. Kept reachable behind a compile-time
    // toggle in doWork() so the new 2-sweep pipeline (doWorkNew) can be
    // A/B-tested against it during the rewrite.
    void doWorkLegacy();

    // The new 2-sweep / 5-step pipeline. See CENTERLINE_REWRITE_PLAN.md.
    void doWorkNew();

    TrackingDataStorage* m_storage;
    Tracking::CenterlineSnakeParams m_snakeParams;
};

#endif // CENTERLINEWORKER_H
