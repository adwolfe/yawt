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
 */
class CenterlineWorker : public QObject {
    Q_OBJECT

public:
    explicit CenterlineWorker(TrackingDataStorage* storage, QObject* parent = nullptr);

public slots:
    void doWork();

signals:
    void progress(int percentage);
    void finished();
    void failed(const QString& reason);

private:
    TrackingDataStorage* m_storage;
};

#endif // CENTERLINEWORKER_H
