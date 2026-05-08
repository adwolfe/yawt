#include "centerlineworker.h"
#include "../data/trackingcommon.h"
#include <QDebug>

CenterlineWorker::CenterlineWorker(TrackingDataStorage* storage, QObject* parent)
    : QObject(parent), m_storage(storage) {}

void CenterlineWorker::doWork() {
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
        std::vector<Tracking::WormTrackPoint> trackPoints = it->second;
        bool trackModified = false;

        for (Tracking::WormTrackPoint& tp : trackPoints) {
            if (tp.quality == Tracking::TrackPointQuality::Merged ||
                tp.quality == Tracking::TrackPointQuality::Lost) {
                continue;
            }

            QMap<int, Tracking::DetectedBlob> frameBlobs =
                m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
            if (!frameBlobs.contains(wormId)) continue;

            Tracking::DetectedBlob blob = frameBlobs[wormId];
            if (!blob.isValid || blob.contourPoints.empty()) continue;

            // Ensure centerline is computed (may already be set by setDetectedBlobForFrame).
            if (blob.centerlinePoints.empty()) {
                Tracking::populateCenterlineFromContour(blob);
                m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);
            }

            // For ring-shaped (coiled) blobs, snap track position to the centerline midpoint.
            // The geometric centroid falls in the hole, so the midpoint of the skeleton is a
            // better representative of where the worm body actually is.
            if (!blob.holeContourPoints.empty() && !blob.centerlinePoints.empty()) {
                int mid = static_cast<int>(blob.centerlinePoints.size()) / 2;
                tp.position = blob.centerlinePoints[static_cast<size_t>(mid)];
                trackModified = true;
            }
        }

        if (trackModified) {
            m_storage->setTrackForItem(wormId, trackPoints);
        }

        processedWorms++;
        emit progress(processedWorms * 100 / totalWorms);
    }

    emit finished();
}
