#pragma once

#include "sharedblob.h"
#include "../data/wormsjsoncodec.h"
#include <QObject>
#include <memory>

// Value-only input: no pointers or callbacks into the live storage or manager.
// Qt maps are shared on capture and detach if the UI subsequently edits them.
struct TrackingExportSnapshot {
    WormsJson::Document worms;
    QList<TableItems::AnnotationItem> items;
    QString directory;
    QString workbookPath;
    double fps = 0.0;
    double umPerPixel = 0.0;
    cv::Size frameSize;
    bool centerlineEnabled = true;
    bool skipMergedFrames = false;
    float maxReversalFraction = 0.25f;
    int nextSharedBlobId = 1;
    QMap<int, QList<SharedBlob>> mergeRecords;
    QMap<int, QMap<int, Tracking::DetectedBlob>> splitResolutions;
    QMap<int, int> wormToSharedBlobId;
    QMap<int, QList<int>> headTailSwaps;
    QMap<int, QList<int>> centerlineReviews;
    QMap<int, QList<int>> motionSwaps;
    QMap<int, QList<int>> geometrySwaps;

    const Tracking::DetectedBlob* findBlob(int frame, int wormId) const;
    QJsonObject mergeStateToJson() const;
};

class TrackingExportWorker : public QObject {
    Q_OBJECT
public:
    explicit TrackingExportWorker(TrackingExportSnapshot snapshot,
                                  std::shared_ptr<void> retiredData = {});
public slots:
    void run();
signals:
    void progress(int percentage, const QString& status);
    // Empty error means every required output was committed successfully.
    void finished(const QString& error);
private:
    struct WormStatistics {
        int single = 0, split = 0, merged = 0, lost = 0;
        int centerlineComputed = 0, centerlineSkipped = 0;
        int cleanTopology = 0, selfCrossedTopology = 0;
        int mergeRuns = 0, longestMergeRun = 0;
        double totalDistance = 0.0, netDisplacement = 0.0;
        double meanArea = 0.0, meanBodyLength = 0.0, meanAspectRatio = 0.0;
    };
    void collectStatistics();
    bool outputTracksToWorkbook(const Tracking::AllWormTracks& tracks, const QString& path) const;
    bool exportHeadTailSwapXlsx(const QMap<int, QList<int>>& swaps,
                               const QMap<int, QList<int>>& reviews, const QString& path) const;
    bool exportProcessingSummary(const QString& path) const;
    bool saveWormSummaryJson(const QString& directory) const;
    TrackingExportSnapshot m_snapshot;
    QMap<int, WormStatistics> m_statistics;
    // Transient video/track buffers are destroyed on this worker's thread.
    std::shared_ptr<void> m_retiredData;
};
