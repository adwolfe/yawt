#pragma once

#include "trackingcommon.h"
#include "../core/centerlinetypes.h"   // Centerline::TipFeatureBaseline

#include <QJsonObject>
#include <QList>
#include <QMap>
#include <QSizeF>
#include <QString>
#include <functional>

/**
 * WormsJson — the single reader and writer for a run's worms.json and
 * roi_points.json. Every component that touches these files goes through here:
 *
 *   TrackingManager        writes both files (Document + mergeState section)
 *   TrackingDataStorage    reads both files into memory
 *   AnalysisSessionModel   light reads: worm ids, tracks, reference points
 *
 * worms.json (version 1), compressed via YawtJsonIO:
 *   version, videoPath, keyFrame
 *   metrics             { roiSizeMultiplier, currentFixedRoiSize{width,height},
 *                         minObservedArea, maxObservedArea,
 *                         minObservedAspectRatio, maxObservedAspectRatio }
 *   items[]             worm items only                       -> itemToJson()
 *   tracks              { "<wormId>": [ point, ... ] }        -> trackPointToJson()
 *   mergeGroupsByFrame  { "<frame>": [ [wormId, ...], ... ] }
 *   tipBaselines        { "<wormId>": { meanAbsCurvature, m2AbsCurvature, curvatureSamples,
 *                         meanWidth, m2Width, widthSamples, meanBodyLength, m2BodyLength,
 *                         lengthSamples } }   present once the centerline pass has run
 *   mergeState          opaque TrackingManager section (physical blobs, split
 *                       resolutions); passed through untouched
 *
 * roi_points.json (version 1), indented:
 *   version, videoPath, keyFrame, items[] (ROI and Start/End/Center items)
 *
 * A track point:
 *   frame, quality, position{x,y}, roi{x,y,width,height}   (roi = search window)
 *   area, aspectRatio, bodyLength   optional, present when derived
 *   tips{head{x,y},tail{x,y}}       optional
 *   blob{...}                       optional, Tracking::blobGeometryToJson()
 *   centerline{...}                 optional, Tracking::blobCenterlineToJson(); only
 *                                   written once the centerline pass has run
 *   detectedBlob{...}               legacy combined object (read only)
 *   centerlinePoints[[x,y],...]     oldest layout (read only)
 */
namespace WormsJson {

struct Metrics {
    bool   valid = false;              // true when the file carried a metrics section
    double roiSizeMultiplier = 1.0;
    QSizeF fixedRoiSize;               // may be invalid even when valid == true
    double minObservedArea = 0.0;
    double maxObservedArea = 0.0;
    double minObservedAspectRatio = 0.0;
    double maxObservedAspectRatio = 0.0;
};

/** In-memory image of a worms.json file. */
struct Document {
    int     version = 1;
    QString videoPath;
    int     keyFrame = -1;
    Metrics metrics;
    QList<TableItems::ClickedItem> items;                        // worm items
    Tracking::AllWormTracks tracks;                              // sorted by frame after read
    QMap<int, QMap<int, Tracking::DetectedBlob>> blobsByFrame;   // frame -> wormId -> blob (filled on read)
    QMap<int, QList<QList<int>>> mergeGroupsByFrame;
    QMap<int, Centerline::TipFeatureBaseline> tipBaselines;    // per-worm Welford stats (Phase A)
    QJsonObject mergeState;                                      // TrackingManager's section, passed through

    /**
     * Optional blob source used when WRITING, so a caller holding blobs in a
     * store need not copy them into blobsByFrame first. When unset, toJson()
     * falls back to blobsByFrame.
     */
    std::function<const Tracking::DetectedBlob*(int frameNumber, int wormId)> blobLookup;
};

// ── Element codecs (shared by worms.json and roi_points.json) ────────────────
QJsonObject itemToJson(const TableItems::ClickedItem& item);
TableItems::ClickedItem itemFromJson(const QJsonObject& obj);

QJsonObject trackPointToJson(const Tracking::WormTrackPoint& p,
                             const Tracking::DetectedBlob* blob);
/**
 * Parse one track point. When @p outBlob is non-null and the point carries a
 * blob ("blob"/"centerline", legacy "detectedBlob" or "centerlinePoints"), it is written there
 * and @p outHasBlob is set. bodyLength is taken from the file when present and
 * otherwise computed from the centerline so legacy files still yield it.
 */
Tracking::WormTrackPoint trackPointFromJson(const QJsonObject& obj,
                                            Tracking::DetectedBlob* outBlob = nullptr,
                                            bool* outHasBlob = nullptr);

// ── Whole documents ──────────────────────────────────────────────────────────
QJsonObject toJson(const Document& doc);
Document    fromJson(const QJsonObject& root);

bool write(const QString& filePath, const Document& doc, QString* error = nullptr);
bool read(const QString& filePath, Document& outDoc, QString* error = nullptr);

// ── roi_points.json ──────────────────────────────────────────────────────────
bool writeRoiPoints(const QString& filePath, const QString& videoPath, int keyFrame,
                    const QList<TableItems::ClickedItem>& items, QString* error = nullptr);
QList<TableItems::ClickedItem> readRoiPoints(const QString& filePath);

// ── Light readers for the Analysis tab ───────────────────────────────────────
QList<int>              readWormIds(const QString& filePath);
Tracking::AllWormTracks readTracks(const QString& filePath);    // no blob geometry materialised
QJsonObject             readMergeState(const QString& filePath); // empty when absent

} // namespace WormsJson
