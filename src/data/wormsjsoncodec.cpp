#include "wormsjsoncodec.h"

#include "../utils/yawtjsonio.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <algorithm>
#include <cmath>

namespace WormsJson {

namespace {

QJsonObject pointObj(double x, double y)
{
    QJsonObject o;
    o["x"] = x;
    o["y"] = y;
    return o;
}

QJsonObject rectObj(const QRectF& r)
{
    QJsonObject o;
    o["x"] = r.x();
    o["y"] = r.y();
    o["width"] = r.width();
    o["height"] = r.height();
    return o;
}

QRectF rectFrom(const QJsonObject& o)
{
    return QRectF(o.value("x").toDouble(), o.value("y").toDouble(),
                  o.value("width").toDouble(), o.value("height").toDouble());
}

cv::Point2f point2fFrom(const QJsonObject& o)
{
    return cv::Point2f(static_cast<float>(o.value("x").toDouble()),
                       static_cast<float>(o.value("y").toDouble()));
}

float arcLength(const std::vector<cv::Point2f>& pts)
{
    double len = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        const cv::Point2f d = pts[i] - pts[i - 1];
        len += std::sqrt(static_cast<double>(d.x * d.x + d.y * d.y));
    }
    return static_cast<float>(len);
}

std::vector<cv::Point2f> pointArrayFrom(const QJsonArray& arr)
{
    std::vector<cv::Point2f> pts;
    pts.reserve(static_cast<size_t>(arr.size()));
    for (const QJsonValue& v : arr) {
        const QJsonArray a = v.toArray();
        if (a.size() >= 2) {
            pts.emplace_back(static_cast<float>(a[0].toDouble()),
                             static_cast<float>(a[1].toDouble()));
        }
    }
    return pts;
}

QJsonObject headerJson(const QString& videoPath, int keyFrame)
{
    QJsonObject root;
    root["version"] = 1;
    root["videoPath"] = videoPath;
    root["keyFrame"] = keyFrame;
    return root;
}

bool readRoot(const QString& filePath, QJsonObject& root, QString* error)
{
    QJsonParseError parseError;
    QString ioError;
    const QJsonDocument doc = YawtJsonIO::readJsonDocument(filePath, &parseError, &ioError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (error) {
            *error = ioError.isEmpty() ? parseError.errorString() : ioError;
        }
        return false;
    }
    root = doc.object();
    return true;
}

} // namespace

// ── Items ────────────────────────────────────────────────────────────────────

QJsonObject itemToJson(const TableItems::ClickedItem& item)
{
    QJsonObject o;
    o["id"] = item.id;
    o["type"] = TableItems::itemTypeToString(item.type);
    o["visible"] = item.visible;
    o["frameOfSelection"] = item.frameOfSelection;

    QJsonObject color;
    color["r"] = item.color.red();
    color["g"] = item.color.green();
    color["b"] = item.color.blue();
    color["a"] = item.color.alpha();
    color["hex"] = item.color.name(QColor::HexArgb);
    o["color"] = color;

    o["initialCentroid"] = pointObj(item.initialCentroid.x(), item.initialCentroid.y());
    o["initialBoundingBox"] = rectObj(item.initialBoundingBox);
    o["originalClickedBoundingBox"] = rectObj(item.originalClickedBoundingBox);
    return o;
}

TableItems::ClickedItem itemFromJson(const QJsonObject& obj)
{
    TableItems::ClickedItem item;
    item.id = obj.value("id").toInt();
    item.type = TableItems::stringToItemType(obj.value("type").toString());
    item.visible = obj.value("visible").toBool(true);
    item.frameOfSelection = obj.value("frameOfSelection").toInt(0);

    if (obj.value("color").isObject()) {
        const QJsonObject c = obj["color"].toObject();
        if (c.contains("r") && c.contains("g") && c.contains("b")) {
            item.color = QColor(c.value("r").toInt(0), c.value("g").toInt(0),
                                c.value("b").toInt(0), c.value("a").toInt(255));
        } else if (c.contains("hex")) {
            item.color = QColor(c.value("hex").toString());
        }
    }
    if (obj.value("initialCentroid").isObject()) {
        const QJsonObject c = obj["initialCentroid"].toObject();
        item.initialCentroid = QPointF(c.value("x").toDouble(), c.value("y").toDouble());
    }
    if (obj.value("initialBoundingBox").isObject()) {
        item.initialBoundingBox = rectFrom(obj["initialBoundingBox"].toObject());
    }
    if (obj.value("originalClickedBoundingBox").isObject()) {
        item.originalClickedBoundingBox = rectFrom(obj["originalClickedBoundingBox"].toObject());
    }
    return item;
}

// ── Track points ─────────────────────────────────────────────────────────────

QJsonObject trackPointToJson(const Tracking::WormTrackPoint& p,
                             const Tracking::DetectedBlob* blob)
{
    QJsonObject o;
    o["frame"] = p.frameNumber;
    o["quality"] = static_cast<int>(p.quality);
    o["position"] = pointObj(static_cast<double>(p.position.x),
                             static_cast<double>(p.position.y));
    o["roi"] = rectObj(p.searchWindow);

    // Derived morphology lives on the point itself (TrackingDataStorage joins it
    // in from the blob store); write what the point holds.
    if (p.area > 0.f)        o["area"] = static_cast<double>(p.area);
    if (p.aspectRatio > 0.f) o["aspectRatio"] = static_cast<double>(p.aspectRatio);
    if (p.bodyLength > 0.f)  o["bodyLength"] = static_cast<double>(p.bodyLength);
    if (p.hasTips) {
        QJsonObject tips;
        tips["head"] = pointObj(static_cast<double>(p.headTip.x), static_cast<double>(p.headTip.y));
        tips["tail"] = pointObj(static_cast<double>(p.tailTip.x), static_cast<double>(p.tailTip.y));
        o["tips"] = tips;
    }

    if (blob) {
        o["detectedBlob"] = Tracking::detectedBlobToJson(*blob);
    }
    return o;
}

Tracking::WormTrackPoint trackPointFromJson(const QJsonObject& obj,
                                            Tracking::DetectedBlob* outBlob,
                                            bool* outHasBlob)
{
    if (outHasBlob) *outHasBlob = false;

    Tracking::WormTrackPoint p;
    p.frameNumber = obj.value("frame").toInt();
    if (obj.value("position").isObject()) {
        p.position = point2fFrom(obj["position"].toObject());
    }
    if (obj.value("roi").isObject()) {
        p.searchWindow = rectFrom(obj["roi"].toObject());
    }
    p.quality = static_cast<Tracking::TrackPointQuality>(
        obj.value("quality").toInt(static_cast<int>(Tracking::TrackPointQuality::Single)));

    if (obj.contains("area"))        p.area = static_cast<float>(obj.value("area").toDouble());
    if (obj.contains("aspectRatio")) p.aspectRatio = static_cast<float>(obj.value("aspectRatio").toDouble());
    if (obj.contains("bodyLength"))  p.bodyLength = static_cast<float>(obj.value("bodyLength").toDouble());

    if (obj.value("tips").isObject()) {
        const QJsonObject tips = obj["tips"].toObject();
        if (tips.contains("head") && tips.contains("tail")) {
            p.headTip = point2fFrom(tips["head"].toObject());
            p.tailTip = point2fFrom(tips["tail"].toObject());
            p.hasTips = true;
        }
    }

    // Blob: current combined object, or the legacy centerline-only array.
    Tracking::DetectedBlob blob;
    bool hasBlob = false;
    if (obj.value("detectedBlob").isObject()) {
        blob = Tracking::detectedBlobFromJson(obj["detectedBlob"].toObject());
        hasBlob = true;
    } else if (obj.value("centerlinePoints").isArray()) {
        blob.centerlinePoints = pointArrayFrom(obj["centerlinePoints"].toArray());
        hasBlob = blob.centerlinePoints.size() >= 2;
    }

    if (hasBlob) {
        // Old files may carry blobs with unset validity/centroid/bbox; complete
        // them from the point so the blob store never holds an unusable entry.
        blob.isValid = true;
        if (blob.centroid.isNull()) {
            blob.centroid = QPointF(static_cast<double>(p.position.x),
                                    static_cast<double>(p.position.y));
        }
        if (blob.boundingBox.isNull()) {
            blob.boundingBox = p.searchWindow;
        }
        if (p.bodyLength <= 0.f && blob.centerlinePoints.size() >= 2) {
            p.bodyLength = arcLength(blob.centerlinePoints);
        }
        if (outBlob) *outBlob = std::move(blob);
        if (outHasBlob) *outHasBlob = true;
    }
    return p;
}

// ── Whole document ───────────────────────────────────────────────────────────

QJsonObject toJson(const Document& doc)
{
    QJsonObject root = headerJson(doc.videoPath, doc.keyFrame);
    root["version"] = doc.version;

    if (doc.metrics.valid) {
        QJsonObject m;
        m["roiSizeMultiplier"] = doc.metrics.roiSizeMultiplier;
        QJsonObject fixed;
        fixed["width"] = doc.metrics.fixedRoiSize.width();
        fixed["height"] = doc.metrics.fixedRoiSize.height();
        m["currentFixedRoiSize"] = fixed;
        m["minObservedArea"] = doc.metrics.minObservedArea;
        m["maxObservedArea"] = doc.metrics.maxObservedArea;
        m["minObservedAspectRatio"] = doc.metrics.minObservedAspectRatio;
        m["maxObservedAspectRatio"] = doc.metrics.maxObservedAspectRatio;
        root["metrics"] = m;
    }

    QJsonArray items;
    for (const TableItems::ClickedItem& item : doc.items) items.append(itemToJson(item));
    root["items"] = items;
    root["itemsCount"] = items.size();

    auto lookupBlob = [&](int frame, int wormId) -> const Tracking::DetectedBlob* {
        if (doc.blobLookup) return doc.blobLookup(frame, wormId);
        auto fit = doc.blobsByFrame.constFind(frame);
        if (fit == doc.blobsByFrame.constEnd()) return nullptr;
        auto wit = fit->constFind(wormId);
        return wit == fit->constEnd() ? nullptr : &wit.value();
    };

    QJsonObject tracks;
    for (const auto& [wormId, points] : doc.tracks) {
        QJsonArray arr;
        for (const Tracking::WormTrackPoint& p : points) {
            arr.append(trackPointToJson(p, lookupBlob(p.frameNumber, wormId)));
        }
        tracks[QString::number(wormId)] = arr;
    }
    root["tracks"] = tracks;
    root["tracksCount"] = static_cast<int>(doc.tracks.size());

    QJsonObject merge;
    for (auto it = doc.mergeGroupsByFrame.constBegin(); it != doc.mergeGroupsByFrame.constEnd(); ++it) {
        QJsonArray groups;
        for (const QList<int>& group : it.value()) {
            QJsonArray g;
            for (int id : group) g.append(id);
            groups.append(g);
        }
        merge[QString::number(it.key())] = groups;
    }
    root["mergeGroupsByFrame"] = merge;

    if (!doc.mergeState.isEmpty()) root["mergeState"] = doc.mergeState;
    return root;
}

Document fromJson(const QJsonObject& root)
{
    Document doc;
    doc.version = root.value("version").toInt(1);
    doc.videoPath = root.value("videoPath").toString();
    doc.keyFrame = root.value("keyFrame").toInt(-1);

    if (root.value("metrics").isObject()) {
        const QJsonObject m = root["metrics"].toObject();
        Metrics& mt = doc.metrics;
        mt.valid = true;
        mt.roiSizeMultiplier = m.value("roiSizeMultiplier").toDouble(mt.roiSizeMultiplier);
        if (m.value("currentFixedRoiSize").isObject()) {
            const QJsonObject f = m["currentFixedRoiSize"].toObject();
            mt.fixedRoiSize = QSizeF(f.value("width").toDouble(), f.value("height").toDouble());
        }
        mt.minObservedArea = m.value("minObservedArea").toDouble(mt.minObservedArea);
        mt.maxObservedArea = m.value("maxObservedArea").toDouble(mt.maxObservedArea);
        mt.minObservedAspectRatio = m.value("minObservedAspectRatio").toDouble(mt.minObservedAspectRatio);
        mt.maxObservedAspectRatio = m.value("maxObservedAspectRatio").toDouble(mt.maxObservedAspectRatio);
    }

    for (const QJsonValue& v : root.value("items").toArray()) {
        if (v.isObject()) doc.items.append(itemFromJson(v.toObject()));
    }

    const QJsonObject tracks = root.value("tracks").toObject();
    for (auto it = tracks.constBegin(); it != tracks.constEnd(); ++it) {
        bool ok = false;
        const int wormId = it.key().toInt(&ok);
        if (!ok || !it.value().isArray()) continue;
        const QJsonArray arr = it.value().toArray();
        std::vector<Tracking::WormTrackPoint> points;
        points.reserve(static_cast<size_t>(arr.size()));
        for (const QJsonValue& pv : arr) {
            if (!pv.isObject()) continue;
            Tracking::DetectedBlob blob;
            bool hasBlob = false;
            Tracking::WormTrackPoint p = trackPointFromJson(pv.toObject(), &blob, &hasBlob);
            if (hasBlob) doc.blobsByFrame[p.frameNumber][wormId] = std::move(blob);
            points.push_back(p);
        }
        std::sort(points.begin(), points.end(),
                  [](const Tracking::WormTrackPoint& a, const Tracking::WormTrackPoint& b) {
                      return a.frameNumber < b.frameNumber;
                  });
        doc.tracks[wormId] = std::move(points);
    }

    const QJsonObject merge = root.value("mergeGroupsByFrame").toObject();
    for (auto it = merge.constBegin(); it != merge.constEnd(); ++it) {
        bool ok = false;
        const int frame = it.key().toInt(&ok);
        if (!ok || !it.value().isArray()) continue;
        QList<QList<int>> groups;
        for (const QJsonValue& gv : it.value().toArray()) {
            if (!gv.isArray()) continue;
            QList<int> group;
            for (const QJsonValue& idv : gv.toArray()) group.append(idv.toInt());
            groups.append(group);
        }
        doc.mergeGroupsByFrame.insert(frame, groups);
    }

    doc.mergeState = root.value("mergeState").toObject();
    return doc;
}

bool write(const QString& filePath, const Document& doc, QString* error)
{
    return YawtJsonIO::writeCompressedJsonDocument(filePath, QJsonDocument(toJson(doc)), error);
}

bool read(const QString& filePath, Document& outDoc, QString* error)
{
    QJsonObject root;
    if (!readRoot(filePath, root, error)) return false;
    outDoc = fromJson(root);
    return true;
}

// ── roi_points.json ──────────────────────────────────────────────────────────

bool writeRoiPoints(const QString& filePath, const QString& videoPath, int keyFrame,
                    const QList<TableItems::ClickedItem>& items, QString* error)
{
    QJsonObject root = headerJson(videoPath, keyFrame);
    QJsonArray arr;
    for (const TableItems::ClickedItem& item : items) arr.append(itemToJson(item));
    root["items"] = arr;
    root["itemsCount"] = arr.size();

    QFile f(filePath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = f.errorString();
        return false;
    }
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

QList<TableItems::ClickedItem> readRoiPoints(const QString& filePath)
{
    QList<TableItems::ClickedItem> items;
    QJsonObject root;
    if (!readRoot(filePath, root, nullptr)) return items;
    for (const QJsonValue& v : root.value("items").toArray()) {
        if (v.isObject()) items.append(itemFromJson(v.toObject()));
    }
    return items;
}

// ── Light readers ────────────────────────────────────────────────────────────

QList<int> readWormIds(const QString& filePath)
{
    QList<int> ids;
    QJsonObject root;
    if (!readRoot(filePath, root, nullptr)) return ids;
    for (const QJsonValue& v : root.value("items").toArray()) {
        if (!v.isObject()) continue;
        const QJsonObject obj = v.toObject();
        // stringToItemType maps the legacy "Fix" type to Worm.
        if (TableItems::stringToItemType(obj.value("type").toString()) == TableItems::ItemType::Worm) {
            ids.append(obj.value("id").toInt());
        }
    }
    return ids;
}

Tracking::AllWormTracks readTracks(const QString& filePath)
{
    Tracking::AllWormTracks tracks;
    QJsonObject root;
    if (!readRoot(filePath, root, nullptr)) return tracks;

    const QJsonObject tracksObj = root.value("tracks").toObject();
    for (auto it = tracksObj.constBegin(); it != tracksObj.constEnd(); ++it) {
        bool ok = false;
        const int wormId = it.key().toInt(&ok);
        if (!ok || !it.value().isArray()) continue;
        std::vector<Tracking::WormTrackPoint> points;
        for (const QJsonValue& pv : it.value().toArray()) {
            if (!pv.isObject()) continue;
            // Blob geometry is parsed only far enough to recover bodyLength on
            // legacy files; it is not kept.
            Tracking::DetectedBlob scratch;
            points.push_back(trackPointFromJson(pv.toObject(), &scratch, nullptr));
        }
        if (points.empty()) continue;
        std::sort(points.begin(), points.end(),
                  [](const Tracking::WormTrackPoint& a, const Tracking::WormTrackPoint& b) {
                      return a.frameNumber < b.frameNumber;
                  });
        tracks[wormId] = std::move(points);
    }
    return tracks;
}

QJsonObject readMergeState(const QString& filePath)
{
    QJsonObject root;
    if (!readRoot(filePath, root, nullptr)) return {};
    return root.value("mergeState").toObject();
}

} // namespace WormsJson
