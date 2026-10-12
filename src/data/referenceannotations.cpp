#include "referenceannotations.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ReferenceAnnotations {
namespace {
QJsonArray point(const QPointF& p) { return {p.x(), p.y()}; }

int integer(const QJsonValue& value, int minimum, int maximum) {
    const double n = value.toDouble(-1);
    if (!value.isDouble() || !std::isfinite(n) || n != std::floor(n) || n < minimum || n > maximum)
        throw std::runtime_error("Invalid integer in reference annotations.");
    return static_cast<int>(n);
}

QString choice(const QJsonValue& value, const QStringList& choices) {
    if (!value.isString() || !choices.contains(value.toString()))
        throw std::runtime_error("Invalid annotation label.");
    return value.toString();
}

QPointF readPoint(const QJsonValue& value, const QSize& size) {
    const auto a = value.toArray();
    if (a.size() != 2 || !a[0].isDouble() || !a[1].isDouble())
        throw std::runtime_error("Invalid reference point.");
    const QPointF p(a[0].toDouble(), a[1].toDouble());
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || p.x() < 0 || p.y() < 0 ||
        p.x() > size.width() - 1 || p.y() > size.height() - 1)
        throw std::runtime_error("Reference point is outside the video.");
    return p;
}

QJsonObject serialize(const Document& d) {
    QJsonArray records;
    for (auto f = d.frames.cbegin(); f != d.frames.cend(); ++f) {
        for (auto w = f->cbegin(); w != f->cend(); ++w) {
            const Frame& a = w.value();
            QJsonObject record{{"frame", f.key()}, {"worm_id", w.key()},
                {"provenance", "manual"}, {"review", a.review},
                {"orientation", a.orientation}, {"visibility", a.visibility},
                {"event", a.event}, {"notes", a.notes}};
            if (a.position) {
                record.insert("position", point(*a.position));
                record.insert("position_kind", "approximate_center");
            }
            if (a.endA) record.insert("end_a", point(*a.endA));
            if (a.endB) record.insert("end_b", point(*a.endB));
            QJsonArray line;
            for (const auto& p : a.centerline) line.append(point(p));
            record.insert("centerline", line);
            records.append(record);
        }
    }
    return {{"format", "yawt_reference_annotations"}, {"schema_version", 1},
        {"frame_index_base", 0}, {"coordinate_system", "loaded_video_pixels"},
        {"centerline_order", "a_to_b"},
        {"video", QJsonObject{{"path", d.video.path}, {"width", d.video.size.width()},
            {"height", d.video.size.height()}, {"frame_count", d.video.frameCount},
            {"fps", d.video.fps}, {"byte_size", static_cast<double>(d.video.byteSize)}}},
        {"source_mapping", QJsonObject{{"path", d.source.path},
            {"crop_x", d.source.cropOrigin.x()}, {"crop_y", d.source.cropOrigin.y()},
            {"first_frame", d.source.firstFrame}}}, {"annotations", records}};
}
} // namespace

QString defaultPath(const QString& videoPath) { return videoPath + ".reference.json"; }

bool save(const QString& path, const Document& document, QString* error) {
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    const auto bytes = QJsonDocument(serialize(document)).toJson(QJsonDocument::Indented);
    if (file.write(bytes) != bytes.size() || !file.commit()) {
        if (error) *error = file.errorString();
        return false;
    }
    return true;
}

bool load(const QString& path, const Video& expected, Document* output, QString* error) {
    try {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error(file.errorString().toStdString());
        QJsonParseError parseError;
        const auto json = QJsonDocument::fromJson(file.readAll(), &parseError);
        if (parseError.error != QJsonParseError::NoError || !json.isObject())
            throw std::runtime_error("Invalid reference JSON.");
        const auto root = json.object();
        if (root["format"] != "yawt_reference_annotations" ||
            integer(root["schema_version"], 1, 1) != 1 ||
            integer(root["frame_index_base"], 0, 0) != 0 ||
            root["coordinate_system"] != "loaded_video_pixels" || root["centerline_order"] != "a_to_b")
            throw std::runtime_error("Unsupported reference annotation format.");
        const auto video = root["video"].toObject();
        const int max = std::numeric_limits<int>::max();
        const QSize size(integer(video["width"], 1, max), integer(video["height"], 1, max));
        const int frames = integer(video["frame_count"], 1, max);
        const double fps = video["fps"].toDouble(-1);
        if (!video["path"].isString() || !video["byte_size"].isDouble() ||
            QFileInfo(video["path"].toString()).fileName() != QFileInfo(expected.path).fileName() ||
            size != expected.size || frames != expected.frameCount || !std::isfinite(fps) || fps <= 0 ||
            std::abs(fps - expected.fps) > 0.001 || video["byte_size"].toDouble() != expected.byteSize)
            throw std::runtime_error("Reference annotations belong to a different video (name, size, frames, or FPS differ).");
        Document d;
        d.video = expected; // Update the local path when the video and sidecar were moved together.
        const auto source = root["source_mapping"].toObject();
        if (!source["path"].isString()) throw std::runtime_error("Missing source-video mapping.");
        d.source.path = source["path"].toString();
        d.source.cropOrigin = QPoint(integer(source["crop_x"], 0, max), integer(source["crop_y"], 0, max));
        d.source.firstFrame = integer(source["first_frame"], 0, max - frames);
        if (!root["annotations"].isArray()) throw std::runtime_error("Missing reference annotations.");
        for (const auto& value : root["annotations"].toArray()) {
            if (!value.isObject()) throw std::runtime_error("Invalid reference annotation.");
            const auto record = value.toObject();
            const int f = integer(record["frame"], 0, frames - 1);
            const int id = integer(record["worm_id"], 1, max);
            if (d.frames.value(f).contains(id)) throw std::runtime_error("Duplicate worm/frame reference.");
            if (record["provenance"] != "manual") throw std::runtime_error("Unsupported annotation provenance.");
            Frame a;
            a.orientation = choice(record["orientation"], {"unknown", "a_is_head", "b_is_head"});
            a.visibility = choice(record["visibility"], {"visible", "partial", "hidden"});
            a.event = choice(record["event"], {"none", "self_contact", "merge", "split", "lost", "reversal"});
            a.review = choice(record["review"], {"unreviewed", "verified", "uncertain"});
            if (!record["notes"].isString() || !record["centerline"].isArray())
                throw std::runtime_error("Missing annotation notes or centerline.");
            a.notes = record["notes"].toString();
            if (record.contains("position")) {
                if (record["position_kind"] != "approximate_center") throw std::runtime_error("Unknown position measurement.");
                a.position = readPoint(record["position"], size);
            }
            if (record.contains("end_a")) a.endA = readPoint(record["end_a"], size);
            if (record.contains("end_b")) a.endB = readPoint(record["end_b"], size);
            for (const auto& p : record["centerline"].toArray()) a.centerline.append(readPoint(p, size));
            d.frames[f].insert(id, a);
        }
        *output = std::move(d);
        return true;
    } catch (const std::exception& e) {
        if (error) *error = QString::fromUtf8(e.what());
        return false;
    }
}
} // namespace ReferenceAnnotations
