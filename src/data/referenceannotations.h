#ifndef REFERENCEANNOTATIONS_H
#define REFERENCEANNOTATIONS_H

#include <QMap>
#include <QPointF>
#include <QSize>
#include <QString>
#include <QVector>
#include <optional>

// Independent human reference data. Never populated from TrackingDataStorage.
namespace ReferenceAnnotations {

struct Frame {
    std::optional<QPointF> position;
    std::optional<QPointF> endA;
    std::optional<QPointF> endB;
    QVector<QPointF> centerline; // Ordered from the A side toward the B side.
    QString orientation = "unknown"; // unknown, a_is_head, b_is_head
    QString visibility = "visible"; // visible, partial, hidden
    QString event = "none"; // none, self_contact, merge, split, lost, reversal
    QString review = "unreviewed"; // unreviewed, verified, uncertain
    QString notes;
    bool operator==(const Frame& other) const {
        return position == other.position && endA == other.endA && endB == other.endB &&
            centerline == other.centerline && orientation == other.orientation && visibility == other.visibility &&
            event == other.event && review == other.review && notes == other.notes;
    }
};

struct Video {
    QString path;
    QSize size;
    int frameCount = 0;
    double fps = 0;
    qint64 byteSize = 0;
};

struct SourceMapping {
    QString path;
    QPoint cropOrigin;
    int firstFrame = 0;
};

struct Document {
    Video video;
    SourceMapping source;
    QMap<int, QMap<int, Frame>> frames; // zero-based video frame -> persistent worm ID
    bool operator==(const Document& other) const {
        return video.path == other.video.path && video.size == other.video.size &&
            video.frameCount == other.video.frameCount && video.fps == other.video.fps &&
            video.byteSize == other.video.byteSize && source.path == other.source.path &&
            source.cropOrigin == other.source.cropOrigin && source.firstFrame == other.source.firstFrame &&
            frames == other.frames;
    }
};

QString defaultPath(const QString& videoPath);
bool save(const QString& path, const Document& document, QString* error);
// On failure, output is untouched; incompatible videos cannot silently reuse labels.
bool load(const QString& path, const Video& expectedVideo, Document* output, QString* error);

} // namespace ReferenceAnnotations
#endif
