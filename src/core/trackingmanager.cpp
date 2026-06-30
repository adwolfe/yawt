// trackingmanager.cpp
/**
 * @file trackingmanager.cpp
 * @brief Coordinates multi-worm tracking, video processing, and cross-thread orchestration.
 *
 * Responsibilities:
 *  - Orchestrate end-to-end tracking: initial video processing, per-worm trackers (forward/backward),
 *    merge/split handling, progress aggregation, and finalization (CSV/JSON/video saving).
 *  - Own worker threads for video processing, per-worm tracking, and optional background video saving.
 *  - Persist run metadata: threshold settings, input blobs, and per-frame merge history.
 *
 * Threading model:
 *  - GUI thread: constructs TrackingManager and receives high-level signals.
 *  - Video processing: one or more QThreads running VideoProcessor workers (chunked processing).
 *  - Worm tracking: one QThread per WormTracker (forward/backward per conceptual worm).
 *  - Video saving: optional background worker thread (VideoSaverWorker).
 *  - All cross-thread communication uses Qt queued signals/slots; custom types must be registered via registerMetaTypes().
 *
 * Lifecycle:
 *  - startFullTrackingProcess(...) initializes run state and launches workers.
 *  - cancelTracking() requests cancellation; workers should cooperatively stop.
 *  - cleanupThreadsAndObjects() joins/cleans worker threads and clears transient state.
 *  - When all trackers finish (or cancel/fail), TrackingManager emits trackingFinishedSuccessfully(...),
 *    trackingCancelled(), or trackingFailed(QString).
 *
 * Signals summary:
 *  - overallTrackingProgress(int): aggregate percent across video processing and trackers.
 *  - trackingStatusUpdate(QString): human-readable status (chunk x/y, saving n%).
 *  - allTracksUpdated(AllWormTracks): incremental consolidated tracks for UI/storage.
 *  - trackingFinishedSuccessfully(QString): successful completion; may include output path.
 *  - trackingFailed(QString): irrecoverable error with reason.
 *  - trackingCancelled(): user/system cancellation completed.
 *
 * Progress aggregation (where/how):
 *  - updateOverallProgress() aggregates progress from video processing and all WormTrackers into a single 0–100 value.
 *  - Emits overallTrackingProgress(percent) frequently; trackingStatusUpdate(message) provides granular context (e.g., "Processing chunk x/y", "Saving video n%").
 *
 * Cancellation paths (what happens when cancelled):
 *  - cancelTracking() sets cancellation flags and requests all workers to stop cooperatively; terminal state emits trackingCancelled().
 *  - cleanupThreadsAndObjects() joins/cleans QThreads and resets transient state; it is safe and idempotent to call after cancel/fail or before a new run.
 *
 * Memory management (processed frames and cleanup):
 *  - assembleProcessedFrames() accumulates chunked results; monitor footprint via getProcessedVideoMemoryUsage() during long videos.
 *  - clearProcessedVideoMemory() frees accumulated processed frames early after save/cancel/fail and emits a status update to inform the UI.
  */
#include "trackingmanager.h"
#include "../data/videometadatastore.h"
#include "../utils/loggingcategories.h"
#include "../utils/debugutils.h"
#include "../utils/yawtjsonio.h"
#include "../debug/debugdatastore.h"

#ifdef TRACKING_DEBUG
#undef TRACKING_DEBUG
#endif
#define TRACKING_DEBUG() YAWT_DEBUG(lcCoreTrackingManager)

#include <QDebug>
#include <QBuffer>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QPair>
#include <QRectF>
#include <QSizeF>
#include <QVector>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QThread>
#include <QDateTime>
#include <QXmlStreamWriter>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm> // For std::reverse, std::min, std::max
#include <numeric>   // For std::accumulate
#include <limits>    // For std::numeric_limits
#include <cmath>     // For std::ceil

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>

// frame_atomic_state.json has been eliminated.
// All merge/split state is now saved inside worms.json (mergeState section).
// The helpers below replace loadFrameAtomicStateFromJson.

// ---------------------------------------------------------------------------
// Shared JSON helpers for DetectedBlob <-> JSON (used by save and load paths)
// ---------------------------------------------------------------------------
static QJsonObject storageDetectedBlobToJson(const Tracking::DetectedBlob& db)
{
    return Tracking::detectedBlobToJson(db);
}

static Tracking::DetectedBlob storageDetectedBlobFromJson(const QJsonObject& obj)
{
    return Tracking::detectedBlobFromJson(obj);
}

// ---------------------------------------------------------------------------
// Load merge/split state from the mergeState section of worms.json.
// Replaces loadFrameAtomicStateFromJson — reads from worms.json in procDir.
// ---------------------------------------------------------------------------
static bool loadMergeStateFromWormsJson(
    const QString& procDir,
    int& outNextPhysicalBlobId,
    QMap<int, QList<FrameSpecificPhysicalBlob>>& outFrameMergeRecords,
    QMap<int, QMap<int, Tracking::DetectedBlob>>& outSplitResolutionMap,
    QMap<int, int>& outWormToPhysicalBlobIdMap)
{
    const QString path = QDir(procDir).absoluteFilePath("worms.json");
    QFile f(path);
    if (!f.exists()) return false;

    QJsonParseError err;
    const QJsonDocument doc = YawtJsonIO::readJsonDocument(path, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return false;

    const QJsonObject root = doc.object();
    if (!root.contains("mergeState") || !root["mergeState"].isObject()) {
        qDebug() << "loadMergeStateFromWormsJson: no mergeState section in" << path;
        return false;
    }
    const QJsonObject ms = root["mergeState"].toObject();

    outNextPhysicalBlobId = ms.value("nextPhysicalBlobId").toInt(outNextPhysicalBlobId);

    // wormToPhysicalBlobIdMap
    outWormToPhysicalBlobIdMap.clear();
    const QJsonValue wormToPhysValue = ms.value("wormToPhysicalBlobIdMap");
    if (wormToPhysValue.isObject()) {
        const QJsonObject wormToPhysObj = wormToPhysValue.toObject();
        for (auto it = wormToPhysObj.constBegin(); it != wormToPhysObj.constEnd(); ++it) {
            bool ok = false;
            const int key = it.key().toInt(&ok);
            if (ok) outWormToPhysicalBlobIdMap.insert(key, it.value().toInt());
        }
    }

    // frameMergeRecords
    outFrameMergeRecords.clear();
    const QJsonValue frameMergeValue = ms.value("frameMergeRecords");
    if (frameMergeValue.isObject()) {
        const QJsonObject frameMergeObj = frameMergeValue.toObject();
        for (auto fit = frameMergeObj.constBegin(); fit != frameMergeObj.constEnd(); ++fit) {
            bool ok = false;
            const int frameNum = fit.key().toInt(&ok);
            if (!ok || !fit.value().isArray()) continue;
            QList<FrameSpecificPhysicalBlob> list;
            for (const QJsonValue& v : fit.value().toArray()) {
                if (!v.isObject()) continue;
                const QJsonObject pbObj = v.toObject();
                FrameSpecificPhysicalBlob pb;
                pb.uniqueId    = pbObj.value("uniqueId").toInt();
                pb.frameNumber = pbObj.value("frameNumber").toInt();
                pb.currentArea = pbObj.value("currentArea").toDouble();
                if (pbObj.contains("currentCentroid")) {
                    const QJsonObject c = pbObj["currentCentroid"].toObject();
                    pb.currentCentroid = cv::Point2f(static_cast<float>(c.value("x").toDouble()),
                                                      static_cast<float>(c.value("y").toDouble()));
                }
                if (pbObj.contains("currentBoundingBox")) {
                    const QJsonObject b = pbObj["currentBoundingBox"].toObject();
                    pb.currentBoundingBox = QRectF(b.value("x").toDouble(), b.value("y").toDouble(),
                                                   b.value("width").toDouble(), b.value("height").toDouble());
                }
                for (const QJsonValue& cv2 : pbObj.value("contourPoints").toArray()) {
                    const QJsonArray a = cv2.toArray();
                    if (a.size() >= 2) pb.contourPoints.push_back(cv::Point(a[0].toInt(), a[1].toInt()));
                }
                for (const QJsonValue& hv : pbObj.value("holeContourPoints").toArray()) {
                    std::vector<cv::Point> hole;
                    for (const QJsonValue& pv : hv.toArray()) {
                        const QJsonArray a = pv.toArray();
                        if (a.size() >= 2) hole.push_back(cv::Point(a[0].toInt(), a[1].toInt()));
                    }
                    pb.holeContourPoints.push_back(std::move(hole));
                }
                for (const QJsonValue& pid : pbObj.value("participatingWormTrackerIDs").toArray())
                    pb.participatingWormTrackerIDs.insert(pid.toInt());
                pb.selectedByWormTrackerId = pbObj.value("selectedByWormTrackerId").toInt();
                list.append(pb);
            }
            outFrameMergeRecords.insert(frameNum, list);
        }
    }

    // splitResolutionMap
    outSplitResolutionMap.clear();
    const QJsonValue splitValue = ms.value("splitResolutionMap");
    if (splitValue.isObject()) {
        const QJsonObject splitObj = splitValue.toObject();
        for (auto sfit = splitObj.constBegin(); sfit != splitObj.constEnd(); ++sfit) {
            bool ok = false;
            const int frameNum = sfit.key().toInt(&ok);
            if (!ok || !sfit.value().isObject()) continue;
            QMap<int, Tracking::DetectedBlob> inner;
            const QJsonObject wormMapObj = sfit.value().toObject();
            for (auto wit = wormMapObj.constBegin(); wit != wormMapObj.constEnd(); ++wit) {
                bool okw = false;
                const int wormId = wit.key().toInt(&okw);
                if (!okw || !wit.value().isObject()) continue;
                inner.insert(wormId, storageDetectedBlobFromJson(wit.value().toObject()));
            }
            outSplitResolutionMap.insert(frameNum, inner);
        }
    }

    qDebug() << "loadMergeStateFromWormsJson: loaded from" << path;
    return true;
}

// (saveFrameAtomicStateToJson removed — data now lives in worms.json mergeState section)

namespace {

constexpr const char* kSpreadsheetMainNs = "http://schemas.openxmlformats.org/spreadsheetml/2006/main";
constexpr const char* kOfficeDocumentRelsNs = "http://schemas.openxmlformats.org/officeDocument/2006/relationships";
constexpr const char* kPackageRelsNs = "http://schemas.openxmlformats.org/package/2006/relationships";
constexpr const char* kContentTypesNs = "http://schemas.openxmlformats.org/package/2006/content-types";

struct WorkbookCell {
    QString value;
    bool isString = true;
};

using WorkbookRow = QList<WorkbookCell>;

struct ZipEntry {
    QString name;
    QByteArray data;
    quint32 crc32 = 0;
    quint32 offset = 0;
};

WorkbookCell stringCell(const QString& value)
{
    return WorkbookCell{value, true};
}

WorkbookCell numberCell(const QString& value)
{
    return WorkbookCell{value, false};
}

QString excelColumnName(int zeroBasedColumn)
{
    QString result;
    int column = zeroBasedColumn;
    do {
        const int remainder = column % 26;
        result.prepend(QChar(static_cast<char>('A' + remainder)));
        column = (column / 26) - 1;
    } while (column >= 0);

    return result;
}

double tipToTipDistance(const QList<QPointF>& points)
{
    if (points.size() < 2) {
        return -1.0;
    }

    const QPointF delta = points.last() - points.first();
    return std::hypot(delta.x(), delta.y());
}

QByteArray buildWorksheetXml(const QList<WorkbookRow>& rows)
{
    QByteArray xmlData;
    QBuffer buffer(&xmlData);
    buffer.open(QIODevice::WriteOnly);

    QXmlStreamWriter xml(&buffer);
    xml.writeStartDocument();
    xml.writeStartElement("worksheet");
    xml.writeDefaultNamespace(QString::fromLatin1(kSpreadsheetMainNs));
    xml.writeStartElement("sheetData");

    for (int rowIndex = 0; rowIndex < rows.size(); ++rowIndex) {
        const WorkbookRow& row = rows.at(rowIndex);
        const int excelRow = rowIndex + 1;

        xml.writeStartElement("row");
        xml.writeAttribute("r", QString::number(excelRow));

        for (int columnIndex = 0; columnIndex < row.size(); ++columnIndex) {
            const WorkbookCell& cell = row.at(columnIndex);
            const QString cellReference = excelColumnName(columnIndex) + QString::number(excelRow);

            xml.writeStartElement("c");
            xml.writeAttribute("r", cellReference);

            if (cell.isString) {
                xml.writeAttribute("t", "inlineStr");
                xml.writeStartElement("is");
                xml.writeTextElement("t", cell.value);
                xml.writeEndElement();
            } else {
                xml.writeTextElement("v", cell.value);
            }

            xml.writeEndElement();
        }

        xml.writeEndElement();
    }

    xml.writeEndElement();
    xml.writeEndElement();
    xml.writeEndDocument();

    return xmlData;
}

QByteArray buildWorkbookXml()
{
    QByteArray xmlData;
    QBuffer buffer(&xmlData);
    buffer.open(QIODevice::WriteOnly);

    QXmlStreamWriter xml(&buffer);
    xml.writeStartDocument();
    xml.writeStartElement("workbook");
    xml.writeDefaultNamespace(QString::fromLatin1(kSpreadsheetMainNs));
    xml.writeNamespace(QString::fromLatin1(kOfficeDocumentRelsNs), "r");
    xml.writeStartElement("sheets");

    xml.writeEmptyElement("sheet");
    xml.writeAttribute("name", "Tracks");
    xml.writeAttribute("sheetId", "1");
    xml.writeAttribute("r:id", "rId1");

    xml.writeEmptyElement("sheet");
    xml.writeAttribute("name", "start-end coords");
    xml.writeAttribute("sheetId", "2");
    xml.writeAttribute("r:id", "rId2");

    xml.writeEmptyElement("sheet");
    xml.writeAttribute("name", "Parameters");
    xml.writeAttribute("sheetId", "3");
    xml.writeAttribute("r:id", "rId3");

    xml.writeEmptyElement("sheet");
    xml.writeAttribute("name", "Centerlines");
    xml.writeAttribute("sheetId", "4");
    xml.writeAttribute("r:id", "rId4");
    xml.writeEndElement();
    xml.writeEndElement();
    xml.writeEndDocument();

    return xmlData;
}

QByteArray buildWorkbookRelsXml()
{
    QByteArray xmlData;
    QBuffer buffer(&xmlData);
    buffer.open(QIODevice::WriteOnly);

    QXmlStreamWriter xml(&buffer);
    xml.writeStartDocument();
    xml.writeStartElement("Relationships");
    xml.writeDefaultNamespace(QString::fromLatin1(kPackageRelsNs));

    xml.writeEmptyElement("Relationship");
    xml.writeAttribute("Id", "rId1");
    xml.writeAttribute("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet");
    xml.writeAttribute("Target", "worksheets/sheet1.xml");

    xml.writeEmptyElement("Relationship");
    xml.writeAttribute("Id", "rId2");
    xml.writeAttribute("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet");
    xml.writeAttribute("Target", "worksheets/sheet2.xml");

    xml.writeEmptyElement("Relationship");
    xml.writeAttribute("Id", "rId3");
    xml.writeAttribute("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet");
    xml.writeAttribute("Target", "worksheets/sheet3.xml");

    xml.writeEmptyElement("Relationship");
    xml.writeAttribute("Id", "rId4");
    xml.writeAttribute("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet");
    xml.writeAttribute("Target", "worksheets/sheet4.xml");

    xml.writeEmptyElement("Relationship");
    xml.writeAttribute("Id", "rId5");
    xml.writeAttribute("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles");
    xml.writeAttribute("Target", "styles.xml");

    xml.writeEndElement();
    xml.writeEndDocument();

    return xmlData;
}

QByteArray buildRootRelsXml()
{
    QByteArray xmlData;
    QBuffer buffer(&xmlData);
    buffer.open(QIODevice::WriteOnly);

    QXmlStreamWriter xml(&buffer);
    xml.writeStartDocument();
    xml.writeStartElement("Relationships");
    xml.writeDefaultNamespace(QString::fromLatin1(kPackageRelsNs));

    xml.writeEmptyElement("Relationship");
    xml.writeAttribute("Id", "rId1");
    xml.writeAttribute("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument");
    xml.writeAttribute("Target", "xl/workbook.xml");

    xml.writeEndElement();
    xml.writeEndDocument();

    return xmlData;
}

QByteArray buildStylesXml()
{
    QByteArray xmlData;
    QBuffer buffer(&xmlData);
    buffer.open(QIODevice::WriteOnly);

    QXmlStreamWriter xml(&buffer);
    xml.writeStartDocument();
    xml.writeStartElement("styleSheet");
    xml.writeDefaultNamespace(QString::fromLatin1(kSpreadsheetMainNs));

    xml.writeStartElement("fonts");
    xml.writeAttribute("count", "1");
    xml.writeStartElement("font");
    xml.writeEmptyElement("sz");
    xml.writeAttribute("val", "11");
    xml.writeEmptyElement("name");
    xml.writeAttribute("val", "Calibri");
    xml.writeEmptyElement("family");
    xml.writeAttribute("val", "2");
    xml.writeEndElement();
    xml.writeEndElement();

    xml.writeStartElement("fills");
    xml.writeAttribute("count", "2");
    xml.writeStartElement("fill");
    xml.writeEmptyElement("patternFill");
    xml.writeAttribute("patternType", "none");
    xml.writeEndElement();
    xml.writeStartElement("fill");
    xml.writeEmptyElement("patternFill");
    xml.writeAttribute("patternType", "gray125");
    xml.writeEndElement();
    xml.writeEndElement();

    xml.writeStartElement("borders");
    xml.writeAttribute("count", "1");
    xml.writeStartElement("border");
    xml.writeEmptyElement("left");
    xml.writeEmptyElement("right");
    xml.writeEmptyElement("top");
    xml.writeEmptyElement("bottom");
    xml.writeEmptyElement("diagonal");
    xml.writeEndElement();
    xml.writeEndElement();

    xml.writeStartElement("cellStyleXfs");
    xml.writeAttribute("count", "1");
    xml.writeEmptyElement("xf");
    xml.writeAttribute("numFmtId", "0");
    xml.writeAttribute("fontId", "0");
    xml.writeAttribute("fillId", "0");
    xml.writeAttribute("borderId", "0");
    xml.writeEndElement();

    xml.writeStartElement("cellXfs");
    xml.writeAttribute("count", "1");
    xml.writeEmptyElement("xf");
    xml.writeAttribute("numFmtId", "0");
    xml.writeAttribute("fontId", "0");
    xml.writeAttribute("fillId", "0");
    xml.writeAttribute("borderId", "0");
    xml.writeAttribute("xfId", "0");
    xml.writeEndElement();

    xml.writeStartElement("cellStyles");
    xml.writeAttribute("count", "1");
    xml.writeEmptyElement("cellStyle");
    xml.writeAttribute("name", "Normal");
    xml.writeAttribute("xfId", "0");
    xml.writeAttribute("builtinId", "0");
    xml.writeEndElement();

    xml.writeEndElement();
    xml.writeEndDocument();

    return xmlData;
}

QByteArray buildContentTypesXml()
{
    QByteArray xmlData;
    QBuffer buffer(&xmlData);
    buffer.open(QIODevice::WriteOnly);

    QXmlStreamWriter xml(&buffer);
    xml.writeStartDocument();
    xml.writeStartElement("Types");
    xml.writeDefaultNamespace(QString::fromLatin1(kContentTypesNs));

    xml.writeEmptyElement("Default");
    xml.writeAttribute("Extension", "rels");
    xml.writeAttribute("ContentType", "application/vnd.openxmlformats-package.relationships+xml");

    xml.writeEmptyElement("Default");
    xml.writeAttribute("Extension", "xml");
    xml.writeAttribute("ContentType", "application/xml");

    xml.writeEmptyElement("Override");
    xml.writeAttribute("PartName", "/xl/workbook.xml");
    xml.writeAttribute("ContentType", "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml");

    xml.writeEmptyElement("Override");
    xml.writeAttribute("PartName", "/xl/styles.xml");
    xml.writeAttribute("ContentType", "application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml");

    xml.writeEmptyElement("Override");
    xml.writeAttribute("PartName", "/xl/worksheets/sheet1.xml");
    xml.writeAttribute("ContentType", "application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml");

    xml.writeEmptyElement("Override");
    xml.writeAttribute("PartName", "/xl/worksheets/sheet2.xml");
    xml.writeAttribute("ContentType", "application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml");

    xml.writeEmptyElement("Override");
    xml.writeAttribute("PartName", "/xl/worksheets/sheet3.xml");
    xml.writeAttribute("ContentType", "application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml");

    xml.writeEmptyElement("Override");
    xml.writeAttribute("PartName", "/xl/worksheets/sheet4.xml");
    xml.writeAttribute("ContentType", "application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml");
    xml.writeEndElement();
    xml.writeEndDocument();

    return xmlData;
}

quint32 crc32ForData(const QByteArray& data)
{
    static quint32 table[256];
    static bool initialized = false;
    if (!initialized) {
        for (quint32 i = 0; i < 256; ++i) {
            quint32 crc = i;
            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 1U) ? (0xEDB88320U ^ (crc >> 1U)) : (crc >> 1U);
            }
            table[i] = crc;
        }
        initialized = true;
    }

    quint32 crc = 0xFFFFFFFFU;
    for (unsigned char byte : data) {
        crc = table[(crc ^ byte) & 0xFFU] ^ (crc >> 8U);
    }
    return crc ^ 0xFFFFFFFFU;
}

bool writeUInt16(QFile& file, quint16 value)
{
    const char bytes[2] = {
        static_cast<char>(value & 0xFF),
        static_cast<char>((value >> 8) & 0xFF)
    };
    return file.write(bytes, sizeof(bytes)) == sizeof(bytes);
}

bool writeUInt32(QFile& file, quint32 value)
{
    const char bytes[4] = {
        static_cast<char>(value & 0xFF),
        static_cast<char>((value >> 8) & 0xFF),
        static_cast<char>((value >> 16) & 0xFF),
        static_cast<char>((value >> 24) & 0xFF)
    };
    return file.write(bytes, sizeof(bytes)) == sizeof(bytes);
}

quint16 dosTime(const QDateTime& dateTime)
{
    const QTime time = dateTime.time();
    return static_cast<quint16>(((time.hour() & 0x1F) << 11) |
                                ((time.minute() & 0x3F) << 5) |
                                ((time.second() / 2) & 0x1F));
}

quint16 dosDate(const QDateTime& dateTime)
{
    const QDate date = dateTime.date();
    const int year = std::max(1980, date.year());
    return static_cast<quint16>((((year - 1980) & 0x7F) << 9) |
                                ((date.month() & 0x0F) << 5) |
                                (date.day() & 0x1F));
}

bool writeStoredZip(const QString& outputFilePath, QList<ZipEntry> entries)
{
    QFile file(outputFilePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }

    const QDateTime now = QDateTime::currentDateTime();
    const quint16 modTime = dosTime(now);
    const quint16 modDate = dosDate(now);

    for (ZipEntry& entry : entries) {
        const QByteArray nameBytes = entry.name.toUtf8();
        entry.crc32 = crc32ForData(entry.data);
        entry.offset = static_cast<quint32>(file.pos());

        if (!writeUInt32(file, 0x04034B50U) ||
            !writeUInt16(file, 20) ||
            !writeUInt16(file, 0) ||
            !writeUInt16(file, 0) ||
            !writeUInt16(file, modTime) ||
            !writeUInt16(file, modDate) ||
            !writeUInt32(file, entry.crc32) ||
            !writeUInt32(file, static_cast<quint32>(entry.data.size())) ||
            !writeUInt32(file, static_cast<quint32>(entry.data.size())) ||
            !writeUInt16(file, static_cast<quint16>(nameBytes.size())) ||
            !writeUInt16(file, 0) ||
            file.write(nameBytes) != nameBytes.size() ||
            file.write(entry.data) != entry.data.size()) {
            file.close();
            return false;
        }
    }

    const quint32 centralDirectoryOffset = static_cast<quint32>(file.pos());
    for (const ZipEntry& entry : std::as_const(entries)) {
        const QByteArray nameBytes = entry.name.toUtf8();
        if (!writeUInt32(file, 0x02014B50U) ||
            !writeUInt16(file, 20) ||
            !writeUInt16(file, 20) ||
            !writeUInt16(file, 0) ||
            !writeUInt16(file, 0) ||
            !writeUInt16(file, modTime) ||
            !writeUInt16(file, modDate) ||
            !writeUInt32(file, entry.crc32) ||
            !writeUInt32(file, static_cast<quint32>(entry.data.size())) ||
            !writeUInt32(file, static_cast<quint32>(entry.data.size())) ||
            !writeUInt16(file, static_cast<quint16>(nameBytes.size())) ||
            !writeUInt16(file, 0) ||
            !writeUInt16(file, 0) ||
            !writeUInt16(file, 0) ||
            !writeUInt16(file, 0) ||
            !writeUInt32(file, 0) ||
            !writeUInt32(file, entry.offset) ||
            file.write(nameBytes) != nameBytes.size()) {
            file.close();
            return false;
        }
    }

    const quint32 centralDirectorySize = static_cast<quint32>(file.pos()) - centralDirectoryOffset;
    const quint16 entryCount = static_cast<quint16>(entries.size());
    const bool success = writeUInt32(file, 0x06054B50U) &&
                         writeUInt16(file, 0) &&
                         writeUInt16(file, 0) &&
                         writeUInt16(file, entryCount) &&
                         writeUInt16(file, entryCount) &&
                         writeUInt32(file, centralDirectorySize) &&
                         writeUInt32(file, centralDirectoryOffset) &&
                         writeUInt16(file, 0);

    file.close();
    return success && file.error() == QFile::NoError;
}

} // namespace

static QString findLatestProcessingDirectory(const QString& videoSpecificDirectory) {
    if (videoSpecificDirectory.isEmpty()) return QString();
    QDir baseDir(videoSpecificDirectory);
    if (!baseDir.exists()) return QString();

    QStringList procDirs = baseDir.entryList(QStringList() << "PROC_*",
                                             QDir::Dirs | QDir::NoDotAndDotDot,
                                             QDir::Name);
    if (procDirs.isEmpty()) return QString();

    return baseDir.absoluteFilePath(procDirs.constLast());
}

// No-storage constructor implementation
TrackingManager::TrackingManager(QObject* parent)
    : QObject(parent),
      m_keyFrameNum(-1),
      m_totalFramesInVideoHint(0),
      m_isTrackingRunning(false),
      m_cancelRequested(false),
      m_videoProcessorsFinishedCount(0),
      m_totalVideoChunksToProcess(0),
      m_videoFps(0.0),
      m_expectedTrackersToFinish(0),
      m_finishedTrackersCount(0),
      m_videoProcessingOverallProgress(0),
      m_nextPhysicalBlobId(1), // Start IDs from 1
      m_storage(nullptr),
      m_debugStore(nullptr)
{
    registerMetaTypes();
}

TrackingManager::TrackingManager(TrackingDataStorage* storage, QObject* parent)
    : TrackingManager(storage, nullptr, parent)
{
}

TrackingManager::TrackingManager(TrackingDataStorage* storage,
                                 Debug::DebugDataStore* debugStore,
                                 QObject* parent)
    : QObject(parent),
    m_keyFrameNum(-1),
    m_totalFramesInVideoHint(0),
    m_isTrackingRunning(false),
    m_cancelRequested(false),
    m_videoProcessorsFinishedCount(0),
    m_totalVideoChunksToProcess(0),
    m_videoFps(0.0),
    m_expectedTrackersToFinish(0),
    m_finishedTrackersCount(0),
    m_videoProcessingOverallProgress(0),
    m_nextPhysicalBlobId(1), // Start IDs from 1
    m_storage(storage),
    m_debugStore(debugStore)
{
    registerMetaTypes();
}

void TrackingManager::registerMetaTypes()
{

    qRegisterMetaType<QMap<int, std::vector<Tracking::WormTrackPoint>>>("QMap<int, std::vector<Tracking::WormTrackPoint>>");
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<Tracking::AllWormTracks>("Tracking::AllWormTracks");
    qRegisterMetaType<QList<Tracking::DetectedBlob>>("QList<Tracking::DetectedBlob>");
    qRegisterMetaType<Tracking::DetectedBlob>("Tracking::DetectedBlob");
    qRegisterMetaType<Tracking::TrackerState>("Tracking::TrackerState");
    YAWT_DEBUG(lcCoreTrackingManager) << "TrackingManager (" << this << ") created with frame-atomic logic. Timer eliminated for direct resolution.";
}

// Destructor
TrackingManager::~TrackingManager() {
    YAWT_DEBUG(lcCoreTrackingManager) << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";

    // Ensure tracking is cancelled and resources are released
    if (m_isTrackingRunning) {
        TRACKING_DEBUG() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking();
        QThread::msleep(300);
    }

    // Force a final cleanup to ensure all resources are released
    cleanupThreadsAndObjects();
    YAWT_DEBUG(lcCoreTrackingManager) << "TrackingManager (" << this << ") DESTRUCTOR - FINISHED cleaning up.";
}

void TrackingManager::setPixelSizePixelsPerUm(double value)
{
    m_pixelSizePixelsPerUm = std::max(0.0, value);
}

// Main entry point for tracking
void TrackingManager::startFullTrackingProcess(
    const QString& videoPath, const QString& dataDirectory, int keyFrameNum,
    const std::vector<Tracking::InitialWormInfo>& initialWorms,
    const Thresholding::ThresholdSettings& settings, int totalFramesInVideoHint) {
    YAWT_DEBUG(lcCoreTrackingManager) << "TrackingManager (" << this << "): startFullTrackingProcess called.";
    if (m_isTrackingRunning) {
        qWarning() << "TrackingManager: Attempted to start tracking while already running.";
        emit trackingFailed("Another tracking process is already running.");
        return;
    }

    cleanupThreadsAndObjects(); // Clean up from any previous run

    m_videoPath = videoPath;
    m_dataDirectory = dataDirectory;
    m_keyFrameNum = keyFrameNum;
    m_initialWormInfos = initialWorms;
    m_thresholdSettings = settings;
    m_totalFramesInVideoHint = totalFramesInVideoHint;

    // Create video-specific directory, try loading latest run state, then create new processing output directory
    m_videoSpecificDirectory = createVideoSpecificDirectory(dataDirectory, videoPath);
    m_processingOutputDirectory.clear();
    if (!m_videoSpecificDirectory.isEmpty()) {
        const QString latestProcDir = findLatestProcessingDirectory(m_videoSpecificDirectory);
        if (!latestProcDir.isEmpty()) {
            QString thresholdFilePath = QDir(latestProcDir).absoluteFilePath("thresholding.json");
            if (QFile::exists(thresholdFilePath)) {
                bool settingsMatch = compareThresholdSettings(thresholdFilePath, settings);
                if (!settingsMatch) {
                    TRACKING_DEBUG() << "Current threshold settings differ from stored settings in" << thresholdFilePath;
                    emit trackingStatusUpdate("Threshold settings differ from previous run");
                } else {
                    bool loaded = loadMergeStateFromWormsJson(latestProcDir,
                                                             m_nextPhysicalBlobId,
                                                             m_frameMergeRecords,
                                                             m_splitResolutionMap,
                                                             m_wormToPhysicalBlobIdMap);
                    if (loaded) {
                        TRACKING_DEBUG() << "TrackingManager: Loaded merge/split state from worms.json in"
                                         << latestProcDir;
                        emit trackingStatusUpdate("Loaded previous merge/split state (retracking enabled)");
                    } else {
                        TRACKING_DEBUG() << "TrackingManager: No merge state found in worms.json (first run or old format).";
                    }
                }
            }
        }

        m_processingOutputDirectory = createProcessingOutputDirectory(m_videoSpecificDirectory);
        if (!m_processingOutputDirectory.isEmpty()) {
            saveThresholdingJson(m_processingOutputDirectory, settings);
            saveInputBlobs(m_processingOutputDirectory, initialWorms);
        }
    }
    m_isTrackingRunning = true;
    m_cancelRequested = false;
    m_videoProcessingOverallProgress = 0;
    m_finishedTrackersCount = 0;
    m_expectedTrackersToFinish = 0;
    m_nextPhysicalBlobId = 1; // Reset for new tracking session

    // Clear data structures
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameMergeRecords.clear();
    m_splitResolutionMap.clear();
    m_wormToPhysicalBlobIdMap.clear();

    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();
    for (const auto& info : m_initialWormInfos) {
        if (!m_wormObjectsMap.contains(info.id)) {
            m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
        }
        m_wormToPhysicalBlobIdMap[info.id] = -1; // Forward tracker
        m_wormToPhysicalBlobIdMap[-info.id] = -1; // Backward tracker
    }

    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();
    m_videoChunkProgressMap.clear();
    m_videoProcessorsFinishedCount = 0;
    m_totalVideoChunksToProcess = 0;
    m_finalProcessedForwardFrames.clear();
    m_finalProcessedReversedFrames.clear();

    emit trackingStatusUpdate("Initializing video processing...");
    emit overallTrackingProgress(0);

    // --- Determine actual total frames, FPS, and frame size ONCE ---
    // (This logic remains the same as your version with parallel video processing)
    cv::VideoCapture preliminaryCap;
    int actualTotalFrames = 0;
    try {
        if (!preliminaryCap.open(m_videoPath.toStdString())) {
            emit trackingFailed("Failed to open video file for preliminary checks: " + m_videoPath);
            m_isTrackingRunning = false; return;
        }
        actualTotalFrames = static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_COUNT));
        m_videoFps = preliminaryCap.get(cv::CAP_PROP_FPS);
        m_videoFrameSize = cv::Size(
            static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_WIDTH)),
            static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_HEIGHT))
            );
        preliminaryCap.release();
    } catch (const cv::Exception& ex) {
        preliminaryCap.release();
        emit trackingFailed("OpenCV exception during preliminary video checks: " + QString(ex.what()));
        m_isTrackingRunning = false; return;
    }
    if (actualTotalFrames <= 0 && m_totalFramesInVideoHint > 0) actualTotalFrames = m_totalFramesInVideoHint;
    else if (actualTotalFrames <= 0) { emit trackingFailed("Could not determine total frames."); m_isTrackingRunning = false; return; }
    if (m_videoFps <= 0) m_videoFps = 25.0;
    if (!m_dataDirectory.isEmpty() && !m_videoPath.isEmpty())
        VideoMetadataStore::saveFps(m_dataDirectory, QFileInfo(m_videoPath).completeBaseName(), m_videoFps);
    if (m_videoFrameSize.width <= 0 || m_videoFrameSize.height <= 0) { emit trackingFailed("Could not determine frame size."); m_isTrackingRunning = false; return; }
    if (m_keyFrameNum < 0 || m_keyFrameNum >= actualTotalFrames) { emit trackingFailed("Keyframe out of bounds."); m_isTrackingRunning = false; return; }

    // --- Parallel Video Processing Logic ---
    // (This logic remains the same as your version with parallel video processing)
    int numThreads = QThread::idealThreadCount();
    numThreads = qMax(1, qMin(numThreads, 8));
    emit trackingStatusUpdate(QString("Processing video in chunks across %1 threads...").arg(numThreads));
    QList<QPair<int, int>> forwardFrameRanges, backwardFrameRanges;
    int forwardSegmentStart = m_keyFrameNum, forwardSegmentEnd = actualTotalFrames;
    int totalForwardSegmentFrames = forwardSegmentEnd - forwardSegmentStart;
    if (totalForwardSegmentFrames > 0) {
        int framesPerThread = std::max(1, static_cast<int>(std::ceil(static_cast<double>(totalForwardSegmentFrames) / numThreads)));
        for (int cs = forwardSegmentStart; cs < forwardSegmentEnd; cs += framesPerThread) {
            forwardFrameRanges.append({cs, std::min(cs + framesPerThread, forwardSegmentEnd)});
        }
    }
    int backwardSegmentStart = 0, backwardSegmentEnd = m_keyFrameNum;
    int totalBackwardSegmentFrames = backwardSegmentEnd - backwardSegmentStart;
    if (totalBackwardSegmentFrames > 0) {
        int framesPerThread = std::max(1, static_cast<int>(std::ceil(static_cast<double>(totalBackwardSegmentFrames) / numThreads)));
        for (int cs = backwardSegmentStart; cs < backwardSegmentEnd; cs += framesPerThread) {
            backwardFrameRanges.append({cs, std::min(cs + framesPerThread, backwardSegmentEnd)});
        }
    }
    m_totalVideoChunksToProcess = forwardFrameRanges.size() + backwardFrameRanges.size();
    if (m_totalVideoChunksToProcess == 0) {
        handleInitialProcessingComplete({}, {}, m_videoFps, m_videoFrameSize); return;
    }
    auto launch_chunk_processor = [this](int sF, int eF, bool isFwd) { /* ... same as your version ... */
                                                                       int chunkId = sF; m_videoChunkProgressMap[chunkId] = 0;
                                                                       VideoProcessor* proc = new VideoProcessor(); QThread* thr = new QThread(); proc->moveToThread(thr);
                                                                       m_videoProcessorThreads.append(thr);
                                                                       connect(thr, &QThread::started, proc, [=](){ proc->processFrameRange(m_videoPath, m_thresholdSettings, sF, eF, chunkId, isFwd); });
                                                                       connect(proc, &VideoProcessor::rangeProcessingComplete, this, &TrackingManager::handleRangeProcessingComplete);
                                                                       connect(proc, &VideoProcessor::processingError, this, &TrackingManager::handleVideoChunkProcessingError);
                                                                       connect(proc, &VideoProcessor::rangeProcessingProgress, this, &TrackingManager::handleRangeProcessingProgress);
                                                                       connect(thr, &QThread::finished, proc, &QObject::deleteLater);
                                                                       connect(thr, &QThread::finished, this, [this, tPtr = QPointer<QThread>(thr)](){ if(tPtr){/*qDebug() << "VidProcThr fin"*/;}});
                                                                       thr->start();
    };
    for (const auto& r : forwardFrameRanges) launch_chunk_processor(r.first, r.second, true);
    for (const auto& r : backwardFrameRanges) launch_chunk_processor(r.first, r.second, false);

    // Timer-based pause resolution has been eliminated in favor of direct resolution
    YAWT_DEBUG(lcCoreTrackingManager) << "TM: Using direct split resolution instead of timer-based approach";
}

void TrackingManager::cancelTracking() {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested && !m_isTrackingRunning) return;
    if (m_cancelRequested) return;
    m_cancelRequested = true;
    bool wasRunning = m_isTrackingRunning;

    // Clear the split resolution map
    m_splitResolutionMap.clear();

    locker.unlock();
    emit trackingStatusUpdate("Cancellation requested...");
    for (QPointer<QThread> thread : m_videoProcessorThreads) { if (thread && thread->isRunning()) thread->requestInterruption(); }
    QList<WormTracker*> trackersToStop;
    locker.relock(); trackersToStop = m_wormTrackersList; locker.unlock();
    for (WormTracker* tracker : trackersToStop) { if (tracker) QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection); }
    locker.relock();
    if ((wasRunning && m_expectedTrackersToFinish == 0 && m_wormTrackersList.isEmpty()) || !wasRunning) {
        m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled();
    }
}

void TrackingManager::cleanupThreadsAndObjects() {
    YAWT_DEBUG(lcCoreTrackingManager) << "TrackingManager: Starting aggressive memory cleanup...";

    // Calculate memory usage before cleanup for reporting
    size_t memoryBefore = getProcessedVideoMemoryUsage();
    size_t tracksMemoryBefore = 0;
    for (const auto& [wormId, points] : m_finalTracks) {
        tracksMemoryBefore += points.size() * sizeof(Tracking::WormTrackPoint);
    }

    // (Largely same as your version, ensuring QPointer safety and clearing new maps)
    QList<QPointer<QThread>> videoThreadsToClean = m_videoProcessorThreads;
    m_videoProcessorThreads.clear();
    for (QPointer<QThread> thread : videoThreadsToClean) { /* ... quit, wait, delete ... */
        if (thread) { if (thread->isRunning()) { thread->requestInterruption(); thread->quit(); if (!thread->wait(1500)) { thread->terminate(); thread->wait();}} delete thread;}
    }
    QList<QPointer<QThread>> trackerThreadsToClean = m_trackerThreads;
    m_trackerThreads.clear();
    for (QPointer<QThread> thread : trackerThreadsToClean) { /* ... quit, wait, delete ... */
        if (thread) { if (thread->isRunning()) { thread->requestInterruption(); thread->quit(); if (!thread->wait(1000)) { thread->terminate(); thread->wait();}} delete thread;}
    }

    QList<QPointer<QThread>> centerlineThreadsToClean = m_centerlineThreads;
    m_centerlineThreads.clear();
    m_centerlineWorkers.clear();
    for (QPointer<QThread> thread : centerlineThreadsToClean) {
        if (thread) {
            if (thread->isRunning()) {
                thread->requestInterruption();
                thread->quit();
                if (!thread->wait(1000)) {
                    thread->terminate();
                    thread->wait();
                }
            }
            delete thread;
        }
    }
    m_centerlineWorkerProgress.clear();
    m_centerlineWorkersFinishedCount = 0;
    m_totalCenterlineWorkers = 0;
    m_centerlineStorageMutex.clear();

    // Aggressively clear worm trackers and objects
    m_wormTrackersList.clear();
    m_wormIdToForwardTrackerInstanceMap.clear();
    m_wormIdToBackwardTrackerInstanceMap.clear();

    // Clear WormObject map with explicit deletion and memory hints
    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();
    QMap<int, WormObject*>().swap(m_wormObjectsMap); // Force deallocation

    // Aggressively clear processed video memory
    clearProcessedVideoMemory();

    // Clear and shrink all chunk processing maps
    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();
    QMap<int, std::vector<cv::Mat>>().swap(m_assembledForwardFrameChunks);
    QMap<int, std::vector<cv::Mat>>().swap(m_assembledBackwardFrameChunks);

    m_videoChunkProgressMap.clear();
    QMap<int, int>().swap(m_videoChunkProgressMap);

    // Aggressively clear final tracks with memory hints
    m_finalTracks.clear();
    Tracking::AllWormTracks().swap(m_finalTracks);

    m_individualTrackerProgress.clear();
    QMap<WormTracker*, int>().swap(m_individualTrackerProgress);

    // Clear all data structures related to split resolution with forced deallocation
    m_frameMergeRecords.clear();
    QMap<int, QList<FrameSpecificPhysicalBlob>>().swap(m_frameMergeRecords);

    m_splitResolutionMap.clear();
    QMap<int, QMap<int, Tracking::DetectedBlob>>().swap(m_splitResolutionMap);

    m_wormToPhysicalBlobIdMap.clear();
    QMap<int, int>().swap(m_wormToPhysicalBlobIdMap);

    // Reset state flags
    m_isTrackingRunning = false;
    m_cancelRequested = false;

    // Reset frame counters and IDs
    m_nextPhysicalBlobId = 1;

    // Calculate and report memory freed
    double memoryMB = (memoryBefore + tracksMemoryBefore) / (1024.0 * 1024.0);
    YAWT_DEBUG(lcCoreTrackingManager) << "TrackingManager: Aggressive cleanup complete - freed approximately"
             << QString::number(memoryMB, 'f', 1) << "MB of tracking data";
    YAWT_DEBUG(lcCoreTrackingManager) << "TrackingManager: All data structures cleared and memory aggressively freed";
}

// --- Video Processing Callbacks ---
void TrackingManager::handleRangeProcessingProgress(int chunkId, int percentage) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    QMutexLocker locker(&m_dataMutex);
    if (m_videoChunkProgressMap.contains(chunkId)) m_videoChunkProgressMap[chunkId] = percentage; else return;
    double totalProgressSum = 0; for (int p : m_videoChunkProgressMap.values()) totalProgressSum += p;
    m_videoProcessingOverallProgress = (m_totalVideoChunksToProcess > 0) ? static_cast<int>(totalProgressSum / m_totalVideoChunksToProcess) : 100;
    locker.unlock(); updateOverallProgress();
}
void TrackingManager::handleRangeProcessingComplete(int chunkId, const std::vector<cv::Mat>& processedFrames, bool wasForwardChunk) {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) return;
    if (wasForwardChunk) m_assembledForwardFrameChunks[chunkId] = processedFrames; else m_assembledBackwardFrameChunks[chunkId] = processedFrames;
    m_videoChunkProgressMap[chunkId] = 100; m_videoProcessorsFinishedCount++;
    if (m_videoProcessorsFinishedCount >= m_totalVideoChunksToProcess) {
        locker.unlock(); assembleProcessedFrames();
    } else {
        double totalProgressSum = 0; for (int p : m_videoChunkProgressMap.values()) totalProgressSum += p;
        m_videoProcessingOverallProgress = (m_totalVideoChunksToProcess > 0) ? static_cast<int>(totalProgressSum / m_totalVideoChunksToProcess) : 100;
        locker.unlock(); updateOverallProgress();
    }
}
void TrackingManager::assembleProcessedFrames() {
    QMutexLocker locker(&m_dataMutex); if (m_cancelRequested) { m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled(); return; }
    m_finalProcessedForwardFrames.clear(); m_finalProcessedReversedFrames.clear();
    // Assemble forward chunks in correct order (sorted by chunk key = start frame)
    QList<int> forwardKeys = m_assembledForwardFrameChunks.keys();
    std::sort(forwardKeys.begin(), forwardKeys.end());
    for (int key : forwardKeys) {
        const auto& chunk = m_assembledForwardFrameChunks[key];
        m_finalProcessedForwardFrames.insert(m_finalProcessedForwardFrames.end(), chunk.begin(), chunk.end());
    }

    // Assemble backward chunks in correct order (sorted by chunk key = start frame)
    std::vector<cv::Mat> tempBackwardFrames;
    QList<int> backwardKeys = m_assembledBackwardFrameChunks.keys();
    std::sort(backwardKeys.begin(), backwardKeys.end());
    for (int key : backwardKeys) {
        const auto& chunk = m_assembledBackwardFrameChunks[key];
        tempBackwardFrames.insert(tempBackwardFrames.end(), chunk.begin(), chunk.end());
    }
    m_finalProcessedReversedFrames = tempBackwardFrames; std::reverse(m_finalProcessedReversedFrames.begin(), m_finalProcessedReversedFrames.end());
    m_assembledForwardFrameChunks.clear(); m_assembledBackwardFrameChunks.clear(); m_videoChunkProgressMap.clear();
    locker.unlock();
    handleInitialProcessingComplete(m_finalProcessedForwardFrames, m_finalProcessedReversedFrames, m_videoFps, m_videoFrameSize);
}
void TrackingManager::handleInitialProcessingComplete(const std::vector<cv::Mat>&, const std::vector<cv::Mat>&, double, cv::Size) {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) { m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled(); return; }
    if (!m_isTrackingRunning) return;
    m_videoProcessingOverallProgress = 100;
    locker.unlock();
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    updateOverallProgress();
    launchWormTrackers();
}
void TrackingManager::handleVideoChunkProcessingError(int chunkId, const QString& errorMessage) {
    QMutexLocker locker(&m_dataMutex); if (m_cancelRequested || !m_isTrackingRunning) return;
    m_cancelRequested = true; m_isTrackingRunning = false;
    locker.unlock();
    for (QPointer<QThread> thread : m_videoProcessorThreads) { if (thread && thread->isRunning()) thread->requestInterruption(); }
    emit trackingFailed("Video processing failed (chunk " + QString::number(chunkId) + "): " + errorMessage);
}

// --- Helper function for signed worm IDs ---
int TrackingManager::getSignedWormId(int conceptualWormId, WormTracker::TrackingDirection direction) {
    return (direction == WormTracker::TrackingDirection::Forward) ? conceptualWormId : -conceptualWormId;
}

int TrackingManager::getUnsignedWormId(int signedWormId) {
    return qAbs(signedWormId);
}

WormTracker::TrackingDirection TrackingManager::getDirectionFromSignedId(int signedWormId) {
    return (signedWormId >= 0) ? WormTracker::TrackingDirection::Forward : WormTracker::TrackingDirection::Backward;
}

// --- Core Frame Update Logic ---
void TrackingManager::handleFrameUpdate(int reportingConceptualWormId,
                                           int originalFrameNumber,
                                           const Tracking::DetectedBlob& primaryBlob, // Anchor blob for track history
                                           const Tracking::DetectedBlob& fullBlob,    // Full blob for merge/state processing
                                           QRectF searchRoiUsed,
                                           Tracking::TrackerState currentState,
                                           const QList<Tracking::DetectedBlob>& splitCandidates)
{
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* reportingTrackerInstance = qobject_cast<WormTracker*>(sender());

    // Get direction and calculate signed worm ID for pause handling
    WormTracker::TrackingDirection direction = reportingTrackerInstance ? reportingTrackerInstance->getDirection() : WormTracker::TrackingDirection::Forward;
    int signedWormId = getSignedWormId(reportingConceptualWormId, direction);

    QString dmsg = QString("TM: WT %1 FN%2 | ").arg(signedWormId).arg(originalFrameNumber);
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** PROCESSING FRAME UPDATE *** State:").arg(signedWormId).arg(originalFrameNumber) << static_cast<int>(currentState) << "FullBlobValid:" << fullBlob.isValid << "SplitCandidates:" << splitCandidates.size();
    // Note: WormObject update uses primaryBlob
    WormObject* wormObject = m_wormObjectsMap.value(reportingConceptualWormId, nullptr);
    if (wormObject) {
        Tracking::WormTrackPoint point;
        point.frameNumberOriginal = originalFrameNumber;
        point.roi = searchRoiUsed;

        if (primaryBlob.isValid) {
            point.position = cv::Point2f(static_cast<float>(primaryBlob.centroid.x()), static_cast<float>(primaryBlob.centroid.y()));
            point.quality = (currentState == Tracking::TrackerState::TrackingSingle) ? Tracking::TrackPointQuality::Single : Tracking::TrackPointQuality::Merged;
        } else {
            point.position = cv::Point2f(0.0f, 0.0f);  // Placeholder (won't be used for display)
            point.quality = Tracking::TrackPointQuality::Lost;
        }

        wormObject->updateTrackPoint(point);
    }

    //QString dmsg = QString("TM: WT %1 FN%2 | ").arg(reportingConceptualWormId).arg(originalFrameNumber);
    // qDebug().noquote() << dmsg << "State" << static_cast<int>(currentState) << "FullBlobValid:" << fullBlob.isValid;

    if (currentState == Tracking::TrackerState::TrackingSingle || currentState == Tracking::TrackerState::TrackingLost) {
        m_wormToPhysicalBlobIdMap[signedWormId] = -1; // No longer part of a specific physical blob
    } else if (currentState == Tracking::TrackerState::TrackingMerged) {
        if (fullBlob.isValid) {
            processFrameSpecificMerge(signedWormId, originalFrameNumber, fullBlob, reportingTrackerInstance);
        } else {
            // qDebug().noquote() << dmsg << "State Merged but fullBlob invalid. Treating as lost for merge logic.";
            m_wormToPhysicalBlobIdMap[signedWormId] = -1;
        }
    } else if (currentState == Tracking::TrackerState::PausedForSplit) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** ENTERING SPLIT PROCESSING ***").arg(signedWormId).arg(originalFrameNumber);
        if (!splitCandidates.isEmpty() && reportingTrackerInstance) {
            // Find the chosen candidate (should be the primaryBlob if valid, otherwise first candidate)
            Tracking::DetectedBlob chosenCandidate = primaryBlob.isValid ? primaryBlob :
                                                    (!splitCandidates.isEmpty() ? splitCandidates.first() : Tracking::DetectedBlob());
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Chosen candidate Area:").arg(signedWormId).arg(originalFrameNumber) << chosenCandidate.area << "Pos:" << chosenCandidate.centroid.x() << "," << chosenCandidate.centroid.y();
            processFrameSpecificSplit(signedWormId, originalFrameNumber, splitCandidates, chosenCandidate, reportingTrackerInstance);
        } else {
             TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|State PausedForSplit but no candidates/instance. Forcing lost.").arg(signedWormId).arg(originalFrameNumber);
            m_wormToPhysicalBlobIdMap[signedWormId] = -1;
            if(reportingTrackerInstance) QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        }
    }

    // Persist the full detected blob into storage (so UI can later read tracker-derived contours)
    if (m_storage && fullBlob.isValid) {
        // Store using the unsigned conceptual worm id (frame differentiates entries)
        int unsignedWormId = getUnsignedWormId(signedWormId);
        m_storage->setDetectedBlobForFrame(originalFrameNumber, unsignedWormId, fullBlob);
    }

    // All split resolution is now immediate without any paused state
}


void TrackingManager::processFrameSpecificMerge(int signedWormId, int frameNumber,
                                                const Tracking::DetectedBlob& reportedFullBlob,
                                                WormTracker* reportingTrackerInstance)
{
    int conceptualWormId = getUnsignedWormId(signedWormId);
    QString dmsg = QString("TM: WT %1 FN%2 | procFrameSpecMerge | ").arg(signedWormId).arg(frameNumber);
    // qDebug().noquote() << dmsg << "Blob @ " << reportedFullBlob.centroid.x() << "," << reportedFullBlob.centroid.y() << " Area: " << reportedFullBlob.area;

    QList<FrameSpecificPhysicalBlob>& blobsOnThisFrame = m_frameMergeRecords[frameNumber];
    FrameSpecificPhysicalBlob* matchedPhysicalBlob = nullptr;

    for (FrameSpecificPhysicalBlob& existingBlobRecord : blobsOnThisFrame) {
        double iou = calculateIoU(reportedFullBlob.boundingBox, existingBlobRecord.currentBoundingBox);
        bool isContained = existingBlobRecord.currentBoundingBox.contains(reportedFullBlob.centroid);
        //cv::Point2f reportedCentroidCv(static_cast<float>(reportedFullBlob.centroid.x()), static_cast<float>(reportedFullBlob.centroid.y()));
        //double distSq = Tracking::sqDistance(reportedCentroidCv, existingBlobRecord.currentCentroid);

        if (iou > PHYSICAL_BLOB_IOU_THRESHOLD || (isContained && iou > 0.01) /*|| distSq < PHYSICAL_BLOB_CENTROID_MAX_DIST_SQ*/) {
            matchedPhysicalBlob = &existingBlobRecord;
            // qDebug().noquote() << dmsg << "Matched existing PhysicalBlobID:" << matchedPhysicalBlob->uniqueId << "IoU:" << iou << "Contained:" << isContained;
            break;
        }
    }

    if (matchedPhysicalBlob) {
        matchedPhysicalBlob->participatingWormTrackerIDs.insert(signedWormId);
        // Update the physical blob's representation with this new information
        matchedPhysicalBlob->currentBoundingBox = matchedPhysicalBlob->currentBoundingBox.united(reportedFullBlob.boundingBox);
        matchedPhysicalBlob->currentArea = qMax(matchedPhysicalBlob->currentArea, reportedFullBlob.area); // ideally should not change
        if (!reportedFullBlob.contourPoints.empty()) {
            matchedPhysicalBlob->contourPoints = reportedFullBlob.contourPoints;
            matchedPhysicalBlob->holeContourPoints = reportedFullBlob.holeContourPoints;
        }
        if(matchedPhysicalBlob->currentBoundingBox.isValid()){
            QPointF newCenter = matchedPhysicalBlob->currentBoundingBox.center();
            matchedPhysicalBlob->currentCentroid = cv::Point2f(static_cast<float>(newCenter.x()), static_cast<float>(newCenter.y()));
        }
        m_wormToPhysicalBlobIdMap[signedWormId] = matchedPhysicalBlob->uniqueId;
        // qDebug().noquote() << dmsg << "Added to existing PhysicalBlobID:" << matchedPhysicalBlob->uniqueId << ". Participants:" << matchedPhysicalBlob->participatingWormTrackerIDs;
    } else {
        FrameSpecificPhysicalBlob newPhysicalBlob;
        newPhysicalBlob.uniqueId = m_nextPhysicalBlobId++;
        newPhysicalBlob.frameNumber = frameNumber;
        newPhysicalBlob.currentBoundingBox = reportedFullBlob.boundingBox;
        if(newPhysicalBlob.currentBoundingBox.isValid()){
            QPointF center = newPhysicalBlob.currentBoundingBox.center();
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(center.x()), static_cast<float>(center.y()));
        } else {
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(reportedFullBlob.centroid.x()), static_cast<float>(reportedFullBlob.centroid.y()));
        }
        newPhysicalBlob.currentArea = reportedFullBlob.area;
        newPhysicalBlob.contourPoints = reportedFullBlob.contourPoints;
        newPhysicalBlob.holeContourPoints = reportedFullBlob.holeContourPoints;
        newPhysicalBlob.participatingWormTrackerIDs.insert(signedWormId);
        // newPhysicalBlob.timeFirstReported = QDateTime::currentDateTime();

        blobsOnThisFrame.append(newPhysicalBlob);
        m_wormToPhysicalBlobIdMap[signedWormId] = newPhysicalBlob.uniqueId;
        // qDebug().noquote() << dmsg << "Created new PhysicalBlobID:" << newPhysicalBlob.uniqueId << ". Participants:" << newPhysicalBlob.participatingWormTrackerIDs;
    }

    // Merge state is fully handled with the frame-specific physical blob representation
}

void TrackingManager::processFrameSpecificSplit(int signedWormId, int frameNumber,
                                               const QList<Tracking::DetectedBlob>& allSplitCandidates,
                                               const Tracking::DetectedBlob& chosenCandidate,
                                               WormTracker* reportingTrackerInstance)
{
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|procFrameSpecSplit - Candidates:").arg(signedWormId).arg(frameNumber) << allSplitCandidates.size() << "Chosen @" << chosenCandidate.centroid.x() << "," << chosenCandidate.centroid.y() << "Area:" << chosenCandidate.area;

    // Get reference to current frame's physical blobs
    QList<FrameSpecificPhysicalBlob>& blobsOnThisFrame = m_frameMergeRecords[frameNumber];

    // Step 1: Create PhysicalBlobIds for all split candidates if they don't exist
    QList<int> candidatePhysicalBlobIds;
    int chosenCandidatePhysicalBlobId = -1; // Track which physical blob ID corresponds to the chosen candidate

    for (const Tracking::DetectedBlob& candidate : allSplitCandidates) {
        if (!candidate.isValid) continue;

        // Check if this candidate matches an existing physical blob
        FrameSpecificPhysicalBlob* existingBlob = nullptr;
        for (FrameSpecificPhysicalBlob& blob : blobsOnThisFrame) {
            double iou = calculateIoU(candidate.boundingBox, blob.currentBoundingBox);
            bool isContained = blob.currentBoundingBox.contains(candidate.centroid);

            if (iou > PHYSICAL_BLOB_IOU_THRESHOLD || (isContained && iou > 0.01)) {
                existingBlob = &blob;
                break;
            }
        }

        int currentBlobId = -1;
        if (existingBlob) {
            // Add this WT to the existing blob's participants
            existingBlob->participatingWormTrackerIDs.insert(signedWormId);
            if (!candidate.contourPoints.empty()) {
                existingBlob->contourPoints = candidate.contourPoints;
                existingBlob->holeContourPoints = candidate.holeContourPoints;
            }
            currentBlobId = existingBlob->uniqueId;
            candidatePhysicalBlobIds.append(currentBlobId);
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Matched existing PhysicalBlobID:").arg(signedWormId).arg(frameNumber) << currentBlobId;
        } else {
            // Create new physical blob for this candidate
            FrameSpecificPhysicalBlob newPhysicalBlob;
            newPhysicalBlob.uniqueId = m_nextPhysicalBlobId++;
            newPhysicalBlob.frameNumber = frameNumber;
            newPhysicalBlob.currentBoundingBox = candidate.boundingBox;
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(candidate.centroid.x()), static_cast<float>(candidate.centroid.y()));
            newPhysicalBlob.currentArea = candidate.area;
            newPhysicalBlob.contourPoints = candidate.contourPoints;
            newPhysicalBlob.holeContourPoints = candidate.holeContourPoints;
            newPhysicalBlob.participatingWormTrackerIDs.insert(signedWormId);
            newPhysicalBlob.selectedByWormTrackerId = 0; // Initially unselected

            blobsOnThisFrame.append(newPhysicalBlob);
            currentBlobId = newPhysicalBlob.uniqueId;
            candidatePhysicalBlobIds.append(currentBlobId);
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Created new PhysicalBlobID:").arg(signedWormId).arg(frameNumber) << currentBlobId << "for split candidate";
        }

        // Check if this is the chosen candidate (by comparing centroids)
        if (qFuzzyCompare(candidate.centroid.x(), chosenCandidate.centroid.x()) &&
            qFuzzyCompare(candidate.centroid.y(), chosenCandidate.centroid.y())) {
            chosenCandidatePhysicalBlobId = currentBlobId;
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Identified chosen candidate as PhysicalBlobID:").arg(signedWormId).arg(frameNumber) << currentBlobId;
        }
    }

    // Set the preferred blob for this worm to be the chosen candidate
    if (chosenCandidatePhysicalBlobId != -1) {
        m_wormToPhysicalBlobIdMap[signedWormId] = chosenCandidatePhysicalBlobId;
        int unsignedWormId = getUnsignedWormId(signedWormId);
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** SETTING PREFERRED BLOB *** Worm").arg(signedWormId).arg(frameNumber) << unsignedWormId
                          << "prefers PhysicalBlobID:" << chosenCandidatePhysicalBlobId;
    } else {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** WARNING: Could not identify chosen candidate in physical blobs ***").arg(signedWormId).arg(frameNumber);
    }

    // Try immediate resolution using our new method
    if (attemptImmediateSplitResolution(signedWormId, frameNumber, allSplitCandidates, chosenCandidate, reportingTrackerInstance)) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Split successfully resolved immediately").arg(signedWormId).arg(frameNumber);
    } else {
        // Fallback - assign the first valid blob or go lost
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Immediate resolution failed, using fallback (go lost)").arg(signedWormId).arg(frameNumber);
        QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][signedWormId] = Tracking::DetectedBlob();
    }
}


bool TrackingManager::attemptImmediateSplitResolution(int signedWormId, int frameNumber,
                                                    const QList<Tracking::DetectedBlob>& allCandidates,
                                                    const Tracking::DetectedBlob& chosenCandidate,
                                                    WormTracker* trackerInstance) {
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|=== STARTING SPLIT RESOLUTION ===").arg(signedWormId).arg(frameNumber);
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Chosen candidate Area:").arg(signedWormId).arg(frameNumber) << chosenCandidate.area
                      << "Pos:" << chosenCandidate.centroid.x() << "," << chosenCandidate.centroid.y();

    if (!trackerInstance) { qWarning() << QString("TM: %1|FN%2|No tracker instance.").arg(signedWormId).arg(frameNumber); return false; }
    if (!chosenCandidate.isValid) { TRACKING_DEBUG() << QString("TM: %1|FN%2|Invalid candidate. Forcing lost.").arg(signedWormId).arg(frameNumber); QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob())); return true; }

    // Find physical blob IDs for this worm
    QList<int> thisWormPhysicalBlobIds;
    if (m_frameMergeRecords.contains(frameNumber)) {
        for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
            if (blob.participatingWormTrackerIDs.contains(signedWormId)) {
                thisWormPhysicalBlobIds.append(blob.uniqueId);
            }
        }
    }

    if (thisWormPhysicalBlobIds.isEmpty()) {
        TRACKING_DEBUG() << QString("TM: %1|FN%2|No PhysicalBlobIds found for this worm. Cannot proceed with resolution.").arg(signedWormId).arg(frameNumber);
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection,
                                 Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][signedWormId] = Tracking::DetectedBlob();
        return true;
    }

    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|This worm's PhysicalBlobIds:").arg(signedWormId).arg(frameNumber) << thisWormPhysicalBlobIds;

    // Show current blob assignments for debugging
    for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|PhysicalBlob").arg(signedWormId).arg(frameNumber) << blob.uniqueId
                          << "selectedBy:" << blob.selectedByWormTrackerId
                          << "participants:" << blob.participatingWormTrackerIDs;
    }

    // Check if a preferred blob is already assigned to this worm
    int preferredBlobId = m_wormToPhysicalBlobIdMap.value(signedWormId, -1);
    Tracking::DetectedBlob blobToAssign;
    blobToAssign.isValid = false;
    bool resolutionSuccess = false;

    // Look for already resolved worms on this frame that might have taken blobs we're interested in
    QMap<int, QList<int>> otherWormBlobAssignments; // wormId -> list of blobIds

    if (m_splitResolutionMap.contains(frameNumber)) {
        QMap<int, Tracking::DetectedBlob> resolvedBlobs = m_splitResolutionMap[frameNumber];
        for (auto it = resolvedBlobs.begin(); it != resolvedBlobs.end(); ++it) {
            int otherWormId = it.key();
            if (otherWormId == signedWormId) continue;

            QList<int> otherWormBlobIds;
            for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.selectedByWormTrackerId == otherWormId) {
                    otherWormBlobIds.append(blob.uniqueId);
                }
            }

            if (!otherWormBlobIds.isEmpty()) {
                otherWormBlobAssignments[otherWormId] = otherWormBlobIds;
                TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Worm").arg(signedWormId).arg(frameNumber) << otherWormId << "already has blobs assigned:" << otherWormBlobIds;
            }
        }
    }

    // First, try to find our preferred blob if it's available
    if (preferredBlobId != -1) {
        bool blobTaken = false;

        // Check if any other worm has already been assigned this blob
        for (auto it = otherWormBlobAssignments.begin(); it != otherWormBlobAssignments.end(); ++it) {
            if (it.value().contains(preferredBlobId)) {
                blobTaken = true;
                TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Preferred blob").arg(signedWormId).arg(frameNumber) << preferredBlobId << "already taken by worm" << it.key();
                break;
            }
        }

        if (!blobTaken) {
            // Try to find and claim this blob
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.uniqueId == preferredBlobId && blob.selectedByWormTrackerId == 0) {
                    // Claim the blob
                    blob.selectedByWormTrackerId = signedWormId;

                    // Create the blob to assign
                    blobToAssign.isValid = true;
                    blobToAssign.centroid = QPointF(blob.currentCentroid.x, blob.currentCentroid.y);
                    blobToAssign.boundingBox = blob.currentBoundingBox;
                    blobToAssign.area = blob.currentArea;
                    blobToAssign.contourPoints = blob.contourPoints;
                    blobToAssign.holeContourPoints = blob.holeContourPoints;

                    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** CLAIMED PREFERRED BLOB ***").arg(signedWormId).arg(frameNumber) << preferredBlobId
                                      << "Area:" << blobToAssign.area
                                      << "Pos:" << blobToAssign.centroid.x() << "," << blobToAssign.centroid.y();
                    resolutionSuccess = true;
                    break;
                }
            }
        }
    }

    // If we couldn't claim our preferred blob, look for any available alternative
    if (!blobToAssign.isValid) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** PREFERRED BLOB UNAVAILABLE - LOOKING FOR ALTERNATIVES ***").arg(signedWormId).arg(frameNumber);
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|PreferredBlobId was:").arg(signedWormId).arg(frameNumber) << preferredBlobId;

        // Get the worm tracker instance to find its last known position
        //WormTracker* tracker = nullptr;
        //for (WormTracker* trackerPtr : m_wormTrackersList) {
        //    if (trackerPtr && getSignedWormId(trackerPtr->getWormId(), trackerPtr->getDirection()) == conceptualWormId) {
        //        tracker = trackerPtr;
        //        break;
        //    }
        // }

        if (trackerInstance) {
            // Find all unassigned blobs that this worm is participating in
            struct BlobWithDistance {
                FrameSpecificPhysicalBlob* blob;
                double distance;
            };
            QList<BlobWithDistance> availableBlobs;

            // Get the last known position
            QPointF lastKnownPos;
            if (chosenCandidate.isValid) {
                // Use the chosen candidate as reference if available
                lastKnownPos = chosenCandidate.centroid;
            } else {
                // Try to use the tracker's last known position
                cv::Point2f trackerPos = trackerInstance->getLastKnownPosition();
                lastKnownPos = QPointF(trackerPos.x, trackerPos.y);
            }

            // Find all available blobs and calculate distances
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.participatingWormTrackerIDs.contains(signedWormId) && blob.selectedByWormTrackerId == 0) {
                    QPointF blobPos(blob.currentCentroid.x, blob.currentCentroid.y);
                    double dx = blobPos.x() - lastKnownPos.x();
                    double dy = blobPos.y() - lastKnownPos.y();
                    double distSq = dx*dx + dy*dy;

                    BlobWithDistance bwd;
                    bwd.blob = &blob;
                    bwd.distance = distSq;
                    availableBlobs.append(bwd);
                }
            }

            // Sort blobs by distance (closest first)
            std::sort(availableBlobs.begin(), availableBlobs.end(),
                     [](const BlobWithDistance& a, const BlobWithDistance& b) {
                         return a.distance < b.distance;
                     });

            // Use the closest blob if available
            if (!availableBlobs.isEmpty()) {
                FrameSpecificPhysicalBlob* closestBlob = availableBlobs.first().blob;

                // Claim this blob
                closestBlob->selectedByWormTrackerId = signedWormId;

                // Update the mapping
                m_wormToPhysicalBlobIdMap[signedWormId] = closestBlob->uniqueId;

                // Create the blob to assign
                blobToAssign.isValid = true;
                blobToAssign.centroid = QPointF(closestBlob->currentCentroid.x, closestBlob->currentCentroid.y);
                blobToAssign.boundingBox = closestBlob->currentBoundingBox;
                blobToAssign.area = closestBlob->currentArea;
                blobToAssign.contourPoints = closestBlob->contourPoints;
                blobToAssign.holeContourPoints = closestBlob->holeContourPoints;

                TRACKING_DEBUG() << QString("TM: %1|FN%2|*** CLAIMED ALTERNATIVE BLOB ***").arg(signedWormId).arg(frameNumber) << closestBlob->uniqueId
                                  << "Area:" << blobToAssign.area
                                  << "Pos:" << blobToAssign.centroid.x() << "," << blobToAssign.centroid.y()
                                  << "Distance:" << QString::number(std::sqrt(availableBlobs.first().distance), 'f', 2);
                resolutionSuccess = true;
            } else {
                TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|No available alternative blobs found.").arg(signedWormId).arg(frameNumber);
            }
        } else {
            // Fallback to original method if we can't find the tracker
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Couldn't find tracker, using original method.").arg(signedWormId).arg(frameNumber);

            // Find any unassigned blob that this worm is participating in
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.participatingWormTrackerIDs.contains(signedWormId) && blob.selectedByWormTrackerId == 0) {
                    // Claim this blob
                    blob.selectedByWormTrackerId = signedWormId;

                    // Update the mapping
                    m_wormToPhysicalBlobIdMap[signedWormId] = blob.uniqueId;

                    // Create the blob to assign
                    blobToAssign.isValid = true;
                    blobToAssign.centroid = QPointF(blob.currentCentroid.x, blob.currentCentroid.y);
                    blobToAssign.boundingBox = blob.currentBoundingBox;
                    blobToAssign.area = blob.currentArea;
                    blobToAssign.contourPoints = blob.contourPoints;
                    blobToAssign.holeContourPoints = blob.holeContourPoints;

                    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** CLAIMED FALLBACK BLOB ***").arg(signedWormId).arg(frameNumber) << blob.uniqueId
                                      << "Area:" << blobToAssign.area
                                      << "Pos:" << blobToAssign.centroid.x() << "," << blobToAssign.centroid.y();
                    resolutionSuccess = true;
                    break;
                }
            }
        }
    }

    // Assign blob or go lost
    if (blobToAssign.isValid) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** FINAL ASSIGNMENT *** Worm").arg(signedWormId).arg(frameNumber) << signedWormId
                          << "gets blob Area:" << blobToAssign.area
                          << "Pos:" << blobToAssign.centroid.x() << "," << blobToAssign.centroid.y();
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, blobToAssign));
        m_splitResolutionMap[frameNumber][signedWormId] = blobToAssign;
        // Annotate this worm's track point for this frame as a Split for downstream storage/visualization
        int unsignedWormId = getUnsignedWormId(signedWormId);
        WormObject* wobj = m_wormObjectsMap.value(unsignedWormId, nullptr);
        if (wobj) {
            Tracking::WormTrackPoint splitPoint;
            splitPoint.frameNumberOriginal = frameNumber;
            splitPoint.position = cv::Point2f(static_cast<float>(blobToAssign.centroid.x()), static_cast<float>(blobToAssign.centroid.y()));
            splitPoint.roi = blobToAssign.boundingBox;
            splitPoint.quality = Tracking::TrackPointQuality::Split;
            wobj->updateTrackPoint(splitPoint);
        }
    } else {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** NO VALID BLOB - GOING LOST ***").arg(signedWormId).arg(frameNumber);
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][signedWormId] = Tracking::DetectedBlob();
    }

    // Show final blob assignments after this worm's resolution
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|=== POST-RESOLUTION BLOB ASSIGNMENTS ===").arg(signedWormId).arg(frameNumber);
    for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|PhysicalBlob").arg(signedWormId).arg(frameNumber) << blob.uniqueId
                          << "selectedBy:" << blob.selectedByWormTrackerId
                          << "Area:" << blob.currentArea;
    }

    return true;
}

// This function has been replaced by immediate resolution in processFrameSpecificSplit

// This function has been replaced by immediate resolution in processFrameSpecificSplit

// --- Helper and Utility functions (calculateIoU, launchWormTrackers, progress, finish handlers etc.) ---
// These remain largely the same as your version with parallel video processing.
// calculateIoU is used by processFrameSpecificMerge.
// launchWormTrackers, updateOverallProgress, checkForAllTrackersFinished, outputTracksToWorkbook,
// handleWormTrackerFinished, handleWormTrackerError, handleWormTrackerProgress are mostly independent
// of the merge group internal logic, dealing with tracker lifecycle and overall progress.

double TrackingManager::calculateIoU(const QRectF& r1, const QRectF& r2) const {
    QRectF intersection = r1.intersected(r2);
    double intersectionArea = intersection.width() * intersection.height();
    if (intersectionArea <= 0) return 0.0;
    double unionArea = (r1.width() * r1.height()) + (r2.width() * r2.height()) - intersectionArea;
    return unionArea > 0 ? (intersectionArea / unionArea) : 0.0;
}

void TrackingManager::launchWormTrackers() { /* ... same as your version ... */
    if (m_cancelRequested) { m_isTrackingRunning = false; emit trackingCancelled(); return; }
    if (m_initialWormInfos.empty()) { emit trackingStatusUpdate("No worms selected."); emit trackingFinishedSuccessfully(""); m_isTrackingRunning = false; return; }
    m_expectedTrackersToFinish = 0; m_finishedTrackersCount = 0; m_individualTrackerProgress.clear();
    m_wormIdToForwardTrackerInstanceMap.clear(); m_wormIdToBackwardTrackerInstanceMap.clear();
    for(QPointer<QThread> t : m_trackerThreads) { if(t) delete t; } m_trackerThreads.clear();

    for (const auto& info : m_initialWormInfos) {
        int wId = info.id; QRectF iRoi = info.initialRoi;
        if (!m_finalProcessedForwardFrames.empty() || m_keyFrameNum == (m_videoFrameSize.width > 0 ? static_cast<int>(m_videoFps * (m_totalFramesInVideoHint > 0 ? m_totalFramesInVideoHint : 1) / m_videoFps) -1 : 0) ) {
            WormTracker* trk = new WormTracker(wId, iRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
            trk->setFrames(&m_finalProcessedForwardFrames); QThread* thr = new QThread(); trk->moveToThread(thr);
            trk->setProperty("wormId", wId); trk->setProperty("direction", "Forward"); m_trackerThreads.append(thr);
            connect(thr, &QThread::started, trk, &WormTracker::startTracking);
            connect(trk, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(thr, &QThread::finished, trk, &QObject::deleteLater);
            connect(thr, &QThread::finished, this, [this, tPtr=QPointer<QThread>(thr), wId](){if(tPtr){/*qDebug()<<"FwdTrkThr fin" << wId;*/}});
            connect(trk, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(trk, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(trk, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
            m_wormTrackersList.append(trk); m_wormIdToForwardTrackerInstanceMap[wId] = trk; m_individualTrackerProgress[trk] = 0;
            thr->start(); m_expectedTrackersToFinish++;
        }
        if (!m_finalProcessedReversedFrames.empty() || m_keyFrameNum == 0) {
            WormTracker* trk = new WormTracker(wId, iRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
            trk->setFrames(&m_finalProcessedReversedFrames); QThread* thr = new QThread(); trk->moveToThread(thr);
            trk->setProperty("wormId", wId); trk->setProperty("direction", "Backward"); m_trackerThreads.append(thr);
            connect(thr, &QThread::started, trk, &WormTracker::startTracking);
            connect(trk, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(thr, &QThread::finished, trk, &QObject::deleteLater);
            connect(thr, &QThread::finished, this, [this, tPtr=QPointer<QThread>(thr), wId](){if(tPtr){/*qDebug()<<"BwdTrkThr fin"<< wId;*/}});
            connect(trk, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(trk, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(trk, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
            m_wormTrackersList.append(trk); m_wormIdToBackwardTrackerInstanceMap[wId] = trk; m_individualTrackerProgress[trk] = 0;
            thr->start(); m_expectedTrackersToFinish++;
        }
    }
    if (m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) { emit trackingFailed("No trackers launched."); m_isTrackingRunning = false; }
    else if (m_expectedTrackersToFinish > 0) { emit trackingStatusUpdate(QString("Launching %1 trackers...").arg(m_expectedTrackersToFinish)); }
    updateOverallProgress();
}
void TrackingManager::updateOverallProgress() { /* ... same as your version ... */
    if (m_cancelRequested && !m_isTrackingRunning) { emit overallTrackingProgress(m_videoProcessingOverallProgress); return; }
    if (!m_isTrackingRunning && !m_cancelRequested) { emit overallTrackingProgress(0); return; }
    double totalProgressValue = 0.0; double videoProcWeight = (m_expectedTrackersToFinish > 0 || m_initialWormInfos.empty()) ? 0.20 : 1.0;
    double trackersWeight = 1.0 - videoProcWeight;
    totalProgressValue += (static_cast<double>(m_videoProcessingOverallProgress) / 100.0) * videoProcWeight;
    double overallTrackerPercentage = 0.0;
    { QMutexLocker locker(&m_dataMutex);
        if (m_expectedTrackersToFinish > 0) {
            double currentPoints = 0; for(int p : m_individualTrackerProgress.values()) currentPoints += p;
            currentPoints += (static_cast<double>(m_finishedTrackersCount) * 100.0);
            double maxPoints = static_cast<double>(m_expectedTrackersToFinish) * 100.0;
            if (maxPoints > 0) overallTrackerPercentage = currentPoints / maxPoints;
            overallTrackerPercentage = qBound(0.0, overallTrackerPercentage, 1.0);
        } else if (m_videoProcessingOverallProgress == 100 && m_initialWormInfos.empty()) overallTrackerPercentage = 1.0;
        else if (m_videoProcessingOverallProgress == 100 && m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) overallTrackerPercentage = 0.0;
    }
    if (m_expectedTrackersToFinish > 0 || (m_initialWormInfos.empty() && videoProcWeight < 1.0) ) totalProgressValue += overallTrackerPercentage * trackersWeight;
    emit overallTrackingProgress(qBound(0, static_cast<int>(totalProgressValue * 100.0), 100));
}
void TrackingManager::checkForAllTrackersFinished() { /* ... same as your version ... */
    QMutexLocker locker(&m_dataMutex);
    bool allDoneOrCancelled = false;
    if (m_isTrackingRunning && (m_finishedTrackersCount >= m_expectedTrackersToFinish)) allDoneOrCancelled = true;
    else if (m_cancelRequested && (m_finishedTrackersCount >= m_expectedTrackersToFinish || m_expectedTrackersToFinish == 0) ) allDoneOrCancelled = true;

    if (allDoneOrCancelled) {
        bool wasCancelled = m_cancelRequested; m_isTrackingRunning = false;

        // Clear any remaining split resolutions
        m_splitResolutionMap.clear();

        locker.unlock();
        if (wasCancelled) {
            m_finalTracks.clear(); for (WormObject* w : m_wormObjectsMap.values()) { if(w) m_finalTracks[w->getId()] = w->getTrackHistory(); }
            if (!m_finalTracks.empty()) emit allTracksUpdated(m_finalTracks);
            emit trackingStatusUpdate("Tracking cancelled."); emit trackingCancelled();
        } else {
            m_finalTracks.clear(); for (WormObject* w : m_wormObjectsMap.values()) { if(w) m_finalTracks[w->getId()] = w->getTrackHistory(); }
            emit allTracksUpdated(m_finalTracks);
            QString outputPath = trackWorkbookOutputPath();
            if (outputTracksToWorkbook(m_finalTracks, outputPath)) {
                emit trackingStatusUpdate("Tracks saved: " + outputPath);

                // Populate merge history storage in one batch before saving JSON state
                populateMergeHistoryInStorage();

                if (!m_processingOutputDirectory.isEmpty()) {
                    // worms.json now contains everything (tracks, centerlines, merge state)
                    if (!saveWormsJson(m_processingOutputDirectory)) {
                        emit trackingStatusUpdate("Warning: Failed to save worms.json");
                    }
                    if (!saveRoiPointsJson(m_processingOutputDirectory)) {
                        emit trackingStatusUpdate("Warning: Failed to save roi_points.json");
                    }
                }

                emit trackingFinishedSuccessfully(outputPath);

                // Launch post-tracking centerline computation in background.
                // cleanup is deferred until handleCenterlineFinished.
                QMetaObject::invokeMethod(this, "startCenterlineComputation", Qt::QueuedConnection);
                return; // skip the cleanup block below; centerline handler will call it
            }
            else { emit trackingStatusUpdate("Failed to save workbook: " + outputPath); emit trackingFailed("Failed to save workbook."); }
        }
        QMetaObject::invokeMethod(this, "cleanupThreadsAndObjects", Qt::QueuedConnection);
    } else if (!m_isTrackingRunning && m_cancelRequested) {
        locker.unlock(); emit trackingCancelled();
    }
}
void TrackingManager::handleWormTrackerFinished() { /* ... same as your version ... */
    WormTracker* ft = qobject_cast<WormTracker*>(sender()); if (!ft) { QMutexLocker l(&m_dataMutex); if(m_expectedTrackersToFinish > m_finishedTrackersCount) m_finishedTrackersCount++; l.unlock(); checkForAllTrackersFinished(); return; }
    int cId = ft->getWormId(); WormTracker::TrackingDirection dir = ft->getDirection();
    QMutexLocker locker(&m_dataMutex); m_wormTrackersList.removeOne(ft);
    if(dir == WormTracker::TrackingDirection::Forward) m_wormIdToForwardTrackerInstanceMap.remove(cId); else m_wormIdToBackwardTrackerInstanceMap.remove(cId);
    m_individualTrackerProgress.remove(ft); m_finishedTrackersCount++; locker.unlock();
    updateOverallProgress(); checkForAllTrackersFinished();
}
void TrackingManager::handleWormTrackerError(int reportingWormId, QString errorMessage) { /* ... same as your version ... */
    WormTracker* et = qobject_cast<WormTracker*>(sender());
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) { /* Handle error during cancel/stop */ if(et && m_wormTrackersList.removeOne(et)){ m_individualTrackerProgress.remove(et); if(m_expectedTrackersToFinish > m_finishedTrackersCount) m_finishedTrackersCount++;} locker.unlock(); updateOverallProgress(); checkForAllTrackersFinished(); return; }
    locker.unlock();
    //qWarning() << "Error from tracker" << reportingWormId << ":" << errorMessage;
    locker.relock();
    if (et && m_wormTrackersList.removeOne(et)) {
        m_individualTrackerProgress.remove(et);
        if(et->getDirection() == WormTracker::TrackingDirection::Forward) m_wormIdToForwardTrackerInstanceMap.remove(reportingWormId); else m_wormIdToBackwardTrackerInstanceMap.remove(reportingWormId);
        m_finishedTrackersCount++;
    } else if (!et && m_expectedTrackersToFinish > m_finishedTrackersCount) { m_finishedTrackersCount++; }
    locker.unlock();
    emit trackingStatusUpdate(QString("Error tracker %1.").arg(reportingWormId));
    updateOverallProgress(); checkForAllTrackersFinished();
}
void TrackingManager::handleWormTrackerProgress(int, int percentDone) { /* ... same as your version ... */
    if (m_cancelRequested || !m_isTrackingRunning) return; WormTracker* trk = qobject_cast<WormTracker*>(sender());
    QMutexLocker locker(&m_dataMutex); if (trk && m_wormTrackersList.contains(trk)) { m_individualTrackerProgress[trk] = percentDone; locker.unlock(); updateOverallProgress(); }
}
QString TrackingManager::trackWorkbookOutputPath() const
{
    const QString baseName = QFileInfo(m_videoPath).completeBaseName();
    const QString workbookName = baseName.isEmpty()
        ? QStringLiteral("tracks.xlsx")
        : baseName + QStringLiteral("_tracks.xlsx");

    if (!m_processingOutputDirectory.isEmpty() && QDir(m_processingOutputDirectory).exists()) {
        return QDir(m_processingOutputDirectory).filePath(workbookName);
    }
    if (m_videoPath.isEmpty()) {
        return QStringLiteral("tracks.xlsx");
    }
    if (!m_videoSpecificDirectory.isEmpty() && QDir(m_videoSpecificDirectory).exists()) {
        return QDir(m_videoSpecificDirectory).filePath(workbookName);
    }
    // Fallback to video directory if video-specific directory is not available.
    return QDir(QFileInfo(m_videoPath).absolutePath()).filePath(workbookName);
}

// OUTPUT: {processingOutputDir}/{basename}_tracks.xlsx
// FORMAT: XLSX (Office Open XML, written as a ZIP with hand-built XML)
// DATA:   Four sheets:
//   "Tracks"          — one row per (worm, frame): WormID, SourceItemID, Frame,
//                        PositionX/Y (4 dp), RoiX/Y/Width/Height, Quality.
//   "start-end coords"— StartPoint and EndPoint items: PointType, SourceItemID,
//                        FrameSelected, PositionX/Y (4 dp).
//   "Parameters"      — Capture Rate (fps) and Pixel Size (pixels/µm).
//   "Centerlines"     — one row per (worm, frame): WormID, SourceItemID, Frame,
//                        TipToTipDistance, Point1..10 X/Y (10 resampled centerline points).
// TRIGGER: Written at tracking finalization, then rewritten after centerline finalization.
bool TrackingManager::outputTracksToWorkbook(const Tracking::AllWormTracks& tracks, const QString& outputFilePath) const {
    if (outputFilePath.isEmpty()) {
        return false;
    }

    QList<WorkbookRow> trackRows;
    trackRows.append(WorkbookRow{
        stringCell("WormID"),
        stringCell("SourceItemID"),
        stringCell("Frame"),
        stringCell("PositionX"),
        stringCell("PositionY"),
        stringCell("RoiX"),
        stringCell("RoiY"),
        stringCell("RoiWidth"),
        stringCell("RoiHeight"),
        stringCell("Quality")
    });

    QMap<int, int> sourceToExportId;
    int nextExportId = 1;
    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        sourceToExportId.insert(it->first, nextExportId++);
    }

    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        const int sourceItemId = it->first;
        const int exportWormId = sourceToExportId.value(sourceItemId);
        std::vector<Tracking::WormTrackPoint> sortedTrackPoints = it->second;
        std::sort(sortedTrackPoints.begin(), sortedTrackPoints.end(),
                  [](const Tracking::WormTrackPoint& lhs, const Tracking::WormTrackPoint& rhs) {
                      return lhs.frameNumberOriginal < rhs.frameNumberOriginal;
                  });

        for (const Tracking::WormTrackPoint& point : sortedTrackPoints) {
            trackRows.append(WorkbookRow{
                numberCell(QString::number(exportWormId)),
                numberCell(QString::number(sourceItemId)),
                numberCell(QString::number(point.frameNumberOriginal)),
                numberCell(QString::number(static_cast<double>(point.position.x), 'f', 4)),
                numberCell(QString::number(static_cast<double>(point.position.y), 'f', 4)),
                numberCell(QString::number(point.roi.x(), 'f', 2)),
                numberCell(QString::number(point.roi.y(), 'f', 2)),
                numberCell(QString::number(point.roi.width(), 'f', 2)),
                numberCell(QString::number(point.roi.height(), 'f', 2)),
                numberCell(QString::number(static_cast<int>(point.quality)))
            });
        }
    }

    QList<WorkbookRow> startEndRows;
    startEndRows.append(WorkbookRow{
        stringCell("PointType"),
        stringCell("SourceItemID"),
        stringCell("FrameSelected"),
        stringCell("PositionX"),
        stringCell("PositionY")
    });

    const TableItems::ClickedItem* startPoint = nullptr;
    const TableItems::ClickedItem* endPoint = nullptr;
    if (m_storage) {
        const QList<TableItems::ClickedItem>& items = m_storage->getAllItems();
        for (const TableItems::ClickedItem& item : items) {
            if (item.type == TableItems::ItemType::StartPoint) {
                startPoint = &item;
            } else if (item.type == TableItems::ItemType::EndPoint) {
                endPoint = &item;
            }
        }
    }

    const auto appendPointRow = [&startEndRows](const TableItems::ClickedItem* item) {
        if (!item) {
            return;
        }

        startEndRows.append(WorkbookRow{
            stringCell(TableItems::itemTypeToString(item->type)),
            numberCell(QString::number(item->id)),
            numberCell(QString::number(item->frameOfSelection)),
            numberCell(QString::number(item->initialCentroid.x(), 'f', 4)),
            numberCell(QString::number(item->initialCentroid.y(), 'f', 4))
        });
    };

    appendPointRow(startPoint);
    appendPointRow(endPoint);

    QList<WorkbookRow> parameterRows;
    parameterRows.append(WorkbookRow{
        stringCell("Parameter"),
        stringCell("Value"),
        stringCell("Unit")
    });
    parameterRows.append(WorkbookRow{
        stringCell("Capture Rate"),
        numberCell(QString::number(m_videoFps, 'f', 6)),
        stringCell("fps")
    });
    parameterRows.append(WorkbookRow{
        stringCell("Pixel Size"),
        numberCell(QString::number(m_pixelSizePixelsPerUm, 'f', 6)),
        stringCell("pixels/um")
    });

    QList<WorkbookRow> centerlineRows;
    WorkbookRow centerlineHeader{
        stringCell("WormID"),
        stringCell("SourceItemID"),
        stringCell("Frame"),
        stringCell("TipToTipDistance")
    };
    for (int pointIndex = 1; pointIndex <= 10; ++pointIndex) {
        centerlineHeader.append(stringCell(QString("Point%1X").arg(pointIndex)));
        centerlineHeader.append(stringCell(QString("Point%1Y").arg(pointIndex)));
    }
    centerlineRows.append(centerlineHeader);

    if (m_storage) {
        for (auto it = tracks.begin(); it != tracks.end(); ++it) {
            const int sourceItemId = it->first;
            const int exportWormId = sourceToExportId.value(sourceItemId);
            std::vector<Tracking::WormTrackPoint> sortedTrackPoints = it->second;
            std::sort(sortedTrackPoints.begin(), sortedTrackPoints.end(),
                      [](const Tracking::WormTrackPoint& lhs, const Tracking::WormTrackPoint& rhs) {
                          return lhs.frameNumberOriginal < rhs.frameNumberOriginal;
                      });

            for (const Tracking::WormTrackPoint& point : sortedTrackPoints) {
                QList<QPointF> centerlinePoints;
                if (point.quality != Tracking::TrackPointQuality::Lost) {
                    const QMap<int, Tracking::DetectedBlob> blobsForFrame =
                        m_storage->getDetectedBlobsForFrame(point.frameNumberOriginal);
                    const auto blobIt = blobsForFrame.constFind(sourceItemId);
                    if (blobIt != blobsForFrame.constEnd()) {
                        centerlinePoints =
                            Tracking::resampleCenterlinePoints(blobIt.value().centerlinePoints, 10);
                    }

                }

                WorkbookRow centerlineRow{
                    numberCell(QString::number(exportWormId)),
                    numberCell(QString::number(sourceItemId)),
                    numberCell(QString::number(point.frameNumberOriginal))
                };
                const double distance = tipToTipDistance(centerlinePoints);
                if (distance >= 0.0) {
                    centerlineRow.append(numberCell(QString::number(distance, 'f', 4)));
                } else {
                    centerlineRow.append(stringCell(""));
                }
                for (int pointIndex = 0; pointIndex < 10; ++pointIndex) {
                    if (pointIndex < centerlinePoints.size()) {
                        const QPointF& centerlinePoint = centerlinePoints.at(pointIndex);
                        centerlineRow.append(numberCell(QString::number(centerlinePoint.x(), 'f', 4)));
                        centerlineRow.append(numberCell(QString::number(centerlinePoint.y(), 'f', 4)));
                    } else {
                        centerlineRow.append(stringCell(""));
                        centerlineRow.append(stringCell(""));
                    }
                }
                centerlineRows.append(centerlineRow);
            }
        }
    }

    QList<ZipEntry> workbookEntries{
        ZipEntry{"[Content_Types].xml", buildContentTypesXml()},
        ZipEntry{"_rels/.rels", buildRootRelsXml()},
        ZipEntry{"xl/workbook.xml", buildWorkbookXml()},
        ZipEntry{"xl/_rels/workbook.xml.rels", buildWorkbookRelsXml()},
        ZipEntry{"xl/styles.xml", buildStylesXml()},
        ZipEntry{"xl/worksheets/sheet1.xml", buildWorksheetXml(trackRows)},
        ZipEntry{"xl/worksheets/sheet2.xml", buildWorksheetXml(startEndRows)},
        ZipEntry{"xl/worksheets/sheet3.xml", buildWorksheetXml(parameterRows)},
        ZipEntry{"xl/worksheets/sheet4.xml", buildWorksheetXml(centerlineRows)}
    };

    return writeStoredZip(outputFilePath, workbookEntries);
}
QString TrackingManager::createVideoSpecificDirectory(const QString& dataDirectory, const QString& videoPath) {
    if (dataDirectory.isEmpty() || videoPath.isEmpty()) {
        qWarning() << "TrackingManager: Invalid data directory or video path";
        return QString();
    }

    QFileInfo videoInfo(videoPath);
    QString videoBaseName = videoInfo.completeBaseName(); // Gets filename without extension
    QString videoSpecificPath = QDir(dataDirectory).absoluteFilePath(videoBaseName);

    QDir videoSpecificDir(videoSpecificPath);
    if (!videoSpecificDir.exists()) {
        if (QDir().mkpath(videoSpecificPath)) {
            TRACKING_DEBUG() << "TrackingManager: Created video-specific directory:" << videoSpecificPath;
        } else {
            qWarning() << "TrackingManager: Failed to create video-specific directory:" << videoSpecificPath;
            return QString();
        }
    } else {
        TRACKING_DEBUG() << "TrackingManager: Using existing video-specific directory:" << videoSpecificPath;
    }

    return videoSpecificPath;
}

// ── Head/tail swap xlsx export ─────────────────────────────────────────────
// Minimal single-sheet xlsx helpers, parallel to the multi-sheet track workbook.

static QByteArray buildSwapContentTypesXml()
{
    QByteArray d; QBuffer b(&d); b.open(QIODevice::WriteOnly);
    QXmlStreamWriter x(&b);
    x.writeStartDocument();
    x.writeStartElement("Types");
    x.writeDefaultNamespace(QString::fromLatin1(kContentTypesNs));
    x.writeEmptyElement("Default"); x.writeAttribute("Extension","rels");
    x.writeAttribute("ContentType","application/vnd.openxmlformats-package.relationships+xml");
    x.writeEmptyElement("Default"); x.writeAttribute("Extension","xml");
    x.writeAttribute("ContentType","application/xml");
    x.writeEmptyElement("Override"); x.writeAttribute("PartName","/xl/workbook.xml");
    x.writeAttribute("ContentType","application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml");
    x.writeEmptyElement("Override"); x.writeAttribute("PartName","/xl/styles.xml");
    x.writeAttribute("ContentType","application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml");
    x.writeEmptyElement("Override"); x.writeAttribute("PartName","/xl/worksheets/sheet1.xml");
    x.writeAttribute("ContentType","application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml");
    x.writeEndElement(); x.writeEndDocument();
    return d;
}

static QByteArray buildSwapWorkbookXml()
{
    QByteArray d; QBuffer b(&d); b.open(QIODevice::WriteOnly);
    QXmlStreamWriter x(&b);
    x.writeStartDocument();
    x.writeStartElement("workbook");
    x.writeDefaultNamespace(QString::fromLatin1(kSpreadsheetMainNs));
    x.writeNamespace(QString::fromLatin1(kOfficeDocumentRelsNs),"r");
    x.writeStartElement("sheets");
    x.writeEmptyElement("sheet"); x.writeAttribute("name","HeadTailSwaps");
    x.writeAttribute("sheetId","1"); x.writeAttribute("r:id","rId1");
    x.writeEndElement(); x.writeEndElement(); x.writeEndDocument();
    return d;
}

static QByteArray buildSwapWorkbookRelsXml()
{
    QByteArray d; QBuffer b(&d); b.open(QIODevice::WriteOnly);
    QXmlStreamWriter x(&b);
    x.writeStartDocument();
    x.writeStartElement("Relationships");
    x.writeDefaultNamespace(QString::fromLatin1(kPackageRelsNs));
    x.writeEmptyElement("Relationship"); x.writeAttribute("Id","rId1");
    x.writeAttribute("Type","http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet");
    x.writeAttribute("Target","worksheets/sheet1.xml");
    x.writeEmptyElement("Relationship"); x.writeAttribute("Id","rId2");
    x.writeAttribute("Type","http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles");
    x.writeAttribute("Target","styles.xml");
    x.writeEndElement(); x.writeEndDocument();
    return d;
}

// OUTPUT: {directoryPath}/worm_summary.json
// FORMAT: JSON (indented)
// DATA:   Per-worm statistics for use by the analysis plugin system: frame quality counts,
//           distance, displacement, speed, and morphology averages (area, body length, aspect ratio).
//           All _um fields are 0 when no pixel calibration is available.
// TRIGGER: Written once at end of centerline computation (tracking finalization).
void TrackingManager::saveWormSummaryJson(const QString& directoryPath) const
{
    if (directoryPath.isEmpty() || !m_storage) return;

    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    const double umPerPixel = (m_pixelSizePixelsPerUm > 0) ? 1.0 / m_pixelSizePixelsPerUm : 0.0;

    QJsonObject wormsObj;

    QList<int> sortedIds;
    for (const auto& entry : tracks) sortedIds.append(entry.first);
    std::sort(sortedIds.begin(), sortedIds.end());

    for (int wormId : sortedIds) {
        const auto& rawPoints = tracks.at(wormId);
        if (rawPoints.empty()) continue;

        std::vector<Tracking::WormTrackPoint> pts = rawPoints;
        std::sort(pts.begin(), pts.end(),
                  [](const Tracking::WormTrackPoint& a, const Tracking::WormTrackPoint& b) {
                      return a.frameNumberOriginal < b.frameNumberOriginal;
                  });

        int nSingle = 0, nSplit = 0, nMerged = 0, nLost = 0;
        double totalDist = 0.0;
        double sumAreaPx = 0.0;
        double sumBodyLenPx = 0.0;
        double sumAspectRatio = 0.0;
        int nAreaSamples = 0, nBodyLenSamples = 0, nAspectSamples = 0;

        for (const Tracking::WormTrackPoint& tp : pts) {
            using Q = Tracking::TrackPointQuality;
            switch (tp.quality) {
            case Q::Single: ++nSingle; break;
            case Q::Split:  ++nSplit;  break;
            case Q::Merged: ++nMerged; break;
            case Q::Lost:   ++nLost;   break;
            }

            // Morphology from detected blobs (skip merged/lost for cleaner stats)
            if (tp.quality == Q::Single || tp.quality == Q::Split) {
                QMap<int, Tracking::DetectedBlob> blobs =
                    m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
                if (blobs.contains(wormId)) {
                    const Tracking::DetectedBlob& blob = blobs[wormId];
                    if (blob.isValid) {
                        if (blob.area > 0) { sumAreaPx += blob.area; ++nAreaSamples; }
                        // Body length: arc length of centerline
                        if (blob.centerlinePoints.size() >= 2) {
                            double arcLen = 0.0;
                            for (size_t k = 1; k < blob.centerlinePoints.size(); ++k) {
                                cv::Point2f d = blob.centerlinePoints[k] - blob.centerlinePoints[k-1];
                                arcLen += std::sqrt(d.x*d.x + d.y*d.y);
                            }
                            sumBodyLenPx += arcLen;
                            ++nBodyLenSamples;
                        }
                        // Aspect ratio from bounding box
                        if (blob.boundingBox.width() > 0 && blob.boundingBox.height() > 0) {
                            double ar = blob.boundingBox.width() / blob.boundingBox.height();
                            sumAspectRatio += (ar < 1.0 ? 1.0 / ar : ar);
                            ++nAspectSamples;
                        }
                    }
                }
            }
        }

        // Distance: sum displacement between consecutive non-lost frames
        cv::Point2f prevPos{};
        bool hasPrev = false;
        for (const Tracking::WormTrackPoint& tp : pts) {
            if (tp.quality == Tracking::TrackPointQuality::Lost) { hasPrev = false; continue; }
            if (hasPrev) {
                cv::Point2f d = tp.position - prevPos;
                totalDist += std::sqrt(d.x*d.x + d.y*d.y);
            }
            prevPos = tp.position;
            hasPrev = true;
        }

        // Net displacement: first to last non-lost position
        cv::Point2f firstPos{}, lastPos{};
        bool hasFirst = false;
        for (const Tracking::WormTrackPoint& tp : pts) {
            if (tp.quality == Tracking::TrackPointQuality::Lost) continue;
            if (!hasFirst) { firstPos = tp.position; hasFirst = true; }
            lastPos = tp.position;
        }
        cv::Point2f netVec = lastPos - firstPos;
        double netDisp = std::sqrt(netVec.x*netVec.x + netVec.y*netVec.y);

        // Speed: total distance / tracked time (non-lost frames, using FPS)
        int nTrackedFrames = nSingle + nSplit + nMerged;
        double trackedTimeSec = (m_videoFps > 0 && nTrackedFrames > 1)
                                    ? (nTrackedFrames - 1) / m_videoFps : 0.0;
        double meanSpeed = (trackedTimeSec > 0) ? totalDist / trackedTimeSec : 0.0;

        double meanArea       = nAreaSamples    > 0 ? sumAreaPx / nAreaSamples       : 0.0;
        double meanBodyLen    = nBodyLenSamples > 0 ? sumBodyLenPx / nBodyLenSamples : 0.0;
        double meanAspectRatio = nAspectSamples > 0 ? sumAspectRatio / nAspectSamples : 0.0;

        QJsonObject w;
        w["frames_tracked"] = static_cast<int>(pts.size());
        w["frames_single"]  = nSingle;
        w["frames_split"]   = nSplit;
        w["frames_merged"]  = nMerged;
        w["frames_lost"]    = nLost;
        w["total_distance_px"]    = totalDist;
        w["total_distance_um"]    = umPerPixel > 0 ? totalDist * umPerPixel : 0.0;
        w["net_displacement_px"]  = netDisp;
        w["net_displacement_um"]  = umPerPixel > 0 ? netDisp * umPerPixel : 0.0;
        w["mean_speed_px_per_s"]  = meanSpeed;
        w["mean_speed_um_per_s"]  = umPerPixel > 0 ? meanSpeed * umPerPixel : 0.0;
        w["mean_area_px2"]        = meanArea;
        w["mean_area_um2"]        = umPerPixel > 0 ? meanArea * umPerPixel * umPerPixel : 0.0;
        w["mean_body_length_px"]  = meanBodyLen;
        w["mean_body_length_um"]  = umPerPixel > 0 ? meanBodyLen * umPerPixel : 0.0;
        w["mean_aspect_ratio"]    = meanAspectRatio;

        wormsObj[QString::number(wormId)] = w;
    }

    QJsonObject root;
    root["fps"]          = m_videoFps;
    root["um_per_pixel"] = umPerPixel;
    root["frame_width"]  = m_videoFrameSize.width;
    root["frame_height"] = m_videoFrameSize.height;
    root["worms"]        = wormsObj;

    const QString path = QDir(directoryPath).filePath("worm_summary.json");
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        qWarning() << "TrackingManager: could not write worm_summary.json to" << path;
        return;
    }
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    qDebug() << "TrackingManager: worm_summary.json saved to" << path;
}

// OUTPUT: caller-specified path (typically {processingOutputDir}/{basename}_summary.txt)
// FORMAT: UTF-8 plain text (human-readable report)
// DATA:   High-level tracking run report: video metadata (fps, frame range, worm count),
//           per-worm statistics (track frame range, quality breakdown by category,
//           centerline topology, merge event count, head/tail swap count).
// TRIGGER: Written once at tracking finalization.
void TrackingManager::exportProcessingSummary(const QString& outputPath) const
{
    if (outputPath.isEmpty() || !m_storage) return;

    QFile file(outputPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "ProcessingSummary: could not write" << outputPath;
        return;
    }
    QTextStream out(&file);
    out.setEncoding(QStringConverter::Utf8);

    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();

    // Overall frame range
    int minFrame = INT_MAX, maxFrame = INT_MIN;
    for (const auto& entry : tracks) {
        for (const auto& tp : entry.second) {
            minFrame = std::min(minFrame, tp.frameNumberOriginal);
            maxFrame = std::max(maxFrame, tp.frameNumberOriginal);
        }
    }

    // Sorted worm IDs
    QList<int> sortedWormIds;
    for (const auto& entry : tracks) sortedWormIds.append(entry.first);
    std::sort(sortedWormIds.begin(), sortedWormIds.end());

    // Padding helpers
    auto lpad = [](const QString& s, int w) {
        return s.rightJustified(w);
    };
    auto pct = [](int n, int total) -> QString {
        if (total <= 0) return QStringLiteral("  0.0%");
        return QString("%1%").arg(100.0 * n / total, 5, 'f', 1);
    };

    // ── Header ──────────────────────────────────────────────────────────────
    const QString rule = QString(62, '=');
    const QString dash  = QString(62, '-');
    out << rule << "\n";
    out << "  YAWT Processing Summary\n";
    out << rule << "\n";
    out << QString("Generated:        %1\n")
               .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
    out << QString("Video:            %1\n").arg(QFileInfo(m_videoPath).fileName());
    out << QString("FPS:              %1\n").arg(m_videoFps, 0, 'f', 2);
    if (minFrame <= maxFrame) {
        out << QString("Frame range:      %1 \xe2\x80\x93 %2  (%3 frames)\n")
                   .arg(minFrame).arg(maxFrame).arg(maxFrame - minFrame + 1);
    }
    out << QString("Worms tracked:    %1\n").arg(sortedWormIds.size());
    out << "\n";
    out << "Settings\n";
    out << QString("  Centerline computation:  %1\n")
               .arg(m_centerlineEnabled ? "enabled" : "disabled");
    out << QString("  Skip merged frames:      %1\n")
               .arg(m_skipMergedFrames ? "yes" : "no");
    out << QString("  Max reversal fraction:   %1\n")
               .arg(m_maxReversalFraction, 0, 'f', 2);
    out << "\n";

    // ── Per-worm sections ────────────────────────────────────────────────────
    for (int wormId : sortedWormIds) {
        const auto& rawPoints = tracks.at(wormId);
        if (rawPoints.empty()) continue;

        // Sort by frame number
        std::vector<Tracking::WormTrackPoint> pts = rawPoints;
        std::sort(pts.begin(), pts.end(),
                  [](const Tracking::WormTrackPoint& a,
                     const Tracking::WormTrackPoint& b) {
                      return a.frameNumberOriginal < b.frameNumberOriginal;
                  });

        const int total = static_cast<int>(pts.size());
        const int wFirst = pts.front().frameNumberOriginal;
        const int wLast  = pts.back().frameNumberOriginal;

        // Per-point stats
        int nSingle = 0, nSplit = 0, nMerged = 0, nLost = 0;
        int nCLcomputed = 0, nCLskipped = 0;
        int nCleanTopo = 0, nSCTopo = 0;
        int mergeRunCount = 0, mergeRunMaxLen = 0;
        int mergeRunCurrent = 0;
        bool inMergeRun = false;

        for (const Tracking::WormTrackPoint& tp : pts) {
            using Q = Tracking::TrackPointQuality;
            const bool isMerged = (tp.quality == Q::Merged);
            const bool isLost   = (tp.quality == Q::Lost);

            switch (tp.quality) {
            case Q::Single: ++nSingle; break;
            case Q::Split:  ++nSplit;  break;
            case Q::Merged: ++nMerged; break;
            case Q::Lost:   ++nLost;   break;
            }

            // Merge run detection
            if (isMerged) {
                ++mergeRunCurrent;
                if (!inMergeRun) { ++mergeRunCount; inMergeRun = true; }
            } else {
                if (inMergeRun) {
                    mergeRunMaxLen = std::max(mergeRunMaxLen, mergeRunCurrent);
                    mergeRunCurrent = 0;
                }
                inMergeRun = false;
            }

            // Centerline and topology — read the blob for accuracy
            if (isLost) {
                ++nCLskipped;
            } else {
                QMap<int, Tracking::DetectedBlob> blobs =
                    m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
                if (blobs.contains(wormId)) {
                    const Tracking::DetectedBlob& blob = blobs[wormId];
                    if (blob.isValid && blob.centerlinePoints.size() >= 2) {
                        ++nCLcomputed;
                    } else {
                        ++nCLskipped;
                    }
                    using T = Tracking::TopologyState;
                    if (blob.topologyState == T::Clean)       ++nCleanTopo;
                    if (blob.topologyState == T::SelfCrossed) ++nSCTopo;
                } else {
                    ++nCLskipped;
                }
            }
        }
        // Flush last merge run
        if (inMergeRun) mergeRunMaxLen = std::max(mergeRunMaxLen, mergeRunCurrent);

        // Head/tail swap counts
        const int dirSwaps = m_dirHeadTailSwapData.value(wormId).size();
        const int geoSwaps = m_geoHeadTailSwapData.value(wormId).size();
        const int netSwaps = m_headTailSwapData.value(wormId).size();

        out << dash << "\n";
        out << QString("WORM %1\n").arg(wormId);
        out << dash << "\n";
        out << QString("Track range:   frames %1 \xe2\x80\x93 %2  (%3 track points)\n")
                   .arg(wFirst).arg(wLast).arg(total);
        out << "\n";

        // Frame quality
        out << "Frame quality (from tracker)\n";
        out << QString("  Single:      %1  (%2)\n").arg(lpad(QString::number(nSingle), 6)).arg(pct(nSingle, total));
        out << QString("  Split:       %1  (%2)\n").arg(lpad(QString::number(nSplit),  6)).arg(pct(nSplit,  total));
        out << QString("  Merged:      %1  (%2)\n").arg(lpad(QString::number(nMerged), 6)).arg(pct(nMerged, total));
        out << QString("  Lost:        %1  (%2)\n").arg(lpad(QString::number(nLost),   6)).arg(pct(nLost,   total));
        out << "\n";

        // Topology (only meaningful for processed frames)
        const int nTopoTotal = nCleanTopo + nSCTopo;
        out << "Centerline topology (processed frames only)\n";
        out << QString("  Clean:       %1  (%2)\n").arg(lpad(QString::number(nCleanTopo), 6)).arg(pct(nCleanTopo, nTopoTotal));
        out << QString("  Self-crossed:%1  (%2)\n").arg(lpad(QString::number(nSCTopo),    6)).arg(pct(nSCTopo,    nTopoTotal));
        out << "\n";

        // Centerline frames
        out << "Centerline frames\n";
        out << QString("  Computed:    %1\n").arg(lpad(QString::number(nCLcomputed), 6));
        out << QString("  Skipped:     %1\n").arg(lpad(QString::number(nCLskipped),  6));
        out << "\n";

        // Merge events
        out << "Merge events\n";
        out << QString("  Merged frames:   %1\n").arg(lpad(QString::number(nMerged), 6));
        out << QString("  Merge runs:      %1\n").arg(lpad(QString::number(mergeRunCount), 6));
        if (mergeRunCount > 0) {
            const double avgLen = static_cast<double>(nMerged) / mergeRunCount;
            out << QString("  Avg run length:  %1 frames  (%2 s)\n")
                       .arg(avgLen, 6, 'f', 1)
                       .arg(avgLen / m_videoFps, 0, 'f', 1);
            out << QString("  Longest run:     %1 frames  (%2 s)\n")
                       .arg(lpad(QString::number(mergeRunMaxLen), 6))
                       .arg(mergeRunMaxLen / m_videoFps, 0, 'f', 1);
        }
        out << "\n";

        // Head/tail refinement
        out << "Head/tail refinement\n";
        out << QString("  Direction-based swapped frames:  %1\n").arg(lpad(QString::number(dirSwaps), 6));
        out << QString("  Geometry-based swapped frames:   %1\n").arg(lpad(QString::number(geoSwaps), 6));
        out << QString("  Net swapped frames (XOR):        %1\n").arg(lpad(QString::number(netSwaps), 6));
        out << "\n";
    }

    out << rule << "\n";
    out << "End of Processing Summary\n";
    out << rule << "\n";
}

// OUTPUT: {processingOutputDir}/{basename}_headtail_swaps.xlsx
// FORMAT: XLSX (Office Open XML, written as a ZIP with hand-built XML)
// DATA:   One sheet with a row per frame and one column per worm;
//           cells contain "SWAP" on frames where the head/tail assignment was reversed,
//           blank otherwise.
// TRIGGER: Written at tracking finalization only when at least one swap was detected.
void TrackingManager::exportHeadTailSwapXlsx(
    const QMap<int, QList<int>>& swapData, const QString& outputPath) const
{
    if (outputPath.isEmpty() || !m_storage) return;

    // Collect the full frame range from all tracks.
    int minFrame = INT_MAX, maxFrame = INT_MIN;
    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    for (const auto& entry : tracks) {
        for (const auto& tp : entry.second) {
            minFrame = std::min(minFrame, tp.frameNumberOriginal);
            maxFrame = std::max(maxFrame, tp.frameNumberOriginal);
        }
    }
    if (minFrame > maxFrame) return;

    // Sorted worm IDs (columns).
    QList<int> wormIds = swapData.keys();
    std::sort(wormIds.begin(), wormIds.end());

    // Build a fast-lookup set of swapped frames per worm.
    QMap<int, QSet<int>> swapSets;
    for (int wid : wormIds)
        for (int f : swapData[wid])
            swapSets[wid].insert(f);

    // Build sheet rows: header + one row per frame.
    QList<WorkbookRow> rows;

    WorkbookRow header;
    header.append(stringCell("Frame"));
    for (int wid : wormIds)
        header.append(stringCell(QString("Worm %1").arg(wid)));
    rows.append(header);

    for (int f = minFrame; f <= maxFrame; ++f) {
        WorkbookRow row;
        row.append(numberCell(QString::number(f)));
        for (int wid : wormIds)
            row.append(swapSets.value(wid).contains(f)
                ? stringCell("SWAP") : stringCell(""));
        rows.append(row);
    }

    QList<ZipEntry> entries{
        ZipEntry{"[Content_Types].xml",        buildSwapContentTypesXml()},
        ZipEntry{"_rels/.rels",                buildRootRelsXml()},
        ZipEntry{"xl/workbook.xml",            buildSwapWorkbookXml()},
        ZipEntry{"xl/_rels/workbook.xml.rels", buildSwapWorkbookRelsXml()},
        ZipEntry{"xl/styles.xml",              buildStylesXml()},
        ZipEntry{"xl/worksheets/sheet1.xml",   buildWorksheetXml(rows)},
    };
    writeStoredZip(outputPath, entries);
}

QString TrackingManager::createProcessingOutputDirectory(const QString& videoSpecificDirectory) {
    if (videoSpecificDirectory.isEmpty()) {
        qWarning() << "TrackingManager: Invalid video-specific directory";
        return QString();
    }

    QDir baseDir(videoSpecificDirectory);
    if (!baseDir.exists()) {
        qWarning() << "TrackingManager: Video-specific directory does not exist:" << videoSpecificDirectory;
        return QString();
    }

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd-HHmmss");
    QString procDirName = QString("PROC_%1").arg(timestamp);
    QString procPath = baseDir.absoluteFilePath(procDirName);

    if (QDir(procPath).exists()) {
        TRACKING_DEBUG() << "TrackingManager: Processing directory already exists:" << procPath;
        return procPath;
    }

    if (QDir().mkpath(procPath)) {
        TRACKING_DEBUG() << "TrackingManager: Created processing directory:" << procPath;
        return procPath;
    }

    qWarning() << "TrackingManager: Failed to create processing directory:" << procPath;
    return QString();
}

// Batch-populate merge groups into the central TrackingDataStorage.
// Converts FrameSpecificPhysicalBlob records (m_frameMergeRecords) into
// the storage representation: QMap<int, QList<QList<int>>> (frame -> list of groups).
void TrackingManager::populateMergeHistoryInStorage() {
    if (!m_storage) return;
    QMutexLocker locker(&m_dataMutex);
    // Iterate frames in m_frameMergeRecords and convert each FrameSpecificPhysicalBlob.participatingWormTrackerIDs
    // into a QList<int> group of unsigned conceptual IDs, then write to storage.
    for (auto it = m_frameMergeRecords.constBegin(); it != m_frameMergeRecords.constEnd(); ++it) {
        int frameNum = it.key();
        const QList<FrameSpecificPhysicalBlob>& blobs = it.value();
        QList<QList<int>> groups;
        groups.reserve(blobs.size());
        for (const FrameSpecificPhysicalBlob& pb : blobs) {
            QList<int> group;
            group.reserve(pb.participatingWormTrackerIDs.size());
            for (int signedId : pb.participatingWormTrackerIDs) {
                int unsignedId = getUnsignedWormId(signedId);
                if (unsignedId >= 0 && !group.contains(unsignedId)) group.append(unsignedId);
            }
            if (!group.isEmpty()) groups.append(group);
        }
        // Write to storage (storage will silently ignore invalid frames)
        m_storage->setMergeGroupsForFrame(frameNum, groups);
    }
}


// OUTPUT: {processingOutputDir}/thresholding.json
// FORMAT: JSON
// DATA:   Thresholding parameters used for this tracking run:
//           algorithm, globalThresholdValue, adaptiveBlockSize, adaptiveCValue,
//           assumeLightBackground, enableBlur, blurKernelSize, blurSigmaX.
// TRIGGER: Written once at the start of tracking before any frames are processed.
void TrackingManager::saveThresholdingJson(const QString& directoryPath, const Thresholding::ThresholdSettings& settings) {
    if (directoryPath.isEmpty()) {
        qWarning() << "TrackingManager: Cannot save thresholding - empty directory path";
        return;
    }

    QJsonObject jsonObj = thresholdSettingsToJson(settings);
    QJsonDocument doc(jsonObj);

    QString filePath = QDir(directoryPath).absoluteFilePath("thresholding.json");
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        TRACKING_DEBUG() << "TrackingManager: Saved thresholding to:" << filePath;
    } else {
        qWarning() << "TrackingManager: Failed to save thresholding to:" << filePath;
    }
}

// OUTPUT: {processingOutputDir}/worms.json
// FORMAT: JSON (indented)
// DATA:   Single authoritative file for all tracking results:
//           version, videoPath, keyFrame, metrics, items (color, centroid, bounding box),
//           tracks (per-frame position, ROI, quality, centerlinePoints for each worm),
//           mergeGroupsByFrame,
//           mergeState (nextPhysicalBlobId, frameMergeRecords, splitResolutionMap,
//                        wormToPhysicalBlobIdMap — used to resume tracking if settings match).
// TRIGGER: Written once at tracking finalization.  Replaces frame_atomic_state.json.
bool TrackingManager::saveWormsJson(const QString& directoryPath) {
    if (!m_storage) {
        qWarning() << "TrackingManager: Cannot save worms.json - storage missing";
        return false;
    }
    if (directoryPath.isEmpty()) {
        qWarning() << "TrackingManager: Cannot save worms.json - empty directory path";
        return false;
    }

    QJsonObject root;
    root["version"] = 1;
    root["videoPath"] = m_videoPath;
    root["keyFrame"] = m_keyFrameNum;

    QJsonObject metricsObj;
    metricsObj["roiSizeMultiplier"] = m_storage->getRoiSizeMultiplier();
    QSizeF fixed = m_storage->getCurrentFixedRoiSize();
    QJsonObject fixedObj;
    fixedObj["width"] = fixed.width();
    fixedObj["height"] = fixed.height();
    metricsObj["currentFixedRoiSize"] = fixedObj;
    metricsObj["minObservedArea"] = m_storage->getMinObservedArea();
    metricsObj["maxObservedArea"] = m_storage->getMaxObservedArea();
    metricsObj["minObservedAspectRatio"] = m_storage->getMinObservedAspectRatio();
    metricsObj["maxObservedAspectRatio"] = m_storage->getMaxObservedAspectRatio();
    root["metrics"] = metricsObj;

    QJsonArray itemsArr;
    const QList<TableItems::ClickedItem>& items = m_storage->getAllItems();
    for (const TableItems::ClickedItem& item : items) {
        if (item.type != TableItems::ItemType::Worm && item.type != TableItems::ItemType::Fix) {
            continue;
        }
        QJsonObject itemObj;
        itemObj["id"] = item.id;
        itemObj["type"] = TableItems::itemTypeToString(item.type);
        itemObj["visible"] = item.visible;
        itemObj["frameOfSelection"] = item.frameOfSelection;

        QJsonObject colorObj;
        colorObj["r"] = item.color.red();
        colorObj["g"] = item.color.green();
        colorObj["b"] = item.color.blue();
        colorObj["a"] = item.color.alpha();
        colorObj["hex"] = item.color.name(QColor::HexArgb);
        itemObj["color"] = colorObj;

        QJsonObject centroidObj;
        centroidObj["x"] = item.initialCentroid.x();
        centroidObj["y"] = item.initialCentroid.y();
        itemObj["initialCentroid"] = centroidObj;

        QJsonObject bboxObj;
        bboxObj["x"] = item.initialBoundingBox.x();
        bboxObj["y"] = item.initialBoundingBox.y();
        bboxObj["width"] = item.initialBoundingBox.width();
        bboxObj["height"] = item.initialBoundingBox.height();
        itemObj["initialBoundingBox"] = bboxObj;

        QJsonObject origBoxObj;
        origBoxObj["x"] = item.originalClickedBoundingBox.x();
        origBoxObj["y"] = item.originalClickedBoundingBox.y();
        origBoxObj["width"] = item.originalClickedBoundingBox.width();
        origBoxObj["height"] = item.originalClickedBoundingBox.height();
        itemObj["originalClickedBoundingBox"] = origBoxObj;

        itemsArr.append(itemObj);
    }
    root["items"] = itemsArr;
    root["itemsCount"] = itemsArr.size();

    // ── Tracks (with centerline points) ──────────────────────────────────────
    QJsonObject tracksObj;
    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        const int wormId = it->first;
        QJsonArray pointsArr;
        for (const Tracking::WormTrackPoint& p : it->second) {
            QJsonObject pObj;
            pObj["frame"]   = p.frameNumberOriginal;
            pObj["quality"] = static_cast<int>(p.quality);

            QJsonObject posObj;
            posObj["x"] = static_cast<double>(p.position.x);
            posObj["y"] = static_cast<double>(p.position.y);
            pObj["position"] = posObj;

            QJsonObject roiObj;
            roiObj["x"]      = p.roi.x();
            roiObj["y"]      = p.roi.y();
            roiObj["width"]  = p.roi.width();
            roiObj["height"] = p.roi.height();
            pObj["roi"] = roiObj;

            // Blob-derived data from the detected-blob store
            const auto blobsForFrame = m_storage->getDetectedBlobsForFrame(p.frameNumberOriginal);
            const auto blobIt = blobsForFrame.constFind(wormId);
            if (blobIt != blobsForFrame.constEnd()) {
                const Tracking::DetectedBlob& blob = blobIt.value();
                pObj["detectedBlob"] = storageDetectedBlobToJson(blob);

                // Centerline points
                QJsonArray clArr;
                for (const cv::Point2f& pt : blob.centerlinePoints) {
                    QJsonArray a;
                    a.append(static_cast<double>(pt.x));
                    a.append(static_cast<double>(pt.y));
                    clArr.append(a);
                }
                if (!clArr.isEmpty()) pObj["centerlinePoints"] = clArr;

                // Morphology
                if (blob.area > 0)
                    pObj["area"] = blob.area;
                if (blob.boundingBox.width() > 0 && blob.boundingBox.height() > 0) {
                    double ar = blob.boundingBox.width() / blob.boundingBox.height();
                    pObj["aspectRatio"] = ar < 1.0 ? 1.0 / ar : ar;
                }

                // Head/tail tip positions
                const bool hasHead = blob.assignedHeadTipIdx >= 0
                                     && blob.assignedHeadTipIdx < static_cast<int>(blob.tipCandidates.size());
                const bool hasTail = blob.assignedTailTipIdx >= 0
                                     && blob.assignedTailTipIdx < static_cast<int>(blob.tipCandidates.size());
                if (hasHead && hasTail) {
                    const cv::Point2f& h = blob.tipCandidates[blob.assignedHeadTipIdx].point;
                    const cv::Point2f& t = blob.tipCandidates[blob.assignedTailTipIdx].point;
                    QJsonObject tips;
                    QJsonObject headObj; headObj["x"] = static_cast<double>(h.x); headObj["y"] = static_cast<double>(h.y);
                    QJsonObject tailObj; tailObj["x"] = static_cast<double>(t.x); tailObj["y"] = static_cast<double>(t.y);
                    tips["head"] = headObj;
                    tips["tail"] = tailObj;
                    pObj["tips"] = tips;
                }
            }

            pointsArr.append(pObj);
        }
        tracksObj[QString::number(wormId)] = pointsArr;
    }
    root["tracks"]      = tracksObj;
    root["tracksCount"] = static_cast<int>(tracks.size());

    // ── Merge groups (high-level, for display) ────────────────────────────────
    QJsonObject mergeObj;
    const QMap<int, QList<QList<int>>> mergeGroups = m_storage->getAllMergeGroups();
    for (auto it = mergeGroups.constBegin(); it != mergeGroups.constEnd(); ++it) {
        QJsonArray groupsArr;
        for (const QList<int>& group : it.value()) {
            QJsonArray groupArr;
            for (int id : group) groupArr.append(id);
            groupsArr.append(groupArr);
        }
        mergeObj[QString::number(it.key())] = groupsArr;
    }
    root["mergeGroupsByFrame"] = mergeObj;

    // ── Merge state (for tracking resumption) ─────────────────────────────────
    {
        QJsonObject ms;
        ms["nextPhysicalBlobId"] = m_nextPhysicalBlobId;

        // wormToPhysicalBlobIdMap
        QJsonObject wormToPhysObj;
        for (auto it = m_wormToPhysicalBlobIdMap.constBegin();
             it != m_wormToPhysicalBlobIdMap.constEnd(); ++it)
            wormToPhysObj[QString::number(it.key())] = it.value();
        ms["wormToPhysicalBlobIdMap"] = wormToPhysObj;

        // frameMergeRecords
        QJsonObject fmObj;
        for (auto fit = m_frameMergeRecords.constBegin();
             fit != m_frameMergeRecords.constEnd(); ++fit) {
            QJsonArray blobsArr;
            for (const FrameSpecificPhysicalBlob& pb : fit.value()) {
                QJsonObject pbObj;
                pbObj["uniqueId"]    = pb.uniqueId;
                pbObj["frameNumber"] = pb.frameNumber;
                pbObj["currentArea"] = pb.currentArea;

                QJsonObject cObj;
                cObj["x"] = static_cast<double>(pb.currentCentroid.x);
                cObj["y"] = static_cast<double>(pb.currentCentroid.y);
                pbObj["currentCentroid"] = cObj;

                QJsonObject bObj;
                bObj["x"]      = pb.currentBoundingBox.x();
                bObj["y"]      = pb.currentBoundingBox.y();
                bObj["width"]  = pb.currentBoundingBox.width();
                bObj["height"] = pb.currentBoundingBox.height();
                pbObj["currentBoundingBox"] = bObj;

                QJsonArray contourArr;
                for (const cv::Point& pt : pb.contourPoints) {
                    QJsonArray a; a.append(pt.x); a.append(pt.y);
                    contourArr.append(a);
                }
                pbObj["contourPoints"] = contourArr;

                QJsonArray holesArr;
                for (const auto& hole : pb.holeContourPoints) {
                    QJsonArray holeArr;
                    for (const cv::Point& pt : hole) {
                        QJsonArray a; a.append(pt.x); a.append(pt.y);
                        holeArr.append(a);
                    }
                    holesArr.append(holeArr);
                }
                pbObj["holeContourPoints"] = holesArr;

                QJsonArray partArr;
                for (int wid : pb.participatingWormTrackerIDs) partArr.append(wid);
                pbObj["participatingWormTrackerIDs"] = partArr;
                pbObj["selectedByWormTrackerId"] = pb.selectedByWormTrackerId;

                blobsArr.append(pbObj);
            }
            fmObj[QString::number(fit.key())] = blobsArr;
        }
        ms["frameMergeRecords"] = fmObj;

        // splitResolutionMap
        QJsonObject splitObj;
        for (auto sfit = m_splitResolutionMap.constBegin();
             sfit != m_splitResolutionMap.constEnd(); ++sfit) {
            QJsonObject wormMapObj;
            for (auto wit = sfit.value().constBegin();
                 wit != sfit.value().constEnd(); ++wit)
                wormMapObj[QString::number(wit.key())] = storageDetectedBlobToJson(wit.value());
            splitObj[QString::number(sfit.key())] = wormMapObj;
        }
        ms["splitResolutionMap"] = splitObj;

        root["mergeState"] = ms;
    }

    QJsonDocument doc(root);
    QString filePath = QDir(directoryPath).absoluteFilePath("worms.json");
    QString error;
    if (YawtJsonIO::writeCompressedJsonDocument(filePath, doc, &error)) {
        TRACKING_DEBUG() << "TrackingManager: Saved compressed worms.json to:" << filePath;
        return true;
    }

    qWarning() << "TrackingManager: Failed to save worms.json to:" << filePath << error;
    return false;
}

// OUTPUT: {processingOutputDir}/roi_points.json
// FORMAT: JSON (indented)
// DATA:   All user-placed point items (ROI, StartPoint, EndPoint, ControlPoint):
//           version, videoPath, keyFrame, items array mirroring the worms.json item structure.
// TRIGGER: Written once at tracking finalization.
bool TrackingManager::saveRoiPointsJson(const QString& directoryPath) const {
    if (!m_storage) {
        qWarning() << "TrackingManager: Cannot save roi_points.json - storage missing";
        return false;
    }
    if (directoryPath.isEmpty()) {
        qWarning() << "TrackingManager: Cannot save roi_points.json - empty directory path";
        return false;
    }

    QJsonObject root;
    root["version"] = 1;
    root["videoPath"] = m_videoPath;
    root["keyFrame"] = m_keyFrameNum;

    QJsonArray itemsArr;
    const QList<TableItems::ClickedItem>& items = m_storage->getAllItems();
    for (const TableItems::ClickedItem& item : items) {
        if (item.type != TableItems::ItemType::ROI &&
            item.type != TableItems::ItemType::StartPoint &&
            item.type != TableItems::ItemType::EndPoint &&
            item.type != TableItems::ItemType::ControlPoint) {
            continue;
        }
        QJsonObject itemObj;
        itemObj["id"] = item.id;
        itemObj["type"] = TableItems::itemTypeToString(item.type);
        itemObj["visible"] = item.visible;
        itemObj["frameOfSelection"] = item.frameOfSelection;

        QJsonObject colorObj;
        colorObj["r"] = item.color.red();
        colorObj["g"] = item.color.green();
        colorObj["b"] = item.color.blue();
        colorObj["a"] = item.color.alpha();
        colorObj["hex"] = item.color.name(QColor::HexArgb);
        itemObj["color"] = colorObj;

        QJsonObject centroidObj;
        centroidObj["x"] = item.initialCentroid.x();
        centroidObj["y"] = item.initialCentroid.y();
        itemObj["initialCentroid"] = centroidObj;

        QJsonObject bboxObj;
        bboxObj["x"] = item.initialBoundingBox.x();
        bboxObj["y"] = item.initialBoundingBox.y();
        bboxObj["width"] = item.initialBoundingBox.width();
        bboxObj["height"] = item.initialBoundingBox.height();
        itemObj["initialBoundingBox"] = bboxObj;

        QJsonObject origBoxObj;
        origBoxObj["x"] = item.originalClickedBoundingBox.x();
        origBoxObj["y"] = item.originalClickedBoundingBox.y();
        origBoxObj["width"] = item.originalClickedBoundingBox.width();
        origBoxObj["height"] = item.originalClickedBoundingBox.height();
        itemObj["originalClickedBoundingBox"] = origBoxObj;

        itemsArr.append(itemObj);
    }
    root["items"] = itemsArr;
    root["itemsCount"] = itemsArr.size();

    QJsonDocument doc(root);
    QString filePath = QDir(directoryPath).absoluteFilePath("roi_points.json");
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        file.write(doc.toJson(QJsonDocument::Indented));
        file.close();
        TRACKING_DEBUG() << "TrackingManager: Saved roi_points.json to:" << filePath;
        return true;
    }

    qWarning() << "TrackingManager: Failed to save roi_points.json to:" << filePath;
    return false;
}

// OUTPUT: {processingOutputDir}/input_blobs.json
// FORMAT: JSON
// DATA:   Initial blob detections that seeded tracking (one entry per worm):
//           centroid, bounding box, area, convexity, and other blob shape descriptors
//           as captured at the key frame before tracking begins.
// TRIGGER: Written once at tracking start to record what the tracker was initialized with.
void TrackingManager::saveInputBlobs(const QString& directoryPath, const std::vector<Tracking::InitialWormInfo>& worms) {
    if (directoryPath.isEmpty()) {
        qWarning() << "TrackingManager: Cannot save input blobs - empty directory path";
        return;
    }

    QJsonArray wormsArray;
    for (const auto& worm : worms) {
        wormsArray.append(initialWormInfoToJson(worm));
    }

    QJsonObject rootObj;
    rootObj["worms"] = wormsArray;
    rootObj["count"] = static_cast<int>(worms.size());

    QJsonDocument doc(rootObj);
    QString filePath = QDir(directoryPath).absoluteFilePath("input_blobs.json");
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        TRACKING_DEBUG() << "TrackingManager: Saved input blobs to:" << filePath;
    } else {
        qWarning() << "TrackingManager: Failed to save input blobs to:" << filePath;
    }
}

bool TrackingManager::compareThresholdSettings(const QString& filePath, const Thresholding::ThresholdSettings& currentSettings) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "TrackingManager: Cannot read threshold settings file:" << filePath;
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "TrackingManager: JSON parse error in threshold settings:" << parseError.errorString();
        return false;
    }

    QJsonObject stored = doc.object();
    QJsonObject current = thresholdSettingsToJson(currentSettings);

    // Compare key settings and log differences
    bool match = true;
    QStringList differences;

    if (stored["algorithm"].toInt() != current["algorithm"].toInt()) {
        differences << QString("algorithm: %1 vs %2").arg(stored["algorithm"].toInt()).arg(current["algorithm"].toInt());
        match = false;
    }
    if (stored["globalThresholdValue"].toInt() != current["globalThresholdValue"].toInt()) {
        differences << QString("globalThresholdValue: %1 vs %2").arg(stored["globalThresholdValue"].toInt()).arg(current["globalThresholdValue"].toInt());
        match = false;
    }
    if (stored["assumeLightBackground"].toBool() != current["assumeLightBackground"].toBool()) {
        differences << QString("assumeLightBackground: %1 vs %2").arg(stored["assumeLightBackground"].toBool()).arg(current["assumeLightBackground"].toBool());
        match = false;
    }
    if (stored["adaptiveBlockSize"].toInt() != current["adaptiveBlockSize"].toInt()) {
        differences << QString("adaptiveBlockSize: %1 vs %2").arg(stored["adaptiveBlockSize"].toInt()).arg(current["adaptiveBlockSize"].toInt());
        match = false;
    }
    if (qAbs(stored["adaptiveCValue"].toDouble() - current["adaptiveCValue"].toDouble()) >= 0.001) {
        differences << QString("adaptiveCValue: %1 vs %2").arg(stored["adaptiveCValue"].toDouble()).arg(current["adaptiveCValue"].toDouble());
        match = false;
    }
    if (stored["enableBlur"].toBool() != current["enableBlur"].toBool()) {
        differences << QString("enableBlur: %1 vs %2").arg(stored["enableBlur"].toBool()).arg(current["enableBlur"].toBool());
        match = false;
    }
    if (stored["blurKernelSize"].toInt() != current["blurKernelSize"].toInt()) {
        differences << QString("blurKernelSize: %1 vs %2").arg(stored["blurKernelSize"].toInt()).arg(current["blurKernelSize"].toInt());
        match = false;
    }
    if (qAbs(stored["blurSigmaX"].toDouble() - current["blurSigmaX"].toDouble()) >= 0.001) {
        differences << QString("blurSigmaX: %1 vs %2").arg(stored["blurSigmaX"].toDouble()).arg(current["blurSigmaX"].toDouble());
        match = false;
    }

    if (!match) {
        TRACKING_DEBUG() << "TrackingManager: Threshold settings differ:";
        for (const QString& diff : differences) {
            TRACKING_DEBUG() << "  " << diff;
        }
    } else {
        TRACKING_DEBUG() << "TrackingManager: Threshold settings match stored values";
    }

    return match;
}

QJsonObject TrackingManager::thresholdSettingsToJson(const Thresholding::ThresholdSettings& settings) const {
    QJsonObject obj;
    obj["algorithm"] = static_cast<int>(settings.algorithm);
    obj["globalThresholdValue"] = settings.globalThresholdValue;
    obj["assumeLightBackground"] = settings.assumeLightBackground;
    obj["adaptiveBlockSize"] = settings.adaptiveBlockSize;
    obj["adaptiveCValue"] = settings.adaptiveCValue;
    obj["enableBlur"] = settings.enableBlur;
    obj["blurKernelSize"] = settings.blurKernelSize;
    obj["blurSigmaX"] = settings.blurSigmaX;
    return obj;
}

QJsonObject TrackingManager::initialWormInfoToJson(const Tracking::InitialWormInfo& worm) const {
    QJsonObject obj;
    obj["id"] = worm.id;

    QJsonObject roiObj;
    roiObj["x"] = worm.initialRoi.x();
    roiObj["y"] = worm.initialRoi.y();
    roiObj["width"] = worm.initialRoi.width();
    roiObj["height"] = worm.initialRoi.height();
    obj["initialRoi"] = roiObj;

    return obj;
}

size_t TrackingManager::getProcessedVideoMemoryUsage() const {
    size_t totalBytes = 0;

    // Calculate memory usage of forward frames
    for (const cv::Mat& frame : m_finalProcessedForwardFrames) {
        if (!frame.empty()) {
            totalBytes += frame.total() * frame.elemSize();
        }
    }

    // Calculate memory usage of backward frames
    for (const cv::Mat& frame : m_finalProcessedReversedFrames) {
        if (!frame.empty()) {
            totalBytes += frame.total() * frame.elemSize();
        }
    }

    return totalBytes;
}

void TrackingManager::clearProcessedVideoMemory() {
    TRACKING_DEBUG() << "TrackingManager: Clearing processed video memory...";

    // Calculate memory usage before clearing
    size_t memoryUsageBefore = getProcessedVideoMemoryUsage();
    size_t forwardSize = m_finalProcessedForwardFrames.size();
    size_t backwardSize = m_finalProcessedReversedFrames.size();

    // Clear and shrink the vectors to free memory
    m_finalProcessedForwardFrames.clear();
    m_finalProcessedReversedFrames.clear();
    std::vector<cv::Mat>().swap(m_finalProcessedForwardFrames);
    std::vector<cv::Mat>().swap(m_finalProcessedReversedFrames);

    // Also clear intermediate chunk storage if still present
    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();

    // Report memory savings
    double memoryMB = memoryUsageBefore / (1024.0 * 1024.0);
    TRACKING_DEBUG() << "TrackingManager: Cleared" << forwardSize << "forward frames and" << backwardSize
             << "backward frames from memory, freed" << QString::number(memoryMB, 'f', 1) << "MB";

    QString statusMessage = QString("Processed video data cleared from memory (freed %1 MB)")
                           .arg(QString::number(memoryMB, 'f', 1));
    emit trackingStatusUpdate(statusMessage);
}


bool TrackingManager::startRetrackingProcess(const QString& thresholdedVideoPath,
                                           int fixBlobId,
                                           const QRectF& initialROI,
                                           int startFrame,
                                           int endFrame,
                                           bool replaceExisting,
                                           bool extendTrack) {
    TRACKING_DEBUG() << "TrackingManager: Starting retracking process for Fix blob" << fixBlobId
                     << "from frame" << startFrame << "to" << endFrame
                     << "using video:" << thresholdedVideoPath;

    // Check if tracking is already running
    if (m_isTrackingRunning) {
        qWarning() << "TrackingManager: Cannot start retracking while main tracking is running";
        return false;
    }

    // Validate parameters
    if (thresholdedVideoPath.isEmpty() || !QFileInfo::exists(thresholdedVideoPath)) {
        qWarning() << "TrackingManager: Thresholded video file does not exist:" << thresholdedVideoPath;
        return false;
    }

    if (startFrame >= endFrame || startFrame < 0) {
        qWarning() << "TrackingManager: Invalid frame range for retracking:" << startFrame << "to" << endFrame;
        return false;
    }

    if (initialROI.isEmpty()) {
        qWarning() << "TrackingManager: Invalid ROI for retracking";
        return false;
    }

    // TODO: Implement the actual retracking logic here
    // This would involve:
    // 1. Opening the saved thresholded video
    // 2. Creating a specialized tracker for the frame range
    // 3. Running tracking on the specified frames
    // 4. Integrating results with existing track data

    // For now, provide placeholder feedback
    qDebug() << "TrackingManager: Retracking process prepared but not yet fully implemented";
    qDebug() << "TrackingManager: Parameters - ROI:" << initialROI
             << "Replace:" << replaceExisting << "Extend:" << extendTrack;

    emit trackingStatusUpdate(QString("Retracking for Fix Blob ID %1 completed (placeholder)").arg(fixBlobId));

    return true;
}

void TrackingManager::setCenterlineSnakeParams(const Centerline::CenterlineSnakeParams& params)
{
    m_centerlineSnakeParams = params;
}

void TrackingManager::setCenterlineEnabled(bool enabled)
{
    m_centerlineEnabled = enabled;
}

void TrackingManager::setSkipMergedFrames(bool skip)
{
    m_skipMergedFrames = skip;
}

void TrackingManager::setSmoothCenterline(bool smooth)
{
    m_smoothCenterline = smooth;
}

void TrackingManager::setMaxReversalFraction(float fraction)
{
    m_maxReversalFraction = qBound(0.f, fraction, 1.f);
}

void TrackingManager::setLoadedRunContext(const QString& videoPath,
                                          const QString& processingOutputDirectory,
                                          int keyFrameNum)
{
    m_videoPath = videoPath;
    m_processingOutputDirectory = processingOutputDirectory;
    m_videoSpecificDirectory = QFileInfo(processingOutputDirectory).absoluteDir().absolutePath();
    m_keyFrameNum = keyFrameNum;
}

void TrackingManager::startCenterlineComputation() {
    if (!m_centerlineEnabled) {
        emit trackingStatusUpdate("Centerline computation disabled.");
        emit centerlineProgress(100);
        emit centerlineFinished();
        return;
    }

    if (!m_storage) {
        emit centerlineFinished();
        return;
    }

    QList<int> wormIds;
    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    for (const auto& trackEntry : tracks) {
        wormIds.append(trackEntry.first);
    }
    if (wormIds.isEmpty()) {
        emit centerlineProgress(100);
        emit centerlineFinished();
        return;
    }

    int numThreads = QThread::idealThreadCount();
    numThreads = qMax(1, qMin(numThreads, 8));
    numThreads = qMin(numThreads, wormIds.size());

    emit trackingStatusUpdate(QString("Computing centerlines across %1 threads...").arg(numThreads));

    m_centerlineThreads.clear();
    m_centerlineWorkers.clear();
    m_centerlineWorkerProgress.clear();
    m_centerlineWorkersFinishedCount = 0;
    m_totalCenterlineWorkers = numThreads;
    m_centerlineStorageMutex = QSharedPointer<QMutex>::create();
    m_headTailSwapData.clear();
    m_dirHeadTailSwapData.clear();
    m_geoHeadTailSwapData.clear();

    {
        QMutexLocker locker(m_centerlineStorageMutex.data());
        m_storage->clearAllTipBaselines();
    }
    if (m_debugStore) {
        QMutexLocker locker(m_centerlineStorageMutex.data());
        m_debugStore->clearCenterline();
    }

    QVector<QList<int>> wormBuckets(numThreads);
    for (int i = 0; i < wormIds.size(); ++i) {
        wormBuckets[i % numThreads].append(wormIds.at(i));
    }

    for (int workerIndex = 0; workerIndex < numThreads; ++workerIndex) {
        auto* thread = new QThread(this);
        auto* worker = new CenterlineWorker(m_storage, m_debugStore);
        worker->setSnakeParams(m_centerlineSnakeParams);
        worker->setWormIds(wormBuckets.at(workerIndex));
        worker->setClearBaselinesAtStart(false);
        worker->setSkipMergedFrames(m_skipMergedFrames);
        worker->setSmoothCenterline(m_smoothCenterline);
        worker->setFps(m_videoFps);
        worker->setMaxReversalFraction(m_maxReversalFraction);
        worker->setSharedStorageMutex(m_centerlineStorageMutex);
        worker->moveToThread(thread);

        m_centerlineThreads.append(thread);
        m_centerlineWorkers.append(worker);
        m_centerlineWorkerProgress[workerIndex] = 0;

        connect(thread, &QThread::started, worker, &CenterlineWorker::doWork);
        connect(worker, &CenterlineWorker::headTailDirectionSwapEvent, this,
                [this](int wormId, QList<int> swappedFrames) {
                    m_dirHeadTailSwapData[wormId] = swappedFrames;
                });
        connect(worker, &CenterlineWorker::headTailGeometrySwapEvent, this,
                [this](int wormId, QList<int> swappedFrames) {
                    m_geoHeadTailSwapData[wormId] = swappedFrames;
                });
        connect(worker, &CenterlineWorker::headTailSwapEvent, this,
                [this](int wormId, QList<int> swappedFrames) {
                    m_headTailSwapData[wormId] = swappedFrames;
                });
        connect(worker, &CenterlineWorker::progress, this,
                [this, workerIndex](int percentage) {
                    m_centerlineWorkerProgress[workerIndex] = percentage;
                    int total = 0;
                    for (int progress : m_centerlineWorkerProgress) {
                        total += progress;
                    }
                    emit centerlineProgress(total / qMax(1, m_totalCenterlineWorkers));
                });
        connect(worker, &CenterlineWorker::finished, this,
                [this, workerIndex]() {
                    m_centerlineWorkerProgress[workerIndex] = 100;
                    ++m_centerlineWorkersFinishedCount;
                    if (m_centerlineWorkersFinishedCount >= m_totalCenterlineWorkers) {
                        handleCenterlineFinished();
                    }
                });
        connect(worker, &CenterlineWorker::failed, this,
                [this](const QString& reason) {
                    handleCenterlineFailed(reason);
                });
        connect(worker, &CenterlineWorker::finished, thread, &QThread::quit);
        connect(worker, &CenterlineWorker::failed, thread, &QThread::quit);
        connect(thread, &QThread::finished, worker, &QObject::deleteLater);
        connect(thread, &QThread::finished, thread, &QObject::deleteLater);

        thread->start();
    }
}

void TrackingManager::handleCenterlineFinished() {
    if (m_totalCenterlineWorkers == 0) {
        return;
    }
    m_totalCenterlineWorkers = 0;
    emit trackingStatusUpdate("Centerline computation complete.");
    emit centerlineProgress(100);

    const QString dir = !m_processingOutputDirectory.isEmpty()
        ? m_processingOutputDirectory
        : m_videoSpecificDirectory;
    const QString baseName = QFileInfo(m_videoPath).completeBaseName();

    // Rewrite worms.json now that centerlines and head/tail assignments have
    // been written back into storage by the post-tracking centerline workers.
    if (!dir.isEmpty()) {
        if (!saveWormsJson(dir)) {
            emit trackingStatusUpdate("Warning: Failed to save final worms.json");
        }
    }

    const QString workbookPath = trackWorkbookOutputPath();
    if (!workbookPath.isEmpty()) {
        if (outputTracksToWorkbook(m_finalTracks, workbookPath)) {
            emit trackingStatusUpdate("Tracks saved with centerlines: " + workbookPath);
        } else {
            emit trackingStatusUpdate("Warning: Failed to save final workbook with centerlines: " + workbookPath);
        }
    }

    // Export head/tail swap report alongside the other output files.
    if (!m_headTailSwapData.isEmpty() && !dir.isEmpty()) {
        const QString path = QDir(dir).filePath(baseName + "_headtail_swaps.xlsx");
        exportHeadTailSwapXlsx(m_headTailSwapData, path);
        emit trackingStatusUpdate("Head/tail swap report saved: " + path);
    }

    // Export human-readable processing summary.
    if (!dir.isEmpty()) {
        const QString summaryPath = QDir(dir).filePath(baseName + "_ProcessingSummary.txt");
        exportProcessingSummary(summaryPath);
        emit trackingStatusUpdate("Processing summary saved: " + summaryPath);
    }

    // Export machine-readable per-worm summary for the plugin/analysis system.
    if (!dir.isEmpty())
        saveWormSummaryJson(dir);

    emit centerlineFinished();
    QMetaObject::invokeMethod(this, "cleanupThreadsAndObjects", Qt::QueuedConnection);
}

void TrackingManager::handleCenterlineFailed(const QString& reason) {
    if (m_totalCenterlineWorkers == 0) {
        return;
    }
    m_totalCenterlineWorkers = 0;
    for (QPointer<QThread> thread : m_centerlineThreads) {
        if (thread && thread->isRunning()) {
            thread->requestInterruption();
            thread->quit();
        }
    }
    qWarning() << "TrackingManager: Centerline computation failed:" << reason;
    emit trackingStatusUpdate("Warning: Centerline computation failed - " + reason);
    emit centerlineFinished();
    QMetaObject::invokeMethod(this, "cleanupThreadsAndObjects", Qt::QueuedConnection);
}
