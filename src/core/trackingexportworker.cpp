#include "trackingexportworker.h"
#include "centerlinegeometry.h"
#include "../utils/jsongeometry.h"
#include <QBuffer>
#include <QDir>
#include <QFileInfo>
#include <QSaveFile>
#include <QTextStream>
#include <QXmlStreamWriter>
#include <QJsonArray>
#include <QDateTime>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

TrackingExportWorker::TrackingExportWorker(TrackingExportSnapshot snapshot,
                                         std::shared_ptr<void> retiredData)
    : m_snapshot(std::move(snapshot)), m_retiredData(std::move(retiredData))
{
}

const Tracking::DetectedBlob* TrackingExportSnapshot::findBlob(int frame, int wormId) const
{
    const auto fit = worms.blobsByFrame.constFind(frame);
    if (fit == worms.blobsByFrame.constEnd()) return nullptr;
    const auto wit = fit->constFind(wormId);
    return wit == fit->constEnd() ? nullptr : &wit.value();
}

void TrackingExportWorker::collectStatistics()
{
    for (const auto& [wormId, points] : m_snapshot.worms.tracks) {
        WormStatistics stats;
        double sumArea = 0.0, sumLength = 0.0, sumAspect = 0.0;
        int areaSamples = 0, lengthSamples = 0, aspectSamples = 0, mergeRun = 0;
        cv::Point2f previous{}, first{}, last{};
        bool hasPrevious = false, hasFirst = false;
        for (const auto& point : points) {
            using Q = Tracking::TrackPointQuality;
            switch (point.quality) {
            case Q::Single: ++stats.single; break;
            case Q::Split: ++stats.split; break;
            case Q::Merged: ++stats.merged; break;
            case Q::Lost: ++stats.lost; break;
            }
            if (point.quality == Q::Merged) {
                if (mergeRun == 0) ++stats.mergeRuns;
                stats.longestMergeRun = std::max(stats.longestMergeRun, ++mergeRun);
            } else {
                mergeRun = 0;
            }
            if (point.quality == Q::Lost) {
                hasPrevious = false;
                ++stats.centerlineSkipped;
                continue;
            }
            if (hasPrevious) {
                const auto delta = point.position - previous;
                stats.totalDistance += std::sqrt(delta.x * delta.x + delta.y * delta.y);
            }
            previous = point.position;
            hasPrevious = true;
            if (!hasFirst) { first = point.position; hasFirst = true; }
            last = point.position;

            const auto* blob = m_snapshot.findBlob(point.frameNumber, wormId);
            if (!blob) { ++stats.centerlineSkipped; continue; }
            if (blob->isValid && blob->centerline.points.size() >= 2)
                ++stats.centerlineComputed;
            else
                ++stats.centerlineSkipped;
            using T = Tracking::TopologyState;
            if (blob->centerline.topology == T::Clean) ++stats.cleanTopology;
            if (blob->centerline.topology == T::SelfCrossed) ++stats.selfCrossedTopology;
            if (!blob->isValid || (point.quality != Q::Single && point.quality != Q::Split)) continue;
            if (blob->area > 0.0) { sumArea += blob->area; ++areaSamples; }
            if (blob->centerline.points.size() >= 2) {
                double length = 0.0;
                for (size_t i = 1; i < blob->centerline.points.size(); ++i) {
                    const auto delta = blob->centerline.points[i] - blob->centerline.points[i - 1];
                    length += std::sqrt(delta.x * delta.x + delta.y * delta.y);
                }
                sumLength += length;
                ++lengthSamples;
            }
            const auto& box = blob->boundingBox;
            if (box.width() > 0.0 && box.height() > 0.0) {
                const double aspect = box.width() / box.height();
                sumAspect += aspect < 1.0 ? 1.0 / aspect : aspect;
                ++aspectSamples;
            }
        }
        const auto net = last - first;
        stats.netDisplacement = hasFirst ? std::sqrt(net.x * net.x + net.y * net.y) : 0.0;
        stats.meanArea = areaSamples ? sumArea / areaSamples : 0.0;
        stats.meanBodyLength = lengthSamples ? sumLength / lengthSamples : 0.0;
        stats.meanAspectRatio = aspectSamples ? sumAspect / aspectSamples : 0.0;
        m_statistics.insert(wormId, stats);
    }
}

void TrackingExportWorker::run()
{
    QString error;
    try {
        // Normalize once here, off the UI thread; normal runs are already sorted.
        // A snapshot must never retain a callback into mutable live storage.
        m_snapshot.worms.blobLookup = {};
        for (auto& [wormId, points] : m_snapshot.worms.tracks) {
            const auto less = [](const auto& a, const auto& b) { return a.frameNumber < b.frameNumber; };
            if (!std::is_sorted(points.begin(), points.end(), less))
                std::sort(points.begin(), points.end(), less);
        }
        const auto require = [](bool success, const QString& path) {
            if (!success) throw std::runtime_error(QString("Could not save %1").arg(path).toStdString());
        };
        emit progress(0, QStringLiteral("Saving results: tracking data..."));
        if (!m_snapshot.directory.isEmpty()) {
            m_snapshot.worms.mergeState = m_snapshot.mergeStateToJson();
            const QString path = QDir(m_snapshot.directory).filePath("worms.json");
            QString detail;
            if (!WormsJson::write(path, m_snapshot.worms, &detail))
                throw std::runtime_error(QString("Could not save %1: %2").arg(path, detail).toStdString());
            QList<TableItems::AnnotationItem> references;
            for (const auto& item : m_snapshot.items)
                if (item.type == TableItems::ItemType::Region ||
                    item.type == TableItems::ItemType::StartPoint ||
                    item.type == TableItems::ItemType::EndPoint ||
                    item.type == TableItems::ItemType::CenterPoint)
                    references.append(item);
            const QString roiPath = QDir(m_snapshot.directory).filePath("roi_points.json");
            if (!WormsJson::writeRoiPoints(roiPath, m_snapshot.worms.videoPath,
                                          m_snapshot.worms.keyFrame, references, &detail))
                throw std::runtime_error(QString("Could not save %1: %2").arg(roiPath, detail).toStdString());
        }
        emit progress(45, QStringLiteral("Saving results: Excel workbook..."));
        require(outputTracksToWorkbook(m_snapshot.worms.tracks, m_snapshot.workbookPath),
                m_snapshot.workbookPath);
        emit progress(80, QStringLiteral("Saving results: reports..."));
        if (!m_snapshot.directory.isEmpty()) {
            const QString baseName = QFileInfo(m_snapshot.worms.videoPath).completeBaseName();
            const QDir dir(m_snapshot.directory);
            if (!m_snapshot.headTailSwaps.isEmpty() || !m_snapshot.centerlineReviews.isEmpty()) {
                const QString path = dir.filePath(baseName + "_headtail_swaps.xlsx");
                require(exportHeadTailSwapXlsx(m_snapshot.headTailSwaps, m_snapshot.centerlineReviews, path), path);
            }
            collectStatistics();
            const QString summaryPath = dir.filePath(baseName + "_ProcessingSummary.txt");
            require(exportProcessingSummary(summaryPath), summaryPath);
            require(saveWormSummaryJson(m_snapshot.directory), dir.filePath("worm_summary.json"));
        }
    } catch (const std::exception& e) {
        error = QString::fromUtf8(e.what());
    }
    emit progress(95, QStringLiteral("Saving results: releasing temporary data..."));
    m_statistics.clear();
    m_snapshot = TrackingExportSnapshot{};
    m_retiredData.reset();
    emit finished(error);
}

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
    static const std::array<quint32, 256> table = [] {
        std::array<quint32, 256> values{};
        for (quint32 i = 0; i < values.size(); ++i) {
            quint32 crc = i;
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc & 1U) ? (0xEDB88320U ^ (crc >> 1U)) : (crc >> 1U);
            values[i] = crc;
        }
        return values;
    }();

    quint32 crc = 0xFFFFFFFFU;
    for (unsigned char byte : data) {
        crc = table[(crc ^ byte) & 0xFFU] ^ (crc >> 8U);
    }
    return crc ^ 0xFFFFFFFFU;
}

bool writeUInt16(QIODevice& file, quint16 value)
{
    const char bytes[2] = {
        static_cast<char>(value & 0xFF),
        static_cast<char>((value >> 8) & 0xFF)
    };
    return file.write(bytes, sizeof(bytes)) == sizeof(bytes);
}

bool writeUInt32(QIODevice& file, quint32 value)
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
    QSaveFile file(outputFilePath);
    if (!file.open(QIODevice::WriteOnly)) {
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
            file.cancelWriting();
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
            file.cancelWriting();
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

    return success && file.commit();
}

} // namespace

// OUTPUT: {runDir}/{basename}_tracks.xlsx
// FORMAT: XLSX (Office Open XML, written as a ZIP with hand-built XML)
// DATA:   Four sheets:
//   "Tracks"          — one row per (worm, frame): WormID, SourceItemID, Frame,
//                        PositionX/Y (4 dp), RoiX/Y/Width/Height, Quality.
//   "start-end coords"— StartPoint and EndPoint items: PointType, SourceItemID,
//                        FrameSelected, PositionX/Y (4 dp).
//   "Parameters"      — Capture Rate (fps) and Pixel Size (pixels/µm).
//   "Centerlines"     — one row per (worm, frame): WormID, SourceItemID, Frame,
//                        TipToTipDistance, Point1..10 X/Y (10 resampled centerline points).
// TRIGGER: Written once, after centerline processing (or when centerlines are disabled).
bool TrackingExportWorker::outputTracksToWorkbook(const Tracking::AllWormTracks& tracks, const QString& outputFilePath) const {
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
        const Tracking::Track& sortedTrackPoints = it->second;

        for (const Tracking::TrackPoint& point : sortedTrackPoints) {
            trackRows.append(WorkbookRow{
                numberCell(QString::number(exportWormId)),
                numberCell(QString::number(sourceItemId)),
                numberCell(QString::number(point.frameNumber)),
                numberCell(QString::number(static_cast<double>(point.position.x), 'f', 4)),
                numberCell(QString::number(static_cast<double>(point.position.y), 'f', 4)),
                numberCell(QString::number(point.searchWindow.x(), 'f', 2)),
                numberCell(QString::number(point.searchWindow.y(), 'f', 2)),
                numberCell(QString::number(point.searchWindow.width(), 'f', 2)),
                numberCell(QString::number(point.searchWindow.height(), 'f', 2)),
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

    const TableItems::AnnotationItem* startPoint = nullptr;
    const TableItems::AnnotationItem* endPoint = nullptr;
    {
        const QList<TableItems::AnnotationItem>& items = m_snapshot.items;
        for (const TableItems::AnnotationItem& item : items) {
            if (item.type == TableItems::ItemType::StartPoint) {
                startPoint = &item;
            } else if (item.type == TableItems::ItemType::EndPoint) {
                endPoint = &item;
            }
        }
    }

    const auto appendPointRow = [&startEndRows](const TableItems::AnnotationItem* item) {
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
        numberCell(QString::number(m_snapshot.fps, 'f', 6)),
        stringCell("fps")
    });
    parameterRows.append(WorkbookRow{
        stringCell("Pixel Size"),
        numberCell(QString::number(m_snapshot.umPerPixel, 'f', 6)),
        stringCell("um/pixel")
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

    {
        for (auto it = tracks.begin(); it != tracks.end(); ++it) {
            const int sourceItemId = it->first;
            const int exportWormId = sourceToExportId.value(sourceItemId);
            const Tracking::Track& sortedTrackPoints = it->second;

            for (const Tracking::TrackPoint& point : sortedTrackPoints) {
                QList<QPointF> centerlinePoints;
                if (point.quality != Tracking::TrackPointQuality::Lost) {
                    if (const auto* blob = m_snapshot.findBlob(point.frameNumber, sourceItemId)) {
                        centerlinePoints = Centerline::resampleCenterlinePoints(blob->centerline.points, 10);
                    }

                }

                WorkbookRow centerlineRow{
                    numberCell(QString::number(exportWormId)),
                    numberCell(QString::number(sourceItemId)),
                    numberCell(QString::number(point.frameNumber))
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
bool TrackingExportWorker::saveWormSummaryJson(const QString& directoryPath) const
{
    if (directoryPath.isEmpty()) return false;

    const Tracking::AllWormTracks& tracks = m_snapshot.worms.tracks;
    const double umPerPixel = m_snapshot.umPerPixel;

    QJsonObject wormsObj;

    QList<int> sortedIds;
    for (const auto& entry : tracks) sortedIds.append(entry.first);

    for (int wormId : sortedIds) {
        const auto& rawPoints = tracks.at(wormId);
        if (rawPoints.empty()) continue;

        const Tracking::Track& pts = rawPoints;
        const WormStatistics& stats = m_statistics.constFind(wormId).value();
        const int nSingle = stats.single, nSplit = stats.split;
        const int nMerged = stats.merged, nLost = stats.lost;
        const double totalDist = stats.totalDistance;
        const double netDisp = stats.netDisplacement;

        // Speed: total distance / tracked time (non-lost frames, using FPS)
        int nTrackedFrames = nSingle + nSplit + nMerged;
        double trackedTimeSec = (m_snapshot.fps > 0 && nTrackedFrames > 1)
                                    ? (nTrackedFrames - 1) / m_snapshot.fps : 0.0;
        double meanSpeed = (trackedTimeSec > 0) ? totalDist / trackedTimeSec : 0.0;

        const double meanArea = stats.meanArea;
        const double meanBodyLen = stats.meanBodyLength;
        const double meanAspectRatio = stats.meanAspectRatio;

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
    root["fps"]          = m_snapshot.fps;
    root["um_per_pixel"] = umPerPixel;
    root["frame_width"]  = m_snapshot.frameSize.width;
    root["frame_height"] = m_snapshot.frameSize.height;
    root["worms"]        = wormsObj;

    const QString path = QDir(directoryPath).filePath("worm_summary.json");
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        qWarning() << "TrackingManager: could not write worm_summary.json to" << path;
        return false;
    }
    const QByteArray bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    return f.write(bytes) == bytes.size() && f.commit();
}

// OUTPUT: caller-specified path (typically {runDir}/{basename}_summary.txt)
// FORMAT: UTF-8 plain text (human-readable report)
// DATA:   High-level tracking run report: video metadata (fps, frame range, worm count),
//           per-worm statistics (track frame range, quality breakdown by category,
//           centerline topology, merge event count, head/tail swap count).
// TRIGGER: Written once at tracking finalization.
bool TrackingExportWorker::exportProcessingSummary(const QString& outputPath) const
{
    if (outputPath.isEmpty()) return false;

    QSaveFile file(outputPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "ProcessingSummary: could not write" << outputPath;
        return false;
    }
    QTextStream out(&file);
    out.setEncoding(QStringConverter::Utf8);

    const Tracking::AllWormTracks& tracks = m_snapshot.worms.tracks;

    // Overall frame range
    int minFrame = INT_MAX, maxFrame = INT_MIN;
    for (const auto& entry : tracks) {
        for (const auto& tp : entry.second) {
            minFrame = std::min(minFrame, tp.frameNumber);
            maxFrame = std::max(maxFrame, tp.frameNumber);
        }
    }

    // Sorted worm IDs
    QList<int> sortedWormIds;
    for (const auto& entry : tracks) sortedWormIds.append(entry.first);

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
    out << QString("Video:            %1\n").arg(QFileInfo(m_snapshot.worms.videoPath).fileName());
    out << QString("FPS:              %1\n").arg(m_snapshot.fps, 0, 'f', 2);
    if (minFrame <= maxFrame) {
        out << QString("Frame range:      %1 \xe2\x80\x93 %2  (%3 frames)\n")
                   .arg(minFrame).arg(maxFrame).arg(maxFrame - minFrame + 1);
    }
    out << QString("Worms tracked:    %1\n").arg(sortedWormIds.size());
    out << "\n";
    out << "Settings\n";
    out << QString("  Centerline computation:  %1\n")
               .arg(m_snapshot.centerlineEnabled ? "enabled" : "disabled");
    out << QString("  Skip merged frames:      %1\n")
               .arg(m_snapshot.skipMergedFrames ? "yes" : "no");
    out << QString("  Max reversal fraction:   %1\n")
               .arg(m_snapshot.maxReversalFraction, 0, 'f', 2);
    out << "\n";

    // ── Per-worm sections ────────────────────────────────────────────────────
    for (int wormId : sortedWormIds) {
        const auto& rawPoints = tracks.at(wormId);
        if (rawPoints.empty()) continue;

        const Tracking::Track& pts = rawPoints;
        const int total = static_cast<int>(pts.size());
        const int wFirst = pts.front().frameNumber, wLast = pts.back().frameNumber;
        const WormStatistics& stats = m_statistics.constFind(wormId).value();
        const int nSingle = stats.single, nSplit = stats.split;
        const int nMerged = stats.merged, nLost = stats.lost;
        const int nCLcomputed = stats.centerlineComputed, nCLskipped = stats.centerlineSkipped;
        const int nCleanTopo = stats.cleanTopology, nSCTopo = stats.selfCrossedTopology;
        const int mergeRunCount = stats.mergeRuns, mergeRunMaxLen = stats.longestMergeRun;

        // Head/tail swap counts
        const int motionSwaps = m_snapshot.motionSwaps.value(wormId).size();
        const int geoSwaps = m_snapshot.geometrySwaps.value(wormId).size();
        const int netSwaps = m_snapshot.headTailSwaps.value(wormId).size();

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
                       .arg(avgLen / m_snapshot.fps, 0, 'f', 1);
            out << QString("  Longest run:     %1 frames  (%2 s)\n")
                       .arg(lpad(QString::number(mergeRunMaxLen), 6))
                       .arg(mergeRunMaxLen / m_snapshot.fps, 0, 'f', 1);
        }
        out << "\n";

        // Head/tail refinement
        out << "Head/tail refinement\n";
        out << QString("  Direction-based swapped frames:  %1\n").arg(lpad(QString::number(motionSwaps), 6));
        out << QString("  Geometry-based swapped frames:   %1\n").arg(lpad(QString::number(geoSwaps), 6));
        out << QString("  Net swapped frames (XOR):        %1\n").arg(lpad(QString::number(netSwaps), 6));
        out << QString("  Frames flagged for review:       %1\n")
                   .arg(lpad(QString::number(m_snapshot.centerlineReviews.value(wormId).size()), 6));
        out << "\n";
    }

    out << rule << "\n";
    out << "End of Processing Summary\n";
    out << rule << "\n";
    out.flush();
    return out.status() == QTextStream::Ok && file.commit();
}

// OUTPUT: {runDir}/{basename}_headtail_swaps.xlsx
// FORMAT: XLSX (Office Open XML, written as a ZIP with hand-built XML)
// DATA:   One sheet with a row per frame and one column per worm;
//           cells contain "SWAP" on frames where the post-pass head/tail naming
//           reversed the continuity pass's labels, "REVIEW" on frames flagged for
//           human assessment, "SWAP REVIEW" for both, blank otherwise.
// TRIGGER: Written at tracking finalization when any swap or review flag exists.
bool TrackingExportWorker::exportHeadTailSwapXlsx(
    const QMap<int, QList<int>>& swapData,
    const QMap<int, QList<int>>& reviewData,
    const QString& outputPath) const
{
    if (outputPath.isEmpty()) return false;

    // Collect the full frame range from all tracks.
    int minFrame = INT_MAX, maxFrame = INT_MIN;
    const Tracking::AllWormTracks& tracks = m_snapshot.worms.tracks;
    for (const auto& entry : tracks) {
        for (const auto& tp : entry.second) {
            minFrame = std::min(minFrame, tp.frameNumber);
            maxFrame = std::max(maxFrame, tp.frameNumber);
        }
    }
    if (minFrame > maxFrame) return false;

    // Sorted worm IDs (columns).
    QSet<int> wormIdSet;
    for (int wid : swapData.keys()) wormIdSet.insert(wid);
    for (int wid : reviewData.keys()) wormIdSet.insert(wid);
    QList<int> wormIds(wormIdSet.begin(), wormIdSet.end());
    std::sort(wormIds.begin(), wormIds.end());

    // Build fast-lookup sets of swapped and flagged frames per worm.
    QMap<int, QSet<int>> swapSets, reviewSets;
    for (int wid : wormIds) {
        for (int f : swapData.value(wid)) swapSets[wid].insert(f);
        for (int f : reviewData.value(wid)) reviewSets[wid].insert(f);
    }

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
        for (int wid : wormIds) {
            QStringList marks;
            if (swapSets.value(wid).contains(f)) marks << QStringLiteral("SWAP");
            if (reviewSets.value(wid).contains(f)) marks << QStringLiteral("REVIEW");
            row.append(stringCell(marks.join(QLatin1Char(' '))));
        }
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
    return writeStoredZip(outputPath, entries);
}

QJsonObject TrackingExportSnapshot::mergeStateToJson() const {
    QJsonObject ms;
    ms["nextPhysicalBlobId"] = nextSharedBlobId;

    // wormToSharedBlobIdMap
    QJsonObject wormToPhysObj;
    for (auto it = wormToSharedBlobId.constBegin();
         it != wormToSharedBlobId.constEnd(); ++it)
        wormToPhysObj[QString::number(it.key())] = it.value();
    ms["wormToPhysicalBlobIdMap"] = wormToPhysObj;

    // frameMergeRecords
    QJsonObject fmObj;
    for (auto fit = mergeRecords.constBegin();
         fit != mergeRecords.constEnd(); ++fit) {
        QJsonArray blobsArr;
        for (const SharedBlob& pb : fit.value()) {
            QJsonObject pbObj;
            pbObj["uniqueId"]    = pb.uniqueId;
            pbObj["frameNumber"] = pb.frameNumber;
            pbObj["currentArea"] = pb.currentArea;

            pbObj["currentCentroid"] = JsonGeometry::toJson(pb.currentCentroid);
            pbObj["currentBoundingBox"] = JsonGeometry::toJson(pb.currentBoundingBox);
            pbObj["contourPoints"] = JsonGeometry::contourToJson(pb.contourPoints);
            pbObj["holeContourPoints"] = JsonGeometry::holeContoursToJson(pb.holeContourPoints);

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
    for (auto sfit = splitResolutions.constBegin();
         sfit != splitResolutions.constEnd(); ++sfit) {
        QJsonObject wormMapObj;
        for (auto wit = sfit.value().constBegin();
             wit != sfit.value().constEnd(); ++wit)
            wormMapObj[QString::number(wit.key())] = Tracking::detectedBlobToJson(wit.value());
        splitObj[QString::number(sfit.key())] = wormMapObj;
    }
    ms["splitResolutionMap"] = splitObj;

    return ms;
}

