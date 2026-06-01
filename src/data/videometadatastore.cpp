#include "videometadatastore.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>

static QJsonObject readRoot(const QString& path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return {};
    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return {};
    return doc.object();
}

static bool writeRoot(const QString& path, const QJsonObject& root)
{
    QFileInfo(path).dir().mkpath(".");   // ensure directory exists
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

QString VideoMetadataStore::metadataPath(const QString& dataDir,
                                          const QString& videoBaseName)
{
    return QDir(dataDir).filePath(videoBaseName + "_metadata.json");
}

// ---------------------------------------------------------------------------
// Internal helper: unit string → conversion factor to µm
// ---------------------------------------------------------------------------
static double unitToUmFactor(const QString& unit)
{
    if (unit == "mm")   return 1000.0;
    if (unit == "cm")   return 10000.0;
    if (unit == "inch") return 25400.0;
    if (unit == "µm")   return 1.0;
    return 0.0; // unknown
}

// OUTPUT: {dataDir}/{videoBaseName}_metadata.json
// FORMAT: JSON (indented); merges into the existing file so other sections are preserved.
// DATA:   Scale calibration: pixelsPerUnit, unit, physicalValue, pixelLength, timestamp;
//           also writes the derived umPerPixel (µm/pixel) for fast reloading.
// TRIGGER: User saves a scale calibration in the measurement UI.
bool VideoMetadataStore::saveScale(const QString& dataDir,
                                    const QString& videoBaseName,
                                    const ScaleCalibration& cal)
{
    const QString path = metadataPath(dataDir, videoBaseName);

    // Read existing root so we don't clobber other sections.
    QJsonObject root = readRoot(path);
    root["version"] = 1;

    QJsonObject scale;
    scale["pixelsPerUnit"]  = cal.pixelsPerUnit;
    scale["unit"]           = cal.unit;
    scale["physicalValue"]  = cal.physicalValue;
    scale["pixelLength"]    = cal.pixelLength;
    scale["timestamp"]      = cal.timestamp.isValid()
                                ? cal.timestamp.toString(Qt::ISODate)
                                : QDateTime::currentDateTime().toString(Qt::ISODate);
    root["scaleCalibration"] = scale;

    // Also write the canonical spatial resolution (µm/pixel) for easy reloading.
    const double factor = unitToUmFactor(cal.unit);
    if (factor > 0 && cal.pixelsPerUnit > 0)
        root["umPerPixel"] = factor / cal.pixelsPerUnit;

    if (!writeRoot(path, root)) {
        qWarning() << "[VideoMetadataStore] Could not write" << path;
        return false;
    }
    qDebug() << "[VideoMetadataStore] Saved scale to" << path;
    return true;
}

// OUTPUT: {dataDir}/{videoBaseName}_metadata.json  (same file as saveScale)
// FORMAT: JSON (indented); merges into the existing file.
// DATA:   Single field: umPerPixel — spatial resolution in micrometers per pixel.
//           Used when the calibration value is updated programmatically without a full
//           ScaleCalibration struct (e.g., unit conversion recalculation).
// TRIGGER: Programmatic update of spatial resolution independent of full scale save.
bool VideoMetadataStore::saveUmPerPixel(const QString& dataDir,
                                         const QString& videoBaseName,
                                         double umPerPixel)
{
    const QString path = metadataPath(dataDir, videoBaseName);
    QJsonObject root = readRoot(path);
    root["version"]    = 1;
    root["umPerPixel"] = umPerPixel;
    if (!writeRoot(path, root)) {
        qWarning() << "[VideoMetadataStore] Could not write" << path;
        return false;
    }
    return true;
}

bool VideoMetadataStore::loadUmPerPixel(const QString& dataDir,
                                         const QString& videoBaseName,
                                         double& umPerPixel)
{
    const QString path = metadataPath(dataDir, videoBaseName);
    QJsonObject root = readRoot(path);
    if (root.isEmpty()) return false;
    // Current field name
    if (root.contains("umPerPixel")) {
        double v = root.value("umPerPixel").toDouble(0.0);
        if (v > 0) { umPerPixel = v; return true; }
    }
    // Legacy field (pixels/µm, written before the unit flip) — invert on load.
    if (root.contains("pixelSizeUm")) {
        double ppu = root.value("pixelSizeUm").toDouble(0.0);
        if (ppu > 0) { umPerPixel = 1.0 / ppu; return true; }
    }
    return false;
}

bool VideoMetadataStore::saveFps(const QString& dataDir,
                                  const QString& videoBaseName,
                                  double fps)
{
    if (fps <= 0) return false;
    const QString path = metadataPath(dataDir, videoBaseName);
    QJsonObject root = readRoot(path);
    root["version"] = 1;
    root["fps"] = fps;
    if (!writeRoot(path, root)) {
        qWarning() << "[VideoMetadataStore] Could not write fps to" << path;
        return false;
    }
    return true;
}

bool VideoMetadataStore::loadFps(const QString& dataDir,
                                  const QString& videoBaseName,
                                  double& fps)
{
    const QString path = metadataPath(dataDir, videoBaseName);
    QJsonObject root = readRoot(path);
    if (root.isEmpty()) return false;
    if (!root.contains("fps")) return false;
    double v = root.value("fps").toDouble(0.0);
    if (v <= 0) return false;
    fps = v;
    return true;
}

bool VideoMetadataStore::loadScale(const QString& dataDir,
                                    const QString& videoBaseName,
                                    ScaleCalibration& cal)
{
    const QString path = metadataPath(dataDir, videoBaseName);
    QJsonObject root = readRoot(path);
    if (root.isEmpty()) return false;

    QJsonObject scale = root.value("scaleCalibration").toObject();
    if (scale.isEmpty()) return false;

    double ppu = scale.value("pixelsPerUnit").toDouble(0.0);
    if (ppu <= 0) return false;

    cal.pixelsPerUnit = ppu;
    cal.unit          = scale.value("unit").toString();
    cal.physicalValue = scale.value("physicalValue").toDouble(1.0);
    cal.pixelLength   = scale.value("pixelLength").toDouble(0.0);
    cal.timestamp     = QDateTime::fromString(
                            scale.value("timestamp").toString(), Qt::ISODate);
    return cal.isValid();
}
