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
    root["scale"] = scale;

    if (!writeRoot(path, root)) {
        qWarning() << "[VideoMetadataStore] Could not write" << path;
        return false;
    }
    qDebug() << "[VideoMetadataStore] Saved scale to" << path;
    return true;
}

bool VideoMetadataStore::loadScale(const QString& dataDir,
                                    const QString& videoBaseName,
                                    ScaleCalibration& cal)
{
    const QString path = metadataPath(dataDir, videoBaseName);
    QJsonObject root = readRoot(path);
    if (root.isEmpty()) return false;

    QJsonObject scale = root.value("scale").toObject();
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
