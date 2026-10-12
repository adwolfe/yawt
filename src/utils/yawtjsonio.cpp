#include "yawtjsonio.h"

#include <QByteArray>
#include <QFile>
#include <QSaveFile>
#include <QJsonParseError>

namespace {

const QByteArray kCompressedJsonMagic("YAWT_COMPRESSED_JSON_V1\n");

QByteArray decodeJsonBytes(const QByteArray& bytes, QString* error)
{
    if (!bytes.startsWith(kCompressedJsonMagic)) {
        return bytes;
    }

    const QByteArray compressed = bytes.mid(kCompressedJsonMagic.size());
    const QByteArray decoded = qUncompress(compressed);
    if (decoded.isEmpty() && !compressed.isEmpty()) {
        if (error) {
            *error = QStringLiteral("Could not decompress compressed JSON payload");
        }
        return {};
    }
    return decoded;
}

} // namespace

QJsonDocument YawtJsonIO::readJsonDocument(const QString& filePath,
                                           QJsonParseError* parseError,
                                           QString* ioError)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (ioError) {
            *ioError = file.errorString();
        }
        return {};
    }

    QString decodeError;
    const QByteArray decoded = decodeJsonBytes(file.readAll(), &decodeError);
    if (!decodeError.isEmpty()) {
        if (ioError) {
            *ioError = decodeError;
        }
        return {};
    }

    QJsonParseError localParseError;
    QJsonDocument doc = QJsonDocument::fromJson(decoded, parseError ? parseError : &localParseError);
    return doc;
}

bool YawtJsonIO::writeCompressedJsonDocument(const QString& filePath,
                                             const QJsonDocument& document,
                                             QString* error)
{
    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) {
            *error = file.errorString();
        }
        return false;
    }

    const QByteArray json = document.toJson(QJsonDocument::Compact);
    // Level 6 preserves nearly all of level 9's size savings without its long
    // compression pause; existing qUncompress readers remain compatible.
    const QByteArray compressed = qCompress(json, 6);
    if (file.write(kCompressedJsonMagic) != kCompressedJsonMagic.size()
        || file.write(compressed) != compressed.size()) {
        if (error) {
            *error = file.errorString();
        }
        return false;
    }

    if (!file.commit()) {
        if (error) *error = file.errorString();
        return false;
    }
    return true;
}
