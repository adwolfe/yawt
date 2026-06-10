#pragma once

#include <QJsonDocument>
#include <QString>

class YawtJsonIO
{
public:
    YawtJsonIO() = delete;

    static QJsonDocument readJsonDocument(const QString& filePath,
                                          QJsonParseError* parseError = nullptr,
                                          QString* ioError = nullptr);
    static bool writeCompressedJsonDocument(const QString& filePath,
                                            const QJsonDocument& document,
                                            QString* error = nullptr);
};
