#include "pluginloader.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSet>
#include <QDebug>

namespace {

struct PluginFields {
    QHash<QString, QString> root;
    QHash<QString, QHash<QString, QString>> maps;
    QStringList errors;
};

static QString jsonValueToString(const QJsonValue& value)
{
    if (value.isString()) return value.toString().trimmed();
    if (value.isDouble()) return QString::number(value.toDouble(), 'g', 17);
    if (value.isBool()) return value.toBool() ? "true" : "false";
    return {};
}

static PluginFields fieldsFromJson(const QByteArray& data)
{
    PluginFields fields;

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(data, &err);
    if (err.error != QJsonParseError::NoError) {
        fields.errors << QString("JSON parse error: %1").arg(err.errorString());
        return fields;
    }
    if (!doc.isObject()) {
        fields.errors << "Root element must be a JSON object";
        return fields;
    }

    const QJsonObject root = doc.object();
    for (auto it = root.constBegin(); it != root.constEnd(); ++it) {
        if (it.value().isObject()) continue;
        fields.root[it.key()] = jsonValueToString(it.value());
    }

    for (const QString& mapKey : {"bindings", "plot"}) {
        const QJsonValue value = root.value(mapKey);
        if (!value.isObject()) continue;

        QHash<QString, QString> map;
        const QJsonObject obj = value.toObject();
        for (auto it = obj.constBegin(); it != obj.constEnd(); ++it)
            map[it.key()] = jsonValueToString(it.value());
        fields.maps[mapKey] = std::move(map);
    }

    return fields;
}

static QString stripYamlComment(const QString& line)
{
    bool inSingle = false;
    bool inDouble = false;
    bool escaped = false;

    for (int i = 0; i < line.length(); ++i) {
        const QChar c = line[i];
        if (inDouble && escaped) {
            escaped = false;
            continue;
        }
        if (inDouble && c == '\\') {
            escaped = true;
            continue;
        }
        if (!inDouble && c == '\'') {
            inSingle = !inSingle;
            continue;
        }
        if (!inSingle && c == '"') {
            inDouble = !inDouble;
            continue;
        }
        if (!inSingle && !inDouble && c == '#') {
            return line.left(i);
        }
    }
    return line;
}

static int firstYamlColon(const QString& line)
{
    bool inSingle = false;
    bool inDouble = false;
    bool escaped = false;

    for (int i = 0; i < line.length(); ++i) {
        const QChar c = line[i];
        if (inDouble && escaped) {
            escaped = false;
            continue;
        }
        if (inDouble && c == '\\') {
            escaped = true;
            continue;
        }
        if (!inDouble && c == '\'') {
            inSingle = !inSingle;
            continue;
        }
        if (!inSingle && c == '"') {
            inDouble = !inDouble;
            continue;
        }
        if (!inSingle && !inDouble && c == ':') {
            return i;
        }
    }
    return -1;
}

static QString unquoteYamlValue(const QString& raw)
{
    const QString value = raw.trimmed();
    if (value.length() < 2) return value;

    if (value.startsWith('"') && value.endsWith('"')) {
        QString out;
        bool escaped = false;
        for (int i = 1; i < value.length() - 1; ++i) {
            const QChar c = value[i];
            if (!escaped) {
                if (c == '\\') {
                    escaped = true;
                } else {
                    out.append(c);
                }
                continue;
            }

            if (c == 'n') out.append('\n');
            else if (c == 't') out.append('\t');
            else out.append(c);
            escaped = false;
        }
        return out.trimmed();
    }

    if (value.startsWith('\'') && value.endsWith('\'')) {
        return QString(value.mid(1, value.length() - 2)).replace("''", "'").trimmed();
    }

    return value;
}

static PluginFields fieldsFromYaml(const QByteArray& data)
{
    PluginFields fields;
    QString currentMap;
    const QString text = QString::fromUtf8(data);
    const QStringList lines = text.split('\n');

    for (int lineNo = 0; lineNo < lines.size(); ++lineNo) {
        const QString uncommented = stripYamlComment(lines[lineNo]).trimmed();
        if (uncommented.isEmpty()) continue;

        const QString original = stripYamlComment(lines[lineNo]);
        int indent = 0;
        while (indent < original.length() && original[indent].isSpace()) ++indent;

        const QString line = original.mid(indent).trimmed();
        const int colon = firstYamlColon(line);
        if (colon <= 0) {
            fields.errors << QString("YAML parse error on line %1: expected key: value").arg(lineNo + 1);
            continue;
        }

        const QString key = line.left(colon).trimmed();
        const QString value = line.mid(colon + 1).trimmed();
        if (key.isEmpty()) {
            fields.errors << QString("YAML parse error on line %1: empty key").arg(lineNo + 1);
            continue;
        }

        if (indent == 0) {
            if (value.isEmpty()) {
                currentMap = key;
                fields.maps[currentMap];
            } else {
                currentMap.clear();
                fields.root[key] = unquoteYamlValue(value);
            }
        } else {
            if (currentMap.isEmpty()) {
                fields.errors << QString("YAML parse error on line %1: nested key without a parent map").arg(lineNo + 1);
                continue;
            }
            if (value.isEmpty()) {
                fields.errors << QString("YAML parse error on line %1: nested maps are not supported").arg(lineNo + 1);
                continue;
            }
            fields.maps[currentMap][key] = unquoteYamlValue(value);
        }
    }

    return fields;
}

static void populateSpecFromFields(PlotPluginSpec& spec, const PluginFields& fields)
{
    spec.errors << fields.errors;
    if (!fields.errors.isEmpty()) return;

    bool ok = false;
    const int parsedVersion = fields.root.value("version", "1").toInt(&ok);
    spec.version = ok ? parsedVersion : 1;
    spec.name = fields.root.value("name").trimmed();
    spec.description = fields.root.value("description").trimmed();
    spec.formula = fields.root.value("formula").trimmed();

    if (spec.name.isEmpty())    spec.errors << "Missing required field: name";
    if (spec.formula.isEmpty()) spec.errors << "Missing required field: formula";

    const QString agg = fields.root.value("aggregate", "per_worm").toLower();
    if      (agg == "per_worm")  spec.aggregate = PlotPluginSpec::Aggregate::PerWorm;
    else if (agg == "per_frame") spec.aggregate = PlotPluginSpec::Aggregate::PerFrame;
    else if (agg == "spatial")   spec.aggregate = PlotPluginSpec::Aggregate::Spatial;
    else spec.errors << QString("Unknown aggregate mode: %1").arg(agg);

    const QHash<QString, QString> bindings = fields.maps.value("bindings");
    for (auto it = bindings.constBegin(); it != bindings.constEnd(); ++it)
        spec.bindings[it.key()] = it.value().trimmed();

    spec.filter = fields.root.value("filter").trimmed();

    spec.reduce = fields.root.value("reduce", "mean").toLower();
    static const QSet<QString> validReduce{
        "mean","median","sum","count","min","max","std","last"
    };
    if (!validReduce.contains(spec.reduce))
        spec.errors << QString("Unknown reduce function: %1").arg(spec.reduce);

    const QHash<QString, QString> plot = fields.maps.value("plot");
    const QString typeStr = plot.value("type", "box").toLower();
    if      (typeStr == "box")     spec.plotType = PlotPluginSpec::PlotType::Box;
    else if (typeStr == "bar")     spec.plotType = PlotPluginSpec::PlotType::Bar;
    else if (typeStr == "line")    spec.plotType = PlotPluginSpec::PlotType::Line;
    else if (typeStr == "scatter") spec.plotType = PlotPluginSpec::PlotType::Scatter;
    else spec.errors << QString("Unknown plot type: %1").arg(typeStr);

    spec.yLabel   = plot.value("y_label");
    spec.yLabelUm = plot.value("y_label_um");
    spec.xLabel   = plot.value("x_label");
}

} // namespace

PlotPluginSpec PluginLoader::load(const QString& filePath)
{
    PlotPluginSpec spec;
    spec.filePath = filePath;

    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly)) {
        spec.errors << QString("Cannot open file: %1").arg(filePath);
        return spec;
    }

    const QByteArray data = f.readAll();
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    const PluginFields fields = (suffix == "yaml" || suffix == "yml")
        ? fieldsFromYaml(data)
        : fieldsFromJson(data);
    populateSpecFromFields(spec, fields);

    spec.isValid = spec.errors.isEmpty();
    if (!spec.isValid)
        qWarning() << "[PluginLoader] Errors in" << QFileInfo(filePath).fileName() << spec.errors;

    return spec;
}

QList<PlotPluginSpec> PluginLoader::loadDirectory(const QString& dirPath)
{
    QList<PlotPluginSpec> result;
    QDir dir(dirPath);
    if (!dir.exists()) return result;

    for (const QFileInfo& fi : dir.entryInfoList({"*.json", "*.yaml", "*.yml"}, QDir::Files, QDir::Name)) {
        result.append(load(fi.absoluteFilePath()));
    }
    return result;
}

QList<PlotPluginSpec> PluginLoader::loadAll(const QStringList& searchDirs)
{
    QList<PlotPluginSpec> result;
    QSet<QString> seenNames;

    for (const QString& dir : searchDirs) {
        for (const PlotPluginSpec& spec : loadDirectory(dir)) {
            if (spec.isValid && seenNames.contains(spec.name)) {
                qDebug() << "[PluginLoader] Skipping duplicate plugin:" << spec.name
                         << "from" << spec.filePath;
                continue;
            }
            if (spec.isValid) seenNames.insert(spec.name);
            result.append(spec);
        }
    }
    return result;
}
