#include "pluginloader.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSet>
#include <QDebug>

PlotPluginSpec PluginLoader::load(const QString& filePath)
{
    PlotPluginSpec spec;
    spec.filePath = filePath;

    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly)) {
        spec.errors << QString("Cannot open file: %1").arg(filePath);
        return spec;
    }

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError) {
        spec.errors << QString("JSON parse error: %1").arg(err.errorString());
        return spec;
    }
    if (!doc.isObject()) {
        spec.errors << "Root element must be a JSON object";
        return spec;
    }

    const QJsonObject root = doc.object();

    spec.version     = root.value("version").toInt(1);
    spec.name        = root.value("name").toString().trimmed();
    spec.description = root.value("description").toString().trimmed();
    spec.formula     = root.value("formula").toString().trimmed();

    if (spec.name.isEmpty())    spec.errors << "Missing required field: name";
    if (spec.formula.isEmpty()) spec.errors << "Missing required field: formula";

    // aggregate
    const QString agg = root.value("aggregate").toString("per_worm").toLower();
    if      (agg == "per_worm")  spec.aggregate = PlotPluginSpec::Aggregate::PerWorm;
    else if (agg == "per_frame") spec.aggregate = PlotPluginSpec::Aggregate::PerFrame;
    else if (agg == "spatial")   spec.aggregate = PlotPluginSpec::Aggregate::Spatial;
    else spec.errors << QString("Unknown aggregate mode: %1").arg(agg);

    // bindings
    if (root.contains("bindings") && root["bindings"].isObject()) {
        const QJsonObject bObj = root["bindings"].toObject();
        for (auto it = bObj.constBegin(); it != bObj.constEnd(); ++it)
            spec.bindings[it.key()] = it.value().toString().trimmed();
    }

    // filter
    spec.filter = root.value("filter").toString().trimmed();

    // reduce
    spec.reduce = root.value("reduce").toString("mean").toLower();
    static const QSet<QString> validReduce{
        "mean","median","sum","count","min","max","std","last"
    };
    if (!validReduce.contains(spec.reduce))
        spec.errors << QString("Unknown reduce function: %1").arg(spec.reduce);

    // plot
    if (root.contains("plot") && root["plot"].isObject()) {
        const QJsonObject plot = root["plot"].toObject();

        const QString typeStr = plot.value("type").toString("box").toLower();
        if      (typeStr == "box")     spec.plotType = PlotPluginSpec::PlotType::Box;
        else if (typeStr == "bar")     spec.plotType = PlotPluginSpec::PlotType::Bar;
        else if (typeStr == "line")    spec.plotType = PlotPluginSpec::PlotType::Line;
        else if (typeStr == "scatter") spec.plotType = PlotPluginSpec::PlotType::Scatter;
        else spec.errors << QString("Unknown plot type: %1").arg(typeStr);

        spec.yLabel   = plot.value("y_label").toString();
        spec.yLabelUm = plot.value("y_label_um").toString();
        spec.xLabel   = plot.value("x_label").toString();
    }

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

    for (const QFileInfo& fi : dir.entryInfoList({"*.json"}, QDir::Files, QDir::Name)) {
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
