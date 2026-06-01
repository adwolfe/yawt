#include "pluginengine.h"
#include "expreval.h"

#include <QColor>
#include <algorithm>
#include <cmath>
#include <numeric>

// ── Standard vocabulary constants ─────────────────────────────────────────
// quality values match TrackPointQuality enum order
static constexpr double Q_SINGLE = 0.0;
static constexpr double Q_MERGED = 1.0;
static constexpr double Q_SPLIT  = 2.0;
static constexpr double Q_LOST   = 3.0;

// ── Variable map builder ──────────────────────────────────────────────────

QMap<QString, double> PluginEngine::buildVars(
    const AnalysisSessionModel::AnalysisWormEntry& worm,
    int pointIdx,
    const PluginRoiPoints& roi,
    double umPerPixel,
    double fps)
{
    const Tracking::WormTrackPoint& p = worm.points[pointIdx];
    const double x = static_cast<double>(p.position.x);
    const double y = static_cast<double>(p.position.y);
    const double t = (fps > 0) ? p.frameNumberOriginal / fps : 0.0;
    const double um = umPerPixel;

    QMap<QString, double> v;

    // Position
    v["x"] = x;   v["y"] = y;
    v["x_um"] = x * um;  v["y_um"] = y * um;

    // Time
    v["frame"] = p.frameNumberOriginal;
    v["t"]     = t;

    // Track quality (enum value as double)
    v["quality"] = static_cast<double>(static_cast<int>(p.quality));
    // Named quality constants for readable filter expressions
    v["Single"] = Q_SINGLE;
    v["Merged"] = Q_MERGED;
    v["Split"]  = Q_SPLIT;
    v["Lost"]   = Q_LOST;

    // Morphology
    v["area"]          = p.area;
    v["area_um2"]      = p.area * um * um;
    v["body_length"]   = p.bodyLength;
    v["body_length_um"]= p.bodyLength * um;
    v["aspect_ratio"]  = p.aspectRatio;

    // Head/tail tips
    if (p.hasTips) {
        v["xhead"] = p.headTip.x;  v["yhead"] = p.headTip.y;
        v["xtail"] = p.tailTip.x;  v["ytail"] = p.tailTip.y;
        v["xhead_um"] = p.headTip.x * um;  v["yhead_um"] = p.headTip.y * um;
        v["xtail_um"] = p.tailTip.x * um;  v["ytail_um"] = p.tailTip.y * um;
    } else {
        v["xhead"] = 0; v["yhead"] = 0; v["xtail"] = 0; v["ytail"] = 0;
        v["xhead_um"] = 0; v["yhead_um"] = 0; v["xtail_um"] = 0; v["ytail_um"] = 0;
    }

    // ROI reference points
    if (roi.hasStart) {
        v["start_x"] = roi.startX;  v["start_y"] = roi.startY;
        double dx = x - roi.startX, dy = y - roi.startY;
        v["dist_to_start"] = std::sqrt(dx*dx + dy*dy);
    }
    if (roi.hasEnd) {
        v["end_x"] = roi.endX;  v["end_y"] = roi.endY;
        double dx = x - roi.endX, dy = y - roi.endY;
        v["dist_to_end"] = std::sqrt(dx*dx + dy*dy);
    }
    if (roi.hasCenter) {
        v["center_x"] = roi.centerX;  v["center_y"] = roi.centerY;
        double dx = x - roi.centerX, dy = y - roi.centerY;
        v["dist_to_center"] = std::sqrt(dx*dx + dy*dy);
    }

    // fps available as a constant in expressions
    v["fps"] = fps;

    return v;
}

// ── Binding resolver ──────────────────────────────────────────────────────

// Recognises "diff(varName)" and computes current - previous value.
// Returns false and sets error on expression evaluation failure.
bool PluginEngine::applyBindings(QMap<QString, double>& vars,
                                 const QMap<QString, double>& prevVars,
                                 const QMap<QString, QString>& bindings,
                                 double fps,
                                 QString& error)
{
    for (auto it = bindings.constBegin(); it != bindings.constEnd(); ++it) {
        const QString& key = it.key();
        const QString& expr = it.value();

        double value = 0.0;

        // Special form: diff(innerExpr)
        if (expr.startsWith("diff(") && expr.endsWith(")")) {
            const QString inner = expr.mid(5, expr.length() - 6).trimmed();

            // diff(t) is a special case: always 1/fps
            if (inner == "t") {
                value = (fps > 0) ? 1.0 / fps : 0.0;
            } else if (prevVars.isEmpty()) {
                // No previous frame — skip this frame (signal with NaN sentinel)
                vars[key] = std::numeric_limits<double>::quiet_NaN();
                continue;
            } else {
                QString err;
                double cur  = ExprEval::evaluate(inner, vars,     &err);
                if (!err.isEmpty()) { error = QString("Binding '%1': %2").arg(key, err); return false; }
                double prev = ExprEval::evaluate(inner, prevVars, &err);
                if (!err.isEmpty()) { error = QString("Binding '%1' (prev): %2").arg(key, err); return false; }
                value = cur - prev;
            }
        } else {
            QString err;
            value = ExprEval::evaluate(expr, vars, &err);
            if (!err.isEmpty()) { error = QString("Binding '%1': %2").arg(key, err); return false; }
        }

        vars[key] = value;
    }
    return true;
}

// ── Reduce ────────────────────────────────────────────────────────────────

double PluginEngine::reduce(const QVector<double>& vals, const QString& method)
{
    if (vals.isEmpty()) return std::numeric_limits<double>::quiet_NaN();

    if (method == "sum")   return std::accumulate(vals.begin(), vals.end(), 0.0);
    if (method == "count") return static_cast<double>(vals.size());
    if (method == "min")   return *std::min_element(vals.begin(), vals.end());
    if (method == "max")   return *std::max_element(vals.begin(), vals.end());
    if (method == "last")  return vals.last();

    const double mean = std::accumulate(vals.begin(), vals.end(), 0.0) / vals.size();
    if (method == "mean") return mean;

    if (method == "std") {
        double sq = 0;
        for (double v : vals) sq += (v - mean) * (v - mean);
        return std::sqrt(sq / vals.size());
    }

    if (method == "median") {
        QVector<double> sorted = vals;
        std::sort(sorted.begin(), sorted.end());
        const int n = sorted.size();
        return (n % 2 == 0) ? (sorted[n/2-1] + sorted[n/2]) / 2.0 : sorted[n/2];
    }

    return mean;
}

// ── Main evaluation ───────────────────────────────────────────────────────

PluginEngine::PluginResult PluginEngine::evaluate(
    const PlotPluginSpec& spec,
    const QList<AnalysisSessionModel::AnalysisGroupData>& data,
    const PluginRoiPoints& roiPoints)
{
    PluginResult result;

    if (!spec.isValid) {
        result.errorMessage = "Plugin spec is invalid: " + spec.errors.join("; ");
        return result;
    }

    // Determine if µm can be used (all worms have calibration)
    bool allHaveUm = true;
    for (const auto& group : data)
        for (const auto& worm : group.worms)
            if (worm.umPerPixel <= 0) { allHaveUm = false; break; }
    result.usedUm = allHaveUm;

    if (spec.aggregate == PlotPluginSpec::Aggregate::PerWorm) {

        for (const auto& group : data) {
            GroupResult gr;
            gr.name = group.name;

            for (const auto& worm : group.worms) {
                const double um  = worm.umPerPixel;
                const double fps = worm.fps;

                QVector<double> frameValues;
                QMap<QString, double> prevVars;

                for (int i = 0; i < static_cast<int>(worm.points.size()); ++i) {
                    QMap<QString, double> vars = buildVars(worm, i, roiPoints, um, fps);

                    QString bindErr;
                    if (!applyBindings(vars, prevVars, spec.bindings, fps, bindErr)) {
                        result.errorMessage = bindErr;
                        return result;
                    }

                    // Skip frames where any binding produced NaN (e.g. diff on first frame)
                    bool hasNaN = false;
                    for (double v : vars.values()) if (std::isnan(v)) { hasNaN = true; break; }

                    prevVars = vars;  // save before potentially skipping
                    if (hasNaN) continue;

                    // Apply filter
                    if (!spec.filter.isEmpty()) {
                        QString err;
                        double pass = ExprEval::evaluate(spec.filter, vars, &err);
                        if (!err.isEmpty()) { result.errorMessage = "Filter: " + err; return result; }
                        if (pass == 0.0) continue;
                    }

                    // Evaluate formula
                    QString err;
                    double val = ExprEval::evaluate(spec.formula, vars, &err);
                    if (!err.isEmpty()) { result.errorMessage = "Formula: " + err; return result; }
                    if (!std::isnan(val) && std::isfinite(val))
                        frameValues.append(val);
                }

                if (!frameValues.isEmpty()) {
                    WormScalar ws;
                    ws.wormId = worm.wormId;
                    ws.label  = worm.label;
                    ws.color  = worm.color;
                    ws.value  = reduce(frameValues, spec.reduce);
                    gr.worms.append(ws);
                }
            }

            if (!gr.worms.isEmpty())
                result.groups.append(gr);
        }

    } else if (spec.aggregate == PlotPluginSpec::Aggregate::PerFrame) {

        for (const auto& group : data) {
            for (const auto& worm : group.worms) {
                const double um  = worm.umPerPixel;
                const double fps = worm.fps;

                WormSeries ws;
                ws.wormId   = worm.wormId;
                ws.label    = worm.label;
                ws.color    = worm.color;
                ws.groupName = group.name;

                QMap<QString, double> prevVars;

                for (int i = 0; i < static_cast<int>(worm.points.size()); ++i) {
                    QMap<QString, double> vars = buildVars(worm, i, roiPoints, um, fps);

                    QString bindErr;
                    if (!applyBindings(vars, prevVars, spec.bindings, fps, bindErr)) {
                        result.errorMessage = bindErr;
                        return result;
                    }

                    bool hasNaN = false;
                    for (double v : vars.values()) if (std::isnan(v)) { hasNaN = true; break; }
                    prevVars = vars;
                    if (hasNaN) continue;

                    if (!spec.filter.isEmpty()) {
                        QString err;
                        double pass = ExprEval::evaluate(spec.filter, vars, &err);
                        if (!err.isEmpty()) { result.errorMessage = "Filter: " + err; return result; }
                        if (pass == 0.0) continue;
                    }

                    QString err;
                    double val = ExprEval::evaluate(spec.formula, vars, &err);
                    if (!err.isEmpty()) { result.errorMessage = "Formula: " + err; return result; }
                    if (!std::isnan(val) && std::isfinite(val))
                        ws.points.append(QPointF(vars.value("t"), val));
                }

                if (!ws.points.isEmpty())
                    result.series.append(ws);
            }
        }

    } else {
        result.errorMessage = "Spatial aggregate not yet implemented";
        return result;
    }

    result.ok = true;
    return result;
}
