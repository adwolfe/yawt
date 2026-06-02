#include "pluginengine.h"
#include "expreval.h"

#include <QColor>
#include <QHash>
#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>

// ── Standard vocabulary constants ─────────────────────────────────────────
static constexpr double Q_SINGLE = 0.0;
static constexpr double Q_MERGED = 1.0;
static constexpr double Q_SPLIT  = 2.0;
static constexpr double Q_LOST   = 3.0;
static constexpr double kNaN     = std::numeric_limits<double>::quiet_NaN();

// ── Pre-compiled binding ──────────────────────────────────────────────────
// Compiled once per evaluate() call; used every frame without re-parsing.

struct CompiledBinding {
    QString       key;
    bool          isDiffT   = false;  // diff(t) → constant 1/fps
    bool          isDiff    = false;  // diff(inner) → inner(vars) - inner(prevVars)
    CompiledExpr  innerFn;            // compiled inner expr for diff()
    CompiledExpr  plainFn;            // compiled expr for plain bindings
};

// ── Build and initialize the var map (once per worm) ─────────────────────

static void initVarMap(VarMap& v,
                       const PluginRoiPoints& roi,
                       const QHash<QString, QString>& bindingKeys)
{
    v.reserve(80);
    // Standard vocabulary — all keys pre-inserted so per-frame updates are
    // in-place writes (no node allocation).
    const QStringList keys = {
        "x","y","x_um","y_um","frame","t",
        "quality","Single","Merged","Split","Lost",
        "area","area_um2","body_length","body_length_um","aspect_ratio",
        "xhead","yhead","xtail","ytail",
        "xhead_um","yhead_um","xtail_um","ytail_um",
        "fps"
    };
    for (const QString& k : keys) v[k] = 0.0;

    // ROI reference points
    if (roi.hasStart)  { v["start_x"]  = roi.startX;  v["start_y"]  = roi.startY;  v["dist_to_start"]  = 0.0; }
    if (roi.hasEnd)    { v["end_x"]    = roi.endX;    v["end_y"]    = roi.endY;    v["dist_to_end"]    = 0.0; }
    if (roi.hasCenter) { v["center_x"] = roi.centerX; v["center_y"] = roi.centerY; v["dist_to_center"] = 0.0; }

    // Named quality constants
    v["Single"] = Q_SINGLE; v["Merged"] = Q_MERGED;
    v["Split"]  = Q_SPLIT;  v["Lost"]   = Q_LOST;

    // Binding keys and their prev_ counterparts
    for (auto it = bindingKeys.constBegin(); it != bindingKeys.constEnd(); ++it) {
        v[it.key()]            = 0.0;
        v["prev_" + it.key()]  = kNaN;  // NaN until frame 1 completes
    }
}

// Update only the raw vocabulary values — no new hash nodes, just overwrites.
static void updateVarMap(VarMap& v,
                         const AnalysisSessionModel::AnalysisWormEntry& worm,
                         int idx,
                         const PluginRoiPoints& roi,
                         double umPerPixel,
                         double fps)
{
    const Tracking::WormTrackPoint& p = worm.points[idx];
    const double x  = static_cast<double>(p.position.x);
    const double y  = static_cast<double>(p.position.y);
    const double um = umPerPixel;
    const double t  = (fps > 0) ? p.frameNumberOriginal / fps : 0.0;

    v["x"] = x;     v["y"] = y;
    v["x_um"] = x*um; v["y_um"] = y*um;
    v["frame"] = p.frameNumberOriginal;
    v["t"]     = t;
    v["quality"] = static_cast<double>(static_cast<int>(p.quality));
    v["area"]          = p.area;
    v["area_um2"]      = p.area * um * um;
    v["body_length"]   = p.bodyLength;
    v["body_length_um"]= p.bodyLength * um;
    v["aspect_ratio"]  = p.aspectRatio;
    v["fps"]           = fps;

    if (p.hasTips) {
        v["xhead"] = p.headTip.x;  v["yhead"] = p.headTip.y;
        v["xtail"] = p.tailTip.x;  v["ytail"] = p.tailTip.y;
        v["xhead_um"] = p.headTip.x*um; v["yhead_um"] = p.headTip.y*um;
        v["xtail_um"] = p.tailTip.x*um; v["ytail_um"] = p.tailTip.y*um;
    }

    if (roi.hasStart) {
        double dx = x-roi.startX, dy = y-roi.startY;
        v["dist_to_start"] = std::sqrt(dx*dx + dy*dy);
    }
    if (roi.hasEnd) {
        double dx = x-roi.endX, dy = y-roi.endY;
        v["dist_to_end"] = std::sqrt(dx*dx + dy*dy);
    }
    if (roi.hasCenter) {
        double dx = x-roi.centerX, dy = y-roi.centerY;
        v["dist_to_center"] = std::sqrt(dx*dx + dy*dy);
    }
}

// ── Unused overload (kept for API compatibility) ───────────────────────────

QHash<QString, double> PluginEngine::buildVars(
    const AnalysisSessionModel::AnalysisWormEntry& worm,
    int pointIdx,
    const PluginRoiPoints& roi,
    double umPerPixel,
    double fps)
{
    VarMap v;
    initVarMap(v, roi, {});
    updateVarMap(v, worm, pointIdx, roi, umPerPixel, fps);
    return v;
}

// ── applyBindings (uses pre-compiled expressions) ─────────────────────────

bool PluginEngine::applyBindings(VarMap& vars,
                                 const VarMap& prevVars,
                                 const QHash<QString, QString>& bindings,
                                 double fps,
                                 QString& error)
{
    // Interpret path — only called from the unused public overload.
    for (auto it = bindings.constBegin(); it != bindings.constEnd(); ++it) {
        const QString& key = it.key();
        const QString& expr = it.value();
        if (expr.startsWith("diff(") && expr.endsWith(")")) {
            const QString inner = expr.mid(5, expr.length() - 6).trimmed();
            if (inner == "t") {
                vars[key] = (fps > 0) ? 1.0 / fps : 0.0;
            } else if (prevVars.isEmpty()) {
                vars[key] = kNaN; continue;
            } else {
                QString err;
                double cur  = ExprEval::evaluate(inner, vars, &err);
                if (!err.isEmpty()) { error = "Binding '" + key + "': " + err; return false; }
                double prev = ExprEval::evaluate(inner, prevVars, &err);
                if (!err.isEmpty()) { error = "Binding '" + key + "' (prev): " + err; return false; }
                vars[key] = cur - prev;
            }
        } else {
            QString err;
            vars[key] = ExprEval::evaluate(expr, vars, &err);
            if (!err.isEmpty()) { error = "Binding '" + key + "': " + err; return false; }
        }
    }
    return true;
}

// ── Reduce ────────────────────────────────────────────────────────────────

double PluginEngine::reduce(const QVector<double>& vals, const QString& method)
{
    if (vals.isEmpty()) return kNaN;
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
        QVector<double> s = vals;
        std::sort(s.begin(), s.end());
        const int n = s.size();
        return (n % 2 == 0) ? (s[n/2-1] + s[n/2]) / 2.0 : s[n/2];
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

    // ── Compile all expressions ONCE ─────────────────────────────────────
    // Bindings
    QList<CompiledBinding> compiledBindings;
    for (auto it = spec.bindings.constBegin(); it != spec.bindings.constEnd(); ++it) {
        CompiledBinding cb;
        cb.key = it.key();
        const QString& expr = it.value();
        if (expr.startsWith("diff(") && expr.endsWith(")")) {
            const QString inner = expr.mid(5, expr.length() - 6).trimmed();
            cb.isDiff = true;
            if (inner == "t") {
                cb.isDiffT = true;
            } else {
                QString err;
                cb.innerFn = ExprEval::compile(inner, &err);
                if (!cb.innerFn) {
                    result.errorMessage = "Binding '" + cb.key + "': " + err;
                    return result;
                }
            }
        } else {
            cb.isDiff = false;
            QString err;
            cb.plainFn = ExprEval::compile(expr, &err);
            if (!cb.plainFn) {
                result.errorMessage = "Binding '" + cb.key + "': " + err;
                return result;
            }
        }
        compiledBindings.append(std::move(cb));
    }

    // Filter (optional)
    CompiledExpr compiledFilter;
    if (!spec.filter.isEmpty()) {
        QString err;
        compiledFilter = ExprEval::compile(spec.filter, &err);
        if (!compiledFilter) { result.errorMessage = "Filter: " + err; return result; }
    }

    // Formula
    CompiledExpr compiledFormula;
    {
        QString err;
        compiledFormula = ExprEval::compile(spec.formula, &err);
        if (!compiledFormula) { result.errorMessage = "Formula: " + err; return result; }
    }

    // µm flag
    bool allHaveUm = true;
    for (const auto& group : data)
        for (const auto& worm : group.worms)
            if (worm.umPerPixel <= 0) { allHaveUm = false; break; }
    result.usedUm = allHaveUm;

    // ── Per-worm frame loop ───────────────────────────────────────────────
    // vars and prevVars are allocated ONCE per worm; values updated in-place
    // each frame — zero per-frame heap allocations after init.

    auto runWormLoop = [&](const AnalysisSessionModel::AnalysisWormEntry& worm,
                           std::function<bool(const VarMap&)> frameCallback) -> bool
    {
        const double um  = worm.umPerPixel;
        const double fps = worm.fps;

        // Pre-allocate var maps with all keys once.
        VarMap vars, prevVars;
        initVarMap(vars,     roiPoints, spec.bindings);
        initVarMap(prevVars, roiPoints, spec.bindings);
        // prevVars starts truly empty (signals "no previous frame" for diff).
        bool hasPrevFrame = false;

        // prevBindingVals: resolved binding values from the previous frame,
        // injected as prev_<name> before each frame's binding evaluation.
        // Seeded with NaN so frame 0 is safely skipped when prev_* is referenced.
        QHash<QString, double> prevBindingVals;
        for (const auto& cb : compiledBindings)
            prevBindingVals[cb.key] = kNaN;

        for (int i = 0; i < static_cast<int>(worm.points.size()); ++i) {
            // 1. Update raw vocabulary values in-place (no allocation).
            updateVarMap(vars, worm, i, roiPoints, um, fps);

            // 2. Inject prev_<name> (O(n) in-place writes, no allocation).
            for (auto pit = prevBindingVals.constBegin();
                 pit != prevBindingVals.constEnd(); ++pit)
                vars["prev_" + pit.key()] = pit.value();

            // 3. Apply compiled bindings.
            for (const CompiledBinding& cb : compiledBindings) {
                double value = 0.0;
                if (cb.isDiff) {
                    if (cb.isDiffT) {
                        value = (fps > 0) ? 1.0 / fps : 0.0;
                    } else if (!hasPrevFrame) {
                        vars[cb.key] = kNaN;
                        continue;
                    } else {
                        value = cb.innerFn(vars) - cb.innerFn(prevVars);
                    }
                } else {
                    value = cb.plainFn(vars);
                }
                vars[cb.key] = value;
            }

            // 4. Capture binding vals for next frame; update prevVars raw vocab.
            for (const auto& cb : compiledBindings)
                prevBindingVals[cb.key] = vars.value(cb.key, kNaN);

            // Update prevVars raw vocabulary in-place (no allocation).
            updateVarMap(prevVars, worm, i, roiPoints, um, fps);
            hasPrevFrame = true;

            // 5. Skip if any binding is NaN.
            bool hasNaN = false;
            for (const auto& cb : compiledBindings)
                if (std::isnan(vars.value(cb.key, kNaN))) { hasNaN = true; break; }
            if (hasNaN) continue;

            // 6. Apply filter.
            if (compiledFilter && compiledFilter(vars) == 0.0) continue;

            // 7. Invoke callback with the fully populated var map.
            if (!frameCallback(vars)) return false;
        }
        return true;
    };

    // ── Aggregate modes ───────────────────────────────────────────────────

    if (spec.aggregate == PlotPluginSpec::Aggregate::PerWorm) {
        for (const auto& group : data) {
            GroupResult gr;
            gr.name = group.name;
            for (const auto& worm : group.worms) {
                QVector<double> frameValues;
                const bool ok = runWormLoop(worm, [&](const VarMap& vars) -> bool {
                    const double val = compiledFormula(vars);
                    if (!std::isnan(val) && std::isfinite(val))
                        frameValues.append(val);
                    return true;
                });
                if (!ok) return result;
                if (!frameValues.isEmpty()) {
                    WormScalar ws;
                    ws.wormId = worm.wormId;
                    ws.label  = worm.label;
                    ws.color  = worm.color;
                    ws.value  = reduce(frameValues, spec.reduce);
                    gr.worms.append(ws);
                }
            }
            if (!gr.worms.isEmpty()) result.groups.append(gr);
        }

    } else if (spec.aggregate == PlotPluginSpec::Aggregate::PerFrame) {
        for (const auto& group : data) {
            for (const auto& worm : group.worms) {
                WormSeries ws;
                ws.wormId    = worm.wormId;
                ws.label     = worm.label;
                ws.color     = worm.color;
                ws.groupName = group.name;
                const bool ok = runWormLoop(worm, [&](const VarMap& vars) -> bool {
                    const double val = compiledFormula(vars);
                    if (!std::isnan(val) && std::isfinite(val))
                        ws.points.append(QPointF(vars.value("t"), val));
                    return true;
                });
                if (!ok) return result;
                if (!ws.points.isEmpty()) result.series.append(ws);
            }
        }

    } else {
        result.errorMessage = "Spatial aggregate not yet implemented";
        return result;
    }

    result.ok = true;
    return result;
}
