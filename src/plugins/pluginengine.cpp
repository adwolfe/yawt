#include "pluginengine.h"
#include "expreval.h"

#include <QColor>
#include <QHash>
#include <QMutex>
#include <QMutexLocker>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
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

enum Slot : int {
    S_X, S_Y, S_X_UM, S_Y_UM, S_FRAME, S_T,
    S_QUALITY, S_SINGLE, S_MERGED, S_SPLIT, S_LOST,
    S_AREA, S_AREA_UM2, S_BODY_LENGTH, S_BODY_LENGTH_UM, S_ASPECT_RATIO,
    S_XHEAD, S_YHEAD, S_XTAIL, S_YTAIL,
    S_XHEAD_UM, S_YHEAD_UM, S_XTAIL_UM, S_YTAIL_UM,
    S_SPEED, S_SPEED_PX, S_SPEED_UM,
    S_HAS_START, S_START_X, S_START_Y, S_DIST_TO_START, S_DIST_TO_START_PX, S_DIST_TO_START_UM,
    S_HAS_END, S_END_X, S_END_Y, S_DIST_TO_END, S_DIST_TO_END_PX, S_DIST_TO_END_UM,
    S_HAS_CENTER, S_CENTER_X, S_CENTER_Y, S_DIST_TO_CENTER, S_DIST_TO_CENTER_PX, S_DIST_TO_CENTER_UM,
    S_FPS,
    S_COUNT
};

using SlotExpr = std::function<double(const QVector<double>&)>;

struct SlotCompileState {
    const QChar* src = nullptr;
    int pos = 0;
    int len = 0;
    QString error;
    std::function<int(const QString&)> resolve;

    QChar cur() const { return (pos < len) ? src[pos] : QChar(0); }
    void skip() { while (pos < len && src[pos].isSpace()) ++pos; }
    void advance() { ++pos; skip(); }
};

static void setSlotErr(SlotCompileState& s, const QString& msg)
{
    if (s.error.isEmpty()) s.error = msg;
}

static SlotExpr compileSlotExpr(SlotCompileState& s);

static QString parseSlotIdent(SlotCompileState& s)
{
    int start = s.pos;
    while (s.pos < s.len && (s.src[s.pos].isLetterOrNumber() || s.src[s.pos] == '_')) ++s.pos;
    const QString id = QString(s.src + start, s.pos - start);
    s.skip();
    return id;
}

static SlotExpr compileSlotLiteral(SlotCompileState& s)
{
    int start = s.pos;
    if (s.cur() == '-') ++s.pos;
    while (s.pos < s.len && (s.src[s.pos].isDigit() || s.src[s.pos] == '.')) ++s.pos;
    if (s.pos < s.len && (s.src[s.pos] == 'e' || s.src[s.pos] == 'E')) {
        ++s.pos;
        if (s.pos < s.len && (s.src[s.pos] == '+' || s.src[s.pos] == '-')) ++s.pos;
        while (s.pos < s.len && s.src[s.pos].isDigit()) ++s.pos;
    }
    const QString tok = QString(s.src + start, s.pos - start);
    s.skip();
    bool ok = false;
    const double v = tok.toDouble(&ok);
    if (!ok) { setSlotErr(s, "Invalid number: " + tok); return nullptr; }
    return [v](const QVector<double>&) { return v; };
}

static SlotExpr compileSlotBuiltin(const QString& name, QVector<SlotExpr> args, SlotCompileState& s)
{
    if (name == "min" || name == "max" || name == "pow") {
        if (args.size() != 2) { setSlotErr(s, name + "() requires 2 arguments"); return nullptr; }
        auto a = std::move(args[0]), b = std::move(args[1]);
        if (name == "min") return [a,b](const QVector<double>& v){ return std::min(a(v), b(v)); };
        if (name == "max") return [a,b](const QVector<double>& v){ return std::max(a(v), b(v)); };
        return [a,b](const QVector<double>& v){ return std::pow(a(v), b(v)); };
    }
    if (args.size() != 1) { setSlotErr(s, name + "() requires 1 argument"); return nullptr; }
    auto a = std::move(args[0]);
    if (name == "sqrt")  return [a](const QVector<double>& v){ double x=a(v); return x<0?0.0:std::sqrt(x); };
    if (name == "abs")   return [a](const QVector<double>& v){ return std::abs(a(v)); };
    if (name == "floor") return [a](const QVector<double>& v){ return std::floor(a(v)); };
    if (name == "ceil")  return [a](const QVector<double>& v){ return std::ceil(a(v)); };
    if (name == "sin")   return [a](const QVector<double>& v){ return std::sin(a(v)); };
    if (name == "cos")   return [a](const QVector<double>& v){ return std::cos(a(v)); };
    if (name == "tan")   return [a](const QVector<double>& v){ return std::tan(a(v)); };
    if (name == "log")   return [a](const QVector<double>& v){ double x=a(v); return x<=0?0.0:std::log(x); };
    if (name == "exp")   return [a](const QVector<double>& v){ return std::exp(a(v)); };
    setSlotErr(s, "Unknown function: " + name);
    return nullptr;
}

static SlotExpr compileSlotPrimary(SlotCompileState& s)
{
    s.skip();
    if (!s.error.isEmpty()) return nullptr;
    if (s.cur() == '-') {
        s.advance();
        auto inner = compileSlotPrimary(s);
        if (!inner) return nullptr;
        return [inner](const QVector<double>& v){ return -inner(v); };
    }
    if (s.cur() == '+') { s.advance(); return compileSlotPrimary(s); }
    if (s.cur() == '(') {
        s.advance();
        auto inner = compileSlotExpr(s);
        if (!inner) return nullptr;
        if (s.cur() != ')') { setSlotErr(s, "Expected ')'"); return nullptr; }
        s.advance();
        return inner;
    }
    if (s.cur().isDigit() || s.cur() == '.') return compileSlotLiteral(s);
    if (s.cur().isLetter() || s.cur() == '_') {
        const QString id = parseSlotIdent(s);
        if (s.cur() == '(') {
            s.advance();
            QVector<SlotExpr> args;
            if (s.cur() != ')') {
                args.append(compileSlotExpr(s));
                if (!args.last()) return nullptr;
                while (s.cur() == ',') {
                    s.advance();
                    args.append(compileSlotExpr(s));
                    if (!args.last()) return nullptr;
                }
            }
            if (s.cur() != ')') { setSlotErr(s, "Expected ')'"); return nullptr; }
            s.advance();
            return compileSlotBuiltin(id, std::move(args), s);
        }
        const int slot = s.resolve ? s.resolve(id) : -1;
        return [slot](const QVector<double>& vars) -> double {
            return (slot >= 0 && slot < vars.size()) ? vars[slot] : 0.0;
        };
    }
    setSlotErr(s, QString("Unexpected character: '%1'").arg(s.cur()));
    return nullptr;
}

static SlotExpr compileSlotPower(SlotCompileState& s)
{
    auto base = compileSlotPrimary(s);
    if (!base || !s.error.isEmpty()) return base;
    if (s.cur() == '^') {
        s.advance();
        auto exp = compileSlotPower(s);
        if (!exp) return nullptr;
        return [base,exp](const QVector<double>& v){ return std::pow(base(v), exp(v)); };
    }
    return base;
}

static SlotExpr compileSlotMul(SlotCompileState& s)
{
    auto lhs = compileSlotPower(s);
    if (!lhs) return nullptr;
    while (s.error.isEmpty() && (s.cur() == '*' || s.cur() == '/')) {
        const bool isDiv = (s.cur() == '/');
        s.advance();
        auto rhs = compileSlotPower(s);
        if (!rhs) return nullptr;
        auto prev = std::move(lhs);
        lhs = isDiv
            ? SlotExpr([prev,rhs](const QVector<double>& v){ double r=rhs(v); return r == 0.0 ? 0.0 : prev(v) / r; })
            : SlotExpr([prev,rhs](const QVector<double>& v){ return prev(v) * rhs(v); });
    }
    return lhs;
}

static SlotExpr compileSlotAdd(SlotCompileState& s)
{
    auto lhs = compileSlotMul(s);
    if (!lhs) return nullptr;
    while (s.error.isEmpty() && (s.cur() == '+' || s.cur() == '-')) {
        const bool isSub = (s.cur() == '-');
        s.advance();
        auto rhs = compileSlotMul(s);
        if (!rhs) return nullptr;
        auto prev = std::move(lhs);
        lhs = isSub
            ? SlotExpr([prev,rhs](const QVector<double>& v){ return prev(v) - rhs(v); })
            : SlotExpr([prev,rhs](const QVector<double>& v){ return prev(v) + rhs(v); });
    }
    return lhs;
}

static SlotExpr compileSlotCompare(SlotCompileState& s)
{
    auto lhs = compileSlotAdd(s);
    if (!lhs || !s.error.isEmpty()) return lhs;
    const QChar c0 = s.cur();
    if (c0 == '=' || c0 == '!' || c0 == '<' || c0 == '>') {
        QString op; op += c0; ++s.pos;
        if (s.pos < s.len && s.src[s.pos] == '=') { op += '='; ++s.pos; }
        s.skip();
        auto rhs = compileSlotAdd(s);
        if (!rhs) return nullptr;
        if      (op == "==") return [lhs,rhs](const QVector<double>& v){ return lhs(v)==rhs(v)?1.0:0.0; };
        else if (op == "!=") return [lhs,rhs](const QVector<double>& v){ return lhs(v)!=rhs(v)?1.0:0.0; };
        else if (op == "<")  return [lhs,rhs](const QVector<double>& v){ return lhs(v)< rhs(v)?1.0:0.0; };
        else if (op == "<=") return [lhs,rhs](const QVector<double>& v){ return lhs(v)<=rhs(v)?1.0:0.0; };
        else if (op == ">")  return [lhs,rhs](const QVector<double>& v){ return lhs(v)> rhs(v)?1.0:0.0; };
        else if (op == ">=") return [lhs,rhs](const QVector<double>& v){ return lhs(v)>=rhs(v)?1.0:0.0; };
        setSlotErr(s, "Unknown operator: " + op);
    }
    return lhs;
}

static SlotExpr compileSlotAnd(SlotCompileState& s)
{
    auto lhs = compileSlotCompare(s);
    if (!lhs) return nullptr;
    while (s.error.isEmpty() && s.cur() == '&' && s.pos+1 < s.len && s.src[s.pos+1] == '&') {
        s.pos += 2; s.skip();
        auto rhs = compileSlotCompare(s);
        if (!rhs) return nullptr;
        auto prev = std::move(lhs);
        lhs = [prev,rhs](const QVector<double>& v){ return (prev(v)!=0.0 && rhs(v)!=0.0)?1.0:0.0; };
    }
    return lhs;
}

static SlotExpr compileSlotOr(SlotCompileState& s)
{
    auto lhs = compileSlotAnd(s);
    if (!lhs) return nullptr;
    while (s.error.isEmpty() && s.cur() == '|' && s.pos+1 < s.len && s.src[s.pos+1] == '|') {
        s.pos += 2; s.skip();
        auto rhs = compileSlotAnd(s);
        if (!rhs) return nullptr;
        auto prev = std::move(lhs);
        lhs = [prev,rhs](const QVector<double>& v){ return (prev(v)!=0.0 || rhs(v)!=0.0)?1.0:0.0; };
    }
    return lhs;
}

static SlotExpr compileSlotTernary(SlotCompileState& s)
{
    auto cond = compileSlotOr(s);
    if (!cond || !s.error.isEmpty()) return cond;
    if (s.cur() == '?') {
        s.advance();
        auto thenE = compileSlotOr(s);
        if (!thenE) return nullptr;
        if (s.cur() != ':') { setSlotErr(s, "Expected ':'"); return nullptr; }
        s.advance();
        auto elseE = compileSlotOr(s);
        if (!elseE) return nullptr;
        return [cond,thenE,elseE](const QVector<double>& v){
            return cond(v) != 0.0 ? thenE(v) : elseE(v); };
    }
    return cond;
}

static SlotExpr compileSlotExpr(SlotCompileState& s) { return compileSlotTernary(s); }

static SlotExpr compileSlotExpression(const QString& expr,
                                      const std::function<int(const QString&)>& resolve,
                                      QString* error)
{
    if (expr.trimmed().isEmpty()) {
        if (error) *error = "Empty expression";
        return nullptr;
    }
    SlotCompileState s;
    s.src = expr.constData();
    s.pos = 0;
    s.len = expr.length();
    s.resolve = resolve;
    s.skip();

    SlotExpr fn = compileSlotExpr(s);
    if (s.error.isEmpty() && s.pos < s.len)
        s.error = QString("Unexpected trailing text at position %1").arg(s.pos);
    if (!s.error.isEmpty()) {
        if (error) *error = s.error;
        return nullptr;
    }
    return fn;
}

static const QHash<QString, int>& standardSlotMap()
{
    static const QHash<QString, int> map{
        {"x", S_X}, {"y", S_Y}, {"x_um", S_X_UM}, {"y_um", S_Y_UM}, {"frame", S_FRAME}, {"t", S_T},
        {"quality", S_QUALITY}, {"Single", S_SINGLE}, {"Merged", S_MERGED}, {"Split", S_SPLIT}, {"Lost", S_LOST},
        {"area", S_AREA}, {"area_um2", S_AREA_UM2}, {"body_length", S_BODY_LENGTH},
        {"body_length_um", S_BODY_LENGTH_UM}, {"aspect_ratio", S_ASPECT_RATIO},
        {"xhead", S_XHEAD}, {"yhead", S_YHEAD}, {"xtail", S_XTAIL}, {"ytail", S_YTAIL},
        {"xhead_um", S_XHEAD_UM}, {"yhead_um", S_YHEAD_UM}, {"xtail_um", S_XTAIL_UM}, {"ytail_um", S_YTAIL_UM},
        {"speed", S_SPEED}, {"speed_px", S_SPEED_PX}, {"speed_um", S_SPEED_UM},
        {"has_start", S_HAS_START}, {"start_x", S_START_X}, {"start_y", S_START_Y},
        {"dist_to_start", S_DIST_TO_START}, {"dist_to_start_px", S_DIST_TO_START_PX}, {"dist_to_start_um", S_DIST_TO_START_UM},
        {"has_end", S_HAS_END}, {"end_x", S_END_X}, {"end_y", S_END_Y},
        {"dist_to_end", S_DIST_TO_END}, {"dist_to_end_px", S_DIST_TO_END_PX}, {"dist_to_end_um", S_DIST_TO_END_UM},
        {"has_center", S_HAS_CENTER}, {"center_x", S_CENTER_X}, {"center_y", S_CENTER_Y},
        {"dist_to_center", S_DIST_TO_CENTER}, {"dist_to_center_px", S_DIST_TO_CENTER_PX}, {"dist_to_center_um", S_DIST_TO_CENTER_UM},
        {"fps", S_FPS}
    };
    return map;
}

struct PlanBinding {
    QString key;
    int slot = -1;
    int prevSlot = -1;
    bool isDiff = false;
    bool isDiffT = false;
    bool isSmooth = false;
    SlotExpr plainFn;
    SlotExpr innerFn;
    SlotExpr smoothValueFn;
    SlotExpr smoothWindowFn;
    SlotExpr smoothFilterFn;
};

struct CompiledPluginPlan {
    int slotCount = S_COUNT;
    QSet<int> requiredSlots;
    QVector<PlanBinding> bindings;
    SlotExpr filterFn;
    SlotExpr formulaFn;
};

static QString pluginPlanCacheKey(const PlotPluginSpec& spec)
{
    QStringList bindingParts;
    QStringList keys = spec.bindings.keys();
    keys.sort();
    for (const QString& key : keys)
        bindingParts << key + "=" + spec.bindings.value(key);

    return QString::number(spec.version) + "\n"
         + spec.filePath + "\n"
         + spec.name + "\n"
         + QString::number(static_cast<int>(spec.aggregate)) + "\n"
         + QString::number(static_cast<int>(spec.plotType)) + "\n"
         + spec.formula + "\n"
         + spec.filter + "\n"
         + spec.reduce + "\n"
         + bindingParts.join("\n");
}

static QSet<QString> identifiersInExpression(const QString& expr)
{
    QSet<QString> ids;
    int pos = 0;
    while (pos < expr.size()) {
        const QChar c = expr[pos];
        if (!c.isLetter() && c != '_') {
            ++pos;
            continue;
        }
        const int start = pos++;
        while (pos < expr.size() && (expr[pos].isLetterOrNumber() || expr[pos] == '_')) {
            ++pos;
        }
        ids.insert(expr.mid(start, pos - start));
    }
    return ids;
}

static bool extractFunctionArgs(const QString& expr, const QString& fnName, QStringList& args)
{
    args.clear();
    const QString trimmed = expr.trimmed();
    const QString prefix = fnName + "(";
    if (!trimmed.startsWith(prefix) || !trimmed.endsWith(")")) {
        return false;
    }

    const QString inner = trimmed.mid(prefix.length(), trimmed.length() - prefix.length() - 1);
    int depth = 0;
    int start = 0;
    for (int i = 0; i < inner.length(); ++i) {
        const QChar c = inner[i];
        if (c == '(') {
            ++depth;
        } else if (c == ')') {
            --depth;
            if (depth < 0) return false;
        } else if (c == ',' && depth == 0) {
            args.append(inner.mid(start, i - start).trimmed());
            start = i + 1;
        }
    }
    if (depth != 0) return false;
    args.append(inner.mid(start).trimmed());
    return true;
}

static QSet<QString> requiredBindingKeys(const QHash<QString, QString>& bindings,
                                         const QString& formula,
                                         const QString& filter)
{
    QSet<QString> required;
    QStringList pending = identifiersInExpression(formula).values();
    pending.append(identifiersInExpression(filter).values());

    while (!pending.isEmpty()) {
        const QString id = pending.takeLast();
        QString bindingKey = id;
        if (!bindings.contains(bindingKey) && bindingKey.startsWith("prev_")) {
            bindingKey = bindingKey.mid(5);
        }
        if (!bindings.contains(bindingKey) || required.contains(bindingKey)) {
            continue;
        }

        required.insert(bindingKey);
        pending.append(identifiersInExpression(bindings.value(bindingKey)).values());
    }

    return required;
}

static QStringList orderedBindingKeys(const QHash<QString, QString>& bindings,
                                      const QSet<QString>& required)
{
    QStringList remaining = bindings.keys();
    remaining.erase(std::remove_if(remaining.begin(), remaining.end(),
                                   [&](const QString& key) { return !required.contains(key); }),
                    remaining.end());
    remaining.sort();
    QStringList ordered;

    while (!remaining.isEmpty()) {
        bool progressed = false;
        for (int i = 0; i < remaining.size();) {
            const QString key = remaining[i];
            const QSet<QString> ids = identifiersInExpression(bindings.value(key));
            bool dependsOnRemaining = false;
            for (const QString& other : remaining) {
                if (other == key) continue;
                if (ids.contains(other)) {
                    dependsOnRemaining = true;
                    break;
                }
            }
            if (dependsOnRemaining) {
                ++i;
                continue;
            }
            ordered.append(key);
            remaining.removeAt(i);
            progressed = true;
        }
        if (!progressed) {
            ordered.append(remaining);
            break;
        }
    }

    return ordered;
}

static std::shared_ptr<CompiledPluginPlan> compilePluginPlan(const PlotPluginSpec& spec, QString& error)
{
    auto plan = std::make_shared<CompiledPluginPlan>();
    QHash<QString, int> dynamicSlots;
    int nextSlot = S_COUNT;

    const QSet<QString> requiredBindings = requiredBindingKeys(spec.bindings, spec.formula, spec.filter);
    const QStringList bindingKeys = orderedBindingKeys(spec.bindings, requiredBindings);
    for (const QString& key : bindingKeys) {
        dynamicSlots.insert(key, nextSlot++);
        dynamicSlots.insert("prev_" + key, nextSlot++);
    }
    plan->slotCount = nextSlot;

    auto resolve = [&](const QString& name) -> int {
        const auto dyn = dynamicSlots.constFind(name);
        if (dyn != dynamicSlots.constEnd()) return dyn.value();

        const auto stdIt = standardSlotMap().constFind(name);
        if (stdIt != standardSlotMap().constEnd()) {
            plan->requiredSlots.insert(stdIt.value());
            return stdIt.value();
        }
        return -1;
    };

    for (const QString& key : bindingKeys) {
        PlanBinding binding;
        binding.key = key;
        binding.slot = dynamicSlots.value(key, -1);
        binding.prevSlot = dynamicSlots.value("prev_" + key, -1);

        const QString expr = spec.bindings.value(key).trimmed();
        QStringList fnArgs;
        if (extractFunctionArgs(expr, "diff", fnArgs)) {
            if (fnArgs.size() != 1) {
                error = "Binding '" + key + "': diff() requires 1 argument";
                return nullptr;
            }
            const QString inner = fnArgs.first();
            binding.isDiff = true;
            if (inner == "t") {
                binding.isDiffT = true;
                plan->requiredSlots.insert(S_T);
            } else {
                QString err;
                binding.innerFn = compileSlotExpression(inner, resolve, &err);
                if (!binding.innerFn) {
                    error = "Binding '" + key + "': " + err;
                    return nullptr;
                }
            }
        } else if (extractFunctionArgs(expr, "smooth", fnArgs)) {
            if (fnArgs.size() < 2 || fnArgs.size() > 3) {
                error = "Binding '" + key + "': smooth() requires 2 or 3 arguments";
                return nullptr;
            }
            binding.isSmooth = true;
            plan->requiredSlots.insert(S_T);
            plan->requiredSlots.insert(S_FRAME);

            QString err;
            binding.smoothValueFn = compileSlotExpression(fnArgs[0], resolve, &err);
            if (!binding.smoothValueFn) {
                error = "Binding '" + key + "' smooth value: " + err;
                return nullptr;
            }
            binding.smoothWindowFn = compileSlotExpression(fnArgs[1], resolve, &err);
            if (!binding.smoothWindowFn) {
                error = "Binding '" + key + "' smooth window: " + err;
                return nullptr;
            }
            if (fnArgs.size() == 3) {
                binding.smoothFilterFn = compileSlotExpression(fnArgs[2], resolve, &err);
                if (!binding.smoothFilterFn) {
                    error = "Binding '" + key + "' smooth filter: " + err;
                    return nullptr;
                }
            }
        } else {
            QString err;
            binding.plainFn = compileSlotExpression(expr, resolve, &err);
            if (!binding.plainFn) {
                error = "Binding '" + key + "': " + err;
                return nullptr;
            }
        }
        plan->bindings.append(std::move(binding));
    }

    if (!spec.filter.isEmpty()) {
        QString err;
        plan->filterFn = compileSlotExpression(spec.filter, resolve, &err);
        if (!plan->filterFn) {
            error = "Filter: " + err;
            return nullptr;
        }
    }

    {
        QString err;
        plan->formulaFn = compileSlotExpression(spec.formula, resolve, &err);
        if (!plan->formulaFn) {
            error = "Formula: " + err;
            return nullptr;
        }
    }

    return plan;
}

static std::shared_ptr<CompiledPluginPlan> cachedPluginPlan(const PlotPluginSpec& spec, QString& error)
{
    static QMutex mutex;
    static QHash<QString, std::shared_ptr<CompiledPluginPlan>> cache;

    const QString key = pluginPlanCacheKey(spec);
    {
        QMutexLocker locker(&mutex);
        auto it = cache.constFind(key);
        if (it != cache.constEnd()) {
            return it.value();
        }
    }

    std::shared_ptr<CompiledPluginPlan> plan = compilePluginPlan(spec, error);
    if (!plan) return nullptr;

    {
        QMutexLocker locker(&mutex);
        cache.insert(key, plan);
    }
    return plan;
}

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
        "speed","speed_px","speed_um",
        "has_start","start_x","start_y","dist_to_start","dist_to_start_px","dist_to_start_um",
        "has_end","end_x","end_y","dist_to_end","dist_to_end_px","dist_to_end_um",
        "has_center","center_x","center_y","dist_to_center","dist_to_center_px","dist_to_center_um",
        "fps"
    };
    for (const QString& k : keys) v[k] = 0.0;

    // ROI reference points
    if (roi.hasStart)  { v["has_start"] = 1.0;  v["start_x"]  = roi.startX;  v["start_y"]  = roi.startY; }
    if (roi.hasEnd)    { v["has_end"] = 1.0;    v["end_x"]    = roi.endX;    v["end_y"]    = roi.endY; }
    if (roi.hasCenter) { v["has_center"] = 1.0; v["center_x"] = roi.centerX; v["center_y"] = roi.centerY; }

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
        const double distPx = std::sqrt(dx*dx + dy*dy);
        const double distUm = (um > 0.0) ? distPx * um : distPx;
        v["dist_to_start_px"] = distPx;
        v["dist_to_start_um"] = distUm;
        v["dist_to_start"] = (um > 0.0) ? distUm : distPx;
    }
    if (roi.hasEnd) {
        double dx = x-roi.endX, dy = y-roi.endY;
        const double distPx = std::sqrt(dx*dx + dy*dy);
        const double distUm = (um > 0.0) ? distPx * um : distPx;
        v["dist_to_end_px"] = distPx;
        v["dist_to_end_um"] = distUm;
        v["dist_to_end"] = (um > 0.0) ? distUm : distPx;
    }
    if (roi.hasCenter) {
        double dx = x-roi.centerX, dy = y-roi.centerY;
        const double distPx = std::sqrt(dx*dx + dy*dy);
        const double distUm = (um > 0.0) ? distPx * um : distPx;
        v["dist_to_center_px"] = distPx;
        v["dist_to_center_um"] = distUm;
        v["dist_to_center"] = (um > 0.0) ? distUm : distPx;
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

struct SpeedSlotState {
    bool hasPrev = false;
    cv::Point2f prevPos{};
    int prevFrame = 0;
    std::deque<std::pair<int, double>> winPx;
    std::deque<std::pair<int, double>> winUm;
    double sumPx = 0.0;
    double sumUm = 0.0;

    void reset() {
        hasPrev = false;
        winPx.clear();
        winUm.clear();
        sumPx = 0.0;
        sumUm = 0.0;
    }
};

struct SmoothSlotState {
    std::deque<std::pair<double, double>> win;
    double sum = 0.0;
    double lastCoord = -std::numeric_limits<double>::infinity();

    void reset() {
        win.clear();
        sum = 0.0;
        lastCoord = -std::numeric_limits<double>::infinity();
    }
};

static double updateSmoothSlot(SmoothSlotState& state,
                               double value,
                               double window,
                               double coord,
                               bool acceptSample)
{
    if (!std::isfinite(coord)) {
        return kNaN;
    }
    if (coord < state.lastCoord) {
        state.reset();
    }
    state.lastCoord = coord;

    if (!std::isfinite(window) || window < 0.0) {
        window = 0.0;
    }

    while (!state.win.empty() && (coord - state.win.front().first) > window) {
        state.sum -= state.win.front().second;
        state.win.pop_front();
    }

    if (!acceptSample || !std::isfinite(value)) {
        return kNaN;
    }

    state.win.emplace_back(coord, value);
    state.sum += value;

    return state.win.empty()
        ? kNaN
        : state.sum / static_cast<double>(state.win.size());
}

static PluginRoiPoints effectiveRoiForWorm(
    const AnalysisSessionModel::AnalysisWormEntry& worm,
    const PluginRoiPoints& fallback)
{
    PluginRoiPoints roi = fallback;
    if (worm.hasStartPoint) {
        roi.hasStart = true;
        roi.startX = worm.startPoint.x();
        roi.startY = worm.startPoint.y();
    }
    if (worm.hasEndPoint) {
        roi.hasEnd = true;
        roi.endX = worm.endPoint.x();
        roi.endY = worm.endPoint.y();
    }
    if (worm.hasCenterPoint) {
        roi.hasCenter = true;
        roi.centerX = worm.centerPoint.x();
        roi.centerY = worm.centerPoint.y();
    }
    return roi;
}

static void initSlotValues(QVector<double>& slotValues,
                           const CompiledPluginPlan& plan,
                           const PluginRoiPoints& roi,
                           double fps)
{
    slotValues.fill(0.0);
    slotValues[S_SINGLE] = Q_SINGLE;
    slotValues[S_MERGED] = Q_MERGED;
    slotValues[S_SPLIT] = Q_SPLIT;
    slotValues[S_LOST] = Q_LOST;
    slotValues[S_FPS] = fps;

    if (roi.hasStart) {
        slotValues[S_HAS_START] = 1.0;
        slotValues[S_START_X] = roi.startX;
        slotValues[S_START_Y] = roi.startY;
    }
    if (roi.hasEnd) {
        slotValues[S_HAS_END] = 1.0;
        slotValues[S_END_X] = roi.endX;
        slotValues[S_END_Y] = roi.endY;
    }
    if (roi.hasCenter) {
        slotValues[S_HAS_CENTER] = 1.0;
        slotValues[S_CENTER_X] = roi.centerX;
        slotValues[S_CENTER_Y] = roi.centerY;
    }

    for (const PlanBinding& binding : plan.bindings) {
        if (binding.prevSlot >= 0 && binding.prevSlot < slotValues.size())
            slotValues[binding.prevSlot] = kNaN;
    }
}

static void updateSlotValues(QVector<double>& slotValues,
                             const CompiledPluginPlan& plan,
                             const AnalysisSessionModel::AnalysisWormEntry& worm,
                             const Tracking::WormTrackPoint& p,
                             const PluginRoiPoints& roi,
                             SpeedSlotState& speedState)
{
    const QSet<int>& req = plan.requiredSlots;
    const double x = static_cast<double>(p.position.x);
    const double y = static_cast<double>(p.position.y);
    const double um = worm.umPerPixel;
    const double fps = worm.fps;

    if (req.contains(S_X)) slotValues[S_X] = x;
    if (req.contains(S_Y)) slotValues[S_Y] = y;
    if (req.contains(S_X_UM)) slotValues[S_X_UM] = x * um;
    if (req.contains(S_Y_UM)) slotValues[S_Y_UM] = y * um;
    if (req.contains(S_FRAME)) slotValues[S_FRAME] = p.frameNumberOriginal;
    if (req.contains(S_T)) slotValues[S_T] = (fps > 0.0) ? p.frameNumberOriginal / fps : 0.0;
    if (req.contains(S_QUALITY)) slotValues[S_QUALITY] = static_cast<double>(static_cast<int>(p.quality));
    if (req.contains(S_AREA)) slotValues[S_AREA] = p.area;
    if (req.contains(S_AREA_UM2)) slotValues[S_AREA_UM2] = p.area * um * um;
    if (req.contains(S_BODY_LENGTH)) slotValues[S_BODY_LENGTH] = p.bodyLength;
    if (req.contains(S_BODY_LENGTH_UM)) slotValues[S_BODY_LENGTH_UM] = p.bodyLength * um;
    if (req.contains(S_ASPECT_RATIO)) slotValues[S_ASPECT_RATIO] = p.aspectRatio;

    if (p.hasTips) {
        if (req.contains(S_XHEAD)) slotValues[S_XHEAD] = p.headTip.x;
        if (req.contains(S_YHEAD)) slotValues[S_YHEAD] = p.headTip.y;
        if (req.contains(S_XTAIL)) slotValues[S_XTAIL] = p.tailTip.x;
        if (req.contains(S_YTAIL)) slotValues[S_YTAIL] = p.tailTip.y;
        if (req.contains(S_XHEAD_UM)) slotValues[S_XHEAD_UM] = p.headTip.x * um;
        if (req.contains(S_YHEAD_UM)) slotValues[S_YHEAD_UM] = p.headTip.y * um;
        if (req.contains(S_XTAIL_UM)) slotValues[S_XTAIL_UM] = p.tailTip.x * um;
        if (req.contains(S_YTAIL_UM)) slotValues[S_YTAIL_UM] = p.tailTip.y * um;
    } else {
        if (req.contains(S_XHEAD)) slotValues[S_XHEAD] = 0.0;
        if (req.contains(S_YHEAD)) slotValues[S_YHEAD] = 0.0;
        if (req.contains(S_XTAIL)) slotValues[S_XTAIL] = 0.0;
        if (req.contains(S_YTAIL)) slotValues[S_YTAIL] = 0.0;
        if (req.contains(S_XHEAD_UM)) slotValues[S_XHEAD_UM] = 0.0;
        if (req.contains(S_YHEAD_UM)) slotValues[S_YHEAD_UM] = 0.0;
        if (req.contains(S_XTAIL_UM)) slotValues[S_XTAIL_UM] = 0.0;
        if (req.contains(S_YTAIL_UM)) slotValues[S_YTAIL_UM] = 0.0;
    }

    const bool needsStartDist = req.contains(S_DIST_TO_START) ||
                                req.contains(S_DIST_TO_START_PX) ||
                                req.contains(S_DIST_TO_START_UM);
    if (needsStartDist && roi.hasStart) {
        const double dx = x - roi.startX;
        const double dy = y - roi.startY;
        const double distPx = std::sqrt(dx * dx + dy * dy);
        const double distUm = (um > 0.0) ? distPx * um : distPx;
        slotValues[S_DIST_TO_START_PX] = distPx;
        slotValues[S_DIST_TO_START_UM] = distUm;
        slotValues[S_DIST_TO_START] = (um > 0.0) ? distUm : distPx;
    }

    const bool needsEndDist = req.contains(S_DIST_TO_END) ||
                              req.contains(S_DIST_TO_END_PX) ||
                              req.contains(S_DIST_TO_END_UM);
    if (needsEndDist && roi.hasEnd) {
        const double dx = x - roi.endX;
        const double dy = y - roi.endY;
        const double distPx = std::sqrt(dx * dx + dy * dy);
        const double distUm = (um > 0.0) ? distPx * um : distPx;
        slotValues[S_DIST_TO_END_PX] = distPx;
        slotValues[S_DIST_TO_END_UM] = distUm;
        slotValues[S_DIST_TO_END] = (um > 0.0) ? distUm : distPx;
    }

    const bool needsCenterDist = req.contains(S_DIST_TO_CENTER) ||
                                 req.contains(S_DIST_TO_CENTER_PX) ||
                                 req.contains(S_DIST_TO_CENTER_UM);
    if (needsCenterDist && roi.hasCenter) {
        const double dx = x - roi.centerX;
        const double dy = y - roi.centerY;
        const double distPx = std::sqrt(dx * dx + dy * dy);
        const double distUm = (um > 0.0) ? distPx * um : distPx;
        slotValues[S_DIST_TO_CENTER_PX] = distPx;
        slotValues[S_DIST_TO_CENTER_UM] = distUm;
        slotValues[S_DIST_TO_CENTER] = (um > 0.0) ? distUm : distPx;
    }

    const bool needsSpeed = req.contains(S_SPEED) ||
                            req.contains(S_SPEED_PX) ||
                            req.contains(S_SPEED_UM);
    if (!needsSpeed) return;

    if (p.quality == Tracking::TrackPointQuality::Lost) {
        speedState.reset();
        slotValues[S_SPEED] = 0.0;
        slotValues[S_SPEED_PX] = 0.0;
        slotValues[S_SPEED_UM] = 0.0;
        return;
    }

    if (!speedState.hasPrev || fps <= 0.0) {
        speedState.hasPrev = true;
        speedState.prevPos = p.position;
        speedState.prevFrame = p.frameNumberOriginal;
        slotValues[S_SPEED] = 0.0;
        slotValues[S_SPEED_PX] = 0.0;
        slotValues[S_SPEED_UM] = 0.0;
        return;
    }

    const int df = p.frameNumberOriginal - speedState.prevFrame;
    if (df > 0) {
        const double dx = static_cast<double>(p.position.x - speedState.prevPos.x);
        const double dy = static_cast<double>(p.position.y - speedState.prevPos.y);
        const double distPx = std::sqrt(dx * dx + dy * dy);
        const double dt = static_cast<double>(df) / fps;
        const double speedPx = dt > 0.0 ? distPx / dt : 0.0;
        const double speedUm = (um > 0.0) ? speedPx * um : speedPx;
        const int speedWindow = std::max(1, static_cast<int>(std::round(2.0 * fps)));

        speedState.winPx.emplace_back(p.frameNumberOriginal, speedPx);
        speedState.sumPx += speedPx;
        speedState.winUm.emplace_back(p.frameNumberOriginal, speedUm);
        speedState.sumUm += speedUm;
        while (!speedState.winPx.empty()
               && (p.frameNumberOriginal - speedState.winPx.front().first) > speedWindow) {
            speedState.sumPx -= speedState.winPx.front().second;
            speedState.winPx.pop_front();
        }
        while (!speedState.winUm.empty()
               && (p.frameNumberOriginal - speedState.winUm.front().first) > speedWindow) {
            speedState.sumUm -= speedState.winUm.front().second;
            speedState.winUm.pop_front();
        }

        const double smoothedPx = speedState.winPx.empty()
            ? speedPx : speedState.sumPx / static_cast<double>(speedState.winPx.size());
        const double smoothedUm = speedState.winUm.empty()
            ? speedUm : speedState.sumUm / static_cast<double>(speedState.winUm.size());
        slotValues[S_SPEED] = (um > 0.0) ? smoothedUm : smoothedPx;
        slotValues[S_SPEED_PX] = smoothedPx;
        slotValues[S_SPEED_UM] = smoothedUm;
    } else {
        slotValues[S_SPEED] = 0.0;
        slotValues[S_SPEED_PX] = 0.0;
        slotValues[S_SPEED_UM] = 0.0;
    }

    speedState.prevPos = p.position;
    speedState.prevFrame = p.frameNumberOriginal;
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

    QString planError;
    const std::shared_ptr<CompiledPluginPlan> plan = cachedPluginPlan(spec, planError);
    if (!plan) {
        result.errorMessage = planError;
        return result;
    }

    // µm flag
    bool allHaveUm = true;
    for (const auto& group : data)
        for (const auto& worm : group.worms)
            if (worm.umPerPixel <= 0) { allHaveUm = false; break; }
    result.usedUm = allHaveUm;

    auto runWormLoop = [&](const AnalysisSessionModel::AnalysisWormEntry& worm,
                           std::function<bool(const QVector<double>&, const Tracking::WormTrackPoint&)> frameCallback) -> bool
    {
        const PluginRoiPoints roi = effectiveRoiForWorm(worm, roiPoints);
        QVector<double> slotValues(plan->slotCount, 0.0);
        QVector<double> prevSlots(plan->slotCount, 0.0);
        QVector<double> prevBindingVals(plan->bindings.size(), kNaN);
        initSlotValues(slotValues, *plan, roi, worm.fps);
        initSlotValues(prevSlots, *plan, roi, worm.fps);

        bool hasPrevFrame = false;
        SpeedSlotState speedState;
        QVector<SmoothSlotState> smoothStates(plan->bindings.size());

        for (const Tracking::WormTrackPoint& point : worm.points) {
            updateSlotValues(slotValues, *plan, worm, point, roi, speedState);

            for (int bi = 0; bi < plan->bindings.size(); ++bi) {
                const PlanBinding& binding = plan->bindings[bi];
                if (binding.prevSlot >= 0) slotValues[binding.prevSlot] = prevBindingVals[bi];
            }

            bool hasNaN = false;
            for (int bi = 0; bi < plan->bindings.size(); ++bi) {
                const PlanBinding& binding = plan->bindings[bi];
                double value = 0.0;
                if (binding.isDiff) {
                    if (binding.isDiffT) {
                        if (!hasPrevFrame) {
                            slotValues[binding.slot] = kNaN;
                            hasNaN = true;
                            continue;
                        }
                        value = slotValues[S_T] - prevSlots[S_T];
                    } else if (!hasPrevFrame) {
                        slotValues[binding.slot] = kNaN;
                        hasNaN = true;
                        continue;
                    } else {
                        value = binding.innerFn(slotValues) - binding.innerFn(prevSlots);
                    }
                } else if (binding.isSmooth) {
                    const bool acceptSample = !binding.smoothFilterFn
                                           || binding.smoothFilterFn(slotValues) != 0.0;
                    const double coord = worm.fps > 0.0
                        ? slotValues[S_T]
                        : slotValues[S_FRAME];
                    const double rawValue = acceptSample
                        ? binding.smoothValueFn(slotValues)
                        : kNaN;
                    value = updateSmoothSlot(smoothStates[bi],
                                             rawValue,
                                             binding.smoothWindowFn(slotValues),
                                             coord,
                                             acceptSample);
                } else {
                    value = binding.plainFn(slotValues);
                }
                slotValues[binding.slot] = value;
                if (std::isnan(value)) {
                    hasNaN = true;
                }
            }

            auto capturePreviousState = [&]() {
                prevSlots = slotValues;
                for (int bi = 0; bi < plan->bindings.size(); ++bi)
                    prevBindingVals[bi] = slotValues[plan->bindings[bi].slot];
                hasPrevFrame = true;
            };

            if (hasNaN) {
                capturePreviousState();
                continue;
            }

            if (plan->filterFn && plan->filterFn(slotValues) == 0.0) {
                capturePreviousState();
                continue;
            }

            if (!frameCallback(slotValues, point)) return false;

            capturePreviousState();
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
                const bool ok = runWormLoop(worm, [&](const QVector<double>& slotValues, const Tracking::WormTrackPoint&) -> bool {
                    const double val = plan->formulaFn(slotValues);
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
                const bool ok = runWormLoop(worm, [&](const QVector<double>& slotValues, const Tracking::WormTrackPoint& point) -> bool {
                    const double val = plan->formulaFn(slotValues);
                    if (!std::isnan(val) && std::isfinite(val))
                        ws.points.append(QPointF(worm.fps > 0.0
                                                 ? static_cast<double>(point.frameNumberOriginal) / worm.fps
                                                 : static_cast<double>(point.frameNumberOriginal),
                                                 val));
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
