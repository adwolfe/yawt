#include "expreval.h"

#include <cmath>
#include <cctype>
#include <limits>

// ─────────────────────────────────────────────────────────────────────────────
// Internal compiler — builds a tree of CompiledExpr closures.
// Forward-declared so mutual recursion works.
// ─────────────────────────────────────────────────────────────────────────────

struct CompileState {
    const QChar* src;
    int          pos;
    int          len;
    QString      error;

    QChar cur() const { return (pos < len) ? src[pos] : QChar(0); }
    void skip() { while (pos < len && src[pos].isSpace()) ++pos; }
    void advance() { ++pos; skip(); }
};

static CompiledExpr compileExpr(CompileState& s);  // forward

static void setErr(CompileState& s, const QString& msg)
{
    if (s.error.isEmpty()) s.error = msg;
}

// ── Numeric literal ────────────────────────────────────────────────────────

static CompiledExpr compileLiteral(CompileState& s)
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
    if (!ok) { setErr(s, "Invalid number: " + tok); return nullptr; }
    return [v](const VarMap&) { return v; };
}

// ── Identifier — variable or function call ─────────────────────────────────

static QString parseIdent(CompileState& s)
{
    int start = s.pos;
    while (s.pos < s.len && (s.src[s.pos].isLetterOrNumber() || s.src[s.pos] == '_')) ++s.pos;
    const QString id = QString(s.src + start, s.pos - start);
    s.skip();
    return id;
}

// ── Built-in functions ─────────────────────────────────────────────────────

static CompiledExpr compileBuiltin(const QString& name,
                                   QVector<CompiledExpr> args,
                                   CompileState& s)
{
    // Two-arg
    if (name == "min" || name == "max" || name == "pow") {
        if (args.size() != 2) { setErr(s, name + "() requires 2 arguments"); return nullptr; }
        auto a = std::move(args[0]), b = std::move(args[1]);
        if (name == "min") return [a,b](const VarMap& v){ return std::min(a(v), b(v)); };
        if (name == "max") return [a,b](const VarMap& v){ return std::max(a(v), b(v)); };
        return [a,b](const VarMap& v){ return std::pow(a(v), b(v)); };
    }
    // One-arg
    if (args.size() != 1) { setErr(s, name + "() requires 1 argument"); return nullptr; }
    auto a = std::move(args[0]);
    if (name == "sqrt")  return [a](const VarMap& v){ double x=a(v); return x<0?0.0:std::sqrt(x); };
    if (name == "abs")   return [a](const VarMap& v){ return std::abs(a(v)); };
    if (name == "floor") return [a](const VarMap& v){ return std::floor(a(v)); };
    if (name == "ceil")  return [a](const VarMap& v){ return std::ceil(a(v)); };
    if (name == "sin")   return [a](const VarMap& v){ return std::sin(a(v)); };
    if (name == "cos")   return [a](const VarMap& v){ return std::cos(a(v)); };
    if (name == "tan")   return [a](const VarMap& v){ return std::tan(a(v)); };
    if (name == "log")   return [a](const VarMap& v){ double x=a(v); return x<=0?0.0:std::log(x); };
    if (name == "exp")   return [a](const VarMap& v){ return std::exp(a(v)); };
    setErr(s, "Unknown function: " + name);
    return nullptr;
}

// ── Primary ────────────────────────────────────────────────────────────────

static CompiledExpr compilePrimary(CompileState& s)
{
    s.skip();
    if (!s.error.isEmpty()) return nullptr;

    // Unary minus
    if (s.cur() == '-') {
        s.advance();
        auto inner = compilePrimary(s);
        if (!inner) return nullptr;
        return [inner](const VarMap& v){ return -inner(v); };
    }
    // Unary plus
    if (s.cur() == '+') { s.advance(); return compilePrimary(s); }

    // Parenthesised expression
    if (s.cur() == '(') {
        s.advance();
        auto inner = compileExpr(s);
        if (!inner) return nullptr;
        if (s.cur() != ')') { setErr(s, "Expected ')'"); return nullptr; }
        s.advance();
        return inner;
    }

    // Numeric literal
    if (s.cur().isDigit() || s.cur() == '.') return compileLiteral(s);

    // Identifier — variable or function call
    if (s.cur().isLetter() || s.cur() == '_') {
        const QString id = parseIdent(s);
        if (!s.error.isEmpty()) return nullptr;

        // Function call
        if (s.cur() == '(') {
            s.advance();
            QVector<CompiledExpr> args;
            if (s.cur() != ')') {
                args.append(compileExpr(s));
                if (!args.last()) return nullptr;
                while (s.cur() == ',') {
                    s.advance();
                    args.append(compileExpr(s));
                    if (!args.last()) return nullptr;
                }
            }
            if (s.cur() != ')') { setErr(s, "Expected ')'"); return nullptr; }
            s.advance();
            return compileBuiltin(id, std::move(args), s);
        }

        // Variable — capture the name in the closure, look it up at call time
        return [id](const VarMap& vars) -> double {
            auto it = vars.constFind(id);
            if (it == vars.constEnd()) return 0.0;  // unknown var → 0
            return it.value();
        };
    }

    setErr(s, QString("Unexpected character: '%1'").arg(s.cur()));
    return nullptr;
}

// ── Power (right-assoc) ───────────────────────────────────────────────────

static CompiledExpr compilePower(CompileState& s)
{
    auto base = compilePrimary(s);
    if (!base || !s.error.isEmpty()) return base;
    if (s.cur() == '^') {
        s.advance();
        auto exp = compilePower(s);  // right-associative
        if (!exp) return nullptr;
        return [base,exp](const VarMap& v){ return std::pow(base(v), exp(v)); };
    }
    return base;
}

// ── Multiplicative ────────────────────────────────────────────────────────

static CompiledExpr compileMul(CompileState& s)
{
    auto lhs = compilePower(s);
    if (!lhs) return nullptr;
    while (s.error.isEmpty() && (s.cur() == '*' || s.cur() == '/')) {
        const bool isDiv = (s.cur() == '/');
        s.advance();
        auto rhs = compilePower(s);
        if (!rhs) return nullptr;
        auto prev = std::move(lhs);
        if (isDiv)
            lhs = [prev,rhs](const VarMap& v){
                double r = rhs(v); return r == 0.0 ? 0.0 : prev(v) / r; };
        else
            lhs = [prev,rhs](const VarMap& v){ return prev(v) * rhs(v); };
    }
    return lhs;
}

// ── Additive ──────────────────────────────────────────────────────────────

static CompiledExpr compileAdd(CompileState& s)
{
    auto lhs = compileMul(s);
    if (!lhs) return nullptr;
    while (s.error.isEmpty() && (s.cur() == '+' || s.cur() == '-')) {
        const bool isSub = (s.cur() == '-');
        s.advance();
        auto rhs = compileMul(s);
        if (!rhs) return nullptr;
        auto prev = std::move(lhs);
        if (isSub)
            lhs = [prev,rhs](const VarMap& v){ return prev(v) - rhs(v); };
        else
            lhs = [prev,rhs](const VarMap& v){ return prev(v) + rhs(v); };
    }
    return lhs;
}

// ── Comparison ────────────────────────────────────────────────────────────

static CompiledExpr compileCompare(CompileState& s)
{
    auto lhs = compileAdd(s);
    if (!lhs || !s.error.isEmpty()) return lhs;
    const QChar c0 = s.cur();
    if (c0 == '=' || c0 == '!' || c0 == '<' || c0 == '>') {
        QString op; op += c0; ++s.pos;
        if (s.pos < s.len && s.src[s.pos] == '=') { op += '='; ++s.pos; }
        s.skip();
        auto rhs = compileAdd(s);
        if (!rhs) return nullptr;
        if      (op == "==") return [lhs,rhs](const VarMap& v){ return lhs(v)==rhs(v)?1.0:0.0; };
        else if (op == "!=") return [lhs,rhs](const VarMap& v){ return lhs(v)!=rhs(v)?1.0:0.0; };
        else if (op == "<")  return [lhs,rhs](const VarMap& v){ return lhs(v)< rhs(v)?1.0:0.0; };
        else if (op == "<=") return [lhs,rhs](const VarMap& v){ return lhs(v)<=rhs(v)?1.0:0.0; };
        else if (op == ">")  return [lhs,rhs](const VarMap& v){ return lhs(v)> rhs(v)?1.0:0.0; };
        else if (op == ">=") return [lhs,rhs](const VarMap& v){ return lhs(v)>=rhs(v)?1.0:0.0; };
        setErr(s, "Unknown operator: " + op);
    }
    return lhs;
}

// ── Logical AND ────────────────────────────────────────────────────────────

static CompiledExpr compileAnd(CompileState& s)
{
    auto lhs = compileCompare(s);
    if (!lhs) return nullptr;
    while (s.error.isEmpty() && s.cur() == '&' && s.pos+1 < s.len && s.src[s.pos+1] == '&') {
        s.pos += 2; s.skip();
        auto rhs = compileCompare(s);
        if (!rhs) return nullptr;
        auto prev = std::move(lhs);
        lhs = [prev,rhs](const VarMap& v){ return (prev(v)!=0.0 && rhs(v)!=0.0)?1.0:0.0; };
    }
    return lhs;
}

// ── Logical OR ─────────────────────────────────────────────────────────────

static CompiledExpr compileOr(CompileState& s)
{
    auto lhs = compileAnd(s);
    if (!lhs) return nullptr;
    while (s.error.isEmpty() && s.cur() == '|' && s.pos+1 < s.len && s.src[s.pos+1] == '|') {
        s.pos += 2; s.skip();
        auto rhs = compileAnd(s);
        if (!rhs) return nullptr;
        auto prev = std::move(lhs);
        lhs = [prev,rhs](const VarMap& v){ return (prev(v)!=0.0 || rhs(v)!=0.0)?1.0:0.0; };
    }
    return lhs;
}

// ── Ternary ───────────────────────────────────────────────────────────────

static CompiledExpr compileTernary(CompileState& s)
{
    auto cond = compileOr(s);
    if (!cond || !s.error.isEmpty()) return cond;
    if (s.cur() == '?') {
        s.advance();
        auto thenE = compileOr(s);
        if (!thenE) return nullptr;
        if (s.cur() != ':') { setErr(s, "Expected ':'"); return nullptr; }
        s.advance();
        auto elseE = compileOr(s);
        if (!elseE) return nullptr;
        return [cond,thenE,elseE](const VarMap& v){
            return cond(v) != 0.0 ? thenE(v) : elseE(v); };
    }
    return cond;
}

// ── Top-level ─────────────────────────────────────────────────────────────

static CompiledExpr compileExpr(CompileState& s) { return compileTernary(s); }

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

CompiledExpr ExprEval::compile(const QString& expr, QString* error)
{
    if (expr.trimmed().isEmpty()) {
        if (error) *error = "Empty expression";
        return nullptr;
    }
    CompileState s;
    s.src = expr.constData();
    s.pos = 0;
    s.len = expr.length();
    s.skip();

    CompiledExpr fn = compileExpr(s);

    if (s.error.isEmpty() && s.pos < s.len)
        s.error = QString("Unexpected trailing text at position %1").arg(s.pos);

    if (!s.error.isEmpty()) {
        if (error) *error = s.error;
        return nullptr;
    }
    return fn;
}

double ExprEval::evaluate(const QString& expr, const VarMap& vars, QString* error)
{
    QString err;
    CompiledExpr fn = compile(expr, &err);
    if (!fn) { if (error) *error = err; return 0.0; }
    return fn(vars);
}
