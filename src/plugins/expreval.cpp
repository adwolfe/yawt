#include "expreval.h"

#include <cmath>
#include <cctype>
#include <limits>

// ── Internal parser state ──────────────────────────────────────────────────

struct ParseState {
    const QChar* src;
    int          pos;
    int          len;
    const QMap<QString, double>* vars;
    QString      error;

    QChar cur() const { return (pos < len) ? src[pos] : QChar(0); }
    void skip() { while (pos < len && src[pos].isSpace()) ++pos; }
    void advance() { ++pos; skip(); }
};

static double parseExpr(ParseState& s);  // forward

static void setError(ParseState& s, const QString& msg)
{
    if (s.error.isEmpty()) s.error = msg;
}

// ── Number literal ─────────────────────────────────────────────────────────

static double parseNumber(ParseState& s)
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
    double v = tok.toDouble(&ok);
    if (!ok) setError(s, QString("Invalid number: %1").arg(tok));
    return v;
}

// ── Identifier (variable or function name) ────────────────────────────────

static QString parseIdent(ParseState& s)
{
    int start = s.pos;
    while (s.pos < s.len && (s.src[s.pos].isLetterOrNumber() || s.src[s.pos] == '_')) ++s.pos;
    const QString id = QString(s.src + start, s.pos - start);
    s.skip();
    return id;
}

// ── Built-in functions ────────────────────────────────────────────────────

static double callBuiltin(const QString& name, const QVector<double>& args, ParseState& s)
{
    // Two-arg functions
    if (name == "min" || name == "max") {
        if (args.size() != 2) { setError(s, name + "() requires 2 arguments"); return 0.0; }
        return (name == "min") ? std::min(args[0], args[1]) : std::max(args[0], args[1]);
    }
    if (name == "pow") {
        if (args.size() != 2) { setError(s, "pow() requires 2 arguments"); return 0.0; }
        return std::pow(args[0], args[1]);
    }
    // One-arg functions
    if (args.size() != 1) { setError(s, name + "() requires 1 argument"); return 0.0; }
    const double v = args[0];
    if (name == "sqrt") {
        if (v < 0) { setError(s, "sqrt of negative"); return 0.0; }
        return std::sqrt(v);
    }
    if (name == "abs")  return std::abs(v);
    if (name == "floor") return std::floor(v);
    if (name == "ceil")  return std::ceil(v);
    if (name == "sin")   return std::sin(v);
    if (name == "cos")   return std::cos(v);
    if (name == "tan")   return std::tan(v);
    if (name == "log") {
        if (v <= 0) { setError(s, "log of non-positive"); return 0.0; }
        return std::log(v);
    }
    if (name == "exp")   return std::exp(v);
    setError(s, QString("Unknown function: %1").arg(name));
    return 0.0;
}

// ── Primary: number | identifier | '(' expr ')' | unary minus ─────────────

static double parsePrimary(ParseState& s)
{
    s.skip();
    if (!s.error.isEmpty()) return 0.0;

    // Unary minus
    if (s.cur() == '-') {
        s.advance();
        return -parsePrimary(s);
    }
    // Unary plus
    if (s.cur() == '+') {
        s.advance();
        return parsePrimary(s);
    }

    // Parenthesised expression
    if (s.cur() == '(') {
        s.advance();
        double v = parseExpr(s);
        if (s.cur() != ')') { setError(s, "Expected ')'"); return 0.0; }
        s.advance();
        return v;
    }

    // Number literal
    if (s.cur().isDigit() || s.cur() == '.') {
        return parseNumber(s);
    }

    // Identifier: variable or function call
    if (s.cur().isLetter() || s.cur() == '_') {
        const QString id = parseIdent(s);
        if (!s.error.isEmpty()) return 0.0;

        // Function call
        if (s.cur() == '(') {
            s.advance();
            QVector<double> args;
            if (s.cur() != ')') {
                args.append(parseExpr(s));
                while (s.cur() == ',') {
                    s.advance();
                    args.append(parseExpr(s));
                }
            }
            if (s.cur() != ')') { setError(s, "Expected ')' after arguments"); return 0.0; }
            s.advance();
            return callBuiltin(id, args, s);
        }

        // Variable
        auto it = s.vars->constFind(id);
        if (it == s.vars->constEnd()) {
            setError(s, QString("Unknown variable: %1").arg(id));
            return 0.0;
        }
        return it.value();
    }

    setError(s, QString("Unexpected character: '%1'").arg(s.cur()));
    return 0.0;
}

// ── Power (right-associative) ─────────────────────────────────────────────

static double parsePower(ParseState& s)
{
    double base = parsePrimary(s);
    if (s.cur() == '^') {
        s.advance();
        double exp = parsePower(s);  // right-associative
        return std::pow(base, exp);
    }
    return base;
}

// ── Multiplicative ────────────────────────────────────────────────────────

static double parseMul(ParseState& s)
{
    double v = parsePower(s);
    while (!s.error.isEmpty() == false && (s.cur() == '*' || s.cur() == '/')) {
        const QChar op = s.cur();
        s.advance();
        double r = parsePower(s);
        if (op == '*') {
            v *= r;
        } else {
            if (r == 0.0) { setError(s, "Division by zero"); return 0.0; }
            v /= r;
        }
    }
    return v;
}

// ── Additive ──────────────────────────────────────────────────────────────

static double parseAdd(ParseState& s)
{
    double v = parseMul(s);
    while (!s.error.isEmpty() == false && (s.cur() == '+' || s.cur() == '-')) {
        const QChar op = s.cur();
        s.advance();
        double r = parseMul(s);
        v = (op == '+') ? v + r : v - r;
    }
    return v;
}

// ── Comparison (==, !=, <, <=, >, >=) ─────────────────────────────────────

static double parseCompare(ParseState& s)
{
    double v = parseAdd(s);
    if (s.error.isEmpty()) {
        const QChar c0 = s.cur();
        if (c0 == '=' || c0 == '!' || c0 == '<' || c0 == '>') {
            QString op;
            op += c0;
            ++s.pos;
            if (s.pos < s.len && s.src[s.pos] == '=') { op += '='; ++s.pos; }
            s.skip();
            double r = parseAdd(s);
            if      (op == "==") v = (v == r) ? 1.0 : 0.0;
            else if (op == "!=") v = (v != r) ? 1.0 : 0.0;
            else if (op == "<")  v = (v <  r) ? 1.0 : 0.0;
            else if (op == "<=") v = (v <= r) ? 1.0 : 0.0;
            else if (op == ">")  v = (v >  r) ? 1.0 : 0.0;
            else if (op == ">=") v = (v >= r) ? 1.0 : 0.0;
            else setError(s, QString("Unknown operator: %1").arg(op));
        }
    }
    return v;
}

// ── Top-level expression ──────────────────────────────────────────────────

static double parseExpr(ParseState& s)
{
    return parseCompare(s);
}

// ── Public API ─────────────────────────────────────────────────────────────

double ExprEval::evaluate(const QString& expr,
                          const QMap<QString, double>& vars,
                          QString* error)
{
    if (expr.trimmed().isEmpty()) {
        if (error) *error = "Empty expression";
        return 0.0;
    }

    ParseState s;
    s.src  = expr.constData();
    s.pos  = 0;
    s.len  = expr.length();
    s.vars = &vars;
    s.skip();

    const double result = parseExpr(s);

    // Check for trailing garbage
    if (s.error.isEmpty() && s.pos < s.len)
        s.error = QString("Unexpected trailing characters at position %1").arg(s.pos);

    if (error) *error = s.error;
    return s.error.isEmpty() ? result : 0.0;
}
