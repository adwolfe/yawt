#pragma once

#include <QString>
#include <QHash>
#include <functional>
#include <memory>

/**
 * ExprEval — recursive-descent expression evaluator with optional compilation.
 *
 * Two usage modes:
 *
 * 1. Interpret (parse + evaluate each call) — simple but slow for hot loops:
 *      double result = ExprEval::evaluate("sqrt(x*x + y*y)", vars, &err);
 *
 * 2. Compile once, call many times — eliminates per-call string parsing:
 *      auto fn = ExprEval::compile("sqrt(x*x + y*y)", &err);
 *      if (fn) {
 *          for (auto& frame : frames)
 *              double v = fn(frameVars);
 *      }
 *
 * The compiled callable is a std::function that holds a closure tree
 * mirroring the parse tree. Variable lookup still goes through the QHash
 * passed at call time, so variable values can change between calls.
 *
 * Supported:
 *   - Double-precision arithmetic: + - * / ^ (right-associative power)
 *   - Unary minus / plus
 *   - Parentheses
 *   - Ternary: cond ? then : else
 *   - Logical: && ||
 *   - Comparisons: == != < <= > >=
 *   - Numeric literals (integer and float)
 *   - Named variables (looked up in the QHash at call time)
 *   - Built-in functions: sqrt abs floor ceil sin cos tan log exp min max pow
 */

using VarMap = QHash<QString, double>;
using CompiledExpr = std::function<double(const VarMap&)>;

class ExprEval
{
public:
    ExprEval() = delete;

    /**
     * Interpret: parse and evaluate in one shot.
     * Use only for one-off evaluations or error checking.
     * For hot paths, use compile() instead.
     */
    static double evaluate(const QString& expr,
                           const VarMap& vars,
                           QString* error = nullptr);

    /**
     * Compile: parse the expression string into a reusable callable.
     * Returns nullptr (empty function) on parse error; sets *error if provided.
     * The resulting function throws no exceptions and returns 0.0 on runtime
     * errors (division by zero, unknown variable, etc.).
     */
    static CompiledExpr compile(const QString& expr,
                                QString* error = nullptr);
};
