#pragma once

#include <QString>
#include <QMap>

/**
 * ExprEval — simple recursive-descent expression evaluator.
 *
 * Supports:
 *   - Double-precision arithmetic: + - * / ^ (right-associative power)
 *   - Unary minus
 *   - Parentheses
 *   - Numeric literals (integer and floating-point)
 *   - Named variables (looked up via the supplied variable map)
 *   - Built-in functions: sqrt abs floor ceil sin cos tan log exp min max
 *
 * Variables and function names are case-sensitive.
 * On error (unknown variable, division by zero, etc.), returns 0.0 and sets
 * the `error` out-parameter to a non-empty string.
 *
 * Usage:
 *   QMap<QString,double> vars{{"x", 3.0}, {"y", 4.0}};
 *   QString err;
 *   double result = ExprEval::evaluate("sqrt(x*x + y*y)", vars, &err);
 *   // result == 5.0, err is empty
 */
class ExprEval
{
public:
    ExprEval() = delete;

    /**
     * Evaluate `expr` with the given variable bindings.
     * @param error  If non-null, set to a description of any error (empty on success).
     * @return Numeric result, or 0.0 on error.
     */
    static double evaluate(const QString& expr,
                           const QMap<QString, double>& vars,
                           QString* error = nullptr);
};
