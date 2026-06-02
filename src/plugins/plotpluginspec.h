#pragma once

#include <QString>
#include <QStringList>
#include <QHash>

/**
 * PlotPluginSpec — in-memory representation of a user-defined plot plugin.
 *
 * A plugin is loaded from a JSON file and describes:
 *  - How to compute a derived metric from per-frame worm track data (formula + bindings)
 *  - How to aggregate that metric across frames and worms
 *  - How to display the result as a plot
 *
 * File format: JSON, typically stored in a per-project or user plugins/ directory.
 * See PluginLoader for the schema details and validation.
 */
struct PlotPluginSpec
{
    // ── Identity ───────────────────────────────────────────────────────────────
    QString filePath;           // Absolute path to the source .json file
    int     version = 1;
    QString name;
    QString description;

    // ── Computation ───────────────────────────────────────────────────────────

    /**
     * Aggregation level:
     *   per_worm  — compute a scalar per worm (reduce across frames), then group
     *   per_frame — keep a value per frame per worm (time series)
     *   spatial   — bin values into a 2D grid (heatmap)
     */
    enum class Aggregate { PerWorm, PerFrame, Spatial };
    Aggregate aggregate = Aggregate::PerWorm;

    /**
     * Math expression evaluated once per frame (or frame-pair for diff bindings).
     * Variables referenced here must either be standard vocabulary terms or
     * keys defined in `bindings`.
     */
    QString formula;

    /**
     * Named intermediate values computed before the main formula.
     * Values starting with "diff(" trigger a consecutive-frame difference:
     *   "diff(x)"  → x[frame] - x[frame-1]   (skips on first frame)
     *   "diff(t)"  → 1.0 / fps
     * Other values are plain expressions evaluated from standard vocabulary.
     */
    QHash<QString, QString> bindings;

    /**
     * Optional filter expression. Frames where this evaluates to 0 are excluded.
     * Standard vocabulary is available. Example: "quality == 1"  (1 = Single)
     */
    QString filter;

    /**
     * How frame-level values are reduced to a per-worm scalar (per_worm mode only).
     * Supported: mean, median, sum, count, min, max, std, last
     */
    QString reduce = "mean";

    // ── Visualisation ─────────────────────────────────────────────────────────

    /**
     * Plot type:
     *   box     — box-and-whisker, one box per group
     *   bar     — mean ± std, one bar per group
     *   line    — time series, one line per worm
     *   scatter — 2D scatter (requires formula_x and formula_y; see extensions)
     */
    enum class PlotType { Box, Bar, Line, Scatter };
    PlotType plotType = PlotType::Box;

    QString yLabel;             // Y-axis label (pixel units)
    QString yLabelUm;           // Y-axis label when µm/pixel is available
    QString xLabel;             // X-axis label (used for line/scatter)

    // ── Validity ──────────────────────────────────────────────────────────────
    bool isValid = false;       // Set true by PluginLoader on successful parse
    QStringList errors;         // Validation errors (non-empty when isValid == false)
};
