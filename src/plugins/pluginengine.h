#pragma once

#include "plotpluginspec.h"
#include "../gui/analysissessionmodel.h"

#include <QString>
#include <QList>
#include <QMap>
#include <QPointF>

/** Optional ROI reference points for use in plugin expressions (start_x, dist_to_end, etc.). */
struct PluginRoiPoints {
    bool    hasStart  = false;
    double  startX    = 0, startY = 0;
    bool    hasEnd    = false;
    double  endX      = 0, endY   = 0;
    bool    hasCenter = false;
    double  centerX   = 0, centerY = 0;
};

/**
 * PluginEngine — evaluates a PlotPluginSpec against analysis data.
 *
 * Input:  a validated PlotPluginSpec + the grouped worm data from AnalysisSessionModel
 * Output: PluginResult, which carries the computed values ready to paint
 *
 * The engine resolves the standard vocabulary (x, y, t, area, …), evaluates
 * bindings, applies the formula and filter, reduces per-worm, and organises
 * everything by group for plotting.
 *
 * ROI reference points (start_x etc.) are optional — pass a default-constructed
 * PluginRoiPoints{} when they are not available.
 */
class PluginEngine
{
public:
    PluginEngine() = delete;

    // ── Output types ─────────────────────────────────────────────────────────

    /** One worm's contribution to a per_worm plot. */
    struct WormScalar {
        int     wormId;
        QString label;
        double  value;
        QColor  color;
    };

    /** One group's data for a grouped plot (box/bar). */
    struct GroupResult {
        QString           name;
        QList<WormScalar> worms;   // one entry per checked worm in this group
    };

    /** One worm's time series for a per_frame plot. */
    struct WormSeries {
        int            wormId;
        QString        label;
        QColor         color;
        QString        groupName;
        QList<QPointF> points;    // (t or frame, value)
    };

    /** Aggregated output from a plugin evaluation. */
    struct PluginResult {
        bool    ok = false;
        QString errorMessage;

        // Populated for per_worm aggregate (box/bar plots)
        QList<GroupResult> groups;

        // Populated for per_frame aggregate (line plots)
        QList<WormSeries> series;

        // Whether µm values were used (umPerPixel > 0 for all involved worms)
        bool usedUm = false;
    };

    /**
     * Evaluate the plugin against the provided group data.
     * @param spec       Validated plugin specification.
     * @param data       Grouped worm data (from AnalysisSessionModel::getGroupedData()).
     * @param roiPoints  Optional ROI reference points for the current project.
     */
    static PluginResult evaluate(const PlotPluginSpec& spec,
                                 const QList<AnalysisSessionModel::AnalysisGroupData>& data,
                                 const PluginRoiPoints& roiPoints = PluginRoiPoints{});

private:
    // Build the variable map for a single track point
    static QMap<QString, double> buildVars(
        const AnalysisSessionModel::AnalysisWormEntry& worm,
        int pointIdx,
        const PluginRoiPoints& roi,
        double umPerPixel,
        double fps);

    // Apply bindings on top of a base variable map
    static bool applyBindings(QMap<QString, double>& vars,
                              const QMap<QString, double>& prevVars,
                              const QMap<QString, QString>& bindings,
                              double fps,
                              QString& error);

    // Reduce a vector of values to a scalar
    static double reduce(const QVector<double>& values, const QString& method);
};
