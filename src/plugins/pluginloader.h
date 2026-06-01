#pragma once

#include "plotpluginspec.h"
#include <QStringList>

/**
 * PluginLoader — reads plot plugin JSON files into PlotPluginSpec objects.
 *
 * JSON schema (all fields except name and formula are optional):
 * {
 *   "version": 1,
 *   "name": "Speed",
 *   "description": "Mean speed per worm per group",
 *   "aggregate": "per_worm",          // per_worm | per_frame | spatial
 *   "formula": "sqrt(dx*dx + dy*dy) / dt",
 *   "bindings": {
 *     "dx": "diff(x)",
 *     "dy": "diff(y)",
 *     "dt": "diff(t)"
 *   },
 *   "filter": "quality == 1",         // 1 = Single
 *   "reduce": "mean",                 // mean|median|sum|count|min|max|std|last
 *   "plot": {
 *     "type": "box",                  // box|bar|line|scatter
 *     "y_label": "Speed (px/s)",
 *     "y_label_um": "Speed (µm/s)",
 *     "x_label": "Group"
 *   }
 * }
 *
 * Standard vocabulary available in formula/filter/bindings:
 *   x, y, x_um, y_um                    — centroid position
 *   xhead, yhead, xtail, ytail           — tip positions
 *   xhead_um, yhead_um, xtail_um, ytail_um
 *   frame, t                             — frame number, time in seconds
 *   area, area_um2                       — blob area
 *   body_length, body_length_um          — centerline arc length
 *   aspect_ratio                         — bounding box long/short
 *   quality                              — 0=Single 1=Merged 2=Split 3=Lost
 *   start_x, start_y, end_x, end_y,
 *   center_x, center_y                   — ROI reference points (if set)
 *   dist_to_start, dist_to_end,
 *   dist_to_center                       — distances to ROI reference points
 */
class PluginLoader
{
public:
    PluginLoader() = delete;

    /** Load a single plugin file. Result has isValid == false on error. */
    static PlotPluginSpec load(const QString& filePath);

    /**
     * Scan a directory for *.json files and attempt to load each as a plugin.
     * Invalid files are included in the result with isValid == false.
     */
    static QList<PlotPluginSpec> loadDirectory(const QString& dirPath);

    /**
     * Load all plugins from multiple search directories.
     * Plugins found earlier in the list take precedence when two files share
     * the same name (later duplicates are skipped).
     */
    static QList<PlotPluginSpec> loadAll(const QStringList& searchDirs);
};
