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
 *   "formula": "speed",
 *   "filter": "quality != Lost && speed > 0",
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
 *   speed, speed_px, speed_um            — 2-second smoothed speed
 *   quality                              — 0=Single 1=Merged 2=Split 3=Lost
 *   start_x, start_y, end_x, end_y,
 *   center_x, center_y                   — ROI reference points (if set)
 *   has_start, has_end, has_center        — 1 when the ROI point is available
 *   dist_to_start, dist_to_end,
 *   dist_to_center                       — distances to ROI reference points
 *   dist_to_start_px, dist_to_start_um,
 *   dist_to_end_px, dist_to_end_um,
 *   dist_to_center_px, dist_to_center_um — explicit distance units
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
