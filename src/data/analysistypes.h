#pragma once

#include "trackingcommon.h"   // Tracking::WormTrackPoint

#include <QColor>
#include <QList>
#include <QPointF>
#include <QString>
#include <vector>

/**
 * Plain data handed from the Analysis tab's session model to the plot widgets
 * and the plugin engine. Lives in src/data so that src/plugins does not depend
 * on a GUI model class.
 *
 * Produced by AnalysisSessionModel::getGroupedData(); consumed by
 * PluginEngine::evaluate() and the analysis group widgets.
 */

/** One worm's complete data as seen by the analysis plots. */
struct AnalysisWormEntry {
    int     wormId = 0;
    QString label;         // "Worm 1" …
    QColor  color;         // group colormap color
    double  umPerPixel = 0.0;  // from the video (run) this worm belongs to; 0 if unknown
    double  fps = 0.0;         // from the video (run) this worm belongs to; 0 if unknown
    QString videoBaseName;
    std::vector<Tracking::WormTrackPoint> points;  // sorted by frameNumber

    // Reference points of the run this worm belongs to (video coordinates).
    bool hasStartPoint = false;
    QPointF startPoint;
    bool hasEndPoint = false;
    QPointF endPoint;
    bool hasCenterPoint = false;
    QPointF centerPoint;
};

/** All checked worms that belong to one analysis group. */
struct AnalysisGroupData {
    QString name;
    QList<AnalysisWormEntry> worms;
};
