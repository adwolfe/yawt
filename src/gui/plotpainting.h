#ifndef PLOTPAINTING_H
#define PLOTPAINTING_H

#include <QPainter>
#include <QRectF>
#include <QString>

/**
 * Shared low-level painting helpers for the hand-drawn plot widgets
 * (PluginPlotWidget and the Group* widgets in analysisgroupwidgets.cpp),
 * so axis styling stays consistent across the Analysis tab.
 */
namespace PlotPainting {

    /**
     * @brief Draw horizontal gridlines, tick value labels, and an optional
     *        rotated Y-axis label.
     *
     * Gridlines span the plot area; tick labels are right-aligned in the strip
     * between @p fullArea.left() and @p plotArea.left(). The rotated axis label
     * (if non-empty) is centered vertically along @p fullArea's left edge.
     * Painter state is saved/restored — no font or pen side effects.
     */
    void drawYAxis(QPainter& p, const QRectF& plotArea, const QRectF& fullArea,
                   double yMin, double yMax, const QString& yLabel,
                   int nTicks = 5);

    /**
     * @brief Draw the left and bottom axis frame lines around the plot area.
     */
    void drawAxisFrame(QPainter& p, const QRectF& plotArea);

} // namespace PlotPainting

#endif // PLOTPAINTING_H
