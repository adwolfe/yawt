#include "plotpainting.h"

#include <QFontMetrics>
#include <QPen>

namespace PlotPainting {

namespace {
const QColor kAxisColor(160, 160, 160);
}

void drawYAxis(QPainter& p, const QRectF& plotArea, const QRectF& fullArea,
               double yMin, double yMax, const QString& yLabel, int nTicks)
{
    if (yMax <= yMin || nTicks < 1) return;

    p.save();
    const int labelH = p.fontMetrics().height();
    p.setPen(QPen(kAxisColor, 1));
    for (int i = 0; i <= nTicks; ++i) {
        const double v = yMin + (yMax - yMin) * i / nTicks;
        const qreal y = plotArea.bottom()
                        - (v - yMin) / (yMax - yMin) * plotArea.height();
        p.drawLine(QPointF(plotArea.left() - 4, y), QPointF(plotArea.right(), y));
        p.drawText(QRectF(fullArea.left(), y - labelH / 2.0,
                          plotArea.left() - fullArea.left() - 6, labelH),
                   Qt::AlignRight | Qt::AlignVCenter, QString::number(v, 'g', 3));
    }
    if (!yLabel.isEmpty()) {
        p.translate(fullArea.left() + labelH, fullArea.top() + fullArea.height() / 2.0);
        p.rotate(-90);
        p.drawText(QRectF(-fullArea.height() / 2.0, -labelH, fullArea.height(), labelH),
                   Qt::AlignCenter, yLabel);
    }
    p.restore();
}

void drawAxisFrame(QPainter& p, const QRectF& plotArea)
{
    p.save();
    p.setPen(QPen(kAxisColor, 1));
    p.drawLine(plotArea.bottomLeft(), plotArea.topLeft());
    p.drawLine(plotArea.bottomLeft(), plotArea.bottomRight());
    p.restore();
}

} // namespace PlotPainting
