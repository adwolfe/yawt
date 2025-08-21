#include "mergeviewer.h"

#include <QPainter>
#include <QPaintEvent>
#include <QFontMetrics>
#include <QLinearGradient>
#include <QDebug>
#include <algorithm>

MergeViewer::MergeViewer(QWidget* parent)
    : QWidget(parent)
{
    // Prefer a fixed vertical size but allow horizontal expansion
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    setMinimumHeight(60);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
}

void MergeViewer::setVisibleWormColors(const QMap<int, QColor>& wormColors)
{
    m_visibleWormColors = wormColors;
    {
        // Avoid creating iterators into a temporary QList returned by keys()
        QList<int> keys = m_visibleWormColors.keys();
        m_visibleIdsCache = QSet<int>(keys.begin(), keys.end());
    }
    update();
}

void MergeViewer::setVisibleWormIds(const QSet<int>& ids)
{
    m_visibleIdsCache = ids;
    // Ensure colors map includes entries for each id? We keep the existing colors if present.
    // Missing ids will render with fallback color in colorForId().
    update();
}

void MergeViewer::setVisibleByFrame(const QMap<int, QSet<int>>& visibleByFrame)
{
    // Provide an explicit per-frame visibility map so bars can be drawn partially per-segment.
    m_visibleByFrame = visibleByFrame;

    // Build the union of all IDs present across all frames so the viewer's labels and color map
    // reflect every worm that appears anywhere in the supplied per-frame map.
    QSet<int> unionIds;
    for (auto it = m_visibleByFrame.constBegin(); it != m_visibleByFrame.constEnd(); ++it) {
        unionIds.unite(it.value());
    }

    // Cache the unioned IDs for rendering (used when m_visibleWormColors is empty or incomplete).
    m_visibleIdsCache = unionIds;

    // Ensure there's at least a fallback color entry for each visible id so colorForId() and
    // label rendering have consistent behavior. Do not override existing explicit colors.
    for (int id : unionIds) {
        if (!m_visibleWormColors.contains(id)) {
            m_visibleWormColors.insert(id, m_defaultColor);
        }
    }

    // If the supplied map contains consecutive frames, derive the center frame from its keys so the
    // viewer aligns segments correctly even if the global videoLoader current frame changes concurrently.
    if (!m_visibleByFrame.isEmpty()) {
        QList<int> keys = m_visibleByFrame.keys();
        std::sort(keys.begin(), keys.end());
        int mid = keys.size() / 2;
        m_currentFrame = keys[mid];
    }

    updateGeometry();
    update();
}

void MergeViewer::setCurrentFrame(int frameNumber)
{
    if (m_currentFrame == frameNumber) return;
    m_currentFrame = frameNumber;
    update();
}

void MergeViewer::setRadius(int radius)
{
    if (radius < 1) radius = 1;
    if (m_radius == radius) return;
    m_radius = radius;
    update();
}

void MergeViewer::setBarSpacing(int px)
{
    if (px < 0) px = 0;
    m_barSpacing = px;
    update();
}

void MergeViewer::setBarHeight(int px)
{
    if (px < 4) px = 4;
    m_barHeight = px;
    updateGeometry();
    update();
}

void MergeViewer::setLeftLabelWidth(int px)
{
    if (px < 8) px = 8;
    m_leftLabelWidth = px;
    updateGeometry();
    update();
}

QSize MergeViewer::sizeHint() const
{
    // suggest a width that allows reasonable spacing, height depends on number of visible items
    int visibleCount = std::max(1, static_cast<int>(m_visibleIdsCache.size()));
    int h = (m_barHeight * visibleCount) + (m_barSpacing * (visibleCount + 1));
    h = std::max(h, 60);
    return QSize(320, h);
}

QSize MergeViewer::minimumSizeHint() const
{
    return QSize(200, 48);
}

void MergeViewer::updateVisibleAndFrame(const QMap<int, QColor>& wormColors, int currentFrame)
{
    m_visibleWormColors = wormColors;
    {
        // Avoid creating iterators into a temporary QList returned by keys()
        QList<int> keys = wormColors.keys();
        m_visibleIdsCache = QSet<int>(keys.begin(), keys.end());
    }
    m_currentFrame = currentFrame;
    updateGeometry();
    update();
}

void MergeViewer::paintEvent(QPaintEvent* ev)
{
    Q_UNUSED(ev);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    const QRect r = rect();

    // Fill background
    p.fillRect(r, palette().window());

    // Compute segments
    const int segments = 2 * m_radius + 1;
    const int contentLeft = m_leftLabelWidth + 8; // small padding after labels
    const int contentRight = r.right() - 6;
    const int contentWidth = std::max(0, contentRight - contentLeft + 1);
    const double segW_d = segments > 0 ? (double)contentWidth / segments : 0.0;
    const int segW = std::max(1, static_cast<int>(std::floor(segW_d)));

    // Draw vertical dashed separators between segments
    QPen sepPen(palette().mid().color());
    sepPen.setStyle(Qt::DashLine);
    sepPen.setWidthF(1.0);
    p.setPen(sepPen);

    for (int i = 1; i < segments; ++i) {
        int x = contentLeft + static_cast<int>(std::round(i * segW_d));
        p.drawLine(QPoint(x, r.top() + 6), QPoint(x, r.bottom() - 6));
    }

    // Prepare visible worm id list (sorted)
    QList<int> ids;
    if (!m_visibleWormColors.isEmpty()) {
        ids = m_visibleWormColors.keys();
    } else {
        ids = m_visibleIdsCache.values();
    }
    std::sort(ids.begin(), ids.end());

    // If there are no visible worms, show placeholder text
    if (ids.isEmpty()) {
        p.setPen(palette().text().color());
        QFont f = font();
        f.setItalic(true);
        p.setFont(f);
        QString msg = QStringLiteral("No visible worms");
        QFontMetrics fm(f);
        int tw = fm.horizontalAdvance(msg);
        p.drawText(contentLeft + (contentWidth - tw) / 2, r.center().y() + fm.ascent() / 2, msg);
        return;
    }

    // Determine vertical layout for bars
    const int totalBars = ids.size();
    const int availableHeight = r.height() - 12; // padding top/bottom
    // compute total needed bar area:
    const int needed = totalBars * m_barHeight + (totalBars + 1) * m_barSpacing;
    int startY = r.top() + 6;
    if (needed <= availableHeight) {
        // center vertically
        startY = r.top() + 6 + (availableHeight - needed) / 2;
    } else {
        // will overlap; clamp spacing
        // we keep m_barHeight and reduce spacing to fit
        int spac = std::max(2, (availableHeight - totalBars * m_barHeight) / (totalBars + 1));
        // recompute needed and startY
        startY = r.top() + 6;
        // override spacing locally
    }

    // Draw bars and labels
    QFont labelFont = font();
    labelFont.setBold(true);
    labelFont.setPointSizeF(labelFont.pointSizeF() - 1);

    // Font for frame number labels at the top of each segment
    QFont frameFont = font();
    frameFont.setPointSizeF(std::max(8.0, frameFont.pointSizeF() - 2));

    QFontMetrics labelFm(labelFont);
    QFontMetrics frameFm(frameFont);

    // Precompute integer start x and width for each segment to make bars stop exactly at dashed lines.
    QVector<int> segStart(segments);
    QVector<int> segWidthVec(segments);
    for (int i = 0; i < segments; ++i) {
        int s = contentLeft + static_cast<int>(std::round(i * segW_d));
        int e = contentLeft + static_cast<int>(std::round((i + 1) * segW_d));
        if (e <= s) e = s + 1;
        segStart[i] = s;
        segWidthVec[i] = std::max(1, e - s);
    }

    // Draw frame number labels centered at the top of each segment
    p.setFont(frameFont);
    p.setPen(palette().text().color());
    for (int i = 0; i < segments; ++i) {
        int frameNum = m_currentFrame - m_radius + i;
        QString ftext = QString::number(frameNum);
        int tw = frameFm.horizontalAdvance(ftext);
        int cx = segStart[i] + (segWidthVec[i] / 2) - (tw / 2);
        int ty = r.top() + 14; // small offset from top
        p.drawText(cx, ty, ftext);
    }

    // Restore label/font for the bars and ids
    p.setFont(labelFont);

    int y = startY;
    for (int id : ids) {
        QColor barColor = colorForId(id);

        // Prepare brush and border once per id
        QColor fill = barColor;
        fill.setAlpha(200);
        QBrush br(fill);
        QColor border = barColor.darker(120);

        // If no per-frame visibility map provided, draw the full-length rounded bar as before.
        if (m_visibleByFrame.isEmpty()) {
            QRect barRect(contentLeft + 1, y + m_barSpacing, contentWidth - 2, m_barHeight);
            p.setBrush(br);
            p.setPen(border);
            p.drawRoundedRect(barRect, 4, 4);
        } else {
            // Build per-segment visibility vector and log it for debugging.
            QVector<bool> visibleVec(segments, false);
            QList<int> frameNums;
            frameNums.reserve(segments);
            for (int i = 0; i < segments; ++i) {
                int frameNum = m_currentFrame - m_radius + i;
                frameNums.append(frameNum);
                auto it = m_visibleByFrame.find(frameNum);
                if (it != m_visibleByFrame.end()) {
                    visibleVec[i] = it.value().contains(id);
                } else {
                    visibleVec[i] = false;
                }
            }

            // Build a human-readable string for the visibility vector (e.g. "10110")
            QStringList visParts;
            visParts.reserve(segments);
            for (int i = 0; i < segments; ++i) {
                visParts << (visibleVec[i] ? QStringLiteral("1") : QStringLiteral("0"));
            }
            qDebug() << "MergeViewer: ID" << id << "frames" << frameNums << "visiblePerSegment" << visParts.join("");

            // Draw per-segment rectangles based on the computed visibility vector.
            for (int i = 0; i < segments; ++i) {
                if (!visibleVec[i]) continue;

                int sx = segStart[i] + 1;
                int sw = std::max(1, segWidthVec[i] - 2);
                QRect segRect(sx, y + m_barSpacing, sw, m_barHeight);

                p.setBrush(br);
                p.setPen(border);
                p.drawRect(segRect);
            }
        }

        // Draw ID label at left edge (within label area)
        QString labelText = QString::number(id);
        int textX = 4;
        // Vertical centering relative to the (first) bar area
        int textY = y + m_barSpacing + (m_barHeight + labelFm.ascent() - labelFm.descent()) / 2;
        p.setPen(palette().text().color());
        p.drawText(textX, textY, labelText);

        y += m_barSpacing + m_barHeight;
    }

    // Draw a subtle separator line between the labels area and the content
    QPen dividerPen(palette().mid().color());
    dividerPen.setWidthF(1.0);
    p.setPen(dividerPen);
    int dividerX = contentLeft - 6;
    p.drawLine(QPoint(dividerX, r.top() + 4), QPoint(dividerX, r.bottom() - 4));
}

QColor MergeViewer::colorForId(int id) const
{
    if (m_visibleWormColors.contains(id)) return m_visibleWormColors.value(id);
    // If not in explicit color map but present in ids cache, return default color.
    return m_defaultColor;
}