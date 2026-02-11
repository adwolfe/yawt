#include "wormtimeline.h"

#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QFontMetrics>
#include <QPainterPath>
#include <QWheelEvent>
#include <QString>
#include <QStringList>
#include <cmath>

#include <algorithm>

WormTimeline::WormTimeline(QWidget* parent)
    : QWidget(parent)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    setMinimumHeight(140);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setMouseTracking(true);
}

void WormTimeline::setTotalFrames(int totalFrames)
{
    if (totalFrames < 0) totalFrames = 0;
    if (m_totalFrames == totalFrames) return;
    m_totalFrames = totalFrames;
    rebuildEventNodes();
    update();
}

void WormTimeline::setKeyframeFrame(int frame)
{
    if (frame < 0) frame = 0;
    if (m_keyframeFrame == frame) return;
    m_keyframeFrame = frame;
    update();
}

void WormTimeline::setCurrentFrame(int frame)
{
    if (frame < 0) frame = 0;
    if (m_currentFrame == frame) return;
    m_currentFrame = frame;
    update();
}

void WormTimeline::setWormColors(const QMap<int, QColor>& idColors)
{
    m_wormColors = idColors;
    updateGeometry();
    update();
}

void WormTimeline::setMergeGroupsByFrame(const QMap<int, QList<QList<int>>>& mergeGroupsByFrame)
{
    m_mergeGroupsByFrame = mergeGroupsByFrame;
    rebuildEventNodes();
    update();
}

QSize WormTimeline::sizeHint() const
{
    int rows = std::max(1, static_cast<int>(m_wormColors.size()));
    int h = rows * std::max(m_minRowSpacing, m_lineThickness + 6) + m_topPadding + m_bottomPadding;
    return QSize(420, std::max(140, h));
}

QSize WormTimeline::minimumSizeHint() const
{
    return QSize(240, 120);
}

QList<int> WormTimeline::sortedWormIds() const
{
    QList<int> ids = m_wormColors.keys();
    std::sort(ids.begin(), ids.end());
    return ids;
}

double WormTimeline::frameToX(int frame, const QRect& contentRect) const
{
    if (m_totalFrames <= 1) return contentRect.left();
    double t = static_cast<double>(frame) / static_cast<double>(m_totalFrames - 1);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return contentRect.left() + t * contentRect.width();
}

double WormTimeline::frameToXWithZoom(int frame, const QRect& contentRect) const
{
    if (m_totalFrames <= 1) return contentRect.left();
    double t = static_cast<double>(frame) / static_cast<double>(m_totalFrames - 1);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    double virtualWidth = contentRect.width() * m_zoom;
    double x = contentRect.left() + t * virtualWidth - m_panX;
    return x;
}

int WormTimeline::xToFrameWithZoom(double x, const QRect& contentRect) const
{
    if (m_totalFrames <= 1) return 0;
    double virtualWidth = contentRect.width() * m_zoom;
    if (virtualWidth <= 0.0) return 0;
    double t = (x - contentRect.left() + m_panX) / virtualWidth;
    t = std::clamp(t, 0.0, 1.0);
    int frame = static_cast<int>(std::round(t * static_cast<double>(m_totalFrames - 1)));
    return frame;
}

static int niceFrameStep(int frameSpan)
{
    if (frameSpan <= 0) return 1;
    const int targets[] = {10, 25, 50, 100, 200, 500, 1000, 2000, 5000, 10000};
    for (int t : targets) {
        if (frameSpan / t <= 8) return t;
    }
    return targets[9];
}

static QString mergeKeyForIds(const QList<int>& ids)
{
    QString key;
    for (int id : ids) {
        if (!key.isEmpty()) key += "-";
        key += QString::number(id);
    }
    return key;
}

static QString wormIdsTooltip(const QList<int>& ids)
{
    if (ids.isEmpty()) return QStringLiteral("Worms: none");
    QStringList parts;
    parts.reserve(ids.size());
    for (int id : ids) {
        parts.append(QString::number(id));
    }
    return QStringLiteral("Worms: %1").arg(parts.join(", "));
}

void WormTimeline::updatePanForZoom(const QRect& contentRect, double zoomFactorX, double zoomFactorY, const QPointF& cursorPos)
{
    if (m_totalFrames <= 1) {
        m_zoom = 1.0;
        m_panX = 0.0;
        m_zoomY = 1.0;
        m_panY = 0.0;
        return;
    }

    double prevVirtualWidth = contentRect.width() * m_zoom;
    double newZoomX = std::clamp(zoomFactorX, 1.0, 12.0);
    double newVirtualWidth = contentRect.width() * newZoomX;

    double t = 0.0;
    if (prevVirtualWidth > 0.0) {
        t = (cursorPos.x() - contentRect.left() + m_panX) / prevVirtualWidth;
    }
    t = std::clamp(t, 0.0, 1.0);

    m_zoom = newZoomX;
    if (newVirtualWidth <= contentRect.width()) {
        m_panX = 0.0;
    } else {
        double desiredPan = t * newVirtualWidth - (cursorPos.x() - contentRect.left());
        double maxPan = newVirtualWidth - contentRect.width();
        m_panX = std::clamp(desiredPan, 0.0, maxPan);
    }

    double prevVirtualHeight = contentRect.height() * m_zoomY;
    double newZoomY = std::clamp(zoomFactorY, 1.0, 12.0);
    double newVirtualHeight = contentRect.height() * newZoomY;

    double ty = 0.0;
    if (prevVirtualHeight > 0.0) {
        ty = (cursorPos.y() - contentRect.top() + m_panY) / prevVirtualHeight;
    }
    ty = std::clamp(ty, 0.0, 1.0);

    m_zoomY = newZoomY;
    if (newVirtualHeight <= contentRect.height()) {
        m_panY = 0.0;
        return;
    }

    double desiredPanY = ty * newVirtualHeight - (cursorPos.y() - contentRect.top());
    double maxPanY = newVirtualHeight - contentRect.height();
    m_panY = std::clamp(desiredPanY, 0.0, maxPanY);
}

void WormTimeline::rebuildEventNodes()
{
    m_eventNodes.clear();
    m_mergeSpans.clear();
    if (m_totalFrames <= 0 || m_mergeGroupsByFrame.isEmpty()) return;

    const int minTransientFrames = 6;

    QMap<QString, QList<int>> prevGroups;
    QMap<QString, int> activeSpans;
    int earliestMergeFrame = -1;
    for (int frame = 0; frame < m_totalFrames; ++frame) {
        QMap<QString, QList<int>> currGroups;
        const QList<QList<int>> groups = m_mergeGroupsByFrame.value(frame);
        if (earliestMergeFrame < 0 && !groups.isEmpty()) {
            earliestMergeFrame = frame;
        }
        for (const QList<int>& group : groups) {
            if (group.size() < 2) continue;
            QList<int> ids = group;
            std::sort(ids.begin(), ids.end());
            QString key;
            for (int id : ids) {
                if (!key.isEmpty()) key += "-";
                key += QString::number(id);
            }
            currGroups.insert(key, ids);
        }

        for (auto it = currGroups.constBegin(); it != currGroups.constEnd(); ++it) {
            if (!activeSpans.contains(it.key())) {
                activeSpans.insert(it.key(), frame);
            }
        }
        for (auto it = prevGroups.constBegin(); it != prevGroups.constEnd(); ++it) {
            if (!currGroups.contains(it.key())) {
                if (activeSpans.contains(it.key())) {
                    MergeSpan span;
                    span.startFrame = activeSpans.value(it.key());
                    span.endFrame = frame;
                    span.wormIds = it.value();
                    m_mergeSpans.append(span);
                    activeSpans.remove(it.key());
                }
            }
        }

        prevGroups = currGroups;
    }

    for (auto it = activeSpans.constBegin(); it != activeSpans.constEnd(); ++it) {
        MergeSpan span;
        span.startFrame = it.value();
        span.endFrame = m_totalFrames - 1;
        span.wormIds = prevGroups.value(it.key());
        m_mergeSpans.append(span);
    }

    // If merge history doesn't show an initial merged state, synthesize one at the start.
    if (earliestMergeFrame > 0) {
        QList<int> allIds = m_wormColors.keys();
        if (allIds.size() > 1) {
            std::sort(allIds.begin(), allIds.end());
            MergeSpan span;
            span.startFrame = 0;
            span.endFrame = earliestMergeFrame;
            span.wormIds = allIds;
            m_mergeSpans.append(span);
        }
    }

    QMap<QString, QList<MergeSpan>> spansByKey;
    for (const MergeSpan& span : m_mergeSpans) {
        spansByKey[mergeKeyForIds(span.wormIds)].append(span);
    }

    m_mergeSpans.clear();
    for (auto it = spansByKey.begin(); it != spansByKey.end(); ++it) {
        QList<MergeSpan>& spans = it.value();
        std::sort(spans.begin(), spans.end(), [](const MergeSpan& a, const MergeSpan& b) {
            return a.startFrame < b.startFrame;
        });

        for (int i = 0; i + 1 < spans.size();) {
            int gap = spans[i + 1].startFrame - spans[i].endFrame;
            if (gap <= minTransientFrames) {
                spans[i].endFrame = std::max(spans[i].endFrame, spans[i + 1].endFrame);
                spans.removeAt(i + 1);
                continue;
            }
            ++i;
        }

        for (const MergeSpan& span : spans) {
            int duration = std::max(0, span.endFrame - span.startFrame + 1);
            if (duration < minTransientFrames) continue;
            m_mergeSpans.append(span);
        }
    }

    m_eventNodes.clear();
    for (const MergeSpan& span : m_mergeSpans) {
        EventNode startNode;
        startNode.frame = span.startFrame;
        startNode.wormIds = span.wormIds;
        m_eventNodes.append(startNode);

        EventNode endNode;
        endNode.frame = span.endFrame;
        endNode.wormIds = span.wormIds;
        m_eventNodes.append(endNode);
    }
}

void WormTimeline::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), palette().window());

    const int labelGap = 3;
    const int rightPad = 6;
    const QRect labelRect(
        m_leftPadding,
        m_topPadding,
        m_rightLabelWidth,
        std::max(1, height() - m_topPadding - m_bottomPadding)
    );
    const QRect contentRect(
        labelRect.right() + 1 + labelGap,
        m_topPadding,
        std::max(1, width() - m_leftPadding - m_rightLabelWidth - labelGap - rightPad),
        std::max(1, height() - m_topPadding - m_bottomPadding)
    );
    const double virtualLeft = contentRect.left() - m_panX;
    const double virtualWidth = contentRect.width() * m_zoom;
    const double virtualRight = virtualLeft + virtualWidth;
    const double virtualHeight = contentRect.height() * m_zoomY;

    QList<int> ids = sortedWormIds();
    if (ids.isEmpty()) {
        p.setPen(palette().text().color());
        QFont f = font();
        f.setItalic(true);
        p.setFont(f);
        const QString msg = QStringLiteral("No worms");
        QFontMetrics fm(f);
        int tw = fm.horizontalAdvance(msg);
        p.drawText(contentRect.left() + (contentRect.width() - tw) / 2, rect().center().y(), msg);
        return;
    }

    int rows = ids.size();
    const double baseSpacing = (rows > 0) ? (static_cast<double>(contentRect.height()) / static_cast<double>(rows)) : 0.0;
    const double minSpacingFloor = static_cast<double>(m_minRowSpacing);
    const double rowSpacing = baseSpacing * m_zoomY;
    const double totalRowsHeight = rowSpacing * rows;
    const double startY = 0.0;
    const double spacingScale = (minSpacingFloor > 0.0) ? (rowSpacing / minSpacingFloor) : 1.0;
    double crowdScale = 1.0;
    if (rowSpacing > 0.0 && rowSpacing < minSpacingFloor) {
        crowdScale = std::clamp(spacingScale, 0.4, 1.0);
    }
    const double lineScale = std::clamp(spacingScale, 0.4, 1.6);

    QMap<int, int> yForId;
    QMap<int, int> indexForId;
    for (int i = 0; i < rows; ++i) {
        double yVirtual = startY + i * rowSpacing + rowSpacing * 0.5;
        int y = static_cast<int>(std::round(contentRect.top() + (yVirtual - m_panY)));
        yForId.insert(ids[i], y);
        indexForId.insert(ids[i], i);
    }

    // Current-frame dashed line
    if (m_totalFrames > 0) {
        double curX = frameToXWithZoom(m_currentFrame, contentRect);
        QPen curPen(QColor(20, 20, 20));
        curPen.setStyle(Qt::DashLine);
        curPen.setWidthF(2.0);
        p.save();
        p.setClipRect(contentRect);
        p.setPen(curPen);
        p.drawLine(QPointF(curX, contentRect.top()), QPointF(curX, contentRect.bottom()));
        p.restore();
    }

    // Precompute event positions and index per worm
    struct EventPos {
        double x = 0.0;
        double y = 0.0;
    };
    QVector<EventPos> eventPos(m_eventNodes.size());
    QMap<int, QVector<int>> wormEvents;

    for (int i = 0; i < m_eventNodes.size(); ++i) {
        const EventNode& node = m_eventNodes[i];
        double x = frameToXWithZoom(node.frame, contentRect);
        double sumY = 0.0;
        int count = 0;
        for (int id : node.wormIds) {
            if (yForId.contains(id)) {
                sumY += yForId.value(id);
                ++count;
                wormEvents[id].append(i);
            }
        }
        if (count == 0) continue;
        eventPos[i].x = x;
        eventPos[i].y = sumY / static_cast<double>(count);
    }

    // Draw worm timelines with local detours into merge/split nodes
    p.save();
    p.setClipRect(contentRect);
    for (int id : ids) {
        QColor c = m_wormColors.value(id, QColor(120, 120, 120));
        const double effectiveThickness = std::clamp(m_lineThickness * lineScale, 2.0, 10.0);
        QPen linePen(c, effectiveThickness, Qt::SolidLine, Qt::RoundCap);
        p.setPen(linePen);
        int y = yForId.value(id);

        QPainterPath path;
        path.moveTo(virtualLeft, y);

        struct SpanCandidate {
            double startX;
            double endX;
            double targetY;
            int priority;
        };
        struct SpanAffect {
            double startX;
            double endX;
            double targetY;
        };
        QVector<SpanCandidate> candidates;
        candidates.reserve(m_mergeSpans.size());

        for (const MergeSpan& span : m_mergeSpans) {
            double startX = frameToXWithZoom(span.startFrame, contentRect);
            double endX = frameToXWithZoom(span.endFrame, contentRect);
            if (endX < virtualLeft || startX > virtualRight) continue;

            double sumY = 0.0;
            double sumIdx = 0.0;
            int count = 0;
            double minY = y;
            double maxY = y;
            bool first = true;
            for (int wid : span.wormIds) {
                if (yForId.contains(wid)) {
                    double wy = yForId.value(wid);
                    sumY += wy;
                    sumIdx += indexForId.value(wid, 0);
                    ++count;
                    if (first) {
                        minY = wy;
                        maxY = wy;
                        first = false;
                    } else {
                        minY = std::min(minY, wy);
                        maxY = std::max(maxY, wy);
                    }
                }
            }
            double mergeY = (count > 0) ? (sumY / static_cast<double>(count)) : y;
            double mergeIndex = (count > 0) ? (sumIdx / static_cast<double>(count)) : indexForId.value(id, 0);

            int duration = std::max(1, span.endFrame - span.startFrame + 1);
            int basePriority = static_cast<int>(span.wormIds.size()) * 100000 + duration;
            bool inGroup = span.wormIds.contains(id);
            if (inGroup) {
                QList<int> sortedGroup = span.wormIds;
                std::sort(sortedGroup.begin(), sortedGroup.end());
                int idx = sortedGroup.indexOf(id);
                int n = sortedGroup.size();
                double step = std::min(rowSpacing * 0.2, effectiveThickness);
                double offset = 0.0;
                if (n > 1 && idx >= 0) {
                    offset = (static_cast<double>(idx) - (static_cast<double>(n - 1) / 2.0)) * step;
                }
                candidates.append({startX, endX, mergeY + offset, basePriority});
            } else if (y >= minY && y <= maxY && count > 1) {
                int idx = indexForId.value(id, 0);
                double targetY = (static_cast<double>(idx) <= mergeIndex) ? minY : maxY;
                if (targetY != y) {
                    candidates.append({startX, endX, targetY, basePriority - 1});
                }
            }
        }

        QVector<SpanAffect> affects;
        if (!candidates.isEmpty()) {
            QVector<double> bounds;
            bounds.reserve(candidates.size() * 2 + 2);
            bounds.append(virtualLeft);
            bounds.append(virtualRight);
            for (const SpanCandidate& c : candidates) {
                bounds.append(std::max(virtualLeft, std::min(virtualRight, c.startX)));
                bounds.append(std::max(virtualLeft, std::min(virtualRight, c.endX)));
            }
            std::sort(bounds.begin(), bounds.end());
            bounds.erase(std::unique(bounds.begin(), bounds.end()), bounds.end());

            auto pickBest = [&](double midX) -> const SpanCandidate* {
                const SpanCandidate* best = nullptr;
                for (const SpanCandidate& c : candidates) {
                    if (midX < c.startX || midX > c.endX) continue;
                    if (!best || c.priority > best->priority) best = &c;
                }
                return best;
            };

            for (int i = 0; i + 1 < bounds.size(); ++i) {
                double a = bounds[i];
                double b = bounds[i + 1];
                if (b <= a) continue;
                double mid = (a + b) * 0.5;
                const SpanCandidate* best = pickBest(mid);
                if (!best) continue;
                if (!affects.isEmpty() && std::abs(affects.last().targetY - best->targetY) < 0.01
                    && std::abs(affects.last().endX - a) < 0.01) {
                    affects.last().endX = b;
                } else {
                    affects.append({a, b, best->targetY});
                }
            }
        }

        for (int spanIndex = 0; spanIndex < affects.size(); ++spanIndex) {
            const SpanAffect& span = affects[spanIndex];
            double startX = span.startX;
            double endX = span.endX;
            double targetY = span.targetY;

            double prevEndX = virtualLeft;
            if (spanIndex > 0) prevEndX = affects[spanIndex - 1].endX;
            double nextStartX = virtualRight;
            if (spanIndex + 1 < affects.size()) nextStartX = affects[spanIndex + 1].startX;

            double spanWidth = std::max(8.0, endX - startX);
            double ramp = std::min(40.0, 0.2 * spanWidth);
            double leftGap = std::max(0.0, startX - prevEndX);
            double rightGap = std::max(0.0, nextStartX - endX);
            ramp = std::min(ramp, 0.45 * leftGap);
            ramp = std::min(ramp, 0.45 * rightGap);

            double rampStart = startX - ramp;
            double rampEnd = endX + ramp;
            if (rampStart < prevEndX) rampStart = prevEndX;
            if (rampEnd > nextStartX) rampEnd = nextStartX;

            path.lineTo(rampStart, y);
            path.lineTo(startX, targetY);
            path.lineTo(endX, targetY);
            path.lineTo(rampEnd, y);
        }

        path.lineTo(virtualRight, y);
        p.drawPath(path);
    }
    p.restore();

    // Draw event nodes on top of lines (clipped to the timeline window)
    m_selectedNodeIndex = std::min(m_selectedNodeIndex, static_cast<int>(m_eventNodes.size()) - 1);
    p.save();
    p.setClipRect(contentRect);
    for (int i = 0; i < m_eventNodes.size(); ++i) {
        EventNode& node = m_eventNodes[i];
        double x = eventPos[i].x;
        double y = eventPos[i].y;
        if (x <= 0.0 && y <= 0.0) continue;

        const double r = std::clamp(6.0 * lineScale, 2.5, 10.0);
        QRectF circleRect(x - r, y - r, r * 2.0, r * 2.0);
        node.bounds = circleRect.adjusted(-3, -3, 3, 3);

        QColor fill(220, 220, 220);
        QColor border(60, 60, 60);
        if (i == m_selectedNodeIndex) {
            fill = QColor(200, 200, 200);
            border = QColor(20, 20, 20);
        }
        if (i == m_hoveredNodeIndex) {
            fill.setAlphaF(0.2);
            border.setAlphaF(0.2);
        }
        p.setPen(QPen(border, 2.0));
        p.setBrush(fill);
        p.drawEllipse(circleRect);
    }
    p.restore();

    // Scale bar (frame ticks) below the worm lines
    {
        int barY = contentRect.bottom() - 6;
        QPen axisPen(palette().mid().color());
        axisPen.setWidthF(1.0);
        p.setPen(axisPen);
        p.drawLine(QPointF(contentRect.left(), barY), QPointF(contentRect.right(), barY));

        double leftFrameF = (m_totalFrames > 1 && virtualWidth > 0.0)
            ? (m_panX / virtualWidth) * static_cast<double>(m_totalFrames - 1)
            : 0.0;
        double rightFrameF = (m_totalFrames > 1 && virtualWidth > 0.0)
            ? ((m_panX + contentRect.width()) / virtualWidth) * static_cast<double>(m_totalFrames - 1)
            : 0.0;

        int leftFrame = std::max(0, static_cast<int>(std::floor(leftFrameF)));
        int rightFrame = std::min(m_totalFrames - 1, static_cast<int>(std::ceil(rightFrameF)));
        int span = std::max(1, rightFrame - leftFrame);
        int step = niceFrameStep(span);
        int firstTick = (leftFrame / step) * step;

        QFont tickFont = font();
        double tickSize = std::max(8.0, tickFont.pointSizeF() - 1);
        if (crowdScale < 1.0) {
            tickSize = std::max(6.0, tickSize * crowdScale);
        }
        tickFont.setPointSizeF(tickSize);
        p.setFont(tickFont);
        QFontMetrics fm(tickFont);

        for (int f = firstTick; f <= rightFrame; f += step) {
            double x = frameToXWithZoom(f, contentRect);
            if (x < contentRect.left() - 1 || x > contentRect.right() + 1) continue;
            p.drawLine(QPointF(x, barY - 4), QPointF(x, barY + 4));
            QString label = QString::number(f);
            int tw = fm.horizontalAdvance(label);
            p.drawText(QPointF(x - tw / 2.0, barY + 14), label);
        }
    }

    // Timeline window border and fixed labels on the left
    p.setPen(QPen(palette().mid().color(), 1.0));
    p.setBrush(Qt::NoBrush);
    p.drawRect(contentRect.adjusted(0, 0, -1, -1));

    p.setPen(palette().text().color());
    QFont labelFont = font();
    labelFont.setBold(true);
    if (crowdScale < 1.0) {
        labelFont.setPointSizeF(std::max(6.0, labelFont.pointSizeF() * crowdScale));
    }
    p.setFont(labelFont);
    p.save();
    p.setClipRect(labelRect);
    for (int id : ids) {
        QColor c = m_wormColors.value(id, QColor(120, 120, 120));
        p.setPen(c);
        int y = yForId.value(id);
        QString label = QString("Worm %1").arg(id);
        p.drawText(QPointF(labelRect.left() + 4, y + 4), label);
    }
    p.restore();
}

void WormTimeline::mousePressEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }

    const int labelGap = 3;
    const int rightPad = 6;
    const QRect contentRect(
        m_leftPadding + m_rightLabelWidth + labelGap,
        m_topPadding,
        std::max(1, width() - m_leftPadding - m_rightLabelWidth - labelGap - rightPad),
        std::max(1, height() - m_topPadding - m_bottomPadding)
    );
    const double curX = frameToXWithZoom(m_currentFrame, contentRect);
    if (std::abs(event->position().x() - curX) <= 6.0) {
        m_isDraggingFrame = true;
        event->accept();
        return;
    }

        for (int i = 0; i < m_eventNodes.size(); ++i) {
            const EventNode& node = m_eventNodes[i];
            if (node.bounds.contains(event->position())) {
                m_selectedNodeIndex = i;
                update();
                emit eventClicked(node.frame, node.wormIds);
                return;
            }
        }
    m_isPanning = true;
    m_lastPanX = event->position().x();
    m_lastPanY = event->position().y();
    event->accept();
}

void WormTimeline::mouseMoveEvent(QMouseEvent* event)
{
    if (m_isDraggingFrame) {
        const int labelGap = 3;
        const int rightPad = 6;
        const QRect contentRect(
            m_leftPadding + m_rightLabelWidth + labelGap,
            m_topPadding,
            std::max(1, width() - m_leftPadding - m_rightLabelWidth - labelGap - rightPad),
            std::max(1, height() - m_topPadding - m_bottomPadding)
        );
        int frame = xToFrameWithZoom(event->position().x(), contentRect);
        m_currentFrame = frame;
        update();
        emit frameScrubbed(frame);
        event->accept();
        return;
    }
    if (!m_isPanning) {
        int hoveredIndex = -1;
        for (int i = 0; i < m_eventNodes.size(); ++i) {
            if (m_eventNodes[i].bounds.contains(event->position())) {
                hoveredIndex = i;
                break;
            }
        }
        if (hoveredIndex != m_hoveredNodeIndex) {
            m_hoveredNodeIndex = hoveredIndex;
            if (m_hoveredNodeIndex >= 0) {
                setToolTip(wormIdsTooltip(m_eventNodes[m_hoveredNodeIndex].wormIds));
            } else {
                setToolTip(QString());
            }
            update();
        }
        QWidget::mouseMoveEvent(event);
        return;
    }

    const int labelGap = 3;
    const int rightPad = 6;
    const QRect contentRect(
        m_leftPadding + m_rightLabelWidth + labelGap,
        m_topPadding,
        std::max(1, width() - m_leftPadding - m_rightLabelWidth - labelGap - rightPad),
        std::max(1, height() - m_topPadding - m_bottomPadding)
    );

    const double dx = event->position().x() - m_lastPanX;
    const double dy = event->position().y() - m_lastPanY;
    m_lastPanX = event->position().x();
    m_lastPanY = event->position().y();

    double virtualWidth = contentRect.width() * m_zoom;
    double virtualHeight = contentRect.height() * m_zoomY;
    if (virtualWidth <= contentRect.width()) {
        m_panX = 0.0;
    } else {
        m_panX = std::clamp(m_panX - dx, 0.0, virtualWidth - contentRect.width());
    }

    if (virtualHeight <= contentRect.height()) {
        m_panY = 0.0;
    } else {
        m_panY = std::clamp(m_panY - dy, 0.0, virtualHeight - contentRect.height());
    }
    update();
    event->accept();
}

void WormTimeline::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        m_isPanning = false;
        m_isDraggingFrame = false;
    }
    QWidget::mouseReleaseEvent(event);
}

void WormTimeline::leaveEvent(QEvent* event)
{
    if (m_hoveredNodeIndex != -1) {
        m_hoveredNodeIndex = -1;
        setToolTip(QString());
        update();
    }
    QWidget::leaveEvent(event);
}

void WormTimeline::wheelEvent(QWheelEvent* event)
{
    const int labelGap = 3;
    const int rightPad = 6;
    const QRect contentRect(
        m_leftPadding + m_rightLabelWidth + labelGap,
        m_topPadding,
        std::max(1, width() - m_leftPadding - m_rightLabelWidth - labelGap - rightPad),
        std::max(1, height() - m_topPadding - m_bottomPadding)
    );

    const double delta = event->angleDelta().y();
    if (delta == 0.0) {
        event->accept();
        return;
    }

    const double factor = (delta > 0.0) ? 1.1 : (1.0 / 1.1);
    updatePanForZoom(contentRect, m_zoom * factor, m_zoomY * factor, event->position());
    update();
    event->accept();
}
