#include "wormtimeline.h"

#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QFontMetrics>
#include <QPainterPath>
#include <QWheelEvent>
#include <QString>
#include <cmath>

#include <algorithm>

WormTimeline::WormTimeline(QWidget* parent)
    : QWidget(parent)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    setMinimumHeight(140);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
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

void WormTimeline::updatePanForZoom(const QRect& contentRect, double zoomFactor, double cursorX)
{
    if (m_totalFrames <= 1) {
        m_zoom = 1.0;
        m_panX = 0.0;
        return;
    }

    double prevVirtualWidth = contentRect.width() * m_zoom;
    double newZoom = std::clamp(zoomFactor, 1.0, 6.0);
    double newVirtualWidth = contentRect.width() * newZoom;

    double t = 0.0;
    if (prevVirtualWidth > 0.0) {
        t = (cursorX - contentRect.left() + m_panX) / prevVirtualWidth;
    }
    t = std::clamp(t, 0.0, 1.0);

    m_zoom = newZoom;
    if (newVirtualWidth <= contentRect.width()) {
        m_panX = 0.0;
        return;
    }

    double desiredPan = t * newVirtualWidth - (cursorX - contentRect.left());
    double maxPan = newVirtualWidth - contentRect.width();
    m_panX = std::clamp(desiredPan, 0.0, maxPan);
}

void WormTimeline::rebuildEventNodes()
{
    m_eventNodes.clear();
    m_mergeSpans.clear();
    if (m_totalFrames <= 0 || m_mergeGroupsByFrame.isEmpty()) return;

    QMap<QString, QList<int>> prevGroups;
    QMap<QString, int> activeSpans;
    for (int frame = 0; frame < m_totalFrames; ++frame) {
        QMap<QString, QList<int>> currGroups;
        const QList<QList<int>> groups = m_mergeGroupsByFrame.value(frame);
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
            if (!prevGroups.contains(it.key())) {
                EventNode node;
                node.frame = frame;
                node.wormIds = it.value();
                m_eventNodes.append(node);
            }
            if (!activeSpans.contains(it.key())) {
                activeSpans.insert(it.key(), frame);
            }
        }
        for (auto it = prevGroups.constBegin(); it != prevGroups.constEnd(); ++it) {
            if (!currGroups.contains(it.key())) {
                EventNode node;
                node.frame = frame;
                node.wormIds = it.value();
                m_eventNodes.append(node);

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
}

void WormTimeline::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), palette().window());

    const QRect contentRect(
        m_leftPadding,
        m_topPadding,
        std::max(1, width() - m_leftPadding - m_rightLabelWidth - 6),
        std::max(1, height() - m_topPadding - m_bottomPadding)
    );
    const double virtualLeft = contentRect.left() - m_panX;
    const double virtualWidth = contentRect.width() * m_zoom;
    const double virtualRight = virtualLeft + virtualWidth;

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
    int rowSpacing = contentRect.height() / rows;
    rowSpacing = std::max(m_minRowSpacing, rowSpacing);
    if (rowSpacing * rows > contentRect.height()) {
        rowSpacing = std::max(8, contentRect.height() / rows);
    }
    int totalRowsHeight = rowSpacing * rows;
    int startY = contentRect.top() + (contentRect.height() - totalRowsHeight) / 2;

    QMap<int, int> yForId;
    for (int i = 0; i < rows; ++i) {
        int y = startY + i * rowSpacing + rowSpacing / 2;
        yForId.insert(ids[i], y);
    }

    // Current-frame dashed line
    if (m_totalFrames > 0) {
        double curX = frameToXWithZoom(m_currentFrame, contentRect);
        QPen curPen(QColor(20, 20, 20));
        curPen.setStyle(Qt::DashLine);
        curPen.setWidthF(2.0);
        p.setPen(curPen);
        p.drawLine(QPointF(curX, contentRect.top()), QPointF(curX, contentRect.bottom()));
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
        QPen linePen(c, m_lineThickness, Qt::SolidLine, Qt::RoundCap);
        p.setPen(linePen);
        int y = yForId.value(id);

        QVector<int> evIdx = wormEvents.value(id);
        std::sort(evIdx.begin(), evIdx.end(), [&](int a, int b) {
            return eventPos[a].x < eventPos[b].x;
        });

        QVector<const MergeSpan*> spansForWorm;
        spansForWorm.reserve(m_mergeSpans.size());
        for (const MergeSpan& span : m_mergeSpans) {
            if (span.wormIds.contains(id)) spansForWorm.append(&span);
        }
        std::sort(spansForWorm.begin(), spansForWorm.end(), [&](const MergeSpan* a, const MergeSpan* b) {
            return a->startFrame < b->startFrame;
        });

        QPainterPath path;
        path.moveTo(virtualLeft, y);

        for (int spanIndex = 0; spanIndex < spansForWorm.size(); ++spanIndex) {
            const MergeSpan* span = spansForWorm[spanIndex];
            double startX = frameToXWithZoom(span->startFrame, contentRect);
            double endX = frameToXWithZoom(span->endFrame, contentRect);
            if (endX < virtualLeft || startX > virtualRight) continue;

            double prevEndX = virtualLeft;
            if (spanIndex > 0) {
                prevEndX = frameToXWithZoom(spansForWorm[spanIndex - 1]->endFrame, contentRect);
            }
            double nextStartX = virtualRight;
            if (spanIndex + 1 < spansForWorm.size()) {
                nextStartX = frameToXWithZoom(spansForWorm[spanIndex + 1]->startFrame, contentRect);
            }

            double sumY = 0.0;
            int count = 0;
            for (int wid : span->wormIds) {
                if (yForId.contains(wid)) {
                    sumY += yForId.value(wid);
                    ++count;
                }
            }
            double mergeY = (count > 0) ? (sumY / static_cast<double>(count)) : y;

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
            path.lineTo(startX, mergeY);
            path.lineTo(endX, mergeY);
            path.lineTo(rampEnd, y);
        }

        path.lineTo(virtualRight, y);
        p.drawPath(path);
    }
    p.restore();

    // Draw event nodes on top of lines
    m_selectedNodeIndex = std::min(m_selectedNodeIndex, static_cast<int>(m_eventNodes.size()) - 1);
    for (int i = 0; i < m_eventNodes.size(); ++i) {
        EventNode& node = m_eventNodes[i];
        double x = eventPos[i].x;
        double y = eventPos[i].y;
        if (x <= 0.0 && y <= 0.0) continue;

        const double r = 8.0;
        QRectF circleRect(x - r, y - r, r * 2.0, r * 2.0);
        node.bounds = circleRect.adjusted(-3, -3, 3, 3);

        QColor fill(220, 220, 220);
        QColor border(60, 60, 60);
        if (i == m_selectedNodeIndex) {
            fill = QColor(200, 200, 200);
            border = QColor(20, 20, 20);
        }
        p.setPen(QPen(border, 2.0));
        p.setBrush(fill);
        p.drawEllipse(circleRect);
    }

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
        tickFont.setPointSizeF(std::max(8.0, tickFont.pointSizeF() - 1));
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

    // Labels on the right (scroll with timeline)
    p.setPen(palette().text().color());
    QFont labelFont = font();
    labelFont.setBold(true);
    p.setFont(labelFont);
    for (int id : ids) {
        QColor c = m_wormColors.value(id, QColor(120, 120, 120));
        p.setPen(c);
        int y = yForId.value(id);
        QString label = QString("Worm %1").arg(id);
        p.drawText(QPointF(virtualRight + 12, y + 4), label);
    }
}

void WormTimeline::mousePressEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }

    const QRect contentRect(
        m_leftPadding,
        m_topPadding,
        std::max(1, width() - m_leftPadding - m_rightLabelWidth - 6),
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
    event->accept();
}

void WormTimeline::mouseMoveEvent(QMouseEvent* event)
{
    if (m_isDraggingFrame) {
        const QRect contentRect(
            m_leftPadding,
            m_topPadding,
            std::max(1, width() - m_leftPadding - m_rightLabelWidth - 6),
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
        QWidget::mouseMoveEvent(event);
        return;
    }

    const QRect contentRect(
        m_leftPadding,
        m_topPadding,
        std::max(1, width() - m_leftPadding - m_rightLabelWidth - 6),
        std::max(1, height() - m_topPadding - m_bottomPadding)
    );

    const double dx = event->position().x() - m_lastPanX;
    m_lastPanX = event->position().x();

    double virtualWidth = contentRect.width() * m_zoom;
    if (virtualWidth <= contentRect.width()) {
        m_panX = 0.0;
        update();
        event->accept();
        return;
    }

    m_panX = std::clamp(m_panX - dx, 0.0, virtualWidth - contentRect.width());
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

void WormTimeline::wheelEvent(QWheelEvent* event)
{
    const QRect contentRect(
        m_leftPadding,
        m_topPadding,
        std::max(1, width() - m_leftPadding - m_rightLabelWidth - 6),
        std::max(1, height() - m_topPadding - m_bottomPadding)
    );

    const double delta = event->angleDelta().y();
    if (delta == 0.0) {
        event->accept();
        return;
    }

    const double factor = (delta > 0.0) ? 1.1 : (1.0 / 1.1);
    updatePanForZoom(contentRect, m_zoom * factor, event->position().x());
    update();
    event->accept();
}
