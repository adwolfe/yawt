#include "wormtimeline.h"

#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QFontMetrics>
#include <QPainterPath>
#include <QWheelEvent>
#include <QString>

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

void WormTimeline::updatePanForZoom(const QRect& contentRect, double zoomFactor, double cursorX)
{
    if (m_totalFrames <= 1) {
        m_zoom = 1.0;
        m_panX = 0.0;
        return;
    }

    double prevZoom = m_zoom;
    double prevVirtualWidth = contentRect.width() * prevZoom;
    double newZoom = std::clamp(zoomFactor, 1.0, 6.0);
    double newVirtualWidth = contentRect.width() * newZoom;

    double t = 0.0;
    if (prevVirtualWidth > 0.0) {
        double localX = cursorX - contentRect.left() + m_panX;
        t = localX / prevVirtualWidth;
    }
    t = std::clamp(t, 0.0, 1.0);

    m_zoom = newZoom;
    if (newVirtualWidth <= contentRect.width()) {
        m_panX = 0.0;
        return;
    }

    double desiredPan = (cursorX - contentRect.left()) - t * newVirtualWidth;
    double maxPan = newVirtualWidth - contentRect.width();
    if (desiredPan < -maxPan) desiredPan = -maxPan;
    if (desiredPan > 0.0) desiredPan = 0.0;
    m_panX = -desiredPan;
}

void WormTimeline::rebuildEventNodes()
{
    m_eventNodes.clear();
    if (m_totalFrames <= 0 || m_mergeGroupsByFrame.isEmpty()) return;

    QMap<QString, QList<int>> prevGroups;
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
        }
        for (auto it = prevGroups.constBegin(); it != prevGroups.constEnd(); ++it) {
            if (!currGroups.contains(it.key())) {
                EventNode node;
                node.frame = frame;
                node.wormIds = it.value();
                m_eventNodes.append(node);
            }
        }

        prevGroups = currGroups;
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

    // Keyframe dotted line
    if (m_totalFrames > 0) {
        double keyX = frameToXWithZoom(m_keyframeFrame, contentRect);
        QPen keyPen(QColor(30, 30, 30));
        keyPen.setStyle(Qt::DashLine);
        keyPen.setWidthF(2.0);
        p.setPen(keyPen);
        p.drawLine(QPointF(keyX, contentRect.top()), QPointF(keyX, contentRect.bottom()));
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
    for (int id : ids) {
        QColor c = m_wormColors.value(id, QColor(120, 120, 120));
        QPen linePen(c, m_lineThickness, Qt::SolidLine, Qt::RoundCap);
        p.setPen(linePen);
        int y = yForId.value(id);

        QVector<int> evIdx = wormEvents.value(id);
        std::sort(evIdx.begin(), evIdx.end(), [&](int a, int b) {
            return eventPos[a].x < eventPos[b].x;
        });

        QPainterPath path;
        path.moveTo(contentRect.left(), y);

        for (int j = 0; j < evIdx.size(); ++j) {
            int idx = evIdx[j];
            double x = eventPos[idx].x;
            double yNode = eventPos[idx].y;

            double prevX = (j > 0) ? eventPos[evIdx[j - 1]].x : contentRect.left();
            double nextX = (j + 1 < evIdx.size()) ? eventPos[evIdx[j + 1]].x : contentRect.right();
            double leftGap = std::max(6.0, x - prevX);
            double rightGap = std::max(6.0, nextX - x);
            double dx = std::min(40.0, 0.2 * std::min(leftGap, rightGap));

            path.lineTo(x - dx, y);
            path.lineTo(x, yNode);
            path.lineTo(x + dx, y);
        }

        path.lineTo(contentRect.right(), y);
        p.drawPath(path);
    }

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

    // Labels on the right
    p.setPen(palette().text().color());
    QFont labelFont = font();
    labelFont.setBold(true);
    p.setFont(labelFont);
    for (int id : ids) {
        QColor c = m_wormColors.value(id, QColor(120, 120, 120));
        p.setPen(c);
        int y = yForId.value(id);
        QString label = QString("Worm %1").arg(id);
        p.drawText(QPointF(contentRect.right() + 12, y + 4), label);
    }
}

void WormTimeline::mousePressEvent(QMouseEvent* event)
{
    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
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

    QWidget::mousePressEvent(event);
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
