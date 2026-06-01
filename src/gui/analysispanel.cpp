#include "analysispanel.h"
#include "analysissessionmodel.h"
#include "analysisgroupwidgets.h"
#include "analysisdialog.h"
#include "trackingdatastorage.h"
#include "trackingcommon.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMdiArea>
#include <QMdiSubWindow>
#include <QPixmap>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollBar>
#include <QSpinBox>
#include <QSplitter>
#include <QTreeView>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QPointF>
#include <QLineF>
#include <QFont>
#include <QFontMetrics>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

// (Shared helpers removed — speed/reversal computation now lives in analysisgroupwidgets.cpp)

// Suppress unused-include warning for the remaining includes kept for other reasons.

static std::vector<std::pair<int,double>> computeSpeedTimeline(
    const std::vector<Tracking::WormTrackPoint>& points,
    double umPerPixel,
    double fps)
{
    std::vector<std::pair<int,double>> result;
    result.reserve(points.size());

    const double unitScale = (umPerPixel > 0.0) ? umPerPixel : 1.0;
    const bool haveFps = (fps > 0.0);
    const int windowFrames = haveFps ? std::max(1, static_cast<int>(std::round(2.0 * fps))) : 1;

    bool hasPrev = false;
    QPointF prevPos;
    int prevFrame = 0;
    std::deque<std::pair<int,double>> window;
    double windowSum = 0.0;

    for (const auto& pt : points) {
        if (pt.quality == Tracking::TrackPointQuality::Lost) {
            hasPrev = false;
            window.clear();
            windowSum = 0.0;
            continue;
        }
        const QPointF pos(pt.position.x, pt.position.y);

        if (!hasPrev) {
            result.push_back({pt.frameNumberOriginal, 0.0});
            hasPrev = true;
            prevPos = pos;
            prevFrame = pt.frameNumberOriginal;
            continue;
        }

        const int frameDelta = pt.frameNumberOriginal - prevFrame;
        if (frameDelta <= 0) {
            result.push_back({pt.frameNumberOriginal, 0.0});
            prevPos = pos;
            prevFrame = pt.frameNumberOriginal;
            continue;
        }

        const double dist = QLineF(prevPos, pos).length() * unitScale;
        const double dt = haveFps ? (static_cast<double>(frameDelta) / fps) : 1.0;
        const double speed = dist / dt;

        window.emplace_back(pt.frameNumberOriginal, speed);
        windowSum += speed;
        while (!window.empty() && (pt.frameNumberOriginal - window.front().first) > windowFrames) {
            windowSum -= window.front().second;
            window.pop_front();
        }
        const double avg = window.empty() ? speed : windowSum / static_cast<double>(window.size());

        result.push_back({pt.frameNumberOriginal, avg});
        prevPos = pos;
        prevFrame = pt.frameNumberOriginal;
    }
    return result;
}

static double computeAverageSpeed(
    const std::vector<Tracking::WormTrackPoint>& points,
    double umPerPixel,
    double fps)
{
    auto timeline = computeSpeedTimeline(points, umPerPixel, fps);
    if (timeline.empty()) return 0.0;
    double sum = 0.0;
    int count = 0;
    for (auto& [frame, spd] : timeline) {
        if (spd > 0.0) { sum += spd; ++count; }
    }
    return count > 0 ? sum / count : 0.0;
}

// Count reversals: direction flip > 90° with minimum displacement
static int countReversals(const std::vector<Tracking::WormTrackPoint>& points,
                          double minDisplacementPx = 2.0)
{
    // Smooth positions first over a small window
    std::vector<QPointF> positions;
    positions.reserve(points.size());
    for (const auto& pt : points) {
        if (pt.quality != Tracking::TrackPointQuality::Lost)
            positions.push_back(QPointF(pt.position.x, pt.position.y));
    }
    if (positions.size() < 3) return 0;

    // Compute velocity vectors between consecutive points
    std::vector<QPointF> vels;
    vels.reserve(positions.size() - 1);
    for (size_t i = 1; i < positions.size(); ++i) {
        const QPointF v = positions[i] - positions[i - 1];
        if (QLineF(QPointF(), v).length() >= minDisplacementPx) {
            vels.push_back(v);
        } else {
            // Carry previous velocity to avoid noise triggering reversals
            vels.push_back(vels.empty() ? QPointF(0, 0) : vels.back());
        }
    }

    int reversals = 0;
    for (size_t i = 1; i < vels.size(); ++i) {
        const double dot = vels[i - 1].x() * vels[i].x() + vels[i - 1].y() * vels[i].y();
        const double mag1 = std::hypot(vels[i - 1].x(), vels[i - 1].y());
        const double mag2 = std::hypot(vels[i].x(), vels[i].y());
        if (mag1 < 1e-9 || mag2 < 1e-9) continue;
        const double cosAngle = dot / (mag1 * mag2);
        if (cosAngle < 0.0) ++reversals; // angle > 90°
    }
    return reversals;
}

// Simple bar-chart helper used by AverageSpeedWidget and ReversalCountWidget
static void drawBarChart(QPainter& painter, const QRect& rect,
                         const QList<int>& wormIds,
                         const std::function<double(int)>& valueForId,
                         const std::function<QColor(int)>& colorForId,
                         const std::function<QString(int)>& labelForId,
                         const QString& yAxisLabel)
{
    painter.fillRect(rect, painter.background());

    if (wormIds.isEmpty()) {
        painter.setPen(painter.pen().color());
        painter.drawText(rect, Qt::AlignCenter, "No data");
        return;
    }

    const qreal margin = 40.0;
    const qreal rightMargin = 16.0;
    const qreal topMargin = 16.0;
    QRectF plotArea(rect.left() + margin, rect.top() + topMargin,
                    rect.width() - margin - rightMargin, rect.height() - margin - topMargin);
    if (plotArea.width() <= 0 || plotArea.height() <= 0) return;

    // Find max value
    double maxVal = 0.0;
    for (int id : wormIds) maxVal = std::max(maxVal, valueForId(id));
    if (maxVal <= 0.0) maxVal = 1.0;

    // Draw axes
    painter.setPen(QPen(painter.pen().color(), 1.0));
    painter.drawLine(plotArea.bottomLeft().toPoint(), plotArea.bottomRight().toPoint());
    painter.drawLine(plotArea.bottomLeft().toPoint(), plotArea.topLeft().toPoint());

    // Y-axis label
    painter.save();
    painter.translate(rect.left() + 12, rect.center().y());
    painter.rotate(-90);
    QFont axisFont = painter.font();
    axisFont.setPointSizeF(axisFont.pointSizeF() * 0.8);
    painter.setFont(axisFont);
    painter.drawText(QRect(-60, -10, 120, 20), Qt::AlignCenter, yAxisLabel);
    painter.restore();

    // Y-axis ticks
    {
        QFont tickFont = painter.font();
        tickFont.setPointSizeF(tickFont.pointSizeF() * 0.75);
        painter.setFont(tickFont);
        for (int t = 0; t <= 4; ++t) {
            const double val = maxVal * t / 4.0;
            const qreal y = plotArea.bottom() - (val / maxVal) * plotArea.height();
            painter.drawLine(QPointF(plotArea.left() - 4, y), QPointF(plotArea.left(), y));
            painter.drawText(QRectF(rect.left(), y - 8, margin - 6, 16),
                             Qt::AlignRight | Qt::AlignVCenter,
                             QString::number(val, 'g', 3));
        }
    }

    // Bars
    const qreal barSpacing = 4.0;
    const qreal totalWidth = plotArea.width();
    const int n = wormIds.size();
    const qreal barWidth = std::max(4.0, (totalWidth - barSpacing * (n + 1)) / n);

    QFont labelFont = painter.font();
    labelFont.setPointSizeF(labelFont.pointSizeF() * 0.8);
    painter.setFont(labelFont);

    for (int i = 0; i < n; ++i) {
        const int id = wormIds[i];
        const double val = valueForId(id);
        const qreal x = plotArea.left() + barSpacing * (i + 1) + barWidth * i;
        const qreal barH = (val / maxVal) * plotArea.height();
        const QRectF bar(x, plotArea.bottom() - barH, barWidth, barH);

        QColor c = colorForId(id);
        if (!c.isValid()) c = Qt::gray;
        painter.fillRect(bar, c);
        painter.setPen(QPen(c.darker(150), 0.5));
        painter.drawRect(bar);

        // X label
        painter.setPen(painter.pen().color());
        painter.drawText(QRectF(x, plotArea.bottom() + 2, barWidth, 18),
                         Qt::AlignHCenter | Qt::AlignTop,
                         labelForId(id));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// AnalysisPanel
// ─────────────────────────────────────────────────────────────────────────────

const char* AnalysisPanel::kPlotNames[AnalysisPanel::kPlotCount] = {
    "Tracks XY",
    "Tracks XY (Speed)",
    "Speed Timeline",
    "Average Speed",
    "Reversals"
};

AnalysisPanel::AnalysisPanel(TrackingDataStorage* storage, QObject* parent)
    : QObject(parent), m_storage(storage)
{
    // The Analysis tab is intentionally decoupled from the Processing tab's
    // real-time storage signals.  The session model is populated from disk only
    // at well-defined moments (setYawtDirectory), never on every worm-click.
}

void AnalysisPanel::setup(const Widgets& widgets)
{
    w = widgets;

    // Splitter sizing
    if (w.splitter) {
        w.splitter->setStretchFactor(0, 0);
        w.splitter->setStretchFactor(1, 1);
        w.splitter->setSizes({220, 600});
    }

    // Speed range spinbox limits
    if (w.speedRangeMinSpin) { w.speedRangeMinSpin->setRange(0.0, 1e6); }
    if (w.speedRangeMaxSpin) { w.speedRangeMaxSpin->setRange(0.0, 1e6); w.speedRangeMaxSpin->setValue(1.0); }

    // Session model — owns all group/video/worm data
    m_sessionModel = new AnalysisSessionModel(this);

    if (w.wormListView) {
        w.wormListView->setModel(m_sessionModel);
        w.wormListView->setHeaderHidden(true);
        w.wormListView->setRootIsDecorated(true);
        w.wormListView->setSelectionMode(QAbstractItemView::SingleSelection);
        w.wormListView->setDragEnabled(true);
        w.wormListView->setAcceptDrops(true);
        w.wormListView->setDragDropMode(QAbstractItemView::DragDrop);
        w.wormListView->setDefaultDropAction(Qt::MoveAction);
        w.wormListView->setDropIndicatorShown(true);
    }

    // "+" button → add a named group
    if (w.addGroupBtn) {
        connect(w.addGroupBtn, &QPushButton::clicked, this, [this]() {
            if (!m_sessionModel) return;
            const QString name = QInputDialog::getText(
                nullptr, "New Group", "Group name:");
            if (!name.trimmed().isEmpty()) {
                m_sessionModel->addGroup(name.trimmed());
                // Expand the new group immediately
                if (w.wormListView) {
                    const QModelIndex newGroup = m_sessionModel->index(
                        m_sessionModel->rowCount() - 1, 0);
                    w.wormListView->expand(newGroup);
                }
            }
        });
    }

    // When any worm's check state changes, forward to plots
    connect(m_sessionModel, &AnalysisSessionModel::checkedWormIdsChanged,
            this, &AnalysisPanel::onSessionCheckedChanged);

    // Plot selector
    if (w.plotSelector) {
        for (int i = 0; i < kPlotCount; ++i) {
            auto* item = new QListWidgetItem(kPlotNames[i], w.plotSelector);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(Qt::Unchecked);
        }
        connect(w.plotSelector, &QListWidget::itemChanged,
                this, &AnalysisPanel::onPlotItemChanged);
    }

    m_subWindows.resize(kPlotCount, nullptr);

    // Settings → propagate to open plots
    auto propagateAll = [this]() {
        if (!w.mdiArea) return;
        for (auto* sw : w.mdiArea->subWindowList())
            if (auto* pw = sw->widget()) propagateSettings(pw);
    };
    if (w.arenaShapeCombo)
        connect(w.arenaShapeCombo, qOverload<int>(&QComboBox::currentIndexChanged),
                this, propagateAll);
    if (w.arenaSizeSpin)
        connect(w.arenaSizeSpin, qOverload<int>(&QSpinBox::valueChanged),
                this, propagateAll);
    if (w.speedRangeCheck)
        connect(w.speedRangeCheck, &QCheckBox::toggled, this, propagateAll);
    if (w.speedRangeMinSpin)
        connect(w.speedRangeMinSpin, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, propagateAll);
    if (w.speedRangeMaxSpin)
        connect(w.speedRangeMaxSpin, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, propagateAll);
}

void AnalysisPanel::setYawtDirectory(const QString& yawtDir)
{
    if (yawtDir.isEmpty() || !m_sessionModel) return;
    m_yawtDir = yawtDir;
    m_sessionModel->scanYawtDirectory(yawtDir);
    if (w.wormListView) {
        w.wormListView->expandAll();
    }
}

void AnalysisPanel::setPixelSizeUmPerPixel(double v)
{
    m_umPerPixel = v;
    if (!w.mdiArea) return;
    for (auto* sw : w.mdiArea->subWindowList())
        if (auto* pw = sw->widget()) propagateSettings(pw);
}

void AnalysisPanel::setVideoFps(double v)
{
    m_videoFps = v;
    if (!w.mdiArea) return;
    for (auto* sw : w.mdiArea->subWindowList())
        if (auto* pw = sw->widget()) propagateSettings(pw);
}

void AnalysisPanel::setSelectedWormIds(const QSet<int>& ids)
{
    if (!m_sessionModel) return;
    m_selectedWormIds = ids;
    m_sessionModel->setCheckedWormIds(ids);
    // checkedWormIdsChanged → onSessionCheckedChanged will propagate to plots
}


void AnalysisPanel::onSessionCheckedChanged()
{
    if (!m_sessionModel) return;
    m_selectedWormIds = m_sessionModel->checkedWormIds();
    propagateSelectionToPlots();
    emit wormSelectionChanged(m_selectedWormIds);
}

void AnalysisPanel::onPlotItemChanged(QListWidgetItem* item)
{
    if (!w.plotSelector || !w.mdiArea) return;
    const int idx = w.plotSelector->row(item);
    if (idx < 0 || idx >= kPlotCount) return;

    if (item->checkState() == Qt::Checked) {
        QWidget* plotWidget = createPlotWidget(idx);
        if (!plotWidget) return;
        auto* sw = w.mdiArea->addSubWindow(plotWidget);
        sw->setWindowTitle(kPlotNames[idx]);
        sw->show();
        m_subWindows[idx] = sw;
    } else {
        if (m_subWindows[idx]) {
            w.mdiArea->removeSubWindow(m_subWindows[idx]);
            m_subWindows[idx]->deleteLater();
            m_subWindows[idx] = nullptr;
        }
    }
    updateMdiLayout();
}

void AnalysisPanel::propagateSelectionToPlots()
{
    // Selection is driven by AnalysisSessionModel; individual plot widgets
    // update themselves when the model signals a change.
}

void AnalysisPanel::propagateSettings(QWidget* pw)
{
    if (!pw) return;
    // All group widgets share the same minimal interface.
    if (auto* p = qobject_cast<GroupTrackXYWidget*>(pw))
        p->setVideoFps(m_videoFps);
    else if (auto* p = qobject_cast<GroupSpeedTimelineWidget*>(pw))
        p->setVideoFps(m_videoFps);
    else if (auto* p = qobject_cast<GroupSpeedBoxWidget*>(pw))
        p->setVideoFps(m_videoFps);
    else if (auto* p = qobject_cast<GroupReversalWidget*>(pw))
        p->setVideoFps(m_videoFps);
}

void AnalysisPanel::updateMdiLayout()
{
    if (w.mdiArea) w.mdiArea->tileSubWindows();
}

QWidget* AnalysisPanel::createPlotWidget(int plotIndex)
{
    QWidget* pw = nullptr;
    switch (plotIndex) {
    case 0: pw = new GroupTrackXYWidget(m_sessionModel);         break;
    case 1: pw = new GroupTrackXYWidget(m_sessionModel);         break;
    case 2: pw = new GroupSpeedTimelineWidget(m_sessionModel);   break;
    case 3: pw = new GroupSpeedBoxWidget(m_sessionModel);        break;
    case 4: pw = new GroupReversalWidget(m_sessionModel);        break;
    default: return nullptr;
    }
    propagateSettings(pw);
    return pw;
}
