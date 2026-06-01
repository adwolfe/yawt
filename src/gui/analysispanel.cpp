#include "analysispanel.h"
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
#include <QLabel>
#include <QListWidget>
#include <QMdiArea>
#include <QMdiSubWindow>
#include <QPixmap>
#include <QPainter>
#include <QPainterPath>
#include <QScrollBar>
#include <QSpinBox>
#include <QSplitter>
#include <QStandardItem>
#include <QStandardItemModel>
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

// ─────────────────────────────────────────────────────────────────────────────
// Shared helpers
// ─────────────────────────────────────────────────────────────────────────────

static QColor colorForIdInList(int id, const QList<TableItems::ClickedItem>& items)
{
    for (const auto& item : items) {
        if (item.id == id) return item.color;
    }
    return QColor();
}

// Compute per-frame smoothed speed (um/s or px/s) using a 2-second sliding window.
// Returns vector of (frameNumber, speed) for valid points.
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
// SpeedTimelineWidget
// ─────────────────────────────────────────────────────────────────────────────

SpeedTimelineWidget::SpeedTimelineWidget(TrackingDataStorage* storage, QWidget* parent)
    : QWidget(parent), m_storage(storage)
{
    setMinimumSize(320, 200);
    if (m_storage) {
        connect(m_storage, &TrackingDataStorage::itemsChanged,
                this, [this](const QList<TableItems::ClickedItem>& items) {
                    refreshItems(items); update();
                });
        connect(m_storage, &TrackingDataStorage::allDataChanged, this, [this]{ update(); });
        connect(m_storage, &TrackingDataStorage::trackAdded,    this, [this](int){ update(); });
        connect(m_storage, &TrackingDataStorage::trackRemoved,  this, [this](int){ update(); });
        refreshItems(m_storage->getAllItems());
    }
}

void SpeedTimelineWidget::setPixelSizeUmPerPixel(double v) { m_umPerPixel = v; update(); }
void SpeedTimelineWidget::setVideoFps(double v)            { m_videoFps   = v; update(); }

void SpeedTimelineWidget::setVisibleWormIds(const QSet<int>& ids)
{
    m_visibleWormIds = ids;
    update();
}

void SpeedTimelineWidget::refreshItems(const QList<TableItems::ClickedItem>& items)
{
    m_items = items;
}

QColor SpeedTimelineWidget::colorForId(int id) const
{
    return colorForIdInList(id, m_items);
}

void SpeedTimelineWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.fillRect(rect(), palette().window());
    painter.setRenderHint(QPainter::Antialiasing, true);

    if (!m_storage) return;
    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    if (tracks.empty()) {
        painter.setPen(palette().text().color());
        painter.drawText(rect(), Qt::AlignCenter, tr("No tracks to display"));
        return;
    }

    const bool filter = !m_visibleWormIds.isEmpty();

    // Collect timelines for visible worms
    struct WormTimeline {
        int id;
        QColor color;
        std::vector<std::pair<int,double>> data; // (frame, speed)
    };
    QList<WormTimeline> timelines;
    int minFrame = std::numeric_limits<int>::max();
    int maxFrame = std::numeric_limits<int>::min();
    double maxSpeed = 0.0;

    for (const auto& [wormId, points] : tracks) {
        if (filter && !m_visibleWormIds.contains(wormId)) continue;
        auto data = computeSpeedTimeline(points, m_umPerPixel, m_videoFps);
        if (data.empty()) continue;
        QColor c = colorForId(wormId);
        if (!c.isValid()) c = palette().highlight().color();
        for (auto& [f, s] : data) {
            minFrame = std::min(minFrame, f);
            maxFrame = std::max(maxFrame, f);
            maxSpeed = std::max(maxSpeed, s);
        }
        timelines.push_back({wormId, c, std::move(data)});
    }

    if (timelines.empty()) {
        painter.setPen(palette().text().color());
        painter.drawText(rect(), Qt::AlignCenter, tr("No visible worms"));
        return;
    }

    if (maxSpeed <= 0.0) maxSpeed = 1.0;
    if (minFrame == maxFrame) maxFrame = minFrame + 1;

    const qreal lMargin = 52.0, rMargin = 12.0, tMargin = 12.0, bMargin = 32.0;
    QRectF plot(lMargin, tMargin, width() - lMargin - rMargin, height() - tMargin - bMargin);
    if (plot.width() <= 0 || plot.height() <= 0) return;

    // Axes
    painter.setPen(QPen(palette().text().color(), 1.0));
    painter.drawLine(plot.bottomLeft().toPoint(), plot.bottomRight().toPoint());
    painter.drawLine(plot.bottomLeft().toPoint(), plot.topLeft().toPoint());

    // Y-axis label
    painter.save();
    {
        QFont f = painter.font(); f.setPointSizeF(f.pointSizeF() * 0.8); painter.setFont(f);
        painter.translate(10, plot.center().y());
        painter.rotate(-90);
        const QString unit = (m_umPerPixel > 0.0 && m_videoFps > 0.0) ? "um/s" : "px/s";
        painter.drawText(QRect(-50, -10, 100, 20), Qt::AlignCenter, QString("Speed (%1)").arg(unit));
    }
    painter.restore();

    // X-axis label
    painter.save();
    {
        QFont f = painter.font(); f.setPointSizeF(f.pointSizeF() * 0.8); painter.setFont(f);
        const QString xLabel = m_videoFps > 0.0 ? "Time (s)" : "Frame";
        painter.drawText(QRectF(plot.left(), plot.bottom() + 14, plot.width(), 16),
                         Qt::AlignCenter, xLabel);
    }
    painter.restore();

    // Tick marks
    {
        QFont f = painter.font(); f.setPointSizeF(f.pointSizeF() * 0.75); painter.setFont(f);
        for (int t = 0; t <= 4; ++t) {
            const double val = maxSpeed * t / 4.0;
            const qreal y = plot.bottom() - (val / maxSpeed) * plot.height();
            painter.drawLine(QPointF(plot.left() - 3, y), QPointF(plot.left(), y));
            painter.drawText(QRectF(0, y - 8, lMargin - 5, 16),
                             Qt::AlignRight | Qt::AlignVCenter,
                             QString::number(val, 'g', 3));
        }
        for (int t = 0; t <= 4; ++t) {
            const double frame = minFrame + (maxFrame - minFrame) * t / 4.0;
            const qreal x = plot.left() + (frame - minFrame) / (maxFrame - minFrame) * plot.width();
            painter.drawLine(QPointF(x, plot.bottom()), QPointF(x, plot.bottom() + 3));
            const QString lbl = m_videoFps > 0.0
                ? QString::number(frame / m_videoFps, 'f', 1)
                : QString::number(static_cast<int>(frame));
            painter.drawText(QRectF(x - 20, plot.bottom() + 4, 40, 14),
                             Qt::AlignCenter, lbl);
        }
    }

    // Plot lines
    painter.save();
    painter.setClipRect(plot);
    for (const auto& wt : timelines) {
        if (wt.data.size() < 2) continue;
        painter.setPen(QPen(wt.color, 1.5));
        QPainterPath path;
        bool first = true;
        for (auto& [f, s] : wt.data) {
            const qreal x = plot.left() + (static_cast<double>(f) - minFrame)
                            / (maxFrame - minFrame) * plot.width();
            const qreal y = plot.bottom() - (s / maxSpeed) * plot.height();
            if (first) { path.moveTo(x, y); first = false; }
            else        { path.lineTo(x, y); }
        }
        painter.drawPath(path);
    }
    painter.restore();
}

// ─────────────────────────────────────────────────────────────────────────────
// AverageSpeedWidget
// ─────────────────────────────────────────────────────────────────────────────

AverageSpeedWidget::AverageSpeedWidget(TrackingDataStorage* storage, QWidget* parent)
    : QWidget(parent), m_storage(storage)
{
    setMinimumSize(200, 160);
    if (m_storage) {
        connect(m_storage, &TrackingDataStorage::itemsChanged,
                this, [this](const QList<TableItems::ClickedItem>& items) {
                    refreshItems(items); update();
                });
        connect(m_storage, &TrackingDataStorage::allDataChanged, this, [this]{ update(); });
        connect(m_storage, &TrackingDataStorage::trackAdded,    this, [this](int){ update(); });
        connect(m_storage, &TrackingDataStorage::trackRemoved,  this, [this](int){ update(); });
        refreshItems(m_storage->getAllItems());
    }
}

void AverageSpeedWidget::setPixelSizeUmPerPixel(double v) { m_umPerPixel = v; update(); }
void AverageSpeedWidget::setVideoFps(double v)            { m_videoFps   = v; update(); }

void AverageSpeedWidget::setVisibleWormIds(const QSet<int>& ids)
{
    m_visibleWormIds = ids;
    update();
}

void AverageSpeedWidget::refreshItems(const QList<TableItems::ClickedItem>& items)
{
    m_items = items;
}

QColor AverageSpeedWidget::colorForId(int id) const
{
    return colorForIdInList(id, m_items);
}

void AverageSpeedWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setBackground(QBrush(palette().window()));
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.fillRect(rect(), palette().window());

    if (!m_storage) return;
    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    const bool filter = !m_visibleWormIds.isEmpty();

    QList<int> ids;
    for (const auto& [wormId, points] : tracks) {
        if (filter && !m_visibleWormIds.contains(wormId)) continue;
        ids.append(wormId);
    }

    if (ids.isEmpty()) {
        painter.setPen(palette().text().color());
        painter.drawText(rect(), Qt::AlignCenter, tr("No data"));
        return;
    }

    const QString unit = (m_umPerPixel > 0.0 && m_videoFps > 0.0) ? "um/s" : "px/s";

    drawBarChart(painter, rect(), ids,
        [&](int id) -> double {
            auto it = tracks.find(id);
            return (it != tracks.end()) ? computeAverageSpeed(it->second, m_umPerPixel, m_videoFps) : 0.0;
        },
        [&](int id) -> QColor { return colorForId(id); },
        [](int id) -> QString { return QString("W%1").arg(id); },
        QString("Avg Speed (%1)").arg(unit));
}

// ─────────────────────────────────────────────────────────────────────────────
// ReversalCountWidget
// ─────────────────────────────────────────────────────────────────────────────

ReversalCountWidget::ReversalCountWidget(TrackingDataStorage* storage, QWidget* parent)
    : QWidget(parent), m_storage(storage)
{
    setMinimumSize(200, 160);
    if (m_storage) {
        connect(m_storage, &TrackingDataStorage::itemsChanged,
                this, [this](const QList<TableItems::ClickedItem>& items) {
                    refreshItems(items); update();
                });
        connect(m_storage, &TrackingDataStorage::allDataChanged, this, [this]{ update(); });
        connect(m_storage, &TrackingDataStorage::trackAdded,    this, [this](int){ update(); });
        connect(m_storage, &TrackingDataStorage::trackRemoved,  this, [this](int){ update(); });
        refreshItems(m_storage->getAllItems());
    }
}

void ReversalCountWidget::setPixelSizeUmPerPixel(double v) { m_umPerPixel = v; update(); }
void ReversalCountWidget::setVideoFps(double v)            { m_videoFps   = v; update(); }

void ReversalCountWidget::setVisibleWormIds(const QSet<int>& ids)
{
    m_visibleWormIds = ids;
    update();
}

void ReversalCountWidget::refreshItems(const QList<TableItems::ClickedItem>& items)
{
    m_items = items;
}

QColor ReversalCountWidget::colorForId(int id) const
{
    return colorForIdInList(id, m_items);
}

void ReversalCountWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setBackground(QBrush(palette().window()));
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.fillRect(rect(), palette().window());

    if (!m_storage) return;
    const Tracking::AllWormTracks& tracks = m_storage->getAllTracks();
    const bool filter = !m_visibleWormIds.isEmpty();

    QList<int> ids;
    for (const auto& [wormId, points] : tracks) {
        if (filter && !m_visibleWormIds.contains(wormId)) continue;
        ids.append(wormId);
    }

    if (ids.isEmpty()) {
        painter.setPen(palette().text().color());
        painter.drawText(rect(), Qt::AlignCenter, tr("No data"));
        return;
    }

    drawBarChart(painter, rect(), ids,
        [&](int id) -> double {
            auto it = tracks.find(id);
            if (it == tracks.end()) return 0.0;
            return static_cast<double>(countReversals(it->second));
        },
        [&](int id) -> QColor { return colorForId(id); },
        [](int id) -> QString { return QString("W%1").arg(id); },
        "Reversals");
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
    if (m_storage) {
        connect(m_storage, &TrackingDataStorage::itemsChanged,
                this, &AnalysisPanel::onStorageItemsChanged);
    }
}

void AnalysisPanel::setup(const Widgets& widgets)
{
    w = widgets;

    // Splitter sizing — done here since XML can't set initial sizes.
    if (w.splitter) {
        w.splitter->setStretchFactor(0, 0);
        w.splitter->setStretchFactor(1, 1);
        w.splitter->setSizes({200, 600});
    }

    // Speed range spinbox limits (too large for convenient XML entry).
    if (w.speedRangeMinSpin) { w.speedRangeMinSpin->setRange(0.0, 1e6); }
    if (w.speedRangeMaxSpin) { w.speedRangeMaxSpin->setRange(0.0, 1e6); w.speedRangeMaxSpin->setValue(1.0); }

    // Worm list model — owned by this controller.
    m_wormListModel = new QStandardItemModel(this);
    if (w.wormListView) {
        w.wormListView->setModel(m_wormListModel);
        w.wormListView->setHeaderHidden(true);
        w.wormListView->setRootIsDecorated(false);
        w.wormListView->setSelectionMode(QAbstractItemView::NoSelection);
        connect(m_wormListModel, &QStandardItemModel::itemChanged,
                this, &AnalysisPanel::onWormItemChanged);
    }

    // Populate plot selector with checkable items.
    if (w.plotSelector) {
        for (int i = 0; i < kPlotCount; ++i) {
            auto* item = new QListWidgetItem(kPlotNames[i], w.plotSelector);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(Qt::Unchecked);
        }
        connect(w.plotSelector, &QListWidget::itemChanged,
                this, &AnalysisPanel::onPlotItemChanged);
    }

    // Sub-window list initialised to nullptrs.
    m_subWindows.resize(kPlotCount, nullptr);

    // Settings → propagate to open plots.
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

    // Load initial data.
    if (m_storage)
        onStorageItemsChanged(m_storage->getAllItems());
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
    if (m_updatingSelection) return;
    m_updatingSelection = true;
    m_selectedWormIds = ids;

    // Sync worm list checkboxes
    for (int row = 0; row < m_wormListModel->rowCount(); ++row) {
        QStandardItem* item = m_wormListModel->item(row);
        if (!item) continue;
        const int wormId = item->data(Qt::UserRole).toInt();
        item->setCheckState(ids.contains(wormId) ? Qt::Checked : Qt::Unchecked);
    }

    m_updatingSelection = false;
    propagateSelectionToPlots();
}

void AnalysisPanel::onStorageItemsChanged(const QList<TableItems::ClickedItem>& items)
{
    rebuildWormList(items);
}

void AnalysisPanel::onWormItemChanged(QStandardItem* item)
{
    if (m_updatingSelection || !item) return;

    // Collect all checked worm IDs
    QSet<int> checked;
    for (int row = 0; row < m_wormListModel->rowCount(); ++row) {
        QStandardItem* it = m_wormListModel->item(row);
        if (it && it->checkState() == Qt::Checked)
            checked.insert(it->data(Qt::UserRole).toInt());
    }

    m_selectedWormIds = checked;
    propagateSelectionToPlots();
    emit wormSelectionChanged(checked);
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

void AnalysisPanel::rebuildWormList(const QList<TableItems::ClickedItem>& items)
{
    m_updatingSelection = true;
    m_wormListModel->clear();

    int wormNumber = 1;
    for (const auto& item : items) {
        if (item.type != TableItems::ItemType::Worm && item.type != TableItems::ItemType::Fix)
            continue;

        auto* row = new QStandardItem(makeColorIcon(item.color),
                                      QString("Worm %1").arg(wormNumber++));
        row->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        row->setCheckState(m_selectedWormIds.contains(item.id) ? Qt::Checked : Qt::Unchecked);
        row->setData(item.id, Qt::UserRole);
        m_wormListModel->appendRow(row);
    }
    m_updatingSelection = false;
}

void AnalysisPanel::propagateSelectionToPlots()
{
    if (!w.mdiArea) return;
    for (auto* sw : w.mdiArea->subWindowList()) {
        QWidget* pw = sw->widget();
        if (!pw) continue;
        if (auto* p = qobject_cast<TrackXYPlotWidget*>(pw))
            p->setVisibleWormIds(m_selectedWormIds);
        else if (auto* p = qobject_cast<SpeedTimelineWidget*>(pw))
            p->setVisibleWormIds(m_selectedWormIds);
        else if (auto* p = qobject_cast<AverageSpeedWidget*>(pw))
            p->setVisibleWormIds(m_selectedWormIds);
        else if (auto* p = qobject_cast<ReversalCountWidget*>(pw))
            p->setVisibleWormIds(m_selectedWormIds);
    }
}

void AnalysisPanel::propagateSettings(QWidget* pw)
{
    if (!pw) return;
    if (auto* p = qobject_cast<TrackXYPlotWidget*>(pw)) {
        if (w.arenaShapeCombo)    p->setArenaShape(w.arenaShapeCombo->currentIndex());
        if (w.arenaSizeSpin)      p->setArenaSizeMm(w.arenaSizeSpin->value());
        p->setPixelSizeUmPerPixel(m_umPerPixel);
        p->setVideoFps(m_videoFps);
        if (w.speedRangeCheck && w.speedRangeMinSpin && w.speedRangeMaxSpin)
            p->setSpeedLutRange(w.speedRangeCheck->isChecked(),
                                w.speedRangeMinSpin->value(),
                                w.speedRangeMaxSpin->value());
    } else if (auto* p = qobject_cast<SpeedTimelineWidget*>(pw)) {
        p->setPixelSizeUmPerPixel(m_umPerPixel);
        p->setVideoFps(m_videoFps);
    } else if (auto* p = qobject_cast<AverageSpeedWidget*>(pw)) {
        p->setPixelSizeUmPerPixel(m_umPerPixel);
        p->setVideoFps(m_videoFps);
    } else if (auto* p = qobject_cast<ReversalCountWidget*>(pw)) {
        p->setPixelSizeUmPerPixel(m_umPerPixel);
        p->setVideoFps(m_videoFps);
    }
}

void AnalysisPanel::updateMdiLayout()
{
    if (w.mdiArea) w.mdiArea->tileSubWindows();
}

QWidget* AnalysisPanel::createPlotWidget(int plotIndex)
{
    QWidget* w = nullptr;
    switch (plotIndex) {
    case 0: {
        auto* p = new TrackXYPlotWidget(m_storage, TrackXYPlotWidget::PlotMode::TrackColor);
        p->setVisibleWormIds(m_selectedWormIds);
        w = p;
        break;
    }
    case 1: {
        auto* p = new TrackXYPlotWidget(m_storage, TrackXYPlotWidget::PlotMode::SpeedColor);
        p->setVisibleWormIds(m_selectedWormIds);
        w = p;
        break;
    }
    case 2: {
        auto* p = new SpeedTimelineWidget(m_storage);
        p->setVisibleWormIds(m_selectedWormIds);
        w = p;
        break;
    }
    case 3: {
        auto* p = new AverageSpeedWidget(m_storage);
        p->setVisibleWormIds(m_selectedWormIds);
        w = p;
        break;
    }
    case 4: {
        auto* p = new ReversalCountWidget(m_storage);
        p->setVisibleWormIds(m_selectedWormIds);
        w = p;
        break;
    }
    default:
        return nullptr;
    }
    propagateSettings(w);
    return w;
}

QIcon AnalysisPanel::makeColorIcon(const QColor& color)
{
    QPixmap pix(14, 14);
    pix.fill(color.isValid() ? color : Qt::gray);
    return QIcon(pix);
}
