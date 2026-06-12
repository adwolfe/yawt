#include "pluginplotwidget.h"
#include "plotcolors.h"
#include "../gui/plotpainting.h"

#include <QPainter>
#include <QPainterPath>
#include <QFontMetrics>
#include <QCryptographicHash>
#include <QMutex>
#include <QMutexLocker>
#include <QTimer>
#include <QtConcurrent/QtConcurrentRun>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace {

QMutex& resultCacheMutex()
{
    static QMutex mutex;
    return mutex;
}

QHash<QString, PluginEngine::PluginResult>& resultCache()
{
    static QHash<QString, PluginEngine::PluginResult> cache;
    return cache;
}

void addHashText(QCryptographicHash& hash, const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    hash.addData(bytes);
    hash.addData(QByteArray(1, '\0'));
}

void addHashNumber(QCryptographicHash& hash, double value)
{
    addHashText(hash, QString::number(value, 'g', 17));
}

QString resultCacheKey(const PlotPluginSpec& spec,
                       quint64 dataRevision,
                       quint64 checkRevision,
                       const PluginRoiPoints& roi)
{
    QCryptographicHash hash(QCryptographicHash::Sha256);
    addHashText(hash, QString::number(dataRevision));
    addHashText(hash, QString::number(checkRevision));
    addHashText(hash, spec.filePath);
    addHashText(hash, QString::number(spec.version));
    addHashText(hash, spec.name);
    addHashText(hash, QString::number(static_cast<int>(spec.aggregate)));
    addHashText(hash, spec.formula);
    addHashText(hash, spec.filter);
    addHashText(hash, spec.reduce);
    addHashText(hash, QString::number(static_cast<int>(spec.plotType)));
    addHashText(hash, spec.yLabel);
    addHashText(hash, spec.yLabelUm);
    addHashText(hash, spec.xLabel);

    QStringList bindingKeys = spec.bindings.keys();
    bindingKeys.sort();
    for (const QString& key : bindingKeys) {
        addHashText(hash, key);
        addHashText(hash, spec.bindings.value(key));
    }

    addHashText(hash, roi.hasStart ? "start" : "no-start");
    addHashNumber(hash, roi.startX);
    addHashNumber(hash, roi.startY);
    addHashText(hash, roi.hasEnd ? "end" : "no-end");
    addHashNumber(hash, roi.endX);
    addHashNumber(hash, roi.endY);
    addHashText(hash, roi.hasCenter ? "center" : "no-center");
    addHashNumber(hash, roi.centerX);
    addHashNumber(hash, roi.centerY);

    return QString::fromLatin1(hash.result().toHex());
}

} // namespace

// ── Box-plot statistics ────────────────────────────────────────────────────

struct BoxStats {
    double q1 = 0, median = 0, q3 = 0;
    double whiskerLow = 0, whiskerHigh = 0;
    int n = 0;
};

static double pctile(const std::vector<double>& s, double p)
{
    if (s.empty()) return 0.0;
    const double idx = p * (s.size() - 1);
    const int lo = static_cast<int>(idx);
    const int hi = lo + 1;
    if (hi >= static_cast<int>(s.size())) return s.back();
    return s[lo] + (idx - lo) * (s[hi] - s[lo]);
}

static BoxStats computeBox(std::vector<double> vals)
{
    BoxStats bs;
    vals.erase(std::remove_if(vals.begin(), vals.end(),
                              [](double v) { return !std::isfinite(v); }),
               vals.end());
    if (vals.empty()) return bs;
    std::sort(vals.begin(), vals.end());
    bs.n = static_cast<int>(vals.size());
    bs.q1     = pctile(vals, 0.25);
    bs.median = pctile(vals, 0.50);
    bs.q3     = pctile(vals, 0.75);
    const double iqr = bs.q3 - bs.q1;
    bs.whiskerLow  = bs.q1 - 1.5 * iqr;
    bs.whiskerHigh = bs.q3 + 1.5 * iqr;
    for (double v : vals)  { if (v >= bs.whiskerLow)  { bs.whiskerLow  = v; break; } }
    for (auto it = vals.rbegin(); it != vals.rend(); ++it)
        { if (*it <= bs.whiskerHigh) { bs.whiskerHigh = *it; break; } }
    return bs;
}

// ── Widget ─────────────────────────────────────────────────────────────────

PluginPlotWidget::PluginPlotWidget(const PlotPluginSpec& spec,
                                   AnalysisSessionModel* model,
                                   QWidget* parent)
    : QWidget(parent), m_spec(spec), m_model(model)
{
    setMinimumSize(260, 200);

    connect(&m_watcher, &QFutureWatcher<PluginEngine::PluginResult>::finished,
            this, &PluginPlotWidget::onComputationFinished);

    if (m_model)
        connect(m_model, &AnalysisSessionModel::checkedWormIdsChanged,
                this, &PluginPlotWidget::refreshData);

    QTimer::singleShot(0, this, &PluginPlotWidget::refreshData);
}

PluginPlotWidget::~PluginPlotWidget()
{
    // Cancel any in-flight computation before destruction.
    m_watcher.cancel();
    m_watcher.waitForFinished();
}

void PluginPlotWidget::setRoiPoints(const PluginRoiPoints& roi)
{
    m_roi = roi;
    refreshData();
}

void PluginPlotWidget::refreshData()
{
    if (!m_model) return;

    // If already computing, note that another refresh is needed and return.
    if (m_computing) { m_pendingRefresh = true; return; }

    PlotPluginSpec spec = m_spec;
    const PluginRoiPoints roi = m_roi;
    const QString cacheKey = resultCacheKey(spec,
                                           m_model->dataRevision(),
                                           m_model->checkRevision(),
                                           roi);

    {
        QMutexLocker locker(&resultCacheMutex());
        const auto it = resultCache().constFind(cacheKey);
        if (it != resultCache().constEnd()) {
            m_result = it.value();
            m_computing = false;
            m_pendingRefresh = false;
            update();
            return;
        }
    }

    m_computing = true;
    m_pendingRefresh = false;
    m_activeCacheKey = cacheKey;
    update();  // paint "Computing…"

    // Take a snapshot of the data on the GUI thread after the cheap cache check.
    auto data = m_model->getGroupedData();

    auto future = QtConcurrent::run([spec = std::move(spec),
                                     data = std::move(data),
                                     roi]() {
        return PluginEngine::evaluate(spec, data, roi);
    });
    m_watcher.setFuture(future);
}

void PluginPlotWidget::onComputationFinished()
{
    m_result = m_watcher.result();
    if (!m_activeCacheKey.isEmpty()) {
        QMutexLocker locker(&resultCacheMutex());
        QHash<QString, PluginEngine::PluginResult>& cache = resultCache();
        if (cache.size() > 64) {
            cache.clear();
        }
        cache.insert(m_activeCacheKey, m_result);
    }
    m_activeCacheKey.clear();
    m_computing = false;
    update();

    // If a refresh was requested while we were computing, run it now.
    if (m_pendingRefresh)
        QMetaObject::invokeMethod(this, &PluginPlotWidget::refreshData, Qt::QueuedConnection);
}

// ── Paint dispatch ─────────────────────────────────────────────────────────

void PluginPlotWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const QRect bg = rect();
    p.fillRect(bg, palette().window());

    // Title
    QFont titleFont = font();
    titleFont.setBold(true);
    p.setFont(titleFont);
    const int titleH = QFontMetrics(titleFont).height() + 6;
    p.setPen(palette().windowText().color());
    p.drawText(QRect(0, 2, bg.width(), titleH),
               Qt::AlignHCenter | Qt::AlignTop, m_spec.name);

    const QRect area(12, titleH + 4, bg.width() - 24, bg.height() - titleH - 16);
    p.setFont(font());

    if (m_computing) {
        paintStatus(p, area, "Computing…");
        return;
    }

    if (!m_result.ok) {
        paintStatus(p, area, m_result.errorMessage.isEmpty() ? "No data" : m_result.errorMessage);
        return;
    }

    switch (m_spec.plotType) {
    case PlotPluginSpec::PlotType::Box:     paintBox(p, area);  break;
    case PlotPluginSpec::PlotType::Bar:     paintBar(p, area);  break;
    case PlotPluginSpec::PlotType::Line:    paintLine(p, area); break;
    case PlotPluginSpec::PlotType::Scatter: paintLine(p, area); break;
    }
}

void PluginPlotWidget::paintStatus(QPainter& p, const QRect& area, const QString& msg)
{
    p.setPen(palette().mid().color());
    p.drawText(area, Qt::AlignCenter | Qt::TextWordWrap, msg);
}

// ── Box plot ───────────────────────────────────────────────────────────────

void PluginPlotWidget::paintBox(QPainter& p, const QRect& area)
{
    const auto& groups = m_result.groups;
    if (groups.isEmpty()) { paintStatus(p, area, "No data"); return; }

    // Y range from all individual worm values.
    double yMin = std::numeric_limits<double>::max();
    double yMax = std::numeric_limits<double>::lowest();
    for (const auto& gr : groups)
        for (const auto& ws : gr.worms) {
            if (!std::isfinite(ws.value)) continue;
            yMin = std::min(yMin, ws.value);
            yMax = std::max(yMax, ws.value);
        }
    if (yMin == std::numeric_limits<double>::max()
            || yMax == std::numeric_limits<double>::lowest()) {
        paintStatus(p, area, "No finite data");
        return;
    }
    if (yMin >= yMax) { yMin -= 0.5; yMax += 0.5; }
    const double yPad = (yMax - yMin) * 0.1;
    yMin -= yPad; yMax += yPad;

    const QFontMetrics fm(font());
    const int labelH = fm.height();
    const QString yLabel = m_result.usedUm && !m_spec.yLabelUm.isEmpty() ? m_spec.yLabelUm : m_spec.yLabel;
    const int yAxisLabelW = yLabel.isEmpty() ? 4 : labelH + 4;
    const int bottomH = labelH + 4;
    const QRect plotArea(area.left() + 40 + yAxisLabelW, area.top() + 4,
                         area.width() - 40 - yAxisLabelW - 4,
                         area.height() - bottomH - 8);

    auto toY = [&](double v) {
        return plotArea.bottom() - static_cast<int>((v - yMin) / (yMax - yMin) * plotArea.height());
    };

    PlotPainting::drawYAxis(p, plotArea, area, yMin, yMax, yLabel);

    const int nGroups = groups.size();
    const bool singleGroup = (nGroups == 1);
    const int slotW = plotArea.width() / std::max(nGroups, 1);
    const int boxW  = std::min(slotW * 2 / 3, 60);

    for (int gi = 0; gi < nGroups; ++gi) {
        const auto& gr = groups[gi];
        std::vector<double> vals;
        vals.reserve(static_cast<size_t>(gr.worms.size()));
        for (const auto& ws : gr.worms)
            vals.push_back(ws.value);
        const BoxStats bs = computeBox(std::move(vals));
        if (bs.n == 0) continue;

        const int cx = plotArea.left() + slotW * gi + slotW / 2;
        // Single group: neutral mid-Inferno for the box structure.
        // Multiple groups: evenly spaced Inferno color per group.
        const QColor baseCol = singleGroup ? infernoColor(0.5) : plotColor(gi, nGroups);
        QColor fill = baseCol; fill.setAlphaF(0.45);

        // Whiskers (always black)
        p.setPen(QPen(Qt::black, 1.5));
        p.drawLine(cx, toY(bs.whiskerLow),  cx, toY(bs.q1));
        p.drawLine(cx, toY(bs.q3), cx, toY(bs.whiskerHigh));
        const int capW = boxW / 4;
        p.drawLine(cx - capW, toY(bs.whiskerLow),  cx + capW, toY(bs.whiskerLow));
        p.drawLine(cx - capW, toY(bs.whiskerHigh), cx + capW, toY(bs.whiskerHigh));

        // Box body — semi-transparent fill, black outline
        const QRect box(cx - boxW/2, toY(bs.q3), boxW, std::max(1, toY(bs.q1) - toY(bs.q3)));
        p.fillRect(box, fill);
        p.setPen(QPen(Qt::black, 1.5));
        p.drawRect(box);

        // Median (always black)
        p.setPen(QPen(Qt::black, 2.5));
        p.drawLine(cx - boxW/2, toY(bs.median), cx + boxW/2, toY(bs.median));

        // Individual data points — hollow circles; per-worm color (single group) or group color.
        const int nWorms = gr.worms.size();
        p.setBrush(Qt::NoBrush);
        for (int wi = 0; wi < nWorms; ++wi) {
            const auto& ws = gr.worms[wi];
            if (!std::isfinite(ws.value)) continue;
            QColor dc = singleGroup ? plotColor(wi, nWorms) : baseCol;
            dc.setAlphaF(0.65);
            p.setPen(QPen(dc, 1.5));
            const int y = toY(ws.value);
            const int jitterRange = std::max(1, boxW / 2);
            const int jitter = nWorms > 1
                ? static_cast<int>(qHash(ws.label) % static_cast<uint>(jitterRange)) - boxW / 4
                : 0;
            p.drawEllipse(QPoint(cx + jitter, y), 3, 3);
        }

        // Group label
        p.setPen(palette().windowText().color());
        p.drawText(QRect(cx - slotW/2, plotArea.bottom() + 4, slotW, labelH),
                   Qt::AlignCenter, gr.name);
    }

    PlotPainting::drawAxisFrame(p, plotArea);
}

// ── Bar plot ───────────────────────────────────────────────────────────────

void PluginPlotWidget::paintBar(QPainter& p, const QRect& area)
{
    const auto& groups = m_result.groups;
    if (groups.isEmpty()) { paintStatus(p, area, "No data"); return; }

    const int nGroupsBar = groups.size();
    struct GStats { QString name; double mean = 0, sd = 0; QColor color; int n = 0; };
    QList<GStats> gstats;
    double yMax = 0;
    for (int gi = 0; gi < nGroupsBar; ++gi) {
        const auto& gr = groups[gi];
        GStats gs;
        gs.name = gr.name;
        gs.color = plotColor(gi, nGroupsBar);
        gs.n = gr.worms.size();
        double sum = 0;
        for (const auto& ws : gr.worms) sum += ws.value;
        gs.mean = gs.n > 0 ? sum / gs.n : 0;
        double sq = 0;
        for (const auto& ws : gr.worms) sq += (ws.value - gs.mean) * (ws.value - gs.mean);
        gs.sd = gs.n > 1 ? std::sqrt(sq / (gs.n - 1)) : 0;
        yMax = std::max(yMax, gs.mean + gs.sd);
        gstats.append(gs);
    }
    if (yMax == 0) yMax = 1.0;
    yMax *= 1.15;

    const QFontMetrics fm(font());
    const int labelH = fm.height();
    const QString yLabel = m_result.usedUm && !m_spec.yLabelUm.isEmpty() ? m_spec.yLabelUm : m_spec.yLabel;
    const int yAxisLabelW = yLabel.isEmpty() ? 4 : labelH + 4;
    const QRect plotArea(area.left() + 40 + yAxisLabelW, area.top() + 4,
                         area.width() - 40 - yAxisLabelW - 4,
                         area.height() - labelH - 12);

    auto toY = [&](double v) {
        return plotArea.bottom() - static_cast<int>(v / yMax * plotArea.height());
    };

    PlotPainting::drawYAxis(p, plotArea, area, 0, yMax, yLabel);

    const int nGroups = gstats.size();
    const int slotW = plotArea.width() / std::max(nGroups, 1);
    const int barW  = std::min(slotW * 2 / 3, 80);

    for (int gi = 0; gi < nGroups; ++gi) {
        const GStats& gs = gstats[gi];
        const int cx = plotArea.left() + slotW * gi + slotW / 2;
        QColor fill = gs.color; fill.setAlphaF(0.45);

        const int barTop = toY(gs.mean);
        const int barBot = toY(0);
        const QRect bar(cx - barW/2, barTop, barW, std::max(1, barBot - barTop));
        p.fillRect(bar, fill);
        p.setPen(QPen(Qt::black, 1.5));
        p.drawRect(bar);

        if (gs.sd > 0) {
            p.setPen(QPen(Qt::black, 1.5));
            p.drawLine(cx, toY(gs.mean + gs.sd), cx, toY(gs.mean - gs.sd));
            p.drawLine(cx - 6, toY(gs.mean + gs.sd), cx + 6, toY(gs.mean + gs.sd));
            p.drawLine(cx - 6, toY(gs.mean - gs.sd), cx + 6, toY(gs.mean - gs.sd));
        }

        p.setPen(palette().windowText().color());
        p.drawText(QRect(cx - slotW/2, plotArea.bottom() + 4, slotW, labelH),
                   Qt::AlignCenter, gs.name);
    }

    PlotPainting::drawAxisFrame(p, plotArea);
}

// ── Line plot ──────────────────────────────────────────────────────────────

void PluginPlotWidget::paintLine(QPainter& p, const QRect& area)
{
    const auto& series = m_result.series;
    if (series.isEmpty()) { paintStatus(p, area, "No data"); return; }

    double xMin = std::numeric_limits<double>::max(),  xMax = std::numeric_limits<double>::lowest();
    double yMin = std::numeric_limits<double>::max(),  yMax = std::numeric_limits<double>::lowest();
    for (const auto& ws : series)
        for (const QPointF& pt : ws.points) {
            xMin = std::min(xMin, pt.x()); xMax = std::max(xMax, pt.x());
            yMin = std::min(yMin, pt.y()); yMax = std::max(yMax, pt.y());
        }
    if (xMin >= xMax) xMax = xMin + 1;
    if (yMin >= yMax) { yMin -= 0.5; yMax += 0.5; }
    const double yPad = (yMax - yMin) * 0.08;
    yMin -= yPad; yMax += yPad;

    const QFontMetrics fm(font());
    const int labelH = fm.height();
    const QString yLabel = m_result.usedUm && !m_spec.yLabelUm.isEmpty() ? m_spec.yLabelUm : m_spec.yLabel;
    const QString xLabel = m_spec.xLabel;
    const int yAxisLabelW = yLabel.isEmpty() ? 4 : labelH + 4;
    const int bottomH = labelH + (xLabel.isEmpty() ? 4 : labelH + 8);
    const QRect plotArea(area.left() + 40 + yAxisLabelW, area.top() + 4,
                         area.width() - 40 - yAxisLabelW - 4,
                         area.height() - bottomH - 8);

    auto toX = [&](double v) {
        return plotArea.left() + static_cast<int>((v - xMin) / (xMax - xMin) * plotArea.width());
    };
    auto toY = [&](double v) {
        return plotArea.bottom() - static_cast<int>((v - yMin) / (yMax - yMin) * plotArea.height());
    };

    PlotPainting::drawYAxis(p, plotArea, area, yMin, yMax, yLabel, 4);

    if (!xLabel.isEmpty()) {
        p.setPen(palette().windowText().color());
        p.drawText(QRect(plotArea.left(), plotArea.bottom() + labelH + 4, plotArea.width(), labelH),
                   Qt::AlignCenter, xLabel);
    }

    // Build color map: single group → one color per worm series; multiple groups → one color per group.
    QStringList uniqueGroups;
    for (const auto& ws : series)
        if (!uniqueGroups.contains(ws.groupName))
            uniqueGroups.append(ws.groupName);
    const bool singleGroupLine = (uniqueGroups.size() == 1);

    QHash<QString, QColor> groupColor;
    for (int gi = 0; gi < uniqueGroups.size(); ++gi)
        groupColor[uniqueGroups[gi]] = plotColor(gi, uniqueGroups.size());

    // Lines
    p.setClipRect(plotArea);
    if (singleGroupLine) {
        // Color each worm series individually across the full Turbo range.
        const int nSeries = series.size();
        for (int si = 0; si < nSeries; ++si) {
            const auto& ws = series[si];
            if (ws.points.size() < 2) continue;
            p.setPen(QPen(plotColor(si, nSeries), 1.5));
            QPainterPath path;
            path.moveTo(toX(ws.points[0].x()), toY(ws.points[0].y()));
            for (int i = 1; i < ws.points.size(); ++i)
                path.lineTo(toX(ws.points[i].x()), toY(ws.points[i].y()));
            p.drawPath(path);
        }
    } else {
        // All worms in the same group share the group's Turbo color.
        for (const auto& ws : series) {
            if (ws.points.size() < 2) continue;
            p.setPen(QPen(groupColor.value(ws.groupName), 1.5));
            QPainterPath path;
            path.moveTo(toX(ws.points[0].x()), toY(ws.points[0].y()));
            for (int i = 1; i < ws.points.size(); ++i)
                path.lineTo(toX(ws.points[i].x()), toY(ws.points[i].y()));
            p.drawPath(path);
        }
    }
    p.setClipping(false);

    PlotPainting::drawAxisFrame(p, plotArea);
}
