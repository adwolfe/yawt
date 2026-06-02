#include "pluginplotwidget.h"

#include <QPainter>
#include <QPainterPath>
#include <QFontMetrics>
#include <QtConcurrent/QtConcurrentRun>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

// ── Box-plot statistics ────────────────────────────────────────────────────

struct BoxStats {
    double q1 = 0, median = 0, q3 = 0;
    double whiskerLow = 0, whiskerHigh = 0;
    QList<double> outliers;
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

static BoxStats computeBox(const QList<double>& vals)
{
    BoxStats bs;
    if (vals.isEmpty()) return bs;
    std::vector<double> s(vals.begin(), vals.end());
    std::sort(s.begin(), s.end());
    bs.n = static_cast<int>(s.size());
    bs.q1     = pctile(s, 0.25);
    bs.median = pctile(s, 0.50);
    bs.q3     = pctile(s, 0.75);
    const double iqr = bs.q3 - bs.q1;
    bs.whiskerLow  = bs.q1 - 1.5 * iqr;
    bs.whiskerHigh = bs.q3 + 1.5 * iqr;
    for (double v : s)
        if (v < bs.whiskerLow || v > bs.whiskerHigh) bs.outliers.append(v);
    for (double v : s)  { if (v >= bs.whiskerLow)  { bs.whiskerLow  = v; break; } }
    for (auto it = s.rbegin(); it != s.rend(); ++it)
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

    refreshData();
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

    // Take a snapshot of the data on the GUI thread.
    const auto data = m_model->getGroupedData();
    const PlotPluginSpec spec = m_spec;
    const PluginRoiPoints roi = m_roi;

    m_computing = true;
    m_pendingRefresh = false;
    update();  // paint "Computing…"

    auto future = QtConcurrent::run([spec, data, roi]() {
        return PluginEngine::evaluate(spec, data, roi);
    });
    m_watcher.setFuture(future);
}

void PluginPlotWidget::onComputationFinished()
{
    m_result = m_watcher.result();
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

// ── Y-axis helpers shared by box and bar ──────────────────────────────────

static void drawYAxis(QPainter& p, const QRect& plotArea, const QRect& fullArea,
                      double yMin, double yMax, const QString& yLabel,
                      const QFontMetrics& fm)
{
    const int labelH = fm.height();
    const int nTicks = 5;
    p.setPen(QPen(p.device() ? QColor(160,160,160) : QColor(160,160,160), 1));
    for (int i = 0; i <= nTicks; ++i) {
        const double v = yMin + (yMax - yMin) * i / nTicks;
        const int y = plotArea.bottom()
                      - static_cast<int>((v - yMin) / (yMax - yMin) * plotArea.height());
        p.drawLine(plotArea.left() - 4, y, plotArea.right(), y);
        p.drawText(QRect(fullArea.left(), y - labelH/2, plotArea.left() - 6, labelH),
                   Qt::AlignRight | Qt::AlignVCenter, QString::number(v, 'g', 3));
    }
    if (!yLabel.isEmpty()) {
        p.save();
        p.translate(fullArea.left() + labelH, fullArea.top() + fullArea.height() / 2);
        p.rotate(-90);
        p.drawText(QRect(-fullArea.height()/2, -labelH, fullArea.height(), labelH),
                   Qt::AlignCenter, yLabel);
        p.restore();
    }
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
        for (const auto& ws : gr.worms) { yMin = std::min(yMin, ws.value); yMax = std::max(yMax, ws.value); }
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

    drawYAxis(p, plotArea, area, yMin, yMax, yLabel, fm);

    const int nGroups = groups.size();
    const int slotW = plotArea.width() / std::max(nGroups, 1);
    const int boxW  = std::min(slotW * 2 / 3, 60);

    for (int gi = 0; gi < nGroups; ++gi) {
        const auto& gr = groups[gi];
        QList<double> vals;
        for (const auto& ws : gr.worms) vals.append(ws.value);
        const BoxStats bs = computeBox(vals);
        if (bs.n == 0) continue;

        const int cx = plotArea.left() + slotW * gi + slotW / 2;
        const QColor col = (gr.worms.isEmpty() ? QColor(100,100,200) : gr.worms[0].color).lighter(130);

        // Whiskers
        p.setPen(QPen(col.darker(150), 1.5));
        p.drawLine(cx, toY(bs.whiskerLow),  cx, toY(bs.q1));
        p.drawLine(cx, toY(bs.q3), cx, toY(bs.whiskerHigh));
        const int capW = boxW / 4;
        p.drawLine(cx - capW, toY(bs.whiskerLow),  cx + capW, toY(bs.whiskerLow));
        p.drawLine(cx - capW, toY(bs.whiskerHigh), cx + capW, toY(bs.whiskerHigh));

        // Box body
        const QRect box(cx - boxW/2, toY(bs.q3), boxW, std::max(1, toY(bs.q1) - toY(bs.q3)));
        p.fillRect(box, col);
        p.setPen(QPen(col.darker(160), 1.5));
        p.drawRect(box);

        // Median
        p.setPen(QPen(col.darker(200), 2.5));
        p.drawLine(cx - boxW/2, toY(bs.median), cx + boxW/2, toY(bs.median));

        // Individual data points
        p.setPen(QPen(col.darker(180), 1));
        p.setBrush(Qt::NoBrush);
        for (const auto& ws : gr.worms) {
            const int y = toY(ws.value);
            const int jitter = gr.worms.size() > 1 ? (qHash(ws.label) % (boxW / 2)) - boxW / 4 : 0;
            p.drawEllipse(QPoint(cx + jitter, y), 3, 3);
        }

        // Group label
        p.setPen(palette().windowText().color());
        p.drawText(QRect(cx - slotW/2, plotArea.bottom() + 4, slotW, labelH),
                   Qt::AlignCenter, gr.name);
    }

    p.setPen(QPen(QColor(160,160,160), 1));
    p.drawLine(plotArea.left(), plotArea.top(), plotArea.left(), plotArea.bottom());
    p.drawLine(plotArea.left(), plotArea.bottom(), plotArea.right(), plotArea.bottom());
}

// ── Bar plot ───────────────────────────────────────────────────────────────

void PluginPlotWidget::paintBar(QPainter& p, const QRect& area)
{
    const auto& groups = m_result.groups;
    if (groups.isEmpty()) { paintStatus(p, area, "No data"); return; }

    struct GStats { QString name; double mean = 0, sd = 0; QColor color; int n = 0; };
    QList<GStats> gstats;
    double yMax = 0;
    for (const auto& gr : groups) {
        GStats gs;
        gs.name = gr.name;
        gs.color = gr.worms.isEmpty() ? QColor(100,100,200) : gr.worms[0].color;
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

    drawYAxis(p, plotArea, area, 0, yMax, yLabel, fm);

    const int nGroups = gstats.size();
    const int slotW = plotArea.width() / std::max(nGroups, 1);
    const int barW  = std::min(slotW * 2 / 3, 80);

    for (int gi = 0; gi < nGroups; ++gi) {
        const GStats& gs = gstats[gi];
        const int cx = plotArea.left() + slotW * gi + slotW / 2;
        const QColor col = gs.color.lighter(140);

        const int barTop = toY(gs.mean);
        const int barBot = toY(0);
        const QRect bar(cx - barW/2, barTop, barW, std::max(1, barBot - barTop));
        p.fillRect(bar, col);
        p.setPen(QPen(col.darker(160), 1.5));
        p.drawRect(bar);

        if (gs.sd > 0) {
            p.setPen(QPen(col.darker(200), 1.5));
            p.drawLine(cx, toY(gs.mean + gs.sd), cx, toY(gs.mean - gs.sd));
            p.drawLine(cx - 6, toY(gs.mean + gs.sd), cx + 6, toY(gs.mean + gs.sd));
            p.drawLine(cx - 6, toY(gs.mean - gs.sd), cx + 6, toY(gs.mean - gs.sd));
        }

        p.setPen(palette().windowText().color());
        p.drawText(QRect(cx - slotW/2, plotArea.bottom() + 4, slotW, labelH),
                   Qt::AlignCenter, gs.name);
    }

    p.setPen(QPen(QColor(160,160,160), 1));
    p.drawLine(plotArea.left(), plotArea.top(), plotArea.left(), plotArea.bottom());
    p.drawLine(plotArea.left(), plotArea.bottom(), plotArea.right(), plotArea.bottom());
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

    // Y axis
    p.setPen(QPen(QColor(160,160,160), 1));
    for (int i = 0; i <= 4; ++i) {
        const double v = yMin + (yMax - yMin) * i / 4;
        const int y = toY(v);
        p.drawLine(plotArea.left() - 4, y, plotArea.right(), y);
        p.drawText(QRect(area.left(), y - labelH/2, plotArea.left() - 6, labelH),
                   Qt::AlignRight | Qt::AlignVCenter, QString::number(v, 'g', 3));
    }
    if (!yLabel.isEmpty()) {
        p.save();
        p.translate(area.left() + labelH, area.top() + area.height() / 2);
        p.rotate(-90);
        p.drawText(QRect(-area.height()/2, -labelH, area.height(), labelH), Qt::AlignCenter, yLabel);
        p.restore();
    }
    if (!xLabel.isEmpty()) {
        p.setPen(palette().windowText().color());
        p.drawText(QRect(plotArea.left(), plotArea.bottom() + labelH + 4, plotArea.width(), labelH),
                   Qt::AlignCenter, xLabel);
    }

    // Lines
    p.setClipRect(plotArea);
    for (const auto& ws : series) {
        if (ws.points.size() < 2) continue;
        p.setPen(QPen(ws.color, 1.5));
        QPainterPath path;
        path.moveTo(toX(ws.points[0].x()), toY(ws.points[0].y()));
        for (int i = 1; i < ws.points.size(); ++i)
            path.lineTo(toX(ws.points[i].x()), toY(ws.points[i].y()));
        p.drawPath(path);
    }
    p.setClipping(false);

    p.setPen(QPen(QColor(160,160,160), 1));
    p.drawLine(plotArea.left(), plotArea.top(), plotArea.left(), plotArea.bottom());
    p.drawLine(plotArea.left(), plotArea.bottom(), plotArea.right(), plotArea.bottom());
}
