#include "analysisgroupwidgets.h"
#include "analysissessionmodel.h"
#include "trackingcommon.h"

#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QFontMetrics>
#include <QLineF>
#include <QPointF>
#include <QRectF>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Shared computation helpers
// ─────────────────────────────────────────────────────────────────────────────

/** Compute per-frame smoothed speed (2-second window). Returns {frame, speed} pairs. */
static std::vector<std::pair<int,double>> speedTimeline(
    const std::vector<Tracking::WormTrackPoint>& pts,
    double umPerPixel, double fps)
{
    std::vector<std::pair<int,double>> result;
    result.reserve(pts.size());

    const double scale  = (umPerPixel > 0.0) ? umPerPixel : 1.0;
    const bool   haveFps = (fps > 0.0);
    const int    window  = haveFps ? std::max(1, static_cast<int>(std::round(2.0 * fps))) : 1;

    bool    hasPrev = false;
    QPointF prevPos;
    int     prevFrame = 0;
    std::deque<std::pair<int,double>> win;
    double  winSum = 0.0;

    for (const auto& p : pts) {
        if (p.quality == Tracking::TrackPointQuality::Lost) {
            hasPrev = false; win.clear(); winSum = 0.0; continue;
        }
        const QPointF pos(p.position.x, p.position.y);
        if (!hasPrev) {
            result.push_back({p.frameNumberOriginal, 0.0});
            hasPrev = true; prevPos = pos; prevFrame = p.frameNumberOriginal; continue;
        }
        const int   df  = p.frameNumberOriginal - prevFrame;
        if (df <= 0) { result.push_back({p.frameNumberOriginal, 0.0}); prevPos = pos; prevFrame = p.frameNumberOriginal; continue; }
        const double d  = QLineF(prevPos, pos).length() * scale;
        const double dt = haveFps ? (static_cast<double>(df) / fps) : 1.0;
        const double spd = d / dt;

        win.emplace_back(p.frameNumberOriginal, spd);
        winSum += spd;
        while (!win.empty() && (p.frameNumberOriginal - win.front().first) > window) {
            winSum -= win.front().second; win.pop_front();
        }
        result.push_back({p.frameNumberOriginal, win.empty() ? spd : winSum / win.size()});
        prevPos = pos; prevFrame = p.frameNumberOriginal;
    }
    return result;
}

/** Average of all non-zero speed values from speedTimeline(). */
static double avgSpeed(const std::vector<Tracking::WormTrackPoint>& pts,
                       double umPerPixel, double fps)
{
    auto tl = speedTimeline(pts, umPerPixel, fps);
    double sum = 0.0; int n = 0;
    for (auto& [f, s] : tl) if (s > 0.0) { sum += s; ++n; }
    return n > 0 ? sum / n : 0.0;
}

/** Count direction reversals (>90° angle flip) with minimum displacement filter. */
static int countReversals(const std::vector<Tracking::WormTrackPoint>& pts,
                          double minDx = 2.0)
{
    std::vector<QPointF> pos;
    pos.reserve(pts.size());
    for (const auto& p : pts)
        if (p.quality != Tracking::TrackPointQuality::Lost)
            pos.push_back({p.position.x, p.position.y});
    if (pos.size() < 3) return 0;

    std::vector<QPointF> vel;
    vel.reserve(pos.size() - 1);
    for (size_t i = 1; i < pos.size(); ++i) {
        QPointF v = pos[i] - pos[i-1];
        if (QLineF(QPointF(), v).length() >= minDx) vel.push_back(v);
        else vel.push_back(vel.empty() ? QPointF() : vel.back());
    }
    int rev = 0;
    for (size_t i = 1; i < vel.size(); ++i) {
        const double dot = vel[i-1].x()*vel[i].x() + vel[i-1].y()*vel[i].y();
        const double m1  = std::hypot(vel[i-1].x(), vel[i-1].y());
        const double m2  = std::hypot(vel[i].x(),   vel[i].y());
        if (m1 < 1e-9 || m2 < 1e-9) continue;
        if (dot / (m1 * m2) < 0.0) ++rev;
    }
    return rev;
}

// ─────────────────────────────────────────────────────────────────────────────
// Box-plot statistics
// ─────────────────────────────────────────────────────────────────────────────
struct BoxStats {
    double q1 = 0, median = 0, q3 = 0;
    double whiskerLow = 0, whiskerHigh = 0;
    double mean = 0, sd = 0;
    QList<double> outliers;
    int    n = 0;
};

static double pctile(const std::vector<double>& sorted, double p)
{
    if (sorted.empty()) return 0.0;
    const double idx = p * (sorted.size() - 1);
    const int    lo  = static_cast<int>(idx);
    const int    hi  = lo + 1;
    if (hi >= static_cast<int>(sorted.size())) return sorted.back();
    return sorted[lo] + (idx - lo) * (sorted[hi] - sorted[lo]);
}

static BoxStats computeBox(QList<double> vals)
{
    BoxStats s;
    s.n = vals.size();
    if (s.n == 0) return s;

    std::vector<double> sv(vals.begin(), vals.end());
    std::sort(sv.begin(), sv.end());

    s.q1     = pctile(sv, 0.25);
    s.median = pctile(sv, 0.50);
    s.q3     = pctile(sv, 0.75);

    const double iqr = s.q3 - s.q1;
    const double lo  = s.q1 - 1.5 * iqr;
    const double hi  = s.q3 + 1.5 * iqr;

    // Whiskers: innermost points still inside lo/hi fences
    s.whiskerLow  = s.q1;
    s.whiskerHigh = s.q3;
    for (double v : sv) { if (v >= lo) { s.whiskerLow  = v; break; } }
    for (int i = static_cast<int>(sv.size())-1; i >= 0; --i)
        { if (sv[i] <= hi) { s.whiskerHigh = sv[i]; break; } }

    for (double v : sv)
        if (v < s.whiskerLow || v > s.whiskerHigh)
            s.outliers.append(v);

    const double sum = std::accumulate(sv.begin(), sv.end(), 0.0);
    s.mean = sum / s.n;
    double sq = 0;
    for (double v : sv) sq += (v - s.mean) * (v - s.mean);
    s.sd = s.n > 1 ? std::sqrt(sq / (s.n - 1)) : 0.0;
    return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// Common drawing helpers
// ─────────────────────────────────────────────────────────────────────────────

/** Draw a labelled Y-axis with ticks and a horizontal zero line. */
static void drawYAxis(QPainter& p, const QRectF& plot, double maxVal,
                      const QString& label, int ticks = 4)
{
    const QColor tc = p.pen().color();
    p.setPen(QPen(tc, 1));
    p.drawLine(plot.bottomLeft().toPoint(), plot.topLeft().toPoint());
    p.drawLine(plot.bottomLeft().toPoint(), plot.bottomRight().toPoint());

    QFont sf = p.font(); sf.setPointSizeF(sf.pointSizeF() * 0.75); p.setFont(sf);
    for (int t = 0; t <= ticks; ++t) {
        const double val = maxVal * t / ticks;
        const qreal  y   = plot.bottom() - (val / maxVal) * plot.height();
        p.drawLine(QPointF(plot.left()-4, y), QPointF(plot.left(), y));
        p.drawText(QRectF(0, y-8, plot.left()-6, 16),
                   Qt::AlignRight|Qt::AlignVCenter, QString::number(val,'g',3));
    }

    // Y-axis label (rotated)
    p.save();
    p.translate(10, plot.center().y());
    p.rotate(-90);
    QFont lf = p.font(); lf.setPointSizeF(lf.pointSizeF() / 0.75 * 0.85); p.setFont(lf);
    p.drawText(QRect(-60, -10, 120, 20), Qt::AlignCenter, label);
    p.restore();
}

/** Draw a single box-and-whisker centred at x within the plot area. */
static void drawBox(QPainter& p, double cx, double boxW,
                    const BoxStats& s, double maxVal, const QRectF& plot,
                    const QColor& col)
{
    if (s.n == 0 || maxVal <= 0) return;

    auto toY = [&](double v) {
        return plot.bottom() - (v / maxVal) * plot.height();
    };

    const double x1 = cx - boxW * 0.5;
    const double x2 = cx + boxW * 0.5;
    const QRectF box(x1, toY(s.q3), boxW, toY(s.q1) - toY(s.q3));

    // Filled box
    QColor fill = col; fill.setAlphaF(0.35);
    p.fillRect(box, fill);
    p.setPen(QPen(col, 1.5));
    p.drawRect(box);

    // Median line
    p.setPen(QPen(col.darker(150), 2.0));
    const double my = toY(s.median);
    p.drawLine(QPointF(x1, my), QPointF(x2, my));

    // Whiskers
    p.setPen(QPen(col, 1.0, Qt::DashLine));
    p.drawLine(QPointF(cx, toY(s.q1)),         QPointF(cx, toY(s.whiskerLow)));
    p.drawLine(QPointF(cx, toY(s.q3)),         QPointF(cx, toY(s.whiskerHigh)));
    p.setPen(QPen(col, 1.5));
    p.drawLine(QPointF(x1+4, toY(s.whiskerLow)),  QPointF(x2-4, toY(s.whiskerLow)));
    p.drawLine(QPointF(x1+4, toY(s.whiskerHigh)), QPointF(x2-4, toY(s.whiskerHigh)));

    // Outlier dots
    p.setPen(QPen(col, 1.0));
    for (double v : s.outliers) {
        const double oy = toY(v);
        p.drawEllipse(QPointF(cx, oy), 3.0, 3.0);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Speed colour helper (for speed-coloured tracks)
// ─────────────────────────────────────────────────────────────────────────────
static QColor speedColor(double spd, double maxSpd)
{
    const double t = (maxSpd > 0) ? std::min(1.0, spd / maxSpd) : 0.0;
    // Blue (cold) → Green → Yellow → Red (hot)
    const double hue = (1.0 - t) * 0.667;   // 0.667 = blue, 0 = red
    return QColor::fromHsvF(hue, 0.9, 0.9);
}

// ─────────────────────────────────────────────────────────────────────────────
// GroupTrackXYWidget
// ─────────────────────────────────────────────────────────────────────────────

GroupTrackXYWidget::GroupTrackXYWidget(AnalysisSessionModel* model, Mode mode, QWidget* parent)
    : QWidget(parent), m_model(model), m_mode(mode)
{
    setMinimumSize(320, 240);
    if (m_model) {
        connect(m_model, &QAbstractItemModel::modelReset,
                this, &GroupTrackXYWidget::refreshData);
        connect(m_model, &AnalysisSessionModel::checkedWormIdsChanged,
                this, &GroupTrackXYWidget::refreshData);
        refreshData();
    }
}

void GroupTrackXYWidget::setVideoFps(double fps) { m_fps = fps; update(); }

void GroupTrackXYWidget::refreshData()
{
    if (m_model) m_data = m_model->getGroupedData();
    update();
}

void GroupTrackXYWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), palette().window());
    p.setRenderHint(QPainter::Antialiasing, true);

    if (m_data.isEmpty()) {
        p.setPen(palette().text().color());
        p.drawText(rect(), Qt::AlignCenter, "No data — add videos to groups");
        return;
    }

    // Compute bounding box of all track points
    double minX =  1e18, minY =  1e18;
    double maxX = -1e18, maxY = -1e18;
    for (const auto& g : m_data)
        for (const auto& w : g.worms)
            for (const auto& pt : w.points)
                if (pt.quality != Tracking::TrackPointQuality::Lost) {
                    minX = std::min(minX, (double)pt.position.x);
                    minY = std::min(minY, (double)pt.position.y);
                    maxX = std::max(maxX, (double)pt.position.x);
                    maxY = std::max(maxY, (double)pt.position.y);
                }
    if (minX >= maxX || minY >= maxY) {
        p.drawText(rect(), Qt::AlignCenter, "No valid track points");
        return;
    }

    const qreal  margin = 12.0;
    const QRectF plot(margin, margin,
                      width()  - 2*margin,
                      height() - 2*margin);

    // Axes
    p.setPen(QPen(palette().text().color(), 1));
    p.drawRect(plot);

    const double rx = (maxX - minX), ry = (maxY - minY);

    auto toPlot = [&](float px, float py) -> QPointF {
        return QPointF(plot.left() + (px - minX) / rx * plot.width(),
                       plot.top()  + (py - minY) / ry * plot.height());
    };

    // Collect speed ranges for SpeedColor mode
    double maxSpd = 0.0;
    if (m_mode == Mode::SpeedColor) {
        for (const auto& g : m_data)
            for (const auto& w : g.worms) {
                auto tl = speedTimeline(w.points, w.umPerPixel, m_fps);
                for (auto& [f, s] : tl) maxSpd = std::max(maxSpd, s);
            }
        if (maxSpd <= 0) maxSpd = 1.0;
    }

    // Draw tracks
    for (const auto& g : m_data) {
        for (const auto& w : g.worms) {
            if (m_mode == Mode::TrackColor) {
                // Solid line in group colour
                p.setPen(QPen(w.color, 1.2));
                QPainterPath path;
                bool first = true;
                for (const auto& pt : w.points) {
                    if (pt.quality == Tracking::TrackPointQuality::Lost) { first = true; continue; }
                    const QPointF qp = toPlot(pt.position.x, pt.position.y);
                    if (first) { path.moveTo(qp); first = false; }
                    else         path.lineTo(qp);
                }
                p.drawPath(path);
            } else {
                // Segment-by-segment speed coloring
                auto tl = speedTimeline(w.points, w.umPerPixel, m_fps);
                // Build (point, speed) pairs
                std::vector<std::pair<QPointF,double>> pts2;
                {
                    size_t si = 0;
                    for (const auto& pt : w.points) {
                        if (pt.quality == Tracking::TrackPointQuality::Lost) continue;
                        const double spd = (si < tl.size()) ? tl[si].second : 0.0;
                        pts2.push_back({toPlot(pt.position.x, pt.position.y), spd});
                        ++si;
                    }
                }
                for (size_t i = 1; i < pts2.size(); ++i) {
                    p.setPen(QPen(speedColor((pts2[i-1].second + pts2[i].second)*0.5, maxSpd), 1.2));
                    p.drawLine(pts2[i-1].first, pts2[i].first);
                }
            }

            // Start dot
            if (!w.points.empty()) {
                for (const auto& pt : w.points)
                    if (pt.quality != Tracking::TrackPointQuality::Lost) {
                        p.setPen(QPen(w.color.darker(150), 1));
                        p.setBrush(w.color);
                        p.drawEllipse(toPlot(pt.position.x, pt.position.y), 3.0, 3.0);
                        p.setBrush(Qt::NoBrush);
                        break;
                    }
            }
        }
    }

    // Legend (group name → colour swatch)
    {
        QFont lf = p.font(); lf.setPointSizeF(lf.pointSizeF() * 0.8); p.setFont(lf);
        const QFontMetrics fm(lf);
        qreal ly = plot.top() + 6;
        for (const auto& g : m_data) {
            if (g.worms.isEmpty()) continue;
            const QColor gc = g.worms.first().color;
            p.fillRect(QRectF(plot.right()-90, ly, 12, 12), gc);
            p.setPen(palette().text().color());
            p.drawText(QRectF(plot.right()-75, ly-1, 74, 14),
                       Qt::AlignLeft|Qt::AlignVCenter,
                       fm.elidedText(g.name, Qt::ElideRight, 70));
            ly += 16;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GroupSpeedTimelineWidget
// ─────────────────────────────────────────────────────────────────────────────

GroupSpeedTimelineWidget::GroupSpeedTimelineWidget(AnalysisSessionModel* model, QWidget* parent)
    : QWidget(parent), m_model(model)
{
    setMinimumSize(320, 200);
    if (m_model) {
        connect(m_model, &QAbstractItemModel::modelReset,     this, &GroupSpeedTimelineWidget::refreshData);
        connect(m_model, &AnalysisSessionModel::checkedWormIdsChanged, this, &GroupSpeedTimelineWidget::refreshData);
        refreshData();
    }
}

void GroupSpeedTimelineWidget::setVideoFps(double fps) { m_fps = fps; update(); }

void GroupSpeedTimelineWidget::refreshData()
{
    if (m_model) m_data = m_model->getGroupedData();
    update();
}

void GroupSpeedTimelineWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), palette().window());
    p.setRenderHint(QPainter::Antialiasing, true);

    if (m_data.isEmpty()) {
        p.setPen(palette().text().color());
        p.drawText(rect(), Qt::AlignCenter, "No data");
        return;
    }

    // For each group, compute per-frame mean speed across worms
    struct GroupLine {
        QString name;
        QColor  color;
        std::map<int,double> frameSum;   // frame -> sum of speed
        std::map<int,int>    frameCount; // frame -> count
    };
    QList<GroupLine> lines;

    int minFrame = INT_MAX, maxFrame = INT_MIN;
    double maxSpd = 0.0;

    for (const auto& g : m_data) {
        if (g.worms.isEmpty()) continue;
        GroupLine gl;
        gl.name  = g.name;
        gl.color = g.worms.first().color;

        for (const auto& w : g.worms) {
            auto tl = speedTimeline(w.points, w.umPerPixel, m_fps);
            for (auto& [f, s] : tl) {
                gl.frameSum[f]   += s;
                gl.frameCount[f] += 1;
                minFrame = std::min(minFrame, f);
                maxFrame = std::max(maxFrame, f);
                maxSpd   = std::max(maxSpd, s);
            }
        }
        lines.append(std::move(gl));
    }

    if (lines.isEmpty() || minFrame >= maxFrame) {
        p.drawText(rect(), Qt::AlignCenter, "Insufficient data");
        return;
    }
    if (maxSpd <= 0) maxSpd = 1.0;

    const qreal lm = 55, rm = 14, tm = 14, bm = 32;
    QRectF plot(lm, tm, width()-lm-rm, height()-tm-bm);
    if (plot.width() <= 0 || plot.height() <= 0) return;

    drawYAxis(p, plot, maxSpd,
              m_fps > 0 ? "Speed (µm/s)" : "Speed (px/s)");

    // X-axis label
    {
        QFont sf = p.font(); sf.setPointSizeF(sf.pointSizeF()*0.8); p.setFont(sf);
        p.setPen(palette().text().color());
        p.drawText(QRectF(plot.left(), plot.bottom()+14, plot.width(), 16),
                   Qt::AlignCenter, m_fps>0 ? "Time (s)" : "Frame");
        for (int t = 0; t <= 4; ++t) {
            const double f = minFrame + (maxFrame-minFrame)*t/4.0;
            const qreal  x = plot.left() + (f-minFrame)/(maxFrame-minFrame)*plot.width();
            p.drawLine(QPointF(x, plot.bottom()), QPointF(x, plot.bottom()+3));
            const QString lbl = m_fps > 0 ? QString::number(f/m_fps,'f',1) : QString::number((int)f);
            p.drawText(QRectF(x-20, plot.bottom()+4, 40, 14), Qt::AlignCenter, lbl);
        }
    }

    // Draw mean lines per group
    p.save();
    p.setClipRect(plot);
    for (const auto& gl : lines) {
        p.setPen(QPen(gl.color, 2.0));
        QPainterPath path;
        bool first = true;
        for (auto& [f, sum] : gl.frameSum) {
            const int cnt = gl.frameCount.count(f) ? gl.frameCount.at(f) : 1;
            const double spd = sum / cnt;
            const qreal x = plot.left() + (f - minFrame) / (double)(maxFrame - minFrame) * plot.width();
            const qreal y = plot.bottom() - (spd / maxSpd) * plot.height();
            if (first) { path.moveTo(x, y); first = false; }
            else         path.lineTo(x, y);
        }
        p.drawPath(path);
    }
    p.restore();

    // Legend
    {
        QFont lf = p.font(); lf.setPointSizeF(lf.pointSizeF()*0.8); p.setFont(lf);
        const QFontMetrics fm(lf);
        qreal ly = plot.top() + 4;
        for (const auto& gl : lines) {
            p.setPen(QPen(gl.color, 2));
            p.drawLine(QPointF(plot.right()-90, ly+6), QPointF(plot.right()-76, ly+6));
            p.setPen(palette().text().color());
            p.drawText(QRectF(plot.right()-73, ly, 72, 14),
                       Qt::AlignLeft|Qt::AlignVCenter,
                       fm.elidedText(gl.name, Qt::ElideRight, 68));
            ly += 16;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GroupSpeedBoxWidget
// ─────────────────────────────────────────────────────────────────────────────

GroupSpeedBoxWidget::GroupSpeedBoxWidget(AnalysisSessionModel* model, QWidget* parent)
    : QWidget(parent), m_model(model)
{
    setMinimumSize(200, 180);
    if (m_model) {
        connect(m_model, &QAbstractItemModel::modelReset,     this, &GroupSpeedBoxWidget::refreshData);
        connect(m_model, &AnalysisSessionModel::checkedWormIdsChanged, this, &GroupSpeedBoxWidget::refreshData);
        refreshData();
    }
}

void GroupSpeedBoxWidget::setVideoFps(double fps) { m_fps = fps; update(); }

void GroupSpeedBoxWidget::refreshData()
{
    if (m_model) m_data = m_model->getGroupedData();
    update();
}

void GroupSpeedBoxWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), palette().window());
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setPen(palette().text().color());

    if (m_data.isEmpty()) {
        p.drawText(rect(), Qt::AlignCenter, "No data");
        return;
    }

    // Compute per-worm average speed for each group
    struct GroupStats { QString name; QColor color; QList<double> vals; BoxStats box; };
    QList<GroupStats> gs;
    double maxVal = 0.0;

    for (const auto& g : m_data) {
        GroupStats s;
        s.name  = g.name;
        s.color = g.worms.isEmpty() ? Qt::gray : g.worms.first().color;
        for (const auto& w : g.worms)
            s.vals.append(avgSpeed(w.points, w.umPerPixel, m_fps));
        s.box = computeBox(s.vals);
        maxVal = std::max(maxVal, s.box.whiskerHigh);
        for (double v : s.box.outliers) maxVal = std::max(maxVal, v);
        gs.append(s);
    }
    if (maxVal <= 0) maxVal = 1.0;
    maxVal *= 1.1;

    const qreal lm = 55, rm = 14, tm = 14, bm = 36;
    QRectF plot(lm, tm, width()-lm-rm, height()-tm-bm);
    if (plot.width() <= 0 || plot.height() <= 0) return;

    const QString yLabel = m_fps > 0 ? "Avg Speed (µm/s)" : "Avg Speed (px/s)";
    drawYAxis(p, plot, maxVal, yLabel);

    const int    n      = gs.size();
    const qreal  spacing = plot.width() / n;
    const qreal  boxW   = spacing * 0.5;

    QFont lf = p.font(); lf.setPointSizeF(lf.pointSizeF()*0.8); p.setFont(lf);
    const QFontMetrics fm(lf);

    for (int i = 0; i < n; ++i) {
        const qreal cx = plot.left() + spacing * (i + 0.5);
        drawBox(p, cx, boxW, gs[i].box, maxVal, plot, gs[i].color);

        // Individual data dots
        for (double v : gs[i].vals) {
            const qreal y = plot.bottom() - (v / maxVal) * plot.height();
            QColor dc = gs[i].color; dc.setAlphaF(0.45);
            p.setPen(QPen(dc, 1));
            p.drawEllipse(QPointF(cx + (qreal(rand()%7) - 3.0), y), 2.5, 2.5);
        }

        // Group name label
        p.setPen(palette().text().color());
        const QString lbl = fm.elidedText(gs[i].name, Qt::ElideRight,
                                           static_cast<int>(spacing));
        p.drawText(QRectF(cx - spacing*0.5, plot.bottom()+4, spacing, 28),
                   Qt::AlignHCenter|Qt::AlignTop, lbl);

        // n label
        p.drawText(QRectF(cx - spacing*0.5, plot.bottom()+18, spacing, 14),
                   Qt::AlignHCenter|Qt::AlignTop,
                   QString("n=%1").arg(gs[i].box.n));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GroupReversalWidget
// ─────────────────────────────────────────────────────────────────────────────

GroupReversalWidget::GroupReversalWidget(AnalysisSessionModel* model, QWidget* parent)
    : QWidget(parent), m_model(model)
{
    setMinimumSize(200, 180);
    if (m_model) {
        connect(m_model, &QAbstractItemModel::modelReset,     this, &GroupReversalWidget::refreshData);
        connect(m_model, &AnalysisSessionModel::checkedWormIdsChanged, this, &GroupReversalWidget::refreshData);
        refreshData();
    }
}

void GroupReversalWidget::setVideoFps(double fps) { m_fps = fps; update(); }

void GroupReversalWidget::refreshData()
{
    if (m_model) m_data = m_model->getGroupedData();
    update();
}

void GroupReversalWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), palette().window());
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setPen(palette().text().color());

    if (m_data.isEmpty()) {
        p.drawText(rect(), Qt::AlignCenter, "No data");
        return;
    }

    // Per-worm reversal counts → box stats per group
    struct GS { QString name; QColor color; QList<double> vals; BoxStats box; };
    QList<GS> gs;
    double maxVal = 0.0;

    for (const auto& g : m_data) {
        GS s;
        s.name  = g.name;
        s.color = g.worms.isEmpty() ? Qt::gray : g.worms.first().color;
        for (const auto& w : g.worms)
            s.vals.append(static_cast<double>(countReversals(w.points)));
        s.box = computeBox(s.vals);
        maxVal = std::max(maxVal, s.box.whiskerHigh);
        for (double v : s.box.outliers) maxVal = std::max(maxVal, v);
        gs.append(s);
    }
    if (maxVal <= 0) maxVal = 1.0;
    maxVal *= 1.1;

    const qreal lm = 55, rm = 14, tm = 14, bm = 36;
    QRectF plot(lm, tm, width()-lm-rm, height()-tm-bm);
    if (plot.width() <= 0 || plot.height() <= 0) return;

    drawYAxis(p, plot, maxVal, "Reversals");

    const int   n       = gs.size();
    const qreal spacing = plot.width() / n;
    const qreal boxW    = spacing * 0.5;

    QFont lf = p.font(); lf.setPointSizeF(lf.pointSizeF()*0.8); p.setFont(lf);
    const QFontMetrics fm(lf);

    for (int i = 0; i < n; ++i) {
        const qreal cx = plot.left() + spacing * (i + 0.5);
        drawBox(p, cx, boxW, gs[i].box, maxVal, plot, gs[i].color);

        // Individual dots (jittered)
        for (double v : gs[i].vals) {
            const qreal y = plot.bottom() - (v / maxVal) * plot.height();
            QColor dc = gs[i].color; dc.setAlphaF(0.45);
            p.setPen(QPen(dc, 1));
            p.drawEllipse(QPointF(cx + (qreal(rand()%7) - 3.0), y), 2.5, 2.5);
        }

        p.setPen(palette().text().color());
        p.drawText(QRectF(cx - spacing*0.5, plot.bottom()+4, spacing, 28),
                   Qt::AlignHCenter|Qt::AlignTop,
                   fm.elidedText(gs[i].name, Qt::ElideRight, (int)spacing));
        p.drawText(QRectF(cx - spacing*0.5, plot.bottom()+18, spacing, 14),
                   Qt::AlignHCenter|Qt::AlignTop,
                   QString("n=%1").arg(gs[i].box.n));
    }
}
