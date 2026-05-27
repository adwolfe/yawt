#include "debugexporter.h"

#include "debugdatastore.h"
#include "../data/trackingdatastorage.h"

#include <QDir>
#include <QFile>
#include <QTextStream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace {

constexpr int kExportScale = 4;
constexpr int kExportPad = 12;
constexpr double kTitleTextScale = 0.42;
constexpr double kLabelTextScale = 0.32;
constexpr double kEdgeTextScale = 0.28;

static float pointDistance(const cv::Point2f& a, const cv::Point2f& b)
{
    const cv::Point2f d = a - b;
    return std::sqrt(d.x * d.x + d.y * d.y);
}

static float polylineLength(const std::vector<cv::Point2f>& points)
{
    float total = 0.f;
    for (size_t i = 1; i < points.size(); ++i) {
        total += pointDistance(points[i - 1], points[i]);
    }
    return total;
}

static cv::Rect computeExportBounds(const Tracking::DetectedBlob& blob)
{
    cv::Rect bounds = cv::boundingRect(blob.contourPoints);
    bounds.x -= kExportPad;
    bounds.y -= kExportPad;
    bounds.width += 2 * kExportPad;
    bounds.height += 2 * kExportPad;
    return bounds;
}

static cv::Rect unionBounds(const cv::Rect& a, const cv::Rect& b)
{
    const int x1 = std::min(a.x, b.x);
    const int y1 = std::min(a.y, b.y);
    const int x2 = std::max(a.x + a.width, b.x + b.width);
    const int y2 = std::max(a.y + a.height, b.y + b.height);
    return cv::Rect(x1, y1, std::max(1, x2 - x1), std::max(1, y2 - y1));
}

static cv::Point worldToCanvas(const cv::Point2f& world, const cv::Rect& bounds)
{
    return cv::Point(static_cast<int>(std::lround((world.x - bounds.x) * kExportScale)),
                     static_cast<int>(std::lround((world.y - bounds.y) * kExportScale)));
}

static std::vector<cv::Point> contourToLocal(const std::vector<cv::Point>& contour,
                                             const cv::Rect& bounds)
{
    std::vector<cv::Point> local;
    local.reserve(contour.size());
    for (const cv::Point& p : contour) {
        local.push_back(cv::Point(p.x - bounds.x, p.y - bounds.y));
    }
    return local;
}

static cv::Mat makeMask(const Tracking::DetectedBlob& blob, const cv::Rect& bounds)
{
    cv::Mat mask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    std::vector<cv::Point> outer = contourToLocal(blob.contourPoints, bounds);
    if (!outer.empty()) {
        cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{outer}, cv::Scalar(255));
    }
    for (const auto& hole : blob.holeContourPoints) {
        std::vector<cv::Point> localHole = contourToLocal(hole, bounds);
        if (!localHole.empty()) {
            cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{localHole}, cv::Scalar(0));
        }
    }
    return mask;
}

static cv::Mat makeBaseCanvas(const Tracking::DetectedBlob& blob, const cv::Rect& bounds)
{
    cv::Mat mask = makeMask(blob, bounds);
    cv::Mat upMask;
    cv::resize(mask, upMask, cv::Size(), kExportScale, kExportScale, cv::INTER_NEAREST);

    cv::Mat canvas = cv::Mat::zeros(upMask.size(), CV_8UC3);
    canvas.setTo(cv::Scalar(50, 50, 50), upMask == 255);
    return canvas;
}

static void drawContoursOverlay(cv::Mat& canvas,
                                const Tracking::DetectedBlob& blob,
                                const cv::Rect& bounds)
{
    std::vector<cv::Point> outer;
    outer.reserve(blob.contourPoints.size());
    for (const cv::Point& p : blob.contourPoints) {
        outer.push_back(cv::Point((p.x - bounds.x) * kExportScale,
                                  (p.y - bounds.y) * kExportScale));
    }
    if (!outer.empty()) {
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{outer},
                      true, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
    }
    for (const auto& hole : blob.holeContourPoints) {
        std::vector<cv::Point> localHole;
        localHole.reserve(hole.size());
        for (const cv::Point& p : hole) {
            localHole.push_back(cv::Point((p.x - bounds.x) * kExportScale,
                                          (p.y - bounds.y) * kExportScale));
        }
        if (!localHole.empty()) {
            cv::polylines(canvas, std::vector<std::vector<cv::Point>>{localHole},
                          true, cv::Scalar(0, 0, 220), 1, cv::LINE_AA);
        }
    }
}

static void drawTitle(cv::Mat& canvas, const QString& title)
{
    cv::putText(canvas, title.toStdString(), cv::Point(8, 22),
                cv::FONT_HERSHEY_SIMPLEX, kTitleTextScale,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

static void drawText(cv::Mat& canvas, const QString& text, cv::Point at,
                     cv::Scalar color = cv::Scalar(255, 255, 255))
{
    cv::putText(canvas, text.toStdString(), at, cv::FONT_HERSHEY_SIMPLEX,
                kLabelTextScale, color, 1, cv::LINE_AA);
}

static void drawSmallText(cv::Mat& canvas, const QString& text, cv::Point at,
                          cv::Scalar color = cv::Scalar(210, 210, 210))
{
    const std::string s = text.toStdString();
    cv::putText(canvas, s, at, cv::FONT_HERSHEY_SIMPLEX,
                kEdgeTextScale, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::putText(canvas, s, at, cv::FONT_HERSHEY_SIMPLEX,
                kEdgeTextScale, color, 1, cv::LINE_AA);
}

static int textWidth(const QString& text, double scale)
{
    int baseline = 0;
    return cv::getTextSize(text.toStdString(), cv::FONT_HERSHEY_SIMPLEX,
                           scale, 1, &baseline).width;
}

static void drawCoordinateEdges(cv::Mat& canvas, const cv::Rect& bounds)
{
    const int xMin = bounds.x;
    const int xMax = bounds.x + bounds.width - 1;
    const int yMin = bounds.y;
    const int yMax = bounds.y + bounds.height - 1;

    const QString leftLabel = QStringLiteral("x=%1").arg(xMin);
    const QString rightLabel = QStringLiteral("x=%1").arg(xMax);
    const QString topLabel = QStringLiteral("y=%1").arg(yMin);
    const QString bottomLabel = QStringLiteral("y=%1").arg(yMax);

    drawSmallText(canvas, leftLabel, cv::Point(8, 42));
    drawSmallText(canvas, topLabel, cv::Point(8, 56));

    const int rightX = std::max(8, canvas.cols - textWidth(rightLabel, kEdgeTextScale) - 8);
    drawSmallText(canvas, rightLabel, cv::Point(rightX, 42));

    const int bottomX = std::max(8, canvas.cols - textWidth(bottomLabel, kEdgeTextScale) - 8);
    drawSmallText(canvas, bottomLabel, cv::Point(bottomX, canvas.rows - 10));
}

static void drawPolyline(cv::Mat& canvas,
                         const std::vector<cv::Point2f>& centerline,
                         const cv::Rect& bounds,
                         cv::Scalar color)
{
    if (centerline.size() < 2) {
        return;
    }

    std::vector<cv::Point> points;
    points.reserve(centerline.size());
    for (const cv::Point2f& p : centerline) {
        points.push_back(worldToCanvas(p, bounds));
    }
    cv::polylines(canvas, std::vector<std::vector<cv::Point>>{points},
                  false, color, 2, cv::LINE_AA);
    for (const cv::Point& p : points) {
        cv::circle(canvas, p, 2, color, cv::FILLED);
    }
    cv::circle(canvas, points.front(), 7, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::circle(canvas, points.back(), 7, cv::Scalar(255, 80, 0), 2, cv::LINE_AA);
}

static void writeStageImage(const cv::Mat& image, const QString& dir, const QString& name)
{
    cv::imwrite(QDir(dir).absoluteFilePath(name).toStdString(), image);
}

static QString pointString(const cv::Point2f& point)
{
    return QStringLiteral("(%1,%2)").arg(point.x, 0, 'f', 2).arg(point.y, 0, 'f', 2);
}

static QString tipSourceString(Tracking::TipCandidate::Source source)
{
    switch (source) {
    case Tracking::TipCandidate::Source::SkeletonEndpoint:
        return QStringLiteral("skel");
    case Tracking::TipCandidate::Source::CurvaturePeak:
        return QStringLiteral("curv");
    case Tracking::TipCandidate::Source::HypothesizedHidden:
        return QStringLiteral("hidden");
    default:
        return QStringLiteral("unknown");
    }
}

static bool validPoint(const cv::Point2f& point)
{
    return point.x >= 0.f && point.y >= 0.f;
}

// ── Tip cap debug images ─────────────────────────────────────────────────────
//
// Two helper functions:
//   writeTipCapOverviewStage   — full-frame view, all tips, contour coloured
//                                by cap classification (left/right/outside)
//   writeTipCapZoomStage       — per-tip zoomed close-up showing all the
//                                computed landmarks and comparison points

static void writeTipCapOverviewStage(const Tracking::DetectedBlob& blob,
                                     const Debug::CenterlineFrameDebug& record,
                                     const cv::Rect& bounds,
                                     const QString& outputDir)
{
    if (record.tipCapDebug.empty()) return;

    cv::Mat canvas = makeBaseCanvas(blob, bounds);

    // Draw each contour point, coloured by which tip-cap zone it belongs to.
    // Points can appear in multiple caps; the last colour written wins.
    // Outside-cap points use the standard green contour colour.
    std::map<std::pair<int,int>, cv::Scalar> pixelColor;
    for (const cv::Point& cp : blob.contourPoints) {
        pixelColor[{cp.x, cp.y}] = cv::Scalar(0, 180, 0); // default: outside-cap green
    }
    for (int ti = 0; ti < static_cast<int>(record.tipCapDebug.size()); ++ti) {
        const Tracking::TipCapDebug& cd = record.tipCapDebug[ti];
        if (!cd.valid) continue;
        const cv::Scalar leftColor  = cv::Scalar(255, 200,  50);  // cyan-gold: left side
        const cv::Scalar rightColor = cv::Scalar( 50, 120, 255);  // orange-red: right side
        for (const cv::Point2f& wp : cd.leftCapPoints) {
            pixelColor[{static_cast<int>(std::lround(wp.x)),
                        static_cast<int>(std::lround(wp.y))}] = leftColor;
        }
        for (const cv::Point2f& wp : cd.rightCapPoints) {
            pixelColor[{static_cast<int>(std::lround(wp.x)),
                        static_cast<int>(std::lround(wp.y))}] = rightColor;
        }
    }
    for (const auto& [xy, color] : pixelColor) {
        const cv::Point cp = worldToCanvas(cv::Point2f(static_cast<float>(xy.first),
                                                       static_cast<float>(xy.second)),
                                           bounds);
        cv::circle(canvas, cp, 2, color, cv::FILLED);
    }

    // Draw skeleton endpoint, search window arrow and lateral extents,
    // apex centroids, bilateral midpoint, and old-snap/peak for each tip.
    for (int ti = 0; ti < static_cast<int>(record.tipCapDebug.size()); ++ti) {
        const Tracking::TipCapDebug& cd = record.tipCapDebug[ti];
        if (!cd.valid) continue;
        const QString& role = (ti < static_cast<int>(record.tipCapRoles.size()))
                              ? record.tipCapRoles[ti] : QString();

        const cv::Point epCv   = worldToCanvas(cd.skelEndpoint, bounds);
        const cv::Point2f fwdEnd(cd.skelEndpoint.x + cd.outwardDir.x * cd.maxForward,
                                 cd.skelEndpoint.y + cd.outwardDir.y * cd.maxForward);
        const cv::Point2f perpDir(-cd.outwardDir.y, cd.outwardDir.x);
        const cv::Point2f sideL(cd.skelEndpoint.x + perpDir.x * cd.maxSide,
                                cd.skelEndpoint.y + perpDir.y * cd.maxSide);
        const cv::Point2f sideR(cd.skelEndpoint.x - perpDir.x * cd.maxSide,
                                cd.skelEndpoint.y - perpDir.y * cd.maxSide);

        // Outward direction arrow
        cv::arrowedLine(canvas, epCv, worldToCanvas(fwdEnd, bounds),
                        cv::Scalar(200, 200, 200), 1, cv::LINE_AA, 0, 0.15);
        // Lateral extent ticks
        cv::line(canvas, worldToCanvas(sideL, bounds), worldToCanvas(sideR, bounds),
                 cv::Scalar(80, 80, 80), 1, cv::LINE_AA);

        // Skeleton endpoint (white circle)
        cv::circle(canvas, epCv, 5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        // Old snap point (grey X)
        if (validPoint(cd.snapPoint)) {
            const cv::Point sp = worldToCanvas(cd.snapPoint, bounds);
            cv::drawMarker(canvas, sp, cv::Scalar(180, 180, 180),
                           cv::MARKER_CROSS, 9, 1, cv::LINE_AA);
        }
        // Old curvature peak (magenta X) if different from snap
        if (cd.hadPeak && validPoint(cd.peakOrSnapPoint)) {
            const cv::Point pp = worldToCanvas(cd.peakOrSnapPoint, bounds);
            cv::drawMarker(canvas, pp, cv::Scalar(255, 80, 255),
                           cv::MARKER_TILTED_CROSS, 9, 1, cv::LINE_AA);
        }

        // Left apex (cyan filled circle)
        if (cd.hasLeft && cd.hasBilateral) {
            cv::circle(canvas, worldToCanvas(cd.leftApex, bounds),
                       4, cv::Scalar(255, 220, 50), cv::FILLED);
        }
        // Right apex (orange filled circle)
        if (cd.hasRight && cd.hasBilateral) {
            cv::circle(canvas, worldToCanvas(cd.rightApex, bounds),
                       4, cv::Scalar(50, 130, 255), cv::FILLED);
        }
        // Bilateral midpoint (yellow diamond)
        if (cd.hasBilateral) {
            const cv::Point bp = worldToCanvas(cd.bilateralTip, bounds);
            const std::vector<cv::Point> diamond = {
                bp + cv::Point(0, -7), bp + cv::Point(5, 0),
                bp + cv::Point(0,  7), bp + cv::Point(-5, 0)
            };
            cv::fillConvexPoly(canvas, diamond, cv::Scalar(0, 240, 255));
            cv::polylines(canvas, std::vector<std::vector<cv::Point>>{diamond},
                          true, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
            const QString roleLabel = role.isEmpty()
                ? QStringLiteral("tip%1 bilateral").arg(ti)
                : QStringLiteral("%1 bilateral").arg(role);
            drawText(canvas, roleLabel, bp + cv::Point(8, -4), cv::Scalar(0, 240, 255));
        } else {
            // No bilateral: label the snap instead
            const QString roleLabel = role.isEmpty()
                ? QStringLiteral("tip%1 snap (no bilateral)").arg(ti)
                : QStringLiteral("%1 snap (no bilateral)").arg(role);
            drawSmallText(canvas, roleLabel,
                          worldToCanvas(cd.snapPoint, bounds) + cv::Point(8, -4));
        }
    }

    drawTitle(canvas, QStringLiteral("02c tip cap overview  "
                                     "cyan=left orange=right green=outside"));
    drawText(canvas,
             QStringLiteral("grey X = old snap  magenta X = old peak  "
                            "cyan dot = left apex  orange dot = right apex  "
                            "yellow diamond = bilateral midpoint"),
             cv::Point(8, canvas.rows - 10));
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, QStringLiteral("02c_tip_cap_overview.png"));
}

// Renders a zoomed close-up of a single tip's cap region.
// zoomScale is applied on top of kExportScale (so total = kExportScale × zoomScale).
static void writeTipCapZoomStage(const Tracking::DetectedBlob& blob,
                                 const Debug::CenterlineFrameDebug& record,
                                 const Tracking::TipCapDebug& cd,
                                 const QString& role,
                                 const QString& fileName,
                                 const QString& outputDir)
{
    if (!cd.valid) return;

    // Build a local crop centred on the skeleton endpoint.
    constexpr int kZoomScale   = 3;
    constexpr int kZoomPad     = 6; // extra world-pixel padding around cap window
    const float halfW = cd.maxSide  + static_cast<float>(kZoomPad);
    const float halfH = cd.maxForward * 1.1f + static_cast<float>(kZoomPad);

    // Bounding box in world coords (axis-aligned, generous).
    const cv::Point2f ep = cd.skelEndpoint;
    const int wxMin = static_cast<int>(std::floor(ep.x - halfW - halfH));
    const int wyMin = static_cast<int>(std::floor(ep.y - halfW - halfH));
    const int wxMax = static_cast<int>(std::ceil( ep.x + halfW + halfH));
    const int wyMax = static_cast<int>(std::ceil( ep.y + halfW + halfH));
    const cv::Rect zBounds(wxMin, wyMin,
                           std::max(1, wxMax - wxMin),
                           std::max(1, wyMax - wyMin));

    // Base canvas: mask fill + contour at kZoomScale.
    const int totalScale = kExportScale * kZoomScale;
    const int cW = zBounds.width  * totalScale;
    const int cH = zBounds.height * totalScale;

    auto worldToCrop = [&](const cv::Point2f& w) -> cv::Point {
        return cv::Point(
            static_cast<int>(std::lround((w.x - zBounds.x) * totalScale)),
            static_cast<int>(std::lround((w.y - zBounds.y) * totalScale)));
    };

    cv::Mat canvas = cv::Mat::zeros(cH, cW, CV_8UC3);

    // Render mask fill.
    {
        cv::Mat zMask = cv::Mat::zeros(zBounds.height, zBounds.width, CV_8UC1);
        std::vector<cv::Point> outer;
        outer.reserve(blob.contourPoints.size());
        for (const cv::Point& p : blob.contourPoints) {
            outer.push_back(cv::Point(p.x - zBounds.x, p.y - zBounds.y));
        }
        if (!outer.empty())
            cv::fillPoly(zMask, std::vector<std::vector<cv::Point>>{outer}, cv::Scalar(255));
        cv::Mat zMaskUp;
        cv::resize(zMask, zMaskUp, cv::Size(cW, cH), 0, 0, cv::INTER_NEAREST);
        canvas.setTo(cv::Scalar(50, 50, 50), zMaskUp == 255);
    }

    // Colour contour points by cap zone.
    for (const cv::Point& cp : blob.contourPoints) {
        const cv::Point2f wp(static_cast<float>(cp.x), static_cast<float>(cp.y));
        const cv::Point cc = worldToCrop(wp);
        if (cc.x < 0 || cc.y < 0 || cc.x >= cW || cc.y >= cH) continue;
        cv::circle(canvas, cc, 2, cv::Scalar(0, 180, 0), cv::FILLED); // default green
    }
    for (const cv::Point2f& wp : cd.leftCapPoints) {
        const cv::Point cc = worldToCrop(wp);
        if (cc.x >= 0 && cc.y >= 0 && cc.x < cW && cc.y < cH)
            cv::circle(canvas, cc, 3, cv::Scalar(255, 220, 50), cv::FILLED);
    }
    for (const cv::Point2f& wp : cd.rightCapPoints) {
        const cv::Point cc = worldToCrop(wp);
        if (cc.x >= 0 && cc.y >= 0 && cc.x < cW && cc.y < cH)
            cv::circle(canvas, cc, 3, cv::Scalar(50, 130, 255), cv::FILLED);
    }

    // Search window boundary: forward cap rectangle in the outward direction.
    {
        const cv::Point2f D  = cd.outwardDir;
        const cv::Point2f P(-D.y, D.x);
        // Four corners of the search window.
        const std::vector<cv::Point2f> corners = {
            ep + P *  cd.maxSide + D * (-1.f),
            ep + P *  cd.maxSide + D * cd.maxForward,
            ep + P * -cd.maxSide + D * cd.maxForward,
            ep + P * -cd.maxSide + D * (-1.f),
        };
        std::vector<cv::Point> cpts;
        for (const cv::Point2f& c : corners) cpts.push_back(worldToCrop(c));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{cpts},
                      true, cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
    }

    // Lateral midline at forward = 0 (skeleton endpoint level).
    {
        const cv::Point2f P(-cd.outwardDir.y, cd.outwardDir.x);
        cv::line(canvas,
                 worldToCrop(ep + P * cd.maxSide),
                 worldToCrop(ep - P * cd.maxSide),
                 cv::Scalar(70, 70, 70), 1, cv::LINE_AA);
    }

    // Outward direction arrow.
    {
        const cv::Point2f tip2(ep.x + cd.outwardDir.x * cd.maxForward,
                               ep.y + cd.outwardDir.y * cd.maxForward);
        cv::arrowedLine(canvas, worldToCrop(ep), worldToCrop(tip2),
                        cv::Scalar(200, 200, 200), 1, cv::LINE_AA, 0, 0.12);
    }

    // Skeleton endpoint (white circle).
    cv::circle(canvas, worldToCrop(ep), 6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // Old snap point (grey X).
    if (validPoint(cd.snapPoint)) {
        cv::drawMarker(canvas, worldToCrop(cd.snapPoint),
                       cv::Scalar(180, 180, 180), cv::MARKER_CROSS, 14, 1, cv::LINE_AA);
        drawSmallText(canvas, QStringLiteral("snap"),
                      worldToCrop(cd.snapPoint) + cv::Point(7, -4),
                      cv::Scalar(180, 180, 180));
    }
    // Old curvature peak (magenta tilted X) if different.
    if (cd.hadPeak && validPoint(cd.peakOrSnapPoint)) {
        cv::drawMarker(canvas, worldToCrop(cd.peakOrSnapPoint),
                       cv::Scalar(255, 80, 255), cv::MARKER_TILTED_CROSS, 14, 1, cv::LINE_AA);
        drawSmallText(canvas, QStringLiteral("peak"),
                      worldToCrop(cd.peakOrSnapPoint) + cv::Point(7, 4),
                      cv::Scalar(255, 80, 255));
    }

    // Left apex centroid (cyan filled) + right apex (orange filled).
    if (cd.hasLeft && cd.hasBilateral) {
        cv::circle(canvas, worldToCrop(cd.leftApex),
                   6, cv::Scalar(255, 220, 50), cv::FILLED);
        drawSmallText(canvas, QStringLiteral("L apex fwd=%1")
                      .arg(cd.leftPeakFwd, 0, 'f', 1),
                      worldToCrop(cd.leftApex) + cv::Point(8, 0),
                      cv::Scalar(255, 220, 50));
    }
    if (cd.hasRight && cd.hasBilateral) {
        cv::circle(canvas, worldToCrop(cd.rightApex),
                   6, cv::Scalar(50, 130, 255), cv::FILLED);
        drawSmallText(canvas, QStringLiteral("R apex fwd=%1")
                      .arg(cd.rightPeakFwd, 0, 'f', 1),
                      worldToCrop(cd.rightApex) + cv::Point(8, 0),
                      cv::Scalar(50, 130, 255));
    }

    // Bilateral midpoint (yellow diamond, large + label).
    if (cd.hasBilateral) {
        const cv::Point bp = worldToCrop(cd.bilateralTip);
        const std::vector<cv::Point> diamond = {
            bp + cv::Point(0, -10), bp + cv::Point(8, 0),
            bp + cv::Point(0,  10), bp + cv::Point(-8, 0)
        };
        cv::fillConvexPoly(canvas, diamond, cv::Scalar(0, 240, 255));
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{diamond},
                      true, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        drawSmallText(canvas, QStringLiteral("bilateral"),
                      bp + cv::Point(11, -3), cv::Scalar(0, 240, 255));
        // Line connecting the two apexes through the midpoint.
        if (cd.hasLeft && cd.hasRight) {
            cv::line(canvas, worldToCrop(cd.leftApex), worldToCrop(cd.rightApex),
                     cv::Scalar(0, 180, 180), 1, cv::LINE_AA);
        }
    }

    // Build the title line.
    const float snap2bilateral = (cd.hasBilateral && validPoint(cd.snapPoint))
        ? pointDistance(cd.bilateralTip, cd.snapPoint) : -1.f;
    const float peak2bilateral = (cd.hasBilateral && cd.hadPeak)
        ? pointDistance(cd.bilateralTip, cd.peakOrSnapPoint) : -1.f;
    QString title = QStringLiteral("02d tip cap zoom  %1  bilateral=%2  "
                                   "fwd L=%3 R=%4  sanity=%5")
        .arg(role.isEmpty() ? QStringLiteral("tip") : role)
        .arg(cd.hasBilateral ? QStringLiteral("YES") : QStringLiteral("NO"))
        .arg(cd.leftPeakFwd, 0, 'f', 1)
        .arg(cd.rightPeakFwd, 0, 'f', 1)
        .arg(cd.sanityPassed ? QStringLiteral("PASS") : QStringLiteral("FAIL"));
    if (snap2bilateral >= 0.f)
        title += QStringLiteral("  snap→bilateral=%1px").arg(snap2bilateral, 0, 'f', 2);
    if (peak2bilateral >= 0.f)
        title += QStringLiteral("  peak→bilateral=%1px").arg(peak2bilateral, 0, 'f', 2);
    drawTitle(canvas, title);

    writeStageImage(canvas, outputDir, fileName);
}

static void writeCenterlineStage(const Tracking::DetectedBlob& blob,
                                 const cv::Rect& bounds,
                                 const std::vector<cv::Point2f>& centerline,
                                 const QString& outputDir,
                                 const QString& fileName,
                                 const QString& title,
                                 cv::Scalar color)
{
    cv::Mat canvas = makeBaseCanvas(blob, bounds);
    drawContoursOverlay(canvas, blob, bounds);
    drawPolyline(canvas, centerline, bounds, color);
    drawTitle(canvas, title);
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, fileName);
}

static void writeSkeletonStage(const Tracking::DetectedBlob& blob,
                               const Debug::CenterlineFrameDebug& record,
                               const cv::Rect& bounds,
                               const QString& outputDir)
{
    cv::Mat canvas = makeBaseCanvas(blob, bounds);
    drawContoursOverlay(canvas, blob, bounds);

    for (const cv::Point2f& world : record.skeletonPixels) {
        cv::circle(canvas, worldToCanvas(world, bounds), 1,
                   cv::Scalar(255, 255, 0), cv::FILLED);
    }

    for (const cv::Point2f& world : record.skeletonEndpointPoints) {
        cv::circle(canvas, worldToCanvas(world, bounds), 6,
                   cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    drawTitle(canvas, QStringLiteral("01 skeleton  degree-1 endpoints=%1")
              .arg(record.skeletonEndpointPoints.size()));
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, QStringLiteral("01_skeleton.png"));
}

static void drawSkeletonPixels(cv::Mat& canvas,
                               const Debug::CenterlineFrameDebug& record,
                               const cv::Rect& bounds,
                               cv::Scalar color = cv::Scalar(255, 255, 0))
{
    for (const cv::Point2f& world : record.skeletonPixels) {
        cv::circle(canvas, worldToCanvas(world, bounds), 1, color, cv::FILLED);
    }
}

static void writeD3RouteKeypointsStage(const Tracking::DetectedBlob& blob,
                                       const Debug::CenterlineFrameDebug& record,
                                       const cv::Rect& bounds,
                                       const QString& outputDir)
{
    if (!record.d3RouteDebugAvailable) {
        return;
    }

    cv::Mat canvas = makeBaseCanvas(blob, bounds);
    drawContoursOverlay(canvas, blob, bounds);
    drawSkeletonPixels(canvas, record, bounds);

    auto drawPoint = [&](const cv::Point2f& point,
                         const QString& label,
                         cv::Scalar color,
                         int radius) {
        if (!validPoint(point)) {
            return;
        }
        const cv::Point cp = worldToCanvas(point, bounds);
        cv::circle(canvas, cp, radius, color, 2, cv::LINE_AA);
        cv::circle(canvas, cp, 2, color, cv::FILLED, cv::LINE_AA);
        drawText(canvas, label, cp + cv::Point(7, -5), color);
    };

    drawPoint(record.d3RouteStart,
              QStringLiteral("START(%1)").arg(record.d3RouteStartIsHead ? "H" : "T"),
              cv::Scalar(0, 255, 255), 8);
    drawPoint(record.d3RouteJunction, QStringLiteral("JUNCTION"),
              cv::Scalar(255, 0, 255), 8);
    drawPoint(record.d3RouteCenter, QStringLiteral("CENTER"),
              cv::Scalar(0, 220, 220), 7);
    drawPoint(record.d3RouteEnd, QStringLiteral("END hidden"),
              cv::Scalar(0, 180, 255), 8);

    drawTitle(canvas, QStringLiteral("04b D-3 route keypoints  cluster=%1 selected=%2%3")
              .arg(record.d3SelectedJunctionCluster)
              .arg(record.d3SelectedCandidate)
              .arg(record.d3JunctionFallbackUsed ? " fallback" : ""));
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, QStringLiteral("04b_d3_route_keypoints.png"));
}

static void writeD3CandidatePathsStage(const Tracking::DetectedBlob& blob,
                                       const Debug::CenterlineFrameDebug& record,
                                       const cv::Rect& bounds,
                                       const QString& outputDir)
{
    if (!record.d3RouteDebugAvailable || record.d3CandidatePaths.empty()) {
        return;
    }

    cv::Mat canvas = makeBaseCanvas(blob, bounds);
    drawContoursOverlay(canvas, blob, bounds);
    drawSkeletonPixels(canvas, record, bounds, cv::Scalar(120, 120, 0));

    const std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 120, 0),
        cv::Scalar(0, 180, 255),
        cv::Scalar(180, 80, 255),
        cv::Scalar(255, 255, 80)
    };
    for (int i = 0; i < static_cast<int>(record.d3CandidatePaths.size()); ++i) {
        const bool selected = i == record.d3SelectedCandidate;
        const cv::Scalar color = selected
            ? cv::Scalar(0, 255, 0)
            : colors[static_cast<size_t>(i) % colors.size()];
        const std::vector<cv::Point2f>& path = record.d3CandidatePaths[i];
        if (path.size() < 2) {
            continue;
        }
        std::vector<cv::Point> points;
        points.reserve(path.size());
        for (const cv::Point2f& p : path) {
            points.push_back(worldToCanvas(p, bounds));
        }
        cv::polylines(canvas, std::vector<std::vector<cv::Point>>{points},
                      false, color, selected ? 3 : 2, cv::LINE_AA);
        for (const cv::Point& p : points) {
            cv::circle(canvas, p, selected ? 2 : 1, color, cv::FILLED);
        }
        drawText(canvas,
                 QStringLiteral("P%1%2").arg(i).arg(selected ? " SELECTED" : ""),
                 points.back() + cv::Point(7, 5),
                 color);
    }

    if (validPoint(record.d3RouteStart)) {
        cv::circle(canvas, worldToCanvas(record.d3RouteStart, bounds),
                   8, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
    if (validPoint(record.d3RouteEnd)) {
        cv::circle(canvas, worldToCanvas(record.d3RouteEnd, bounds),
                   8, cv::Scalar(0, 180, 255), 2, cv::LINE_AA);
    }
    if (validPoint(record.d3RouteJunction)) {
        cv::circle(canvas, worldToCanvas(record.d3RouteJunction, bounds),
                   8, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    }

    drawTitle(canvas, QStringLiteral("04c D-3 possible paths  cluster=%1 selected=%2%3")
              .arg(record.d3SelectedJunctionCluster)
              .arg(record.d3SelectedCandidate)
              .arg(record.d3JunctionFallbackUsed ? " fallback" : ""));
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, QStringLiteral("04c_d3_possible_paths.png"));
}

static void writeDistanceTransformStage(const Debug::CenterlineFrameDebug& record,
                                        const cv::Rect& bounds,
                                        const QString& outputDir)
{
    if (record.distanceTransform.empty()) {
        cv::Mat canvas = cv::Mat::zeros(bounds.height * kExportScale,
                                        bounds.width * kExportScale,
                                        CV_8UC3);
        drawTitle(canvas, QStringLiteral("02 distance transform unavailable"));
        drawCoordinateEdges(canvas, bounds);
        writeStageImage(canvas, outputDir, QStringLiteral("02_distance_transform.png"));
        return;
    }

    cv::Mat dt8;
    double maxValue = 0.0;
    for (float value : record.distanceTransform.values) {
        maxValue = std::max(maxValue, static_cast<double>(value));
    }
    cv::Mat dtSource(record.distanceTransform.rows, record.distanceTransform.cols, CV_32F);
    for (int y = 0; y < record.distanceTransform.rows; ++y) {
        float* row = dtSource.ptr<float>(y);
        for (int x = 0; x < record.distanceTransform.cols; ++x) {
            row[x] = record.distanceTransform.at(y, x);
        }
    }
    dtSource.convertTo(dt8, CV_8U, maxValue > 0.0 ? 255.0 / maxValue : 0.0);

    cv::Mat dtFull = cv::Mat::zeros(bounds.height, bounds.width, CV_8U);
    const int dx = record.distanceTransform.localBounds.x - bounds.x;
    const int dy = record.distanceTransform.localBounds.y - bounds.y;
    for (int y = 0; y < dt8.rows; ++y) {
        const int dstY = y + dy;
        if (dstY < 0 || dstY >= dtFull.rows) {
            continue;
        }
        for (int x = 0; x < dt8.cols; ++x) {
            const int dstX = x + dx;
            if (dstX < 0 || dstX >= dtFull.cols) {
                continue;
            }
            dtFull.at<uchar>(dstY, dstX) = dt8.at<uchar>(y, x);
        }
    }

    cv::Mat dtUp;
    cv::resize(dtFull, dtUp, cv::Size(), kExportScale, kExportScale,
               cv::INTER_NEAREST);
    cv::Mat heat;
    cv::applyColorMap(dtUp, heat, cv::COLORMAP_INFERNO);
    drawTitle(heat, QStringLiteral("02 distance transform  max=%1px")
              .arg(maxValue, 0, 'f', 1));
    drawCoordinateEdges(heat, bounds);
    writeStageImage(heat, outputDir, QStringLiteral("02_distance_transform.png"));
}

static void writeContourCurvatureStage(const Tracking::DetectedBlob& blob,
                                       const Debug::CenterlineFrameDebug& record,
                                       const cv::Rect& bounds,
                                       const QString& outputDir)
{
    if (record.contourCurvaturePoints.empty() ||
        record.contourCurvatures.size() != record.contourCurvaturePoints.size()) {
        return;
    }

    cv::Mat canvas = makeBaseCanvas(blob, bounds);
    drawContoursOverlay(canvas, blob, bounds);

    float maxAbs = 0.f;
    for (float k : record.contourCurvatures) {
        maxAbs = std::max(maxAbs, std::abs(k));
    }
    if (maxAbs < 1e-6f) {
        maxAbs = 1.f;
    }

    auto curvatureColor = [&](float k) -> cv::Scalar {
        const float scaled = std::min(std::abs(k) / maxAbs, 1.f);
        cv::Mat value(1, 1, CV_8UC1, cv::Scalar(static_cast<int>(std::lround(255.f * scaled))));
        cv::Mat color;
        cv::applyColorMap(value, color, cv::COLORMAP_INFERNO);
        const cv::Vec3b bgr = color.at<cv::Vec3b>(0, 0);
        return cv::Scalar(bgr[0], bgr[1], bgr[2]);
    };

    for (size_t i = 0; i < record.contourCurvaturePoints.size(); ++i) {
        const cv::Point p = worldToCanvas(record.contourCurvaturePoints[i], bounds);
        cv::circle(canvas, p, 2, curvatureColor(record.contourCurvatures[i]), cv::FILLED);
    }

    for (int idx : record.contourCurvaturePeaks) {
        if (idx < 0 || idx >= static_cast<int>(record.contourCurvaturePoints.size())) {
            continue;
        }
        const cv::Point p = worldToCanvas(record.contourCurvaturePoints[idx], bounds);
        cv::rectangle(canvas, p - cv::Point(4, 4), p + cv::Point(4, 4),
                      cv::Scalar(255, 80, 255), 1, cv::LINE_AA);
    }

    for (int i = 0; i < static_cast<int>(record.tipCandidates.size()); ++i) {
        const Tracking::TipCandidate& tip = record.tipCandidates[i];
        const cv::Point p = worldToCanvas(tip.point, bounds);
        cv::circle(canvas, p, 6, cv::Scalar(80, 255, 80), 2, cv::LINE_AA);
        drawText(canvas, QStringLiteral("tip%1").arg(i), p + cv::Point(7, -5),
                 cv::Scalar(80, 255, 80));
    }

    drawTitle(canvas, QStringLiteral("02b contour curvature  max|k|=%1 peaks=%2")
              .arg(maxAbs, 0, 'f', 3)
              .arg(record.contourCurvaturePeaks.size()));
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, QStringLiteral("02b_contour_curvature.png"));
}

static void writeTipStage(const Tracking::DetectedBlob& blob,
                          const Debug::CenterlineFrameDebug& record,
                          const cv::Rect& bounds,
                          const QString& outputDir)
{
    cv::Mat canvas = makeBaseCanvas(blob, bounds);
    drawContoursOverlay(canvas, blob, bounds);

    for (int i = 0; i < static_cast<int>(record.tipCandidates.size()); ++i) {
        const Tracking::TipCandidate& tip = record.tipCandidates[i];
        const cv::Point point = worldToCanvas(tip.point, bounds);
        cv::Scalar color(80, 255, 80);
        if (tip.source == Tracking::TipCandidate::Source::CurvaturePeak) {
            color = cv::Scalar(255, 80, 255);
        } else if (tip.source == Tracking::TipCandidate::Source::HypothesizedHidden) {
            color = cv::Scalar(0, 180, 255);
        }

        cv::circle(canvas, point, 7, color, cv::FILLED);
        cv::circle(canvas, point, 7, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        drawText(canvas,
                 QStringLiteral("tip%1 %2 |k|=%3 w=%4")
                     .arg(i)
                     .arg(tipSourceString(tip.source))
                     .arg(std::abs(tip.curvature), 0, 'f', 3)
                     .arg(tip.width, 0, 'f', 1),
                 point + cv::Point(8, -5),
                 color);
    }

    drawTitle(canvas, QStringLiteral("03 recorded tip candidates  n=%1")
              .arg(record.tipCandidates.size()));
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, QStringLiteral("03_true_tips.png"));
}

static void writeHeadTailStage(const Tracking::DetectedBlob& blob,
                               const Debug::CenterlineFrameDebug& record,
                               const cv::Rect& bounds,
                               const QString& outputDir)
{
    cv::Mat canvas = makeBaseCanvas(blob, bounds);
    drawContoursOverlay(canvas, blob, bounds);

    auto drawPrediction = [&](const cv::Point2f& point,
                              const QString& label,
                              cv::Scalar color) {
        if (!validPoint(point)) {
            return;
        }
        const cv::Point canvasPoint = worldToCanvas(point, bounds);
        cv::drawMarker(canvas, canvasPoint, color, cv::MARKER_CROSS, 13, 1, cv::LINE_AA);
        drawText(canvas, label, canvasPoint + cv::Point(7, -4), color);
    };

    drawPrediction(record.predictedHead, QStringLiteral("Hpred"), cv::Scalar(0, 0, 220));
    drawPrediction(record.predictedTail, QStringLiteral("Tpred"), cv::Scalar(220, 80, 0));
    drawPrediction(record.predictedCenter, QStringLiteral("Cpred"), cv::Scalar(0, 220, 220));
    if (record.hiddenTipHypothesized) {
        drawPrediction(record.hiddenTipTarget, QStringLiteral("hidden target"),
                       cv::Scalar(0, 180, 255));
        drawPrediction(record.hiddenTipFinal, QStringLiteral("hidden final"),
                       cv::Scalar(0, 255, 255));
    }

    auto drawAssignedTip = [&](int idx, const QString& label, cv::Scalar color) {
        if (idx < 0 || idx >= static_cast<int>(record.tipCandidates.size())) {
            return;
        }
        const cv::Point point = worldToCanvas(record.tipCandidates[idx].point, bounds);
        cv::circle(canvas, point, 10, color, 2, cv::LINE_AA);
        drawText(canvas, label, point + cv::Point(-5, 5), color);
    };
    drawAssignedTip(record.assignedHeadTipIdx, QStringLiteral("H"), cv::Scalar(0, 0, 255));
    drawAssignedTip(record.assignedTailTipIdx, QStringLiteral("T"), cv::Scalar(255, 80, 0));

    drawTitle(canvas, QStringLiteral("04b final assigned head/tail  topo=%1  branch=%2")
              .arg(Tracking::topologyStateToString(record.topology))
              .arg(Debug::centerlineBranchToString(record.branch)));
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, QStringLiteral("04b_final_assigned_head_tail.png"));
    writeStageImage(canvas, outputDir, QStringLiteral("04_head_tail.png"));
}

static void writeHiddenPredictionMaskDiffStage(const Tracking::DetectedBlob& currentBlob,
                                               const Tracking::DetectedBlob* previousBlob,
                                               const Debug::CenterlineFrameDebug& record,
                                               const cv::Rect& currentBounds,
                                               const QString& outputDir)
{
    if (!previousBlob || !previousBlob->isValid || previousBlob->contourPoints.empty() ||
        !record.hiddenTipHypothesized || !validPoint(record.hiddenTipTarget)) {
        return;
    }

    const cv::Rect previousBounds = computeExportBounds(*previousBlob);
    const cv::Rect bounds = unionBounds(currentBounds, previousBounds);
    cv::Mat currentMask = makeMask(currentBlob, bounds);
    cv::Mat previousMask = makeMask(*previousBlob, bounds);

    cv::Mat inversePrevious;
    cv::bitwise_not(previousMask, inversePrevious);
    cv::Mat enteredMask;
    cv::bitwise_and(currentMask, inversePrevious, enteredMask);

    cv::Mat upCurrent;
    cv::Mat upPrevious;
    cv::Mat upVacated;
    cv::resize(currentMask, upCurrent, cv::Size(), kExportScale, kExportScale,
               cv::INTER_NEAREST);
    cv::resize(previousMask, upPrevious, cv::Size(), kExportScale, kExportScale,
               cv::INTER_NEAREST);
    cv::resize(enteredMask, upVacated, cv::Size(), kExportScale, kExportScale,
               cv::INTER_NEAREST);

    cv::Mat canvas = cv::Mat::zeros(upCurrent.size(), CV_8UC3);
    canvas.setTo(cv::Scalar(35, 35, 35), upPrevious == 255);
    canvas.setTo(cv::Scalar(75, 75, 75), upCurrent == 255);
    canvas.setTo(cv::Scalar(0, 210, 255), upVacated == 255);

    drawContoursOverlay(canvas, *previousBlob, bounds);
    drawContoursOverlay(canvas, currentBlob, bounds);

    auto drawPoint = [&](const cv::Point2f& point,
                         const QString& label,
                         cv::Scalar color,
                         int radius,
                         int marker = cv::MARKER_CROSS) {
        if (!validPoint(point)) {
            return;
        }
        const cv::Point cp = worldToCanvas(point, bounds);
        cv::drawMarker(canvas, cp, color, marker, radius * 2, 1, cv::LINE_AA);
        cv::circle(canvas, cp, radius, color, 1, cv::LINE_AA);
        drawText(canvas, label, cp + cv::Point(7, -5), color);
    };

    const bool hiddenIsHead =
        record.assignedHeadTipIdx >= 0 &&
        record.assignedHeadTipIdx < static_cast<int>(record.tipCandidates.size()) &&
        record.tipCandidates[record.assignedHeadTipIdx].source ==
            Tracking::TipCandidate::Source::HypothesizedHidden;
    const cv::Point2f last = hiddenIsHead
        ? record.predictorBefore.lastHeadPos
        : record.predictorBefore.lastTailPos;
    const cv::Point2f velocity = hiddenIsHead
        ? record.predictorBefore.velHead
        : record.predictorBefore.velTail;
    const cv::Point2f velocityTarget = record.predictorBefore.hasVelocity
        ? last + velocity
        : last;

    drawPoint(last, hiddenIsHead ? QStringLiteral("last head") : QStringLiteral("last tail"),
              cv::Scalar(255, 255, 255), 5, cv::MARKER_TILTED_CROSS);
    drawPoint(velocityTarget, QStringLiteral("velocity target"),
              cv::Scalar(200, 80, 0), 5);

    const int totalVacatedArea = cv::countNonZero(enteredMask);
    drawPoint(record.hiddenTipTarget, QStringLiteral("hidden target"),
              cv::Scalar(0, 180, 255), 7);

    drawTitle(canvas, QStringLiteral("04a hidden prediction mask diff  yellow=prev & ~current"));
    drawText(canvas,
             QStringLiteral("current=gray previous=dark yellow=vacated area=%1 selected=%2")
                 .arg(totalVacatedArea)
                 .arg(record.hiddenTipMaskDiffSelectedArea),
             cv::Point(8, canvas.rows - 10));
    drawCoordinateEdges(canvas, bounds);
    writeStageImage(canvas, outputDir, QStringLiteral("04a_hidden_prediction_maskdiff.png"));
}

} // namespace

namespace Debug {

bool DebugExporter::exportCenterlineFrame(const TrackingDataStorage* storage,
                                          const DebugDataStore* debugStore,
                                          int wormId,
                                          int frameNumber,
                                          const QString& outputDir,
                                          QString* outErrorMsg)
{
    auto fail = [&](const QString& message) -> bool {
        if (outErrorMsg) {
            *outErrorMsg = message;
        }
        return false;
    };

    if (!storage) {
        return fail(QStringLiteral("no tracking storage"));
    }
    if (!debugStore) {
        return fail(QStringLiteral("no debug storage"));
    }
    if (outputDir.isEmpty()) {
        return fail(QStringLiteral("empty output directory"));
    }
    if (!QDir().mkpath(outputDir)) {
        return fail(QStringLiteral("could not create output directory"));
    }

    CenterlineFrameDebug record;
    if (!debugStore->getCenterlineFrame(wormId, frameNumber, record)) {
        return fail(QStringLiteral("No centerline debug record exists for worm %1 frame %2. Run/rerun centerline first.")
                    .arg(wormId).arg(frameNumber));
    }

    const QMap<int, Tracking::DetectedBlob> frameBlobs =
        storage->getDetectedBlobsForFrame(frameNumber);
    if (!frameBlobs.contains(wormId)) {
        return fail(QStringLiteral("no stored blob for worm %1 on frame %2")
                    .arg(wormId).arg(frameNumber));
    }
    const Tracking::DetectedBlob blob = frameBlobs[wormId];
    if (!blob.isValid || blob.contourPoints.empty()) {
        return fail(QStringLiteral("stored blob is invalid or has no contour"));
    }
    Tracking::DetectedBlob previousBlob;
    const int previousFrame = frameNumber - (record.directionStep == 0 ? 1 : record.directionStep);
    const QMap<int, Tracking::DetectedBlob> previousFrameBlobs =
        storage->getDetectedBlobsForFrame(previousFrame);
    const bool hasPreviousBlob =
        previousFrameBlobs.contains(wormId) &&
        previousFrameBlobs[wormId].isValid &&
        !previousFrameBlobs[wormId].contourPoints.empty();
    if (hasPreviousBlob) {
        previousBlob = previousFrameBlobs[wormId];
    }

    const cv::Rect bounds = computeExportBounds(blob);

    {
        cv::Mat canvas = makeBaseCanvas(blob, bounds);
        drawContoursOverlay(canvas, blob, bounds);
        drawTitle(canvas, QStringLiteral("00 mask  worm %1  frame %2")
                  .arg(wormId).arg(frameNumber));
        drawText(canvas,
                 QStringLiteral("outer=green holes=red  area=%1 hullArea=%2 holes=%3")
                     .arg(blob.area, 0, 'f', 0)
                     .arg(blob.convexHullArea, 0, 'f', 0)
                     .arg(blob.holeContourPoints.size()),
                 cv::Point(8, canvas.rows - 10));
        drawCoordinateEdges(canvas, bounds);
        writeStageImage(canvas, outputDir, QStringLiteral("00_mask.png"));
    }

    writeSkeletonStage(blob, record, bounds, outputDir);
    writeDistanceTransformStage(record, bounds, outputDir);
    writeContourCurvatureStage(blob, record, bounds, outputDir);
    writeTipStage(blob, record, bounds, outputDir);
    writeHeadTailStage(blob, record, bounds, outputDir);
    writeTipCapOverviewStage(blob, record, bounds, outputDir);
    // Per-tip zoomed close-ups — head first (if present), then tail.
    for (int ti = 0; ti < static_cast<int>(record.tipCapDebug.size()); ++ti) {
        const QString& role = (ti < static_cast<int>(record.tipCapRoles.size()))
                              ? record.tipCapRoles[ti] : QString();
        const QString fileSuffix = role.isEmpty()
            ? QStringLiteral("tip%1").arg(ti) : role;
        writeTipCapZoomStage(blob, record, record.tipCapDebug[ti],
                             role, QStringLiteral("02d_tip_cap_%1.png").arg(fileSuffix),
                             outputDir);
    }
    writeHiddenPredictionMaskDiffStage(blob,
                                       hasPreviousBlob ? &previousBlob : nullptr,
                                       record,
                                       bounds,
                                       outputDir);
    writeD3RouteKeypointsStage(blob, record, bounds, outputDir);
    writeD3CandidatePathsStage(blob, record, bounds, outputDir);

    writeCenterlineStage(blob, bounds, record.initialCenterline, outputDir,
                         QStringLiteral("05_initial_centerline.png"),
                         QStringLiteral("05 initial centerline  %1")
                             .arg(centerlineBranchToString(record.branch)),
                         cv::Scalar(0, 200, 255));
    writeCenterlineStage(blob, bounds, record.resampledCenterline, outputDir,
                         QStringLiteral("06_resampled.png"),
                         QStringLiteral("06 resampled centerline"),
                         cv::Scalar(200, 200, 255));
    writeCenterlineStage(blob, bounds, record.finalCenterline, outputDir,
                         QStringLiteral("07_snake_refined.png"),
                         QStringLiteral("07 final after snake/orientation"),
                         cv::Scalar(80, 255, 255));
    writeCenterlineStage(blob, bounds, record.finalCenterline, outputDir,
                         QStringLiteral("09_final.png"),
                         QStringLiteral("09 final  arcLen=%1 px  crossSum=%2")
                             .arg(record.finalArcLength, 0, 'f', 1)
                             .arg(record.finalTurningAngle, 0, 'f', 3),
                         cv::Scalar(0, 255, 0));

    QFile logFile(QDir(outputDir).absoluteFilePath(QStringLiteral("log.txt")));
    if (!logFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return fail(QStringLiteral("could not write log.txt"));
    }

    QTextStream log(&logFile);
    log << "=== Centerline process export (recorded live state) ===\n";
    log << "worm: " << wormId << "  frame: " << frameNumber << "\n";
    log << "outputDir: " << outputDir << "\n";
    log << "directionStep: " << record.directionStep
        << "  keyframeBootstrap: " << (record.keyframeBootstrap ? "Y" : "N") << "\n";
    log << "topology: " << Tracking::topologyStateToString(record.topology)
        << "  inMergeGroup: " << (record.inMergeGroup ? "Y" : "N") << "\n";
    log << "branch: " << centerlineBranchToString(record.branch) << "\n";
    log << "flags: snakeRan=" << (record.snakeRan ? "Y" : "N")
        << " fallbackUsed=" << (record.fallbackUsed ? "Y" : "N")
        << " syntheticHoleUsed=" << (record.syntheticHoleUsed ? "Y" : "N")
        << " hiddenTipHypothesized=" << (record.hiddenTipHypothesized ? "Y" : "N")
        << " rhrFlipped=" << (record.rhrFlipped ? "Y" : "N") << "\n";
    log << "\n--- Predictor before frame ---\n";
    log << "hasPrev=" << (record.predictorBefore.hasPrev ? "Y" : "N")
        << " hasVelocity=" << (record.predictorBefore.hasVelocity ? "Y" : "N")
        << " refDistance=" << record.predictorBefore.refDistance << "\n";
    log << "lastHead=" << pointString(record.predictorBefore.lastHeadPos)
        << " lastTail=" << pointString(record.predictorBefore.lastTailPos)
        << " lastCenter=" << pointString(record.predictorBefore.lastCenterPos) << "\n";
    log << "velHead=" << pointString(record.predictorBefore.velHead)
        << " velTail=" << pointString(record.predictorBefore.velTail)
        << " velCenter=" << pointString(record.predictorBefore.velCenter) << "\n";
    log << "predictedHead=" << pointString(record.predictedHead)
        << " predictedTail=" << pointString(record.predictedTail)
        << " predictedCenter=" << pointString(record.predictedCenter) << "\n";
    log << "\n--- Baseline before frame ---\n";
    log << "curvatureSamples=" << record.baselineBefore.curvatureSamples
        << " meanAbsCurvature=" << record.baselineBefore.meanAbsCurvature
        << " curvatureStdDev=" << record.baselineBefore.curvatureStdDev() << "\n";
    log << "widthSamples=" << record.baselineBefore.widthSamples
        << " meanWidth=" << record.baselineBefore.meanWidth
        << " widthStdDev=" << record.baselineBefore.widthStdDev() << "\n";
    log << "lengthSamples=" << record.baselineBefore.lengthSamples
        << " meanBodyLength=" << record.baselineBefore.meanBodyLength
        << " bodyLengthStdDev=" << record.baselineBefore.bodyLengthStdDev() << "\n";
    log << "\n--- Centerline decisions ---\n";
    log << "refLength=" << record.refLength
        << " previousCrossSum=" << record.previousTurningAngle << "\n";
    log << "initialArcLength=" << record.initialArcLength
        << " finalArcLength=" << record.finalArcLength
        << " finalCrossSum=" << record.finalTurningAngle << "\n";
    log << "assignedHeadTipIdx=" << record.assignedHeadTipIdx
        << " assignedTailTipIdx=" << record.assignedTailTipIdx << "\n";
    if (record.hiddenTipHypothesized) {
        log << "hiddenTipTarget=" << pointString(record.hiddenTipTarget)
            << " hiddenTipFinal=" << pointString(record.hiddenTipFinal)
            << " hiddenToPred=" << pointDistance(record.hiddenTipFinal, record.hiddenTipTarget) << "\n";
        log << "hiddenTipMaskDiffArea=" << record.hiddenTipMaskDiffArea
            << " selected=" << record.hiddenTipMaskDiffSelectedArea << "\n";
    }
    if (record.d3RouteDebugAvailable) {
        log << "d3RouteStart=" << pointString(record.d3RouteStart)
            << " role=" << (record.d3RouteStartIsHead ? "head" : "tail")
            << " junction=" << pointString(record.d3RouteJunction)
            << " center=" << pointString(record.d3RouteCenter)
            << " end=" << pointString(record.d3RouteEnd)
            << " selectedCandidate=" << record.d3SelectedCandidate << "\n";
        log << "d3JunctionClusters=" << record.d3JunctionClusterCount
            << " selectedCluster=" << record.d3SelectedJunctionCluster
            << " fallback=" << (record.d3JunctionFallbackUsed ? "Y" : "N") << "\n";
        for (const QString& detail : record.d3JunctionDiagnostics) {
            log << "  " << detail << "\n";
        }
        log << "d3CandidatePaths=" << record.d3CandidatePaths.size() << "\n";
        for (int i = 0; i < static_cast<int>(record.d3CandidatePaths.size()); ++i) {
            const auto& path = record.d3CandidatePaths[i];
            log << "  path" << i << " points=" << path.size()
                << " length=" << polylineLength(path);
            if (!path.empty()) {
                log << " start=" << pointString(path.front())
                    << " end=" << pointString(path.back());
            }
            log << (i == record.d3SelectedCandidate ? " SELECTED" : "") << "\n";
        }
    }
    log << "tipCandidates=" << record.tipCandidates.size() << "\n";
    for (int i = 0; i < static_cast<int>(record.tipCandidates.size()); ++i) {
        const Tracking::TipCandidate& tip = record.tipCandidates[i];
        log << "  tip" << i << " point=" << pointString(tip.point)
            << " curvature=" << tip.curvature
            << " width=" << tip.width
            << " source=" << static_cast<int>(tip.source) << "\n";
    }
    log << "\n--- Bilateral cap-midpoint selection ---\n";
    log << "tipCapDebug entries=" << record.tipCapDebug.size() << "\n";
    for (int i = 0; i < static_cast<int>(record.tipCapDebug.size()); ++i) {
        const Tracking::TipCapDebug& cd = record.tipCapDebug[i];
        const QString& role = (i < static_cast<int>(record.tipCapRoles.size()))
                              ? record.tipCapRoles[i] : QString();
        log << "  tip" << i << " role=" << (role.isEmpty() ? QStringLiteral("?") : role) << "\n";
        if (!cd.valid) { log << "    (no bilateral debug captured)\n"; continue; }
        log << "    skelEndpoint=" << pointString(cd.skelEndpoint)
            << " outwardDir=(" << cd.outwardDir.x << "," << cd.outwardDir.y << ")"
            << " dtAtEp=" << cd.dtAtEp << "\n";
        log << "    searchWindow: maxForward=" << cd.maxForward
            << " maxSide=" << cd.maxSide
            << " leftCapPts=" << cd.leftCapPoints.size()
            << " rightCapPts=" << cd.rightCapPoints.size() << "\n";
        log << "    leftPeakFwd=" << cd.leftPeakFwd
            << " rightPeakFwd=" << cd.rightPeakFwd
            << " hasLeft=" << (cd.hasLeft ? "Y" : "N")
            << " hasRight=" << (cd.hasRight ? "Y" : "N")
            << " sanityPassed=" << (cd.sanityPassed ? "Y" : "N") << "\n";
        if (cd.hasBilateral) {
            log << "    leftApex=" << pointString(cd.leftApex)
                << " rightApex=" << pointString(cd.rightApex) << "\n";
            log << "    bilateralTip=" << pointString(cd.bilateralTip) << "\n";
        } else {
            log << "    bilateral: NOT computed\n";
        }
        log << "    snapPoint=" << pointString(cd.snapPoint);
        if (cd.hadPeak)
            log << " peakPoint=" << pointString(cd.peakOrSnapPoint);
        log << " hadPeak=" << (cd.hadPeak ? "Y" : "N") << "\n";
        if (cd.hasBilateral && validPoint(cd.snapPoint))
            log << "    snap→bilateral dist=" << pointDistance(cd.bilateralTip, cd.snapPoint) << "px\n";
        if (cd.hasBilateral && cd.hadPeak)
            log << "    peak→bilateral dist=" << pointDistance(cd.bilateralTip, cd.peakOrSnapPoint) << "px\n";
    }
    for (const QString& decision : record.decisions) {
        log << "decision: " << decision << "\n";
    }
    log << "\n--- Stored result comparison ---\n";
    log << "stored centerline points = " << blob.centerlinePoints.size() << "\n";
    log << "stored topologyState     = " << Tracking::topologyStateToString(blob.topologyState) << "\n";
    log << "stored head/tail tipIdx  = " << blob.assignedHeadTipIdx
        << " / " << blob.assignedTailTipIdx << "\n";
    if (blob.centerlinePoints.size() >= 2) {
        log << "stored head world = " << pointString(blob.centerlinePoints.front()) << "\n";
        log << "stored tail world = " << pointString(blob.centerlinePoints.back()) << "\n";
        std::vector<cv::Point2f> stored(blob.centerlinePoints.begin(), blob.centerlinePoints.end());
        log << "stored arcLength = " << polylineLength(stored) << "\n";
    }
    return true;
}

} // namespace Debug
