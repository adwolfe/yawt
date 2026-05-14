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

    drawTitle(canvas, QStringLiteral("04 head/tail final labels  topo=%1  branch=%2")
              .arg(Tracking::topologyStateToString(record.topology))
              .arg(Debug::centerlineBranchToString(record.branch)));
    drawCoordinateEdges(canvas, bounds);
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
    writeTipStage(blob, record, bounds, outputDir);
    writeHeadTailStage(blob, record, bounds, outputDir);
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
