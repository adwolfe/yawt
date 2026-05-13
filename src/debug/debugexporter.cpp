#include "debugexporter.h"

#include "debugdatastore.h"
#include "../data/trackingdatastorage.h"

#include <QDir>
#include <QFile>
#include <QTextStream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

namespace {

constexpr int kExportScale = 4;
constexpr int kExportPad = 12;

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
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

static void drawText(cv::Mat& canvas, const QString& text, cv::Point at,
                     cv::Scalar color = cv::Scalar(255, 255, 255))
{
    cv::putText(canvas, text.toStdString(), at, cv::FONT_HERSHEY_SIMPLEX,
                0.45, color, 1, cv::LINE_AA);
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
    writeStageImage(canvas, outputDir, fileName);
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
        writeStageImage(canvas, outputDir, QStringLiteral("00_mask.png"));
    }

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
                         QStringLiteral("09 final  arcLen=%1 px  turning=%2 rad")
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
        << " previousTurningAngle=" << record.previousTurningAngle << "\n";
    log << "initialArcLength=" << record.initialArcLength
        << " finalArcLength=" << record.finalArcLength
        << " finalTurningAngle=" << record.finalTurningAngle << "\n";
    log << "assignedHeadTipIdx=" << record.assignedHeadTipIdx
        << " assignedTailTipIdx=" << record.assignedTailTipIdx << "\n";
    if (record.hiddenTipHypothesized) {
        log << "hiddenTipTarget=" << pointString(record.hiddenTipTarget)
            << " hiddenTipFinal=" << pointString(record.hiddenTipFinal)
            << " hiddenToPred=" << pointDistance(record.hiddenTipFinal, record.hiddenTipTarget) << "\n";
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
