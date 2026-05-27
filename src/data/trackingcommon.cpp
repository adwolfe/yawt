
#include "trackingcommon.h" // Lowercase include
#include "../core/centerlineprocessor.h"
#include <QtMath>       // For qSqrt, qPow
#include <QDebug>       // For qWarning/qDebug
#include "../utils/loggingcategories.h"

#include <algorithm>
#include <cmath>
#include <limits>


namespace Tracking {

namespace {

constexpr int kCenterlinePaddingPixels = 2;



} // namespace

// ── Single-pass endpoint detector ───────────────────────────────────────────
//
// Used by the centerline pipeline (centerlineworker doWork). Builds the
// skeleton once, then derives every downstream quantity from it.
//
// Design notes:
//
// * Curvature math + width probe are kept stable so baseline samples stay
//   commensurable across frames.
//
// * Mask construction reuses buildCenterlineMask, including the MORPH_CLOSE
//   applied for ring blobs.
//
// * Endpoint pruning preserves the longest-path pair when more than two
//   degree-1 nodes are present (skeleton noise spurs). Never drops below two
//   if two existed; never drops the only one if only one exists.
//
// * Each surviving skeleton endpoint is "extended" to the strongest curvature
//   peak within DT(endpoint) * 1.5 pixels (search radius scales with local
//   body thickness). Strongest, not nearest — picking the nearest peak snaps
//   onto body kinks for tightly-coiled worms.

bool populateCenterlineFromContour(DetectedBlob& blob)
{
    blob.centerlinePoints.clear();

    if (!blob.isValid || blob.contourPoints.size() < 3) {
        return false;
    }

    cv::Mat mask;
    cv::Rect localBounds = Centerline::buildCenterlineMask(blob, mask);
    if (mask.empty()) return false;

    blob.centerlinePoints = Centerline::extractCenterlineFromMask(
        mask,
        cv::Point2f(static_cast<float>(localBounds.x), static_cast<float>(localBounds.y)));

    return blob.centerlinePoints.size() >= 2;
}

bool populateCenterlineFromContourWithCut(DetectedBlob& blob,
                                          const cv::Point2f& cutStart,
                                          const cv::Point2f& cutEnd,
                                          int cutThickness)
{
    blob.centerlinePoints.clear();

    if (!blob.isValid || blob.contourPoints.size() < 3 || blob.holeContourPoints.empty()) {
        return false;
    }

    cv::Mat mask;
    cv::Rect localBounds = Centerline::buildCenterlineMask(blob, mask);
    if (mask.empty()) return false;

    const cv::Point localStart(qRound(cutStart.x - localBounds.x),
                               qRound(cutStart.y - localBounds.y));
    const cv::Point localEnd(qRound(cutEnd.x - localBounds.x),
                             qRound(cutEnd.y - localBounds.y));
    const int thickness = std::max(1, cutThickness);
    cv::line(mask, localStart, localEnd, cv::Scalar(0), thickness, cv::LINE_8);

    blob.centerlinePoints = Centerline::extractCenterlineFromMask(
        mask,
        cv::Point2f(static_cast<float>(localBounds.x), static_cast<float>(localBounds.y)));

    return blob.centerlinePoints.size() >= 2;
}

QList<QPointF> extractOrderedCenterlinePoints(const DetectedBlob& blob)
{
    QList<QPointF> centerlinePoints;
    if (!blob.isValid) {
        return centerlinePoints;
    }

    DetectedBlob centerlineBlob = blob;
    if (centerlineBlob.centerlinePoints.empty() && !centerlineBlob.contourPoints.empty()) {
        populateCenterlineFromContour(centerlineBlob);
    }

    if (centerlineBlob.centerlinePoints.empty()) {
        return centerlinePoints;
    }

    centerlinePoints.reserve(static_cast<qsizetype>(centerlineBlob.centerlinePoints.size()));
    for (const cv::Point2f& point : centerlineBlob.centerlinePoints) {
        centerlinePoints.append(QPointF(point.x, point.y));
    }

    return centerlinePoints;
}

QList<QPointF> resampleCenterlinePoints(const QList<QPointF>& points, int pointCount)
{
    QList<QPointF> sampledPoints;
    if (points.isEmpty() || pointCount <= 0) {
        return sampledPoints;
    }

    sampledPoints.reserve(pointCount);
    if (points.size() == 1 || pointCount == 1) {
        for (int i = 0; i < pointCount; ++i) {
            sampledPoints.append(points.first());
        }
        return sampledPoints;
    }

    std::vector<double> cumulativeDistance(static_cast<size_t>(points.size()), 0.0);
    for (int i = 1; i < points.size(); ++i) {
        const QPointF delta = points.at(i) - points.at(i - 1);
        cumulativeDistance[static_cast<size_t>(i)] =
            cumulativeDistance[static_cast<size_t>(i - 1)] + std::hypot(delta.x(), delta.y());
    }

    const double totalLength = cumulativeDistance.back();
    if (qFuzzyIsNull(totalLength)) {
        for (int i = 0; i < pointCount; ++i) {
            sampledPoints.append(points.first());
        }
        return sampledPoints;
    }

    int segmentIndex = 1;
    for (int sampleIndex = 0; sampleIndex < pointCount; ++sampleIndex) {
        const double targetDistance =
            totalLength * static_cast<double>(sampleIndex) / static_cast<double>(pointCount - 1);

        while (segmentIndex < points.size() - 1 &&
               cumulativeDistance[static_cast<size_t>(segmentIndex)] < targetDistance) {
            ++segmentIndex;
        }

        const double previousDistance = cumulativeDistance[static_cast<size_t>(segmentIndex - 1)];
        const double nextDistance = cumulativeDistance[static_cast<size_t>(segmentIndex)];
        const double segmentLength = nextDistance - previousDistance;
        const double t = qFuzzyIsNull(segmentLength)
                             ? 0.0
                             : (targetDistance - previousDistance) / segmentLength;

        const QPointF a = points.at(segmentIndex - 1);
        const QPointF b = points.at(segmentIndex);
        sampledPoints.append(a + (b - a) * t);
    }

    return sampledPoints;
}

QList<QPointF> resampleCenterlinePoints(const std::vector<cv::Point2f>& points, int pointCount)
{
    QList<QPointF> convertedPoints;
    convertedPoints.reserve(static_cast<qsizetype>(points.size()));
    for (const cv::Point2f& point : points) {
        convertedPoints.append(QPointF(point.x, point.y));
    }
    return resampleCenterlinePoints(convertedPoints, pointCount);
}

QList<QPointF> extractResampledCenterlinePoints(const DetectedBlob& blob, int pointCount)
{
    return resampleCenterlinePoints(extractOrderedCenterlinePoints(blob), pointCount);
}

// Uses thresholded mat to find the nearest blob to a click and selects it.
DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea,
                             double maxArea,
                             double maxDistanceForSelection) {
    DetectedBlob result;
    result.isValid = false;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1) {
        YAWT_WARN(lcDataCommon) << "findClickedBlob: Invalid input image (empty or not CV_8UC1).";
        return result;
    }

    cv::Point clickCvPoint(qRound(clickPointVideoCoords.x()), qRound(clickPointVideoCoords.y()));
    if (clickCvPoint.x < 0 || clickCvPoint.y < 0 ||
        clickCvPoint.x >= binaryImage.cols || clickCvPoint.y >= binaryImage.rows) {
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Click out of bounds. Click:"
                                 << clickPointVideoCoords << "Image size:"
                                 << QSize(binaryImage.cols, binaryImage.rows);
        return result;
    }

    const int clickPixel = static_cast<int>(binaryImage.at<uchar>(clickCvPoint));
    YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Click:"
                             << clickPointVideoCoords
                             << "Pixel:" << clickPixel
                             << "Image size:" << QSize(binaryImage.cols, binaryImage.rows)
                             << "Min/Max area:" << minArea << "/" << maxArea
                             << "Max dist:" << maxDistanceForSelection;

    std::vector<std::vector<cv::Point>> contours;
    // Use a copy of binaryImage for findContours if it modifies the input
    cv::findContours(binaryImage.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: No contours found.";
        return result;
    }

    int bestContourIdx = -1;
    double minDistanceSqToCentroid = std::numeric_limits<double>::max();
    bool clickInsideABlob = false;
    int areaPassedCount = 0;
    double minContourArea = std::numeric_limits<double>::max();
    double maxContourArea = 0.0;

    // Pass 1: Check for contours whose bounding box *contains* the click point.
    // Prioritize these. If multiple, could pick smallest area or closest centroid.
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < minContourArea) minContourArea = area;
        if (area > maxContourArea) maxContourArea = area;
        if (area < minArea || area > maxArea) { // Apply area filter
            continue;
        }
        areaPassedCount++;
        cv::Rect br = cv::boundingRect(contours[i]);
        if (br.contains(clickCvPoint)) {
            // This contour is a strong candidate.
            // If we find one, we can potentially stop and use this one.
            // For now, let's take the first valid one we find that contains the click.
            // A more refined approach might be to find the one with the smallest area that contains the click.
            cv::Moments mu = cv::moments(contours[i]);
            if (mu.m00 > 0) { // Check for valid moments
                double distSq = qPow( (mu.m10 / mu.m00) - clickPointVideoCoords.x(), 2) +
                                qPow( (mu.m01 / mu.m00) - clickPointVideoCoords.y(), 2);
                if (distSq < minDistanceSqToCentroid) { // Prefer the one whose centroid is closer if multiple contain click
                    minDistanceSqToCentroid = distSq;
                    bestContourIdx = static_cast<int>(i);
                    clickInsideABlob = true;
                }
            }
        }
    }

    // Pass 2: If click was not inside any blob's bounding box, find the blob with the closest centroid.
    if (!clickInsideABlob) {
        minDistanceSqToCentroid = std::numeric_limits<double>::max(); // Reset for this pass
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < minArea || area > maxArea) { // Apply area filter
                continue;
            }
            cv::Moments mu = cv::moments(contours[i]);
            if (mu.m00 > 0) { // Check for valid moments (non-zero area)
                cv::Point2f centroid(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
                double distSq = qPow(centroid.x - clickPointVideoCoords.x(), 2) +
                                qPow(centroid.y - clickPointVideoCoords.y(), 2);

                if (distSq < minDistanceSqToCentroid) {
                    minDistanceSqToCentroid = distSq;
                    bestContourIdx = static_cast<int>(i);
                }
            }
        }
        // Check if the closest one found is within the maxDistanceForSelection
        if (qSqrt(minDistanceSqToCentroid) > maxDistanceForSelection) {
            bestContourIdx = -1; // Too far, invalidate selection
        }
    }


    // If a suitable contour was found by either method
    if (bestContourIdx != -1) {
        const auto& bestContour = contours[bestContourIdx];
        cv::Moments mu = cv::moments(bestContour);
        // Double check mu.m00 > 0, though area filter should imply this
        if (mu.m00 > 0) {
            result.centroid = QPointF(static_cast<double>(mu.m10 / mu.m00), static_cast<double>(mu.m01 / mu.m00));
            cv::Rect brCv = cv::boundingRect(bestContour);
            result.boundingBox = QRectF(brCv.x, brCv.y, brCv.width, brCv.height);
            result.area = cv::contourArea(bestContour); // Already calculated, but store it
            result.contourPoints = bestContour; // These points are relative to binaryImage origin
            result.isValid = true;
            populateCenterlineFromContour(result);
            // touchesROIboundary is not relevant for findClickedBlob as it operates on the whole image or a pre-defined mask.
        }
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Selected contour idx:"
                                 << bestContourIdx
                                 << "Centroid:" << result.centroid
                                 << "Area:" << result.area
                                 << "BBox:" << result.boundingBox;
    } else {
        const double minAreaForLog = (minContourArea == std::numeric_limits<double>::max()) ? 0.0 : minContourArea;
        const double dist = (minDistanceSqToCentroid == std::numeric_limits<double>::max())
                                ? -1.0
                                : qSqrt(minDistanceSqToCentroid);
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: No valid blob."
                                 << "Contours:" << contours.size()
                                 << "Area-passing:" << areaPassedCount
                                 << "Area min/max:" << minAreaForLog << "/" << maxContourArea
                                 << "Click inside bbox:" << clickInsideABlob
                                 << "Nearest centroid dist:" << dist;
    }

    return result;
}


QList<DetectedBlob> findAllPlausibleBlobsInRoi(const cv::Mat& binaryImage,
                                               const QRectF& roiToSearch, // This is in full image coordinates
                                               double minArea,
                                               double maxArea,
                                               double minAspectRatio,
                                               double maxAspectRatio) {
    QList<DetectedBlob> plausibleBlobs;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1 || roiToSearch.isEmpty() || roiToSearch.width() <=0 || roiToSearch.height() <=0) {
        YAWT_WARN(lcDataCommon) << "findAllPlausibleBlobsInRoi: Invalid input image or ROI.";
        return plausibleBlobs;
    }

    // Define the OpenCV ROI from QRectF (roiToSearch is in full image coordinates)
    cv::Rect roiCv(static_cast<int>(qRound(roiToSearch.x())),
                   static_cast<int>(qRound(roiToSearch.y())),
                   static_cast<int>(qRound(roiToSearch.width())),
                   static_cast<int>(qRound(roiToSearch.height())));

    // Ensure ROI is within the image boundaries
    // This creates the actual ROI that will be used on binaryImage
    cv::Rect actualRoiCv = roiCv & cv::Rect(0, 0, binaryImage.cols, binaryImage.rows);

    if (actualRoiCv.width <= 0 || actualRoiCv.height <= 0) {
        // qDebug() << "findAllPlausibleBlobsInRoi: ROI after clamping is invalid or outside image.";
        return plausibleBlobs; // ROI is outside image or has no area
    }

    cv::Mat roiImage = binaryImage(actualRoiCv); // Extract the sub-image for contour finding
    std::vector<std::vector<cv::Point>> contoursInSubImage;
    std::vector<cv::Vec4i> hierarchy;
    // RETR_CCOMP gives a 2-level hierarchy (outer contours + their holes).
    // This lets us subtract hole areas from outer contour areas, so a coiled worm
    // whose thresholded shape is a ring is measured by actual pixel area rather than
    // the much-larger disk area that RETR_EXTERNAL + contourArea would produce.
    cv::findContours(roiImage.clone(), contoursInSubImage, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contoursInSubImage.size(); ++i) {
        // Only process outer contours (parent index == -1 in RETR_CCOMP)
        if (hierarchy[i][3] != -1) continue;

        const auto& contourInSub = contoursInSubImage[i];
        double outerArea = cv::contourArea(contourInSub);

        // Subtract areas of direct-child hole contours to get true foreground pixel area.
        // This handles the ring topology produced by a self-touching coiled worm.
        double holeArea = 0.0;
        for (int childIdx = hierarchy[i][2]; childIdx != -1; childIdx = hierarchy[childIdx][0]) {
            holeArea += cv::contourArea(contoursInSubImage[childIdx]);
        }
        double area = outerArea - holeArea;

        // Calculate convex hull area using the outer boundary
        std::vector<cv::Point> hull;
        cv::convexHull(contourInSub, hull);
        double hullArea = cv::contourArea(hull);

        if (area < minArea || area > maxArea) {
            continue; // Filter by area
        }

        // Bounding box of the contour, relative to roiImage (the sub-image)
        cv::Rect brInSub = cv::boundingRect(contourInSub);
        if (brInSub.width == 0 || brInSub.height == 0) {
            continue; // Skip zero-dimension bounding boxes
        }

        // Aspect ratio (using dimensions from brInSub)
        double currentAspectRatio = static_cast<double>(brInSub.width) / static_cast<double>(brInSub.height);
        if (currentAspectRatio < 1.0) {
            currentAspectRatio = 1.0 / currentAspectRatio; // Ensure aspect ratio is >= 1
        }

        // Aspect ratio filter (currently commented out in your provided code)
        // if (currentAspectRatio < minAspectRatio || currentAspectRatio > maxAspectRatio) {
        //     continue;
        // }

        cv::Moments mu = cv::moments(contourInSub);
        if (mu.m00 > 0) { // Check for valid moments (non-zero area)
            DetectedBlob blob;
            blob.isValid = true;
            blob.area = area;
            blob.convexHullArea = hullArea;

            // Convert centroid and bounding box to full image coordinates
            // Centroid in sub-image: (mu.m10 / mu.m00), (mu.m01 / mu.m00)
            // Add actualRoiCv.x and actualRoiCv.y to convert to full image coordinates
            blob.centroid = QPointF(actualRoiCv.x + (mu.m10 / mu.m00),
                                    actualRoiCv.y + (mu.m01 / mu.m00));

            // Bounding box in sub-image: brInSub
            // Add actualRoiCv.x and actualRoiCv.y to convert to full image coordinates
            blob.boundingBox = QRectF(actualRoiCv.x + brInSub.x,
                                      actualRoiCv.y + brInSub.y,
                                      brInSub.width,
                                      brInSub.height);

            // Offset outer contour points to full frame coordinates
            blob.contourPoints.reserve(contourInSub.size());
            for(const cv::Point& ptInSub : contourInSub) {
                blob.contourPoints.push_back(cv::Point(ptInSub.x + actualRoiCv.x, ptInSub.y + actualRoiCv.y));
            }

            // Offset hole contour points to full frame coordinates
            for (int childIdx = hierarchy[i][2]; childIdx != -1; childIdx = hierarchy[childIdx][0]) {
                std::vector<cv::Point> holeInFullFrame;
                holeInFullFrame.reserve(contoursInSubImage[childIdx].size());
                for (const cv::Point& ptInSub : contoursInSubImage[childIdx]) {
                    holeInFullFrame.push_back(cv::Point(ptInSub.x + actualRoiCv.x, ptInSub.y + actualRoiCv.y));
                }
                blob.holeContourPoints.push_back(std::move(holeInFullFrame));
            }

            // --- Set touchesROIboundary flag ---
            // Check if the bounding box of the contour (brInSub, which is relative to roiImage)
            // touches the edges of roiImage.
            // roiImage has dimensions actualRoiCv.width and actualRoiCv.height.
            // Note: actualRoiCv.width and actualRoiCv.height are the dimensions of roiImage.
            if (brInSub.x <= 0 ||
                brInSub.y <= 0 ||
                (brInSub.x + brInSub.width) >= actualRoiCv.width ||
                (brInSub.y + brInSub.height) >= actualRoiCv.height) {
                blob.touchesROIboundary = true;
            } else {
                blob.touchesROIboundary = false;
            }
            // A more precise check could iterate over contour points if needed, but bounding box is usually sufficient.
            // For example, if any point in contourInSub has x=0, y=0, x=actualRoiCv.width-1, or y=actualRoiCv.height-1.
            // However, the bounding box check is simpler and often what's implied.

            populateCenterlineFromContour(blob);
            plausibleBlobs.append(blob);
        }
    }
    std::sort(plausibleBlobs.begin(), plausibleBlobs.end(), [](const DetectedBlob& a, const DetectedBlob& b) {
        return a.area > b.area; // For descending order; largest blob first
    });
    return plausibleBlobs;
}


} // namespace Tracking
