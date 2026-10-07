
#include "trackingcommon.h" // Lowercase include
#include "../core/centerlineprocessor.h"
#include <QtMath>       // For qSqrt, qPow
#include <QDebug>       // For qWarning/qDebug
#include <QJsonArray>
#include <QJsonObject>
#include "../utils/loggingcategories.h"
#include "../utils/jsongeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>


namespace Tracking {

namespace {

QJsonArray tipCandidatesToJson(const std::vector<TipCandidate>& tips)
{
    QJsonArray arr;
    for (const TipCandidate& tip : tips) {
        QJsonObject t;
        t["x"] = static_cast<double>(tip.point.x);
        t["y"] = static_cast<double>(tip.point.y);
        t["curvature"] = static_cast<double>(tip.curvature);
        t["width"] = static_cast<double>(tip.width);
        t["source"] = static_cast<int>(tip.source);
        arr.append(t);
    }
    return arr;
}

std::vector<TipCandidate> tipCandidatesFromJson(const QJsonArray& arr)
{
    std::vector<TipCandidate> tips;
    for (const QJsonValue& tv : arr) {
        if (!tv.isObject()) continue;
        const QJsonObject t = tv.toObject();
        TipCandidate tip;
        tip.point = cv::Point2f(static_cast<float>(t.value("x").toDouble()),
                                static_cast<float>(t.value("y").toDouble()));
        tip.curvature = static_cast<float>(t.value("curvature").toDouble());
        tip.width = static_cast<float>(t.value("width").toDouble());
        tip.source = static_cast<TipCandidate::Source>(t.value("source").toInt(0));
        tips.push_back(tip);
    }
    return tips;
}

} // namespace

QJsonObject blobGeometryToJson(const DetectedBlob& db)
{
    QJsonObject obj;
    obj["isValid"] = db.isValid;
    obj["area"] = db.area;
    obj["convexHullArea"] = db.convexHullArea;
    obj["touchesROIboundary"] = db.touchesSearchWindow;   // key kept for file compatibility

    obj["centroid"] = JsonGeometry::toJson(db.centroid);
    obj["boundingBox"] = JsonGeometry::toJson(db.boundingBox);
    obj["contourPoints"] = JsonGeometry::contourToJson(db.contourPoints);
    obj["holeContourPoints"] = JsonGeometry::holeContoursToJson(db.holeContourPoints);
    return obj;
}

void blobGeometryFromJson(const QJsonObject& obj, DetectedBlob& db)
{
    db.isValid = obj.value("isValid").toBool(false);
    db.area = obj.value("area").toDouble(0.0);
    db.convexHullArea = obj.value("convexHullArea").toDouble(0.0);
    db.touchesSearchWindow = obj.value("touchesROIboundary").toBool(false);

    if (obj.value("centroid").isObject()) {
        db.centroid = JsonGeometry::pointFromJson(obj["centroid"].toObject());
    }
    if (obj.value("boundingBox").isObject()) {
        db.boundingBox = JsonGeometry::rectFromJson(obj["boundingBox"].toObject());
    }
    db.contourPoints = JsonGeometry::contourFromJson(obj.value("contourPoints").toArray());
    db.holeContourPoints = JsonGeometry::holeContoursFromJson(obj.value("holeContourPoints").toArray());
}

QJsonObject blobCenterlineToJson(const BlobCenterline& cl)
{
    QJsonObject obj;
    obj["points"] = JsonGeometry::pointsToJson(cl.points);
    obj["hasCutPoint"] = cl.hasCutPoint;
    if (cl.hasCutPoint) {
        obj["cutPoint"] = JsonGeometry::toJson(cl.cutPoint);
    }
    obj["tipCandidates"] = tipCandidatesToJson(cl.tipCandidates);
    obj["headTipIdx"] = cl.headTipIdx;
    obj["tailTipIdx"] = cl.tailTipIdx;
    obj["topology"] = static_cast<int>(cl.topology);
    if (cl.needsReview) {
        obj["needsReview"] = true;
        obj["reviewReason"] = cl.reviewReason;
    }
    return obj;
}

BlobCenterline blobCenterlineFromJson(const QJsonObject& obj)
{
    BlobCenterline cl;
    cl.points = JsonGeometry::pointsFromJson(obj.value("points").toArray());
    cl.hasCutPoint = obj.value("hasCutPoint").toBool(false);
    if (cl.hasCutPoint && obj.value("cutPoint").isObject()) {
        cl.cutPoint = JsonGeometry::cvPointFromJson(obj["cutPoint"].toObject());
    }
    cl.tipCandidates = tipCandidatesFromJson(obj.value("tipCandidates").toArray());
    cl.headTipIdx = obj.value("headTipIdx").toInt(-1);
    cl.tailTipIdx = obj.value("tailTipIdx").toInt(-1);
    cl.topology = static_cast<TopologyState>(
        obj.value("topology").toInt(static_cast<int>(TopologyState::Unknown)));
    cl.needsReview = obj.value("needsReview").toBool(false);
    cl.reviewReason = obj.value("reviewReason").toString();
    return cl;
}

QJsonObject detectedBlobToJson(const DetectedBlob& db)
{
    // Combined layout: geometry keys plus the pre-split centerline key names.
    QJsonObject obj = blobGeometryToJson(db);
    const BlobCenterline& cl = db.centerline;
    obj["centerlinePoints"] = JsonGeometry::pointsToJson(cl.points);
    obj["hasCenterlineCutPoint"] = cl.hasCutPoint;
    if (cl.hasCutPoint) {
        obj["centerlineCutPoint"] = JsonGeometry::toJson(cl.cutPoint);
    }
    obj["tipCandidates"] = tipCandidatesToJson(cl.tipCandidates);
    obj["assignedHeadTipIdx"] = cl.headTipIdx;
    obj["assignedTailTipIdx"] = cl.tailTipIdx;
    obj["topologyState"] = static_cast<int>(cl.topology);
    return obj;
}

DetectedBlob detectedBlobFromJson(const QJsonObject& obj)
{
    DetectedBlob db;
    blobGeometryFromJson(obj, db);
    if (obj.value("centerline").isObject()) {
        db.centerline = blobCenterlineFromJson(obj["centerline"].toObject());
        return db;
    }
    BlobCenterline& cl = db.centerline;
    cl.points = JsonGeometry::pointsFromJson(obj.value("centerlinePoints").toArray());
    cl.hasCutPoint = obj.value("hasCenterlineCutPoint").toBool(false);
    if (cl.hasCutPoint && obj.value("centerlineCutPoint").isObject()) {
        cl.cutPoint = JsonGeometry::cvPointFromJson(obj["centerlineCutPoint"].toObject());
    }
    cl.tipCandidates = tipCandidatesFromJson(obj.value("tipCandidates").toArray());
    cl.headTipIdx = obj.value("assignedHeadTipIdx").toInt(-1);
    cl.tailTipIdx = obj.value("assignedTailTipIdx").toInt(-1);
    cl.topology = static_cast<TopologyState>(
        obj.value("topologyState").toInt(static_cast<int>(TopologyState::Unknown)));
    return db;
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
            // touchesSearchWindow is not relevant for findClickedBlob as it operates on the whole image or a pre-defined mask.
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
                                               const QRectF& roiToSearch, // This is in full-frame video coordinates
                                               double minArea,
                                               double maxArea,
                                               double minAspectRatio,
                                               double maxAspectRatio) {
    QList<DetectedBlob> plausibleBlobs;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1 || roiToSearch.isEmpty() || roiToSearch.width() <=0 || roiToSearch.height() <=0) {
        YAWT_WARN(lcDataCommon) << "findAllPlausibleBlobsInRoi: Invalid input image or ROI.";
        return plausibleBlobs;
    }

    // Define the OpenCV ROI from QRectF (roiToSearch is in full-frame video coordinates)
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

            // Convert centroid and bounding box to full-frame video coordinates
            // Centroid in sub-image: (mu.m10 / mu.m00), (mu.m01 / mu.m00)
            // Add actualRoiCv.x and actualRoiCv.y to convert to full-frame video coordinates
            blob.centroid = QPointF(actualRoiCv.x + (mu.m10 / mu.m00),
                                    actualRoiCv.y + (mu.m01 / mu.m00));

            // Bounding box in sub-image: brInSub
            // Add actualRoiCv.x and actualRoiCv.y to convert to full-frame video coordinates
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

            // --- Set touchesSearchWindow flag ---
            // Check if the bounding box of the contour (brInSub, which is relative to roiImage)
            // touches the edges of roiImage.
            // roiImage has dimensions actualRoiCv.width and actualRoiCv.height.
            // Note: actualRoiCv.width and actualRoiCv.height are the dimensions of roiImage.
            if (brInSub.x <= 0 ||
                brInSub.y <= 0 ||
                (brInSub.x + brInSub.width) >= actualRoiCv.width ||
                (brInSub.y + brInSub.height) >= actualRoiCv.height) {
                blob.touchesSearchWindow = true;
            } else {
                blob.touchesSearchWindow = false;
            }
            // A more precise check could iterate over contour points if needed, but bounding box is usually sufficient.
            // For example, if any point in contourInSub has x=0, y=0, x=actualRoiCv.width-1, or y=actualRoiCv.height-1.
            // However, the bounding box check is simpler and often what's implied.

            plausibleBlobs.append(blob);
        }
    }
    std::sort(plausibleBlobs.begin(), plausibleBlobs.end(), [](const DetectedBlob& a, const DetectedBlob& b) {
        return a.area > b.area; // For descending order; largest blob first
    });
    return plausibleBlobs;
}


} // namespace Tracking
