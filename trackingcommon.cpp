
#include "trackingcommon.h" // Lowercase include
#include <QtMath>       // For qSqrt, qPow


namespace TrackingHelper {

DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea,
                             double maxArea,
                             double maxDistanceForSelection) {
    DetectedBlob result;
    result.isValid = false;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1) {
        qWarning() << "findClickedBlob: Invalid input image (empty or not CV_8UC1).";
        return result;
    }

    std::vector<std::vector<cv::Point>> contours;
    // Use a copy of binaryImage for findContours if it modifies the input
    cv::findContours(binaryImage.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        // qDebug() << "findClickedBlob: No contours found in the image.";
        return result;
    }

    cv::Point clickCvPoint(qRound(clickPointVideoCoords.x()), qRound(clickPointVideoCoords.y()));
    int bestContourIdx = -1;
    double minDistanceSqToCentroid = std::numeric_limits<double>::max();
    bool clickInsideABlob = false;

    // Pass 1: Check for contours whose bounding box *contains* the click point.
    // Prioritize these. If multiple, could pick smallest area or closest centroid.
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < minArea || area > maxArea) { // Apply area filter
            continue;
        }
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
        }
    }

    return result;
}


QList<DetectedBlob> findAllPlausibleBlobsInRoi(const cv::Mat& binaryImage,
                                               const QRectF& roiToSearch,
                                               double minArea,
                                               double maxArea,
                                               double minAspectRatio,
                                               double maxAspectRatio) {
    QList<DetectedBlob> plausibleBlobs;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1 || roiToSearch.isEmpty() || roiToSearch.width() <=0 || roiToSearch.height() <=0) {
        qWarning() << "findAllPlausibleBlobsInRoi: Invalid input image or ROI.";
        return plausibleBlobs;
    }

    // Define the OpenCV ROI from QRectF
    cv::Rect roiCv(static_cast<int>(roiToSearch.x()),
                   static_cast<int>(roiToSearch.y()),
                   static_cast<int>(roiToSearch.width()),
                   static_cast<int>(roiToSearch.height()));

    // Ensure ROI is within the image boundaries
    roiCv &= cv::Rect(0, 0, binaryImage.cols, binaryImage.rows);

    if (roiCv.width <= 0 || roiCv.height <= 0) {
        // qDebug() << "findAllPlausibleBlobsInRoi: ROI after clamping is invalid.";
        return plausibleBlobs; // ROI is outside image or has no area
    }

    cv::Mat roiImage = binaryImage(roiCv); // Extract the ROI
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roiImage.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // Use clone as findContours can modify

    for (const auto& contourInRoi : contours) {
        double area = cv::contourArea(contourInRoi);
        if (area < minArea || area > maxArea) {
            continue; // Filter by area
        }

        cv::Rect brInRoi = cv::boundingRect(contourInRoi);
        if (brInRoi.width == 0 || brInRoi.height == 0) {
            continue; // Skip zero-dimension bounding boxes
        }

        double aspectRatio = static_cast<double>(brInRoi.width) / static_cast<double>(brInRoi.height);
        if (aspectRatio < 1.0) {
            aspectRatio = 1.0 / aspectRatio; // Ensure aspect ratio is >= 1 for easier comparison
        }

        // For now, I don't care what the aspect ratio is of these worms.
        //if (aspectRatio < minAspectRatio || aspectRatio > maxAspectRatio) {
        //    continue; // Filter by aspect ratio
        //}

        cv::Moments mu = cv::moments(contourInRoi);
        if (mu.m00 > 0) { // Check for valid moments (non-zero area)
            DetectedBlob blob;
            // Centroid and bounding box need to be offset by the ROI's top-left corner
            // to be in the coordinate system of the original binaryImage.
            blob.centroid = QPointF(roiCv.x + (mu.m10 / mu.m00),
                                    roiCv.y + (mu.m01 / mu.m00));
            blob.boundingBox = QRectF(roiCv.x + brInRoi.x,
                                      roiCv.y + brInRoi.y,
                                      brInRoi.width,
                                      brInRoi.height);
            blob.area = area;

            // Offset contour points to be in full frame coordinates
            blob.contourPoints.reserve(contourInRoi.size());
            for(const cv::Point& ptInRoi : contourInRoi) {
                blob.contourPoints.push_back(cv::Point(ptInRoi.x + roiCv.x, ptInRoi.y + roiCv.y));
            }

            blob.isValid = true;
            plausibleBlobs.append(blob);
        }
    }
    std::sort(plausibleBlobs.begin(), plausibleBlobs.end(), [](const DetectedBlob& a, const DetectedBlob& b) {
        return a.area > b.area; // For descending order; largest blob first
    });
    return plausibleBlobs;
}


} // namespace TrackingHelper
