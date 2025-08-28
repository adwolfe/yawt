
#include "trackingcommon.h" // Lowercase include
#include <QtMath>       // For qSqrt, qPow
#include <QDebug>       // For qWarning/qDebug


namespace Tracking {

// Uses thresholded mat to find the nearest blob to a click and selects it.
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
            // touchesROIboundary is not relevant for findClickedBlob as it operates on the whole image or a pre-defined mask.
        }
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
        qWarning() << "findAllPlausibleBlobsInRoi: Invalid input image or ROI.";
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
    // Find contours within the sub-image (roiImage). Coordinates will be relative to roiImage.
    cv::findContours(roiImage.clone(), contoursInSubImage, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contourInSub : contoursInSubImage) {
        double area = cv::contourArea(contourInSub);
        
        // Calculate convex hull area (area without holes)
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

            // Offset contour points to be in full frame coordinates
            blob.contourPoints.reserve(contourInSub.size());
            for(const cv::Point& ptInSub : contourInSub) {
                blob.contourPoints.push_back(cv::Point(ptInSub.x + actualRoiCv.x, ptInSub.y + actualRoiCv.y));
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

            plausibleBlobs.append(blob);
        }
    }
    std::sort(plausibleBlobs.begin(), plausibleBlobs.end(), [](const DetectedBlob& a, const DetectedBlob& b) {
        return a.area > b.area; // For descending order; largest blob first
    });
    return plausibleBlobs;
}


} // namespace Tracking
