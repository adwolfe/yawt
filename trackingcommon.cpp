
#include "trackingcommon.h" // Lowercase include
#include <QtMath>       // For qSqrt, qPow


namespace TrackingHelper {

DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea,
                             double maxDistanceForSelection) {
    DetectedBlob result;
    result.isValid = false;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1) {
        // qDebug() << "findClickedBlob: Invalid input image (empty or not CV_8UC1)";
        return result;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        // qDebug() << "findClickedBlob: No contours found";
        return result;
    }

    cv::Point clickCvPoint(qRound(clickPointVideoCoords.x()), qRound(clickPointVideoCoords.y()));
    int bestContourIdx = -1;
    double minDistanceSq = std::numeric_limits<double>::max();

    // First, check for contours whose bounding box contains the click
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < minArea) {
            continue;
        }
        cv::Rect br = cv::boundingRect(contours[i]);
        if (br.contains(clickCvPoint)) {
            // This contour is a candidate if click is inside its bounding box
            // If multiple, we might prefer the smallest area, or just the first found
            // For simplicity, let's take the first one found that contains the click.
            // A more sophisticated approach might find the smallest such bounding box.
            bestContourIdx = static_cast<int>(i);
            break; // Found a contour containing the click
        }
    }

    // If no contour's bounding box contained the click, find the closest centroid
    if (bestContourIdx == -1) {
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < minArea) {
                continue;
            }
            cv::Moments mu = cv::moments(contours[i]);
            if (mu.m00 > 0) {
                cv::Point2f centroid(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
                double distSq = qPow(centroid.x - clickPointVideoCoords.x(), 2) +
                                qPow(centroid.y - clickPointVideoCoords.y(), 2);

                if (distSq < minDistanceSq && qSqrt(distSq) <= maxDistanceForSelection) {
                    minDistanceSq = distSq;
                    bestContourIdx = static_cast<int>(i);
                }
            }
        }
    }

    if (bestContourIdx != -1) {
        const auto& bestContour = contours[bestContourIdx];
        cv::Moments mu = cv::moments(bestContour);
        if (mu.m00 > 0) {
            result.centroid = QPointF(static_cast<double>(mu.m10 / mu.m00), static_cast<double>(mu.m01 / mu.m00));
            cv::Rect brCv = cv::boundingRect(bestContour);
            result.boundingBox = QRectF(brCv.x, brCv.y, brCv.width, brCv.height);
            result.area = cv::contourArea(bestContour);
            result.contourPoints = bestContour;
            result.isValid = true;
        }
    }
    return result;
}

}
