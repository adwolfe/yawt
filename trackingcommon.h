#ifndef TRACKINGCOMMON_H
#define TRACKINGCOMMON_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <QPointF>
#include <QRectF>
#include <QMetaEnum>
#include <vector> // For std::vector
#include <limits> // For std::numeric_limits

// This header defines types and structures common to video loading,
// processing, and tracking to avoid circular dependencies.

/**
 * @brief Defines available thresholding algorithms.
 * (Moved from videoloader.h)
 */

enum class ThresholdAlgorithm {
    Global,         // Simple global threshold
    Otsu,           // Otsu's binarization (auto global threshold)
    AdaptiveMean,   // Adaptive threshold using mean of neighborhood
    AdaptiveGaussian // Adaptive threshold using Gaussian weighted sum of neighborhood
};


/**
 * @brief Structure to hold parameters for thresholding and pre-processing.
 * (Moved from TrackData.h, uses ThresholdAlgorithm defined above)
 */
struct ThresholdSettings {
    // General setting for interpreting pixel values (background vs. foreground)
    bool assumeLightBackground = true;

    // Main algorithm choice for thresholding.
    ThresholdAlgorithm algorithm = ThresholdAlgorithm::Global;

    // --- Parameters for Global Thresholding ---
    int globalThresholdValue = 127;

    // --- Parameters for Adaptive Thresholding ---
    int adaptiveBlockSize = 3;    // Must be odd, >=3.
    double adaptiveCValue = 0.0;   // Constant subtracted from the mean/weighted mean.

    // --- Pre-processing: Gaussian Blur ---
    bool enableBlur = false;        // Whether to apply Gaussian blur before thresholding.
    int blurKernelSize = 3;        // Must be odd, >=3.
    double blurSigmaX = 0.0;       // 0 for auto calculation from kernel size.
};



namespace TrackingHelper {

struct DetectedBlob {
    QPointF centroid;
    QRectF boundingBox;
    double area = 0.0;
    std::vector<cv::Point> contourPoints; // Store the actual contour points
    bool isValid = false;
};

/**
     * @brief Finds the blob in a binary image closest to a click point.
     * @param binaryImage The input 8-bit single-channel binary image (CV_8UC1). Non-zero pixels are foreground.
     * @param clickPointVideoCoords The click coordinates in the same space as the binaryImage.
     * @param minArea Minimum contour area to be considered a valid blob.
     * @param maxDistanceForSelection Max distance from click to a blob's centroid if click is not inside any bounding box.
     * @return DetectedBlob structure. Check DetectedBlob::isValid.
     */
DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea = 5.0,
                             double maxDistanceForSelection = 30.0); // Pixels

} // namespace TrackingHelper


#endif // TRACKINGCOMMON_H
