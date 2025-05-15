#ifndef TRACKINGCOMMON_H
#define TRACKINGCOMMON_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> // For cv::moments, cv::findContours, etc.
#include <QPointF>
#include <QRectF>
#include <QMetaEnum>
#include <QList>    // Changed from std::vector for DetectedBlob list in WormTracker signal
#include <vector>   // For std::vector<cv::Point>
#include <limits>   // For std::numeric_limits
#include <QtMath>   // For qSqrt, qPow

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

/**
     * @brief Structure to hold information about a detected blob.
     */
struct DetectedBlob {
    QPointF centroid;                     // Centroid of the blob in video coordinates
    QRectF boundingBox;                   // Bounding box of the blob in video coordinates
    double area = 0.0;                    // Area of the blob
    std::vector<cv::Point> contourPoints; // Raw contour points (in video coordinates)
    bool isValid = false;                 // Flag indicating if this blob data is valid

    // Default constructor
    DetectedBlob() : area(0.0), isValid(false) {}
};

/**
     * @brief Finds the blob in a binary image closest to a click point, or the one containing the click.
     * This function first looks for blobs whose bounding box contains the click.
     * If none are found, it then looks for the blob whose centroid is closest to the click,
     * within a specified maximum distance.
     * @param binaryImage The input 8-bit single-channel binary image (CV_8UC1).
     * Non-zero pixels are considered foreground.
     * @param clickPointVideoCoords The click coordinates in the same coordinate system as the binaryImage.
     * @param minArea Minimum contour area to be considered a valid blob.
     * @param maxArea Maximum contour area to be considered a valid blob.
     * @param maxDistanceForSelection Max distance (in pixels) from click to a blob's centroid
     * if the click is not inside any blob's bounding box.
     * @return DetectedBlob structure. Check DetectedBlob::isValid to see if a suitable blob was found.
     */
DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea = 5.0,
                             double maxArea = 10000.0, // Added maxArea
                             double maxDistanceForSelection = 30.0);

/**
     * @brief Finds all plausible blobs within a given ROI of a binary image.
     * @param binaryImage The input 8-bit single-channel binary image (CV_8UC1).
     * @param roiToSearch The QRectF defining the region of interest in video coordinates.
     * @param minArea Minimum area for a blob to be considered.
     * @param maxArea Maximum area for a blob to be considered.
     * @param minAspectRatio Minimum aspect ratio (width/height or height/width, always >= 1).
     * @param maxAspectRatio Maximum aspect ratio.
     * @return QList of DetectedBlob structs for all plausible blobs found.
     */
QList<DetectedBlob> findAllPlausibleBlobsInRoi(const cv::Mat& binaryImage,
                                               const QRectF& roiToSearch,
                                               double minArea,
                                               double maxArea,
                                               double minAspectRatio,
                                               double maxAspectRatio);

} // namespace TrackingHelper


#endif // TRACKINGCOMMON_H
