#ifndef THRESHOLDINGUTILS_H
#define THRESHOLDINGUTILS_H

#include <opencv2/opencv.hpp>
#include "trackingcommon.h" // Assuming this contains ThresholdSettings struct

namespace ThresholdingUtils {

    // Core thresholding function - used by both VideoLoader and VideoProcessor
    void applyThresholding(const cv::Mat& inputFrame, cv::Mat& outputFrame,
                          const Thresholding::ThresholdSettings& settings);

    // Median background computation
    cv::Mat computeMedianBackground(const std::vector<cv::Mat>& sampleFrames);

    // Median background subtraction
    void subtractMedianBackground(const cv::Mat& inputFrame, cv::Mat& outputFrame,
                                 const cv::Mat& medianBackground,
                                 bool clipToZero = true);

    // Combined function to perform both operations
    void thresholdWithBackgroundSubtraction(
        const cv::Mat& inputFrame,
        cv::Mat& outputFrame,
        const Thresholding::ThresholdSettings& settings,
        const cv::Mat& medianBackground,
        bool performSubtraction = true);

} // namespace ThresholdingUtils

#endif // THRESHOLDINGUTILS_H
