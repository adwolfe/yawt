#ifndef THRESHOLDINGUTILS_H
#define THRESHOLDINGUTILS_H

#include <opencv2/opencv.hpp>
#include "trackingcommon.h" // Assuming this contains ThresholdSettings struct

namespace ThresholdingUtils {

    // Core thresholding function - used by both VideoLoader and VideoProcessor
    void applyThresholding(const cv::Mat& inputFrame, cv::Mat& outputFrame,
                          const Thresholding::ThresholdSettings& settings);

    // Sample up to 31 evenly spaced frames without moving the playback decoder.
    cv::Mat computeVideoBackground(const QString& videoPath);

    // Median background computation
    cv::Mat computeMedianBackground(const std::vector<cv::Mat>& sampleFrames);

} // namespace ThresholdingUtils

#endif // THRESHOLDINGUTILS_H
