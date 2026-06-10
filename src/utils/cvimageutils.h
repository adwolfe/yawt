#ifndef CVIMAGEUTILS_H
#define CVIMAGEUTILS_H

#include <QImage>
#include <opencv2/core.hpp>

namespace CvImageUtils {

    /**
     * @brief Convert an OpenCV cv::Mat to a QImage, always returning a deep copy.
     *
     * Handles the common pixel formats encountered in YAWT:
     *   - CV_8UC3  : interpreted as BGR (OpenCV's default) and swapped to RGB.
     *   - CV_8UC1  : 8-bit grayscale.
     *   - 4-channel: BGRA, converted to BGR then RGB.
     *   - Float/other depths with 1 or 3 channels: scaled to 8-bit (x255).
     *
     * The result never aliases @p mat's pixel buffer — the QImage(uchar*, ...)
     * constructor only wraps memory, so the data is always copied before return.
     * This makes it safe to convert temporaries or about-to-be-evicted cache
     * entries. Returns a null QImage for empty or unsupported inputs.
     */
    QImage matToQImage(const cv::Mat& mat);

} // namespace CvImageUtils

#endif // CVIMAGEUTILS_H
