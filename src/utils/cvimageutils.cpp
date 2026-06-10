#include "cvimageutils.h"

#include <QtGlobal>
#include <opencv2/imgproc.hpp>

namespace CvImageUtils {

QImage matToQImage(const cv::Mat& mat) {
    // Always return a deep copy: the QImage(uchar* data, ...) constructor only
    // wraps the source buffer, so the result would dangle once the backing
    // cv::Mat (often a local or about-to-be-evicted cache entry) is released.
    if (mat.empty()) {
        return QImage();
    }

    if (mat.type() == CV_8UC3) {
        // OpenCV stores colour as BGR; rgbSwapped() both reorders and deep-copies.
        return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step),
                      QImage::Format_RGB888).rgbSwapped();
    }
    if (mat.type() == CV_8UC1) {
        return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step),
                      QImage::Format_Grayscale8).copy();
    }

    cv::Mat temp;
    try {
        if (mat.channels() == 4) {
            cv::cvtColor(mat, temp, cv::COLOR_BGRA2BGR);
            return QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step),
                          QImage::Format_RGB888).rgbSwapped();
        }
        if (mat.channels() == 3) { // non-8-bit 3-channel
            mat.convertTo(temp, CV_8UC3, 255.0);
            return QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step),
                          QImage::Format_RGB888).rgbSwapped();
        }
        if (mat.channels() == 1) { // non-8-bit single channel
            mat.convertTo(temp, CV_8UC1, 255.0);
            return QImage(temp.data, temp.cols, temp.rows, static_cast<int>(temp.step),
                          QImage::Format_Grayscale8).copy();
        }
    } catch (const cv::Exception& ex) {
        qWarning("CvImageUtils::matToQImage: OpenCV conversion exception: %s", ex.what());
        return QImage();
    }

    return QImage();
}

} // namespace CvImageUtils
