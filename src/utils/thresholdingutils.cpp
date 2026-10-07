#include "thresholdingutils.h"
#include <QDebug>
#include <algorithm>
#include "loggingcategories.h"

namespace ThresholdingUtils {

void applyThresholding(const cv::Mat& inputFrame, cv::Mat& outputFrame, 
                       const Thresholding::ThresholdSettings& settings) {
    if (inputFrame.empty()) {
        outputFrame = cv::Mat();
        return;
    }

    cv::Mat grayFrame;
    if (inputFrame.channels() == 3 || inputFrame.channels() == 4) {
        cv::cvtColor(inputFrame, grayFrame, inputFrame.channels() == 4
                     ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = inputFrame.clone();
    }
    if (grayFrame.type() != CV_8UC1) {
        double low, high;
        cv::minMaxLoc(grayFrame, &low, &high);
        if (high > low)
            grayFrame.convertTo(grayFrame, CV_8U, 255.0 / (high - low),
                                -low * 255.0 / (high - low));
        else
            grayFrame = cv::Mat::zeros(grayFrame.size(), CV_8U);
    }

    cv::Mat foregroundMask;
    if (settings.enableBackgroundSubtraction) {
        if (settings.medianBackground.empty() ||
            settings.medianBackground.size() != grayFrame.size() ||
            settings.medianBackground.type() != CV_8UC1) {
            YAWT_WARN(lcUtilsThresholding) << "Missing or incompatible background model";
            outputFrame.release();
            return;
        }
        if (settings.assumeLightBackground) {
            // Dark foreground on a white baseline, preserving inverse threshold semantics.
            cv::Mat contrast;
            cv::subtract(settings.medianBackground, grayFrame, contrast);
            cv::compare(contrast, 0, foregroundMask, cv::CMP_GT);
            cv::subtract(cv::Scalar::all(255), contrast, grayFrame);
        } else {
            // Bright foreground on a black baseline.
            cv::subtract(grayFrame, settings.medianBackground, grayFrame);
            cv::compare(grayFrame, 0, foregroundMask, cv::CMP_GT);
        }
    }

    // Optional: Apply Gaussian blur
    if(settings.enableBlur) {
        // Ensure kernel size is odd and positive
        int kernelSize = settings.blurKernelSize;
        if (kernelSize % 2 == 0) kernelSize++;
        if (kernelSize <= 0) kernelSize = 1;
        
        try {
            cv::GaussianBlur(grayFrame, grayFrame, cv::Size(kernelSize, kernelSize), settings.blurSigmaX);
        } catch (const cv::Exception& ex) {
            YAWT_WARN(lcUtilsThresholding) << "GaussianBlur Exception:" << ex.what();
        }
    }

    int thresholdTypeOpenCV = settings.assumeLightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;

    // Ensure adaptive block size is odd and greater than 1
    int adaptiveBlock = settings.adaptiveBlockSize;
    if (adaptiveBlock <= 1) adaptiveBlock = 3;
    else if (adaptiveBlock % 2 == 0) adaptiveBlock++;

    try {
        switch (settings.algorithm) {
        case Thresholding::ThresholdAlgorithm::Global:
            cv::threshold(grayFrame, outputFrame, settings.globalThresholdValue, 255, thresholdTypeOpenCV);
            break;
        case Thresholding::ThresholdAlgorithm::Otsu:
            cv::threshold(grayFrame, outputFrame, 0, 255, thresholdTypeOpenCV | cv::THRESH_OTSU);
            break;
        case Thresholding::ThresholdAlgorithm::AdaptiveMean:
            cv::adaptiveThreshold(grayFrame, outputFrame, 255,
                                  cv::ADAPTIVE_THRESH_MEAN_C, thresholdTypeOpenCV,
                                  adaptiveBlock, settings.adaptiveCValue);
            break;
        case Thresholding::ThresholdAlgorithm::AdaptiveGaussian:
            cv::adaptiveThreshold(grayFrame, outputFrame, 255,
                                  cv::ADAPTIVE_THRESH_GAUSSIAN_C, thresholdTypeOpenCV,
                                  adaptiveBlock, settings.adaptiveCValue);
            break;
        default:
            cv::threshold(grayFrame, outputFrame, settings.globalThresholdValue, 255, thresholdTypeOpenCV);
            break;
        }
        // Adaptive thresholds can classify a perfectly flat dark background as foreground.
        // A pixel must also differ from the model in the selected foreground direction.
        if (!foregroundMask.empty()) cv::bitwise_and(outputFrame, foregroundMask, outputFrame);
    } catch (const cv::Exception& ex) {
        YAWT_WARN(lcUtilsThresholding) << "Thresholding Exception:" << ex.what();
        outputFrame = cv::Mat();
    }
}

cv::Mat computeMedianBackground(const std::vector<cv::Mat>& sampleFrames) {
    if (sampleFrames.empty()) return {};
    std::vector<cv::Mat> frames;
    for (const auto& frame : sampleFrames) {
        if (frame.empty()) return {};
        cv::Mat gray;
        if (frame.channels() == 3 || frame.channels() == 4)
            cv::cvtColor(frame, gray, frame.channels() == 4
                         ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY);
        else
            gray = frame;
        if (gray.type() != CV_8UC1 || gray.size() != sampleFrames.front().size()) return {};
        frames.push_back(gray);
    }
    cv::Mat median(frames.front().size(), CV_8UC1);
    std::vector<uchar> values(frames.size());
    for (int y = 0; y < median.rows; ++y) {
        for (int x = 0; x < median.cols; ++x) {
            for (size_t i = 0; i < frames.size(); ++i) values[i] = frames[i].ptr<uchar>(y)[x];
            auto middle = values.begin() + values.size() / 2;
            std::nth_element(values.begin(), middle, values.end());
            median.ptr<uchar>(y)[x] = *middle;
        }
    }
    return median;
}

cv::Mat computeVideoBackground(const QString& videoPath) {
    try {
        cv::VideoCapture capture(videoPath.toStdString());
        if (!capture.isOpened()) return {};
        const int count = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_COUNT));
        if (count <= 0) return {};
        const int samples = std::min(31, count);
        std::vector<cv::Mat> frames;
        for (int i = 0; i < samples; ++i) {
            const int index = samples == 1 ? 0 : static_cast<int>(
                static_cast<long long>(i) * (count - 1) / (samples - 1));
            if (!capture.set(cv::CAP_PROP_POS_FRAMES, index)) return {};
            cv::Mat frame, gray;
            if (!capture.read(frame) || frame.empty()) return {};
            if (frame.channels() == 3 || frame.channels() == 4)
                cv::cvtColor(frame, gray, frame.channels() == 4
                             ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY);
            else gray = frame;
            frames.push_back(gray);
        }
        return computeMedianBackground(frames);
    } catch (const cv::Exception& ex) {
        YAWT_WARN(lcUtilsThresholding) << "Background estimation failed:" << ex.what();
        return {};
    }
}

} // namespace ThresholdingUtils
