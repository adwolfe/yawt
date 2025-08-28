#include "thresholdingutils.h"
#include <QDebug>
#include <algorithm>

namespace ThresholdingUtils {

void applyThresholding(const cv::Mat& inputFrame, cv::Mat& outputFrame, 
                       const Thresholding::ThresholdSettings& settings) {
    if (inputFrame.empty()) {
        outputFrame = cv::Mat();
        return;
    }

    cv::Mat grayFrame;
    if (inputFrame.channels() == 3 || inputFrame.channels() == 4) {
        cv::cvtColor(inputFrame, grayFrame, cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = inputFrame.clone(); // Already grayscale or single channel
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
            qWarning() << "GaussianBlur Exception:" << ex.what();
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
    } catch (const cv::Exception& ex) {
        qWarning() << "Thresholding Exception:" << ex.what();
        outputFrame = cv::Mat();
    }
}

cv::Mat computeMedianBackground(const std::vector<cv::Mat>& sampleFrames) {
    if (sampleFrames.empty()) {
        return cv::Mat();
    }
    
    // Convert all frames to grayscale
    std::vector<cv::Mat> grayFrames;
    for (const auto& frame : sampleFrames) {
        cv::Mat gray;
        if (frame.channels() == 3 || frame.channels() == 4) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = frame.clone();
        }
        grayFrames.push_back(gray);
    }
    
    // Create a 3D matrix to hold all frames
    cv::Mat_<uchar> frames3D(static_cast<int>(grayFrames.size()), 
                             grayFrames[0].rows * grayFrames[0].cols, 1);
    
    // Copy each frame into the 3D matrix
    for (size_t i = 0; i < grayFrames.size(); i++) {
        cv::Mat frame = grayFrames[i].reshape(1, 1); // Flatten to 1D array
        frame.copyTo(frames3D.row(static_cast<int>(i)));
    }
    
    // Calculate median for each pixel
    cv::Mat medianFrame(grayFrames[0].size(), CV_8UC1);
    for (int i = 0; i < frames3D.cols; i++) {
        cv::Mat column = frames3D.col(i);
        std::vector<uchar> values;
        values.assign(column.begin<uchar>(), column.end<uchar>());
        std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
        medianFrame.data[i] = values[values.size()/2];
    }
    
    return medianFrame;
}

void subtractMedianBackground(const cv::Mat& inputFrame, cv::Mat& outputFrame,
                             const cv::Mat& medianBackground, bool clipToZero) {
    if (inputFrame.empty() || medianBackground.empty()) {
        outputFrame = inputFrame.clone();
        return;
    }
    
    cv::Mat grayInput;
    if (inputFrame.channels() == 3 || inputFrame.channels() == 4) {
        cv::cvtColor(inputFrame, grayInput, cv::COLOR_BGR2GRAY);
    } else {
        grayInput = inputFrame.clone();
    }
    
    // Perform subtraction
    cv::Mat subtracted;
    cv::absdiff(grayInput, medianBackground, subtracted);
    
    if (clipToZero) {
        // For cases where we want positive differences only
        cv::subtract(grayInput, medianBackground, outputFrame, cv::noArray(), CV_8U);
    } else {
        outputFrame = subtracted;
    }
}

void thresholdWithBackgroundSubtraction(
    const cv::Mat& inputFrame, 
    cv::Mat& outputFrame,
    const Thresholding::ThresholdSettings& settings,
    const cv::Mat& medianBackground,
    bool performSubtraction) {
    
    if (inputFrame.empty()) {
        outputFrame = cv::Mat();
        return;
    }
    
    cv::Mat preprocessedFrame;
    
    if (performSubtraction && !medianBackground.empty()) {
        subtractMedianBackground(inputFrame, preprocessedFrame, medianBackground, true);
    } else {
        if (inputFrame.channels() == 3 || inputFrame.channels() == 4) {
            cv::cvtColor(inputFrame, preprocessedFrame, cv::COLOR_BGR2GRAY);
        } else {
            preprocessedFrame = inputFrame.clone();
        }
    }
    
    // Apply thresholding to the preprocessed frame
    applyThresholding(preprocessedFrame, outputFrame, settings);
}

} // namespace ThresholdingUtils