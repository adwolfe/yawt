// VideoProcessor.cpp
#include "VideoProcessor.h"
#include <QDebug>
#include <QThread> // For interruption check
#include <algorithm> // For std::reverse

VideoProcessor::VideoProcessor(QObject *parent)
    : QObject(parent), m_keyFrameNum(-1), m_totalFramesHint(0), m_processingActive(false)
{
}

VideoProcessor::~VideoProcessor() {
    qDebug() << "VideoProcessor destroyed.";
}

void VideoProcessor::startInitialProcessing(const QString& videoPath, int keyFrameNum, const ThresholdSettings& settings, int totalFramesHint) {
    m_videoPath = videoPath;
    m_keyFrameNum = keyFrameNum;
    m_thresholdSettings = settings;
    m_totalFramesHint = totalFramesHint;
    m_processingActive = true;

    emit processingStarted();
    qDebug() << "VideoProcessor: Starting initial processing for" << videoPath << "Keyframe:" << keyFrameNum;

    cv::VideoCapture cap;
    try {
        if (!cap.open(m_videoPath.toStdString())) {
            emit processingError("Failed to open video file: " + m_videoPath);
            return;
        }
    } catch (const cv::Exception& ex) {
        emit processingError("OpenCV exception while opening video: " + QString(ex.what()));
        return;
    }


    int totalFrames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    if (totalFrames <= 0 && m_totalFramesHint > 0) {
        totalFrames = m_totalFramesHint; // Use hint if CAP_PROP_FRAME_COUNT fails
    } else if (totalFrames <= 0) {
        emit processingError("Could not determine total number of frames.");
        cap.release();
        return;
    }

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 25.0; // Default FPS if not available

    cv::Size frameSize(
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT))
        );
    if (frameSize.width <= 0 || frameSize.height <= 0) {
        emit processingError("Could not determine frame size.");
        cap.release();
        return;
    }


    if (m_keyFrameNum < 0 || m_keyFrameNum >= totalFrames) {
        emit processingError(QString("Keyframe index %1 is out of bounds (0-%2).").arg(m_keyFrameNum).arg(totalFrames - 1));
        cap.release();
        return;
    }

    std::vector<cv::Mat> forwardFrames;
    std::vector<cv::Mat> backwardFramesTemp; // Frames from 0 to keyframe-1, to be reversed later

    cv::Mat currentFrame, processedFrame;
    int framesProcessedCount = 0;

    for (int i = 0; i < totalFrames && m_processingActive; ++i) {
        if (QThread::currentThread()->isInterruptionRequested()) {
            m_processingActive = false;
            emit processingError("Initial processing was interrupted.");
            cap.release();
            return;
        }

        if (!cap.read(currentFrame) || currentFrame.empty()) {
            qWarning() << "VideoProcessor: Failed to read frame or empty frame at index" << i;
            // If this happens before all expected frames are read, it might be an issue.
            // For now, we continue, but this could be an error condition.
            if (i < totalFrames -1) { // If not the very last frame, it's more concerning
                // emit processingError(QString("Failed to read frame %1 of %2.").arg(i).arg(totalFrames));
                // cap.release();
                // return;
            }
            continue;
        }

        applyThresholding(currentFrame, processedFrame, m_thresholdSettings);

        if (i >= m_keyFrameNum) {
            forwardFrames.push_back(processedFrame.clone()); // Clone because processedFrame is reused
        } else { // i < m_keyFrameNum
            backwardFramesTemp.push_back(processedFrame.clone());
        }

        framesProcessedCount++;
        if (framesProcessedCount % 20 == 0 || framesProcessedCount == totalFrames) { // Update progress periodically
            emit initialProcessingProgress(static_cast<int>((static_cast<double>(framesProcessedCount) / totalFrames) * 100.0));
        }
    }
    cap.release();

    if (!m_processingActive) { // Processing was stopped or interrupted
        return; // Error signal already emitted or will be by caller
    }

    // Reverse the backwardFramesTemp to get the correct order for "reversed" tracking
    // The "reversedFrames" will start with the frame *before* the keyframe, then the one before that, etc.
    std::vector<cv::Mat> reversedFrames = backwardFramesTemp;
    std::reverse(reversedFrames.begin(), reversedFrames.end());

    qDebug() << "VideoProcessor: Initial processing complete. Forward frames:" << forwardFrames.size()
             << "Reversed frames:" << reversedFrames.size();
    emit initialProcessingComplete(forwardFrames, reversedFrames, fps, frameSize);
}

void VideoProcessor::applyThresholding(const cv::Mat& inputFrame, cv::Mat& outputFrame, const ThresholdSettings& settings) {
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

    // Optional: Apply Gaussian blur (consider making kernel size/sigma part of ThresholdSettings)
    cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5, 5), 0);

    int thresholdTypeOpenCV = settings.assumeLightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;

    switch (settings.algorithm) {
    case ThresholdAlgorithm::Global:
        cv::threshold(grayFrame, outputFrame, settings.thresholdValue, 255, thresholdTypeOpenCV);
        break;
    case ThresholdAlgorithm::Otsu:
        cv::threshold(grayFrame, outputFrame, 0, 255, thresholdTypeOpenCV | cv::THRESH_OTSU);
        break;
    case ThresholdAlgorithm::AdaptiveMean:
        cv::adaptiveThreshold(grayFrame, outputFrame, 255,
                              cv::ADAPTIVE_THRESH_MEAN_C, thresholdTypeOpenCV,
                              settings.adaptiveBlockSize, settings.adaptiveCValue);
        break;
    case ThresholdAlgorithm::AdaptiveGaussian:
        cv::adaptiveThreshold(grayFrame, outputFrame, 255,
                              cv::ADAPTIVE_THRESH_GAUSSIAN_C, thresholdTypeOpenCV,
                              settings.adaptiveBlockSize, settings.adaptiveCValue);
        break;
    default:
        // Fallback to global if algorithm unknown
        cv::threshold(grayFrame, outputFrame, settings.thresholdValue, 255, thresholdTypeOpenCV);
        break;
    }
}
