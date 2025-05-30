// VideoProcessor.cpp
#include "videoprocessor.h"
#include <QDebug>
#include <QThread> // For interruption check
#include <algorithm> // For std::reverse

VideoProcessor::VideoProcessor(QObject *parent)
    : QObject(parent)
{
    qDebug() << "VideoProcessor (" << this << ") created.";
}

VideoProcessor::~VideoProcessor() {
    qDebug() << "VideoProcessor (" << this << ") destroyed.";
}

// NEW IMPLEMENTATION
void VideoProcessor::processFrameRange(
    const QString& videoPath,
    const Thresholding::ThresholdSettings& settings,
    int startFrameAbsolute,
    int endFrameAbsolute,
    int chunkId,
    bool isForwardChunk)
{
    bool processingActive = true; // Local flag for this processing task
    qDebug() << "VideoProcessor (" << this << "): Starting processing for chunk" << chunkId
             << "Frames:" << startFrameAbsolute << "to" << endFrameAbsolute -1
             << (isForwardChunk ? "(ForwardSegment)" : "(BackwardSegment)");

    cv::VideoCapture cap;
    try {
        if (!cap.open(videoPath.toStdString())) {
            emit processingError(chunkId, "Failed to open video file: " + videoPath);
            return;
        }
    } catch (const cv::Exception& ex) {
        emit processingError(chunkId, "OpenCV exception while opening video: " + QString(ex.what()));
        return;
    }

    // It's important to set the starting position for reading frames.
    // CAP_PROP_POS_FRAMES is 0-based index of the frame to be decoded/captured next.
    if (startFrameAbsolute > 0) { // No need to set if starting from frame 0
        if (!cap.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(startFrameAbsolute))) {
            qWarning() << "VideoProcessor (" << this << ") Chunk" << chunkId << ": Failed to set video capture to frame" << startFrameAbsolute;
            // Depending on the video backend, seeking might not be perfectly accurate or supported.
            // We can try to read frames until we reach the desired startFrame as a fallback,
            // but this is less efficient. For now, we'll proceed and rely on set().
        }
    }

    std::vector<cv::Mat> processedFramesInChunk;
    cv::Mat currentFrame, processedFrame;
    int framesProcessedInThisChunk = 0;
    int totalFramesInThisChunk = endFrameAbsolute - startFrameAbsolute;

    if (totalFramesInThisChunk <= 0) {
        qDebug() << "VideoProcessor (" << this << ") Chunk" << chunkId << ": No frames to process in this range.";
        cap.release();
        emit rangeProcessingComplete(chunkId, processedFramesInChunk, isForwardChunk);
        return;
    }

    // The loop should go from the current position up to (but not including) endFrameAbsolute
    // or until totalFramesInThisChunk frames are processed.
    for (int i = 0; i < totalFramesInThisChunk && processingActive; ++i) {
        int currentVideoFrameIndex = startFrameAbsolute + i; // Actual frame number in the video

        if (QThread::currentThread()->isInterruptionRequested()) {
            processingActive = false;
            qDebug() << "VideoProcessor (" << this << ") Chunk" << chunkId << ": Processing was interrupted.";
            // Don't emit error here, let TrackingManager handle cancellation flow
            cap.release();
            return; // Exit cleanly
        }

        if (!cap.read(currentFrame) || currentFrame.empty()) {
            qWarning() << "VideoProcessor (" << this << ") Chunk" << chunkId << ": Failed to read frame or empty frame at video index" << currentVideoFrameIndex
                       << "(chunk frame" << i << "). Expected end:" << endFrameAbsolute -1;
            // If this happens before all expected frames in the chunk are read, it might be an issue.
            // This could mean the video ended prematurely or there was a read error.
            break; // Stop processing this chunk
        }

        applyThresholding(currentFrame, processedFrame, settings);
        processedFramesInChunk.push_back(processedFrame.clone()); // Frames are in natural video order

        framesProcessedInThisChunk++;
        if (framesProcessedInThisChunk % 10 == 0 || framesProcessedInThisChunk == totalFramesInThisChunk) { // Update progress periodically
            emit rangeProcessingProgress(chunkId, static_cast<int>((static_cast<double>(framesProcessedInThisChunk) / totalFramesInThisChunk) * 100.0));
        }
    }
    cap.release();

    if (!processingActive) { // Processing was stopped or interrupted
        qDebug() << "VideoProcessor (" << this << ") Chunk" << chunkId << ": Exiting due to interruption/stop request.";
        return;
    }

    qDebug() << "VideoProcessor (" << this << ") Chunk" << chunkId << ": Processing complete. Processed" << processedFramesInChunk.size() << "frames.";
    emit rangeProcessingComplete(chunkId, processedFramesInChunk, isForwardChunk);
}


void VideoProcessor::applyThresholding(const cv::Mat& inputFrame, cv::Mat& outputFrame, const Thresholding::ThresholdSettings& settings) {
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
        cv::GaussianBlur(grayFrame, grayFrame, cv::Size(kernelSize, kernelSize), settings.blurSigmaX);
    }

    int thresholdTypeOpenCV = settings.assumeLightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;

    // Ensure adaptive block size is odd and greater than 1
    int adaptiveBlock = settings.adaptiveBlockSize;
    if (adaptiveBlock <= 1) adaptiveBlock = 3;
    else if (adaptiveBlock % 2 == 0) adaptiveBlock++;


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
}
