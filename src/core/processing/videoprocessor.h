#ifndef VIDEOPROCESSOR_H
#define VIDEOPROCESSOR_H

#include <QObject>
#include <QString>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include "trackingcommon.h" // For ThresholdSettings

class VideoProcessor : public QObject {
    Q_OBJECT

public:
    explicit VideoProcessor(QObject *parent = nullptr);
    ~VideoProcessor();

public slots:
    // OLD: void startInitialProcessing(const QString& videoPath, int keyFrameNum, const Thresholding::ThresholdSettings& settings, int totalFramesHint);

    // NEW: Process a specific range of frames
    void processFrameRange(
        const QString& videoPath, // Video path for this processor instance
        const Thresholding::ThresholdSettings& settings, // Settings for this processor instance
        int startFrameAbsolute,    // Absolute start frame index in the video
        int endFrameAbsolute,      // Absolute end frame index (exclusive)
        int chunkId,               // An identifier for this processing job/chunk
        bool isForwardChunk        // To know if these frames are for forward or backward tracking segments
        );

signals:
    // OLD: void processingStarted();
    // OLD: void initialProcessingProgress(int percentage);
    // OLD: void initialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
    //                               const std::vector<cv::Mat>& reversedFrames,
    //                               double fps,
    //                               cv::Size frameSize);

    // NEW: Emit results for a specific chunk
    // The processedFrames are always in their natural video order (e.g., frame 10, 11, 12...)
    void rangeProcessingComplete(
        int chunkId,
        const std::vector<cv::Mat>& processedFrames,
        bool wasForwardChunk // To help TrackingManager sort them correctly
        );

    void rangeProcessingProgress(int chunkId, int percentage);
    void processingError(int chunkId, const QString& errorMessage); // Added chunkId for better error tracking

private:
    void applyThresholding(const cv::Mat& inputFrame, cv::Mat& outputFrame, const Thresholding::ThresholdSettings& settings);

    // Member variables for settings passed during construction or first call, if needed.
    // For this design, videoPath and settings are passed directly to processFrameRange.
    // bool m_processingActive; // Can be local to processFrameRange
};

#endif // VIDEOPROCESSOR_H
