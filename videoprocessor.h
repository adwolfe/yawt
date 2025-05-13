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
    void startInitialProcessing(const QString& videoPath, int keyFrameNum, const ThresholdSettings& settings, int totalFramesHint);

signals:
    void processingStarted();
    void initialProcessingProgress(int percentage);
    // Emits processed frames: forward (keyframe to end), reversed (keyframe-1 to start)
    void initialProcessingComplete(const std::vector<cv::Mat>& forwardFrames,
                                   const std::vector<cv::Mat>& reversedFrames,
                                   double fps,
                                   cv::Size frameSize);
    void processingError(const QString& errorMessage);

private:
    void applyThresholding(const cv::Mat& inputFrame, cv::Mat& outputFrame, const ThresholdSettings& settings);

    QString m_videoPath;
    int m_keyFrameNum; // 0-indexed
    ThresholdSettings m_thresholdSettings;
    int m_totalFramesHint; // For progress calculation
    bool m_processingActive;
};

#endif // VIDEOPROCESSOR_H

